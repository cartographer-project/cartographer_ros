"""This is the script executed by workers of the quality control pipline."""

import argparse
import datetime
import json
from os.path import basename
from pprint import pprint
import re
import subprocess

from google.cloud import bigquery
from google.cloud import datastore


class Pattern(object):
  """Defines a pattern for regular expression matching."""

  def __init__(self, pattern):
    self.regex = re.compile(pattern, re.MULTILINE)

  def extract(self, inp):
    """Returns a dictionary of named capture groups to extracted output.

    Args:
      inp: input to parse

    Returns an empty dict of no match was found.
    """
    match = self.regex.search(inp)
    if match is None:
      return {}
    return match.groupdict()


# Pattern matchers for the various fields of the '/usr/bin/time -v' output
USER_TIME_PATTERN = Pattern(
    r'^\s*User time \(seconds\): (?P<user_time>\d+.\d+|\d+)')
SYSTEM_TIME_PATTERN = Pattern(
    r'^\s*System time \(seconds\): (?P<system_time>\d+.\d+|\d+)')
WALL_TIME_PATTERN = Pattern(
    r'^\s*Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): '
    r'((?P<hours>\d{1,2}):|)(?P<minutes>\d{1,2}):(?P<seconds>\d{2}\.\d{2})')
MAX_RES_SET_SIZE_PATTERN = Pattern(
    r'^\s*Maximum resident set size \(kbytes\): (?P<max_set_size>\d+)')

# Pattern matcher for extracting the HEAD commit SHA-1 hash.
GIT_SHA1_PATTERN = Pattern(r'^(?P<sha1>[0-9a-f]{40})\s+HEAD')


def get_head_git_sha1():
  """Returns the SHA-1 hash of the commit tagged HEAD."""
  output = subprocess.check_output([
      'git', 'ls-remote',
      'https://github.com/googlecartographer/cartographer.git'
  ])
  parsed = GIT_SHA1_PATTERN.extract(output)
  return parsed['sha1']


def extract_stats(inp):
  """Returns a dictionary of stats."""
  result = {}

  parsed = USER_TIME_PATTERN.extract(inp)
  result['user_time_secs'] = float(parsed['user_time'])

  parsed = SYSTEM_TIME_PATTERN.extract(inp)
  result['system_time_secs'] = float(parsed['system_time'])

  parsed = WALL_TIME_PATTERN.extract(inp)
  result['wall_time_secs'] = float(parsed['hours'] or 0.) * 3600 + float(
      parsed['minutes']) * 60 + float(parsed['seconds'])

  parsed = MAX_RES_SET_SIZE_PATTERN.extract(inp)
  result['max_set_size_kbytes'] = int(parsed['max_set_size'])

  return result


def retrieve_entity(datastore_client, kind, identifier):
  """Convenience function for Datastore entity retrieval."""
  key = datastore_client.key(kind, identifier)
  return datastore_client.get(key)


def create_job_selector(worker_id, num_workers):
  """Constructs a round-robin job selector."""
  return lambda job_id: job_id % num_workers == worker_id


def run_cmd(cmd):
  """Runs command both printing its stdout output and returning it as string."""
  p = subprocess.Popen(
      cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
  run_cmd.output = []

  def process(line):
    run_cmd.output.append(line)
    print line.rstrip()

  while p.poll() is None:
    process(p.stdout.readline())
  process(p.stdout.read())
  return '\n'.join(run_cmd.output)


class Job(object):
  """Represents a single job to be executed.

  A job consists of a combination of rosbag and configuration and launch files.
  """

  def __init__(self, datastore_client, job_id):
    self.id = job_id
    entity = retrieve_entity(datastore_client, 'Job', job_id)
    self.launch_file = entity['launch_file']
    self.assets_writer_launch_file = entity['assets_writer_launch_file']
    self.assets_writer_config_file = entity['assets_writer_config_file']
    self.rosbag = entity['rosbag']

  def __repr__(self):
    return 'Job: id : {} launch_file: {} rosbag: {}'.format(
        self.id, self.launch_file, self.rosbag)

  def run(self, ros_distro, run_id):
    """Runs the job with ROS distro 'ros_distro'."""
    print 'running job {}'.format(self.id)
    # Copy the rosbag to scratch space
    scratch_dir = '/data/{}'.format(self.id)
    rosbag_filename = basename(self.rosbag)
    run_cmd('mkdir {}'.format(scratch_dir))
    run_cmd('gsutil cp gs://{} {}/{}'.format(self.rosbag, scratch_dir,
                                             rosbag_filename))

    # Creates pbstream
    output = run_cmd(
        '/bin/bash -c \"source /opt/ros/{}/setup.bash && source '
        '/opt/cartographer_ros/setup.bash && /usr/bin/time -v roslaunch '
        'cartographer_ros {} bag_filenames:={}/{} no_rviz:=true\"'.format(
            ros_distro, self.launch_file, scratch_dir, rosbag_filename))

    # Creates assets.
    run_cmd('/bin/bash -c \"source /opt/ros/{}/setup.bash && source '
            '/opt/cartographer_ros/setup.bash && /usr/bin/time -v roslaunch '
            'cartographer_ros {} bag_filenames:={}/{} '
            'pose_graph_filename:={}/{}.pbstream config_file:={}\"'.format(
                ros_distro, self.assets_writer_launch_file, scratch_dir,
                rosbag_filename, scratch_dir, rosbag_filename,
                self.assets_writer_config_file))

    # Copies assets to bucket.
    run_cmd('gsutil cp {}/{}.pbstream '
            'gs://cartographer-ci-artifacts/{}/{}/{}.pbstream'.format(
                scratch_dir, rosbag_filename, run_id, self.id, rosbag_filename))
    run_cmd('gsutil cp {}/{}_* gs://cartographer-ci-artifacts/{}/{}/'.format(
        scratch_dir, rosbag_filename, run_id, self.id))

    info = extract_stats(output)
    info['rosbag'] = rosbag_filename
    return info


class Worker(object):
  """Represents a single worker that executes a sequence of Jobs."""

  def __init__(self, datastore_client, pipeline_id, run_id):
    entity = retrieve_entity(datastore_client, 'PipelineConfig', pipeline_id)
    self.pipeline_id = pipeline_id
    self.jobs = [Job(datastore_client, job_id) for job_id in entity['jobs']]
    self.scratch_dir = entity['scratch_dir']
    self.ros_distro = entity['ros_distro']
    self.run_id = run_id

  def __repr__(self):
    result = 'Worker: pipeline_id: {}\n'.format(self.pipeline_id)
    for job in self.jobs:
      result += '{}\n'.format(str(job))
    return result

  def run_jobs(self, selector):
    outputs = {}
    for idx, job in enumerate(self.jobs):
      if selector(idx):
        output = job.run(self.ros_distro, self.run_id)
        outputs[job.id] = output
      else:
        print 'job {}: skip'.format(job.id)
    return outputs


def publish_stats_to_big_query(stats_dict, now, head_sha1):
  """Publishes metrics to BigQuery."""
  bigquery_client = bigquery.Client()
  dataset = bigquery_client.dataset('Cartographer')
  table = dataset.table('metrics')
  rows = []
  for job_identifier, job_info in stats_dict.iteritems():
    data_string = """[
            \"{}-{}-{}\",
            \"{}\",
            {},
            \"{}\",
            {},
            {},
            {},
            {}
        ]""".format(now.year, now.month, now.day, head_sha1, job_identifier,
                    job_info['rosbag'], job_info['user_time_secs'],
                    job_info['system_time_secs'], job_info['wall_time_secs'],
                    job_info['max_set_size_kbytes'])
    data = json.loads(data_string)
    rows.append(data)

  table.reload()
  errors = table.insert_data(rows)
  if not errors:
    print 'Pushed {} row(s) into Cartographer:metrics'.format(len(rows))
  else:
    print 'Errors:'
    pprint(errors)


def parse_arguments():
  """Parses the command line arguments."""
  parser = argparse.ArgumentParser(
      description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('--worker_id', type=int)
  parser.add_argument('--num_workers', type=int)
  parser.add_argument('--pipeline_id', type=str)
  return parser.parse_args()


def main():
  args = parse_arguments()
  ds_client = datastore.Client()
  job_selector = create_job_selector(int(args.worker_id), int(args.num_workers))
  head_sha1 = get_head_git_sha1()
  now = datetime.datetime.now()
  pipeline_run_id = '{}-{}-{}_{}'.format(now.year, now.month, now.day,
                                         head_sha1)
  worker = Worker(ds_client, args.pipeline_id, pipeline_run_id)
  stats_dict = worker.run_jobs(job_selector)
  publish_stats_to_big_query(stats_dict, now, head_sha1)


if __name__ == '__main__':
  main()
