"""This is the script executed by workers of the quality control pipline."""

import argparse
import datetime
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

  def extract(self, text):
    """Returns a dictionary of named capture groups to extracted output.

    Args:
      text: input to parse

    Returns an empty dict if no match was found.
    """
    match = self.regex.search(text)
    if match is None:
      return {}
    return match.groupdict()

  def extract_last_occurence(self, text):
    """Returns tuple of extracted outputs.

    Args:
      text: input to parse

    Returns the information extracted from the last match. Returns
    None if no match was found.
    """
    matches = self.regex.findall(text)
    if not matches:
      return None
    return matches[-1]


# BigQuery table schema
SCHEMA = [
    bigquery.SchemaField('date', 'DATE'),
    bigquery.SchemaField('commit_sha1', 'STRING'),
    bigquery.SchemaField('job_id', 'INTEGER'),
    bigquery.SchemaField('rosbag', 'STRING'),
    bigquery.SchemaField('user_time_secs', 'FLOAT'),
    bigquery.SchemaField('system_time_secs', 'FLOAT'),
    bigquery.SchemaField('wall_time_secs', 'FLOAT'),
    bigquery.SchemaField('max_set_size_kbytes', 'INTEGER'),
    bigquery.SchemaField('constraints_count', 'INTEGER'),
    bigquery.SchemaField('constraints_score_minimum', 'FLOAT'),
    bigquery.SchemaField('constraints_score_maximum', 'FLOAT'),
    bigquery.SchemaField('constraints_score_mean', 'FLOAT'),
    bigquery.SchemaField('ground_truth_abs_trans_err', 'FLOAT'),
    bigquery.SchemaField('ground_truth_abs_trans_err_dev', 'FLOAT'),
    bigquery.SchemaField('ground_truth_sqr_trans_err', 'FLOAT'),
    bigquery.SchemaField('ground_truth_sqr_trans_err_dev', 'FLOAT'),
    bigquery.SchemaField('ground_truth_abs_rot_err', 'FLOAT'),
    bigquery.SchemaField('ground_truth_abs_rot_err_dev', 'FLOAT'),
    bigquery.SchemaField('ground_truth_sqr_rot_err', 'FLOAT'),
    bigquery.SchemaField('ground_truth_sqr_rot_err_dev', 'FLOAT')
]

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
CONSTRAINT_STATS_PATTERN = Pattern(
    r'Score histogram:[\n\r]+'
    r'Count:\s+(?P<constraints_count>\d+)\s+'
    r'Min:\s+(?P<constraints_score_min>\d+\.\d+)\s+'
    r'Max:\s+(?P<constraints_score_max>\d+\.\d+)\s+'
    r'Mean:\s+(?P<constraints_score_mean>\d+\.\d+)')
GROUND_TRUTH_STATS_PATTERN = Pattern(
    r'Result:[\n\r]+'
    r'Abs translational error (?P<abs_trans_err>\d+\.\d+) '
    r'\+/- (?P<abs_trans_err_dev>\d+\.\d+) m[\n\r]+'
    r'Sqr translational error (?P<sqr_trans_err>\d+\.\d+) '
    r'\+/- (?P<sqr_trans_err_dev>\d+\.\d+) m\^2[\n\r]+'
    r'Abs rotational error (?P<abs_rot_err>\d+\.\d+) '
    r'\+/- (?P<abs_rot_err_dev>\d+\.\d+) deg[\n\r]+'
    r'Sqr rotational error (?P<sqr_rot_err>\d+\.\d+) '
    r'\+/- (?P<sqr_rot_err_dev>\d+\.\d+) deg\^2')

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

  parsed = CONSTRAINT_STATS_PATTERN.extract_last_occurence(inp)
  print parsed
  result['constraints_count'] = int(parsed[0])
  result['constraints_score_min'] = float(parsed[1])
  result['constraints_score_max'] = float(parsed[2])
  result['constraints_score_mean'] = float(parsed[3])

  return result


def extract_ground_truth_stats(input_text):
  """Returns a dictionary of ground truth stats."""
  result = {}
  parsed = GROUND_TRUTH_STATS_PATTERN.extract(input_text)
  for name in ('abs_trans_err', 'sqr_trans_err', 'abs_rot_err', 'sqr_rot_err'):
    result['ground_truth_{}'.format(name)] = float(parsed[name])
    result['ground_truth_{}_dev'.format(name)] = float(
        parsed['{}_dev'.format(name)])
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
  print cmd
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


def run_ros_cmd(ros_distro, ros_cmd):
  """Runs command similar to run_cmd but sets ROS environment variables."""
  cmd = ('/bin/bash -c \"source /opt/ros/{}/setup.bash && source '
         '/opt/cartographer_ros/setup.bash && {}\"').format(
             ros_distro, ros_cmd)
  return run_cmd(cmd)


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
    self.ros_package = entity['ros_package']

  def __repr__(self):
    return 'Job: id : {} launch_file: {} rosbag: {}'.format(
        self.id, self.launch_file, self.rosbag)

  def run(self, ros_distro, run_id):
    """Runs the job with ROS distro 'ros_distro'."""
    print 'running job {}'.format(self.id)

    # Garbage collect any left-overs from previous runs.
    run_cmd('rm -rf /data/*')

    # Copy the rosbag to scratch space
    scratch_dir = '/data/{}'.format(self.id)
    rosbag_filename = basename(self.rosbag)
    run_cmd('mkdir {}'.format(scratch_dir))
    run_cmd('gsutil cp gs://{} {}/{}'.format(self.rosbag, scratch_dir,
                                             rosbag_filename))

    # Creates pbstream
    output = run_ros_cmd(ros_distro,
                         '/usr/bin/time -v roslaunch {} {} '
                         'bag_filenames:={}/{} no_rviz:=true'.format(
                             self.ros_package, self.launch_file, scratch_dir,
                             rosbag_filename))
    info = extract_stats(output)

    # Creates assets.
    run_ros_cmd(
        ros_distro, '/usr/bin/time -v roslaunch {} {} '
        'bag_filenames:={}/{} pose_graph_filename:='
        '{}/{}.pbstream config_file:={}'.format(
            self.ros_package, self.assets_writer_launch_file, scratch_dir,
            rosbag_filename, scratch_dir, rosbag_filename,
            self.assets_writer_config_file))

    # Copies assets to bucket.
    run_cmd('gsutil cp {}/{}.pbstream '
            'gs://cartographer-ci-artifacts/{}/{}/{}.pbstream'.format(
                scratch_dir, rosbag_filename, run_id, self.id, rosbag_filename))
    run_cmd('gsutil cp {}/{}_* gs://cartographer-ci-artifacts/{}/{}/'.format(
        scratch_dir, rosbag_filename, run_id, self.id))

    # Download ground truth relations file.
    run_cmd('gsutil cp gs://cartographer-ci-ground-truth/{}/relations.pb '
            '{}/relations.pb'.format(self.id, scratch_dir))

    # Calculate metrics.
    output = run_ros_cmd(ros_distro, 'cartographer_compute_relations_metrics '
                         '-relations_filename {}/relations.pb '
                         '-pose_graph_filename {}/{}.pbstream'.format(
                             scratch_dir, scratch_dir, rosbag_filename))

    # Add ground truth stats.
    info.update(extract_ground_truth_stats(output))

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
  rows_to_insert = []
  for job_identifier, job_info in stats_dict.iteritems():
    print job_info
    data = ('{}-{}-{}'.format(
        now.year, now.month,
        now.day), head_sha1, job_identifier, job_info['rosbag'],
            job_info['user_time_secs'], job_info['system_time_secs'],
            job_info['wall_time_secs'], job_info['max_set_size_kbytes'],
            job_info['constraints_count'], job_info['constraints_score_min'],
            job_info['constraints_score_max'],
            job_info['constraints_score_mean'],
            job_info['ground_truth_abs_trans_err'],
            job_info['ground_truth_abs_trans_err_dev'],
            job_info['ground_truth_sqr_trans_err'],
            job_info['ground_truth_sqr_trans_err_dev'],
            job_info['ground_truth_abs_rot_err'],
            job_info['ground_truth_abs_rot_err_dev'],
            job_info['ground_truth_sqr_rot_err'],
            job_info['ground_truth_sqr_rot_err_dev'])

    rows_to_insert.append(data)

  errors = bigquery_client.create_rows(
      table, rows_to_insert, selected_fields=SCHEMA)
  if not errors:
    print 'Pushed {} row(s) into Cartographer:metrics'.format(
        len(rows_to_insert))
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
