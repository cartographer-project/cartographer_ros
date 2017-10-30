import argparse
import datetime
import functools
import json
from os.path import basename
from pprint import pprint
import re
import subprocess
import sys

from google.cloud import bigquery
from google.cloud import datastore


# Defines a pattern for regular expression matching.
class Pattern:
    def __init__(self, pattern):
        self.regex =  re.compile(pattern, re.MULTILINE)

    # Returns a dictionary of named capture groups to extracted output. Returns an empty dict of no match was found.
    def extract(self, input):
        match = self.regex.search(input)
        if match == None:
            return {}
        return match.groupdict()


# Pattern matchers for the various fields of the '/usr/bin/time -v' output
USER_TIME_PATTERN = Pattern(r'^\s*User time \(seconds\): (?P<user_time>\d+.\d+|\d+)')
SYSTEM_TIME_PATTERN = Pattern(r'^\s*System time \(seconds\): (?P<system_time>\d+.\d+|\d+)')
WALL_TIME_PATTERN = Pattern(r'^\s*Elapsed \(wall clock\) time \(h:mm:ss or m:ss\): ((?P<hours>\d{1,2}):|)'
                            r'(?P<minutes>\d{1,2}):(?P<seconds>\d{2}\.\d{2})')
MAX_RES_SET_SIZE_PATTERN = Pattern(r'^\s*Maximum resident set size \(kbytes\): (?P<max_set_size>\d+)')


# Pattern matcher for extracting the HEAD commit SHA-1 hash.
GIT_SHA1_PATTERN = Pattern(r'^(?P<sha1>[0-9a-f]{40})\s+HEAD')


# Returns the SHA-1 hash of the commit tagged HEAD.
def get_tot_git_sha1():
    output = subprocess.check_output(['git', 'ls-remote', 'https://github.com/googlecartographer/cartographer.git'])
    dict = GIT_SHA1_PATTERN.extract(output)
    return dict['sha1']


# Returns a dictionary of stats.
def extract_stats(input):
    result = {}

    dict = USER_TIME_PATTERN.extract(input)
    result['user_time_secs'] = float(dict['user_time'])

    dict = SYSTEM_TIME_PATTERN.extract(input)
    result['system_time_secs'] = float(dict['system_time'])

    dict = WALL_TIME_PATTERN.extract(input)
    wall_time_secs = 0.0
    if (dict['hours'] != None):
        wall_time_secs += int(dict['hours']) * 3600
    wall_time_secs += int(dict['minutes']) * 60
    wall_time_secs += float(dict['seconds'])
    result['wall_time_secs'] = wall_time_secs

    dict = MAX_RES_SET_SIZE_PATTERN.extract(input)
    result['max_set_size_kbytes'] = int(dict['max_set_size'])

    return result


# Convenience function for Datastore key construction and entity retrieval.
def retrieve_entity(datastore_client, kind, id):
    key = datastore_client.key(kind, id)
    return datastore_client.get(key)


# Constructs a round-robin job selector.
def create_job_selector(worker_id, num_workers):
    return lambda job_id: job_id % num_workers == worker_id


# Runs the provided command both printing its stdout output to stdout and returning the output as a string.
def run_cmd(cmd):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output = ''
    while p.poll() is None:
        l = p.stdout.readline()
        output += l + '\n'
        print l

    l = p.stdout.read()
    output += l
    print l
    return output


# Represents a single job to be executed. A job consists of a combination of rosbag and configuration and launch files.
class Job:
    def __init__(self, datastore_client, job_id):
        self.id = job_id
        entity = retrieve_entity(datastore_client, "Job", job_id)
        self.launch_file = entity['launch_file']
        self.assets_writer_launch_file = entity['assets_writer_launch_file']
        self.assets_writer_config_file = entity['assets_writer_config_file']
        self.rosbag = entity['rosbag']

    def __str__(self):
        return 'id : {} launch_file: {} rosbag: {}'.format(self.id, self.launch_file, self.rosbag)

    def run(self, ros_distro, run_id):
        print 'running job {}'.format(self.id)
        # Copy the rosbag to scratch space
        scratch_dir = '/data/{}'.format(self.id)
        rosbag_filename = basename(self.rosbag)
        run_cmd('mkdir {}'.format(scratch_dir))
        run_cmd('gsutil cp gs://{} {}/{}'.format(self.rosbag, scratch_dir, rosbag_filename))

        # Creates pbstream
        output = run_cmd('/bin/bash -c \"source /opt/ros/{}/setup.bash && source /opt/cartographer_ros/setup.bash && '
                '/usr/bin/time -v roslaunch cartographer_ros {} '
                'bag_filenames:={}/{} no_rviz:=true\"'.format(ros_distro, self.launch_file, scratch_dir,
                                                              rosbag_filename))

        # Creates assets.
        run_cmd('/bin/bash -c \"source /opt/ros/{}/setup.bash && source /opt/cartographer_ros/setup.bash && '
                '/usr/bin/time -v roslaunch cartographer_ros {} '
                'bag_filenames:={}/{} pose_graph_filename:={}/{}.pbstream '
                'config_file:={}\"'.format(ros_distro, self.assets_writer_launch_file, scratch_dir, rosbag_filename,
                                           scratch_dir, rosbag_filename, self.assets_writer_config_file))

        # Copy assets to bucket.
        run_cmd('gsutil cp {}/{}.pbstream gs://cartographer-ci-artifacts/{}/{}/{}.pbstream'.format(scratch_dir, rosbag_filename, run_id, self.id, rosbag_filename))
        run_cmd('gsutil cp {}/{}_* gs://cartographer-ci-artifacts/{}/{}/'.format(scratch_dir, rosbag_filename, run_id, self.id, rosbag_filename))

        info = extract_stats(output)
        info['rosbag'] = rosbag_filename
        return info


# Represents a single worker that executes a sequence of Jobs.
class Worker:
    def __init__(self, datastore_client, pipeline_id, run_id):
        entity = retrieve_entity(datastore_client, "PipelineConfig", pipeline_id)
        self.pipeline_id = pipeline_id
        self.jobs = map(functools.partial(Job, datastore_client), entity['jobs'])
        self.scratch_dir = entity['scratch_dir']
        self.ros_distro = entity['ros_distro']
        self.run_id = run_id

    def __str__(self):
        result = 'pipeline_id: {}\n'.format(self.pipeline_id)
        for job in self.jobs:
            result += '{}\n'.format(str(job))
        return result

    def run_jobs(self, job_selector):
        outputs = {}
        for idx, job in enumerate(self.jobs):
            if job_selector(idx):
                output = job.run(self.ros_distro, self.run_id)
                outputs[job.id] = output
            else:
                print 'job {}: skip'.format(job.id)
        return outputs


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--worker_id', type=int)
    parser.add_argument('--num_workers', type=int)
    parser.add_argument('--pipeline_id', type=str)

    args = parser.parse_args()

    datastore_client = datastore.Client()
    bigquery_client = bigquery.Client()
    dataset = bigquery_client.dataset('Cartographer')
    table = dataset.table('metrics')



    job_selector = create_job_selector(int(args.worker_id),
                                        int(args.num_workers))

    tot_sha1 = get_tot_git_sha1()
    now = datetime.datetime.now()
    run_id = '{}-{}-{}_{}'.format(now.year, now.month, now.day, tot_sha1)

    worker = Worker(datastore_client,
                    args.pipeline_id,
                    run_id)


    # Publish stats to BigQuery.
    stats_dict = worker.run_jobs(job_selector)
    rows = []
    for job_id, info in stats_dict.iteritems():
        print info
        data_string = '''[
            \"{}-{}-{}\",
            \"{}\",
            {},
            \"{}\",
            {},
            {},
            {},
            {}
        ]'''.format(now.year, now.month, now.day, tot_sha1, job_id,
                    info['rosbag'], info['user_time_secs'], info['system_time_secs'], info['wall_time_secs'], info['max_set_size_kbytes']
                    )
        print data_string
        data = json.loads(data_string)
        rows.append(data)

    table.reload()
    errors = table.insert_data(rows)
    if not errors:
        print('Pushed {} row(s) into Cartographer:metrics'.format(len(rows)))
    else:
        print('Errors:')
        pprint(errors)
