#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2018 Magazino GmbH
#                The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import itertools
import random

import rospy
from tf import transformations
from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList

DESC = '''
Samples a number of random landmarks at a specified rate.
Can be used to test the timing of landmark input or the effect of erroneous
landmarks with duplicate IDs (if --allow_duplicate_ids is set).

For example:

./publish_fake_random_landmarks.py  \\
  --publish_period 0.1  \\
  --id_vocabulary A B C  \\
  --id_length 5  \\
  --sample_period 1.0

will publish empty landmark lists at 10Hz and random landmarks every second.
IDs are also sampled, using the cartesian product of the provided "vocabulary".
In the above example, a sampled ID could be e.g. "AACBC" (length=5).
'''

TOPIC = "landmark"


class LandmarkSamplerOptions(object):

  def __init__(self, *args, **kwargs):
    self.allow_duplicate_ids = False
    self.id_vocabulary = {}
    self.id_length = 0
    self.max_distance = 0.
    self.num_landmarks = 0
    self.rotation_weight = 0.
    self.translation_weight = 0.
    for name, arg in kwargs.items():
      setattr(self, name, arg)


class LandmarkIdSampler(object):

  def __init__(self, id_vocabulary, id_length):
    # Precompute all combinations of the symbols in the vocabulary.
    # WARNING: can be huge, check before potentially blocking the system.
    if len(id_vocabulary)**id_length > 1e6:
      raise ValueError("ID sampling space is too large")
    self.sampling_space = list(
        itertools.product(*(id_vocabulary for i in range(id_length))))

  def sample_id(self):
    # Draw a random combination of symbols and stringify it.
    random_index = random.randint(0, len(self.sampling_space) - 1)
    sampled_id = "".join(self.sampling_space[random_index])
    return sampled_id


class LandmarkSampler(object):

  def __init__(self, options):
    if not isinstance(options, LandmarkSamplerOptions):
      raise TypeError("expected LandmarkSamplerOptions")
    self.options = options
    rospy.loginfo("Initializing landmark ID sampler...")
    self.landmark_id_sampler = LandmarkIdSampler(options.id_vocabulary,
                                                 options.id_length)
    self._sampled_ids = []

  def random_landmark(self):
    landmark = LandmarkEntry()
    landmark.translation_weight = self.options.translation_weight
    landmark.rotation_weight = self.options.rotation_weight
    landmark.id = self.landmark_id_sampler.sample_id()
    if landmark.id in self._sampled_ids:
      if not self.options.allow_duplicate_ids:
        rospy.logwarn("Ignoring duplicate ID: {}".format(landmark.id))
        return None
      else:
        rospy.logwarn("Duplicate ID: {}".format(landmark.id))
    self._sampled_ids.append(landmark.id)

    vector = transformations.random_vector(3) * self.options.max_distance
    landmark.tracking_from_landmark_transform.position.x = vector[0]
    landmark.tracking_from_landmark_transform.position.y = vector[1]
    landmark.tracking_from_landmark_transform.position.z = vector[2]

    quaternion = transformations.random_quaternion()
    landmark.tracking_from_landmark_transform.orientation.x = quaternion[0]
    landmark.tracking_from_landmark_transform.orientation.y = quaternion[1]
    landmark.tracking_from_landmark_transform.orientation.z = quaternion[2]
    landmark.tracking_from_landmark_transform.orientation.w = quaternion[3]
    return landmark

  def random_landmark_list(self):
    landmark_list = LandmarkList()
    landmark_list.header.stamp = rospy.Time.now()
    for _ in range(self.options.num_landmarks):
      random_landmark = self.random_landmark()
      if random_landmark is not None:
        landmark_list.landmarks.append(random_landmark)
    return landmark_list


class SampledLandmarkPublisher(object):

  def __init__(self, publish_period, sample_period, landmark_sampler_options):
    self.landmark_sampler = LandmarkSampler(landmark_sampler_options)
    rospy.loginfo("Publishing landmarks to topic: {}".format(TOPIC))
    self.publisher = rospy.Publisher(TOPIC, LandmarkList, queue_size=1)
    self.publish_timer = rospy.Timer(
        rospy.Duration(publish_period), self.publish_empty_landmark_list)
    self.sample_timer = rospy.Timer(
        rospy.Duration(sample_period), self.publish_random_landmark_list)

  def publish_random_landmark_list(self, timer_event):
    self.publisher.publish(self.landmark_sampler.random_landmark_list())

  def publish_empty_landmark_list(self, timer_event):
    landmark_list = LandmarkList(rospy.Header(stamp=rospy.Time.now()), [])
    self.publisher.publish(landmark_list)


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(
      description=DESC, formatter_class=argparse.RawTextHelpFormatter)
  parser.add_argument("--translation_weight", type=float, default=1e5)
  parser.add_argument("--rotation_weight", type=float, default=1e5)
  parser.add_argument(
      "--publish_period",
      type=float,
      default=0.1,
      help="Baseline period for publishing empty landmark lists.")
  parser.add_argument(
      "--sample_period",
      type=float,
      default=5.,
      help="Period at which randomly sampled landmarks are published.")
  parser.add_argument(
      "--num_landmarks",
      type=int,
      default=5,
      help="The number of random landmarks published simultaneously.")
  parser.add_argument(
      "--max_distance",
      type=float,
      default=1.0,
      help="Maximum distance of a random landmark to the tracking frame.")
  parser.add_argument(
      "--id_vocabulary",
      nargs='+',
      default={"a", "b", "c", "1", "2", "3"},
      help="Set of symbols that can appear in random landmark IDs.")
  parser.add_argument(
      "--id_length",
      type=int,
      default=5,
      help="The length of the random landmark IDs (number of symbols).")
  parser.add_argument(
      "--allow_duplicate_ids",
      action="store_true",
      help="Publish landmarks with IDs that have already been published.")

  args, unknown = parser.parse_known_args(rospy.myargv()[1:])
  rospy.init_node("landmark_sampler")

  landmark_sampler_options = LandmarkSamplerOptions(**args.__dict__)
  sampler = SampledLandmarkPublisher(args.publish_period, args.sample_period,
                                     landmark_sampler_options)

  rospy.spin()
