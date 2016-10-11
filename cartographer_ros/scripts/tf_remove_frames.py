#!/usr/bin/python
# -*- coding: utf-8 -*-

# Copyright 2016 The Cartographer Authors
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

import rospy
from tf.msg import tfMessage


def main():
  rospy.init_node('tf_remove_frames')
  publisher = rospy.Publisher('/tf_out', tfMessage, queue_size=1)
  remove_frames = rospy.get_param('~remove_frames', [])

  def callback(msg):
    msg.transforms = [
        t for t in msg.transforms
        if t.header.frame_id.lstrip('/') not in remove_frames and
        t.child_frame_id.lstrip('/') not in remove_frames
    ]
    publisher.publish(msg)

  rospy.Subscriber('/tf_in', tfMessage, callback)
  rospy.spin()


if __name__ == '__main__':
  main()
