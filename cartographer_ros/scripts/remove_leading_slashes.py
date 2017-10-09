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
"""A simple tool to remove leading slashes from frame names."""

import argparse
import os
import rosbag


def ParseArgs():
  argument_parser = argparse.ArgumentParser(
      description="Removes leading slashes from frame names.")
  argument_parser.add_argument("input", type=str, help="Input bag")
  return argument_parser.parse_args()


def RewriteMsg(msg):
  if hasattr(msg, "header"):
    if msg.header.frame_id.startswith("/"):
      msg.header.frame_id = msg.header.frame_id[1:]
  if hasattr(msg, "child_frame_id"):
    if msg.child_frame_id.startswith("/"):
      msg.child_frame_id = msg.child_frame_id[1:]
  if hasattr(msg, "transforms"):
    for transform_msg in msg.transforms:
      RewriteMsg(transform_msg)


def Main():
  options = ParseArgs()
  with rosbag.Bag(os.path.splitext(options.input)[0] + ".filtered.bag",
                  "w") as outbag:
    for topic, msg, t in rosbag.Bag(options.input).read_messages():
      RewriteMsg(msg)
      outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)


if __name__ == "__main__":
  Main()
