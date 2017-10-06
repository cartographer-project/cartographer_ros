#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""A simple tool to remove leading slashes from bags."""

import rosbag
import argparse

def parse_args():
  p = argparse.ArgumentParser(description=
    "Removes leading slashes from topic names in a bag."
  )
  p.add_argument("input", type=str, help = "Input bag")
  return p.parse_args()

def rewrite(msg):
  if hasattr(msg, "header"):
    if msg.header.frame_id.startswith("/"):
      msg.header.frame_id = msg.header.frame_id[1:]
  if hasattr(msg, "child_frame_id"):
    if msg.child_frame_id.startswith("/"):
      msg.child_frame_id = msg.child_frame_id[1:]
  if hasattr(msg, "transforms"):
    for transform_msg in msg.transforms:
      rewrite(transform_msg)

def main():
  o = parse_args()
  with rosbag.Bag(o.input[:-4] + '.filtered.bag', 'w') as outbag:
      for topic, msg, t in rosbag.Bag(o.input).read_messages():
        rewrite(msg)
        outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)

if __name__ == "__main__":
  main()
