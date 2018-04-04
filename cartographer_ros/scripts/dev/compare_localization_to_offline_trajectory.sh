#!/bin/bash

# Copyright 2018 The Cartographer Authors
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

set -o errexit
set -o verbose

MAPFILE="$1"
BAGFILE="$2"
if [ "$#" -ne 2 ]; then
  echo "\n\nUsage: $0 <frozen_map_file> <bag_file>\n"
  exit 1;
fi

LAUNCHSCRIPT='
<launch>
  <node name="cartographer_offline_node" pkg="cartographer_ros"
      required="true"
      type="cartographer_offline_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basenames backpack_2d.lua
          -urdf_filenames $(find cartographer_ros)/urdf/backpack_2d.urdf
          -load_state_filename '$MAPFILE'
          -bag_filenames '$BAGFILE'
          ">
    <remap from="echoes" to="horizontal_laser_2d" />
  </node>
</launch>'
echo $LAUNCHSCRIPT | roslaunch -

LAUNCHSCRIPT='
<launch>
  <param name="/use_sim_time" value="true" />
  <param name="robot_description"
      textfile="$(find cartographer_ros)/urdf/backpack_2d.urdf" />
  <node name="robot_state_publisher" pkg="robot_state_publisher"
      type="robot_state_publisher" />
  <node name="cartographer_node" pkg="cartographer_ros"
      required="true"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename backpack_2d_localization_evaluation.lua
          -load_state_filename '$MAPFILE'
          -load_frozen_state true
          ">
    <remap from="echoes" to="horizontal_laser_2d" />
  </node>
  <node name="playbag" pkg="rosbag" type="play"
      required="true"
      args="--clock '$BAGFILE'" />
  <node name="recordtf" pkg="rosbag" type="record"
      args="
          tf
          -O '$BAGFILE'.tf-result
          " />
</launch>
'
echo $LAUNCHSCRIPT | roslaunch -

rosrun cartographer_ros cartographer_dev_trajectory_comparison \
    -bag_filename $BAGFILE.tf-result.bag \
    -pbstream_filename $BAGFILE.pbstream
