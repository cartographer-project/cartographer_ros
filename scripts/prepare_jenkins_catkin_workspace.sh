#!/bin/sh

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -o errexit
set -o verbose

. /opt/ros/${ROS_DISTRO}/setup.sh

# Create a new workspace in 'catkin_ws'.
mkdir -p catkin_ws/src
cd catkin_ws/src
wstool init

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge ../../cartographer_ros/cartographer_ros.rosinstall
wstool merge -y https://raw.githubusercontent.com/googlecartographer/cartographer_fetch/master/cartographer_fetch.rosinstall
wstool merge -y https://raw.githubusercontent.com/magazino/cartographer_magazino/master/cartographer_magazino.rosinstall
wstool set cartographer -v ${CARTOGRAPHER_VERSION} -y
wstool remove cartographer_ros
wstool update
