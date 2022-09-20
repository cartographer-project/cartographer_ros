#!/bin/bash

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

# Install CMake, Ninja, stow.
sudo apt-get update
sudo apt-get install -y lsb-release cmake ninja-build stow

# Install GMock library and header files for newer distributions.
if [[ "$(lsb_release -sc)" = "focal" || "$(lsb_release -sc)" = "buster" ]]
then
  sudo apt-get install -y libgmock-dev
fi

. /opt/ros/${ROS_DISTRO}/setup.sh

cd catkin_ws

# Install rosdep dependencies.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
