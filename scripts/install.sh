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
mkdir catkin_ws
cd catkin_ws
wstool init

# Merge the cartographer_ros rosinstall file and fetch code for dependencies.
wstool merge ../cartographer_ros/cartographer_ros.rosinstall
wstool update

# Use the local version of cartographer_ros to include local modifications.
rm -rf src/cartographer_ros
mv ../cartographer_ros src

# Install deb dependencies.
sudo apt-get update
sudo apt-get install -y libwebp-dev  # TODO(whess): Move to rosdep.
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
rm -rf /var/lib/apt/lists/*

# Build and run tests.
catkin_make_isolated
catkin_make_isolated --catkin-make-args tests
catkin_make_isolated --catkin-make-args test

# TODO(whess): Fix installing cartographer_ros. For now we use a workaround:
echo Working around install issues...
mkdir /opt/cartographer_ros
ln -Tsf ${PWD}/devel_isolated/setup.bash /opt/cartographer_ros/setup.bash
exit

# Install under /opt/cartographer_ros.
catkin_make_isolated --install-space /opt/cartographer_ros --install

# Clean up.
cd ..
rm -rf catkin_ws
