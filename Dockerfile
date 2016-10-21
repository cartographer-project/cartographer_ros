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

FROM ros:indigo
# wstool needs the updated rosinstall file to clone the correct repos.
COPY cartographer_ros.rosinstall cartographer_ros/
# rosdep needs the updated package.xml files to install the correct debs.
COPY cartographer_ros/package.xml cartographer_ros/cartographer_ros/
COPY cartographer_ros_msgs/package.xml cartographer_ros/cartographer_ros_msgs/
COPY cartographer_rviz/package.xml cartographer_ros/cartographer_rviz/
COPY ceres_solver/package.xml cartographer_ros/ceres_solver/
COPY scripts/install_debs.sh cartographer_ros/scripts/
RUN cartographer_ros/scripts/install_debs.sh && rm -rf /var/lib/apt/lists/*
COPY . cartographer_ros
RUN cartographer_ros/scripts/install_cartographer_ros.sh && rm -rf catkin_ws
COPY scripts/ros_entrypoint.sh /
