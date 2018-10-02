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

cmake_minimum_required(VERSION 2.8.3)

project(cartographer_ros_msgs)

set(PACKAGE_DEPENDENCIES
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${PACKAGE_DEPENDENCIES})

add_message_files(
  DIRECTORY msg
  FILES
    LandmarkEntry.msg
    LandmarkList.msg
    SensorTopics.msg
    StatusCode.msg
    StatusResponse.msg
    SubmapEntry.msg
    SubmapList.msg
    SubmapTexture.msg
    TrajectoryOptions.msg
    TrajectoryStates.msg
)

add_message_files(
  DIRECTORY msg/metrics
  FILES
    HistogramBucket.msg
    MetricFamily.msg
    MetricLabel.msg
    Metric.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    FinishTrajectory.srv
    GetTrajectoryStates.srv
    ReadMetrics.srv
    StartTrajectory.srv
    SubmapQuery.srv
    WriteState.srv
)

generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
)
