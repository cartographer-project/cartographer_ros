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

string tracking_frame
string published_frame
string odom_frame
bool provide_odom_frame
bool use_odometry
bool use_nav_sat
bool use_landmarks
bool publish_frame_projected_to_2d
int32 num_laser_scans
int32 num_multi_echo_laser_scans
int32 num_subdivisions_per_laser_scan
int32 num_point_clouds
float64 rangefinder_sampling_ratio
float64 odometry_sampling_ratio
float64 fixed_frame_pose_sampling_ratio
float64 imu_sampling_ratio
float64 landmarks_sampling_ratio

# This is a binary-encoded
# 'cartographer.mapping.proto.TrajectoryBuilderOptions' proto.
string trajectory_builder_options_proto
