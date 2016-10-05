.. Copyright 2016 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=======
ROS API
=======

The following ROS API is provided by `cartographer_node`_.

Command-line Flags
==================

.. TODO(damonkohler): Use an options list if it can be made to render nicely.

\-\-configuration_directory
  First directory in which configuration files are searched, second is always
  the Cartographer installation to allow including files from there.

\-\-configuration_basename
  Basename (i.e. not containing any directory prefix) of the configuration file
  (e.g. backpack_3d.lua).

Subscribed Topics
=================

The following range data topics are mutually exclusive. At least one source of
range data is required.

scan (`sensor_msgs/LaserScan`_)
  Only supported in 2D. If *use_horizontal_laser* is enabled in the
  :doc:`configuration`, this topic will be used as input for SLAM.

echoes (`sensor_msgs/MultiEchoLaserScan`_)
  Only supported in 2D. If *use_horizontal_multi_echo_laser* is enabled in the
  :doc:`configuration`, this topic will be used as input for SLAM. Only the
  first echo is used.

points2 (`sensor_msgs/PointCloud2`_)
  Only supported in 3D. If *num_lasers_3d* is set to 1 in the
  :doc:`configuration`, this topic will be used as input for SLAM. If
  *num_lasers_3d* is greater than 1, multiple numbered points2 topics (i.e.
  points2_1, points2_2, points2_3, ...  up to and including *num_lasers_3d*)
  will be used as inputs for SLAM.

The following additional sensor data topics may also be provided.

imu (`sensor_msgs/Imu`_)
  Supported in 2D (optional) and 3D (required). This topic will be used as
  input for SLAM.

odom (`nav_msgs/Odometry`_)
  Supported in 2D (optional) and 3D (optional). If *use_odometry_data* is
  enabled in the :doc:`configuration`, this topic will be used as input for
  SLAM.

Published Topics
================

map (`nav_msgs/OccupancyGrid`_)
  Only supported in 2D. If subscribed to, a background thread will continuously
  compute and publish the map. The time between updates will increase with the
  size of the map. For faster updates, use the submaps APIs.

scan_matched_points2 (`sensor_msgs/PointCloud2`_)
  Point cloud as it was used for the purpose of scan-to-submap matching. This
  cloud may be both filtered and projected depending on the
  :doc:`configuration`.

submap_list (`cartographer_ros_msgs/SubmapList`_)
  List of all submaps, including the pose and latest version number of each
  submap, across all trajectories.

Services
========

submap_query (`cartographer_ros_msgs/SubmapQuery`_)
  Fetches the requested submap.

finish_trajectory (`cartographer_ros_msgs/FinishTrajectory`_)
  Finishes the current trajectory by flushing all queued sensor data, running a
  final optimization, and writing the map to disk. Currently, this also
  triggers shutdown.

Required tf Transforms
======================

Transforms from all incoming sensor data frames to the :doc:`configured
<configuration>` *tracking_frame* and *published_frame* must be available.
Typically, these are published periodically by a `robot_state_publisher` or a
`static_transform_publisher`.

Provided tf Transforms
======================

The transformation between the :doc:`configured <configuration>` *map_frame*
and *published_frame* is always provided.

If *provide_odom_frame* is enabled in the :doc:`configuration`, a continuous
(i.e. unaffected by loop closure) transform between the :doc:`configured
<configuration>` *odom_frame* and *published_frame* will be provided.

.. _robot_state_publisher: http://wiki.ros.org/robot_state_publisher
.. _static_transform_publisher: http://wiki.ros.org/tf#static_transform_publisher
.. _cartographer_node: https://github.com/googlecartographer/cartographer_ros/blob/master/cartographer_ros/src/cartographer_node_main.cc
.. _cartographer_ros_msgs/FinishTrajectory: https://github.com/googlecartographer/cartographer_ros/blob/master/cartographer_ros_msgs/srv/FinishTrajectory.srv
.. _cartographer_ros_msgs/SubmapList: https://github.com/googlecartographer/cartographer_ros/blob/master/cartographer_ros_msgs/msg/SubmapList.msg
.. _cartographer_ros_msgs/SubmapQuery: https://github.com/googlecartographer/cartographer_ros/blob/master/cartographer_ros_msgs/srv/SubmapQuery.srv
.. _nav_msgs/OccupancyGrid: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
.. _nav_msgs/Odometry: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html
.. _sensor_msgs/Imu: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
.. _sensor_msgs/LaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
.. _sensor_msgs/MultiEchoLaserScan: http://docs.ros.org/api/sensor_msgs/html/msg/MultiEchoLaserScan.html
.. _sensor_msgs/PointCloud2: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
