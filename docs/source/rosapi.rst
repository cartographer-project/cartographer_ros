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

Published Topics
================

map (`nav_msgs/OccupancyGrid`_)
  Currently only supported in 2D. If subscribed to, a background thread will
  continuously compute and publish the map. Depending on the size of the map, it
  can take a few seconds between updates.

scan_matched_points2 (`sensor_msgs/PointCloud2`_)
  Point cloud as it was used for the purpose of scan-to-submap matching.

submap_list (`cartographer_ros_msgs/SubmapList`_)
  List of all submaps of all trajectories, containing the pose and latest
  version number of each submap.

.. _cartographer_ros_msgs/SubmapList: https://github.com/googlecartographer/cartographer_ros/blob/master/cartographer_ros_msgs/msg/SubmapList.msg
.. _nav_msgs/OccupancyGrid: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
.. _sensor_msgs/PointCloud2: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
