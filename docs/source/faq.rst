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

==========================
Frequently asked questions
==========================

Why is laser data rate in the 3D bags higher than the maximum reported 20 Hz rotation speed of the VLP-16?
----------------------------------------------------------------------------------------------------------

The VLP-16 in the example bags is configured to rotate at 20 Hz. However, the
frequency of UDP packets the VLP-16 sends is much higher and independent of
the rotation frequency. The example bags contain a `sensor_msgs/PointCloud2`__
per UDP packet, not one per revolution.

__ http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html

In the `corresponding Cartographer configuration file`__ you see
`TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160` which means we
accumulate 160 per-UDP-packet point clouds into one larger point cloud, which
incorporates motion estimation by combining constant velocity and IMU
measurements, for matching. Since there are two VLP-16s, 160 UDP packets is
enough for roughly 2 revolutions, one per VLP-16.

__ https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros/configuration_files/backpack_3d.lua

Why is IMU data required for 3D SLAM but not for 2D?
----------------------------------------------------

In 2D, Cartographer supports running the correlative scan matcher, which is normally used for finding loop closure constraints, for local SLAM.
It is computationally expensive but can often render the incorporation of odometry or IMU data unnecessary.
2D also has the benefit of assuming a flat world, i.e. up is implicitly defined.

In 3D, an IMU is required mainly for measuring gravity.
Gravity is an attractive quantity to measure since it does not drift and is a very strong signal and typically comprises most of any measured accelerations.
Gravity is needed for two reasons:

1. There are no assumptions about the world in 3D.
To properly world align the resulting trajectory and map, gravity is used to define the z-direction.

2. Roll and pitch can be derived quite well from IMU readings once the direction of gravity has been established.
This saves work for the scan matcher by reducing the search window in these dimensions.

How do I build cartographer_ros without rviz support?
-----------------------------------------------------

The simplest solution is to create an empty file named `CATKIN_IGNORE`__ in the `cartographer_rviz` package directory.

__ http://wiki.ros.org/catkin/workspaces

How do I fix the "You called InitGoogleLogging() twice!" error?
---------------------------------------------------------------

Building `rosconsole` with the `glog` back end can lead to this error.
Use the `log4cxx` or `print` back end, selectable via the `ROSCONSOLE_BACKEND` CMake argument, to avoid this issue.
