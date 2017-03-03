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

The laser data in the 3D bags is much higher than the maximum reported 20 Hz rotation speed that the VLP-16 can do. Why?
----------------------------------------------------------------------------------------------------------------------

The VLP-16 in the example bags is configured to rotate at 20 Hz. However, the
frequency of UDP packets the VLP-16 sends is much higher and independent of
the rotation frequency. The example bags contain a `sensor_msgs/PointCloud2`__
per UDP packet, not one per revolution.

__ http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html

In the `corresponding Cartographer configuration file`__ you see
`TRAJECTORY_BUILDER_3D.scans_per_accumulation = 160` which means we accumulate
160 per-UDP-packet point clouds into one larger point cloud, which
incorporates motion estimation by combining constant velocity and IMU
measurements, for matching. Since there are two VLP-16s, 160 UDP packets is
enough for roughly 2 revolutions, one per VLP-16.

__ https://github.com/googlecartographer/cartographer_ros/blob/master/cartographer_ros/configuration_files/backpack_3d.lua
