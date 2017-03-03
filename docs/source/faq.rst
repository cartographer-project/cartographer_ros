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

The VLP-16 in the example bags is configured to rotate at 20 Hz. This has no influence though on the frequency of UDP packets the VLP-16 sends which is much higher. The example bags contain a `sensor_msgs/PointCloud2`__ per UDP packet, not one per revolution.

__ http://www.ros.org/doc/api/sensor_msgs/html/msg/PointCloud2.html

In the `corresponding Cartographer configuration file`__ you see `TRAJECTORY_BUILDER_3D.scans_per_accumulation = 160` which means we accumulate 160 of these to a point cloud for matching. Since
there are two VLP-16s this number is data from slightly more than one
revolution.

Your platform moves during the time you require to accumulate the point cloud,
and you would like to give your SLAM a chance of figuring out this movement.
Doing this accumulation during SLAM allows for unwarping the point cloud:
based on rotation velocities from the IMU and a constant velocity model. 

__ https://github.com/googlecartographer/cartographer_ros/blob/master/cartographer_ros/configuration_files/backpack_3d.lua
