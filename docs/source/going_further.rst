.. Copyright 2018 The Cartographer Authors

.. Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

..      http://www.apache.org/licenses/LICENSE-2.0

.. Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

=============
Going further
=============

Cartographer is not only a great SLAM algorithm, it also comes with a fully-featured implementation that brings lots of "extra" features.
This page lists some of those less known functionalities.

More input
==========

If you have a source of odometry (such as a wheel encoder) publishing on a ``nav_msgs/Odometry`` topic and want to use it to improve Cartographer's localization, you can add an input to your ``.lua`` configuration files:

..  code-block:: lua

    use_odometry = true

The messages will be expected on the ``odom`` topic.

A GPS publishing on a ``sensor_msgs/NavSatFix`` topic named ``fix`` can improve the global SLAM:

..  code-block:: lua

    use_nav_sat = true

For landmarks publishing on a ``cartographer_ros_msgs/LandmarkList`` (`message defined in cartographer_ros`_) topic named ``landmark``:

..  code-block:: lua

    use_landmarks = true

.. _message defined in cartographer_ros: https://github.com/cartographer-project/cartographer_ros/blob/4b39ee68c7a4d518bf8d01a509331e2bc1f514a0/cartographer_ros_msgs/msg/LandmarkList.msg

Localization only
=================

If you have a map you are happy with and want to reduce computations, you can use the localization-only mode of Cartographer which will run SLAM against the existing map and won't build a new one.
This is enabled by running ``cartographer_node`` with a ``-load_state_filename`` argument and by defining the following line in your lua config:

..  code-block:: lua

    TRAJECTORY_BUILDER.pure_localization_trimmer = {
        max_submaps_to_keep = 3,
    }

IMU Calibration
===============

When performing the global optimization, Ceres tries to improve the pose between your IMU and range finding sensors.
A well chosen acquisition with lots of loop closure constraints (for instance if your robot goes on a straight line and then back) can improve the quality of those corrections and become a reliable source of pose correction.
You can then use Cartographer as part of your calibration process to improve the quality of your robot's extrinsic calibration.

Multi-trajectories SLAM
=======================

Cartographer can perform SLAM from multiple robots emiting data in parallel.
The global SLAM is able to detect shared paths and will merge the maps built by the different robots as soon as it becomes possible.
This is achieved through the usage of two ROS services ``start_trajectory`` and ``finish_trajectory``. (refer to the ROS API reference documentation for more details on their usage)

Cloud integration with gRPC
===========================

Cartographer is built around Protobuf messages which make it very flexible and interoperable.
One of the advantages of that architecture is that it is easy to distribute on machines spread over the Internet.
The typical use case would be a fleet of robots navigating on a known map, they could have their SLAM algorithm run on a remote powerful centralized localization server running a multi-trajectories Cartographer instance.

**TODO**: Instructions on how to get started with a gRPC Cartographer instance
