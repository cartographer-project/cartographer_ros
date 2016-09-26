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

============================
Cartographer ROS Integration
============================

.. toctree::
   :maxdepth: 2
   :hidden:

   options

`Cartographer`_ is a system that provides real-time simultaneous localization
and mapping `SLAM`_ across multiple platforms and sensor configurations. This
project provides Cartographer's ROS integration.

.. _Cartographer: https://github.com/googlecartographer/cartographer
.. _SLAM: https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping

.. _getting-started:

Building & Installation
=======================

Installation has been tested on Ubuntu 14.04 (Trusty) with ROS Indigo, but may
also work on Ubuntu 16.04 (Xenial) with ROS Kinetic. We recommend using
`wstool <http://wiki.ros.org/wstool>`_ and
`rosdep <http://wiki.ros.org/rosdep>`_. For faster builds, we also recommend
using `Ninja <https://ninja-build.org>`_.

  .. code-block:: bash

    # Install wstool and rosdep.
    sudo apt-get update
    sudo apt-get install -y python-wstool python-rosdep ninja-build

    # Create a new workspace in 'catkin_ws'.
    mkdir catkin_ws
    cd catkin_ws
    wstool init

    # Merge the cartographer_ros rosinstall file and fetch code for dependencies.
    wstool merge https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
    wstool update

    # Install deb dependencies.
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

    # Build and install.
    catkin_make_isolated --install --use-ninja
    source install_isolated/setup.bash

Running the demo
================

Now that Cartographer and Cartographer's ROS integration are installed,
download the example bag, a 2D backpack collection of the `Deutsches Museum
<https://en.wikipedia.org/wiki/Deutsches_Museum>`_, to a known location, in
this case ``~/Downloads``, and use ``roslaunch`` to bring up the demo:

  .. code-block:: bash

    wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
    roslaunch cartographer_ros demo_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag

The launch file will bring up ``roscore`` and ``rviz`` automatically.
