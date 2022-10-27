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

==========================
Compiling Cartographer ROS
==========================

System Requirements
===================

The Cartographer ROS requirements are the same as `the ones from Cartographer`_.

The following `ROS distributions`_ are currently supported:

* Melodic
* Noetic

.. _the ones from Cartographer: https://google-cartographer.readthedocs.io/en/latest/#system-requirements
.. _ROS distributions: http://wiki.ros.org/Distributions

Building & Installation
=======================

In order to build Cartographer ROS, we recommend using `wstool <http://wiki.ros.org/wstool>`_ and `rosdep
<http://wiki.ros.org/rosdep>`_. For faster builds, we also recommend using
`Ninja <https://ninja-build.org>`_.

On Ubuntu Focal with ROS Noetic use these commands to install the above tools:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow

On older distributions:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install -y python-wstool python-rosdep ninja-build stow

After the tools are installed, create a new cartographer_ros workspace in 'catkin_ws'.

.. code-block:: bash

    mkdir catkin_ws
    cd catkin_ws
    wstool init src
    wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
    wstool update -t src

Now you need to install cartographer_ros' dependencies.
First, we use ``rosdep`` to install the required packages.
The command 'sudo rosdep init' will print an error if you have already executed it since installing ROS. This error can be ignored.

.. code-block:: bash

    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

Cartographer uses the `abseil-cpp`_ library that needs to be manually installed using this script:

.. code-block:: bash

    src/cartographer/scripts/install_abseil.sh 

Due to conflicting versions you might need to uninstall the ROS abseil-cpp using

.. code-block:: bash

   sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp 

Build and install.

.. code-block:: bash

    catkin_make_isolated --install --use-ninja

This builds Cartographer from the latest HEAD of the master branch.
If you want a specific version, you need to change the version in the cartographer_ros.rosinstall.

.. _abseil-cpp: https://abseil.io/
