# Cartographer Project Overview

See https://github.com/googlecartographer/cartographer

## Installation

Installation has been tested on Ubuntu 14.04 (Trusty) with ROS Indigo and on Ubuntu 16.04 (Xenial) with ROS Kinetic. For ROS Kinetic, simply replace the two occurrences of "indigo" with "kinetic" in the instructions below. There are multiple options for building cartographer_ros as part of a ROS workspace. Two common use cases are described below.

These dependencies always have to be installed:

    # Install the required libraries that are available as debs
    sudo apt-get install \
      ros-indigo-tf2-eigen \
      g++ \
      google-mock \
      libboost-all-dev \
      liblua5.2-dev \
      libprotobuf-dev \
      libsuitesparse-dev \
      libwebp-dev \
      protobuf-compiler \
      python-sphinx \
      libpcap-dev  # For 3D SLAM with Velodynes


### Standalone Workspace

    # Set up your Catkin workspace
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    source /opt/ros/indigo/setup.bash
    catkin_init_workspace

    # Clone the necessary repos into your Catkin workspace
    git clone https://github.com/googlecartographer/cartographer.git
    git clone https://github.com/googlecartographer/cartographer_ros.git
    git clone https://github.com/ethz-asl/ceres_catkin.git  # Caution! Make sure you do not have "suitesparse" in your Catkin workspace
    git clone https://github.com/ethz-asl/glog_catkin.git
    git clone https://github.com/ethz-asl/gflags_catkin.git
    git clone https://github.com/ethz-asl/catkin_simple.git
    git clone https://github.com/ros-drivers/velodyne.git  # For 3D SLAM with Velodynes

    # Build everything in your Catkin workspace
    cd ~/catkin_ws
    catkin_make_isolated
    source devel_isolated/setup.bash

### Using wstool

If cartographer_ros is to be used as part of a pre-existing workspace/existing project, using [wstool](http://wiki.ros.org/wstool) is recommended.

    # Enter workspace root (i.e. the folder that has "src" as a subfolder)
    # Merge the cartographer_ros rosinstall file
    wstool merge https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall

    # Update workspace
    wstool update

    # Build workspace contents. It is recommended to use catkin tools:
    catkin build
