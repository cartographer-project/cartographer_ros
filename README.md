# Cartographer Project Overview

See https://github.com/googlecartographer/cartographer

## Installation

On Ubuntu 14.04 (Trusty) with ROS Indigo installed:

    # Install the required libraries that are available as debs
    sudo apt-get install \
      ros-indigo-tf2-eigen \
      g++ \
      google-mock \
      libboost-all-dev \
      liblua5.2-dev \
      libprotobuf-dev \
      libwebp-dev \
      protobuf-compiler \
      python-sphinx \
      libblas-dev \
      liblapack-dev \
      libpcap-dev  # For 3D SLAM with Velodynes

    # Set up your Catkin workspace
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    source /opt/ros/indigo/setup.bash
    catkin_init_workspace

    # Clone the necessary repos into your Catkin workspace
    git clone https://github.com/googlecartographer/cartographer.git
    git clone https://github.com/googlecartographer/cartographer_ros.git
    git clone https://github.com/ethz-asl/ceres_catkin.git
    git clone https://github.com/ethz-asl/suitesparse.git
    git clone https://github.com/ethz-asl/glog_catkin.git
    git clone https://github.com/ethz-asl/gflags_catkin.git
    git clone https://github.com/ethz-asl/catkin_simple.git
    git clone https://github.com/ros-drivers/velodyne.git  # For 3D SLAM with Velodynes

    # Build everything in your Catkin workspace
    cd ~/catkin_ws
    catkin_make_isolated
    source devel_isolated/setup.bash
