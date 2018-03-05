# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM ros:kinetic

ARG CARTOGRAPHER_VERSION=master

# Xenial's base image doesn't ship with sudo.
RUN apt-get update && apt-get install -y sudo time && rm -rf /var/lib/apt/lists/*
# First, we invalidate the entire cache if googlecartographer/cartographer has
# changed. This file's content changes whenever master changes. See:
# http://stackoverflow.com/questions/36996046/how-to-prevent-dockerfile-caching-git-clone
ADD https://api.github.com/repos/googlecartographer/cartographer/git/refs/heads/master \
    cartographer_ros/cartographer_version.json

# wstool needs the updated rosinstall file to clone the correct repos.
COPY cartographer_ros.rosinstall cartographer_ros/
COPY scripts/prepare_jenkins_catkin_workspace.sh cartographer_ros/scripts/

# Invalidates the Docker cache to ensure this command is always executed.
ARG CACHEBUST=1
RUN CARTOGRAPHER_VERSION=$CARTOGRAPHER_VERSION \
    cartographer_ros/scripts/prepare_jenkins_catkin_workspace.sh

# rosdep needs the updated package.xml files to install the correct debs.
COPY cartographer_ros/package.xml catkin_ws/src/cartographer_ros/cartographer_ros/
COPY cartographer_ros_msgs/package.xml catkin_ws/src/cartographer_ros/cartographer_ros_msgs/
COPY cartographer_rviz/package.xml catkin_ws/src/cartographer_ros/cartographer_rviz/
COPY scripts/install_debs.sh cartographer_ros/scripts/
RUN cartographer_ros/scripts/install_debs.sh && rm -rf /var/lib/apt/lists/*

# Install proto3.
RUN /catkin_ws/src/cartographer/scripts/install_proto3.sh

# Build, install, and test all packages individually to allow caching. The
# ordering of these steps must match the topological package ordering as
# determined by Catkin.
COPY scripts/install.sh cartographer_ros/scripts/
COPY scripts/catkin_test_results.sh cartographer_ros/scripts/

COPY cartographer_ros_msgs catkin_ws/src/cartographer_ros/cartographer_ros_msgs/
RUN cartographer_ros/scripts/install.sh --pkg cartographer_ros_msgs && \
    cartographer_ros/scripts/install.sh --pkg cartographer_ros_msgs \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_ros_msgs

RUN cartographer_ros/scripts/install.sh --pkg ceres-solver

RUN cartographer_ros/scripts/install.sh --pkg cartographer && \
    cartographer_ros/scripts/install.sh --pkg cartographer --make-args test

COPY cartographer_ros catkin_ws/src/cartographer_ros/cartographer_ros/
RUN cartographer_ros/scripts/install.sh --pkg cartographer_ros && \
    cartographer_ros/scripts/install.sh --pkg cartographer_ros \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_ros

COPY cartographer_rviz catkin_ws/src/cartographer_ros/cartographer_rviz/
RUN cartographer_ros/scripts/install.sh --pkg cartographer_rviz && \
    cartographer_ros/scripts/install.sh --pkg cartographer_rviz \
        --catkin-make-args run_tests && \
    cartographer_ros/scripts/catkin_test_results.sh build_isolated/cartographer_rviz

RUN cartographer_ros/scripts/install.sh --pkg cartographer_toru
RUN cartographer_ros/scripts/install.sh --pkg cartographer_fetch

COPY scripts/ros_entrypoint.sh /
# A BTRFS bug may prevent us from cleaning up these directories.
# https://btrfs.wiki.kernel.org/index.php/Problem_FAQ#I_cannot_delete_an_empty_directory
RUN rm -rf cartographer_ros catkin_ws || true

RUN sudo apt-get update
RUN sudo apt-get -y install openjdk-8-jdk python-pip

ENV HOME /home/jenkins
RUN addgroup --system --gid 10000 jenkins
RUN adduser --system --ingroup jenkins --home $HOME --uid 10000 jenkins

LABEL Description="This is a base image, which provides the Jenkins agent executable (slave.jar)" Vendor="Jenkins project" Version="3.17"

ARG VERSION=3.17
ARG AGENT_WORKDIR=/home/jenkins/agent

RUN curl --create-dirs -sSLo /usr/share/jenkins/slave.jar https://repo.jenkins-ci.org/public/org/jenkins-ci/main/remoting/${VERSION}/remoting-${VERSION}.jar \
 && chmod 755 /usr/share/jenkins \
 && chmod 644 /usr/share/jenkins/slave.jar
# USER jenkins
ENV AGENT_WORKDIR=${AGENT_WORKDIR}
RUN mkdir /home/jenkins/.jenkins && mkdir -p ${AGENT_WORKDIR}

VOLUME /home/jenkins/.jenkins
VOLUME ${AGENT_WORKDIR}
WORKDIR /home/jenkins

COPY jenkins/jenkins-slave /usr/local/bin/jenkins-slave

ENV CLOUDSDK_CORE_DISABLE_PROMPTS 1
ENV PATH /opt/google-cloud-sdk/bin:$PATH

USER root

# Install Google Cloud Components
RUN curl https://sdk.cloud.google.com | bash && mv google-cloud-sdk /opt
RUN gcloud components install kubectl

RUN pip install --upgrade google-cloud-datastore
RUN pip install --upgrade google-cloud-bigquery
COPY jenkins/worker.py /worker.py

# USER root
ENTRYPOINT ["jenkins-slave"]
