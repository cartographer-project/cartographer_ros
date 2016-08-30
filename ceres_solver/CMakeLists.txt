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

cmake_minimum_required(VERSION 2.8.7)

project(ceres_solver)

include(ExternalProject)
set(TAG 6a13e39e8171f450fbb89188d97f198def81937e) # Version 1.11
ExternalProject_Add(ceres_src
  GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver
  GIT_TAG ${TAG}
  CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DBUILD_EXAMPLES=OFF
      -DBUILD_TESTING=OFF
)

# Ceres is installed via the ExternalProject_Add command above. However, we
# must provide this no-op install target to satisfy Catkin.
install(CODE "")
