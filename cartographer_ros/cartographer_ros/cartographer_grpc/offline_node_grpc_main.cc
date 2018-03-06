/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/cloud/client/map_builder_stub.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "ros/ros.h"

DEFINE_string(server_address, "localhost:50051",
              "gRPC server address to "
              "stream the sensor data to.");

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::ros::init(argc, argv, "cartographer_grpc_offline_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  const cartographer_ros::MapBuilderFactory map_builder_factory =
      [](const ::cartographer::mapping::proto::MapBuilderOptions&) {
        return ::cartographer::common::make_unique<
            ::cartographer::cloud::MapBuilderStub>(FLAGS_server_address);
      };

  cartographer_ros::RunOfflineNode(map_builder_factory);

  ::ros::shutdown();
}
