/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer_ros/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "glog/log_severity.h"

#define ROS_INFO_STREAM(str) std::cout << str << std::endl;
#define ROS_WARN_STREAM(str) std::cout << str << std::endl;
#define ROS_ERROR_STREAM(str) std::cerr << str << std::endl;
#define ROS_FATAL_STREAM(str) std::cerr << str << std::endl;

namespace cartographer_ros {

namespace {

const char* GetBasename(const char* filepath) {
  const char* base = std::strrchr(filepath, '/');
  return base ? (base + 1) : filepath;
}

}  // namespace

ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }

void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  (void)base_filename;
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
  switch (severity) {
    case ::google::GLOG_INFO:
      ROS_INFO_STREAM(message_string);
      break;

    case ::google::GLOG_WARNING:
      ROS_WARN_STREAM(message_string);
      break;

    case ::google::GLOG_ERROR:
      ROS_ERROR_STREAM(message_string);
      break;

    case ::google::GLOG_FATAL:
      ROS_FATAL_STREAM(message_string);
      will_die_ = true;
      break;
  }
}

void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace cartographer_ros
