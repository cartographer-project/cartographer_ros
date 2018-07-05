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

#ifndef CARTOGRAPHER_ROS_METRICS_INTERNAL_GAUGE_H
#define CARTOGRAPHER_ROS_METRICS_INTERNAL_GAUGE_H

#include <atomic>
#include <map>
#include <string>

#include "cartographer/metrics/gauge.h"
#include "cartographer_ros_msgs/Metric.h"

namespace cartographer_ros {
namespace metrics {

class Gauge : public ::cartographer::metrics::Gauge {
 public:
  explicit Gauge(const std::map<std::string, std::string>& labels)
      : labels_(labels) {}

  void Decrement(const double value) override { Add(-1. * value); }

  void Decrement() override { Decrement(1.); }

  void Increment(const double value) override { Add(value); }

  void Increment() override { Increment(1.); }

  void Set(double value) override { value_.store(value); }
  double Value() const { return value_.load(); }

  cartographer_ros_msgs::Metric ToRosMessage() {
    cartographer_ros_msgs::Metric msg;
    msg.type = cartographer_ros_msgs::Metric::TYPE_GAUGE;
    for (const auto& label : labels_) {
      cartographer_ros_msgs::MetricLabel label_msg;
      label_msg.key = label.first;
      label_msg.value = label.second;
      msg.labels.push_back(label_msg);
    }
    msg.value = Value();
    return msg;
  }

 private:
  void Add(const double value) {
    double expected = value_.load();
    double new_value = expected + value;
    while (!value_.compare_exchange_weak(expected, new_value)) {
      new_value = expected + value;
    }
  }

  const std::map<std::string, std::string> labels_;
  std::atomic<double> value_{0.};
};

}  // namespace metrics
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_METRICS_INTERNAL_GAUGE_H
