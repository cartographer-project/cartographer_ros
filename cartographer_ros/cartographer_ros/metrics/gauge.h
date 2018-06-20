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

#ifndef CARTOGRAPHER_ROS_METRICS_GAUGE_H
#define CARTOGRAPHER_ROS_METRICS_GAUGE_H

#include <atomic>

#include "cartographer/metrics/gauge.h"

namespace cartographer_ros {
namespace metrics {

class Gauge : public ::cartographer::metrics::Gauge {
 public:
  void Decrement(const double by_value) override { Add(-1. * by_value); }

  void Decrement() override { Decrement(1.); }

  void Increment(const double by_value) override { Add(by_value); }

  void Increment() override { Increment(1.); }

  void Set(double value) override {
    double expected = value_.load();
    while (!value_.compare_exchange_weak(expected, value)) {
      ;
    }
  }
  double Value() { return value_.load(); }

 private:
  void Add(const double value) {
    double current = Value();
    Set(current + value);
  }

  std::atomic<double> value_{0.};
};
}  // namespace metrics
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_METRICS_GAUGE_H
