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

namespace cartographer_ros {
namespace metrics {

class Gauge {
 public:
  void Decrement(const double decrement) { Add(-1. * decrement); }

  void Increment(const double increment) { Add(increment); }

  double Value() { return value_.load(); }

 private:
  void Add(const double value) {
    double expected = value_.load();
    while (!value_.compare_exchange_weak(expected, expected + value)) {
      ;
    }
  }

  std::atomic<double> value_{0.};
};
}
}

#endif
