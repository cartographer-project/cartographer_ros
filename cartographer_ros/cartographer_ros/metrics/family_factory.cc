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

#include "cartographer_ros/metrics/family_factory.h"

#include "cartographer/common/make_unique.h"
#include "cartographer_ros/metrics/internal/counter.h"
#include "cartographer_ros/metrics/internal/gauge.h"
#include "cartographer_ros/metrics/internal/histogram.h"
#include "cartographer_ros_msgs/Metrics.h"

namespace cartographer_ros {
namespace metrics {
using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

FamilyFactory::FamilyFactory() {}
::cartographer::metrics::Family<::cartographer::metrics::Counter>*
FamilyFactory::NewCounterFamily(const std::string& name,
                                const std::string& description) {
  auto wrapper =
      ::cartographer::common::make_unique<CounterFamily>(name, description);
  auto* ptr = wrapper.get();
  counter_families_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Gauge>*
FamilyFactory::NewGaugeFamily(const std::string& name,
                              const std::string& description) {
  auto wrapper =
      ::cartographer::common::make_unique<GaugeFamily>(name, description);
  auto* ptr = wrapper.get();
  gauge_families_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Histogram>*
FamilyFactory::NewHistogramFamily(const std::string& name,
                                  const std::string& description,
                                  const BucketBoundaries& boundaries) {
  auto wrapper = ::cartographer::common::make_unique<HistogramFamily>(
      name, description, boundaries);
  auto* ptr = wrapper.get();
  histogram_families_.emplace_back(std::move(wrapper));
  return ptr;
}

cartographer_ros_msgs::Metrics FamilyFactory::CollectMetrics() const {
  cartographer_ros_msgs::Metrics metrics_msg;
  for (const auto& counter_family : counter_families_) {
    metrics_msg.counter_families.push_back(counter_family->ToRosMessage());
  }
  for (const auto& gauge_family : gauge_families_) {
    metrics_msg.gauge_families.push_back(gauge_family->ToRosMessage());
  }
  for (const auto& histogram_family : histogram_families_) {
    metrics_msg.histogram_families.push_back(histogram_family->ToRosMessage());
  }
  return metrics_msg;
}

}  // namespace metrics
}  // namespace cartographer_ros
