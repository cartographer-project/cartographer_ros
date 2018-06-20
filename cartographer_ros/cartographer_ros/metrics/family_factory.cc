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
#include "cartographer_ros/metrics/counter.h"
#include "cartographer_ros/metrics/gauge.h"
#include "cartographer_ros/metrics/histogram.h"

namespace cartographer_ros {
namespace metrics {
namespace {

using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

class CounterFamily
    : public ::cartographer::metrics::Family<::cartographer::metrics::Counter> {
 public:
  Counter* Add(const std::map<std::string, std::string>& labels) override {
    auto wrapper = ::cartographer::common::make_unique<Counter>();
    auto* ptr = wrapper.get();
    wrappers_.emplace_back(std::move(wrapper));
    return ptr;
  }

 private:
  std::vector<std::unique_ptr<Counter>> wrappers_;
};

class GaugeFamily
    : public ::cartographer::metrics::Family<::cartographer::metrics::Gauge> {
 public:
  Gauge* Add(const std::map<std::string, std::string>& labels) override {
    auto wrapper = ::cartographer::common::make_unique<Gauge>();
    auto* ptr = wrapper.get();
    wrappers_.emplace_back(std::move(wrapper));
    return ptr;
  }

 private:
  std::vector<std::unique_ptr<Gauge>> wrappers_;
};

class HistogramFamily : public ::cartographer::metrics::Family<
                            ::cartographer::metrics::Histogram> {
 public:
  HistogramFamily(const BucketBoundaries& boundaries)
      : boundaries_(boundaries) {}

  Histogram* Add(const std::map<std::string, std::string>& labels) override {
    auto wrapper = ::cartographer::common::make_unique<Histogram>(boundaries_);
    auto* ptr = wrapper.get();
    wrappers_.emplace_back(std::move(wrapper));
    return ptr;
  }

 private:
  std::vector<std::unique_ptr<Histogram>> wrappers_;
  const BucketBoundaries boundaries_;
};

}  // namespace

// TODO MichaelGrupp: initialize registry_
FamilyFactory::FamilyFactory() {}

::cartographer::metrics::Family<::cartographer::metrics::Counter>*
FamilyFactory::NewCounterFamily(const std::string& name,
                                const std::string& description) {
  auto wrapper = ::cartographer::common::make_unique<CounterFamily>();
  auto* ptr = wrapper.get();
  counters_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Gauge>*
FamilyFactory::NewGaugeFamily(const std::string& name,
                              const std::string& description) {
  auto wrapper = ::cartographer::common::make_unique<GaugeFamily>();
  auto* ptr = wrapper.get();
  gauges_.emplace_back(std::move(wrapper));
  return ptr;
}

::cartographer::metrics::Family<::cartographer::metrics::Histogram>*
FamilyFactory::NewHistogramFamily(const std::string& name,
                                  const std::string& description,
                                  const BucketBoundaries& boundaries) {
  auto wrapper =
      ::cartographer::common::make_unique<HistogramFamily>(boundaries);
  auto* ptr = wrapper.get();
  histograms_.emplace_back(std::move(wrapper));
  return ptr;
}

}  // namespace metrics
}  // namespace cartographer_ros
