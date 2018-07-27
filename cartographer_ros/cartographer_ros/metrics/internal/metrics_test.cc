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

#include <algorithm>
#include <array>
#include <numeric>

#include "cartographer/metrics/histogram.h"
#include "cartographer_ros/metrics/internal/counter.h"
#include "cartographer_ros/metrics/internal/gauge.h"
#include "cartographer_ros/metrics/internal/histogram.h"
#include "gtest/gtest.h"

namespace cartographer_ros {
namespace metrics {

TEST(Metrics, GaugeTest) {
  Gauge gauge({});
  EXPECT_EQ(gauge.Value(), 0.);
  gauge.Increment(1.2);
  EXPECT_EQ(gauge.Value(), 1.2);
  gauge.Increment();
  EXPECT_EQ(gauge.Value(), 2.2);
  gauge.Decrement(2.2);
  EXPECT_EQ(gauge.Value(), 0.);
  gauge.Decrement();
  EXPECT_EQ(gauge.Value(), -1.);
}

TEST(Metrics, CounterTest) {
  Counter counter({});
  EXPECT_EQ(counter.Value(), 0.);
  counter.Increment(1.2);
  EXPECT_EQ(counter.Value(), 1.2);
  counter.Increment(0.8);
  EXPECT_EQ(counter.Value(), 2.);
  counter.Increment();
  EXPECT_EQ(counter.Value(), 3.);
}

TEST(Metrics, HistogramFixedWidthTest) {
  auto boundaries = ::cartographer::metrics::Histogram::FixedWidth(1, 3);
  Histogram histogram({}, boundaries);

  // Observe some values that fit in finite buckets.
  std::array<double, 3> values = {{0., 2, 2.5}};
  for (const auto& value : values) {
    histogram.Observe(value);
  }
  //     1     2     3    inf
  //  1  |  0  |  2  |  0  |
  EXPECT_EQ(histogram.CountsByBucket()[1], 1);
  EXPECT_EQ(histogram.CountsByBucket()[2], 0);
  EXPECT_EQ(histogram.CountsByBucket()[3], 2);
  EXPECT_EQ(histogram.CumulativeCount(), values.size());
  EXPECT_EQ(histogram.Sum(), std::accumulate(values.begin(), values.end(), 0.));

  // Values above the last bucket boundary should go to the "infinite" bucket.
  histogram.Observe(3.5);
  //     1     2     3    inf
  //  1  |  0  |  2  |  1  |
  EXPECT_EQ(histogram.CountsByBucket()[kInfiniteBoundary], 1);
}

TEST(Metrics, HistogramScaledPowersOfTest) {
  auto boundaries =
      ::cartographer::metrics::Histogram::ScaledPowersOf(2, 1, 2048);
  Histogram histogram({}, boundaries);

  // Observe some values that fit in finite buckets.
  std::array<double, 3> values = {{256, 512, 666}};
  for (const auto& value : values) {
    histogram.Observe(value);
  }
  // ... 256   512   1024  inf
  // ...  |  1  |  2  |  0  |
  EXPECT_EQ(histogram.CountsByBucket()[256], 0);
  EXPECT_EQ(histogram.CountsByBucket()[512], 1);
  EXPECT_EQ(histogram.CountsByBucket()[1024], 2);
  EXPECT_EQ(histogram.CumulativeCount(), values.size());
  EXPECT_EQ(histogram.Sum(), std::accumulate(values.begin(), values.end(), 0.));

  // Values above the last bucket boundary should go to the "infinite" bucket.
  histogram.Observe(2048);
  // ... 256   512   1024  inf
  // ...  |  1  |  2  |  1  |
  EXPECT_EQ(histogram.CountsByBucket()[kInfiniteBoundary], 1);
}

}  // namespace metrics
}  // namespace cartographer_ros
