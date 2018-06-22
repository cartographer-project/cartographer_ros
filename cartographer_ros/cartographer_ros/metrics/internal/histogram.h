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

#ifndef CARTOGRAPHER_ROS_METRICS_HISTOGRAM_H
#define CARTOGRAPHER_ROS_METRICS_HISTOGRAM_H

#include <algorithm>
#include <map>
#include <numeric>
#include <vector>

#include "cartographer/common/mutex.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer_ros/metrics/internal/counter.h"
#include "cartographer_ros_msgs/Histogram.h"
#include "glog/logging.h"

namespace cartographer_ros {
namespace metrics {

constexpr double kInfiniteBoundary = std::numeric_limits<double>::infinity();

using BucketBoundaries = ::cartographer::metrics::Histogram::BucketBoundaries;

class Histogram : public ::cartographer::metrics::Histogram {
 public:
  Histogram(const std::map<std::string, std::string>& labels,
            const BucketBoundaries& bucket_boundaries)
      : labels_(labels),
        bucket_boundaries_(bucket_boundaries),
        bucket_counts_(bucket_boundaries.size() + 1) {
    ::cartographer::common::MutexLocker lock(&mutex_);
    CHECK(std::is_sorted(std::begin(bucket_boundaries_),
                         std::end(bucket_boundaries_)));
  }

  Histogram(const BucketBoundaries& bucket_boundaries)
      : Histogram({}, bucket_boundaries) {}

  void Observe(double value) override {
    ::cartographer::common::MutexLocker lock(&mutex_);
    auto bucket_index =
        std::distance(bucket_boundaries_.begin(),
                      std::upper_bound(bucket_boundaries_.begin(),
                                       bucket_boundaries_.end(), value));
    sum_ += value;
    bucket_counts_[bucket_index] += 1;
  }

  std::map<double, double> CountsByBucket() {
    ::cartographer::common::MutexLocker lock(&mutex_);
    std::map<double, double> counts_by_bucket;
    // Add the finite buckets.
    for (size_t i = 0; i < bucket_boundaries_.size(); ++i) {
      counts_by_bucket[bucket_boundaries_[i]] = bucket_counts_[i];
    }
    // Add the "infinite" bucket.
    counts_by_bucket[kInfiniteBoundary] = bucket_counts_.back();
    return counts_by_bucket;
  }

  double Sum() {
    ::cartographer::common::MutexLocker lock(&mutex_);
    return sum_;
  }

  double CumulativeCount() {
    ::cartographer::common::MutexLocker lock(&mutex_);
    return std::accumulate(bucket_counts_.begin(), bucket_counts_.end(), 0.);
  }

  cartographer_ros_msgs::Histogram ToRosMessage() {
    cartographer_ros_msgs::Histogram msg;
    for (const auto& label : labels_) {
      cartographer_ros_msgs::Label label_msg;
      label_msg.first = label.first;
      label_msg.second = label.second;
      msg.labels.push_back(label_msg);
    }
    for (const auto& bucket : CountsByBucket()) {
      cartographer_ros_msgs::HistogramBucket bucket_msg;
      bucket_msg.bucket_boundary = bucket.first;
      bucket_msg.count = bucket.second;
      msg.counts_by_bucket.push_back(bucket_msg);
    }
    return msg;
  }

 private:
  cartographer::common::Mutex mutex_;
  const std::map<std::string, std::string> labels_;
  const BucketBoundaries bucket_boundaries_;
  std::vector<double> bucket_counts_ GUARDED_BY(mutex_);
  double sum_ GUARDED_BY(mutex_);
};
}  // namespace metrics
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_METRICS_HISTOGRAM_H
