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

#include "cartographer/common/make_unique.h"

#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/tf_bridge.h"

namespace cartographer_ros {

TfBridge::TfBridge(const std::string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string& frame_id) const {
  tf2::Duration timeout(tf2::durationFromSec(lookup_transform_timeout_sec_));
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
  try {
    const ::builtin_interfaces::msg::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, tf2::TimePointZero,
                              timeout)
            .header.stamp;
    const ::builtin_interfaces::msg::Time requested_time = ToRos(time);
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted{std::chrono::nanoseconds{requested_time.sec*1000000000LL+requested_time.nanosec}};
    std::chrono::system_clock::time_point recovered = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted);

    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> converted_tf_time{std::chrono::nanoseconds{latest_tf_time.sec*1000000000LL+latest_tf_time.nanosec}};
    std::chrono::system_clock::time_point recovered_tf_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(converted_tf_time);

    if (recovered_tf_time >= recovered) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = tf2::durationFromSec(0.0);
    }

    // TODO(clalancette): We are currently having some problems where the
    // conversions from tf2 time to cartographer time is rounding.  In turn
    // this causes tf2 to complain about extrapolating into the future.  For
    // right now, just always ask for the data "now", which works around the
    // problem.  We'll need to address this for real in the future.
    //return ::cartographer::common::make_unique<
    //    ::cartographer::transform::Rigid3d>(ToRigid3d(buffer_->lookupTransform(
    //    tracking_frame_, frame_id, recovered, timeout)));
    return ::cartographer::common::make_unique<
        ::cartographer::transform::Rigid3d>(ToRigid3d(buffer_->lookupTransform(
        tracking_frame_, frame_id, recovered, timeout)));
  } catch (const tf2::TransformException& ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

}  // namespace cartographer_ros
