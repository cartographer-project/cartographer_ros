/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer_ros/color_manager.h"

#include <cmath>

#include "glog/logging.h"

namespace cartographer_ros {

constexpr float kColorManagerInitialHue = 0.69;
constexpr float kColorManagerSaturation = 0.85f;
constexpr float kColorManagerValue = 0.77f;
constexpr float kGoldenRatioConjugate = (std::sqrt(5) - 1) / 2.f;

ColorRGB HSVToRGB(const float h, const float s, const float v) {
  const float h_6 = (h == 1.f) ? 0.f : 6 * h;
  const int h_i = std::floor(h_6);
  const float f = h_6 - h_i;

  const float p = v * (1.f - s);
  const float q = v * (1.f - f * s);
  const float t = v * (1.f - (1.f - f) * s);

  if (h_i == 0) {
    return {v, t, p};
  } else if (h_i == 1) {
    return {q, v, p};
  } else if (h_i == 2) {
    return {p, v, t};
  } else if (h_i == 3) {
    return {p, q, v};
  } else if (h_i == 4) {
    return {t, p, v};
  } else if (h_i == 5) {
    return {v, p, q};
  } else {
    return {0.f, 0.f, 0.f};
  }
}

ColorRGB GetColor(int id) {
  CHECK_GE(id, 0);
  // Uniform color sampling using the golden ratio from
  // http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
  const float hue =
      std::fmod(kColorManagerInitialHue + kGoldenRatioConjugate * id, 1.f);
  return HSVToRGB(hue, kColorManagerSaturation, kColorManagerValue);
}

}  // namespace cartographer_ros
