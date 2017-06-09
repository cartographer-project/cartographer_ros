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

constexpr float kColorManagerSaturation = 0.85f;
constexpr float kColorManagerValue = 0.77f;
constexpr float kGoldenRatioConjugate = (std::sqrt(5) - 1) / 2.f;

ColorManager::ColorManager(const float initial_hue)
    : initial_hue_(initial_hue) {}

ColorManager::ColorRGB ColorManager::HSVToRGB(const float h, const float s,
                                              const float v) {
  ColorRGB out_rgb;
  float p, q, t, f, h_6;

  h_6 = (h == 1.f) ? 0.f : 6 * h;
  int h_i = std::floor(h_6);
  f = h_6 - h_i;

  p = v * (1.f - s);
  q = v * (1.f - f * s);
  t = v * (1.f - (1.f - f) * s);

  if (h_i == 0)
    out_rgb = {v, t, p};
  else if (h_i == 1)
    out_rgb = {q, v, p};
  else if (h_i == 2)
    out_rgb = {p, v, t};
  else if (h_i == 3)
    out_rgb = {p, q, v};
  else if (h_i == 4)
    out_rgb = {t, p, v};
  else if (h_i == 5)
    out_rgb = {v, p, q};
  else
    out_rgb = {0.f, 0.f, 0.f};

  return out_rgb;
}

ColorManager::ColorRGB ColorManager::GetColor(int id) {
  CHECK_GE(id, 0);
  // Uniform color sampling using the golden ratio from
  // http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
  const float hue =
      std::fmod(initial_hue_ + kGoldenRatioConjugate * (id + 8), 1.f);
  return HSVToRGB(hue, kColorManagerSaturation, kColorManagerValue);
}

}  // namespace cartographer_ros
