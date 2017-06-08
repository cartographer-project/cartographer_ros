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

ColorManager::ColorManager(const double initial_hue, const double saturation,
                           const double value)
    : current_hue_(initial_hue), saturation_(saturation), value_(value) {}

ColorManager::ColorRGB ColorManager::HSVToRGB(const double h, const double s,
                                              const double v) {
  ColorRGB out_rgb;
  double p, q, t, f, h_6;

  h_6 = (h == 1.) ? 0. : 6 * h;
  int h_i = std::floor(h_6);
  f = h_6 - h_i;

  p = v * (1. - s);
  q = v * (1. - f * s);
  t = v * (1. - (1. - f) * s);

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
    out_rgb = {0., 0., 0.};

  return out_rgb;
}

ColorManager::ColorRGB ColorManager::GetNextColor() {
  // Uniform color sampling using the golden ratio from
  // http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
  constexpr double golden_ratio_conjugate = (std::sqrt(5) - 1) / 2.;
  ColorRGB next_color = HSVToRGB(current_hue_, saturation_, value_);
  current_hue_ += golden_ratio_conjugate;
  current_hue_ = std::fmod(current_hue_, 1.);
  generated_colors_.push_back(next_color);
  return next_color;
}

ColorManager::ColorRGB ColorManager::GetColor(int id) {
  CHECK_GE(id, 0);
  while ((int)generated_colors_.size() <= id) {
    GetNextColor();
  }
  return generated_colors_[id];
}

}  // namespace cartographer_ros
