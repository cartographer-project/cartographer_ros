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

#ifndef CARTOGRAPHER_ROS_COLOR_MANAGER_H_
#define CARTOGRAPHER_ROS_COLOR_MANAGER_H_

#include <vector>

namespace cartographer_ros {

class ColorManager {
 public:
  // A class for online generation of a colour palette, with every two direct
  //  successors having large contrast. All parameters are from [0, 1].
  ColorManager(const double initial_hue, const double saturation,
               const double value);

  struct ColorRGB {
    // r, g, b are from [0,1]
    double r;
    double g;
    double b;
  };

  ColorRGB GetNextColor();
  ColorRGB GetColor(int id);

 private:
  double current_hue_;
  double saturation_;
  double value_;
  std::vector<ColorRGB> generated_colors_;

  ColorRGB HSVToRGB(const double h, const double s, const double v);
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_COLOR_MANAGER_H_
