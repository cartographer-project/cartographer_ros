-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.


VOXEL_SIZE = 2e-2


include "transform.lua"

options = {
  tracking_frame = "base_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 30.,
    },
    {
        action = "vertical_range_filter",
        min_z = -0.2,
        max_z = 1.1
      },
    {
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = VOXEL_SIZE,
    },

    {
      action = "fixed_ratio_sampler",
      sampling_ratio = .999,
    },
    {
      action = "dump_num_points",
    },

    {
      action = "intensity_to_color",
      frame_id = horizontal_vlp16_link,
      min_intensity = 0,
      max_intensity = 100,
    },
    {
      action = "intensity_to_color",
      frame_id = vertical_vlp16_link,
      min_intensity = 0,
      max_intensity = 100,
    },
    {
      action = "intensity_to_color",
      frame_id = tof_camera_11_link,
      min_intensity = 0,
      max_intensity = 255,
    },
    {
      action = "write_ply",
      filename = "points.ply",
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all_intensity",
      transform = YZ_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all_intensity",
      transform = XY_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all_intensity",
      transform = XZ_TRANSFORM,
    },
   
    

   
  }
}
return options
