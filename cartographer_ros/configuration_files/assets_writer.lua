VOXEL_SIZE = 5e-2

options = {
  tracking_frame = "base_link",
  pipeline = {
    -- {
      -- action = "fixed_ratio_sampler",
      -- sampling_ratio = 1.,
    -- },
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },
    -- {
      -- action = "voxel_filter_and_remove_moving_objects",
      -- voxel_size = VOXEL_SIZE,
    -- },
    {
      action = "dump_num_points",
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all",
      transform = {
        translation = { 0., 0., 0. },
        rotation = { 0. , 0., math.pi, },
      },
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all",
      transform = {
        translation = { 0., 0., 0. },
        rotation = { 0., -math.pi / 2., 0., },
      },
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all",
      transform = {
        translation = { 0., 0., 0. },
        rotation = { 0. , 0., -math.pi / 2, },
      },
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      separate_floors = true,
      filename = "xray_yz_level_",
      transform = {
        translation = { 0., 0., 0. },
        rotation = { 0. , 0., math.pi, },
      },
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      separate_floors = true,
      filename = "xray_xy_level_",
      transform = {
        translation = { 0., 0., 0. },
        rotation = { 0., -math.pi / 2., 0., },
      },
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      separate_floors = true,
      filename = "xray_xz_level_",
      transform = {
        translation = { 0., 0., 0. },
        rotation = { 0. , 0., -math.pi / 2, },
      },
    },
    -- {
      -- action = "write_xyz",
      -- filename = "points.xyz",
    -- },
    {
      action = "write_ply",
      filename = "points.ply",
    },
  }
}

return options
