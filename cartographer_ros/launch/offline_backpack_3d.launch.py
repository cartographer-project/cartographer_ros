"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ## ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames')
    no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value='false')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_3d.rviz')
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files')
    configuration_basenames_arg = DeclareLaunchArgument('configuration_basenames', default_value = 'backpack_3d.lua')
    urdf_filenames_arg = DeclareLaunchArgument('urdf_filenames', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/urdf/backpack_3d.urdf')

    ## ***** Nodes *****
    offline_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('cartographer_ros').find('cartographer_ros') + '/launch/offline_node.launch.py'),
        launch_arguments = {
            'bag_filenames': LaunchConfiguration('bag_filenames'),
            'no_rviz': LaunchConfiguration('no_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'configuration_directory': LaunchConfiguration('configuration_directory'),
            'configuration_basenames': LaunchConfiguration('configuration_basenames'),
            'urdf_filenames': LaunchConfiguration('urdf_filenames')}.items(),

        )
    set_remap1 = SetRemap('horizontal_laser_3d', 'points2_1')
    set_remap2 = SetRemap('vertical_laser_3d', 'points2_2')

    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        no_rviz_arg,
        rviz_config_arg,
        configuration_directory_arg,
        configuration_basenames_arg,
        urdf_filenames_arg,

        # Nodes
        set_remap1,
        set_remap2,
        offline_node_launch,
    ])
