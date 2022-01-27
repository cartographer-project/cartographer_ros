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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown

def generate_launch_description():

    ## ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames')
    no_rviz_arg = DeclareLaunchArgument('no_rviz')
    rviz_config_arg = DeclareLaunchArgument('rviz_config')
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory')
    configuration_basenames_arg = DeclareLaunchArgument('configuration_basenames')
    urdf_filenames_arg = DeclareLaunchArgument('urdf_filenames')

    ## ***** Nodes *****
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', LaunchConfiguration('rviz_config')],
        parameters = [{'use_sim_time': True}],
        condition = UnlessCondition(LaunchConfiguration('no_rviz'))
    )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    cartographer_offline_node_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_offline_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basenames', LaunchConfiguration('configuration_basenames'),
            '-urdf_filenames', LaunchConfiguration('urdf_filenames'),
            '-bag_filenames', LaunchConfiguration('bag_filenames')],
        output = 'screen'
        )


    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        no_rviz_arg,
        rviz_config_arg,
        configuration_directory_arg,
        configuration_basenames_arg,
        urdf_filenames_arg,

        # Nodes
        rviz_node,
        cartographer_occupancy_grid_node,
        cartographer_offline_node_node,
    ])
