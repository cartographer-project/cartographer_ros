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

from os import getenv

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import Shutdown

def generate_launch_description():

    ## ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument('bag_filename')
    no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value = 'False')
    keep_running_arg = DeclareLaunchArgument('keep_running', default_value = 'False')

    ## ***** Nodes *****
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz'],
        parameters = [{'use_sim_time': True}],
        condition = UnlessCondition(LaunchConfiguration('no_rviz'))
    )

    cartographer_offline_node_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_offline_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basenames', 'mir-100-mapping.lua',
            '-urdf_filenames', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/urdf/mir-100.urdf',
            '-use_bag_transforms', 'false',
            '-keep_running', LaunchConfiguration('keep_running'),
            '-bag_filenames', LaunchConfiguration('bag_filename')],
        remappings = [
            ('f_scan', 'scan_1'),
            ( 'b_scan', 'scan_2'),
            ( 'imu_data','imu'),
            ('odom','ignore_odom'),
            ('odom_enc','odom')],
        output = 'screen'
        )


    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        no_rviz_arg,
        keep_running_arg,

        # Nodes
        rviz_node,
        cartographer_offline_node_node,
    ])
