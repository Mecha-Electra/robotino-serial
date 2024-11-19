# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.descriptions
from launch_ros.actions import Node


def generate_launch_description():

    # pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    # pkg_robotino_navigation = get_package_share_directory('robotino_navigation')
    pkg_robotino_sim = get_package_share_directory('neo_robotino_sim')
    pkg_robotino_serial = get_package_share_directory('robotino-serial')
    pkg_lidar = get_package_share_directory('sllidar_ros2')

    board = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pkg_robotino_serial, 'launch', 'robotino2.launch')),
    )

    LiDAR = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a2m8_launch.py')),
        launch_arguments={
            'frame_id':'lidar_link',
            'serial_port':'/dev/ttyUSB1'
        }.items(),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robotino_sim, 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'use_sim_time': 'false'
        }.items(),
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[os.path.join(pkg_robotino_serial, 'config', 'laser_filters.yaml')],
        remappings=[('/scan', 'scan'),  # Dados originais do LIDAR
                    ('scan_filtered', '/lidar')]  # Dados filtrados
    )

    return LaunchDescription([
        board,
        LiDAR,
        laser_filter_node,
        robot_description
    ])