#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
#                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
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
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    
    package_name = 'nav_slam'
    config_dir = get_package_share_directory(package_name)
    
    # Build RViz config file path
    rviz_config_file = os.path.join(config_dir, 'config', 'rviz.rviz')

    # Hybrid A* path planning node
    hybrid_astar = Node(
        package='nav_slam',
        executable='hybrid_astar',
        name='hybrid_astar',
        output='screen',
    )
    
    # Map publisher node
    map_pub = Node(
        package='nav_slam',
        executable='map_pub',
        name='map_pub',
        output='screen',
    )
    
    # Odom to map TF publisher
    odom_map_tf = Node(
        package='nav_slam',
        executable='odom_map_tf',
        name='odom_map_tf',
        output='screen',
    )
    
    # Point cloud publisher
    points_pub_map = Node(
        package='nav_slam',
        executable='points_pub_map',
        name='points_pub_map',
        output='screen',
    )
    
    # Navigation controller (pure pursuit)
    start_nav = Node(
        package='nav_slam',
        executable='start_nav',
        name='start_nav',
        output='screen',
    )
    
    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
  
    # Add all nodes to launch description
    ld.add_action(hybrid_astar)
    ld.add_action(map_pub)
    ld.add_action(odom_map_tf)
    ld.add_action(points_pub_map)
    ld.add_action(start_nav)
    ld.add_action(rviz2_node)

    return ld
