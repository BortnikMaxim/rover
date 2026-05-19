#!/usr/bin/env python3
"""
Launch file for SLAM (Simultaneous Localization and Mapping).
Uses slam_toolbox for online mapping.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('kolestel_robot')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dir, 'params', 'slam_params.yaml'),
        description='Full path to SLAM parameters file'
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_slam_params_file,
        slam_toolbox_node
    ])
