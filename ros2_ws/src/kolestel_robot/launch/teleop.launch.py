#!/usr/bin/env python3
"""
Launch file for teleoperation.
Includes keyboard and joystick control options.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('kolestel_robot')
    
    # Launch configurations
    use_joy = LaunchConfiguration('use_joy')
    joy_dev = LaunchConfiguration('joy_dev')
    
    # Declare arguments
    declare_use_joy = DeclareLaunchArgument(
        'use_joy',
        default_value='false',
        description='Use joystick for control (otherwise keyboard)'
    )
    
    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device'
    )
    
    # Keyboard teleop
    teleop_keyboard = ExecuteProcess(
        cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
             '--ros-args', '--remap', 'cmd_vel:=/cmd_vel'],
        output='screen',
        condition=UnlessCondition(use_joy)
    )
    
    # Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }],
        condition=IfCondition(use_joy)
    )
    
    # Teleop joy
    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'teleop_joy.yaml')],
        remappings=[('cmd_vel', '/cmd_vel')],
        condition=IfCondition(use_joy)
    )
    
    return LaunchDescription([
        declare_use_joy,
        declare_joy_dev,
        teleop_keyboard,
        joy_node,
        teleop_joy
    ])
