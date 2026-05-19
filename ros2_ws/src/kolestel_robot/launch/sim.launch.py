#!/usr/bin/env python3
"""
Main simulation launch file for Kolestel 001 robot.
Launches Gazebo Sim (Harmonic), spawns the robot, sets up ros_gz_bridge,
robot_state_publisher, and optionally RViz.

Usage:
    ros2 launch kolestel_robot sim.launch.py
    ros2 launch kolestel_robot sim.launch.py world:=obstacles.sdf rviz:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('kolestel_robot')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file name (in worlds/ directory)'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Gazebo world name (usually the world file base name, e.g. empty for empty.sdf)'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'sim.rviz'),
        description='RViz configuration file'
    )
    
    # Robot description from xacro (for robot_state_publisher)
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot_gz.urdf.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # ==================== GAZEBO SIM ====================
    # Launch Gazebo Sim with the specified world
    world_path = PathJoinSubstitution([
        FindPackageShare('kolestel_robot'),
        'worlds',
        world
    ])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )
    
    # ==================== GENERATE URDF FILE ====================
    # ros_gz_sim create can reliably spawn from a URDF file.
    urdf_tmp = os.path.join('/tmp', 'kolestel_robot.urdf')
    generate_urdf = ExecuteProcess(
        cmd=['bash', '-lc', f"xacro {xacro_file} > {urdf_tmp}"],
        output='screen'
    )

    # ==================== SPAWN ROBOT ====================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', world_name,
            '-name', 'kolestel_robot',
            '-file', urdf_tmp,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Delay spawn a bit so Gazebo is fully ready
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])
    
    # ==================== ROBOT STATE PUBLISHER ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ==================== ROS_GZ_BRIDGE ====================
    # Bridge for clock, cmd_vel, odom, tf, and sensors
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velocity command
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # TF
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # LiDAR
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Front camera
            '/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image',
            # Rear camera
            '/camera/rear/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ==================== RVIZ ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_world,
        declare_world_name,
        declare_rviz,
        declare_rviz_config,
        
        # Launch Gazebo Sim
        gazebo,

        # Generate URDF file for spawning
        generate_urdf,
        
        # Robot state publisher
        robot_state_publisher,

        # Spawn robot (delayed)
        delayed_spawn,
        
        # ROS-Gazebo bridge
        ros_gz_bridge,
        
        # RViz
        rviz_node,
    ])
