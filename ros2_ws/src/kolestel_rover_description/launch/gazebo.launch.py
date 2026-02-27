import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg = get_package_share_directory('kolestel_rover_description')
    xacro_file = os.path.join(pkg, 'urdf', 'kolestel_rover.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'warehouse.sdf')
    bridge_cfg  = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 1. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'{world_file} -r -v 3'}.items()
    )

    # 3. Spawn робота
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'kolestel_rover',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ]
    )

    # 4. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_cfg,
            'use_sim_time': True
        }]
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
               'kolestel_rover/base_footprint/gpu_lidar',
               'laser_link']
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
               'kolestel_rover/base_footprint/imu_sensor',
               'imu_link']
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0',
               'kolestel_rover/base_footprint/depth_camera',
               'camera_depth_frame']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rsp,
        gz_sim,
        lidar_tf,
        imu_tf,
        camera_tf,
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=5.0, actions=[bridge]),
    ])
