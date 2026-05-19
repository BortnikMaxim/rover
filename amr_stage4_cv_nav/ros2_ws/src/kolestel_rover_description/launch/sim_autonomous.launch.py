"""
Autonomous-navigation mode with global + local costmap and selectable planner.

Supported algorithm values (launch arg `algorithm`):
  dijkstra | a_star | greedy_bfs | jps | theta_star | d_star_lite

For d_star_lite a separate node (dstar_lite_node) is used because it maintains
state across goals. For the other five, grid_planner_node runs all of them.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _planner_nodes(context, *args, **kwargs):
    pkg = get_package_share_directory('kolestel_rover_description')
    algorithm = LaunchConfiguration('algorithm').perform(context)
    common_params = {
        'use_sim_time': True,
        'map_pgm_path': os.path.join(pkg, 'maps', 'warehouse_nav2_map.pgm'),
        'map_yaml_path': os.path.join(pkg, 'maps', 'warehouse_nav2_map.yaml'),
        'base_frame': 'base_footprint',
        'map_frame': 'map',
        'robot_radius_m': 0.30,
        'occupancy_threshold': 200,
    }

    if algorithm == 'd_star_lite':
        return [Node(
            package='kolestel_rover_description',
            executable='dstar_lite_node.py',
            name='dstar_lite_node',
            output='screen',
            parameters=[common_params],
        )]
    return [Node(
        package='kolestel_rover_description',
        executable='grid_planner_node.py',
        name='grid_planner_node',
        output='screen',
        parameters=[{**common_params, 'algorithm': algorithm, 'use_external_costmap': True}],
    )]


def generate_launch_description():
    pkg = get_package_share_directory('kolestel_rover_description')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'publish_static_map_to_odom': 'true'}.items(),
    )

    global_costmap = Node(
        package='kolestel_rover_description',
        executable='global_costmap_node.py',
        name='global_costmap_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_pgm_path': os.path.join(pkg, 'maps', 'warehouse_nav2_map.pgm'),
            'map_yaml_path': os.path.join(pkg, 'maps', 'warehouse_nav2_map.yaml'),
            'map_frame': 'map',
            'robot_radius_m': 0.30,
            'inflation_radius_m': 0.80,
            'cost_decay_rate': 5.0,
            'occupancy_threshold': 200,
        }],
    )

    local_costmap = Node(
        package='kolestel_rover_description',
        executable='local_costmap_node.py',
        name='local_costmap_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'window_size_m': 6.0,
            'resolution': 0.10,
            'publish_rate_hz': 5.0,
            'robot_radius_m': 0.30,
            'inflation_radius_m': 0.60,
            'decay_rate': 5.0,
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'lidar_topic': '/lidar_map',
            'depth_topic': '/camera/depth/points',
            'min_obstacle_height_m': 0.05,
            'max_obstacle_height_m': 2.0,
        }],
    )

    path_follower = Node(
        package='kolestel_rover_description',
        executable='path_follower_node.py',
        name='path_follower_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'lookahead_m': 1.0,
            'linear_speed': 0.4,
            'max_angular_speed': 0.4,
            'arrival_tolerance_m': 0.3,
            'control_rate_hz': 20.0,
            'obstacle_stop_distance_m': 1.0,
            'obstacle_lateral_half_width_m': 0.5,
            'obstacle_min_forward_m': 0.2,
        }],
    )

    depth_cloud_republisher = Node(
        package='kolestel_rover_description',
        executable='depth_cloud_republisher.py',
        name='depth_cloud_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/camera/depth/points',
            'output_topic': '/camera/depth/points_base',
            'target_frame': 'base_footprint',
        }],
    )

    lidar_map_republisher = Node(
        package='kolestel_rover_description',
        executable='lidar_map_republisher.py',
        name='lidar_map_republisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/lidar',
            'output_topic': '/lidar_map',
            'target_frame': 'map',
        }],
    )

    yolo_node = Node(
        package='kolestel_rover_description',
        executable='yolo_node.py',
        name='yolo_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image',
            'debug_topic': '/yolo/debug_image',
            'model_path': 'yolov8n-oiv7.pt',
            'confidence_threshold': 0.15,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'algorithm', default_value='a_star',
            description='dijkstra | a_star | greedy_bfs | jps | theta_star | d_star_lite'
        ),
        gazebo_launch,
        TimerAction(period=9.0, actions=[global_costmap]),
        TimerAction(period=10.0, actions=[path_follower, depth_cloud_republisher, lidar_map_republisher]),
        TimerAction(period=11.0, actions=[local_costmap, OpaqueFunction(function=_planner_nodes)]),
        TimerAction(period=13.0, actions=[yolo_node]),
    ])
