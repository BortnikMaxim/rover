from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('kolestel_robot')
    model_path = os.path.join(pkg, 'models', 'kolestel_robot')
    world_path = os.path.join(pkg, 'worlds', 'empty.sdf')

    # Make sure Gazebo can find the model folder
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{pkg}:{model_path}:" + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    spawn = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'kolestel_robot',
             '-file', os.path.join(model_path, 'model.sdf')],
        output='screen'
    )

    # Bridge cmd_vel (ROS->GZ), odom and tf (GZ->ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([gz_model_path, gazebo, spawn, bridge])
