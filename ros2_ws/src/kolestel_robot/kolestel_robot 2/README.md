# Kolestel 001 Robot - ROS 2 Jazzy + Gazebo Sim (Harmonic)

This package provides a complete ROS 2 simulation environment for the **Kolestel 001** mobile robot, designed for **Gazebo Sim (Harmonic)** with **ros_gz** integration. It includes a detailed URDF model based on technical drawings, Gazebo Sim plugins for sensors and control, launch files, and configurations for navigation and SLAM.

## Target Platform

| Component | Version |
|-----------|---------|
| **OS** | Ubuntu 24.04 |
| **ROS 2** | Jazzy Jalisco |
| **Simulator** | Gazebo Sim (Harmonic) |
| **Integration** | ros_gz (ros_gz_sim, ros_gz_bridge) |

## Features

- **URDF/Xacro Model**: Modular robot description with configurable parameters
- **Gazebo Sim Plugins**: Native gz-sim plugins for diff-drive, sensors, and physics
- **Skid-Steer Drive**: 4-wheel drive controlled via `gz-sim-diff-drive-system`
- **Sensor Suite**: LiDAR, front/rear cameras, IMU with Gazebo Sim sensors
- **ROS-Gazebo Bridge**: Automatic topic bridging via `ros_gz_bridge`
- **Teleoperation**: Keyboard and joystick control support
- **SLAM & Navigation**: Configurations for slam_toolbox and Nav2

## Robot Specifications (from drawings)

| Parameter | Value |
|-----------|-------|
| Overall dimensions | 1289 × 1000 × 1342 mm |
| Wheelbase | 889 mm |
| Track width | 1000 mm |
| Wheel diameter | 400 mm |
| Mass | ~160 kg |
| Drive type | Skid-steer (4WD) |

## Dependencies

Install the required ROS 2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2
```

## Installation

1. Clone or copy the package to your workspace:

```bash
cd ~/ros2_ws/src
# Copy kolestel_robot folder here
```

2. Build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select kolestel_robot
source install/setup.bash
```

## Quick Start

### Launch Full Simulation

This command starts Gazebo Sim, spawns the robot, launches ros_gz_bridge, and opens RViz:

```bash
ros2 launch kolestel_robot sim.launch.py
```

### Launch with Different World

```bash
ros2 launch kolestel_robot sim.launch.py world:=obstacles.sdf
```

### Launch without RViz

```bash
ros2 launch kolestel_robot sim.launch.py rviz:=false
```

### Teleoperation

In a separate terminal, run keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View Model Only (no simulation)

```bash
ros2 launch kolestel_robot display.launch.py
```

## ROS Topics

After launching the simulation, the following topics are available:

| Topic | Type | Description |
|-------|------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Simulation clock |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (ROS → Gazebo) |
| `/odom` | `nav_msgs/Odometry` | Odometry (Gazebo → ROS) |
| `/tf` | `tf2_msgs/TFMessage` | Transforms (odom → base_footprint) |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint states |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/imu/data` | `sensor_msgs/Imu` | IMU data |
| `/camera/front/image` | `sensor_msgs/Image` | Front camera image |
| `/camera/rear/image` | `sensor_msgs/Image` | Rear camera image |

## Package Structure

```
kolestel_robot/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── controllers.yaml
│   ├── ros_gz_bridge.yaml      # Bridge configuration reference
│   └── teleop_joy.yaml
├── launch/
│   ├── sim.launch.py           # Main simulation launch (Gazebo + RViz + Bridge)
│   ├── display.launch.py       # Model visualization only
│   ├── slam.launch.py          # SLAM mapping
│   └── navigation.launch.py    # Nav2 navigation
├── maps/
│   └── map.yaml
├── meshes/                     # Placeholder for visual meshes
├── params/
│   ├── nav2_params.yaml
│   └── slam_params.yaml
├── rviz/
│   ├── display.rviz
│   ├── sim.rviz               # Simulation visualization
│   └── navigation.rviz
├── scripts/
│   ├── robot_controller.py
│   └── teleop_keyboard.py
├── urdf/
│   ├── robot_gz.urdf.xacro    # Main model for Gazebo Sim
│   ├── robot_properties.xacro  # Physical parameters
│   ├── inertia_macros.xacro
│   ├── wheel.xacro
│   ├── sensors.xacro
│   └── gz_plugins.xacro       # Gazebo Sim plugins
└── worlds/
    ├── empty.sdf              # Empty world
    └── obstacles.sdf          # World with obstacles
```

## Key Files

### URDF Model

The robot model is defined in `urdf/robot_gz.urdf.xacro`, which includes:

- `robot_properties.xacro`: All physical dimensions and masses (edit this to tune)
- `gz_plugins.xacro`: Gazebo Sim plugins (diff-drive, sensors)
- `sensors.xacro`: Sensor link definitions (LiDAR, cameras, IMU)

### Gazebo Sim Plugins

The following Gazebo Sim plugins are configured in `gz_plugins.xacro`:

| Plugin | Purpose |
|--------|---------|
| `gz-sim-diff-drive-system` | Skid-steer drive control |
| `gz-sim-joint-state-publisher-system` | Wheel joint states |
| GPU LiDAR sensor | 360° laser scanner |
| Camera sensors | Front and rear cameras |
| IMU sensor | Inertial measurement unit |

### Launch Arguments

The `sim.launch.py` accepts the following arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Use Gazebo clock |
| `world` | `empty.sdf` | World file to load |
| `rviz` | `true` | Launch RViz |
| `rviz_config` | `rviz/sim.rviz` | RViz config file |

## SLAM and Navigation

### Create a Map

1. Launch simulation:
   ```bash
   ros2 launch kolestel_robot sim.launch.py world:=obstacles.sdf
   ```

2. Launch SLAM:
   ```bash
   ros2 launch kolestel_robot slam.launch.py
   ```

3. Drive the robot with teleop to build the map.

4. Save the map:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/kolestel_robot/maps/my_map
   ```

### Autonomous Navigation

```bash
ros2 launch kolestel_robot navigation.launch.py map:=/path/to/map.yaml
```

## Troubleshooting

### Robot doesn't move

1. Check that Gazebo is running (not paused)
2. Verify `/cmd_vel` topic is being published: `ros2 topic echo /cmd_vel`
3. Check bridge is running: `ros2 topic list | grep cmd_vel`

### No sensor data

1. Verify ros_gz_bridge is running
2. Check Gazebo Sim sensor visualization
3. Ensure correct topic names in bridge configuration

### TF issues

1. Check TF tree: `ros2 run tf2_tools view_frames`
2. Verify odom → base_footprint transform is published

## License

Apache-2.0

---
*Adapted for ROS 2 Jazzy + Gazebo Sim (Harmonic) by Manus AI*
