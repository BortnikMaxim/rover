#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "ROS workspace is not built: $ROS_WS/install/setup.bash" >&2
  echo "Run: $ROOT_DIR/build_ros2.sh" >&2
  exit 1
fi

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

export AMR_WAREHOUSE_MAP_PATH="$ROOT_DIR/shared/warehouse_map.yaml"
ros2 launch kolestel_rover_description sim_nav2.launch.py
