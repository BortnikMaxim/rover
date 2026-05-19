#!/usr/bin/env bash
# Run the autonomous-navigation mode (grid_planner + path_follower).
# Optional first arg: algorithm name (default a_star).
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROS_WS="$ROOT_DIR/ros2_ws"
ACTIVE_WORLD="$ROS_WS/install/kolestel_rover_description/share/kolestel_rover_description/worlds/user_saved_layout.sdf"

ALGO="${1:-a_star}"

if [ ! -f "$ROS_WS/install/setup.bash" ]; then
  echo "ROS workspace is not built: $ROS_WS/install/setup.bash" >&2
  echo "Run: $ROOT_DIR/build_ros2.sh" >&2
  exit 1
fi

echo "[run_gazebo_autonomous] algorithm: $ALGO"
echo "[run_gazebo_autonomous] world: $ACTIVE_WORLD"

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

export AMR_WAREHOUSE_MAP_PATH="$ROOT_DIR/shared/warehouse_map.yaml"
export AMR_GAZEBO_WORLD_PATH="$ACTIVE_WORLD"
ros2 launch kolestel_rover_description sim_autonomous.launch.py "algorithm:=$ALGO"
