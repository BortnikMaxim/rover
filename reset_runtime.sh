#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

pkill -f uvicorn 2>/dev/null || true
pkill -f robot_status_bridge.py 2>/dev/null || true
pkill -f task_nav2_bridge.py 2>/dev/null || true
pkill -f task_command_bridge.py 2>/dev/null || true
pkill -f "ros2 launch kolestel_rover_description" 2>/dev/null || true
pkill -f "ros_gz_bridge.*parameter_bridge" 2>/dev/null || true
pkill -f "ros_gz_sim.*create" 2>/dev/null || true
pkill -f "odom_to_tf.py" 2>/dev/null || true
pkill -f "pointcloud_to_livox.py" 2>/dev/null || true
pkill -f "warehouse_map_visualizer.py" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "map_server" 2>/dev/null || true
pkill -f "planner_server" 2>/dev/null || true
pkill -f "controller_server" 2>/dev/null || true
pkill -f "velocity_smoother" 2>/dev/null || true
pkill -f "behavior_server" 2>/dev/null || true
pkill -f "bt_navigator" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "ruby.*gz sim" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true

rm -f "$ROOT_DIR/web_app/app/backend/amr.db"
rm -f "$ROOT_DIR/web_app/app/backend/amr.db-shm"
rm -f "$ROOT_DIR/web_app/app/backend/amr.db-wal"

echo "Runtime state cleared."
