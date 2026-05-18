#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$ROOT_DIR/ros2_ws"
rm -rf build install log
colcon build --packages-select kolestel_rover_description
