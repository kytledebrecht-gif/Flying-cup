#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
if ! docker ps --format '{{.Names}}' | grep -qx potato-rover-control; then
  echo "Control core is not running; starting it now..."
  "$ROOT/scripts/start_control.sh"
  sleep 1
fi
exec docker exec -it potato-rover-control bash -lc   'source /opt/ros/noetic/setup.bash && source /rover_ws/devel/setup.bash && rosrun potato_base_driver wasd_teleop.py'
