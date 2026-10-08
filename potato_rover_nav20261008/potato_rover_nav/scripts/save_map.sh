#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
NAME=${1:-}
if [[ -z "$NAME" || ! "$NAME" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "Usage: $0 <map_name>  (letters, digits, _ and - only)" >&2
  exit 2
fi
docker inspect potato-mapping >/dev/null 2>&1 || {
  echo "ERROR: potato-mapping is not running" >&2
  exit 3
}
mkdir -p "$ROOT/catkin_ws/maps"
docker exec potato-mapping bash -lc   "source /opt/ros/noetic/setup.bash; source /rover_ws/devel/setup.bash; rosrun map_server map_saver -f /rover_ws/maps/$NAME"
docker exec potato-mapping chown "$(id -u):$(id -g)" \
  "/rover_ws/maps/$NAME.pgm" "/rover_ws/maps/$NAME.yaml"
echo "Saved:"
ls -l "$ROOT/catkin_ws/maps/$NAME.pgm" "$ROOT/catkin_ws/maps/$NAME.yaml"
