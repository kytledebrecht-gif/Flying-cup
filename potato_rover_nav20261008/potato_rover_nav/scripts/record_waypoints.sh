#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
FILE_ARG=${1:-waypoints.yaml}

if ! docker ps --format '{{.Names}}' | grep -qx potato-navigation; then
  echo "ERROR: potato-navigation is not running. Start navigation with RViz first." >&2
  exit 3
fi
if [[ "$FILE_ARG" = /* ]]; then
  HOST_FILE=$FILE_ARG
else
  HOST_FILE=$ROOT/catkin_ws/src/potato_navigation/config/$FILE_ARG
fi
case "$HOST_FILE" in
  "$ROOT"/catkin_ws/*) CONTAINER_FILE=$(printf '%s' "$HOST_FILE" | sed "s#^$ROOT/catkin_ws#/rover_ws#") ;;
  *) echo "ERROR: waypoint file must be inside $ROOT/catkin_ws" >&2; exit 4 ;;
esac
mkdir -p "$(dirname "$HOST_FILE")"

echo "Recording RViz /clicked_point into: $HOST_FILE"
echo "Select Publish Point in RViz, click free cells, then press Ctrl+C here."
docker exec -it --user "$(id -u):$(id -g)" -e HOME=/tmp potato-navigation bash -lc   "source /opt/ros/noetic/setup.bash && source /rover_ws/devel/setup.bash && exec rosrun potato_navigation record_waypoint.py _output_file:=$CONTAINER_FILE"
