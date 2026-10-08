#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
FILE_ARG=waypoints.yaml
WAIT_AMCL=true
NAVIGATION_MODE=smooth

usage() { echo "Usage: $0 [FILE.yaml] [--smooth|--precise] [--no-wait-amcl]"; }
while (( $# )); do
  case "$1" in
    --no-wait-amcl) WAIT_AMCL=false; shift ;;
    --smooth) NAVIGATION_MODE=smooth; shift ;;
    --precise) NAVIGATION_MODE=precise; shift ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; usage; exit 2 ;;
    *) FILE_ARG=$1; shift ;;
  esac
done
if ! docker ps --format '{{.Names}}' | grep -qx potato-navigation; then
  echo "ERROR: potato-navigation is not running." >&2
  exit 3
fi
if [[ "$FILE_ARG" = /* ]]; then
  HOST_FILE=$FILE_ARG
else
  HOST_FILE=$ROOT/catkin_ws/src/potato_navigation/config/$FILE_ARG
fi
[[ -f "$HOST_FILE" ]] || { echo "ERROR: waypoint YAML not found: $HOST_FILE" >&2; exit 4; }
case "$HOST_FILE" in
  "$ROOT"/catkin_ws/*) CONTAINER_FILE=$(printf '%s' "$HOST_FILE" | sed "s#^$ROOT/catkin_ws#/rover_ws#") ;;
  *) echo "ERROR: waypoint file must be inside $ROOT/catkin_ws" >&2; exit 5 ;;
esac

echo "Starting waypoint navigation: $HOST_FILE"
echo "mode=$NAVIGATION_MODE wait_for_amcl=$WAIT_AMCL; Ctrl+C cancels active move_base goals."
docker exec -it --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_LOG_DIR=/tmp/potato-ros-log-$(id -u) potato-navigation bash -lc   "mkdir -p /tmp/potato-ros-log-$(id -u) && source /opt/ros/noetic/setup.bash && source /rover_ws/devel/setup.bash && exec roslaunch potato_navigation waypoint_nav.launch waypoints_file:=$CONTAINER_FILE wait_for_amcl:=$WAIT_AMCL navigation_mode:=$NAVIGATION_MODE"
