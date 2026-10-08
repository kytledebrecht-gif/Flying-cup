#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
if docker ps --format '{{.Names}}' | grep -qx potato-roscore; then
  exit 0
fi
docker rm -f potato-roscore >/dev/null 2>&1 || true
docker run --rm -d --name potato-roscore --network host   -v "$ROOT/catkin_ws:/rover_ws" -w /rover_ws   potato-rover-noetic:latest bash -lc   'source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec roscore' >/dev/null
for _ in {1..30}; do
  if docker exec potato-roscore bash -lc     'source /opt/ros/noetic/setup.bash; rostopic list >/dev/null 2>&1'; then
    echo "ROS master ready: potato-roscore"
    exit 0
  fi
  sleep 0.1
done
echo "ERROR: ROS master did not become ready" >&2
docker logs potato-roscore >&2 || true
exit 4
