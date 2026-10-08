#!/usr/bin/env bash
set -euo pipefail
if ! docker ps --format '{{.Names}}' | grep -qx potato-navigation; then
  echo "potato-navigation is not running."
  exit 0
fi
docker exec potato-navigation bash -lc   'source /opt/ros/noetic/setup.bash && source /rover_ws/devel/setup.bash && rosnode kill /waypoint_navigator'   >/dev/null 2>&1 || true
echo "Waypoint navigator stopped; active goal was cancelled and mux/watchdog remain active."
