#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
NODE=/dev/potato_stm32
[[ -e "$NODE" ]] || { echo "ERROR: $NODE not found" >&2; exit 2; }
REAL_NODE=$(readlink -f "$NODE")
"$ROOT/scripts/ensure_roscore.sh"
docker rm -f potato-rover-control >/dev/null 2>&1 || true
docker run -d --rm --name potato-rover-control --network host --ipc host   --device="$REAL_NODE:/dev/potato_stm32"   -v "$ROOT/catkin_ws:/rover_ws" -w /rover_ws   potato-rover-noetic:latest bash -lc   'source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch potato_base_driver control.launch'
echo 'Control core started. Logs: docker logs -f potato-rover-control'
