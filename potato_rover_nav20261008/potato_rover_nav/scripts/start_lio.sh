#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
RVIZ=false
[[ "${1:-}" == "--rviz" ]] && RVIZ=true
ip -4 addr show enP8p1s0 | grep -q '192.168.1.5/24' || { echo 'ERROR: enP8p1s0 lacks 192.168.1.5/24' >&2; exit 2; }
ping -I 192.168.1.5 -c 1 -W 1 192.168.1.192 >/dev/null || { echo 'ERROR: MID-360 192.168.1.192 unreachable' >&2; exit 3; }
docker rm -f potato-lio >/dev/null 2>&1 || true
args=(--rm -d --name potato-lio --network host --ipc host -v "$ROOT/catkin_ws:/rover_ws" -w /rover_ws)
if [[ "$RVIZ" == true ]]; then
  export DISPLAY=${DISPLAY:-:0}; XAUTH=${XAUTHORITY:-$HOME/.Xauthority}
  args+=(-e DISPLAY="$DISPLAY" -e XAUTHORITY=/root/.Xauthority -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v "$XAUTH:/root/.Xauthority:ro")
fi
docker run "${args[@]}" potato-rover-noetic:latest bash -lc "source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch potato_lio_bringup lio.launch rviz:=$RVIZ"
echo "LIO started (rviz=$RVIZ). Logs: docker logs -f potato-lio"
