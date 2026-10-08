#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
RVIZ=false
[[ "${1:-}" == "--rviz" ]] && RVIZ=true

ip -4 addr show enP8p1s0 | grep -q '192.168.1.5/24' || {
  echo 'ERROR: enP8p1s0 lacks 192.168.1.5/24' >&2
  exit 2
}
ping -I 192.168.1.5 -c 1 -W 1 192.168.1.192 >/dev/null || {
  echo 'ERROR: MID-360 192.168.1.192 unreachable' >&2
  exit 3
}

mkdir -p "$ROOT/catkin_ws/maps"
"$ROOT/scripts/ensure_roscore.sh"
docker rm -f potato-lio potato-mapping >/dev/null 2>&1 || true
args=(--rm -d --name potato-mapping --network host --ipc host
      -v "$ROOT/catkin_ws:/rover_ws" -w /rover_ws)
if [[ "$RVIZ" == true ]]; then
  LOCAL_DISPLAY=:0
  LOCAL_XAUTH=/run/user/$(id -u)/gdm/Xauthority
  [[ -S /tmp/.X11-unix/X0 ]] || { echo 'ERROR: Jetson local X11 display :0 is unavailable' >&2; exit 6; }
  [[ -r "$LOCAL_XAUTH" ]] || { echo "ERROR: Jetson local Xauthority unavailable: $LOCAL_XAUTH" >&2; exit 7; }
  args+=(-e DISPLAY="$LOCAL_DISPLAY" -e XAUTHORITY=/root/.Xauthority
         -v /tmp/.X11-unix:/tmp/.X11-unix:rw
         -v "$LOCAL_XAUTH:/root/.Xauthority:ro")
fi

docker run "${args[@]}" potato-rover-noetic:latest bash -lc   "source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch potato_mapping mapping.launch rviz:=$RVIZ"
echo "Mapping started (rviz=$RVIZ)."
echo "Logs: docker logs -f potato-mapping"
echo "Save: $ROOT/scripts/save_map.sh <map_name>"
