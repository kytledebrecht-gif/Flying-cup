#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
MAP_ARG=my_room.yaml
RVIZ=false
START_CONTROL=true
USE_AMCL=true

usage() {
  echo "Usage: $0 [--rviz] [--map NAME.yaml|ABSOLUTE_PATH] [--no-control] [--no-amcl]"
  echo "  AMCL is enabled by default and is the sole map->odom broadcaster."
  echo "  --no-amcl requires another localization source to publish map->odom."
}

while (( $# )); do
  case "$1" in
    --rviz) RVIZ=true; shift ;;
    --no-control) START_CONTROL=false; shift ;;
    --no-amcl) USE_AMCL=false; shift ;;
    --map) [[ $# -ge 2 ]] || { usage; exit 2; }; MAP_ARG=$2; shift 2 ;;
    --map-x|--map-y|--map-yaw)
      echo "ERROR: static map->odom alignment was removed. AMCL now owns map->odom." >&2
      exit 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ "$MAP_ARG" = /* ]]; then
  HOST_MAP=$MAP_ARG
else
  HOST_MAP=$ROOT/catkin_ws/maps/$MAP_ARG
fi
[[ -f "$HOST_MAP" ]] || { echo "ERROR: map YAML not found: $HOST_MAP" >&2; exit 3; }
case "$HOST_MAP" in
  "$ROOT"/catkin_ws/*) CONTAINER_MAP=$(printf '%s' "$HOST_MAP" | sed "s#^$ROOT/catkin_ws#/rover_ws#") ;;
  *) echo "ERROR: map must be inside $ROOT/catkin_ws so Docker can read it" >&2; exit 4 ;;
esac

for conflict in potato-mapping potato-lio; do
  if docker ps --format '{{.Names}}' | grep -qx "$conflict"; then
    echo "ERROR: $conflict is running. Stop it before navigation to avoid duplicate LiDAR/TF publishers." >&2
    exit 5
  fi
done

"$ROOT/scripts/ensure_roscore.sh"
if [[ "$START_CONTROL" == true ]] && ! docker ps --format '{{.Names}}' | grep -qx potato-rover-control; then
  "$ROOT/scripts/start_control.sh"
  sleep 1
fi

docker rm -f potato-navigation >/dev/null 2>&1 || true
args=(--rm -d --name potato-navigation --network host --ipc host
      -v "$ROOT/catkin_ws:/rover_ws" -w /rover_ws)
if [[ "$RVIZ" == true ]]; then
  LOCAL_DISPLAY=:0
  LOCAL_XAUTH=/run/user/$(id -u)/gdm/Xauthority
  [[ -S /tmp/.X11-unix/X0 ]] || { echo 'ERROR: Jetson local X11 display :0 is unavailable' >&2; exit 6; }
  [[ -r "$LOCAL_XAUTH" ]] || { echo "ERROR: Jetson local Xauthority unavailable: $LOCAL_XAUTH" >&2; exit 7; }

  # The HDMI dummy plug advertises 4K at only 17 Hz.  Rendering RViz at that
  # resolution wastes several CPU cores (the ROS1 container uses Mesa), and it
  # also makes RustDesk encode four times as many pixels.  Keep the local Jetson
  # desktop at 1080p whenever the local RViz view is requested.
  if command -v xrandr >/dev/null 2>&1; then
    RVIZ_OUTPUT=$(DISPLAY="$LOCAL_DISPLAY" XAUTHORITY="$LOCAL_XAUTH" xrandr --query |
      awk '/ connected primary/{print $1; primary=1; exit} / connected/{if (!fallback) fallback=$1} END{if (!primary && fallback) print fallback}')
    if [[ -n "$RVIZ_OUTPUT" ]]; then
      DISPLAY="$LOCAL_DISPLAY" XAUTHORITY="$LOCAL_XAUTH" \
        xrandr --output "$RVIZ_OUTPUT" --mode 1920x1080 --rate 60 >/dev/null 2>&1 || \
        echo "WARNING: could not switch $RVIZ_OUTPUT to 1920x1080@60; continuing with the current mode." >&2
    fi
  fi

  args+=(-e DISPLAY="$LOCAL_DISPLAY" -e XAUTHORITY=/root/.Xauthority
         -v /tmp/.X11-unix:/tmp/.X11-unix:rw
         -v "$LOCAL_XAUTH:/root/.Xauthority:ro")
fi

docker run "${args[@]}" potato-rover-noetic:latest bash -lc   "source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec roslaunch potato_navigation nav.launch map_file:=$CONTAINER_MAP rviz:=$RVIZ use_amcl:=$USE_AMCL auto_global_localization:=$USE_AMCL"
echo "Navigation started (map=$HOST_MAP rviz=$RVIZ control=$START_CONTROL amcl=$USE_AMCL)."
echo "No goal has been sent. Wait for /amcl_converged=true before navigation."
echo "Status: $ROOT/scripts/nav_status.sh"
echo "Logs: docker logs -f potato-navigation"
