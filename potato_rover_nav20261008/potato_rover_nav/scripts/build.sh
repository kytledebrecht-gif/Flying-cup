#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/potato/Potato_Workspace/potato_rover_nav
docker build --network host -t potato-rover-noetic:latest "$ROOT"
docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$ROOT/catkin_ws:/rover_ws" -w /rover_ws potato-rover-noetic:latest bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make -DROS_EDITION=ROS1 -j4 -l4'
