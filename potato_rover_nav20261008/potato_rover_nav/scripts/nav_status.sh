#!/usr/bin/env bash
set -euo pipefail
docker ps --format '{{.Names}} {{.Status}}' | grep -E 'potato-(navigation|rover-control|roscore)' || true
if ! docker ps --format '{{.Names}}' | grep -qx potato-roscore; then
  echo "ROS master is not running."
  exit 1
fi
docker exec potato-roscore bash -lc '
  source /opt/ros/noetic/setup.bash
  source /rover_ws/devel/setup.bash
  echo "=== nodes ==="
  rosnode list | grep -E "amcl|waypoint|move_base|laserMapping|lio_planar|cloud_to_scan|map_server|cmd_vel_mux|stm32" | sort || true
  echo "=== AMCL convergence ==="
  timeout 3 rostopic echo -n1 /amcl_converged || true
  echo "=== AMCL covariance ==="
  timeout 3 rostopic echo -n1 /amcl_pose/pose/covariance || true
  echo "=== TF map -> base_link ==="
  timeout 3 rosrun tf tf_echo map base_link | head -8 || true
  echo "=== navigation output ==="
  timeout 3 rostopic echo -n1 /cmd_vel/nav || true
  echo "=== arbitrated safe output ==="
  timeout 3 rostopic echo -n1 /cmd_vel/safe || true
'
