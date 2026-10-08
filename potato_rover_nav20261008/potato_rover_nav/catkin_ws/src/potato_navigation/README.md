# potato_navigation

Incremental ROS Noetic navigation package for the Potato rover.

## TF ownership

- FAST-LIO planar adapter: `odom -> base_link`
- AMCL: the only publisher of `map -> odom`
- Static sensor transforms remain under `base_link`

Do not add a static `map -> odom` publisher while AMCL is enabled.

## Topics and safety

`move_base` is remapped to `/cmd_vel/nav`. The existing `cmd_vel_mux` selects
teleop before navigation and publishes `/cmd_vel/safe`; the STM32 driver remains
the only serial owner and retains its 250 ms watchdog.

## Start navigation

```bash
cd /home/potato/Potato_Workspace/potato_rover_nav
./scripts/start_nav.sh --rviz --map my_room.yaml
docker logs -f potato-navigation
```

The helper requests `/global_localization` after a five-second grace period unless
RViz `2D Pose Estimate` is used. Wait until `/amcl_converged` is true.

## Record and run waypoints

```bash
./scripts/record_waypoints.sh patrol.yaml
# Select RViz Publish Point, click points, then Ctrl+C.
./scripts/start_waypoints.sh patrol.yaml
./scripts/stop_waypoints.sh
```

Edit `config/patrol.yaml` to set each yaw and wait_time, and set `loop: true` for
continuous patrol. `failure_policy` defaults to skip after one retry.
