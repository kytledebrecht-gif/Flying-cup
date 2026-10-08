# Potato Rover Navigation

ROS 1 Noetic Docker runtime. This project does not call PX4 or MAVLink.

## Build

From `/home/potato/Potato_Workspace/potato_rover_nav`:

```bash
./scripts/build.sh
```

The build uses ROS 1 explicitly (`-DROS_EDITION=ROS1`) so that `livox_ros_driver2`
generates the ROS 1 `CustomMsg` used by FAST-LIO.

## MID-360 + FAST-LIO

The wired interface is `enP8p1s0` at `192.168.1.5/24` with no gateway.
The MID-360 is `192.168.1.192`.

```bash
./scripts/start_lio.sh          # headless
./scripts/start_lio.sh --rviz   # local desktop only
docker logs -f potato-lio
./scripts/stop_lio.sh
```

Main topics:

- `/livox/lidar`: `livox_ros_driver2/CustomMsg`, nominal 10 Hz
- `/livox/imu`: `sensor_msgs/Imu`, nominal 200 Hz
- `/Odometry`: `nav_msgs/Odometry`, nominal 10 Hz
- `/cloud_registered`: registered world-frame cloud
- `/cloud_registered_body`: registered body-frame cloud

Configuration:

- Driver/IP: `catkin_ws/src/potato_lio_bringup/config/MID360_config.json`
- FAST-LIO: `catkin_ws/src/potato_lio_bringup/config/fast_lio_mid360.yaml`
- Launch: `catkin_ws/src/potato_lio_bringup/launch/lio.launch`

Quick health check:

```bash
docker exec potato-lio bash -lc 'source /opt/ros/noetic/setup.bash; source /rover_ws/devel/setup.bash; rostopic hz /livox/lidar'
docker exec potato-lio bash -lc 'source /opt/ros/noetic/setup.bash; source /rover_ws/devel/setup.bash; rostopic hz /livox/imu'
docker exec potato-lio bash -lc 'source /opt/ros/noetic/setup.bash; source /rover_ws/devel/setup.bash; rostopic echo -n1 /Odometry'
```

## 2D mapping

The mapping stack uses FAST-LIO for motion estimation, publishes the standard
map -> odom -> base_link TF tree, projects a height-filtered 720-beam /scan,
and builds /map with Gmapping. It does not use rf2o or an accumulated 3D cloud.

    ./scripts/start_mapping.sh          # headless
    ./scripts/start_mapping.sh --rviz   # Jetson local desktop
    docker logs -f potato-mapping
    ./scripts/save_map.sh my_room
    ./scripts/stop_mapping.sh

Important tuning files:

- Scan heights/ranges and sensor extrinsics: catkin_ws/src/potato_mapping/launch/mapping.launch
- Gmapping: catkin_ws/src/potato_mapping/config/gmapping.yaml
- RViz: catkin_ws/src/potato_mapping/rviz/mapping.rviz
- Saved maps: catkin_ws/maps/

## Chassis control

The active path is WASD/nav topic -> command mux -> one exclusive STM32 serial owner.
Do not run the legacy direct-serial script in parallel.

    ./scripts/start_wasd.sh       # automatically starts the control core
    ./scripts/stop_control.sh

Keys: W/S forward/reverse, A/D rotate, SPACE immediate stop, Q stop and quit.
Use [ and ] to change speed from 10% to 100%. The default is 50%, and the
STM32 protocol limit remains +/-200 units. A 250 ms driver watchdog, immediate
zero handling, and a 400 units/s non-zero acceleration ramp are active.

Mapping and WASD are independent and may run in two terminals. Both use the
dedicated potato-roscore container; stopping mapping does not stop chassis
control, and stopping control does not stop mapping.

STM32 stable device: /dev/potato_stm32. Topics: /cmd_vel/teleop,
/cmd_vel/nav, /cmd_vel/safe.
