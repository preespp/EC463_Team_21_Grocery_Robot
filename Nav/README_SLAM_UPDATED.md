# ROS2 Bringup for SICK PicoScan150 + Cartographer + Nav2

This README is the **actual runbook** for this repo on Jetson.

- ROS 2: Humble
- LiDAR driver: `sick_scan_xd` (`sick_picoscan.launch.py`)
- SLAM/Localization: Cartographer (`Nav/carto_cfg/*.launch.py`)
- Base controller: STM32 UART2 bridge (`tools/nav2_serial_bridge.py`)

## Frame and Topic Convention (current project)

- TF chain: `map -> odom -> base_link`
- Robot base frame used by Nav2: `base_link`
- Cartographer input point cloud: `/cloud_all_fields_fullframe`
- Cartographer IMU input: `/sick_scansegment_xd/imu`
- Odometry topic consumed by Nav2: `/odom` (from serial bridge)
- Command topic: `/cmd_vel`

## Why these SICK parameters

Based on `sick_scan_xd` docs/examples, the relevant launch arguments for PicoScan include:

- `hostname` (LiDAR IP)
- `udp_receiver_ip` (Jetson IP)
- `publish_frame_id` (frame id for published data)
- `cloud_topic`, `publish_laserscan_fullframe_topic`, `imu_topic`

In this project we pin them to the topics expected by Cartographer/Nav2.

## Terminal 0: Common environment

In each terminal:

```bash
source /opt/ros/humble/setup.bash
```

If you have a workspace overlay, source it as usual.

## Phase A: Mapping run (manual keyboard control + save map)

### 1. Start SICK PicoScan driver

```bash
ros2 launch sick_scan_xd sick_picoscan.launch.py \
  hostname:=192.168.8.150 \
  udp_receiver_ip:=192.168.8.249 \
  publish_frame_id:=base_link \
  publish_imu_frame_id:=base_link \
  custom_pointclouds:='cloud_all_fields_fullframe' \
  cloud_all_fields_fullframe:='coordinateNotation=3 updateMethod=0 fields=x,y,z,i,range,azimuth,elevation,t,ts,lidar_sec,lidar_nsec,ring,layer,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/cloud_all_fields_fullframe frameid=base_link publish=1' \
  publish_laserscan_fullframe_topic:=/scan_fullframe \
  imu_topic:=/sick_scansegment_xd/imu
```

Note: `sick_picoscan.launch` defaults custom point clouds to `frameid=world`.  
The explicit `cloud_all_fields_fullframe:=... frameid=base_link ...` override is required to avoid Cartographer TF warnings about missing `world`.

### 2. Start Cartographer mapping

```bash
ros2 launch /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Nav/carto_cfg/my_carto.launch.py
```

### 3. Start STM32 serial bridge (`/cmd_vel` -> UART2)

```bash
python3 /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/nav2_serial_bridge.py \
  --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p cmd_topics:='["/cmd_vel","/cmd_vel_nav","/cmd_vel_smoothed"]' \
  -p telemetry_enabled:=true \
  -p left_switch:=1 \
  -p right_switch:=1 \
  -p frame_id:=odom \
  -p child_frame_id:=base_link \
  -p publish_tf:=false
```

Notes:

- `publish_tf:=false` avoids TF conflict with Cartographer (Cartographer already publishes `odom -> base_link`).
- If your USB serial path is different, change `serial_port`.
- Use `telemetry_enabled:=true` when STM32 Serialplot telemetry is on UART2 (shared command + telemetry link).
- For command-only debugging, you can temporarily set `telemetry_enabled:=false`.
- For keyboard control, keep both switches in UP (`left_switch:=1`, `right_switch:=1`), otherwise chassis logic may stay disabled.

### 3.5 Collision detection

Open another terminal and change directory to workspace

```bash
source install/setup.bash
ros2 launch robot_perception ultrasonic_launch.py
```

```bash
python3 /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/teleop_cmd_vel_collision.py \
  --topic /cmd_vel \
  --linear 0.6 \
  --angular 1.2
```

### 4. Manual keyboard teleop (publishes `/cmd_vel`)

```bash
python3 /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/teleop_cmd_vel.py \
  --topic /cmd_vel \
  --linear 0.6 \
  --angular 1.2
```

Keys: `W/S/A/D/Q/E`, `Space` stop.

### 5. (Optional) RViz

```bash
ros2 run rviz2 rviz2
```

### 6. Save Cartographer map to `Maps/`

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream', include_unfinished_submaps: true}"
```

### 7. Export pbstream to yaml/pgm (required for Nav2 static map)

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream \
  -map_filestem /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1 \
  -resolution 0.05
```

This generates:

- `/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml`
- `/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pgm`

## Phase B: Localization + Nav2 navigation

Keep these running:

- SICK driver
- Serial bridge

Then start:

### 1. Cartographer localization (frozen pbstream)

```bash
ros2 launch /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Nav/carto_cfg/my_carto_localization.launch.py \
  load_state_filename:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream \
  publish_occupancy_grid:=false
```

### 2. Start map server (load saved map YAML)

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=false \
  -p yaml_filename:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml
```

### 3. Activate map server lifecycle

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=false \
  -p autostart:=true \
  -p node_names:='["map_server"]'
```

### 4. Nav2 stack

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Nav/nav2_params_cartographer.yaml
```

Notes:

- This parameter file disables AMCL TF broadcast so Cartographer remains the only `map -> odom` publisher.
- `navigation_launch.py` does not launch `map_server`, so start `map_server` first (steps 2 and 3).

### 5. Nav2 RViz

```bash
ros2 launch nav2_bringup rviz_launch.py
```

Now set a Nav2 goal in RViz and verify `/cmd_vel` is being published.

### After this step, the content has not been verified yet.
### 6. Send Nav2 goal or waypoints

Check Nav2 actions are available:

```bash
ros2 action list | grep -E 'navigate_to_pose|follow_waypoints'
ros2 action info /navigate_to_pose
ros2 action info /follow_waypoints
```

Send a single goal from CLI (`map` frame):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

Send multiple waypoints from CLI (`map` frame):

```bash
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 0.5, y: 1.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}
  ]}"
```

RViz method:

- Use the Nav2 Goal tool (`2D Goal Pose`) for a single destination.
- Use the Nav2 Waypoint tool/plugin to queue multiple goals.

While a goal is active, verify command output:

```bash
ros2 topic hz /cmd_vel
```

## Quick checks

### Topic checks

```bash
ros2 topic list
ros2 topic hz /cloud_all_fields_fullframe
ros2 topic info /map
ros2 topic echo /map_metadata --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel_smoothed --once
```

### TF checks

```bash
ros2 run tf2_tools view_frames
```

Expect a connected tree including `map`, `odom`, `base_link`.

## Common pitfalls

1. Wrong localization launch filename case on Linux.
Use `my_carto_localization.launch.py` (lowercase `l`), not `my_carto_Localization.launch.py`.

2. No motion with keyboard teleop.
Confirm serial bridge is running and STM32 UART port/baud are correct.

3. Nav2 costmap no obstacles.
Confirm point cloud topic exists and matches `/cloud_all_fields_fullframe`.

4. No saved map appears in RViz / `/map` has no publisher.
Start `map_server` and activate lifecycle manager (`node_names:='["map_server"]'`) before launching `navigation_launch.py`.

5. TF conflict warnings.
Keep serial bridge `publish_tf:=false` while Cartographer is publishing `odom -> base_link`.

6. Two maps flashing / map jumps / TF disappearing.
Do not let multiple localization/map publishers run on the same topics and transforms:
- In localization phase keep Cartographer `publish_occupancy_grid:=false` to avoid `/map` conflict with Nav2 map_server.
- Keep AMCL TF broadcast disabled in `Nav/nav2_params_cartographer.yaml` when Cartographer localization is active.
