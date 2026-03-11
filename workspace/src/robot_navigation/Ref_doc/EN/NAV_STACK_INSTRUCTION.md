# Navigation Stack Instruction

## 0. Environment setup

```bash
cd <repo_root>/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If not built yet:
```bash
colcon build --symlink-install --packages-select robot_navigation
source install/setup.bash
```

## 1. Recommended workflow (primary path)

### 1.1 Mapping phase
Starts: LiDAR, static TF, Cartographer mapping, serial bridge, EKF.

```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

For D0 parameter validation (PointCloud2 path), use the quality config directly:
```bash
ros2 run robot_navigation nav_assistant mapping-stack \
  --cartographer-config-basename pico_2d_mapping_quality.lua
```

With RViz:
```bash
ros2 run robot_navigation nav_assistant mapping-stack --with-rviz true
```

### 1.2 Manual driving
```bash
ros2 run robot_navigation nav_assistant teleop
```

### 1.3 Save and export map
```bash
ros2 run robot_navigation nav_assistant save-map --map-name testmap1
ros2 run robot_navigation nav_assistant export-map --map-name testmap1
```

### 1.4 Localization + Nav2 phase
Starts: LiDAR, static TF, serial bridge, EKF, Cartographer localization, map_server, Nav2.

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1
```

With RViz:
```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1 --with-nav2-rviz true
```

### 1.5 Send goals from CLI
```bash
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

Waypoints:
```bash
ros2 run robot_navigation nav_assistant waypoints \
  --pose 1.0,0.0,0.0 \
  --pose 1.5,0.5,0.0 \
  --pose 0.5,1.0,0.0
```

## 2. Component-by-component startup (debug mode)

Use these only when you need isolated debugging.

### 2.1 SICK LiDAR driver
```bash
ros2 run sick_scan_xd sick_generic_caller \
  $(ros2 pkg prefix sick_scan_xd)/share/sick_scan_xd/launch/sick_picoscan.launch \
  hostname:=192.168.8.150 \
  udp_receiver_ip:=192.168.8.249 \
  publish_frame_id:=lidar_link \
  publish_imu_frame_id:=imu_link \
  tf_publish_rate:=0.0 \
  imu_udp_port:=7503 \
  scandataformat:=2 \
  send_sopas_start_stop_cmd:=0 \
  host_FREchoFilter:=2 \
  host_set_FREchoFilter:=1 \
  host_set_LFPangleRangeFilter:=0 \
  host_set_LFPintervalFilter:=0 \
  publish_laserscan_segment_topic:=/scan_segment \
  custom_pointclouds:=cloud_all_fields_fullframe \
  cloud_all_fields_fullframe:='coordinateNotation=3 updateMethod=0 fields=x,y,z,i,range,azimuth,elevation,t,ts,lidar_sec,lidar_nsec,ring,layer,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/cloud_all_fields_fullframe frameid=lidar_link publish=1' \
  imu_topic:=/sick_scansegment_xd/imu
```

### 2.2 Static TF (`base_link -> lidar_link`, `lidar_link -> lidar_link_1`, `lidar_link -> imu_link`)
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.2413 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id lidar_link
```

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id lidar_link --child-frame-id lidar_link_1
```

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0124 --y 0.0185 --z -0.0484 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id lidar_link --child-frame-id imu_link
```

Notes:

- Current project preset follows the user-validated mount assumption: `lidar_link` is modeled at the bracket center and used as the project's optical-origin proxy.
- `0.2413 m` comes from `20 in / 2 - 1 in / 2 = 9.5 in = 0.2413 m`, i.e. half the 20 in base minus half the standard 1.00 in 80/20 front bar width.
- Segmented LaserScan is now explicitly forced to `last echo only`: `host_FREchoFilter=2`.
- The identity TF `lidar_link -> lidar_link_1` is added for the segmented LaserScan frame suffix used by `sick_scan_xd`.
- `0.0124, 0.0185, -0.0484 m` comes from the SICK operating instructions as IMU position relative to the optical origin.

### 2.3 Cartographer mapping
```bash
ros2 launch robot_navigation cartographer_mapping.launch.py
```

### 2.4 Cartographer localization
```bash
ros2 launch robot_navigation cartographer_localization.launch.py \
  load_state_filename:=<repo_root>/Maps/testmap1.pbstream \
  configuration_basename:=pico_2d_localization.lua
```

### 2.5 Serial bridge (`STM32 -> /odom_raw`)
```bash
ros2 run robot_navigation nav2_serial_bridge --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p cmd_topics:='["/cmd_vel","/cmd_vel_nav","/cmd_vel_smoothed"]' \
  -p odom_topic:=/odom_raw \
  -p frame_id:=odom \
  -p child_frame_id:=base_link \
  -p publish_tf:=false \
  -p fallback_odom:=false
```

### 2.6 EKF (`/odom_raw + IMU -> /odom`)
```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file <repo_root>/workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml \
  -r odometry/filtered:=/odom
```

### 2.7 map_server + lifecycle manager
```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=<repo_root>/Maps/testmap1.yaml
```

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true \
  -p node_names:="[map_server]"
```

### 2.8 Nav2 bringup
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=<repo_root>/workspace/src/robot_navigation/config/nav2_params_cartographer.yaml
```

## 3. Common health checks

```bash
ros2 run robot_navigation nav_assistant quick-check
ros2 node info /cartographer_node | grep points2
ros2 topic hz /odom
ros2 topic hz /cloud_all_fields_fullframe
ros2 action list | grep navigate_to_pose
```

## 4. Current key defaults (for quick reference)

- `odom_topic`: `/odom_raw`
- `use_ekf`: `true`
- `fallback_odom`: `false`
- LiDAR frame: `lidar_link`
- Static TF default: `base_link -> lidar_link = (0.2413, 0, 0, 0, 0, 0)`
- `with_nav2_rviz`: `false`
