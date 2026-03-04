# Navigation Stack Instruction

## 0. 环境准备

```bash
cd <repo_root>/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果还没编译:
```bash
colcon build --symlink-install --packages-select robot_navigation
source install/setup.bash
```

## 1. 推荐流程（最稳，直接可用）

### 1.1 建图阶段（会启动 LiDAR + static TF + Cartographer + 串口桥 + EKF）
```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

可视化版本:
```bash
ros2 run robot_navigation nav_assistant mapping-stack --with-rviz true
```

### 1.2 手动遥控
```bash
ros2 run robot_navigation nav_assistant teleop
```

### 1.3 保存 + 导出地图（推荐导出分辨率 0.03）
```bash
ros2 run robot_navigation nav_assistant save-map --map-name testmap1
ros2 run robot_navigation nav_assistant export-map --map-name testmap1 --resolution 0.03
```

### 1.4 定位 + Nav2 阶段（会启动 LiDAR + static TF + 串口桥 + EKF + Cartographer localization + map_server + Nav2）
```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1
```

带 RViz:
```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1 --with-nav2-rviz true
```

### 1.5 命令行发导航目标
```bash
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

多点:
```bash
ros2 run robot_navigation nav_assistant waypoints \
  --pose 1.0,0.0,0.0 \
  --pose 1.5,0.5,0.0 \
  --pose 0.5,1.0,0.0
```

## 2. 按组件分开启动（调试模式）

以下用于单独排查节点问题，不是日常首选。

### 2.1 SICK LiDAR 驱动
```bash
ros2 run sick_scan_xd sick_generic_caller \
  $(ros2 pkg prefix sick_scan_xd)/share/sick_scan_xd/launch/sick_picoscan.launch \
  hostname:=192.168.8.150 \
  udp_receiver_ip:=192.168.8.249 \
  publish_frame_id:=lidar_link \
  publish_imu_frame_id:=lidar_link \
  tf_publish_rate:=0.0 \
  imu_udp_port:=7503 \
  scandataformat:=2 \
  send_sopas_start_stop_cmd:=0 \
  host_set_FREchoFilter:=0 \
  host_set_LFPangleRangeFilter:=0 \
  host_set_LFPintervalFilter:=0 \
  custom_pointclouds:=cloud_all_fields_fullframe \
  cloud_all_fields_fullframe:='coordinateNotation=3 updateMethod=0 fields=x,y,z,i,range,azimuth,elevation,t,ts,lidar_sec,lidar_nsec,ring,layer,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/cloud_all_fields_fullframe frameid=lidar_link publish=1' \
  publish_laserscan_fullframe_topic:=/scan_fullframe \
  imu_topic:=/sick_scansegment_xd/imu
```

### 2.2 static TF (`base_link -> lidar_link`)
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.254 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id lidar_link
```

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

### 2.5 串口桥（STM32 -> /odom_raw）
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

### 2.6 EKF（/odom_raw + IMU -> /odom）
```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file <repo_root>/workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml \
  -r odometry/filtered:=/odom
```

### 2.7 map_server + lifecycle
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

## 3. 常用检查命令

```bash
ros2 run robot_navigation nav_assistant quick-check
ros2 topic hz /odom
ros2 topic hz /cloud_all_fields_fullframe
ros2 action list | grep navigate_to_pose
```

## 4. 你当前代码里的关键默认值（便于对照）

- `odom_topic` 默认: `/odom_raw`
- `use_ekf` 默认: `true`
- `fallback_odom` 默认: `false`
- LiDAR frame 默认: `lidar_link`
- static TF 默认: `base_link -> lidar_link = (0.254, 0, 0, 0, 0, 0)`
- `with_nav2_rviz` 默认: `false`
