# Navigation Stack Instruction

## 0. 环境准备

本项目过去文档里统一使用的 EC463 Linux 路径是：

- Linux repo root: `/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot`
- Linux workspace: `/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace`
- Linux Maps: `/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps`

后文出现的 `<repo_root>` 都可以按上面的实际路径替换。

通用写法：

```bash
cd <repo_root>/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果你就是在这套 EC463 Linux 环境里直接运行：

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
```

如果还没编译：

```bash
colcon build --symlink-install --packages-select robot_navigation
source install/setup.bash
```

## 1. 推荐流程

### 1.1 建图阶段

默认主路径已经是 segmented `LaserScan`：

```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

如果需要强制回到旧的 `PointCloud2` 配置：

```bash
ros2 run robot_navigation nav_assistant mapping-stack \
  --cartographer-config-basename pico_2d_mapping_quality.lua
```

带 RViz：

```bash
ros2 run robot_navigation nav_assistant mapping-stack --with-rviz true
```

### 1.2 手动遥控

```bash
ros2 run robot_navigation nav_assistant teleop
```

### 1.3 保存和导出地图

```bash
ros2 run robot_navigation nav_assistant save-map --map-name testmap1
ros2 run robot_navigation nav_assistant export-map --map-name testmap1 --resolution 0.03
```

### 1.4 定位和 Nav2 阶段

当前默认运行时主链是 `AMCL + map_server + Nav2`：

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1
```

显式指定 `map yaml`：

```bash
ros2 run robot_navigation nav_assistant localization-stack \
  --map-name testmap1 \
  --map-yaml <repo_root>/Maps/testmap1.yaml
```

本机 EC463 Linux 路径示例：

```bash
ros2 run robot_navigation nav_assistant localization-stack \
  --map-name testmap1 \
  --map-yaml /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml
```

带 RViz：

```bash
ros2 run robot_navigation nav_assistant localization-stack \
  --map-name testmap1 \
  --with-nav2-rviz true
```

无头 Jetson：

```bash
ros2 run robot_navigation nav_assistant localization-stack \
  --map-name testmap1 \
  --with-nav2-rviz false
```

显式选择 Nav2 profile：

```bash
ros2 run robot_navigation nav_assistant localization-stack \
  --map-name testmap1 \
  --nav-profile smac_mppi_omni
```

AMCL 固定初始位姿：

```bash
ros2 run robot_navigation nav_assistant amcl-initialpose \
  --x 0.0 --y 0.0 --yaw 0.0
```

AMCL 全局重定位：

```bash
ros2 run robot_navigation nav_assistant amcl-global-localize
```

Legacy 路径，只有在回归对比时才用 Cartographer runtime localization：

```bash
ros2 run robot_navigation nav_assistant localization-stack \
  --runtime-localizer cartographer \
  --map-name testmap1
```

### 1.5 发送导航目标

```bash
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

多点：

```bash
ros2 run robot_navigation nav_assistant waypoints \
  --pose 1.0,0.0,0.0 \
  --pose 1.5,0.5,0.0 \
  --pose 0.5,1.0,0.0
```

### 1.6 只读验证命令

验证 AMCL 运行时链路：

```bash
ros2 run robot_navigation nav_assistant verify-localization \
  --map-yaml <repo_root>/Maps/testmap1.yaml
```

本机 EC463 Linux 路径示例：

```bash
ros2 run robot_navigation nav_assistant verify-localization \
  --map-yaml /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml
```

验证当前 Nav2 profile：

```bash
ros2 run robot_navigation nav_assistant verify-nav-profile \
  --nav-profile smac_mppi_omni
```

打印当前 runbook：

```bash
ros2 run robot_navigation nav_assistant print-runbook --map-name testmap1
```

## 2. 分组件调试启动

以下命令用于拆分排查，不是日常首选。

### 2.1 SICK LiDAR 驱动

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

### 2.2 static TF

`base_link -> lidar_link`

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.2413 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id lidar_link
```

`lidar_link -> lidar_link_1`

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id lidar_link --child-frame-id lidar_link_1
```

`lidar_link -> imu_link`

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0124 --y 0.0185 --z -0.0484 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id lidar_link --child-frame-id imu_link
```

说明：

- 当前项目预设 `base_link -> lidar_link.x = 0.2413 m`
- `0.2413 m = 20 in / 2 - 1 in / 2 = 9.5 in`
- segmented `LaserScan` 当前显式固定为 `last echo only`
- `lidar_link -> lidar_link_1` 是给 `sick_scan_xd` 的 segmented frame suffix 用的
- `0.0124, 0.0185, -0.0484 m` 来自 SICK 说明书中的 IMU 相对 optical origin 偏移

### 2.3 Cartographer mapping

```bash
ros2 launch robot_navigation cartographer_mapping.launch.py
```

### 2.4 Cartographer localization

```bash
ros2 launch robot_navigation cartographer_localization.launch.py \
  load_state_filename:=<repo_root>/Maps/testmap1.pbstream \
  configuration_basename:=pico_2d_localization_scan_segment.lua
```

本机 EC463 Linux 路径示例：

```bash
ros2 launch robot_navigation cartographer_localization.launch.py \
  load_state_filename:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream \
  configuration_basename:=pico_2d_localization_scan_segment.lua
```

### 2.5 串口桥

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

### 2.6 EKF

```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file <repo_root>/workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml \
  -r odometry/filtered:=/odom
```

本机 EC463 Linux 路径示例：

```bash
ros2 run robot_localization ekf_node --ros-args \
  --params-file /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml \
  -r odometry/filtered:=/odom
```

### 2.7 map_server 和 lifecycle

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=<repo_root>/Maps/testmap1.yaml
```

本机 EC463 Linux 路径示例：

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml
```

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true \
  -p node_names:="[map_server]"
```

### 2.8 Nav2 bringup

AMCL 主路径：

```bash
ros2 launch robot_navigation nav2_amcl_localization_stack.launch.py \
  map_yaml:=<repo_root>/Maps/testmap1.yaml
```

本机 EC463 Linux 路径示例：

```bash
ros2 launch robot_navigation nav2_amcl_localization_stack.launch.py \
  map_yaml:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml
```

Legacy Cartographer + Nav2 参数：

```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=<repo_root>/workspace/src/robot_navigation/config/nav2_params_cartographer.yaml
```

## 3. 常用检查命令

```bash
ros2 run robot_navigation nav_assistant quick-check
ros2 run robot_navigation nav_assistant verify-localization --map-yaml <repo_root>/Maps/testmap1.yaml
ros2 run robot_navigation nav_assistant verify-nav-profile --nav-profile smac_mppi_omni
ros2 topic hz /odom
ros2 topic hz /scan_segment
ros2 action list | grep navigate_to_pose
```

如果当前跑的是 legacy Cartographer runtime，再额外检查：

```bash
ros2 node info /cartographer_node | grep scan
```

## 4. 当前关键默认值

- `runtime-localizer` 默认: `amcl`
- `nav-profile` 默认: `smac_mppi_omni`
- `mapping-stack` 默认配置: `pico_2d_mapping_quality_scan_segment.lua`
- `localization-stack` 默认配置: `pico_2d_localization_scan_segment.lua`
- `odom_topic` 默认: `/odom_raw`
- `use_ekf` 默认: `true`
- `fallback_odom` 默认: `false`
- LiDAR frame 默认: `lidar_link`
- IMU frame 默认: `imu_link`
- static TF 默认: `base_link -> lidar_link = (0.2413, 0, 0, 0, 0, 0)`
- static TF 默认: `lidar_link -> imu_link = (0.0124, 0.0185, -0.0484, 0, 0, 0)`
- `with_nav2_rviz` 默认: `false`
