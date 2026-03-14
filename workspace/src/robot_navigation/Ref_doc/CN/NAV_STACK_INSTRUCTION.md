# Navigation Stack Instruction

## 0. 环境准备

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --symlink-install --packages-select robot_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

后续只重编译这个包时:
```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --symlink-install --packages-select robot_navigation
source install/setup.bash
```

## 1. 推荐流程（最稳，直接可用）

### 1.1 建图阶段（会启动 LiDAR + static TF + Cartographer + 串口桥 + EKF）
```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

当前 mapping-stack 默认会在 Cartographer 前面打开一个 `base_link` 裁剪滤波器，
裁剪框是 `x=[-0.2540, 0.1397] m`、`y=[-0.2794, 0.2794] m`、
`z=[-1.0, 1.0] m`，也就是现在这台车后部 `15.5 x 22 in` 的自车回波过滤框。
只有做专项排查时才建议用 `--with-base-link-crop false` 关闭。

D0 参数验证建议直接使用质量配置（PointCloud2 路径）：
```bash
ros2 run robot_navigation nav_assistant mapping-stack \
  --cartographer-config-basename pico_2d_mapping_quality.lua
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
ros2 run robot_navigation nav_assistant save-map --map-name testmapMain
ros2 run robot_navigation nav_assistant export-map --map-name testmapMain --resolution 0.03
```

### 1.4 定位 + Nav2 阶段（会启动 LiDAR + static TF + 串口桥 + EKF + Cartographer localization + map_server + Nav2）
当前工作命令:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz true
```

等价的直接 launch 版本:
```bash
ros2 launch robot_navigation nav2_localization_stack.launch.py \
  pbstream_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.pbstream \
  map_yaml:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.yaml \
  nav2_params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml \
  with_nav2_rviz:=true
```

说明：这个裁剪滤波只清理 Cartographer 的点云输入，不替代 Nav2 自己的
robot footprint / robot box。
Localization + Nav2 默认保持裁剪关闭，只有你显式传
`--with-base-link-crop true` 时才会打开。
当前 Nav2 默认参数文件是 `nav2_params_smac_mppi_omni.yaml`，
也就是 `SmacPlanner2D + MPPIController(Omni)` 这套组合。

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
  publish_imu_frame_id:=imu_link \
  tf_publish_rate:=0.0 \
  imu_udp_port:=7503 \
  scandataformat:=2 \
  send_sopas_start_stop_cmd:=0 \
  host_set_FREchoFilter:=0 \
  host_set_LFPangleRangeFilter:=0 \
  host_set_LFPintervalFilter:=0 \
  custom_pointclouds:=cloud_all_fields_fullframe \
  cloud_all_fields_fullframe:='coordinateNotation=3 updateMethod=0 fields=x,y,z,i,range,azimuth,elevation,t,ts,lidar_sec,lidar_nsec,ring,layer,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/cloud_all_fields_fullframe frameid=lidar_link publish=1' \
  imu_topic:=/sick_scansegment_xd/imu
```

### 2.2 static TF (`base_link -> lidar_link`, `lidar_link -> imu_link`)
```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.2413 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id lidar_link
```

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.0124 --y 0.0185 --z -0.0484 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id lidar_link --child-frame-id imu_link
```

Notes:

- 当前项目预设按你的安装假设处理：`lidar_link` 取 bracket 中心，且该中心与当前 project 的 optical-origin proxy 视为同一点。
- `0.2413 m` 来自 `20 in / 2 - 1 in / 2 = 9.5 in = 0.2413 m`，也就是 20 英寸底盘半宽减去标准 1 英寸 80/20 前横梁半宽。
- `0.0124, 0.0185, -0.0484 m` comes from the SICK operating instructions as IMU position relative to the optical origin.

### 2.3 Cartographer mapping
```bash
ros2 launch robot_navigation cartographer_mapping.launch.py
```

说明：单独直接起 `cartographer_mapping.launch.py` 时，不会自动带上默认裁剪滤波。
完整 mapping stack 里，Cartographer 实际接的是经过 `base_link` 裁剪后的
`/cloud_all_fields_fullframe_filtered`。

### 2.4 Cartographer localization
```bash
ros2 launch robot_navigation cartographer_localization.launch.py \
  load_state_filename:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.pbstream \
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
  -p yaml_filename:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.yaml
```

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true \
  -p node_names:="[map_server]"
```

### 2.8 Nav2 bringup
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml
```

## 3. 常用检查命令

```bash
ros2 run robot_navigation nav_assistant quick-check
ros2 node info /cartographer_node | grep points2
ros2 topic hz /odom
ros2 topic hz /cloud_all_fields_fullframe
ros2 action list | grep navigate_to_pose
```

## 4. 你当前代码里的关键默认值（便于对照）

- `odom_topic` 默认: `/odom_raw`
- `use_ekf` 默认: `true`
- `fallback_odom` 默认: `false`
- LiDAR frame 默认: `lidar_link`
- static TF 默认: `base_link -> lidar_link = (0.2413, 0, 0, 0, 0, 0)`
- `mapping-stack with_base_link_crop` 默认: `true`
- `localization-stack with_base_link_crop` 默认: `false`
- Cartographer 裁剪框默认:
  `x=[-0.2540, 0.1397] m`、`y=[-0.2794, 0.2794] m`、`z=[-1.0, 1.0] m`
- Nav2 的 footprint / robot box 仍然独立于这个裁剪逻辑
- Nav2 默认参数文件: `config/nav2_params_smac_mppi_omni.yaml`
- 当前 planner/controller: `SmacPlanner2D + MPPIController(Omni)`
- `localization-stack cmd_topics` 默认: `["/cmd_vel"]`
- `with_nav2_rviz` 默认: `false`
