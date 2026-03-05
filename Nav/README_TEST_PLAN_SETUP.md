# Grocery Robot Test Plan Setup

This document defines the test setup and execution plan for the Team 21 Grocery Robot demo pipeline.
It combines:

- SLAM and Nav2 bringup (`Nav/README_SLAM_UPDATED.md`)
- Power distribution and wiring diagram (`Nav/README_POWER_BLOCK_DIAGRAM.md`)
- Base command bridge (`STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/nav2_serial_bridge.py`)
- Remote teleop (`STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/teleop_cmd_vel.py`)
- Behavior Tree order execution (`workspace/src/robot_task_manager/robot_task_manager/bt_executor.py`)
- Web GUI order injection (`order-api/server.js`)

## 1. Test Objectives

The test session validates three demo blocks:

1. Bringup and command path: SSH into Jetson, start bridge, verify `/cmd_vel` reaches the base.
2. Mapping and map handoff: run Cartographer mapping, save map, export static map files.
3. Nav2 + task flow: load map in Nav2, click goals in RViz, and trigger Behavior Tree with remotely submitted order items.

## 2. Hardware Setup

### 2.1 Required Hardware

- NVIDIA Jetson Orin Nano Super Developer Kit (Ubuntu 22.04)
- DJI Development Board A (base controller board, UART link target)
- SICK PicoScan150 LiDAR (Ethernet)
- Robot chassis with motor drivers and wheel encoders
- Power system for Jetson, control board, and sensors
- USB-to-UART connection from Jetson to DJI A-board (for bridge telemetry/command path)
- Ethernet link between Jetson and LiDAR (direct or via switch)
- Optional remote laptop for SSH and web GUI access

### 2.2 Recommended Wiring and Interface Map

- LiDAR -> Jetson: Ethernet UDP, static IP pair (example in this repo: LiDAR `192.168.8.150`, Jetson `192.168.8.249`)
- Jetson -> DJI A-board: serial UART via `/dev/ttyUSB0` at `115200`
- Jetson -> network: SSH and browser access for remote operation

### 2.3 Safety Before Power-On

- Robot on open floor, no loose cables near wheels
- E-stop reachable
- Wheels free to move
- Correct supply voltage polarity confirmed

## 3. Software Stack

### 3.1 OS and Middleware

- Ubuntu 22.04 on Jetson
- ROS 2 Humble

### 3.2 ROS/Navigation Components

- `sick_scan_xd` for SICK PicoScan
- `cartographer_ros` for SLAM/localization
- `nav2_bringup`, `nav2_map_server`, `nav2_lifecycle_manager`
- `tf2_ros`

### 3.3 Python Components Used by This Repo

- `rclpy`
- `pyserial` (required by `nav2_serial_bridge.py`)
- `py_trees` (required by task manager BT)
- `requests` (task manager polling `order-api`)
- `firebase-admin` (inventory updates in BT node)

### 3.4 Web/API Components

- Node.js (18+ recommended)
- `express`, `body-parser`, `cors`, `firebase-admin`
- Web UI served by `order-api/server.js` on port `3000`

## 4. Repository Components Under Test

- Base bridge: `STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/nav2_serial_bridge.py`
  - Subscribes to `/cmd_vel` (also `/cmd_vel_nav`, `/cmd_vel_smoothed` by parameter)
  - Sends command frames to UART
  - Publishes `/odom`
- Keyboard teleop: `STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/teleop_cmd_vel.py`
- SLAM runbook: `Nav/README_SLAM_UPDATED.md`
- Nav2 params: `Nav/nav2_params_cartographer.yaml`
  - `bt_navigator` and `behavior_server` enabled
  - AMCL `tf_broadcast: false` to avoid map TF conflicts during Cartographer localization
- BT executor: `workspace/src/robot_task_manager/robot_task_manager/bt_executor.py`
  - Receives orders via `/order/new` service and Node API polling
  - Builds customer/employee trees
- Order API/UI: `order-api/server.js`, `order-api/public/*.html`

## 5. Environment Preparation

Set a repo root on Jetson:

```bash
export REPO_ROOT=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot
```

In every ROS terminal:

```bash
source /opt/ros/humble/setup.bash
source $REPO_ROOT/workspace/install/setup.bash
```

If workspace is not built yet:

```bash
cd $REPO_ROOT/workspace
colcon build --symlink-install
source install/setup.bash
```

Install missing Python dependencies if needed:

```bash
python3 -m pip install --user pyserial py_trees requests firebase-admin
```

Initialize and install Node dependencies for `order-api`:

```bash
npm init -y
npm pkg set type=module
npm install express body-parser cors firebase-admin
```

Credential prerequisites:

- `order-api/credential.json` must exist for API server Firestore access.
- `workspace/src/robot_task_manager/robot_task_manager/bt_nodes/credential.json` must exist for BT inventory update node.

## 6. Pre-Test Checklist

- `ros2 topic list` returns expected ROS topics
- `ls /dev/ttyUSB*` confirms serial device exists
- LiDAR ping reachable from Jetson
- No stale processes occupying serial or port `3000`
- Required map folder exists: `$REPO_ROOT/Maps`

## 7. Test Execution Plan

### 7.1 Block A: Bringup and `/cmd_vel` Command Path

Goal: verify teleop commands are received by bridge and forwarded to base controller.

Terminal A1 (bridge):

```bash
python3 $REPO_ROOT/STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/nav2_serial_bridge.py \
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

Terminal A2 (teleop):

```bash
python3 $REPO_ROOT/STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/teleop_cmd_vel.py \
  --topic /cmd_vel \
  --linear 0.6 \
  --angular 1.2
```

Checks:

- Bridge logs show no repeated serial write errors.
- `ros2 topic hz /cmd_vel` reports active publish rate.
- Chassis responds to `W/S/A/D/Q/E` and stops on `Space`.

Pass criteria:

- Stable command stream and expected base movement.

### 7.2 Block B: Mapping Run and Map Export

Goal: complete one mapping loop and generate `.pbstream + .yaml + .pgm`.

Terminal B1 (LiDAR):

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

Terminal B2 (Cartographer mapping):

```bash
ros2 launch $REPO_ROOT/Nav/carto_cfg/my_carto.launch.py
```

Terminal B3 (bridge): use the same command from Block A.

Terminal B4 (teleop): use the same command from Block A.

Terminal B5 (optional visualization):

```bash
ros2 run rviz2 rviz2
```

After driving the full route, save and export map:

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$REPO_ROOT/Maps/testmap1.pbstream', include_unfinished_submaps: true}"
```

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename $REPO_ROOT/Maps/testmap1.pbstream \
  -map_filestem $REPO_ROOT/Maps/testmap1 \
  -resolution 0.05
```

Pass criteria:

- `testmap1.pbstream`, `testmap1.yaml`, `testmap1.pgm` exist.
- RViz/map output is visually consistent with environment.

### 7.3 Block C: Localization + Nav2 Goal Navigation

Goal: run full Nav2 stack on saved map and execute point-to-point navigation.

Keep running:

- LiDAR driver
- Serial bridge

Start localization:

```bash
ros2 launch $REPO_ROOT/Nav/carto_cfg/my_carto_localization.launch.py \
  load_state_filename:=$REPO_ROOT/Maps/testmap1.pbstream \
  publish_occupancy_grid:=false
```

Start map server:

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=false \
  -p yaml_filename:=$REPO_ROOT/Maps/testmap1.yaml
```

Activate lifecycle:

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=false \
  -p autostart:=true \
  -p node_names:='["map_server"]'
```

Start Nav2:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=$REPO_ROOT/Nav/nav2_params_cartographer.yaml
```

Start Nav2 RViz:

```bash
ros2 launch nav2_bringup rviz_launch.py
```

Execution checks:

- In RViz, set a goal with `2D Goal Pose`.
- Verify command output during active goal:

```bash
ros2 topic hz /cmd_vel
```

- Optional CLI goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

Pass criteria:

- Robot receives Nav2 velocity commands and drives to target pose.

### 7.4 Block D: Behavior Tree + Remote GUI Order Injection

Goal: demonstrate remote operator submitting order items and BT processing them.

Terminal D1 (API server on Jetson):

```bash
cd $REPO_ROOT/order-api
node server.js
```

Terminal D2 (BT executor on Jetson):

```bash
ros2 run robot_task_manager bt_executor
```

Remote access options:

- Option 1: run browser directly on Jetson and open `http://localhost:3000`
- Option 2: from remote laptop, use SSH tunnel then open local browser:

```bash
ssh -L 3000:localhost:3000 <jetson_user>@<jetson_ip>
```

Then browse:

- `http://localhost:3000/login.html`
- Login as customer (or continue as guest), add two products, submit order

Expected BT behavior:

- BT logs show order load: `items=2`
- Mode resolved to `customer` or `employee`
- Tree starts and ticks (`BT status: ...`)
- Demo movement leaf publishes `/cmd_vel` for each item (`MoveDistanceForCurrentItem`)
- Completion is reported back to API (`/api/order/complete`)

Pass criteria:

- Order is accepted and processed end-to-end without node crash.

Fallback (no GUI) service injection:

```bash
ros2 service call /order/new robot_interfaces/srv/NewOrder \
  "{order: {order_id: 9001, role: customer, requester_id: demo_user, items: [{product_id: '002', name: 'Apple', aisle: 'A2', rack: 3, shelf_level: 1, qty: 1, price: 20.0, stock: 120}, {product_id: '003', name: 'Orange', aisle: 'A3', rack: 2, shelf_level: 1, qty: 1, price: 15.0, stock: 50}]}}"
```

## 8. Observability and Evidence Collection

Collect during test:

- Console logs from bridge, Cartographer, Nav2, BT executor, and `order-api`
- `ros2 topic hz /cmd_vel`
- `ros2 topic echo /odom --once`
- `ros2 topic info /map`
- `ros2 action list | grep -E 'navigate_to_pose|follow_waypoints'`
- RViz screenshots (map, pose, goal markers)

## 9. Known Limitations (Current Stage)

- Robustness is still under active development.
- Current navigation integration is primarily LiDAR + odometry in the live loop.
- IMU fusion tuning and validation are planned next.
- Planner setup currently uses NavFn (`use_astar: false`) in `Nav/nav2_params_cartographer.yaml`; evaluation of stronger planning/recovery strategies is planned.
- BT movement currently uses a temporary fixed-distance `/cmd_vel` demo leaf (`MoveDistanceForCurrentItem`) and should later be replaced by full Nav2 action integration.

## 10. Regression Checklist

Run before each demo day:

1. Serial bridge starts cleanly and publishes `/odom`.
2. Teleop drives base and deadman stop works.
3. Mapping can save and export map files.
4. Nav2 can load map and execute at least one goal.
5. GUI/API path can submit an order and BT processes all items.
6. No TF conflicts (`map -> odom -> base_link` remains consistent).
