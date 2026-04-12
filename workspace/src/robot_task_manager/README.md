# robot_task_manager

`robot_task_manager` is the Behavior Tree orchestration package for the grocery robot. It accepts orders, manages shared blackboard state, resolves semantic targets, coordinates navigation and manipulation, and runs the customer and restock workflows for the ViperX-based system.

## What This Package Is

This package is responsible for:

- running the Behavior Tree executor
- managing order intake from ROS services and the Node.js backend
- maintaining blackboard state for navigation, semantics, and manipulation
- coordinating navigation goals, shelf targets, camera verification, and arm actions
- running customer and employee / restock workflows

## Tech Stack

- ROS 2 Humble
- Python ROS 2 nodes with `rclpy`
- `py_trees` Behavior Trees
- TF2
- shared custom interfaces from `robot_interfaces`
- integration with `robot_navigation`, `robot_manipulation`, and `robot_vision`

## Main Nodes

- `bt_executor`: older BT entrypoint
- `bt_executor_viperX`: current ViperX-focused Behavior Tree executor

## Main Files And Modules

- `robot_task_manager/bt_executor_viperx.py`: main runtime executor
- `robot_task_manager/blackboard.py`: shared state setup and reset helpers
- `robot_task_manager/trees/customer_viperx.py`: ViperX customer workflow
- `robot_task_manager/trees/restock_viperx.py`: ViperX employee / restock workflow
- `robot_task_manager/bt_nodes/`: BT nodes for navigation, semantics, arm actions, rack actions, and ViperX vision handling
- `config/viperx_bt_states.yaml`: BT-side command naming template

## High-Level ViperX Flow

For the customer workflow, the package currently does the following:

1. accepts or polls an order
2. resolves the semantic target into a navigation goal and shelf metadata
3. navigates to the target area unless navigation is skipped
4. moves the ViperX arm to an observation pose
5. verifies the object from `/detections_json`
6. transforms the detected point into `vx300/base_link`
7. sends a motion request to `/pick_viperx`
8. places the item in a basket pose

## Build

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
colcon build --base-paths src --packages-select robot_interfaces robot_manipulation robot_task_manager robot_vision
source install/setup.bash
```

## Environment

```bash
source /opt/ros/humble/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
```

## Common Workflows

### 1. Recommended Full Integrated Launch

Use the package launch file that starts arm, navigation, camera, BT, and optional ultrasonic sensing:

```bash
ros2 launch robot_task_manager viperx_nav_camera_bt.launch.py \
  motor_port:=/dev/ttyUSB1 \
  serial_port:=/dev/ttyUSB0 \
  include_ultrasonic:=true
```

Useful arguments:

- `include_ultrasonic:=true|false`
- `launch_navigation:=true|false`
- `skip_navigation:=true|false`
- `show_live_window:=true|false`
- `pbstream_file:=...`
- `map_yaml:=...`

### 2. Quick Test: Arm + Camera + BT Only

Terminal 1:

```bash
ros2 launch robot_manipulation vx300_moveit.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1
```

Terminal 2:

```bash
ros2 run robot_vision camera_vision --ros-args \
  -p parent_frame:=vx300s/ee_gripper_link \
  -p camera_mount_frame:=camera_mount_frame \
  -p camera_optical_frame:=camera_color_optical_frame
```

Terminal 3:

```bash
ros2 run robot_task_manager bt_executor_viperX --ros-args -p skip_navigation:=true
```

Terminal 4:

```bash
ros2 service call /order/new robot_interfaces/srv/NewOrder "{order: {order_id: 1, role: customer, requester_id: test, items: [{product_id: TEST1, name: bottle, aisle: '0', rack: 0, shelf_level: 1, qty: 1, price: 0.0, stock: 1}]}}"
```

### 3. Full Stack With Backend / UI

Run arm, camera, and BT as above, but start the BT without `skip_navigation:=true` and launch the normal Node.js backend / UI separately.

## Order Intake

The executor provides:

- `/order/new` using `robot_interfaces/srv/NewOrder`

It can also poll the Node.js backend at `http://localhost:3000` for orders.

## Vision And Semantic Integration

- camera detections come from `robot_vision/camera_vision.py` on `/detections_json`
- semantic resolution comes from `/semantic_map/resolve_target`
- arm motions are sent through `robot_interfaces/action/PickArm` to `/pick_viperx`

The BT uses command-style arm presets such as:

- `startup_arm_pose`
- `return_arm_pose`
- `place_arm_pose`
- `shelf_level_1_pose`
- `shelf_level_2_pose`
- `shelf_level_3_pose`

Those command names must also exist in `robot_manipulation/config/viperx_arm_server.yaml`.

## Blackboard Formats

Important blackboard shapes include:

- `bb.nav_goal = (x, y, yaw)`
- `bb.shelf_pose = {"command": "shelf_level_2_pose"}`
- `bb.detected_object_pose = {"frame_id": "vx300/base_link", "x": ..., "y": ..., "z": ...}`

## Notes

- `skip_navigation:=true` is useful for table demos and arm-only debugging.
- The ViperX workflow assumes command-based shelf poses rather than direct z-height control.
- If a new BT command is added, the arm server config must be updated to recognize it.
