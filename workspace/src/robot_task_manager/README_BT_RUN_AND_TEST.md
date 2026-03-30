# Behavior Tree Run And Test Guide

This document explains how to run and test the task manager with the other robot components.

It also includes notes for:

- testing only one subsystem at a time
- testing the arm without waiting for an order from the UI / Node.js order source
- temporarily commenting out BT lines to isolate one part of the flow

## 1. Relevant Components

Main pieces involved in the ViperX task flow:

- `robot_task_manager`
  - `bt_executor_viperX`
  - ViperX customer and restock trees
- `robot_manipulation`
  - MoveIt launch
  - `viperx_arm_server`
- `robot_vision`
  - `camera_vision`
- Node.js backend
  - order polling on `http://localhost:3000`

## 2. Environment Setup

Use the normal environment in each terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
```

If you changed source files and need to rebuild:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --base-paths src --packages-select robot_task_manager robot_manipulation robot_vision robot_interfaces
```

## 3. Normal Full-System Test

### Terminal 1: MoveIt + arm server

```bash
ros2 launch robot_manipulation vx300_moveit.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1
```

This gives you:

- MoveIt
- robot controllers
- `/pick_viperx`

### Terminal 2: Camera vision

If not already launched by a combined manipulation launch:

```bash
ros2 run robot_vision camera_vision --ros-args \
  -p parent_frame:=vx300s/ee_gripper_link \
  -p camera_mount_frame:=camera_mount_frame \
  -p camera_optical_frame:=camera_color_optical_frame
```

### Terminal 3: Task manager BT

```bash
ros2 run robot_task_manager bt_executor_viperX
```

This starts the ViperX executor in [`bt_executor_viperx.py`](./robot_task_manager/bt_executor_viperx.py).

It will:

- create service `/order/new`
- poll the backend at `http://localhost:3000/api/order/latest`
- build either the customer or restock tree based on `order.role`

### Terminal 4: UI / backend

Run your normal Node.js order backend and UI.

If the backend is not running, `bt_executor_viperX` will log:

- `Node.js unreachable`

## 4. Testing Without The UI

You have two main options.

### Option A: Use the `/order/new` ROS service

This is the cleanest way to test the BT without the UI.

The executor provides:

- `/order/new` using `robot_interfaces/srv/NewOrder`

You can send a manual request from ROS tooling or a small helper script.

Recommended use:

- create a single test order
- set `role` to `customer` or `employee`
- include exactly one item for simple testing

### Option B: Temporarily bypass order intake in the executor

For local debugging, you can temporarily modify [`bt_executor_viperx.py`](./robot_task_manager/bt_executor_viperx.py) and directly call:

- `self.accept_order(order)`

with a hand-constructed `Order()` in code.

This is useful when:

- you only want to test tree logic
- Node.js is not available
- you want repeatable one-item tests

## 5. Arm-Only Testing Without A Real Order

If you only want to test preset arm states or motion wiring, you do not need the full order/UI loop.

Recommended approach:

1. Start MoveIt + `/pick_viperx`
2. Start the BT executor or a minimal test script
3. Use a blackboard with hardcoded values such as:

```python
bb.pose = {"command": "startup_arm_pose"}
bb.home_pose = {"command": "return_arm_pose"}
bb.basket_pose = {"command": "place_arm_pose"}
bb.shelf_pose = {"command": "shelf_level_1_pose"}
```

4. Build a minimal tree that only includes:

```python
bt_nodes.RepositionViperXArm(goal_key="pose", bb=bb)
```

or:

```python
bt_nodes.RepositionViperXArm(goal_key="home_pose", bb=bb)
```

This is the safest way to validate:

- command names
- arm server preset mapping
- hardware motion for recorded states

## 6. Testing One Component At A Time

It is okay to comment out some BT lines while testing one subsystem.

This is often the fastest way to isolate bugs.

Examples:

### Test only navigation

In [`customer_viperx.py`](./robot_task_manager/robot_task_manager/trees/customer_viperx.py), temporarily comment out:

- `detect_and_approach`
- `MoveToDetectedPose`
- gripper and basket nodes

Keep only:

- `SetCurrentItem`
- semantic target resolution
- `NavigateToGoalPose`

### Test only camera detection

Temporarily comment out:

- `MoveToDetectedPose`
- `MoveViperXGripper`
- basket and inventory steps

Keep:

- `RepositionViperXArm(goal_key="pose", bb=bb)`
- `VerifyViperXPosition(bb=bb)`

This checks:

- `/detections_json`
- product matching
- TF conversion to `vx300/base_link`

### Test only preset arm states

Temporarily comment out:

- semantic node
- navigation
- camera detection
- inventory update

Keep only one or two `RepositionViperXArm(...)` calls with known commands.

### Test only basket placement

Set:

```python
bb.basket_pose = {"command": "place_arm_pose"}
```

Then keep only:

- `SelectBasketSlot(bb=bb)` or hardcode `bb.basket_pose`
- `RepositionViperXArm(goal_key="basket_pose", bb=bb)`
- `MoveViperXGripper(command="open")`

## 7. Recommended Incremental Test Order

Use this sequence to reduce risk.

1. Test `/pick_viperx` preset commands only
   - `startup_arm_pose`
   - `return_arm_pose`
   - `place_arm_pose`
2. Test camera-only detection
   - confirm `/detections_json` is being published
   - confirm BT transforms detection into `vx300/base_link`
3. Test semantic-only navigation
   - confirm `bb.nav_goal`
   - confirm shelf level maps to correct `bb.shelf_pose`
4. Test a one-item customer tree
5. Test a one-item restock tree

## 8. Useful ROS Checks

### Check arm action server

```bash
ros2 action list | grep pick_viperx
```

### Check camera detections

```bash
ros2 topic hz /detections_json
ros2 topic echo /detections_json --once
```

### Check order service

```bash
ros2 service list | grep order
```

### Check BT logs

The executor prints:

- order load events
- tree status each tick
- completion status

The leaf nodes also set `feedback_message`, which helps when a specific step fails.

## 9. Important Current Assumptions

The BT now expects these shelf preset commands to exist eventually:

- `shelf_level_1_pose`
- `shelf_level_2_pose`
- `shelf_level_3_pose`

Those names are referenced on the task-manager side, but the manipulation arm server must also be updated to support them before shelf command testing will pass.

## 10. Current Fastest Debug Paths

### Fastest way to test only arm preset states

- run MoveIt + arm server
- run a tiny tree with only `RepositionViperXArm`

### Fastest way to test only camera integration

- run camera node
- keep `pose -> VerifyViperXPosition`
- comment out arm/gripper/place nodes

### Fastest way to test BT without UI

- use `/order/new`
- or temporarily inject a hand-built order in `bt_executor_viperx.py`

## 11. Suggested Temporary Comment Targets

When isolating components, these are the safest lines to comment out temporarily in:

- [`customer_viperx.py`](./robot_task_manager/trees/customer_viperx.py)
- [`restock_viperx.py`](./robot_task_manager/trees/restock_viperx.py)

Common temporary removals:

- `bt_nodes.ChangeInventory(...)`
  - avoids backend side effects during testing
- `bt_nodes.NavigateToGoalPose(...)`
  - when only testing arm/camera
- `bt_nodes.VerifyViperXPosition(...)`
  - when only testing preset arm states
- `bt_nodes.MoveToDetectedPose(...)`
  - when only testing detection

After testing, restore the full tree before integration runs.
