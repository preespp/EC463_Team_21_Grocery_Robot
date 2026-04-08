# Pickup Full-System Runbook

This document keeps only one standard path:

**the full pickup system startup path**

Target readers:

- testers taking over the project for the first time
- anyone who wants to bring up the entire pickup chain directly
- anyone who does not want to decide between "minimal mode" and "full mode"

## 1. Bottom Line First

The repository does not yet have a true one-command full-system bringup.

So the most reliable method right now is not to search for a universal launch file. The reliable method is to start the following components in a fixed order:

1. backend
2. optional UI
3. navigation stack
4. optional `rosbridge_server` for remote SLAM map viewing
5. MoveIt + `/pick_viperx`
6. `camera_vision`
7. `bt_executor_viperX`

Then drive the full pickup flow either through:

- backend / UI order submission
- or a manual `/order/new` request

## 2. Important Rules for the Current Full-System Setup

### 2.1 Do not launch `vx300_auto_pick.launch.py` for the BT main flow

During full pickup testing, **do not** use:

```bash
ros2 launch robot_manipulation vx300_auto_pick.launch.py ...
```

Reason:

- that launch also starts the old `vision_auto_pick.py`
- the old `vision_auto_pick.py` also subscribes to `/detections_json`
- it also sends actions to `/pick_viperx`
- it will compete with `bt_executor_viperX` for arm control

So the correct full-system combination is:

- launch navigation separately
- use `vx300_moveit.launch.py` for the arm
- launch `camera_vision` separately
- launch `bt_executor_viperX` separately

### 2.2 Use stable by-id serial device names

The repository has already switched to stable USB device paths instead of relying on `ttyUSB0` / `ttyUSB1` ordering.

For full-system testing, the base and arm should stay fixed as:

- mobile base: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- arm: `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0`

### 2.3 Keep Nav2 RViz off by default in this runbook

This runbook keeps Nav2 RViz disabled by default.

Reason:

- this path is intended to be usable from a headless or remote machine
- we want remote live-map display to go through `rosbridge_server`
- this avoids depending on a local desktop session just to watch the map

If you do want a local Nav2 RViz window for debugging, you can manually change:

- `--with-nav2-rviz false`

to:

- `--with-nav2-rviz true`

### 2.4 BT main entry point

The entry point for the full pickup flow is:

```bash
ros2 run robot_task_manager bt_executor_viperX
```

Note the capital `X` at the end.

## 3. One-Time Preparation

If you changed source code, rebuild first:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --base-paths src --packages-select \
  robot_interfaces robot_navigation robot_manipulation robot_vision robot_task_manager
```

For all ROS terminals below, prepare the environment like this:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
```

To reduce operator mistakes, the command blocks for terminals 3 through 7 already include the full `cd` and `source` setup and can be copied directly.

## 4. Standard Full Startup Order

The following is the currently recommended full-system startup order.

### Terminal 1: Backend

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/order-api-postgre
npm install
npm run dev
```

After this starts correctly, the backend should be available at:

- `http://localhost:3000`

Prerequisites:

- PostgreSQL is already prepared
- backend schema and seed data have already been initialized

### Terminal 2: Fleet Manager UI

If you need the frontend management UI, open another terminal:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/order-api-postgre/fleet-manager
npm install
npm run dev -- --host 0.0.0.0
```

The frontend is usually available at:

- `http://localhost:5174`

Notes:

- the pickup main path truly depends on the backend
- fleet-manager is mainly a visualization and management entry point

### Terminal 3: Navigation Stack

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 run robot_navigation nav_assistant localization-stack \
  --map-name testmapMain \
  --with-nav2-rviz false \
  --serial-port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

This starts:

- localization
- Nav2
- semantic map service
- serial bridge

This runbook intentionally keeps local Nav2 RViz off by default.

### Terminal 4: Optional rosbridge for remote real-time SLAM map

One-time install if it is not already present:

```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-server
```

Startup command:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Inside the embedded app, connect to:

- `ws://192.168.8.249:9090`

Notes:

- the connection page defaults to the current hostname and port `9090`
- if you are connecting from another machine, make sure the hostname/IP is changed to the robot host
- this is the recommended way to show the real-time SLAM map remotely in this runbook

### Terminal 5: MoveIt + `/pick_viperx`

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 launch robot_manipulation vx300_moveit.launch.py \
  robot_model:=vx300s \
  robot_name:=vx300s \
  motor_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0
```

This provides:

- MoveIt
- ViperX controllers
- `/pick_viperx`

### Terminal 6: Camera Vision

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 run robot_vision camera_vision --ros-args \
  -p parent_frame:=vx300s/ee_gripper_link \
  -p camera_mount_frame:=camera_mount_frame \
  -p camera_optical_frame:=camera_color_optical_frame
```

If you want the live image window, add:

```bash
-p show_live_window:=true
```

### Terminal 7: BT Executor

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 run robot_task_manager bt_executor_viperX
```

After this step, the BT will start to:

- poll backend orders
- build the ViperX customer tree
- resolve semantic targets
- navigate the base to the stop position first
- then execute scan / detect / pick / place

## 5. Ways to Submit Orders

In the full system, the preferred way is to submit orders through the backend or UI.

If you are only testing the chain, you can still keep a manual backup path.

### Method A: Submit from UI / backend

This is the closest to the real system behavior.

The chain is:

- a user page or backend API submits an order
- the backend writes the order
- `bt_executor_viperX` polls `http://localhost:3000/api/order/latest`
- the BT automatically accepts and executes the order

### Method B: Manually call `/order/new`

If the UI is not ready yet, but you still want the full ROS system to be up, you can submit a manual order:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 service call /order/new robot_interfaces/srv/NewOrder "{order: {order_id: 1, role: customer, requester_id: test, items: [{product_id: '3', name: 'Apple', aisle: '0', rack: 0, shelf_level: 2, qty: 1, price: 0.0, stock: 1}]}}"
```

This still counts as a full-system test because:

- backend
- navigation
- MoveIt
- vision
- BT

are all already running.

## 6. Recommended Item Names for Testing

To reduce name-mismatch problems, prefer item names that already exist in the semantic map:

- `Green Tea`
- `Water`
- `Apple`
- `Orange`
- `Lemon`
- `Can`
- `Bag of Chips`

The safest approach is:

- fill both `product_id` and `name`
- use the exact values above for `name`

## 7. Bringup Checks After Startup

### Check 1: Arm action server

```bash
ros2 action list | grep pick_viperx
```

You should see:

- `/pick_viperx`

### Check 2: Navigation action

```bash
ros2 action list | grep navigate_to_pose
```

You should see:

- `/navigate_to_pose`

### Check 3: Semantic resolve service

```bash
ros2 service list | grep semantic_map
```

You should at least see:

- `/semantic_map/resolve_target`

### Check 4: Vision detection

```bash
ros2 topic hz /detections_json
ros2 topic echo /detections_json --once
```

### Check 5: BT service

```bash
ros2 service list | grep order
```

You should see:

- `/order/new`

### Check 6: Backend

Open this in a browser:

- `http://localhost:3000`

At minimum, confirm that the backend process is actually listening.

### Check 7: rosbridge for remote map viewing

If you launched the optional bridge terminal:

```bash
ros2 node list | grep rosbridge
```

Inside the embedded app, confirm it connects to:

- `ws://192.168.8.249:9090`

## 8. Expected Behavior of a Full Pickup Run

After an order truly enters the system, the current main flow should look like this:

1. the backend has an order
2. the BT polls and fetches it
3. the semantic map resolves the `nav_goal` for the current item
4. the mobile base navigates to the target stop position
5. the arm moves to a scan pose
6. `camera_vision` detects the target
7. the BT generates `pregrasp / grasp / lift` poses
8. `/pick_viperx` executes the grasp
9. the arm moves to the basket pose
10. inventory is updated
11. the flow ends after all items are done

## 9. Most Common Failure Points Right Now

### Problem 1: BT and old `vision_auto_pick.py` were launched together

Result:

- two upper-level controllers both send actions to `/pick_viperx`

How to avoid it:

- launch only `vx300_moveit.launch.py`
- do not launch `vx300_auto_pick.launch.py`

### Problem 2: Base and arm serial ports were swapped

For full-system testing, keep them fixed as:

- navigation: `--serial-port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- arm: `motor_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0`

### Problem 3: Item names do not match

If the following three names do not match:

- the `name` in the order
- the `product_names` in the semantic map
- the class name published by vision

then you may see:

- semantic resolve cannot find the target
- or vision sees it but the BT rejects it

### Problem 4: Only checking the UI, not whether the ROS nodes are truly up

The current system is assembled from multiple separate processes.

So if the UI opens, that does not mean:

- Nav2 is up
- `/pick_viperx` is up
- `camera_vision` is publishing detections
- the BT is able to accept orders

You still need to check each item in Section 7.

### Problem 5: The remote SLAM page cannot connect

Common causes:

- `rosbridge_server` is not running
- the embedded app is still trying to use the wrong hostname
- port `9090` is blocked by the network or firewall

## 10. Final Advice for Testers

If your goal is to answer "can the full system actually run pickup", then stop separating minimal mode and full mode and just launch the terminals in the order shown in this document.

The standard full-system startup order is:

**backend -> UI -> navigation -> optional rosbridge -> MoveIt -> camera_vision -> bt_executor_viperX**

After that point, problems should no longer be blamed on "the wrong entry point was launched". You should instead check:

- whether the order was written into the backend
- whether semantic map resolution succeeded
- whether Nav2 truly reached the target
- whether `/detections_json` contains a valid target
- whether `/pick_viperx` executed successfully

## 11. Patch Log 2026-04-08

### 11.1 Nav2 docking-distance tuning

The goal of this patch round was:

- verify whether the final pickup docking stage was mainly being pushed away from the shelf by `local_costmap` inflation
- change only local inflation behavior instead of relaxing global route planning at the same time

Applied in:

- `workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml`

Scope of this round:

- change only `local_costmap.inflation_layer`
- keep `global_costmap` unchanged
- keep `robot_radius` unchanged
- keep semantic `nav_pose` unchanged

Before vs after:

| Parameter | Before | After |
| --- | --- | --- |
| `local_costmap.inflation_layer.inflation_radius` | `0.55` | `0.42` |
| `local_costmap.inflation_layer.cost_scaling_factor` | `3.0` | `6.0` |
| `global_costmap.inflation_layer.inflation_radius` | `0.55` | `0.55` |
| `global_costmap.inflation_layer.cost_scaling_factor` | `3.0` | `3.0` |

Reason:

- the current issue is mainly the final stop near the target shelf, not an overly conservative global route
- `local_costmap` has the most direct effect on the last meter of MPPI approach and stop behavior
- reducing local `inflation_radius` shrinks the inflated keep-away band outside the shelf
- increasing local `cost_scaling_factor` makes inflation cost fall off faster away from the obstacle boundary, reducing the chance of stopping early before the desired pickup standoff
- `global_costmap` was intentionally left unchanged in this round to isolate the variable and avoid introducing path-planning side effects first

Testing note:

- this patch changes the source config in `robot_navigation`
- if you want the default localization stack to truly load this version of the parameters, rebuild `robot_navigation` before testing
- if you do not rebuild, launch may still read the older installed config

### 11.2 Return-home recovery after customer-order failure

Applied in:

- `workspace/src/robot_task_manager/robot_task_manager/trees/customer_viperx.py`
- `workspace/src/robot_task_manager/robot_task_manager/bt_executor_viperx.py`
- `workspace/src/robot_task_manager/robot_task_manager/blackboard.py`

Goals of this round:

- if a customer pickup order fails before completion, the base should still attempt to return to `home_goal`
- allow a few retries for return-home navigation while preserving the overall order result as `FAILED`
- keep "failure recovery after incomplete order execution" separate from "return-home at the end of a successful order"

Behavior change:

- before: `SetHome -> MaybeNavigateToGoalPose` only ran after the full order completed successfully
- after: if any key step fails before all items are done, the tree enters a failure-recovery branch and runs `SetHome -> Retry(MaybeNavigateToGoalPose)`, then explicitly preserves the tree result as `FAILURE`
- the success-path return-home navigation now also uses retries

New parameter:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `return_home_retry_attempts` | `3` | number of retries for navigating the base back to `home_goal` during successful shutdown or failure recovery in the customer pickup tree |

Implementation notes:

- the blackboard now carries `customer_order_items_completed` to distinguish "order body completed" from "order body failed early"
- the failure return-home branch only runs if `repeat_each_item` did not complete
- if failure recovery reaches home successfully, the tree still returns `FAILURE`, so the backend continues to treat the order as a failed order
- if return-home still fails after retries on the success path, the overall order is still treated as failed

Trigger conditions:

- success path: once customer pickup finishes `repeat_each_item` successfully, the tree runs one retry-enabled return to `home_goal`
- failure path: `ReturnHomeOnFailure` only runs when customer pickup fails before the order body completes
- more precisely, the robot does not "go home immediately when something looks wrong"; the current step must first bubble up a final `FAILURE`

Typical cases that can trigger failure-side return-home:

- `SetCurrentItem` fails, for example because the order state is inconsistent or the index is already out of range
- Nav2 navigation to the shelf stop position fails finally
- the vision-search stage fails finally, for example after center / left / right scans still do not produce a stable target, or the wait exceeds `search_timeout_sec`
- `PrepareDetectedPickPoses` fails, for example because the detected point is outside the arm workspace
- the grasp motion still fails after `RetryGrabFromPregrasp` exhausts its retry budget
- the basket placement flow fails, such as no valid basket slot or an arm step inside the place sequence fails
- the arm fails to return to `home_pose` after the item flow
- `ChangeInventory` fails, for example because the backend inventory API request fails

Important boundaries:

- `ResolveCurrentItemSemanticTargetViperX` falls back to legacy coordinates by default, so semantic-resolution problems do not always trigger return-home; it only triggers if that node finally returns `FAILURE`
- the detection stage is not "one look then fail"; it first waits for fresh detections, checks stability, and scans center / left / right
- this automatic return-home logic currently exists only in the customer ViperX pickup tree, not in the employee/restock tree

Testing note:

- this patch only changes the customer ViperX tree; the employee/restock tree is unchanged
- `colcon` is not available in the current environment, so this was validated only at Python syntax level, not with a ROS runtime test
