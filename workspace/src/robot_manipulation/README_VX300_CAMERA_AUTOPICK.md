# VX300S + Camera Auto-Pick (Handoff)

This document summarizes today's integration work for:

- Interbotix VX300S arm bringup
- MoveIt-based action server (`/pick_viperx`)
- Camera detection bridge (`/detections_json` -> auto pick sequence)
- Safe preview and real-hardware test modes

## 1) What Is Running

- `viperx_arm_server` (`robot_manipulation/src/viperx_arm_server.cpp`)
  - Accepts `robot_interfaces/action/PickArm` goals on `/pick_viperx`
  - Executes arm pose goals and gripper commands through MoveIt
- `vision_auto_pick.py` (`robot_manipulation/scripts/vision_auto_pick.py`)
  - Subscribes to `/detections_json`
  - Filters detections and transforms into arm base frame
  - Sends pick-place sequence goals to `/pick_viperx`
- `camera_vision` (`robot_vision/robot_vision/camera_vision.py`)
  - Publishes detections with depth points in `camera_color_optical_frame`
  - Broadcasts `ee_gripper_link -> camera_mount_frame -> camera_color_optical_frame`
  - Broadcasts object TFs in the optical frame

## 2) Important Config Files

- Auto-pick behavior:
  - `workspace/src/robot_manipulation/config/vx300_auto_pick.yaml`
- Arm server behavior:
  - `workspace/src/robot_manipulation/config/viperx_arm_server.yaml`
- Camera mount defaults:
  - `workspace/src/robot_vision/robot_vision/camera_vision.py`
  - default `mount_xyz` is set to `-0.0635,0.0,0.0635`

## 3) Build + Environment

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --packages-select robot_manipulation robot_vision

source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
```

## 4) Launch (Main Entry)

```bash
ros2 launch robot_manipulation vx300_auto_pick.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1
```

This starts:

- MoveIt + controllers + `xs_sdk`
- `viperx_arm_server`
- `vision_auto_pick`

Optional: also launch camera in same launch:

```bash
ros2 launch robot_manipulation vx300_auto_pick.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1 \
  launch_camera_vision:=true \
  vision_parent_frame:=vx300s/ee_gripper_link \
  vision_camera_mount_frame:=camera_mount_frame \
  vision_camera_optical_frame:=camera_color_optical_frame
```

## 5) Preview Mode (No Hardware Motion)

Use preview mode when validating trajectories:

```bash
ros2 launch robot_manipulation vx300_auto_pick.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1 \
  use_viperx_preview:=true
```

Requirements for preview to work:

- `dry_run: false` in `vx300_auto_pick.yaml` (goals must be sent)
- `use_viperx_preview:=true` (server plans/publishes only)

Preview trajectory topic:

- `/display_planned_path`

## 6) Real Hardware Test (Safe First Pass)

Set in `vx300_auto_pick.yaml`:

- `dry_run: false`
- `pick_once: true`
- `enable_auto_pick: true`

Recommended conservative workspace for initial test:

- `workspace_min_x: 0.15`
- `workspace_max_x: 0.42`
- `workspace_min_y: -0.15`
- `workspace_max_y: 0.15`
- `workspace_min_z: 0.03`
- `workspace_max_z: 0.35`

Start one-bottle test with clear table and emergency stop ready.

## 7) Gripper Close Behavior

Current close target is configured as:

- `close_gripper_position: 0.045` in `vx300_auto_pick.yaml`
- `closed_gripper_pos: 0.045` in `viperx_arm_server.yaml`

This is a relatively open close target. If grasp is weak, reduce toward `0.028 ~ 0.035`.

## 8) Camera Mount Assumption

Current assumption:

- `parent_frame = vx300s/ee_gripper_link`
- `camera_mount_frame = camera_mount_frame`
- `camera_optical_frame = camera_color_optical_frame`
- `mount_xyz = (-0.0635, 0.0, 0.0635)` meters
- `mount_rpy_deg = (0, 0, 0)` for the mount/body frame
- `optical_frame_rpy_deg = (-90, 0, -90)` for the fixed optical-frame rotation

Tune `mount_xyz/mount_rpy_deg` if the camera is rigidly mounted but the pose is off.
Leave `optical_frame_rpy_deg` at the standard value unless you intentionally redefine the optical frame.

## 9) Success Indicators

Normal auto-pick logs:

- `Queued pick target ...`
- `Starting auto-pick sequence ...`
- `Auto-pick step 1/8 ... step 8/8`
- `Auto-pick sequence complete`

If `pick_once: true`, expect:

- `pick_once=true -> auto-pick disabled after successful pick`

## 10) Common Issues + Fixes

- `Action servers: 0` for `/pick_viperx`:
  - Launch stack with `vx300_auto_pick.launch.py` or `vx300_moveit.launch.py` with arm server enabled.
- Camera node error `Couldn't resolve requests`:
  - Replug RealSense and retry; use conservative stream settings if needed.
- Detection exists but no pick queued:
  - Check `target_class`, confidence threshold, depth validity, TF availability, workspace bounds.
- Arm motion looks wrong:
  - Stop immediately and retune camera mount transform and workspace constraints before retry.

## 11) Stop and Clean Restart

```bash
pkill -INT -f "ros2 launch|ros2 run|rviz2|move_group|viperx_arm_server|vision_auto_pick.py|camera_vision|xs_sdk|ros2_control_node|robot_state_publisher|spawner"
sleep 2
pkill -9 -f "rviz2|move_group|viperx_arm_server|vision_auto_pick.py|camera_vision|xs_sdk|ros2_control_node|robot_state_publisher|spawner"
ros2 daemon stop
sleep 1
ros2 daemon start
```

