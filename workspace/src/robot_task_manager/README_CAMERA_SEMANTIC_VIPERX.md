# Camera + Semantic + ViperX Integration

This document explains how the current ViperX behavior tree uses:

- camera detections from `robot_vision`
- semantic shelf resolution from `robot_task_manager`
- arm/gripper actions through `/pick_viperx`

It also summarizes the important data formats shared across these components.

## 1. High-Level Flow

For the customer ViperX tree in [`customer_viperx.py`](./robot_task_manager/trees/customer_viperx.py):

1. `SetCurrentItem(bb)` picks the next order item and populates basic blackboard state.
2. `ResolveCurrentItemSemanticTargetViperX(bb)` resolves:
   - `bb.nav_goal`
   - `bb.shelf_height`
   - `bb.shelf_pose` from a hardcoded shelf-level command
3. `NavigateToGoalPose(goal_key="nav_goal", bb=bb)` moves the mobile base to the target shelf area.
4. `RepositionViperXArm(goal_key="pose", bb=bb)` moves the arm to an observation preset.
5. `VerifyViperXPosition(bb=bb)` listens to `/detections_json`, finds the expected product, and transforms the detection into `vx300/base_link`.
6. `MoveToDetectedPose(bb=bb)` moves the arm to that live detected object pose.
7. `MoveViperXGripper("close")` grabs the object.
8. `SelectBasketSlot(bb=bb)` chooses a hardcoded basket slot preset.
9. `RepositionViperXArm(goal_key="basket_pose", bb=bb)` moves to that basket preset.
10. `MoveViperXGripper("open")` releases the object.

## 2. Semantic Resolution

The ViperX semantic node is [`ResolveCurrentItemSemanticTargetViperX`](./robot_task_manager/bt_nodes/semantic_nodes.py).

It uses the semantic service `/semantic_map/resolve_target` and writes:

- `bb.nav_goal`: navigation target `(x, y, yaw)`
- `bb.shelf_height`: semantic rack level, usually `1`, `2`, or `3`
- `bb.shelf_pose`: selected from `bb.shelf_poses[rack_level]`
- semantic metadata:
  - `bb.slot_id`
  - `bb.anchor_id`
  - `bb.rack_id`
  - `bb.semantic_id`
  - `bb.semantic_target_label`
  - `bb.nav_goal_source`

Important behavior:

- The ViperX version does not use `rack_goal` for shelf movement anymore.
- Instead, shelf level selects a preset shelf command such as:
  - `shelf_level_1_pose`
  - `shelf_level_2_pose`
  - `shelf_level_3_pose`

## 3. Camera Detection Flow

The camera node is [`camera_vision.py`](../robot_vision/robot_vision/camera_vision.py).

It publishes a JSON string on `/detections_json`.

Each detection contains:

```json
{
  "class_id": 39,
  "class_name": "bottle",
  "confidence": 0.88,
  "bbox": [100, 120, 220, 340],
  "center_px": [160, 230],
  "distance_m": 0.36,
  "surface_distance_m": 0.34,
  "point_camera_optical_m": [0.10, 0.05, 0.36],
  "grasp_px": [160, 230]
}
```

The payload also includes:

```json
{
  "timestamp": 1710000000.0,
  "camera_optical_frame": "camera_color_optical_frame",
  "camera_mount_frame": "camera_mount_frame",
  "imu_rpy_rad": [0.0, 0.0, 0.0],
  "detections": [...]
}
```

Important frame note:

- `point_camera_optical_m` is in the camera optical frame, not the navigation map.
- In the BT, [`VerifyViperXPosition`](./robot_task_manager/bt_nodes/viperx_nodes.py) transforms that point into `vx300/base_link` before saving it to `bb.detected_object_pose`.

## 4. Blackboard Data Formats

The blackboard setup is in [`blackboard.py`](./robot_task_manager/blackboard.py).

### 4.1 Preset Arm States

Preset states are stored as command dictionaries:

```python
bb.pose = {"command": "startup_arm_pose"}
bb.home_pose = {"command": "return_arm_pose"}
bb.basket_pose = {"command": "place_arm_pose"}
bb.shelf_pose = {"command": "shelf_level_2_pose"}
```

These are consumed by [`RepositionViperXArm`](./robot_task_manager/bt_nodes/viperx_nodes.py), which forwards the command name to `/pick_viperx` as `PickArm.Goal.planning_group`.

### 4.2 Shelf Level Mapping

Shelf presets are stored in:

```python
bb.shelf_poses = {
    1: {"command": "shelf_level_1_pose"},
    2: {"command": "shelf_level_2_pose"},
    3: {"command": "shelf_level_3_pose"},
}
```

`ResolveCurrentItemSemanticTargetViperX` picks the correct `bb.shelf_pose` from this mapping.

### 4.3 Vision-Detected Cartesian Target

Live detection results are stored as Cartesian pose dictionaries:

```python
bb.detected_object_pose = {
    "frame_id": "vx300/base_link",
    "x": 0.25,
    "y": 0.03,
    "z": 0.18,
    "source_frame": "camera_color_optical_frame",
    "distance_m": 0.31,
    "class_name": "bottle",
    "confidence": 0.87,
}
```

This format is consumed by [`MoveToDetectedPose`](./robot_task_manager/bt_nodes/viperx_nodes.py).

### 4.4 Navigation Goal

Navigation uses:

```python
bb.nav_goal = (x, y, yaw)
```

This is consumed by [`NavigateToGoalPose`](./robot_task_manager/bt_nodes/navigation_nodes.py).

## 5. ViperX Action Format

The arm action is [`PickArm.action`](../robot_interfaces/action/PickArm.action).

Goal format:

```text
geometry_msgs/PoseStamped target_pose
string planning_group
string ee_link
float32 pregrasp_offset_m
float32 retreat_offset_m
float32 gripper_close_position
bool use_cartesian_approach
```

There are two main usage patterns.

### 5.1 Preset Joint Command

Used by `RepositionViperXArm` when blackboard value is `{"command": ...}`:

```text
planning_group = "startup_arm_pose"
target_pose.header.frame_id = "vx300/base_link"
```

The arm server maps that command to configured joint arrays.

### 5.2 Cartesian Pose Goal

Used by `MoveToDetectedPose`:

```text
planning_group = "arm"
target_pose = PoseStamped(frame_id="vx300/base_link", ...)
```

This is how the BT moves to the object detected by vision.

## 6. BT Preset State Config

The BT-side preset names are documented in:

- [`config/viperx_bt_states.yaml`](./config/viperx_bt_states.yaml)

That file is a template for command names only.

The actual raw joint arrays must be configured in:

- [`robot_manipulation/config/viperx_arm_server.yaml`](../robot_manipulation/config/viperx_arm_server.yaml)

If you add a new command name like `shelf_level_2_pose`, the arm server must also be updated to recognize it.

## 7. Important Distinction Between Two Arm Nodes

### `RepositionViperXArm`

Use this for:

- observation pose
- basket pose
- home pose
- hardcoded shelf poses

It is the generic arm movement node for BT-controlled preset states or planned pose targets.

### `MoveToDetectedPose`

Use this for:

- the live object location from camera detection

It specifically reads `bb.detected_object_pose` and moves to the transformed detection point.

## 8. Current Limitation

The BT now assumes shelf approach is command-based, not z-height-based.

That means these command names must eventually exist in the arm server:

- `shelf_level_1_pose`
- `shelf_level_2_pose`
- `shelf_level_3_pose`

Until those are added to the manipulation server/config, `bb.shelf_pose` will have the correct BT format but the action server will not yet know how to execute those commands.
