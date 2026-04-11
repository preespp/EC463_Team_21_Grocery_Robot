# robot_vision

`robot_vision` contains the camera and detection stack for the grocery robot. It provides RealSense-based object perception, barcode utilities, and published detection data used by the manipulation and task-management packages.

## What This Package Is

This package is responsible for:

- running the main `camera_vision` node used by the ViperX stack
- publishing object detections on `/detections_json`
- publishing camera-related TF frames used by the arm and BT flows
- hosting an additional `realsense_combination` node for alternate payload/image publishing
- providing barcode-related utilities

## Tech Stack

- ROS 2 Humble
- Python ROS 2 nodes with `rclpy`
- Intel RealSense via `pyrealsense2`
- OpenCV and `cv_bridge`
- YOLO / Ultralytics object detection
- TF2 broadcasting

## Main Executables

- `camera_vision`: main production camera + detection node
- `realsense_combination`: alternate RealSense pipeline node
- `bar_code`: barcode utility node

## Main Launch Files

- `launch/realsense_combination.launch.py`: launch wrapper for the `realsense_combination` node

## Build

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_vision
source install/setup.bash
```

## How To Run

### 1. Run The Main Camera Node

```bash
ros2 run robot_vision camera_vision --ros-args \
  -p parent_frame:=vx300s/ee_gripper_link \
  -p camera_mount_frame:=camera_mount_frame \
  -p camera_optical_frame:=camera_color_optical_frame
```

Enable the live preview window:

```bash
ros2 run robot_vision camera_vision --ros-args \
  -p show_live_window:=true \
  -p parent_frame:=vx300s/ee_gripper_link \
  -p camera_mount_frame:=camera_mount_frame \
  -p camera_optical_frame:=camera_color_optical_frame
```

### 2. Run The Alternate RealSense Combination Node

```bash
ros2 launch robot_vision realsense_combination.launch.py
```

Or directly:

```bash
ros2 run robot_vision realsense_combination
```

### 3. Run The Barcode Utility

```bash
ros2 run robot_vision bar_code
```

## Main Published Data

`camera_vision` publishes a JSON string on `/detections_json`.

Each detection includes fields such as:

- `class_id`
- `class_name`
- `confidence`
- `bbox`
- `center_px`
- `distance_m`
- `surface_distance_m`
- `point_camera_optical_m`
- `grasp_px`

The payload also includes frame metadata such as:

- `camera_optical_frame`
- `camera_mount_frame`
- `timestamp`

## Integration Notes

- `robot_task_manager` consumes `/detections_json` during the ViperX BT flow.
- `robot_manipulation` can use the detections for auto-pick and frame transformation into the arm base.
- The main ViperX camera parent frame is usually `vx300s/ee_gripper_link`.

## Notes

- The package expects the RealSense and vision dependencies to be installed on the machine.
- `camera_vision` is the node used by the current production ViperX stack.
