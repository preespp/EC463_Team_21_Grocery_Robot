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
  - `workspace/src/robot_manipulation/config/vx300s_auto_pick.yaml`
- Arm server behavior:
  - `workspace/src/robot_manipulation/config/viperx_arm_server.yaml`
- Camera mount defaults:
  - `workspace/src/robot_vision/robot_vision/camera_vision.py`
  - default `mount_xyz` is set to `-0.0635,0.0,0.0635`

Launch files now prefer model-specific config filenames when they exist. For VX300S, that means
`vx300s_auto_pick.yaml`, `vx300s_moveit_modes.yaml`, and `vx300s_xsarm_modes.yaml` are used
automatically instead of the older shared `vx300_*` names.

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

## 11) MoveIt Hand-Eye Calibration Notes

This workspace can run the MoveIt hand-eye calibration RViz plugin, but there are two different
camera paths:

- `robot_vision/camera_vision` is the custom RealSense + YOLO node used for auto-pick.
- `realsense2_camera` is the standard ROS RealSense driver and is the safer choice for calibration.

Important difference:

- `camera_vision` publishes `/camera/color/image_raw`, detections, and TF.
- `camera_vision` does **not** publish `sensor_msgs/CameraInfo`.
- The MoveIt calibration plugin expects a normal camera feed with both image and camera info.

### Build the calibration packages

Current status on this machine:

- the calibration packages are already built under `/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/install`
- you do **not** need to rebuild if you are only trying to use the plugin
- only rebuild if you changed the calibration source, cleaned `build/` or `install/`, or want to refresh the package

If the existing build is still present, just source:

```bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/install/setup.bash
```

Use the rebuild command below only when needed.

From the repository root:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot
source /opt/ros/humble/setup.bash
export MAKEFLAGS='-j1 -l1'
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build \
  --base-paths workspace/src \
  --packages-up-to moveit_calibration_gui \
  --cmake-clean-cache \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
```

After it finishes:

```bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/install/setup.bash
```

### Launch MoveIt for calibration

```bash
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/install/setup.bash

ros2 launch robot_manipulation vx300_moveit.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1 use_moveit_rviz:=true
```

### Preferred camera launch for calibration

Do **not** run `camera_vision` and `realsense2_camera` against the same physical RealSense at the
same time.

For calibration, prefer:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true
```

### Camera setup for calibration

Keep this simple:

- for calibration, use the default ROS RealSense node
- do **not** use the custom `camera_vision` node for calibration unless it is extended to publish
  `CameraInfo`

Default launch command:

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true
```

If you want to verify that the image is visible, open:

```bash
ros2 run rqt_image_view rqt_image_view
```

and select the RealSense color image topic.

### RViz plugin workflow

In RViz:

1. Add the display type `moveit_rviz_plugin/HandEyeCalibration`.
2. In the `Target` tab, choose `HandEyeTarget/Charuco`.
3. Select the color image topic.
4. Wait until the target board is detected.
5. In the `Context` tab, use:
   - `Sensor configuration`: `Eye-in-hand`
   - `Sensor frame`: `camera_color_optical_frame`
   - `Object frame`: `handeye_target`
   - `End-effector frame`: `vx300s/ee_gripper_link`
   - `Robot base frame`: `vx300s/base_link`
6. In the `Calibrate` tab:
   - `Planning Group`: `interbotix_arm`
   - `AX=XB Solver`: `OpenCV`
7. Take `8-12` samples from varied poses, then click `Solve`.
8. Save the resulting camera pose and convert it into a static transform publisher in launch.

## 12) Forearm Roll Direction Mismatch

### Symptom

The `forearm_roll` joint in RViz / MoveIt rotates in the opposite direction from the real motor.
If this happens, planning, visualization, and calibration are not trustworthy until the sign is
fixed.

### What the `Drive_Mode` bit means

The low bit of `Drive_Mode` sets the sign convention:

- `0`: CCW is positive
- `1`: CW is positive

For this issue, the important point is simple:

- if the real motion is the exact opposite of the model, `Drive_Mode` usually needs to flip from
  `0` to `1`

This changes **direction**, not rotational range.

### Files involved

Local copy used by `robot_manipulation/vx300_bringup.launch.py`:

- `workspace/src/robot_manipulation/config/vx300s.yaml`

Upstream Interbotix copy used by the standard Interbotix MoveIt path:

- `workspace/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/vx300s.yaml`

Keep the two files in sync if you use both bringup paths.

### Setting to check

For `forearm_roll`, the relevant entry is:

```yaml
forearm_roll:
  ID: 6
  Baud_Rate: 3
  Return_Delay_Time: 0
  Drive_Mode: 0
  Velocity_Limit: 131
  Min_Position_Limit: 0
  Max_Position_Limit: 4095
  Secondary_ID: 255
```

If the joint direction is backwards, change only:

```yaml
Drive_Mode: 1
```

### How to apply the change

`Drive_Mode` is stored in motor EEPROM, so editing the YAML is not enough by itself. One startup
must be done with `load_configs:=true` so the driver writes the value to the motor.

```bash
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash

ros2 launch robot_manipulation vx300_bringup.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1 \
  load_configs:=true
```

After the EEPROM write succeeds, go back to the normal launch flow with `load_configs:=false`.
Test only a very small forearm-roll motion first.

## 13) Stop and Clean Restart

```bash
pkill -INT -f "ros2 launch|ros2 run|rviz2|move_group|viperx_arm_server|vision_auto_pick.py|camera_vision|xs_sdk|ros2_control_node|robot_state_publisher|spawner"
sleep 2
pkill -9 -f "rviz2|move_group|viperx_arm_server|vision_auto_pick.py|camera_vision|xs_sdk|ros2_control_node|robot_state_publisher|spawner"
ros2 daemon stop
sleep 1
ros2 daemon start
```
