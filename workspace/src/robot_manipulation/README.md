# robot_manipulation

`robot_manipulation` is the robot arm and rack package. It contains the ViperX / VX300 manipulation stack, MoveIt integration, BT-facing arm action servers, simulation helpers, rack control, and supporting launch/config files.

## What This Package Is

This package is responsible for:

- bringing up the Interbotix VX300 or VX300S arm
- exposing the BT-facing `/pick_viperx` action server
- running MoveIt-based manipulation
- running optional camera-driven auto-pick flows
- hosting older standalone arm waypoint and Gazebo test workflows
- controlling rack-related subsystems

## Tech Stack

- ROS 2 Humble
- C++ nodes with `rclcpp` and `rclcpp_action`
- MoveIt 2
- Interbotix XS arm stack
- TF2
- optional Gazebo / `ros_gz`
- shared `robot_interfaces` actions

## Main Executables And Roles

- `viperx_arm_server`: main BT-facing arm action server on `/pick_viperx`
- `vision_auto_pick.py`: listens to vision detections and sends pick goals
- `arm_waypoint_server`: older standalone waypoint-based action server
- `arm_controller`, `arm_demo_controller`, `arm_to_gazebo`, `rack_controller`: older or specialized test/control nodes

## Key Launch Files

- `vx300_bringup.launch.py`: bring up real VX300 / VX300S hardware
- `vx300_moveit.launch.py`: MoveIt + controllers + optional `/pick_viperx`
- `vx300_auto_pick.launch.py`: MoveIt plus auto-pick flow, optionally with `camera_vision`
- `vx300_auto_pick_measured_tf.launch.py`: measured eye-in-hand TF version of auto-pick
- `viperx_arm_server.launch.py`: launch only the BT-facing arm action server stack
- `arm_waypoint_launch.py`: older standalone waypoint server flow
- `arm_sim_launch.py`: older Gazebo arm simulation flow
- `assembly_arm_sim_launch.py`: CAD assembly arm simulation flow
- `rack_launch.py`: rack subsystem

## Important Config And Asset Files

- `config/viperx_arm_server.yaml`: preset commands, place pose, return pose, gripper behavior
- `config/vx300_auto_pick.yaml` and `config/vx300s_auto_pick.yaml`: auto-pick behavior
- `config/vx300s_moveit_modes.yaml`: model-specific hardware mode settings
- `meshes/`: CAD and robot meshes referenced by URDF / Xacro
- `urdf/`: arm and simulation descriptions

## Build

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
colcon build --base-paths src --packages-select robot_interfaces robot_manipulation robot_vision
source install/setup.bash
```

## Environment

Use this environment in terminals that run the ViperX stack:

```bash
source /opt/ros/humble/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
```

## Common Workflows

### 1. Bring Up MoveIt And `/pick_viperx`

```bash
ros2 launch robot_manipulation vx300_moveit.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1
```

This is the main launch for BT-driven arm control.

### 2. Run Auto-Pick With The Current Default Camera Calibration

```bash
ros2 launch robot_manipulation vx300_auto_pick.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1 \
  launch_camera_vision:=true
```

### 3. Run Auto-Pick With Measured Camera TF

```bash
ros2 launch robot_manipulation vx300_auto_pick_measured_tf.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1
```

### 4. Preview-Only Validation Without Hardware Motion

```bash
ros2 launch robot_manipulation vx300_auto_pick_measured_tf.launch.py \
  robot_model:=vx300s robot_name:=vx300s motor_port:=/dev/ttyUSB1 \
  use_viperx_preview:=true
```

### 5. Run The Older Standalone Waypoint Server

```bash
ros2 launch robot_manipulation arm_waypoint_launch.py
```

Send a test goal:

```bash
ros2 action send_goal /pick_arm_waypoint robot_interfaces/action/PickArm \
"{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planning_group: 'home', ee_link: '', pregrasp_offset_m: 0.0, retreat_offset_m: 0.0, gripper_close_position: -1.0, use_cartesian_approach: false}" \
--feedback
```

### 6. Run The CAD Assembly Arm Simulation

Build example:

```bash
cd ~/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_manipulation --cmake-force-configure --cmake-args \
  -DBUILD_ROBOT_MANIPULATION_NODES=ON \
  -DBUILD_ARM_MOTOR_NODE=OFF \
  -DBUILD_ARM_CONTROLLER_NODE=OFF \
  -DBUILD_ARM_DEMO_CONTROLLER_NODE=ON \
  -DBUILD_ARM_TO_GAZEBO_NODE=ON \
  -DBUILD_ARM_WAYPOINT_SERVER_NODE=OFF \
  -DBUILD_RACK_CONTROLLER_NODE=OFF
source install/setup.bash
```

Then:

```bash
ros2 launch robot_manipulation assembly_arm_sim_launch.py run_arm_controller:=false run_demo_controller:=true
```

## Camera Auto-Pick Notes

- `camera_vision` publishes `/detections_json`
- `vision_auto_pick.py` transforms detections into the arm base frame
- `/pick_viperx` accepts `robot_interfaces/action/PickArm` goals
- model-specific auto-pick configs are preferred when present, such as `vx300s_auto_pick.yaml`

## Simulation And Mesh Notes

- `meshes/` contains the CAD mesh files used by the URDF and Gazebo flows
- the CAD assembly arm expects filenames such as `base_link.stl`, `inner-ring_1.stl`, `outer-ring_1.stl`, and the other checked-in mesh assets

## Git Notes

The Interbotix folders in `workspace/interbotix_ws/src/` are local upstream dependencies and separate repositories. When committing manipulation changes in the main project repo, stage `workspace/src/robot_manipulation` and avoid accidentally staging the nested Interbotix repos.
