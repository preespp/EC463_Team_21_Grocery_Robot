# Launch

`arm_launch.py`
Launches the original manipulation stack that pairs the MoveIt action server with the legacy I2C arm bridge.

`rack_launch.py`
Launches the rack subsystem.

For testing each functionality or node, different configuration or integration of each node might be different. After finishing, each launch file, edit this file and explain each functionality.

`arm_waypoint_launch.py`:
- Standalone arm waypoint action server (`/pick_arm_waypoint`) + arm motor bridge launcher.
- Full run/test/CLI guide: `workspace/src/robot_manipulation/README_ARM_WAYPOINT.md`
`vx300_bringup.launch.py`
Launches a local Interbotix VX300 or VX300S bringup using the official motor configs, local URDF/xacro files, and `interbotix_xs_sdk`.

`vx300_quick_move.launch.py`
Starts the VX300 bringup and runs a short demo motion after startup. Use `use_sim:=true` first to verify the stack without moving hardware.

`vx300_moveit.launch.py`
Launches the real VX300 or VX300S with the official Interbotix MoveIt stack, ros2_control trajectory controllers, and the MoveIt RViz window.
Supports opt-in vision auto-pick via `use_auto_pick:=true`.

`vx300_auto_pick.launch.py`
Launches `vx300_moveit.launch.py` with auto-pick enabled by default.
Optionally starts `robot_vision/camera_vision` with `launch_camera_vision:=true`.
Full handoff and operation guide: `workspace/src/robot_manipulation/README_VX300_CAMERA_AUTOPICK.md`

`viperx_arm_server.launch.py`
Launches a BT-facing action server (`/pick_viperx`) that accepts `robot_interfaces/action/PickArm` goals, executes ViperX end-effector pose goals through MoveIt, and handles `open_gripper` / `close_gripper` commands.

Working VX300S MoveIt launch on this machine:

```bash
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 launch robot_manipulation vx300_moveit.launch.py robot_model:=vx300s motor_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0
```

This launch should open the MoveIt RViz window automatically.
It now also starts `viperx_arm_server` by default (`use_viperx_arm_server:=true`), exposing `/pick_viperx` for BT action clients.

## Camera-Triggered Auto Pick (Detection -> Pick)

1. Start MoveIt + ViperX action server + auto-pick bridge:

```bash
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 launch robot_manipulation vx300_auto_pick.launch.py robot_model:=vx300s motor_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0
```

2. If `camera_vision` is not already running, launch it in another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 run robot_vision camera_vision --ros-args \
  -p parent_frame:=vx300s/ee_gripper_link \
  -p camera_mount_frame:=camera_mount_frame \
  -p camera_optical_frame:=camera_color_optical_frame
```

3. Optional safety/debug mode first (`dry_run=true`, no arm motion):

```bash
ros2 launch robot_manipulation vx300_moveit.launch.py robot_model:=vx300s motor_port:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YTG6-if00-port0 use_auto_pick:=true \
  auto_pick_config:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_manipulation/config/vx300_auto_pick.yaml
```

Then edit `/workspace/src/robot_manipulation/config/vx300_auto_pick.yaml` and set:
- `dry_run: true`
- `target_class: "bottle"` (or your product class)

When stable detections arrive on `/detections_json`, `vision_auto_pick` sends a safe 4-stage sequence:
`open_gripper -> move_pregrasp -> move_grasp_and_close -> lift`.

## Git Notes

The folders below are local upstream Interbotix clones used by the VX300 launch files:

- `workspace/interbotix_ws/src/interbotix_ros_core`
- `workspace/interbotix_ws/src/interbotix_ros_manipulators`
- `workspace/interbotix_ws/src/interbotix_ros_toolboxes`

They are separate Git repositories, not normal folders inside the main
`EC463_Team_21_Grocery_Robot` repository. By default, the main project should
track only `workspace/src/robot_manipulation` and keep the Interbotix repos as
local dependencies.

Recommended workflow for committing this package only:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot
git restore --staged workspace/interbotix_ws/src/interbotix_ros_core \
                    workspace/interbotix_ws/src/interbotix_ros_manipulators \
                    workspace/interbotix_ws/src/interbotix_ros_toolboxes
git add .gitignore
git add workspace/src/robot_manipulation
git status
```

The root `.gitignore` should keep these three local repos ignored so `git add *`
does not accidentally stage them in the outer repository.

If the team ever wants the main repo to own the Interbotix files directly, first
move each nested `.git` directory somewhere safe, remove the ignore rules, and
then `git add` those folders as normal directories. This vendors the upstream
code into the main project and makes future upstream updates more manual.
