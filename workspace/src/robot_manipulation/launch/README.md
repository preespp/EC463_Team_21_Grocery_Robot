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

Working VX300 MoveIt launch on this machine:

```bash
source /opt/ros/humble/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/interbotix_ws/install/setup.bash
source /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/install/setup.bash
ros2 launch robot_manipulation vx300_moveit.launch.py robot_model:=vx300 motor_port:=/dev/ttyUSB1
```

This launch should open the MoveIt RViz window automatically.

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
