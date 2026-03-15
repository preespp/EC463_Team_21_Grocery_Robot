# Launch

`arm_launch.py`
Launches the original manipulation stack that pairs the MoveIt action server with the legacy I2C arm bridge.

`rack_launch.py`
Launches the rack subsystem.

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
