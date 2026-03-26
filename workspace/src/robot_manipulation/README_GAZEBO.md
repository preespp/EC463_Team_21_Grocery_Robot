# All zeros – home position
Publish data to reset
ros2 topic pub /arm/joint_state std_msgs/msg/Float32MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0]}" -r 1

Publish data (FK)
# Base yaw +45°, shoulder 30°, elbow 60°, wrist roll -45°, gripper half-closed
# (numbers in radians)
ros2 topic pub /arm/joint_state std_msgs/msg/Float32MultiArray \
  "{data: [0.7854, 0.5236, 1.0472, -0.7854, 0.4]}" -r 1

Check Sanity of URDF File
check_urdf src/mvp_robot/urdf/five_dof_arm.urdf

Run Gazebo with world file we have
ign gazebo shapes.sdf  # Fortress uses "ign gazebo" instead of "gz sim"

Set Up Command
cd ~/Desktop/EC463_Team_21_Grocery_Robot/workspace
source install/setup.bash

Terminal 1 Gazebo
ign gazebo

Terminal 2 Spawn URDF (World must be run before spawn)
ros2 run ros_gz_sim create   -world empty   -file $(ros2 pkg prefix mvp_robot)/share/mvp_robot/urdf/five_dof_arm.urdf   -name five_dof_arm   -x 0 -y 0 -z 0.1

Terminal 3 ROS2-Gazebo Bridge (Change config file)
ros2 run ros_gz_bridge parameter_bridge   --ros-args   -p config_file:=$(ros2 pkg prefix mvp_robot)/share/mvp_robot/config/arm_bridge.yaml

Terminal 4 Run ROS2 node to convert normal topic to topic that put into bridge
ros2 run mvp_robot arm_to_gazebo

Terminal 5 run normal ROS2 Node we want in real hardware
ros2 run mvp_robot arm_controller

