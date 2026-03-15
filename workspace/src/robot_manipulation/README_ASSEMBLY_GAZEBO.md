# Assembly Arm Gazebo Run Guide (Short)

This guide runs the new CAD arm:
`urdf/assembly_robotic_arm_URDF.xacro`

## 1) Build (once after code changes)

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

## 2) Terminal A: start Gazebo

```bash
cd ~/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix robot_manipulation)/share
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
```

If GUI rendering warnings are noisy, run headless server:

```bash
source /opt/ros/humble/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r -s empty.sdf"
```

## 3) Terminal B: spawn assembly arm

```bash
cd ~/Desktop/EC463_Team_21_Grocery_Robot/workspace
source install/setup.bash
sleep 2
ign service -l | grep '/world/.*/create'
xacro src/robot_manipulation/urdf/assembly_robotic_arm_URDF.xacro > /tmp/assembly_robotic_arm.urdf
ros2 run ros_gz_sim create -world empty -name assembly_robotic_arm -allow_renaming true -z 0.8 -file /tmp/assembly_robotic_arm.urdf
```

Verify spawn:

```bash
source /opt/ros/humble/setup.bash
ign model --list | grep assembly_robotic_arm
```

If base orientation is wrong, respawn with roll / pitch / yaw:

```bash
ros2 run ros_gz_sim create -world empty -name assembly_robotic_arm -allow_renaming true \
  -x 0 -y 0 -z 0.8 -R 1.5708 -P 0 -Y 0 -file /tmp/assembly_robotic_arm.urdf
```

Try `-R -1.5708` if needed.

## 4) Terminal C: start ROS bridge + demo controller

```bash
cd ~/Desktop/EC463_Team_21_Grocery_Robot/workspace
source install/setup.bash
ros2 launch robot_manipulation assembly_arm_sim_launch.py run_arm_controller:=false run_demo_controller:=true
```

## 5) Terminal D: send one demo pose goal

Short output (no feedback spam):

```bash
cd ~/Desktop/EC463_Team_21_Grocery_Robot/workspace
source install/setup.bash
ros2 action send_goal /pick_arm_demo robot_interfaces/action/PickArm \
'{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.05, z: 0.22}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planning_group: "", ee_link: "", pregrasp_offset_m: 0.08, retreat_offset_m: 0.10, gripper_close_position: -1.0, use_cartesian_approach: false}'
```

Verbose feedback mode:

```bash
ros2 action send_goal /pick_arm_demo robot_interfaces/action/PickArm '<same_goal_json>' --feedback
```

## Quick checks

- Check bridge topics:

```bash
ros2 topic list | grep '^/arm/joint[123]_cmd$'
```

- Check meshes exist (CAD STL files):

```bash
ls ~/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_manipulation/meshes/*.stl | wc -l
```

Expected for this CAD xacro: `10`.

## Note about linkage separation

This CAD URDF is exported in a way that can visually separate links for larger joint motion.
Use the demo controller's limited joint range in `config/arm_demo_controller_assembly.yaml` for stable showcase movement.
