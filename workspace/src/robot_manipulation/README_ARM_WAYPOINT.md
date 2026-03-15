# Arm Waypoint Server (Standalone, No MoveIt2)

This guide is for `arm_waypoint_server`, a separate ROS2 action server that executes predefined arm waypoints by publishing `trajectory_msgs/JointTrajectory` to `/arm/joint_trajectory_cmd`.

## 1. What This Node Does

- Action type: `robot_interfaces/action/PickArm`
- Default action name: `/pick_arm_waypoint`
- Command selector: `goal.planning_group`
- Supported commands:
  - `pickup` (default if empty)
  - `home`
  - `pregrasp`
  - `grasp`
  - `retreat`
  - `open_gripper`
  - `close_gripper`

## 2. Build

Run from workspace root:

```bash
cd EC463_Team_21_Grocery_Robot/workspace
colcon build --packages-select robot_interfaces robot_manipulation
source install/setup.bash
```

## 3. Run Server

### Option A: Launch server + arm motor bridge together

```bash
ros2 launch robot_manipulation arm_waypoint_launch.py
```

### Option B: Run server only

```bash
ros2 run robot_manipulation arm_waypoint_server \
  --ros-args \
  --params-file EC463_Team_21_Grocery_Robot/workspace/src/robot_manipulation/config/arm_waypoint_server.yaml
```

## 4. Quick Checks

In a new terminal (after `source install/setup.bash`):

```bash
ros2 action list | grep pick_arm_waypoint
ros2 action info /pick_arm_waypoint
ros2 topic echo /arm/joint_trajectory_cmd
```

If action is up, `ros2 action info /pick_arm_waypoint` should show an action server.

## 5. Send Goals from CLI

### Full pickup sequence

```bash
ros2 action send_goal /pick_arm_waypoint robot_interfaces/action/PickArm \
"{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planning_group: 'pickup', ee_link: '', pregrasp_offset_m: 0.0, retreat_offset_m: 0.0, gripper_close_position: 0.0, use_cartesian_approach: false}" \
--feedback
```

### Single waypoint examples

Home:

```bash
ros2 action send_goal /pick_arm_waypoint robot_interfaces/action/PickArm \
"{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planning_group: 'home', ee_link: '', pregrasp_offset_m: 0.0, retreat_offset_m: 0.0, gripper_close_position: -1.0, use_cartesian_approach: false}" \
--feedback
```

Open gripper:

```bash
ros2 action send_goal /pick_arm_waypoint robot_interfaces/action/PickArm \
"{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planning_group: 'open_gripper', ee_link: '', pregrasp_offset_m: 0.0, retreat_offset_m: 0.0, gripper_close_position: -1.0, use_cartesian_approach: false}" \
--feedback
```

Close gripper:

```bash
ros2 action send_goal /pick_arm_waypoint robot_interfaces/action/PickArm \
"{target_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planning_group: 'close_gripper', ee_link: '', pregrasp_offset_m: 0.0, retreat_offset_m: 0.0, gripper_close_position: -1.0, use_cartesian_approach: false}" \
--feedback
```

## 6. Test Checklist

1. Start `arm_waypoint_server` (and `arm_motor` if using real bridge).
2. Confirm action exists: `ros2 action list`.
3. Send `home` goal and verify `/arm/joint_trajectory_cmd` publishes one point.
4. Send `pickup` and verify stage feedback progresses to `completed`.
5. Send invalid command (for example `planning_group: 'abc'`) and verify server returns `abort` with error message.
6. While one goal is active, send another goal and verify it is rejected.

## 7. Common Issues

- Action not found:
  - Check `action_name` in `arm_waypoint_server.yaml` matches your CLI path.
- No arm movement:
  - Verify `arm_motor` is running and subscribed to `/arm/joint_trajectory_cmd`.
- Wrong pose/angles:
  - Tune `waypoint_home/pregrasp/grasp/retreat` in:
    - `workspace/src/robot_manipulation/config/arm_waypoint_server.yaml`

## 8. BT Integration Note

For your BT action client, send `PickArm.Goal` to `/pick_arm_waypoint` and set:

- `planning_group='pickup'` for full sequence
- Or one of `home/pregrasp/grasp/retreat/open_gripper/close_gripper`

Other goal fields are currently ignored by this standalone server except `gripper_close_position` during `pickup`.
