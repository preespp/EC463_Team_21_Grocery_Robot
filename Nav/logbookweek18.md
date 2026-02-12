# Logbook Week 18 - Nav2 + SLAM Integration Session

Date: 2026-02-10  
Repo: `EC463_Team_21_Grocery_Robot`  
Focus: Validate end-to-end SLAM/localization/navigation workflow and fix bringup confusion points.

## Session Goals

- Verify ROS2 command path to base controller (not just raw UART teleop).
- Confirm SLAM-updated workflow works with Cartographer + Nav2.
- Make sure saved map is actually loaded into Nav2.
- Document exact working commands in project README.

## Important Findings

1. `uart2_diag.py` is a direct UART diagnostic tool and does not validate ROS topic flow by itself.
2. SICK driver warnings about remap syntax are mostly non-blocking; the critical failures were SOPAS write errors and no UDP scan data received.
3. The command:
   - `ros2 launch nav2_bringup navigation_launch.py ... map:=...`
   did not load a map because `navigation_launch.py` does not start `map_server`.
4. Root cause of "no map in Nav2": `/map` had no publisher until `map_server` and its lifecycle manager were launched explicitly.

## Working Bringup Sequence Verified

1. Start LiDAR driver (`sick_scan_xd`) with project topic/frame overrides.
2. Start Cartographer localization using saved `.pbstream`.
3. Start `map_server` with saved YAML map.
4. Activate `map_server` lifecycle.
5. Launch Nav2 stack with `nav2_params_cartographer.yaml`.
6. Launch Nav2 RViz and send goals.

## Commands Added/Validated

Map server:

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p use_sim_time:=false \
  -p yaml_filename:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml
```

Lifecycle activation:

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=false \
  -p autostart:=true \
  -p node_names:='["map_server"]'
```

Nav2 stack:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Nav/nav2_params_cartographer.yaml
```

Single goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

Waypoint mission:

```bash
ros2 action send_goal /follow_waypoints nav2_msgs/action/FollowWaypoints \
  "{poses: [
    {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}},
    {header: {frame_id: map}, pose: {position: {x: 0.5, y: 1.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}
  ]}"
```

Verification checks:

```bash
ros2 topic info /map
ros2 topic echo /map_metadata --once
ros2 topic hz /cmd_vel
ros2 action list | grep -E 'navigate_to_pose|follow_waypoints'
```

## What Was Accomplished Today

- Confirmed drivetrain responds correctly in verified stages.
- Clarified Cartographer vs Nav2 map server responsibilities.
- Identified and fixed workflow gap causing missing static map in Nav2.
- Updated `Nav/README_SLAM_UPDATED.md` with:
  - explicit `map_server` + lifecycle steps,
  - corrected Nav2 bringup expectations,
  - waypoint/goal command section,
  - quick checks and pitfalls.
- Established a reproducible command set for both single-goal and waypoint navigation tests.

## Next Recommended Validation

1. Run a full cold-start test from terminal setup to waypoint completion.
2. Record one successful `/follow_waypoints` run with timestamps and command logs.
3. Save final RViz/Nav2 screenshots and attach to future weekly logbook entries.
