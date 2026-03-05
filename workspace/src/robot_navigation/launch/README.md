# Robot Navigation Launches

This package provides one-line workflows for the commands in
`Nav/README_SLAM_UPDATED.md`.

## One-line stack bringup

Mapping phase (LiDAR + Cartographer + serial bridge):

```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

Localization + Nav2 phase:

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1
```

P1 mission (fixed-loop auto mapping + return + save/export):

```bash
ros2 run robot_navigation nav_assistant mission-p1
ros2 run robot_navigation nav_assistant mission-p1 --interactive-override true
```

Frontier mission (auto explore + return + save/export):

```bash
ros2 run robot_navigation nav_assistant frontier-mission
ros2 run robot_navigation nav_assistant frontier-mission --interactive-override true
```

For headless Jetson:

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1 --with-nav2-rviz false
```

Both stacks now default to:

- LiDAR frame `lidar_link` + static transform `base_link -> lidar_link`.
- `cmd_vel_arbiter`: manual `/cmd_vel_manual` + auto `/cmd_vel_auto,/cmd_vel_nav,/cmd_vel_smoothed` -> `/cmd_vel`.
- Raw bridge odom on `/odom_raw`.
- EKF fusion (`robot_localization`) output on `/odom`.

If you prefer direct launch usage:

```bash
ros2 launch robot_navigation slam_mapping_stack.launch.py
ros2 launch robot_navigation nav2_localization_stack.launch.py \
  pbstream_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.pbstream \
  map_yaml:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmap1.yaml
```

## Teleop

```bash
ros2 run robot_navigation nav_assistant teleop
ros2 run robot_navigation nav_assistant teleop-collision
ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: false}"
ros2 service call /finish_mapping std_srvs/srv/Trigger "{}"
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
ros2 service call /frontier_explorer/stop std_srvs/srv/Trigger "{}"
```

## Map save/export helpers

```bash
ros2 run robot_navigation nav_assistant save-map --map-name testmap1
ros2 run robot_navigation nav_assistant export-map --map-name testmap1
```

## Nav2 goals and waypoints

```bash
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
ros2 run robot_navigation nav_assistant waypoints \
  --pose 1.0,0.0,0.0 \
  --pose 1.5,0.5,0.0 \
  --pose 0.5,1.0,0.0
```

## Motion macros

Run one preset:

```bash
ros2 run robot_navigation nav_assistant motion --preset box_loop --topic /cmd_vel_manual
```

One-key macro pad:

```bash
ros2 run robot_navigation nav_assistant motion-pad --topic /cmd_vel_manual
```

Keys: `1..4` run preset combos, `space` stop, `q` quit.

## Runbook output and quick checks

```bash
ros2 run robot_navigation nav_assistant print-runbook --map-name testmap1
ros2 run robot_navigation nav_assistant quick-check
```
