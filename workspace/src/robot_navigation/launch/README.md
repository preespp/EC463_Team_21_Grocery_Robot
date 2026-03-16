# Robot Navigation Launches

This package provides one-line workflows for the commands in
`Nav/README_SLAM_UPDATED.md`.

Current rebuild command:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --symlink-install --packages-select robot_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## One-line stack bringup

Mapping phase (LiDAR + Cartographer + serial bridge):

```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

Mapping phase with D0 quality profile (PointCloud2 path):

```bash
ros2 run robot_navigation nav_assistant mapping-stack \
  --cartographer-config-basename pico_2d_mapping_quality.lua
```

Localization + Nav2 phase:

```bash
cd /home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz true
```

For headless Jetson:

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz false
```

Current stack defaults:

- LiDAR frame `lidar_link` + static transform `base_link -> lidar_link`.
- Raw bridge odom on `/odom_raw`.
- EKF fusion (`robot_localization`) output on `/odom`.
- Localization + Nav2 currently loads `config/nav2_params_smac_mppi_omni.yaml`.
- Global planner is `nav2_smac_planner/SmacPlanner2D`.
- Local controller is `nav2_mppi_controller::MPPIController` with `motion_model: "Omni"`.
- Localization + Nav2 bridges only `["/cmd_vel"]` by default.
- Mapping stack enables a `base_link` crop filter before Cartographer with
  `x=[-0.2540, 0.1397] m`, `y=[-0.2794, 0.2794] m`, `z=[-1.0, 1.0] m`
  (`15.5 x 22 in` rear self-hit filter box).
- Localization + Nav2 stack keeps crop disabled by default unless explicitly enabled.
- Nav2's robot footprint/box remains separate; this crop only cleans the
  Cartographer input path.

If you prefer direct launch usage:

```bash
ros2 launch robot_navigation slam_mapping_stack.launch.py
ros2 launch robot_navigation nav2_localization_stack.launch.py \
  pbstream_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.pbstream \
  map_yaml:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.yaml \
  nav2_params_file:=/home/grocerybot/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml \
  with_nav2_rviz:=true
```

Disable the default mapping crop filter only when needed for debugging:

```bash
ros2 run robot_navigation nav_assistant mapping-stack --with-base-link-crop false
```

Enable the crop manually for localization only if you want to test it:

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-base-link-crop true
```

## Teleop

```bash
ros2 run robot_navigation nav_assistant teleop
ros2 run robot_navigation nav_assistant teleop-collision
```

## Map save/export helpers

```bash
ros2 run robot_navigation nav_assistant save-map --map-name testmapMain
ros2 run robot_navigation nav_assistant export-map --map-name testmapMain
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
ros2 run robot_navigation nav_assistant motion --preset box_loop --topic /cmd_vel
```

One-key macro pad:

```bash
ros2 run robot_navigation nav_assistant motion-pad --topic /cmd_vel
```

Keys: `1..4` run preset combos, `space` stop, `q` quit.

## Runbook output and quick checks

```bash
ros2 run robot_navigation nav_assistant print-runbook --map-name testmapMain
ros2 run robot_navigation nav_assistant quick-check
```
