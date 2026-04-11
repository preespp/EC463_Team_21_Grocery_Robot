# robot_navigation

`robot_navigation` is the mobile-base navigation package for the grocery robot. It provides LiDAR bringup, Cartographer mapping/localization, Nav2 integration, STM32 serial bridging, teleoperation, map save/export helpers, semantic map serving, and CLI shortcuts through `nav_assistant`.

## What This Package Is

This package is responsible for:

- LiDAR and IMU launch integration
- Cartographer mapping and localization
- Nav2 localization and planning stack
- serial bridge from ROS `/cmd_vel` topics to the base controller
- raw odometry and optional EKF fusion
- semantic map service support for higher-level task planning
- teleop, goal sending, waypoint sending, and motion macros

## Tech Stack

- ROS 2 Humble
- Python ROS 2 nodes with `rclpy`
- Cartographer
- Nav2
- `robot_localization`
- TF2
- serial bridge tooling for the base controller
- shared interfaces from `robot_interfaces`

## Main Components

- `robot_navigation/nav_assistant.py`: one-command CLI helper
- `robot_navigation/nav2_serial_bridge.py`: `/cmd_vel` to STM32 bridge and `/odom_raw` publisher
- `launch/slam_mapping_stack.launch.py`: mapping stack
- `launch/nav2_localization_stack.launch.py`: localization + Nav2 + semantic map stack
- `launch/cartographer_mapping.launch.py` and `launch/cartographer_localization.launch.py`: Cartographer-only pieces

## Build

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_navigation
source install/setup.bash
```

## Main Command Entry Point

```bash
ros2 run robot_navigation nav_assistant <subcommand>
```

## Common Workflows

### 1. Run Mapping

```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

Use the higher-quality profile if needed:

```bash
ros2 run robot_navigation nav_assistant mapping-stack \
  --cartographer-config-basename pico_2d_mapping_quality.lua
```

Optional extras:

```bash
ros2 run robot_navigation nav_assistant mapping-stack --with-rviz true --with-collision true
```

### 2. Teleoperate The Robot

```bash
ros2 run robot_navigation nav_assistant teleop
ros2 run robot_navigation nav_assistant teleop-collision
```

### 3. Save And Export A Map

```bash
ros2 run robot_navigation nav_assistant save-map --map-name testmapMain
ros2 run robot_navigation nav_assistant export-map --map-name testmapMain
```

### 4. Run Localization + Nav2

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz true
```

For headless systems:

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmapMain --with-nav2-rviz false
```

### 5. Send Goals Or Waypoints

```bash
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

```bash
ros2 run robot_navigation nav_assistant waypoints \
  --pose 1.0,0.0,0.0 \
  --pose 1.5,0.5,0.0 \
  --pose 0.5,1.0,0.0
```

### 6. Run Motion Macros

```bash
ros2 run robot_navigation nav_assistant motion --preset box_loop --topic /cmd_vel
ros2 run robot_navigation nav_assistant motion-pad --topic /cmd_vel
```

### 7. Direct Launch Usage

```bash
ros2 launch robot_navigation slam_mapping_stack.launch.py
```

```bash
ros2 launch robot_navigation nav2_localization_stack.launch.py \
  pbstream_file:=/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.pbstream \
  map_yaml:=/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/Maps/testmapMain.yaml \
  nav2_params_file:=/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_navigation/config/nav2_params_smac_mppi_omni.yaml \
  with_nav2_rviz:=true
```

## Current Defaults

- LiDAR frame: `lidar_link`
- IMU frame: `imu_link`
- raw bridge odom: `/odom_raw`
- EKF output: `/odom`
- localization command topics bridged by default: `["/cmd_vel"]`
- current Nav2 params file: `config/nav2_params_smac_mppi_omni.yaml`
- current global planner: `nav2_smac_planner/SmacPlanner2D`
- current local controller: `nav2_mppi_controller::MPPIController`

## Collision Detection Integration

If ultrasonic collision detection is connected, run:

```bash
ros2 launch robot_perception ultrasonic_launch.py
```

If the hardware is not connected and you need to fake safe topics during testing:

```bash
ros2 topic pub -r 100 /front_alert std_msgs/msg/Bool "{data: false}"
ros2 topic pub -r 100 /left_alert std_msgs/msg/Bool "{data: false}"
ros2 topic pub -r 100 /right_alert std_msgs/msg/Bool "{data: false}"
ros2 topic pub -r 100 /back_alert std_msgs/msg/Bool "{data: false}"
```

## Map Output

By default, maps are saved to the repo-level `Maps/` folder as:

- `<map_name>.pbstream`
- `<map_name>.yaml`
- `<map_name>.pgm`

## Helpful Checks

```bash
ros2 run robot_navigation nav_assistant print-runbook --map-name testmapMain
ros2 run robot_navigation nav_assistant quick-check
```

## Notes

- Disable EKF only for troubleshooting.
- Mapping enables the base-link crop filter by default; localization keeps it off unless explicitly enabled.
- This package is the navigation side of the current production stack, not the old MVP prototype flow.
