# robot_navigation

ROS 2 package for Team 21 navigation workflow (SICK PicoScan + Cartographer + Nav2 + STM32 serial bridge).

This package is designed so users can run the full workflow with short commands instead of manually typing long runbook commands.

## What this package includes

- One-line stack launch for mapping phase.
- One-line stack launch for localization + Nav2 phase.
- Keyboard teleop wrappers.
- Map save/export helpers.
- Nav2 goal and waypoint helpers.
- Motion preset macros and one-key motion pad.

Main command entrypoint:

```bash
ros2 run robot_navigation nav_assistant <subcommand>
```

## Folder overview

- `robot_navigation/nav_assistant.py`: unified CLI helper.
- `robot_navigation/cmd_vel_arbiter.py`: manual-first cmd_vel arbiter with `/manual_override`.
- `robot_navigation/mission_orchestrator.py`: P1 mission state machine (`BOOT->AUTO_MAP_V1->RETURN_HOME->SAVE_EXPORT->LOCALIZE_READY`).
- `robot_navigation/frontier_explorer.py`: P2 frontier exploration core.
- `robot_navigation/semantic_overlay.py`: P4 semantic shelf overlay and query services.
- `robot_navigation/nav_profile_report.py`: P5 profile comparison report tool.
- `robot_navigation/nav2_serial_bridge.py`: `/cmd_vel` to STM32 UART bridge and raw odom publisher (`/odom_raw` by default).
- `robot_navigation/teleop_cmd_vel.py`: manual keyboard teleop.
- `robot_navigation/teleop_cmd_vel_collision.py`: teleop with collision stop input.
- `launch/slam_mapping_stack.launch.py`: mapping stack launch.
- `launch/nav2_localization_stack.launch.py`: localization + Nav2 stack launch.
- `config/pico_2d.lua`, `config/pico_2d_localization.lua`: Cartographer configs.
- `config/nav2_params_cartographer.yaml`: Nav2 parameter set.

## Quick start

Assuming ROS 2 Humble and dependencies are installed:

```bash
cd <repo_root>/workspace
colcon build --symlink-install --packages-select robot_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 0. Running Collision Detection (Required hardware setup with 4 directions)

Run this command in separated terminal

```bash
ros2 launch robot_perception ultrasonic_launch.py
```

#### If not integrate collision detection

Run this command in 4 separated terminals to set flag as 0 (1 is object detected in range)

```bash
ros2 topic pub -r 100 /right_alert std_msgs/msg/Bool "{data: false}"
```

```bash
ros2 topic pub -r 100 /front_alert std_msgs/msg/Bool "{data: false}"
```

```bash
ros2 topic pub -r 100 /left_alert std_msgs/msg/Bool "{data: false}"
```

```bash
ros2 topic pub -r 100 /back_alert std_msgs/msg/Bool "{data: false}"
```

### 1. Start mapping stack

```bash
ros2 run robot_navigation nav_assistant mapping-stack
```

Optional:

```bash
ros2 run robot_navigation nav_assistant mapping-stack --with-rviz true --with-collision true
```

### 1.1 One-command P1 mission

```bash
ros2 run robot_navigation nav_assistant mission-p1
```

This launches mapping stack + Nav2 + mission orchestrator and runs:

`BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`

### 1.2 One-command frontier mission (P2-P4)

```bash
ros2 run robot_navigation nav_assistant frontier-mission
ros2 run robot_navigation nav_assistant frontier-mission --interactive-override true
```

This launches:

`slam_mapping_stack + nav2 + frontier_explorer + mission_orchestrator(frontier mode) [+ semantic_overlay optional]`

Enable semantic overlay:

```bash
ros2 run robot_navigation nav_assistant frontier-mission \
  --with-semantic-overlay true \
  --shelves-file <abs_path_to_shelves.yaml>
```

Enable semantic overlay + automatic map-pattern alignment (P4 V2):

```bash
ros2 run robot_navigation nav_assistant frontier-mission \
  --with-semantic-overlay true \
  --shelves-file <abs_path_to_shelves.yaml> \
  --semantic-reference-map-yaml <abs_path_to_reference_map_yaml> \
  --semantic-auto-align-on-start true
```

Same-terminal override console mode:

```bash
ros2 run robot_navigation nav_assistant mission-p1 --interactive-override true
```

Keys in console mode:

- `m`: publish `/manual_override=true`
- `a`: publish `/manual_override=false` (resume mission)
- `f`: call `/finish_mapping`
- `q`: stop mission launch and quit

### 2. Drive robot manually

```bash
ros2 run robot_navigation nav_assistant teleop
```

Force manual takeover / release:

```bash
ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: false}"
```

Mission control services:

```bash
ros2 service call /start_mission std_srvs/srv/Trigger "{}"
ros2 service call /pause_mission std_srvs/srv/Trigger "{}"
ros2 service call /resume_mission std_srvs/srv/Trigger "{}"
ros2 service call /finish_mapping std_srvs/srv/Trigger "{}"
```

Frontier explorer services:

```bash
ros2 service call /frontier_explorer/start std_srvs/srv/Trigger "{}"
ros2 service call /frontier_explorer/stop std_srvs/srv/Trigger "{}"
```

### 3. Save and export map

```bash
ros2 run robot_navigation nav_assistant save-map --map-name testmap1
ros2 run robot_navigation nav_assistant export-map --map-name testmap1
```

### 4. Start localization + Nav2

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1
```

Default behavior in both mapping/localization stacks:

- LiDAR and IMU are published in `lidar_link`.
- Static TF `base_link -> lidar_link` is published with default offset `(x=0.254, y=0, z=0, rpy=0,0,0)`.
- `cmd_vel_arbiter` merges manual (`/cmd_vel_manual`) and auto (`/cmd_vel_auto,/cmd_vel_nav,/cmd_vel_smoothed`) into `/cmd_vel`.
- Bridge publishes `/odom_raw`.
- EKF (`robot_localization`) fuses `/odom_raw + /sick_scansegment_xd/imu` and publishes filtered `/odom`.

Headless Jetson default:

```bash
ros2 run robot_navigation nav_assistant localization-stack --map-name testmap1 --with-nav2-rviz false
```

### 5. Send goal or waypoints

```bash
ros2 run robot_navigation nav_assistant goal --x 1.0 --y 0.0 --yaw 0.0
```

```bash
ros2 run robot_navigation nav_assistant waypoints \
  --pose 1.0,0.0,0.0 \
  --pose 1.5,0.5,0.0 \
  --pose 0.5,1.0,0.0
```

## Map output location

By default, maps are written to the repo-level `Maps/` folder:

- `save-map` writes: `<repo_root>/Maps/<map_name>.pbstream`
- `export-map` writes: `<repo_root>/Maps/<map_name>.yaml` and `<repo_root>/Maps/<map_name>.pgm`

For example with `--map-name testmap1`:

- `<repo_root>/Maps/testmap1.pbstream`
- `<repo_root>/Maps/testmap1.yaml`
- `<repo_root>/Maps/testmap1.pgm`

You can override location with `--maps-dir`:

```bash
ros2 run robot_navigation nav_assistant save-map --maps-dir /tmp/maps --map-name lab_a
ros2 run robot_navigation nav_assistant export-map --maps-dir /tmp/maps --map-name lab_a
```

## Motion presets

Run one preset sequence:

```bash
ros2 run robot_navigation nav_assistant motion --preset box_loop --topic /cmd_vel_manual
```

Interactive one-key mode:

```bash
ros2 run robot_navigation nav_assistant motion-pad --topic /cmd_vel_manual
```

Keys:

- `1`: `forward_stop`
- `2`: `strafe_test`
- `3`: `spin_scan`
- `4`: `box_loop`
- `space`: stop
- `q`: quit

## Helpful helpers

Print short runbook commands:

```bash
ros2 run robot_navigation nav_assistant print-runbook --map-name testmap1
```

Quick topic/action checks:

```bash
ros2 run robot_navigation nav_assistant quick-check
```

## Notes

- Default serial port is `/dev/ttyUSB0` and default baud is `115200`.
- Default auto command topics consumed by arbiter are:
  `[/cmd_vel_auto, /cmd_vel_nav, /cmd_vel_smoothed]`
- Bridge consumes only arbiter output topic: `/cmd_vel`.
- Semantic overlay query service:
  `/semantic_overlay/query_shelf_pose` (`robot_interfaces/srv/QueryShelfPose`).
- Semantic overlay two-point alignment service:
  `/semantic_overlay/set_alignment` (`robot_interfaces/srv/SetSemanticAlignment`).
- Semantic overlay auto alignment service (map pattern matching):
  `/semantic_overlay/auto_align` (`std_srvs/srv/Trigger`).
- P5 profile files:
  `config/nav2_params_explore_slow.yaml`, `config/nav2_params_task_run.yaml`.
- Generate profile comparison markdown:
  `ros2 run robot_navigation nav_profile_report --base <base.yaml> --target <target.yaml> --output <report.md>`.
- To disable EKF for troubleshooting, use:
  `--use-ekf false --odom-topic /odom` on `mapping-stack` or `localization-stack`.
- For Linux deployment, use lowercase launch filename `my_carto_localization.launch.py` if you run Nav-level launch scripts directly.
