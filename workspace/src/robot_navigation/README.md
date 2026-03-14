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

Default mapping-stack behavior now enables a `base_link` crop filter before
Cartographer to remove rear chassis self-hits. The current default crop box
in `base_link` is `x=[-0.2540, 0.1397] m`, `y=[-0.2794, 0.2794] m`,
`z=[-1.0, 1.0] m`, which matches the current rear `15.5 x 22 in` filter box
on the project robot. Disable it only when debugging with
`--with-base-link-crop false`.

D0 quality-profile run (PointCloud2 input path):
```bash
ros2 run robot_navigation nav_assistant mapping-stack \
  --cartographer-config-basename pico_2d_mapping_quality.lua
```

Optional:

```bash
ros2 run robot_navigation nav_assistant mapping-stack --with-rviz true --with-collision true
```

### 2. Drive robot manually

```bash
ros2 run robot_navigation nav_assistant teleop
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

Default behavior in the current stacks:

- LiDAR is published in `lidar_link`.
- IMU is published in `imu_link`.
- Static TF `base_link -> lidar_link` is published with default offset `(x=0.2413, y=0, z=0, rpy=0,0,0)`.
- Static TF `lidar_link -> imu_link` is published with default offset `(x=0.0124, y=0.0185, z=-0.0484, rpy=0,0,0)`.
- Mapping stack Cartographer consumes `/cloud_all_fields_fullframe_filtered`, produced by the default `base_link` crop filter.
- Localization + Nav2 stack keeps the older default with crop disabled unless you pass `--with-base-link-crop true`.
- The crop filter removes rear chassis self-hits before Cartographer; Nav2 still relies on its own robot footprint/box for planning and collision behavior.
- Cartographer tracks `imu_link` so raw IMU input is colocated with the tracking frame.
- Cartographer still publishes `base_link` projected to 2D, so Nav2 keeps the usual planar robot frame.
- The `0.2413 m` LiDAR forward offset is the current project preset for this robot mount: `20 in / 2 - 1 in / 2 = 9.5 in = 0.2413 m`, with the front mount centered on a standard 1.00 in 80/20 bar.
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
ros2 run robot_navigation nav_assistant motion --preset box_loop --topic /cmd_vel
```

Interactive one-key mode:

```bash
ros2 run robot_navigation nav_assistant motion-pad --topic /cmd_vel
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
- Default command topics bridged to STM32 are:
  `[/cmd_vel, /cmd_vel_nav, /cmd_vel_smoothed]`
- To disable EKF for troubleshooting, use:
  `--use-ekf false --odom-topic /odom` on `mapping-stack` or `localization-stack`.
- For Linux deployment, use lowercase launch filename `my_carto_localization.launch.py` if you run Nav-level launch scripts directly.
