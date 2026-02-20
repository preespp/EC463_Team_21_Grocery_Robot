# Robot Navigation + Localization Adaptation Plan

## Purpose
This document summarizes the current navigation/localization architecture in `robot_navigation`, then proposes a systematic plan to add `robot_localization` while preserving the current workflow style (mapping phase + localization/Nav2 phase + headless Jetson usage).

---

## 0. Geometry Lock-In (Must Be Done First)

Before EKF, Nav2 tuning, or TF refactors, this stack needs one geometry truth source shared by STM32 kinematics, odom interpretation, and Nav2 collision geometry.

### 0.1 User-confirmed physical inputs
- Base outer plate: `20 x 20 in` (`0.508 x 0.508 m`)
- LiDAR mount: middle of front bar (not robot center)
- Mecanum wheel center spacing (left to right): `25.5 in` (`0.6477 m`)

### 0.2 Current geometry settings snapshot (by file)

STM32 base kinematics (`test` firmware source currently in repo):
- `Wheel_Radius = 0.0762 m` (3 in radius / 6 in diameter)
- `Wheel_Base_Half_Length = 0.1905 m` (15.0 in full center spacing)
- `Wheel_Base_Half_Width = 0.3175 m` (25.0 in full center spacing)
- `Wheel_Direction = {1, -1, 1, -1}` (wiring sign compensation)

Files:
- `STM32/Base_Control_v2/robowalker2024bottominfantry-main/test/User_File/3_Chariot/1_Module/Chassis/crt_chassis.h`
- `STM32/Base_Control_v2/robowalker2024bottominfantry-main/test/User_File/3_Chariot/1_Module/Chassis/crt_chassis.cpp`

Scope note:
- Repository also contains alternate kinematics code paths (for example `STM32/Base_Control/...` and `Nav/Kinematic_esp/...`) with different geometry constants.
- This matching table is based on the current `Base_Control_v2/.../test/...` constants and the active ROS launch path in `workspace/src/robot_navigation/launch/*`.

ROS/Nav2 geometry-sensitive settings:
- Local/global costmap use `robot_radius: 0.22`
- DWB effectively differential (`max_vel_y=0`, `min_vel_y=0`, `acc_lim_y=0`, `decel_lim_y=0`)
- Velocity smoother also zeros Y limits (`max_velocity[1]=0`, `min_velocity[1]=0`)

File:
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`

LiDAR frame handling (current):
- Point cloud is published with `frameid=base_link`
- Driver launch passes `publish_frame_id:=base_link`
- No explicit `base_link -> lidar_link` static TF in stack launch files

Files:
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`

Cartographer frame usage (current):
- `tracking_frame = base_link`
- `published_frame = base_link`
- `use_odometry = false`
- `use_imu_data = false`

Files:
- `workspace/src/robot_navigation/config/pico_2d.lua`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua`

### 0.3 Quantified mismatches

#### A) Wheel-center width mismatch (known)
- User-measured full width: `25.5 in` (`0.6477 m`)
- STM32 configured full width: `25.0 in` (`0.6350 m`)
- Mismatch: `0.5 in` (`0.0127 m`) total, `0.25 in` (`0.00635 m`) per side
- Relative error: `1.96%` low

Immediate constant fix:
- `Wheel_Base_Half_Width`: `0.3175 -> 0.32385`

#### B) Front-rear wheel-center spacing mismatch (not yet measured)
- STM32 currently uses `15.0 in` full (from `0.1905 m` half length)
- If wheel-center spacing is approximately square (assumption-based), expected front-rear may be near `25.5 in`
- Assumption-only mismatch would be `10.5 in` (`0.2667 m`) low (`41.2%`)

Required action:
- Measure actual front-center to rear-center wheel distance and update `Wheel_Base_Half_Length` accordingly.

#### C) Nav2 collision radius mismatch
Current value:
- `robot_radius = 0.22 m`

Reference envelopes from known data:
- Base plate only (20x20): minimum circle radius from center to corner  
  `sqrt((0.254)^2 + (0.254)^2) = 0.3592 m`
- Wheel-aware half-width lower bound using 25.5 in center spacing and 3 in wheel radius  
  `0.32385 + 0.0762 = 0.40005 m`

Current radius underestimation:
- Versus base-only minimum: `0.1392 m` low (`38.75%`)
- Versus wheel-aware lower bound: `0.18005 m` low (`45.01%`)

#### D) LiDAR extrinsic mismatch
- Current stack treats LiDAR point cloud frame as `base_link` (implicit zero offset).
- Physical mounting is front-center on a 20 in base, so with `base_link` at geometric center:
  - expected `x` offset is about `+0.254 m`
  - expected `y` offset is about `0.0 m`
  - `z` and roll/pitch/yaw must be measured

### 0.4 Radius/footprint fix options

Option 1 (quick safety patch, circular):
- Set `robot_radius` to `0.40 m` (conservative lower bound from known wheel-width data).

Option 2 (recommended, accurate):
- Replace `robot_radius` with explicit `footprint` polygon from measured outer envelope.
- For immediate interim base-only footprint (20x20, no wheel overhang):
  - `[[0.254, 0.254], [0.254, -0.254], [-0.254, -0.254], [-0.254, 0.254]]`
- After measuring wheel overhang, update polygon to true collision envelope.

### 0.5 LiDAR offset migration plan (front-middle mount)

1. Keep `base_link` at robot center (same reference used by odom/kinematics interpretation).
2. Introduce `lidar_link` and publish static TF:
   - `base_link -> lidar_link = (x=+0.254, y=0.0, z=<measured>, roll/pitch/yaw=<measured>)`
3. Update SICK driver launch args:
   - `publish_frame_id:=lidar_link`
   - pointcloud config `frameid=lidar_link`
4. Keep Cartographer `tracking_frame=base_link` (good), and let TF map LiDAR into base frame.
5. Validate in TF:
   - `ros2 run tf2_tools view_frames`
   - `ros2 run tf2_ros tf2_echo base_link lidar_link`

### 0.6 Section 0 first-action checklist

1. Update STM32 `Wheel_Base_Half_Width` for measured `25.5 in`.
2. Measure and update STM32 `Wheel_Base_Half_Length` (do not leave assumption).
3. Update Nav2 costmap from `robot_radius=0.22` to safe radius/footprint.
4. Introduce `lidar_link` and static offset TF in both mapping/localization launches.
5. Re-run odom and TF validation before moving to EKF integration phases.

---

## 1. Current Structure (As-Is)

### 1.1 Runtime stacks

### Mapping stack (`slam_mapping_stack.launch.py`)
- Starts SICK driver (`sick_generic_caller`) with project-specific point cloud/frame args.
- Starts Cartographer mapping (`cartographer_mapping.launch.py`).
- Starts STM32 serial bridge (`nav2_serial_bridge`) with `/cmd_vel*` subscriptions and `/odom` publishing.
- Optional ultrasonic collision launch.
- Optional RViz.

Relevant files:
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
- `workspace/src/robot_navigation/launch/cartographer_mapping.launch.py`

### Localization + Nav2 stack (`nav2_localization_stack.launch.py`)
- Starts SICK driver.
- Starts serial bridge.
- Starts Cartographer localization (`load_state_filename` from `.pbstream`, frozen state).
- Starts `map_server` + map lifecycle manager.
- Starts Nav2 bringup (`navigation_launch.py`) with `nav2_params_cartographer.yaml`.
- Optional Nav2 RViz (default currently headless/off).

Relevant files:
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
- `workspace/src/robot_navigation/launch/cartographer_localization.launch.py`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua`

### CLI workflow helper
- `nav_assistant` wraps mapping/localization/teleop/map save/export/goal/waypoint commands.
- Current headless-friendly default for localization is `with_nav2_rviz=false`.

Relevant file:
- `workspace/src/robot_navigation/robot_navigation/nav_assistant.py`

### BT integration status (task manager side)
- `NavigateToGoalPose` is currently placeholder and returns success if a goal exists.
- Active customer/restock trees still use demo `MoveDistanceForCurrentItem`.

Relevant files:
- `workspace/src/robot_task_manager/robot_task_manager/bt_nodes/navigation_nodes.py`
- `workspace/src/robot_task_manager/robot_task_manager/trees/customer.py`
- `workspace/src/robot_task_manager/robot_task_manager/trees/restock.py`

---

## 2. Current Data Flow and Localization Logic

## 2.1 High-level flow

```text
SICK LiDAR ---> /cloud_all_fields_fullframe ----> Cartographer
                                              |
STM32 telemetry ---> nav2_serial_bridge ---> /odom topic (nav_msgs/Odometry)
STM32 cmd link <--- nav2_serial_bridge <--- /cmd_vel, /cmd_vel_nav, /cmd_vel_smoothed

Localization phase:
  map_server publishes static /map from YAML
  Cartographer loads pbstream and performs scan matching
  Nav2 consumes map + TF + odom data for planning/control
```

## 2.2 Current odom generation from base (important)

`nav2_serial_bridge` currently:
- Parses telemetry (minimum 6 floats):  
  `[target_vx, target_vy, target_omega, now_vx, now_vy, now_omega, ...optional wheel data]`
- Integrates pose in `odom` frame from measured velocities:
  - `x += cos(yaw)*vx*dt - sin(yaw)*vy*dt`
  - `y += sin(yaw)*vx*dt + cos(yaw)*vy*dt`
  - `yaw += omega*dt`
- Publishes `nav_msgs/Odometry` on `/odom`.
- If telemetry is stale and `fallback_odom=true`, integrates using commanded velocities as fallback.

Relevant file:
- `workspace/src/robot_navigation/robot_navigation/nav2_serial_bridge.py`

## 2.3 Current `/odom` message format (from base bridge)

Topic type:
- `nav_msgs/msg/Odometry`

Fields currently populated:
- `header.stamp`: bridge clock timestamp
- `header.frame_id`: default `"odom"`
- `child_frame_id`: default `"base_link"`
- `pose.pose.position.{x,y}`: integrated
- `pose.pose.orientation`: integrated yaw quaternion
- `pose.covariance`: from `pose_covariance_diagonal` param
- `twist.twist.linear.{x,y}`: from telemetry `now_vx, now_vy`
- `twist.twist.angular.z`: from telemetry `now_omega`
- `twist.covariance`: from `twist_covariance_diagonal` param

## 2.3.1 STM32 -> `robot_localization` format rematch contract (required)

`robot_localization` cannot consume raw UART float frames directly.  
The STM32 data must be converted into standard ROS messages first, then fused.

### Current STM32 telemetry frame contract (as decoded by bridge)
- Byte layout:  
  `[telemetry_header][N * float32 little-endian][optional uint8 checksum]`
- Default header: `0xAB`
- Default channel count: `N=6`
- Float order (minimum):
  1. `target_vx` (m/s)
  2. `target_vy` (m/s)
  3. `target_omega` (rad/s)
  4. `now_vx` (m/s)
  5. `now_vy` (m/s)
  6. `now_omega` (rad/s)
- Optional extra floats are allowed (wheel data, etc.) and currently ignored by EKF path.
- Checksum (if enabled): `sum(data_bytes) & 0xFF`

### Required ROS-side rematch for EKF input
- Bridge output topic should be split to raw odom:
  - `/odom_raw` (`nav_msgs/msg/Odometry`)
- Keep these frame IDs:
  - `header.frame_id = "odom"`
  - `child_frame_id = "base_link"`
- Required unit/sign conventions (REP-105):
  - `linear.x > 0`: robot forward
  - `linear.y > 0`: robot left
  - `angular.z > 0`: CCW rotation
- Timestamps must be monotonic and current ROS time.

### What `robot_localization` should consume from base odom
Because bridge pose is integrated from the same velocity source, feeding both pose and twist can double-count one sensor.

Recommended first configuration:
- Use only twist terms from `/odom_raw`:
  - `vx`, `vy`, `vyaw`
- Ignore pose terms from `/odom_raw` in EKF.

Recommended `odom0_config` (15 bools):
```yaml
odom0_config: [false, false, false,
               false, false, false,
               true,  true,  false,
               false, false, true,
               false, false, false]
```

### IMU format expected by `robot_localization`
- Topic type: `sensor_msgs/msg/Imu`
- Frame: IMU frame linked to `base_link` by static TF
- At minimum for 2D fusion:
  - `angular_velocity.z`
  - optionally yaw orientation quaternion (`orientation`)
- Covariances must be realistic and non-zero for used terms.

Example 2D IMU selector:
```yaml
imu0_config: [false, false, false,
              false, false, true,
              false, false, false,
              false, false, true,
              false, false, false]
```

### Covariance requirements (important)
- Do not publish zero covariance for signals you want EKF to trust.
- Keep `pose_covariance_diagonal` / `twist_covariance_diagonal` tuned to real sensor quality.
- If fallback odom is based on commands (`fallback_odom=true`), increase covariance during fallback or disable fallback for EKF input to avoid over-trusting synthetic motion.

## 2.4 Current Cartographer usage

In both mapping and localization Lua configs:
- `use_odometry = false`
- `use_imu_data = false`
- `provide_odom_frame = true`

So Cartographer is running scan-matching-based localization without odom/IMU fusion input.

Relevant files:
- `workspace/src/robot_navigation/config/pico_2d.lua`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua`

## 2.5 Current TF conflict prevention

Current launch setup prevents major TF collisions:
- Bridge TF publishing is disabled (`publish_tf: false`).
- AMCL TF broadcast is disabled (`tf_broadcast: false`) to avoid map->odom conflicts when Cartographer is active.

Relevant files:
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`

---

## 3. Current Usage (Operator Workflow)

### Mapping
1. `ros2 run robot_navigation nav_assistant mapping-stack`
2. `ros2 run robot_navigation nav_assistant teleop`
3. `ros2 run robot_navigation nav_assistant save-map --map-name <name>`
4. `ros2 run robot_navigation nav_assistant export-map --map-name <name>`

### Localization/Nav2 (headless default)
1. `ros2 run robot_navigation nav_assistant localization-stack --map-name <name>`
2. Goal sending:
   `ros2 run robot_navigation nav_assistant goal --x ... --y ... --yaw ...`

---

## 4. Current Evaluation (Technical)

## 4.1 What is good
- End-to-end stack is launchable through one-line commands.
- LiDAR arguments are now explicitly applied in stack launches.
- Headless localization default is configured.
- TF ownership conflicts are mostly controlled.

## 4.2 Current limitations
- Odom and localization are not truly fused:
  - bridge odom is one estimate,
  - Cartographer scan-match estimate is separate.
- Cartographer ignores wheel odom and IMU, so drift risk increases in repetitive aisles or aggressive motion.
- Nav2 config is effectively differential (lateral `y` limits are zero), so mecanum holonomic capability is not used.
- Task manager BT navigation to Nav2 action is not yet integrated.

---

## 5. Why Add `robot_localization` Here

`robot_localization` provides EKF/UKF fusion using standard ROS message topics:
- `nav_msgs/Odometry`
- `sensor_msgs/Imu`
- optional other pose/twist sources

Benefits for this project:
- Stabilizes local odom with IMU + base odom fusion.
- Reduces sensitivity to short telemetry dropouts/noise.
- Produces smoother velocity/orientation for Nav2 and/or Cartographer odom input.
- Provides a systematic place to tune covariances and sensor trust.

---

## 6. Recommended Integration Strategy

## Strategy A (recommended first: low-risk, minimal TF disruption)
- Keep current TF ownership pattern (Cartographer continues TF role as today).
- Bridge publishes raw base odom to `/odom_raw` (instead of `/odom`).
- Add EKF (`robot_localization`) to fuse `/odom_raw + /imu/data` and publish filtered odom topic `/odom` (or `/odometry/filtered` with remap).
- Enable Cartographer `use_odometry=true` so scan matching is aided by fused odom.
- Keep IMU usage in Cartographer off initially, then evaluate turning it on later.

Why this is safer:
- Minimizes TF ownership changes.
- Keeps current launch/user workflow nearly identical.
- Adds fusion quality improvements without a large frame-graph refactor.

## Strategy B (later/advanced: canonical TF separation)
- EKF owns `odom -> base_link` TF.
- Cartographer publishes only `map -> odom`.
- Requires careful frame/lua and TF ownership redesign and validation.

---

## 7. Systematic Change Plan (Files + Estimated Scope)

Estimates are for manual edits only (not counting testing time).

## Phase 0: Geometry + frame correction (must run first)

Modify STM32 geometry constants:
- `STM32/Base_Control_v2/robowalker2024bottominfantry-main/test/User_File/3_Chariot/1_Module/Chassis/crt_chassis.h`
  - Set `Wheel_Base_Half_Width = 0.32385f` for measured `25.5 in` left-right wheel-center spacing.
  - Update `Wheel_Base_Half_Length` after front-rear center spacing is physically measured.

Modify Nav2 collision geometry:
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`
  - Replace `robot_radius: 0.22` with either:
    - quick safe radius (`0.40`), or
    - measured footprint polygon (recommended).

Modify LiDAR extrinsics handling:
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
  - Change point cloud frame from `base_link` to `lidar_link`.
  - Add a `static_transform_publisher` for `base_link -> lidar_link` with measured translation/rotation.

Estimated change:
- Files: 4-5
- LOC: 60-140

## Phase 1: Dependency + packaging prep
- Add dependency in package metadata:
  - `workspace/src/robot_navigation/package.xml` add `<depend>robot_localization</depend>`
- Ensure deployment installation includes `ros-humble-robot-localization`.

Estimated change:
- Files: 1
- LOC: 1-3

## Phase 2: Add EKF config and launch wiring

Add new file:
- `workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml`
  - Inputs:
    - `odom0: /odom_raw`
    - `imu0: /imu/data`
  - Output:
    - filtered odom topic (`/odom` recommended for compatibility)
  - Start with conservative covariances and 2D mode.

Modify:
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
  - Add launch arg for EKF toggle (`use_ekf:=true/false`).
  - Add `robot_localization` `ekf_node` when enabled.
  - Add optional args for imu/odom topics.
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
  - Optional: same EKF toggle for mapping phase consistency.

Estimated change:
- Files: 3
- LOC: 140-240

## Phase 3: Bridge topic split (`raw` vs `filtered`)

Modify:
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
  - Set bridge `odom_topic:=/odom_raw`.
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
  - Set bridge `odom_topic:=/odom_raw` (if EKF used during mapping).
- Optionally add launch arg pass-through for `odom_topic`.

Estimated change:
- Files: 2
- LOC: 20-40

## Phase 4: Cartographer odom assistance

Modify Lua:
- `workspace/src/robot_navigation/config/pico_2d.lua`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua`

Set:
- `use_odometry = true`
- Keep `use_imu_data = false` for first rollout.

Estimated change:
- Files: 2
- LOC: 2-6

## Phase 5: Holonomic Nav2 tuning (mecanum enablement)

Modify:
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`

Start point:
- `max_vel_y`: `0.20` (from `0.0`)
- `min_vel_y`: `-0.20` (from `0.0`)
- `acc_lim_y`: `1.5` (from `0.0`)
- `decel_lim_y`: `-1.5` (from `0.0`)
- `vy_samples`: `15` (from `5`)
- Velocity smoother arrays: set non-zero Y entries.

Estimated change:
- Files: 1
- LOC: 12-25

## Phase 6: BT Nav2 action integration (outside robot_navigation package but critical)

Modify:
- `workspace/src/robot_task_manager/robot_task_manager/bt_nodes/navigation_nodes.py`
  - Replace placeholder with real Nav2 action client behavior.
- Update trees to use `NavigateToGoalPose` where intended.

Estimated change:
- Files: 2-3
- LOC: 120-260

---

## 8. Approximate Total Change Budget

For Strategy A + geometry correction + holonomic tuning + BT action integration:
- New files: 1
- Modified files: 11-14
- Estimated net LOC: 360-710
- Regression risk: Medium (geometry + TF + tuning assumptions)

For Strategy A + geometry correction only (no BT integration yet):
- New files: 1
- Modified files: 8-11
- Estimated net LOC: 240-450
- Regression risk: Low-to-medium

---

## 9. Validation Plan (Required)

## 9.1 Static checks
- Build:
  - `colcon build --symlink-install --packages-select robot_navigation`
- Launch arg checks:
  - `ros2 launch robot_navigation nav2_localization_stack.launch.py --show-args`

## 9.2 Runtime checks
- Topic:
  - `ros2 topic echo /odom_raw --once`
  - `ros2 topic echo /odom --once`
  - `ros2 topic hz /odom`
  - `ros2 topic echo /imu/data --once`
- TF:
  - `ros2 run tf2_tools view_frames`
  - `ros2 run tf2_ros tf2_echo base_link lidar_link`
  - Confirm single publisher ownership for each transform.
- Nav2 behavior:
  - Send single goal and verify stable heading + reduced oscillation.
  - Test lateral path cases (aisle side-shifts) after holonomic tuning.

## 9.3 Acceptance criteria
- No TF conflicts/warnings for duplicate frame publishers.
- `base_link -> lidar_link` exists and reflects non-zero front offset.
- EKF output remains continuous under short telemetry interruptions.
- Localization drift reduced in straight-aisle runs vs current baseline.
- Nav2 can generate/execute feasible lateral motion when appropriate.

---

## 10. Operational Notes / Caveats

- If bridge telemetry becomes unreliable, `fallback_odom` can mask issues by integrating commanded velocity; this is useful for continuity but weak for localization truth.
- Set realistic covariance values in bridge and EKF; zero/overconfident covariances will degrade fusion.
- Keep one clear TF authority per transform edge.
- `robot_localization` is not currently declared in this package dependencies and must be added/installed for the integration plan.

---

## 11. Suggested Execution Order

1. Phase 0 (geometry + frame correction)  
2. Phase 1 (dependency)  
3. Phase 2 (EKF wiring)  
4. Phase 3 (odom topic split)  
5. Phase 4 (Cartographer odom on)  
6. Validate baseline improvements  
7. Phase 5 (holonomic tuning)  
8. Phase 6 (BT Nav2 action integration)

This sequence gives measurable improvements early while containing risk.
