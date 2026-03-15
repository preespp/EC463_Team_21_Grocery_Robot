# Robot Navigation + Localization Adaptation Plan

## Purpose
This document summarizes the current navigation/localization architecture in `robot_navigation`, then proposes a systematic plan to add `robot_localization` while preserving the current workflow style (mapping phase + localization/Nav2 phase + headless Jetson usage).

Collision detection is implemented on serial-uart transmission module to low-level control, not on the high-level autonomy logic. Current publish rate is 100 Hz.

---

## 0. Geometry Lock-In (Must Be Done First)

Before EKF, Nav2 tuning, or TF refactors, this stack needs one geometry truth source shared by STM32 kinematics, odom interpretation, and Nav2 collision geometry.

### 0.1 User-confirmed physical inputs
- Base outer plate: `20 x 20 in` (`0.508 x 0.508 m`)
- LiDAR mount: middle of front bar (not robot center)
- Mecanum wheel center spacing (left to right): `25.5 in` (`0.6477 m`)

For the current mount preset used in this repo:
- Project assumption: the bracket center is used as the LiDAR optical-origin proxy.
- Confirmed front-bar profile for the preset: standard `1.00 in` 80/20 profile width.
- Therefore `base_link -> lidar_link.x = 0.508 / 2 - 0.0254 / 2 = 0.2413 m`.
- Keep `lidar_link -> imu_link = (0.0124, 0.0185, -0.0484) m` from the SICK operating instructions.

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
  - expected `x` offset is about `+0.2413 m` for the current preset (`20 in / 2 - 1 in / 2`)
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
   - `base_link -> lidar_link = (x=+0.2413, y=0.0, z=<measured>, roll/pitch/yaw=<measured>)`
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

---

## 12. Implementation Packet (Sensor Fusion + `robot_localization`)

This section is the concrete rollout plan to implement sensor fusion from base odom + LiDAR IMU with minimal TF risk.

### 12.1 Scope and source of truth

- Runtime source of truth for bridge/launch/config is `workspace/src/robot_navigation/*`.
- `STM32/.../tools/*.py` is treated as archive/reference only (not runtime source of truth).
- STM32 firmware geometry constants remain authoritative for chassis kinematics.

### 12.2 Phase SF-1: Geometry + LiDAR extrinsics lock

Modify:
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`

Actions:
1. Add static TF publisher: `base_link -> lidar_link` with measured `(x, y, z, roll, pitch, yaw)`.
2. Change SICK driver frame args from `base_link` to `lidar_link`:
   - `publish_frame_id`
   - `publish_imu_frame_id`
   - pointcloud `frameid=...`

Goal:
- Sensor frame reflects real mount location instead of implicit zero-offset on `base_link`.

### 12.3 Phase SF-2: Bridge raw odom contract for fusion

Modify:
- `workspace/src/robot_navigation/launch/slam_mapping_stack.launch.py`
- `workspace/src/robot_navigation/launch/nav2_localization_stack.launch.py`
- `workspace/src/robot_navigation/robot_navigation/nav2_serial_bridge.py` (parameter use only, no protocol break)

Actions:
1. Set bridge output topic to `/odom_raw`.
2. Keep `publish_tf:=false` on bridge.
3. Set `fallback_odom:=false` for EKF input path.

Goal:
- EKF receives sensor-based raw odom only, without command-integrated synthetic fallback.

### 12.4 Phase SF-3: Add EKF node

Modify:
- `workspace/src/robot_navigation/package.xml`
- new file `workspace/src/robot_navigation/config/ekf_odom_base_imu.yaml`
- both stack launch files to spawn EKF

Actions:
1. Add `<depend>robot_localization</depend>`.
2. Add EKF config with:
   - `two_d_mode: true`
   - `publish_tf: false` (keep TF authority unchanged in first rollout)
   - `odom0: /odom_raw` with twist-only selection (`vx`, `vy`, `vyaw`)
   - `imu0: /sick_scansegment_xd/imu`
   - `sensor_timeout` tuned for asynchronous startup (recommend `0.2~0.3 s`)
3. Remap EKF output `odometry/filtered -> /odom`.

Goal:
- Stable fused odom output without requiring synchronous sensor startup.

### 12.5 Phase SF-4: Cartographer odom assistance

Modify:
- `workspace/src/robot_navigation/config/pico_2d.lua`
- `workspace/src/robot_navigation/config/pico_2d_localization.lua`

Actions:
1. Set `use_odometry = true`.
2. Keep `use_imu_data = false` in first rollout (avoid double-weighting IMU while EKF already fuses it).

Goal:
- Improve local scan-matching stability with fused odom prior.

### 12.6 Phase SF-5: Holonomic tuning (after fusion baseline passes)

Modify:
- `workspace/src/robot_navigation/config/nav2_params_cartographer.yaml`

Actions:
1. Enable lateral motion limits (`max_vel_y`, `min_vel_y`, `acc_lim_y`, `decel_lim_y`, `vy_samples`).
2. Update velocity smoother Y-axis limits consistently.

Goal:
- Recover mecanum holonomic behavior after localization baseline is stable.

### 12.7 Phase SF-6: Documentation and profile split

Modify docs:
- `workspace/src/robot_navigation/README.md`
- `Nav/README_SLAM_UPDATED.md`

Add two run profiles:
1. `normal_mode`:
   - `/odom_raw -> EKF -> /odom`
   - Cartographer `use_odometry=true`
2. `bench_mode` (wheel-off-ground / maintenance):
   - disable EKF or exclude `/odom_raw` from EKF
   - set Cartographer `use_odometry=false`

Goal:
- Avoid test contamination from wheel-slip/air-spin conditions.

---

## 13. Node-by-Node Test and Validation Targets

Use this section as a one-by-one checklist during bringup.

### 13.1 `sick_generic_caller` (LiDAR + IMU)

Expected:
1. `/cloud_all_fields_fullframe` active at stable sensor rate.
2. `/sick_scansegment_xd/imu` active and timestamped.
3. Published sensor frame is `lidar_link` (not `base_link`).

Validation:
1. `ros2 topic hz /cloud_all_fields_fullframe`
2. `ros2 topic hz /sick_scansegment_xd/imu`
3. `ros2 topic echo /cloud_all_fields_fullframe --once` (check `header.frame_id`)
4. `ros2 topic echo /sick_scansegment_xd/imu --once` (check `header.frame_id`)

Expected command output:
1. `ros2 topic hz /cloud_all_fields_fullframe`
   - contains `average rate: ...`
   - rate is stable (no repeated `0.0`/timeout line)
2. `ros2 topic hz /sick_scansegment_xd/imu`
   - contains `average rate: ...`
   - rate is stable and non-zero
3. `ros2 topic echo /cloud_all_fields_fullframe --once`
   - contains `header:`
   - contains `frame_id: lidar_link`
4. `ros2 topic echo /sick_scansegment_xd/imu --once`
   - contains `header:`
   - contains `frame_id: lidar_link` (or your chosen dedicated IMU frame if you configured one)

Pass criteria:
1. No frame mismatch warnings from Cartographer TF lookup.
2. Both topics present before Nav2 activation.

### 13.2 `static_transform_publisher` (`base_link -> lidar_link`)

Expected:
1. Exactly one static TF edge `base_link -> lidar_link`.
2. Transform values equal measured geometry.

Validation:
1. `ros2 run tf2_ros tf2_echo base_link lidar_link`
2. `ros2 run tf2_tools view_frames`

Expected command output:
1. `ros2 run tf2_ros tf2_echo base_link lidar_link`
   - prints `At time ...`
   - prints `Translation: [x, y, z]` matching measured mount offset
   - prints `Rotation: in Quaternion ...` matching measured RPY conversion
2. `ros2 run tf2_tools view_frames`
   - generated frame graph contains edge `base_link -> lidar_link`
   - only one parent for `lidar_link` (no duplicate broadcaster conflict)

Pass criteria:
1. Edge exists and is stable.
2. No duplicate TF publisher conflict on same edge.

### 13.3 `nav2_serial_bridge` (raw odom producer)

Expected:
1. Startup log shows `odom_topic=/odom_raw`.
2. `publish_tf=false`.
3. Telemetry decoding stable (no persistent checksum storm).
4. `/odom_raw` published continuously from telemetry.

Validation:
1. `ros2 topic hz /odom_raw`
2. `ros2 topic echo /odom_raw --once`
3. Observe bridge logs for bad checksum/outlier counters.

Expected command output:
1. bridge startup log includes:
   - `odom_topic=/odom_raw`
   - `frame=odom->base_link`
   - `publish_tf=False`
2. `ros2 topic hz /odom_raw`
   - contains `average rate: ...`
   - expected near telemetry output rate (typically around `180-210 Hz` with current firmware setting)
3. `ros2 topic echo /odom_raw --once`
   - contains `header.frame_id: odom`
   - contains `child_frame_id: base_link`
4. bridge runtime logs:
   - no continuous checksum mismatch flood
   - no persistent "No valid telemetry frame decoded yet" warning loop

Pass criteria:
1. `/odom_raw` available and monotonic.
2. No persistent parser mismatch condition.

### 13.4 `ekf_filter_node` (`robot_localization`)

Expected:
1. Node starts even if IMU and odom start at different moments.
2. Fused output `/odom` remains continuous once inputs arrive.
3. No TF conflict (`publish_tf=false` in initial rollout).

Validation:
1. `ros2 topic hz /odom`
2. `ros2 topic echo /odom --once`
3. `ros2 node list | grep ekf`

Expected command output:
1. `ros2 node list | grep ekf`
   - contains EKF node name (for example `ekf_filter_node`)
2. `ros2 topic hz /odom`
   - contains `average rate: ...`
   - rate tracks EKF frequency target (for example `~50 Hz` if configured as 50)
3. `ros2 topic echo /odom --once`
   - contains `header.frame_id: odom`
   - contains `child_frame_id: base_link`
   - pose/twist values are finite and update continuously across repeated samples

Pass criteria:
1. `/odom` rate near EKF configured frequency.
2. No large discontinuities when delayed sensor starts publishing.

### 13.5 `cartographer_node` (mapping/localization)

Expected:
1. Consumes pointcloud in `lidar_link` via TF to `base_link`.
2. Uses fused odom prior (`use_odometry=true`).
3. Maintains connected TF chain `map -> odom -> base_link`.

Validation:
1. `ros2 run tf2_tools view_frames`
2. Monitor Cartographer logs for transform timeout/extrapolation warnings.

Expected command output:
1. `ros2 run tf2_tools view_frames`
   - connected chain includes `map -> odom -> base_link -> lidar_link`
2. cartographer logs:
   - startup includes trajectory creation (for example `Added trajectory ...`)
   - no repeated transform timeout / extrapolation error spam during steady run

Pass criteria:
1. Reduced scan-matching instability in straight aisles/turn entries.
2. No repeated transform lookup failures.

### 13.6 `map_server` + `lifecycle_manager`

Expected:
1. `map_server` active before Nav2 planner/controller start using static map.

Validation:
1. `ros2 lifecycle get /map_server`
2. `ros2 topic info /map`

Expected command output:
1. `ros2 lifecycle get /map_server`
   - contains `active` (or `active [3]` depending ROS 2 output style)
2. `ros2 topic info /map`
   - contains at least one publisher
   - topic type shown as `nav_msgs/msg/OccupancyGrid`

Pass criteria:
1. Lifecycle state is `active`.
2. `/map` publisher present before sending goals.

### 13.7 Nav2 core nodes (`planner_server`, `controller_server`, `bt_navigator`)

Expected:
1. Bringup reaches active state.
2. Goal execution produces `/cmd_vel` stream.
3. Motion behavior is stable with fused `/odom`.

Validation:
1. `ros2 action list | grep navigate_to_pose`
2. `ros2 topic hz /cmd_vel`
3. Send one test goal in clear environment.

Expected command output:
1. `ros2 action list | grep navigate_to_pose`
   - contains `/navigate_to_pose`
2. `ros2 topic hz /cmd_vel`
   - while goal is active, contains non-zero `average rate`
3. goal execution (`ros2 action send_goal /navigate_to_pose ...`)
   - response includes goal accepted
   - final result reports success for reachable test goal

Pass criteria:
1. Goal accepted and completed without TF timeout failure.
2. Reduced heading oscillation versus pre-fusion baseline.

### 13.8 Wheel-off-ground fallback (`bench_mode`)

Purpose:
- Prevent false odom confidence when wheels spin in air.

Settings:
1. Disable EKF or remove `/odom_raw` from EKF input.
2. Set Cartographer `use_odometry=false`.
3. Keep command/telemetry transport tests only.

Validation:
1. Confirm stack still runs for communication diagnostics.
2. Do not use bench-mode data as mapping/localization quality evidence.

Expected command output:
1. `ros2 topic hz /odom_raw`
   - remains active (communication path still testable)
2. if EKF disabled in bench mode:
   - `ros2 node list | grep ekf` returns no EKF node
   - `/odom` may be absent or unchanged by EKF path (expected for this profile)
3. Cartographer bench-mode run:
   - no requirement to pass map-quality metrics from this mode

Pass criteria:
1. Communication path can be debugged safely.
2. No production map quality claims from suspended-wheel runs.
