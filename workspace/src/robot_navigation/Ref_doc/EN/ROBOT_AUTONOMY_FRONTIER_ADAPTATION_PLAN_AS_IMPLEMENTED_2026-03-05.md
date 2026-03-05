# Robot Autonomy Frontier Adaptation Plan (As-Implemented Update)

Date: 2026-03-05

## 0. Purpose

This document is the implementation-aligned copy of the original adaptation plan.
The original file is kept unchanged:
`ROBOT_AUTONOMY_FRONTIER_ADAPTATION_PLAN.md`.

This update keeps the same adaptation direction, and adds:
1. Current implementation status (P0-P5) from repository code.
2. Concrete module behavior and interfaces actually present in code.
3. Remaining validation items that still require hardware acceptance.

---

## 1. Current Baseline (Code-Verified)

## 1.1 Existing runtime pipeline

1. `robot_localization` EKF:
   `/odom_raw + /sick_scansegment_xd/imu -> /odom`
2. `Cartographer` (online SLAM):
   `use_odometry=true`, `use_imu_data=false`, scan matching + pose graph
3. `Nav2`:
   global planner `NavFnPlanner (use_astar=false)`, local controller `DWBLocalPlanner`
4. Command authority and safety:
   `cmd_vel_arbiter` merges manual and auto commands to `/cmd_vel`,
   then `nav2_serial_bridge` applies ultrasonic directional blocking
   (`/front_alert`, `/back_alert`, `/left_alert`, `/right_alert`) before STM32 output.
5. Resolution defaults in code:
   map export default `0.03`, Nav2 local/global costmap default `0.03`.

## 1.2 Previous gaps vs current status

| Item from original plan | Current status |
|---|---|
| Frontier exploration node missing | Implemented (`frontier_explorer.py`) |
| Manual-first command arbitration missing | Implemented (`cmd_vel_arbiter.py`) |
| Mission state machine missing | Implemented (`mission_orchestrator.py`) |
| One-command autonomous mission launch missing | Implemented (`auto_map_mission_v1.launch.py`, `auto_frontier_mission.launch.py`, `nav_assistant`) |
| Semantic shelf overlay missing | Implemented (`semantic_overlay.py`) with manual and automatic alignment |

Additional current status:
1. P5 profile switching is wired into `frontier-mission` via `--explore-profile`.
2. P5 comparison report tool is implemented (`nav_profile_report`).
3. Automatic runtime regression monitor daemon is intentionally not included in this version.

---

## 2. Target Architecture (As Implemented)

```text
LiDAR + IMU + Raw wheel odom
          |       |
          +-------+-------> EKF (/odom)
                           |
                           v
Cartographer online SLAM (scan matching + loop closure + map<->odom)
                           |
                           +--> /map (online occupancy grid)
                           |
                           v
                        Nav2
                           ^
                           |
                Frontier Explorer (NavigateToPose goals)
                           |
                      /cmd_vel_auto

Teleop --> /cmd_vel_manual ----+
                               v
                     cmd_vel_arbiter --> /cmd_vel --> nav2_serial_bridge --> STM32
                               ^
                     /manual_override (Bool)

Mission Orchestrator (frontier mode):
BOOT -> AUTO_EXPLORE -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY

Mission Orchestrator (P1 fixed mode):
BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY

Semantic Overlay (optional in frontier mission):
shelves.yaml/json + reference_map_yaml + /map -> aligned markers + shelf pose service
```

---

## 3. Implementation Order and Priority (Status and Details)

## P0 (Highest): Control Authority + Safety Closure

Status: Implemented in code.

As implemented:
1. `cmd_vel_arbiter` inputs:
   `/cmd_vel_manual`, `/cmd_vel_auto` (plus configured auto topics), `/manual_override`.
2. Arbitration logic:
   - `manual_override=true`: manual if fresh, else stop.
   - `manual_override=false`: manual (fresh) > auto (fresh) > stop.
3. Timeout behavior:
   `manual_cmd_timeout` and `auto_cmd_timeout` (default `0.35s` each).
4. Source-switch safety:
   optional zero frame on source switch (`stop_on_source_switch=true` by default).
5. Low-level directional obstacle blocking remains in `nav2_serial_bridge`.

Validation still required on robot:
1. takeover latency (`<= 150 ms`) and stopping distance (`<= 0.20 m`) need hardware measurement.

---

## P1: Mission State Machine Skeleton (No Frontier Yet)

Status: Implemented in code.

As implemented:
1. `mission_orchestrator` provides:
   `start_mission`, `pause_mission`, `resume_mission`, `finish_mapping`.
2. P1 state flow:
   `BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`.
3. `BOOT` captures `home_pose` from TF (`map -> base_link`) after `boot_capture_delay_sec`.
4. `AUTO_MAP_V1` completion conditions:
   - `mapping_timeout_sec`
   - `mapping_max_distance_m`
   - `finish_mapping` service request
5. Manual override behavior:
   mission pauses/cancels active action on override and auto-resumes on release.
6. Map naming behavior:
   collision-safe unique naming when output artifact names already exist.

---

## P2: Frontier Explorer Integration

Status: Implemented in code.

As implemented:
1. `frontier_explorer` subscribes `/map`, extracts frontiers as:
   unknown (`-1`) cells adjacent to free (`0`) cells.
2. Frontier clustering uses BFS connected components.
3. Reachability and path cost are grid-based:
   - BFS distance field from robot free cell
   - path length = `steps * map_resolution`
   - optional filter `max_candidate_path_len_m` (0 disables cap)
4. Candidate score:
   `score = w_gain * info_gain - w_path * path_len - w_risk * risk`.
5. Failure handling:
   goal timeout cancel, temporary blacklist, repeated-failure cooldown.
6. Stop conditions:
   - no-frontier rounds limit
   - low new-area ratio over time window
   - max exploration time
7. Mission integration:
   started/stopped by orchestrator via `/frontier_explorer/start` and `/frontier_explorer/stop`.

---

## P3: Return Home + Save/Export Automation

Status: Implemented in code.

As implemented:
1. `RETURN_HOME` dispatches `NavigateToPose(home_pose)` with bounded retries (`home_retry_limit`).
2. `SAVE_EXPORT` pipeline:
   - save pbstream via `/write_state`
   - export map via `cartographer_pbstream_to_ros_map`
3. Success path transitions to `LOCALIZE_READY`.
4. Failure path transitions to `ERROR` with reason logging.

---

## P4: Semantic Shelf Overlay

Status: Implemented in code (V1 + V2 style alignment support).

As implemented:
1. `semantic_overlay` loads shelves from YAML/JSON (`shelf_id`, `x`, `y`, `yaw`).
2. Services:
   - `/semantic_overlay/query_shelf_pose`
   - `/semantic_overlay/set_alignment` (manual two-point transform)
   - `/semantic_overlay/reload`
   - `/semantic_overlay/auto_align` (automatic map-pattern alignment trigger)
3. Marker publication behavior:
   transformed markers and labels in map frame.
4. Marker cleanup behavior:
   `DELETEALL` on refresh path + stale marker delete to avoid RViz residue after shelf changes.
5. Automatic alignment behavior:
   - subscribes current `/map` occupancy grid
   - loads reference map pattern from `reference_map_yaml` (yaml + pgm)
   - local search around current transform (yaw/translation window)
   - optional coarse global fallback search and one-step refinement
   - startup retry mode with `auto_align_on_start=true`
6. Manual auto-align service success now clears pending startup retries to avoid duplicate re-runs.

---

## P5: Parameter Stabilization + Regression

Status: Implemented for profile workflow; runtime regression tracking remains manual.

As implemented:
1. Two Nav2 profile files are present:
   - `config/nav2_params_explore_slow.yaml`
   - `config/nav2_params_task_run.yaml`
2. `nav_assistant frontier-mission` supports:
   - `--explore-profile slow|task`
   - optional explicit `--nav2-params-file` override
3. Profile comparison tool:
   `ros2 run robot_navigation nav_profile_report --base ... --target ... --output ...`
4. Automatic regression monitor daemon is not part of this version by decision.

Remaining acceptance work:
1. CPU/memory/frequency/dynamic-obstacle metrics still require test-run measurement logs.

---

## 4. NAV2 Parameter Baseline and Tuning Guidance

Note:
"Current rationale / expected effect" is engineering inference from current values and stack behavior.

## 4.1 Costmap parameters

| Parameter Path | Current | Meaning | Current Rationale / Expected Effect | Suggested Tuning (Slow Exploration) |
|---|---:|---|---|---|
| `local_costmap.update_frequency` | `5.0` | local map update rate | usable dynamic response, moderate load | test `8~10` if CPU allows |
| `local_costmap.publish_frequency` | `2.0` | local map publish rate | RViz update only, not core control loop | keep |
| `local_costmap.width/height` | `3/3` | local window size (m) | near-field focus, short lookahead | increase to `4~5` for exploration |
| `local_costmap.resolution` | `0.03` | local grid resolution | better precision, higher load | keep `0.03` |
| `local_costmap.robot_radius` | `0.40` | collision envelope | conservative, safer clearance | test `0.42~0.45` if occlusion risk is high |
| `inflation_radius` | `0.55` | obstacle inflation radius | conservative pathing | keep or increase to `0.60` |
| `cost_scaling_factor` | `3.0` | inflation decay slope | balanced | lower if edge-scraping, higher if too conservative |
| `voxel_layer.z_resolution/z_voxels` | `0.05 / 16` | 3D voxelization | sufficient for current platform | keep |
| `cloud.obstacle_max_range` | `2.5` | obstacle observation range | near-mid range focus | consider `3.0` in larger spaces |
| `cloud.raytrace_max_range` | `3.0` | clearing raytrace range | supports obstacle clearing at mid range | keep |
| `global_costmap.resolution` | `0.03` | global grid resolution | fine global planning, more load | fallback `0.05` only if compute constrained |
| `global_costmap.update_frequency` | `1.0` | global map update rate | good for mostly static global view | keep |
| `track_unknown_space` | `true` | unknown space tracking | required for frontier behavior | must remain `true` |

## 4.2 Planner and controller parameters

| Parameter Path | Current | Meaning | Current Rationale / Expected Effect | Suggested Tuning (Slow Exploration) |
|---|---:|---|---|---|
| `GridBased.plugin` | `NavFnPlanner` | global planner | stable baseline | keep |
| `GridBased.use_astar` | `false` | Dijkstra/A* switch | robust and predictable | keep `false` first, benchmark `true` later |
| `GridBased.tolerance` | `0.5` | goal tolerance for planning | easier convergence in clutter | reduce to `0.2~0.3` for tighter terminal accuracy |
| `controller_frequency` | `20.0` | controller loop rate | standard, should be monitored under load | reduce to `15` only if needed |
| `FollowPath.max_vel_x` | `0.26` | max forward speed | medium speed | reduce to `0.15~0.20` for exploration |
| `FollowPath.max_vel_y` | `0.20` | max lateral speed | holonomic enabled | reduce to `0.08~0.15` for exploration |
| `FollowPath.max_vel_theta` | `1.0` | max angular speed | aggressive turning | reduce to `0.6~0.8` for exploration |
| `acc_lim_x/y/theta` | `2.5/1.5/3.2` | acceleration limits | responsive but may be sharp | reduce for smoother low-risk motion |
| `decel_lim_x/y/theta` | `-2.5/-1.5/-3.2` | deceleration limits | strong braking | tune with accel as a pair |
| `vx/vy/vtheta_samples` | `20/15/20` | trajectory sampling density | good quality, heavier compute | start with `15/10/15` for exploration |
| `sim_time` | `1.7` | rollout horizon | medium lookahead | tune in `1.5~2.0` range |
| `BaseObstacle.scale` | `0.02` | obstacle critic weight | may under-weight obstacle penalty | raise to `0.05~0.10` first |
| `PathAlign/PathDist` | `32/32` | path adherence pressure | can over-stick to path | lower slightly in dynamic crowds |
| `GoalDist.scale` | `24` | goal-seeking pressure | good convergence | lower if overshoot/shortcut behavior appears |

## 4.3 Progress, recovery, smoother

| Parameter Path | Current | Meaning | Current Rationale / Expected Effect | Suggested Tuning (Slow Exploration) |
|---|---:|---|---|---|
| `progress_checker.required_movement_radius` | `0.5` | required displacement to count progress | may be too large in tight spaces | reduce to `0.2~0.3` |
| `progress_checker.movement_time_allowance` | `10.0` | progress timeout window | moderate | increase to `12~15` in crowded areas |
| `goal_checker.xy/yaw_goal_tolerance` | `0.25/0.25` | final pose tolerance | practical for navigation | keep |
| `behavior_plugins` | `spin/backup/drive_on_heading/wait/assisted_teleop` | recovery behaviors | complete baseline set | prioritize `wait + spin + replan` |
| `velocity_smoother.feedback` | `OPEN_LOOP` | smoothing feedback mode | simple and stable | keep initially |
| `velocity_smoother.max_velocity` | `[0.26,0.20,1.0]` | velocity limits for smoothing | aligned with DWB | lower with exploration profile |

## 4.4 AMCL status

| Parameter Path | Current | Note |
|---|---:|---|
| `amcl.tf_broadcast` | `false` | prevents `map->odom` conflict with Cartographer |
| `amcl.scan_topic` | `/scan_fullframe` | config exists, but Cartographer is active localization owner |

---

## 5. EKF + Cartographer Fusion Chain

## 5.1 Current configuration summary

1. EKF fuses `/odom_raw` and IMU into `/odom`.
2. Cartographer uses odom prior (`use_odometry=true`).
3. Cartographer does not directly consume IMU (`use_imu_data=false`).

## 5.2 Guidance

1. Keep layered fusion as-is for now.
2. Prioritize timestamp consistency, TF integrity, and covariance realism.
3. Evaluate Cartographer `use_imu_data=true` only after baseline stability is proven and drift cases justify it.

---

## 6. Implemented Module Details

## 6.1 `cmd_vel_arbiter`

Implemented behavior:
1. Inputs:
   `/cmd_vel_manual`, configured auto topics (`/cmd_vel_auto`, `/cmd_vel_nav`, `/cmd_vel_smoothed` by default), and `/manual_override`.
2. Output:
   `/cmd_vel`.
3. Priority:
   - `manual_override=true`: manual if fresh, else stop.
   - `manual_override=false`: manual if fresh, else auto if fresh, else stop.
4. Safety guards:
   - source timeout stop
   - optional stop frame on source switch (`stop_on_source_switch=true`)
   - output topic is filtered out from auto source list to avoid self-loop.

## 6.2 `mission_orchestrator`

Implemented responsibilities:
1. Own mission state transitions and action sequencing.
2. Handle preemption:
   - on manual override: pause and cancel active action
   - on release: auto resume when pause reason is manual override.
3. Support two mapping modes:
   - `fixed`: `AUTO_MAP_V1`
   - `frontier`: `AUTO_EXPLORE`

Core interfaces:
1. Services:
   `start_mission`, `pause_mission`, `resume_mission`, `finish_mapping`.
2. Action clients:
   `navigate_to_pose`, `follow_waypoints` (namespace-aware resolution).
3. Service clients:
   `/write_state`, `/frontier_explorer/start`, `/frontier_explorer/stop`.

## 6.3 `frontier_explorer`

Implemented algorithm:
1. Frontier extraction:
   unknown cell adjacent to free cell.
2. Clustering:
   BFS connected components.
3. Candidate selection:
   centroid-proximal reachable free neighbor.
4. Reachability/path cost:
   BFS distance field on free-space grid from robot cell.
5. Score:
   `score = w_gain * info_gain - w_path * path_len - w_risk * risk`.
6. Failure policy:
   timeout cancel, blacklist with TTL, repeated-failure cooldown.
7. Stop conditions:
   no-frontier rounds, low-new-area window, max explore time.

Public interfaces:
1. Services:
   `/frontier_explorer/start`, `/frontier_explorer/stop`.
2. Topics:
   `/frontier_explorer/state`, `/frontier_explorer/done`, `/frontier_explorer/current_goal`.

## 6.4 `auto_frontier_mission.launch.py`

Implemented composition:
1. `slam_mapping_stack.launch.py` (includes `cmd_vel_arbiter`, `nav2_serial_bridge`, Cartographer, EKF, optional ultrasonic/collision launch).
2. Nav2 bringup.
3. `frontier_explorer`.
4. `mission_orchestrator` in frontier mode.
5. Optional `semantic_overlay` (toggle by `with_semantic_overlay`).

Implemented parameter wiring highlights:
1. Frontier robustness parameters (goal timeout, blacklist, recovery, path-length cap).
2. Mission save/export parameters.
3. Semantic overlay parameters including automatic alignment tuning.

## 6.5 `semantic_overlay`

Implemented responsibilities:
1. Load shelves from YAML/JSON.
2. Apply rigid alignment transform and publish markers.
3. Provide shelf pose query and alignment services.
4. Provide automatic map-pattern alignment against `/map`.
5. Clear stale markers after reload/config changes.

Implemented interfaces:
1. Services:
   `/semantic_overlay/query_shelf_pose`,
   `/semantic_overlay/set_alignment`,
   `/semantic_overlay/auto_align`,
   `/semantic_overlay/reload`.
2. Topic:
   `/semantic_overlay/markers`.

## 6.6 `nav_assistant` and P5 profile flow

Implemented behavior:
1. `mission-p1` and `frontier-mission` one-command launch paths.
2. `frontier-mission` profile selection:
   `--explore-profile slow|task` with optional explicit nav2 param file.
3. Frontier mission wiring includes semantic auto-alignment CLI options.
4. P5 comparison support:
   `nav_profile_report` compares profile YAMLs and emits markdown report.

---

## 7. Validation Plan and Acceptance Criteria

Primary executable checklist for current code:
`ROBOT_AUTONOMY_FRONTIER_P5_ACCEPTANCE_CHECKLIST.md`

## 7.1 Perception and localization chain

Checks:
1. `/cloud_all_fields_fullframe`, `/sick_scansegment_xd/imu` alive
2. `/odom_raw`, `/odom` continuous
3. TF chain complete: `map -> odom -> base_link -> lidar_link`

Commands:
```bash
ros2 topic hz /cloud_all_fields_fullframe
ros2 topic hz /sick_scansegment_xd/imu
ros2 topic hz /odom_raw
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

Pass criteria:
1. no persistent TF conflicts/timeouts
2. `/odom` rate stable near EKF target rate

## 7.2 `cmd_vel_arbiter` (manual takeover)

Checks:
1. trigger `manual_override=true` while auto motion is active
2. robot stops auto command quickly
3. manual command controls platform
4. release override and verify auto resume

Pass criteria:
1. takeover latency <= 150 ms
2. stopping distance <= 0.20 m (slow profile)
3. no authority flapping between manual and auto

## 7.3 Frontier exploration

Checks:
1. new exploration goals are continuously generated
2. failed goals enter temporary blacklist
3. stop condition triggers as designed

Pass criteria:
1. no persistent loop for 10+ minutes
2. map coverage grows and then converges

## 7.4 Return home and map artifacts

Checks:
1. `home_pose` recorded correctly
2. return-home execution succeeds
3. `.pbstream + .yaml + .pgm` files generated

Pass criteria:
1. return-home success >= 95% (10 runs)
2. final error `xy <= 0.15 m`, `yaw <= 10 deg`

## 7.5 Dynamic obstacles and recovery behavior

Checks:
1. robot slows/reroutes/waits under pedestrian interference
2. ultrasonic directional blocking still works
3. recovery sequence avoids unsafe backward maneuvers under blind-zone conditions

Pass criteria:
1. zero collision events in test runs
2. stuck-recovery success >= 90%

## 7.6 Performance regression at 0.03 resolution

Checks:
1. CPU and memory stability over long run
2. controller/planner frequency stability
3. no persistent planning timeout storms

Pass criteria:
1. 30-minute run without instability
2. no sustained high-frequency oscillation

---

## 8. Suggested Milestone Deliverables

| Milestone | Scope | Current status |
|---|---|---|
| `M1` | `cmd_vel_arbiter + manual_override + takeover path` | Implemented; hardware acceptance still required |
| `M2` | `mission_orchestrator` fixed mode + return-home + save/export | Implemented |
| `M3` | `frontier_explorer` + failure handling + stop conditions | Implemented |
| `M4` | `auto_frontier_mission.launch.py` one-command startup | Implemented |
| `M5` | `semantic_overlay` V1 + V2-style auto alignment | Implemented |
| `M6` | profile stabilization and regression evidence | Partially implemented (profile/report tooling done; runtime metric collection manual) |

---

## 9. Risks and Mitigations

1. LiDAR blind sector from physical occlusion:
   keep ultrasonic low-level gating, prioritize forward motion and spin-based re-observation.
2. Authority switching instability:
   enforce source lock + timeout in arbiter.
3. Higher compute load at `0.03`:
   start with slow profile, increase speed only after timing stability is proven.
4. Frontier dead loops:
   use blacklist TTL + failover logic.
5. Return-home failures:
   add bounded retries and intermediate waypoint fallback.

---

## 10. Conclusion

The adaptation path is now implemented end-to-end through the mission runtime chain:

`EKF + Cartographer + Nav2 + cmd_vel arbitration + Mission Orchestrator + Frontier + Semantic Overlay`

Current remaining work is primarily acceptance and tuning evidence, not core module creation:
1. hardware metrics for takeover latency and stopping distance,
2. multi-run return-home/error statistics,
3. performance and dynamic-obstacle regression evidence for both Nav2 profiles.
