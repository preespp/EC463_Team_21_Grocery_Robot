# Robot Autonomy Frontier Adaptation Plan

## 0. Purpose

This plan defines how to evolve the current stack into a full "online mapping + online localization + autonomous exploration + manual override" workflow.

It includes:
1. Execution order and priority.
2. Current NAV2 parameter baseline from code, with rationale and expected behavior.
3. Design for unfinished modules (frontier, command arbitration, mission orchestration, semantic shelf overlay).
4. Validation methods and acceptance criteria for each module.

---

## 1. Current Baseline (Code-Verified)

## 1.1 Existing runtime pipeline

1. `robot_localization` EKF:
   `/odom_raw + /sick_scansegment_xd/imu -> /odom`
2. `Cartographer` (online SLAM):
   `use_odometry=true`, `use_imu_data=false`, scan matching + pose graph
3. `Nav2`:
   global planner `NavFnPlanner (use_astar=false)`, local controller `DWBLocalPlanner`
4. Low-level safety:
   ultrasonic alerts (`/front_alert`, `/back_alert`, `/left_alert`, `/right_alert`) directly limit motion in serial bridge
5. Resolution defaults in code:
   map export default `0.03`, Nav2 local/global costmap default `0.03`

## 1.2 Gaps still not implemented

1. No frontier exploration node in repository.
2. No strict command priority mux (`manual > auto`) for `/cmd_vel`.
3. No mission orchestrator state machine for end-to-end automation.
4. No top-level one-command launch for full autonomous mission.
5. No semantic shelf overlay module (`shelves.yaml/json` + transform alignment).

---

## 2. Target Architecture

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

Mission Orchestrator:
BOOT -> AUTO_EXPLORE -> RETURN_HOME -> SAVE_EXPORT -> LOAD_SEMANTIC -> READY_FOR_TASKS
```

---

## 3. Implementation Order and Priority

## P0 (Highest): Control Authority + Safety Closure

Goal:
Guarantee immediate manual takeover at any time.

Tasks:
1. Add `cmd_vel_arbiter`:
   `/cmd_vel_manual` high priority, `/cmd_vel_auto` low priority, output `/cmd_vel`.
2. Add `/manual_override` (`std_msgs/Bool`):
   - `true`: immediate takeover, zero-stop, cancel active auto navigation goal.
   - `false`: auto flow may resume.
3. Route all autonomous motion to `/cmd_vel_auto`.
4. Keep current ultrasonic low-level directional blocking unchanged.

Done criteria:
1. Manual takeover latency <= 150 ms.
2. Stopping distance after takeover <= 0.20 m at exploration speed profile.
3. Releasing override resumes autonomous flow without deadlock.

---

## P1: Mission State Machine Skeleton (No Frontier Yet)

Goal:
Make the full pipeline runnable with deterministic behavior before frontier complexity.

Tasks:
1. Add `mission_orchestrator`.
2. Implement state flow:
   `BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`.
3. `AUTO_MAP_V1` uses a closed-loop fixed scan trajectory in map frame.
4. Mapping completion conditions:
   - max time threshold
   - max traveled distance threshold
   - manual `finish_mapping` service trigger

Done criteria:
1. One command completes mapping + return-home + save/export.
2. Manual override works in any state and supports resume.

---

## P2: Frontier Explorer Integration

Goal:
Replace fixed-trajectory mapping with true frontier-driven exploration.

Tasks:
1. Add `frontier_explorer` node:
   - subscribe `/map`
   - frontier extraction: unknown cell adjacent to free cell
   - clustering: BFS connected components
   - candidate generation: centroid or nearest reachable point
   - reachability check: planner service or `ComputePathToPose`
   - scoring: information gain - path cost - risk cost
2. Failure handling:
   - timeout/no-progress: cancel goal, blacklist candidate
   - repeated failures: recovery chain (`wait/spin/replan`), then fallback
3. Exploration stop conditions:
   - `N` consecutive no-frontier rounds
   - map growth below threshold
   - maximum exploration time reached

Done criteria:
1. Robot continuously generates and executes autonomous exploration goals.
2. No persistent looping at the same frontier region.
3. Stop condition transitions correctly to `RETURN_HOME`.

---

## P3: Return Home + Save/Export Automation

Goal:
Reliably return to start and persist outputs.

Tasks:
1. Record `home_pose` (`map->base_link`) after startup stabilization window.
2. `RETURN_HOME` sends `NavigateToPose(home_pose)`.
3. `SAVE_EXPORT`:
   - save `.pbstream` via `/write_state`
   - export `.yaml/.pgm` via `cartographer_pbstream_to_ros_map`
   - timestamped naming to avoid overwrite

Done criteria:
1. Return-home success rate >= 95% across 10 runs.
2. Return-home error: `xy <= 0.15 m`, `yaw <= 10 deg`.
3. Generated map outputs are loadable by localization stack.

---

## P4: Semantic Shelf Overlay

Goal:
Overlay semantic shelf coordinates without modifying occupancy grid pixels.

Tasks:
1. Define `shelves.yaml/json` schema (`shelf_id -> pose`).
2. Add `semantic_overlay` node:
   - load reference semantic map
   - estimate `T_ref_map_to_new_map`
   - publish transformed shelf markers + query service
3. V1 alignment:
   same start pose assumption + manual 2-point calibration
4. V2 alignment:
   automatic registration (ICP or feature-based)

Done criteria:
1. Overlay error <= 0.20 m in V1.
2. Task layer can query shelf pose by `shelf_id` and send Nav2 goal.

---

## P5: Parameter Stabilization + Regression

Goal:
Stabilize navigation quality and compute load at `0.03` resolution.

Tasks:
1. Maintain two profiles:
   - slow exploration profile
   - normal task execution profile
2. Tune parameters incrementally (change 1-2 at a time with logs).
3. Track key metrics:
   - CPU/memory load
   - planner/controller timing stability
   - dynamic obstacle handling success
   - manual override responsiveness

Done criteria:
1. Control loop remains stable (no persistent drop below target frequencies).
2. Fewer oscillation/stall events under dynamic obstacles.
3. Reproducible baseline report for both profiles.

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

## 6. Unfinished Module Design

## 6.1 `cmd_vel_arbiter` (new)

Responsibilities:
1. Inputs: `/cmd_vel_manual`, `/cmd_vel_auto`, `/manual_override`.
2. Output: `/cmd_vel`.
3. Arbitration:
   - `manual_override=true`: pass manual only.
   - `manual_override=false`: manual still has active priority when present; timeout falls back to auto.
4. Safety:
   - source timeout -> send stop
   - source switch -> inject one stop frame to prevent command discontinuity

## 6.2 `mission_orchestrator` (new)

Responsibilities:
1. Own mission state transitions and cross-node action sequencing.
2. Handle preemption:
   - on manual override: pause/cancel active auto actions
   - on release: resume from a valid state

Core interfaces:
1. services/actions:
   `start_mission`, `pause_mission`, `resume_mission`, `finish_mapping`
2. action clients:
   `navigate_to_pose`, `follow_waypoints`
3. service client:
   `/write_state`

## 6.3 `frontier_explorer` (new)

Algorithm plan:
1. frontier extraction: `unknown(-1)` adjacent to `free(0)`
2. clustering: BFS connected components
3. target candidate: centroid + nearest reachable fallback
4. score:
   `score = w_gain * info_gain - w_path * path_len - w_risk * risk`
5. failure policy:
   - short-term blacklist with TTL
   - repeated failures trigger recovery chain

Default stop conditions:
1. `no_frontier_rounds >= 5`
2. `new_area_ratio < 1%` for `60s`
3. `max_explore_time` (e.g., 20 min)

## 6.4 `auto_frontier_mission.launch.py` (new)

Internal composition:
1. `slam_mapping_stack.launch.py` (`with_collision:=true` by default)
2. Nav2 bringup for exploration
3. `cmd_vel_arbiter`
4. `frontier_explorer` (toggleable)
5. `mission_orchestrator`

Recommended launch args:
1. `map_name:=run_<timestamp>`
2. `with_rviz:=true/false`
3. `explore_profile:=slow`

## 6.5 `semantic_overlay` (new)

Responsibilities:
1. load `shelves.yaml/json`
2. estimate `T_ref_map_to_new_map`
3. publish shelf markers + query service

V1:
1. same start pose assumption + manual 2-point calibration
2. target overlay error `<= 0.20m`

---

## 7. Validation Plan and Acceptance Criteria

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

1. `M1`: `cmd_vel_arbiter + manual_override + takeover test scripts`
2. `M2`: `mission_orchestrator` V1 (fixed trajectory) + return-home + auto save/export
3. `M3`: `frontier_explorer` + failover + stop conditions
4. `M4`: `auto_frontier_mission.launch.py` one-command startup
5. `M5`: `semantic_overlay` V1
6. `M6`: tuning and regression report (exploration profile vs task profile)

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

On the current codebase, the lowest-risk and highest-leverage path remains:

`EKF + Cartographer + Nav2 + Frontier + Safety + Mission Orchestrator`

The first mandatory step is not frontier itself, but command authority architecture (`cmd_vel` arbitration + manual override), because that is the control and safety foundation for all autonomous behavior.
