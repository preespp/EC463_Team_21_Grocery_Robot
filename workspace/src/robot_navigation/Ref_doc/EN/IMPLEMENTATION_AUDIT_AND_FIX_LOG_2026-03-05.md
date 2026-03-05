# Robot Navigation Implementation Audit and Fix Log (March 5, 2026)

## 1. Scope

This audit reviewed the implemented autonomy stack in `robot_navigation`, including:

- `cmd_vel_arbiter`
- `mission_orchestrator`
- `frontier_explorer`
- `semantic_overlay`
- `auto_map_mission_v1.launch.py`
- `auto_frontier_mission.launch.py`
- `nav_assistant` command flows

The goal was to check implementation status, explain runtime behavior, identify non-working/unclear points, and apply targeted fixes.

---

## 2. Current Job Status

Implementation status is strong and covers the planned path from P0 to P4:

- `P0`: Manual/auto command arbitration with override is implemented.
- `P1`: Mission orchestrator fixed-trajectory mode is implemented.
- `P2`: Frontier explorer node is implemented and integrated.
- `P3`: Return-home + map save/export automation is implemented.
- `P4`: Semantic shelf overlay and query/alignment services are implemented.

Build/sanity check after fixes:

- `python3 -m compileall ...`: pass
- `colcon build --packages-select robot_navigation`: pass

---

## 3. How It Works End-to-End

### 3.1 Motion authority and safety

1. Manual commands publish to `/cmd_vel_manual`.
2. Autonomous commands publish to `/cmd_vel_auto`, `/cmd_vel_nav`, `/cmd_vel_smoothed`.
3. `cmd_vel_arbiter` applies priority:
   - `manual_override=true`: manual only.
   - otherwise manual when active; fallback to auto.
4. Arbiter outputs unified `/cmd_vel`.
5. `nav2_serial_bridge` consumes `/cmd_vel` and applies low-level ultrasonic directional blocking before sending to STM32.

### 3.2 Mission orchestration

- `mission-p1` path:
  `BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`
- `frontier-mission` path:
  `BOOT -> AUTO_EXPLORE -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`

Mission control interfaces:

- `/start_mission`
- `/pause_mission`
- `/resume_mission`
- `/finish_mapping`

### 3.3 Frontier mode behavior

`frontier_explorer`:

- reads `/map`
- extracts unknown/free boundary frontiers
- clusters with BFS
- scores candidates by gain/path/risk
- sends `NavigateToPose`
- handles failures with timeout/blacklist/recovery cooldown
- stops when one of the stop conditions is reached

### 3.4 Semantic overlay

`semantic_overlay`:

- loads shelf data from YAML/JSON
- applies rigid transform alignment
- publishes markers in map frame
- supports:
  - shelf pose query service
  - 2-point alignment service
  - reload service

---

## 4. Issues Found and Fixed

## Fix A: Nav2 namespace action mismatch (functional)

### Problem

When `nav2_namespace` was non-empty, Nav2 action servers moved under namespace (for example `/nav2/navigate_to_pose`), but:

- `mission_orchestrator` still targeted root actions by default
- `frontier_explorer` still targeted root `navigate_to_pose`

This could cause repeated "waiting for action server" and blocked autonomy under namespaced bringup.

### Fix

Added namespace-aware action resolution in both nodes and passed namespace from launch files.

Changed files:

- `robot_navigation/mission_orchestrator.py`
- `robot_navigation/frontier_explorer.py`
- `launch/auto_map_mission_v1.launch.py`
- `launch/auto_frontier_mission.launch.py`

### Result

Both mission and frontier flows now resolve action names correctly when `nav2_namespace` is set or unset.

## Fix B: Empty action failure message in logs (observability)

### Problem

Action result logs often showed empty `msg=''` on failures, reducing debuggability.

### Fix

Added result summarization helpers to capture useful fields (`error_msg`, `error_code`, `message`, `missed_waypoints`) and include them in stored action results.

Changed files:

- `robot_navigation/mission_orchestrator.py`
- `robot_navigation/frontier_explorer.py`

### Result

Failure logs now include actionable details instead of blank messages in most cases.

---

## 5. Clarifications (No Code Change Required)

1. `home_retry_limit` is interpreted as retry budget after failures in practice (not just one total attempt).
2. `mission_orchestrator` now protects map artifact naming by generating a unique map name when collisions are detected.
3. `frontier_explorer` is service-driven in mission mode by default (`autostart_frontier=false`), and mission orchestrator triggers start/stop.

---

## 6. Recommended Test Procedure (Hardware)

### 6.1 P0 arbitration

```bash
ros2 run robot_navigation nav_assistant mapping-stack
ros2 run robot_navigation nav_assistant teleop --topic /cmd_vel_manual
ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /manual_override std_msgs/msg/Bool "{data: false}"
```

Expected: manual takeover immediate, auto resumes after override release.

### 6.2 P1 mission

```bash
ros2 run robot_navigation nav_assistant mission-p1 --interactive-override true
```

Expected state sequence on `/mission_state`:

`BOOT -> AUTO_MAP_V1 -> RETURN_HOME -> SAVE_EXPORT -> LOCALIZE_READY`

### 6.3 P2-P4 frontier mission

```bash
ros2 run robot_navigation nav_assistant frontier-mission --interactive-override true
```

Expected:

- frontier goals are produced
- mission transitions to `RETURN_HOME` when frontier reports done/stop condition
- map artifacts are saved/exported

### 6.4 Semantic overlay

```bash
ros2 service call /semantic_overlay/reload std_srvs/srv/Trigger "{}"
```

Query one shelf (replace `A1` with valid id):

```bash
ros2 service call /semantic_overlay/query_shelf_pose robot_interfaces/srv/QueryShelfPose "{shelf_id: 'A1'}"
```

---

## 7. Remaining Risk Notes

1. Full runtime validation still depends on hardware-in-loop behavior (LiDAR quality, serial latency, wheel odom quality, local clutter).
2. Lint/docstyle tests may still fail unless style rules are normalized across the package (not a functional blocker).

---

## 8. Additional Runtime Fix (March 5, 2026)

## Fix C: `nav2_serial_bridge` launch-time crash due parameter type mismatch

### Problem

In stack launch files, `nav2_serial_bridge` received:

- `cmd_topics := /cmd_vel` (string)

But the node originally declared `cmd_topics` as string-array only, which caused:

- `InvalidParameterTypeException` at startup
- `nav2_serial_bridge` process exit in mission launches

### Fix

1. `nav2_serial_bridge` now accepts dynamic typing for `cmd_topics` and normalizes:
   - single string
   - string list/array
   - unset (fallback to `cmd_topic`)
2. `slam_mapping_stack.launch.py` and `nav2_localization_stack.launch.py` keep explicit bridge command-topic wiring.

Changed files:

- `robot_navigation/robot_navigation/nav2_serial_bridge.py`
- `robot_navigation/launch/slam_mapping_stack.launch.py`
- `robot_navigation/launch/nav2_localization_stack.launch.py`

### Verification

After rebuild:

- `python3 -m compileall ...`: pass
- `colcon build --packages-select robot_navigation`: pass
- `ros2 launch robot_navigation auto_frontier_mission.launch.py ...` (smoke run):
  - `nav2_serial_bridge` stays alive
  - no `InvalidParameterTypeException`
  - `cmd_vel_arbiter`, `frontier_explorer`, and `mission_orchestrator` all initialize
