# Base Control v2 + Nav2 Baseline (Pre-Refactor)

This file records the current baseline before major Nav2 refactoring.

## 0) Snapshot Metadata

- Captured at: `2026-02-08 22:30:25 -05:00`
- Git branch: `robot_navigation`
- Git HEAD: `cca45fb48d92c31d6fa608ad0455f03d5f9c92e9`
- Note: working tree already contains local modifications when this snapshot was taken.

---

## 1) Current TF / frame_id Baseline

### 1.1 Configured frame IDs (from code/config)

- Nav2 serial bridge defaults:
  - `frame_id = odom`
  - `child_frame_id = base_link`
  - `publish_tf = false`
  - Source: `STM32/Base_Control_v2/robowalker2024bottominfantry-main/tools/nav2_serial_bridge.py`

- Runbook launch command for serial bridge sets:
  - `-p frame_id:=odom`
  - `-p child_frame_id:=base_link`
  - `-p publish_tf:=false`
  - Source: `Nav/README_SLAM_UPDATED.md`

- Cartographer config (`pico_2d.lua` and `pico_2d_localization.lua`):
  - `map_frame = "map"`
  - `odom_frame = "odom"`
  - `tracking_frame = "base_link"`
  - `published_frame = "base_link"`
  - `provide_odom_frame = true`
  - Sources:
    - `Nav/carto_cfg/pico_2d.lua`
    - `Nav/carto_cfg/pico_2d_localization.lua`

- Nav2 params expect:
  - `global_frame = map`
  - `odom_frame = odom`
  - `robot_base_frame = base_link`
  - Source: `Nav/nav2_params_cartographer.yaml`

- SICK launch command in repo docs uses:
  - `publish_frame_id:=base_link`
  - Source: `Nav/carto_cfg/command_slam.txt` and `Nav/README_SLAM_UPDATED.md`

### 1.2 TF ownership in current runbook

With the documented startup (`publish_tf:=false` on serial bridge):

- `map -> odom`: produced by Cartographer
- `odom -> base_link`: produced by Cartographer (`provide_odom_frame = true`, `published_frame = base_link`)
- Serial bridge publishes `/odom` message only (no TF broadcast)

Expected chain in this setup:

```text
map
`-- odom
    `-- base_link
```

### 1.3 Historical observed TF snapshot in repo

Saved `view_frames` outputs show:

- `map -> odom`
- `odom -> world`

Sources:

- `Nav/carto_cfg/frames_2026-01-28_16.50.58.gv`
- `Nav/carto_cfg/frames_2026-01-28_17.16.34.gv`

Important mismatch:

- Historical capture child frame was `world`, while current config/runbook expects `base_link`.
- This should be re-verified immediately after pull/refactor.

### 1.4 Quick post-change TF verification commands

```bash
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
```

Checks:

1. `/odom.header.frame_id` should be `odom`
2. `/odom.child_frame_id` should be `base_link`
3. TF graph should include connected `map -> odom -> base_link`
4. Only one node should own `odom -> base_link` at runtime

---

## 2) Base_Control_v2 Code Structure Understanding

Scope: `STM32/Base_Control_v2/robowalker2024bottominfantry-main`

### 2.1 Top-level layering

- `test/`: STM32 firmware target actually executed on board
- `tools/`: host-side Python tools (`pc_control.py`, `nav2_serial_bridge.py`, `teleop_cmd_vel.py`)
- `docs/`: architecture/protocol notes (`CONTROL_LOGIC.md`, `INTERFACES_PC_CAN.md`, `LOGBOOK.md`)
- `reference/`: reference assets and legacy material

### 2.2 Runtime entry and scheduler (firmware side)

- Entry:
  - `test/Core/Src/main.c`
  - Calls `Task_Init()` once, then `Task_Loop()` in while loop

- Scheduler/callback hub:
  - `test/User_File/5_Task/tsk_config_and_callback.cpp`
  - Initializes:
    - CAN1 callback routing
    - UART2 callback for PC frame parsing + Serialplot fallback
    - TIM4/TIM5 periodic callbacks
  - Main periodic logic in `Task1ms_TIM5_Callback()`:
    - 100 ms alive checks
    - 1 s alive checks
    - 10 ms interaction callback
    - 2 ms chassis resolution/control callback
    - 1 ms PC parse + chassis control callback
    - 5 ms telemetry publish (6 float channels, 200 Hz)
    - 1 ms CAN send / UART / watchdog service callbacks

### 2.3 Core control classes and responsibilities

- `Class_PC` (`test/User_File/2_Device/PC/dvc_pc.*`)
  - Parses UART2 host frame (`0xAC + payload + checksum`)
  - Converts keyboard/switch raw bits into DR16-style states
  - Has 100 ms alive watchdog; timeout disables control and triggers UART reinit

- `Class_Robot` (`test/User_File/4_Interaction/ita_robot.*`)
  - High-level policy for chassis-only build
  - Reads `Class_PC` outputs, applies deadzone and slope planning
  - Sets chassis velocity/omega targets
  - Safety gate: disable chassis if PC offline or left switch down

- `Class_Chassis` (`test/User_File/3_Chariot/1_Module/Chassis/crt_chassis.*`)
  - 2 ms loop:
    - `Self_Resolution()` (mecanum forward kinematics from wheel omega)
    - `Kinematics_Inverse_Resolution()` (target wheel omegas)
    - `Output_To_Dynamics()` (outer PID on vx/vy/omega)
    - `Dynamics_Inverse_Resolution()` (target wheel current)
    - `Output_To_Motor()` (apply current to 4 wheel motors)
  - Uses wheel sign compensation map:
    - `Wheel_Direction = {1, -1, 1, -1}` (FL/FR/RL/RR)
  - CAN command sent to ID `0x200`; feedback from `0x201`..`0x204`

### 2.4 Host-ROS integration data path

Forward path:

```text
Nav2 /cmd_vel
-> nav2_serial_bridge.py
-> UART2 host frame (0xAC, 26 bytes)
-> Class_PC parse
-> Class_Robot _Chassis_Control
-> Class_Chassis target wheel currents
-> CAN1 0x200
-> C620/M3508 motors
```

Feedback path:

```text
Motor feedback (CAN)
-> chassis odom estimate in firmware
-> Serialplot telemetry (0xAB, floats)
-> nav2_serial_bridge.py telemetry parser
-> /odom publish (frame_id, child_frame_id configurable)
-> optional TF broadcast if publish_tf=true
```

### 2.5 Coupling points to watch during Nav2 refactor

1. TF single ownership:
   - Avoid dual publishers for `odom -> base_link` (Cartographer vs serial bridge).

2. Frame naming consistency:
   - Keep `map`, `odom`, `base_link` aligned across Cartographer, Nav2 params, SICK launch args, and odom message fields.

3. Odom message contract:
   - Ensure `/odom` keeps stable `header.frame_id` and `child_frame_id`.

4. Command sign conventions:
   - Serial bridge maps ROS `Twist` into PC protocol with sign flips for `left_x` and `yaw`; keep this consistent with chassis behavior.

5. Velocity limits alignment:
   - Nav2 planner/controller velocity limits and bridge normalization limits should remain physically consistent.

---

## 3) Before/After Comparison Template

Use this table after pull/refactor:

| Item | Before (this file) | After |
| --- | --- | --- |
| Serial bridge `frame_id` | `odom` | |
| Serial bridge `child_frame_id` | `base_link` | |
| Serial bridge `publish_tf` | `false` (runbook) | |
| Cartographer `map_frame` | `map` | |
| Cartographer `odom_frame` | `odom` | |
| Cartographer `published_frame` | `base_link` | |
| Observed TF chain | `map -> odom -> base_link` (expected) / historical `.gv`: `map -> odom -> world` | |
| Nav2 `robot_base_frame` | `base_link` | |
| Nav2 odom topic | `/odom` | |
