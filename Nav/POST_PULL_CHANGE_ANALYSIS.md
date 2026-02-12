# Post-Pull Change Analysis (Base Control v2 + Nav2)

This document compares the pre-pull baseline against the current pulled state and records what changed vs what is still broken.

## 1) Comparison Scope

- Before reference: `Nav/BASE_CONTROL_V2_NAV2_BASELINE.md` (captured `2026-02-08 22:30:25 -05:00`)
- After snapshot time: `2026-02-08 22:38:57 -05:00`
- Current branch: `robot_navigation`
- Current HEAD: `5175c58e08702e6a18e8c59bd9ca85ac66ebb83c`

## 2) High-Level Verdict

- GitHub pull introduced real changes in TF naming and serial robustness/debugging.
- However, the core bringup chain is still not operational end-to-end.
- Confirmed unresolved status (from current test outcome):
  - Jetson <-> STM32 Serialplot still has frame mismatch issues.
  - `/cmd_vel` path still cannot reliably drive the chassis.
  - Therefore the full control/feedback chain is still not running and requires substantial rework.

## 3) What Changed

### 3.1 TF / frame convention changed to `world` as robot base

Compared with the previous baseline (`base_link`), current configs are now aligned to `world`:

- `Nav/README_SLAM_UPDATED.md`
  - TF chain documented as `map -> odom -> world`
  - robot base frame documented as `world`
  - serial bridge launch example uses `-p child_frame_id:=world`
- `Nav/carto_cfg/pico_2d.lua`
  - `tracking_frame = "world"`
  - `published_frame = "world"`
- `Nav/carto_cfg/pico_2d_localization.lua`
  - `tracking_frame = "world"`
  - `published_frame = "world"`
- `Nav/nav2_params_cartographer.yaml`
  - `base_frame_id: "world"`
  - `robot_base_frame: world`

Net effect:

- Current intended TF chain is now:
  - `map -> odom -> world`

### 3.2 Serial bridge (`nav2_serial_bridge.py`) got stronger diagnostics and parser hardening

Key additions:

- Added telemetry plausibility thresholds:
  - `telemetry_max_linear_abs`
  - `telemetry_max_angular_abs`
- Added runtime counters and warnings:
  - command RX/TX counters
  - telemetry OK/bad/outlier counters
  - periodic warnings for missing `/cmd_vel`
  - hints for telemetry format mismatch (baud/channels/checksum)
- Added handling for possible command-frame echo on RX line.
- Default `child_frame_id` is now `world`.

### 3.3 STM32 serialplot TX path added busy-state guard

In `STM32/Base_Control_v2/robowalker2024bottominfantry-main/test/User_File/2_Device/Serialplot/dvc_serialplot.cpp`:

- Added null checks for UART handle.
- Added `gState` readiness check before building/sending frame.
- Intended effect: avoid mutating TX buffer while previous IT transmission is still in progress.

### 3.4 Runbook updated for explicit telemetry/switch parameters

`Nav/README_SLAM_UPDATED.md` now explicitly pins:

- `telemetry_header:=[171]`
- `telemetry_channels:=6`
- `telemetry_checksum:=true`
- `left_switch:=1`
- `right_switch:=1`

Intent:

- reduce ambiguity between host bridge settings and firmware protocol.
- avoid chassis disable due to switch state.

## 4) What Did NOT Change / Still Broken

Even after the above updates, current result remains:

1. Serialplot frame mismatch persists between Jetson parser and STM32 telemetry stream.
2. `/cmd_vel` does not consistently result in motion at chassis level.
3. Full chain is not validated as working:
   - Nav2 planner/controller -> `/cmd_vel` -> serial bridge -> STM32 parse -> chassis control -> motor actuation
   - and reverse odom/telemetry feedback path.

## 5) Why the Problem Is Still Likely Open

Current code improves resilience, but does not fully close integration risk:

- Parser and firmware are still tightly coupled to exact byte-level assumptions (header, payload layout, checksum, timing).
- Shared UART link still carries bidirectional traffic; contention/echo/timing issues can remain under load.
- There is still no explicit command ACK handshake proving STM32 consumed the latest host command frame.
- Frame naming switched to `world`, but this alone does not solve serial protocol synchronization.

## 6) Required Next-Step Direction (Major Rework)

Recommended rework priorities:

1. Define one strict bidirectional protocol spec (version, msg type, length, seq, CRC).
2. Add explicit command ACK/status message from STM32 to Jetson.
3. Add deterministic host-side parser state machine tests with recorded UART byte logs.
4. Add STM32-side counters for:
   - valid command frames
   - checksum failures
   - last command age
5. Validate bringup by staged checkpoints:
   - checkpoint A: `/cmd_vel` reception counter increments on STM32
   - checkpoint B: target wheel current changes
   - checkpoint C: motor RPM changes
   - checkpoint D: odom telemetry decoded with zero frame drops over N seconds

---

This analysis should be read together with:

- `Nav/BASE_CONTROL_V2_NAV2_BASELINE.md`
- `Nav/LOGBOOK_PREP_NAV2_SERIAL_CHAIN.md`

