# Logbook Prep: Nav2 <-> Jetson <-> STM32 Serial Chain

Purpose: provide a ready-to-append record format for future debugging sessions, with the first post-pull entry pre-filled.

## 1) Record Template

Copy this block for each new session:

```md
## Session YYYY-MM-DD HH:MM (TZ)

- Branch:
- HEAD:
- Goal:
- Runtime setup:
  - LiDAR launch args:
  - Cartographer config:
  - Nav2 params file:
  - Serial bridge args:
- Expected behavior:
- Actual behavior:
- Key evidence:
  - `/cmd_vel` observed?:
  - STM32 command parse counter:
  - Telemetry valid frames / bad checksum:
  - TF tree summary:
- Root-cause hypothesis:
- Actions taken:
- Result after action:
- Next action:
```

## 2) Current Entry (Post-Pull)

## Session 2026-02-08 22:38 (-05:00)

- Branch: `robot_navigation`
- HEAD: `5175c58e08702e6a18e8c59bd9ca85ac66ebb83c`
- Goal:
  - verify whether pulled GitHub updates resolve frame mismatch and command path.
- Runtime setup:
  - TF convention switched to `map -> odom -> world`.
  - serial bridge configured with explicit telemetry args (`header/channels/checksum`) and switches UP.
  - Cartographer/Nav2 configs aligned to `world` as robot base frame.
- Expected behavior:
  - stable telemetry decoding (no persistent frame mismatch)
  - `/cmd_vel` drives chassis motion
  - complete chain operational.
- Actual behavior:
  - frame mismatch issue still present in Jetson <-> STM32 Serialplot path
  - `/cmd_vel` still not reliably driving chassis
  - full chain still not run through successfully.
- Key evidence:
  - code-level changes exist (parser hardening + TX busy guard), but runtime objective still unmet.
- Root-cause hypothesis:
  - protocol sync remains brittle on shared UART path
  - no ACK-level confirmation that STM32 accepted latest command frame
  - frame naming fix (`world`) is orthogonal to serial decoding reliability.
- Actions taken:
  - reviewed post-pull diffs and updated runbook assumptions
  - recorded mismatch between expected fixed protocol and actual runtime behavior.
- Result after action:
  - diagnosis is clearer, but issue not fixed.
- Next action:
  - move from ad-hoc checks to explicit protocol+instrumentation rework.

## 3) Open Problem Statement (Keep This Updated)

- P0: Jetson <-> STM32 telemetry frame mismatch not solved.
- P0: `/cmd_vel` command path to motor actuation not solved.
- P0: End-to-end Nav2 control loop not solved.
- Impact:
  - navigation stack cannot be considered integrated.
  - current codebase still requires major communication-layer refactor.

## 4) Next Session Checklist

Before testing:

1. capture exact launch commands used in that run.
2. capture serial bridge logs (including counters/warnings).
3. capture STM32-side counters (valid frame count, checksum error count, PC status).
4. capture one `tf2_tools view_frames` output artifact.

Pass criteria for session closure:

1. command parse counter increases with `/cmd_vel`.
2. chassis wheel current/rpm responds to command.
3. telemetry decode remains stable for continuous run (no persistent mismatch).
4. TF tree remains connected and conflict-free.

