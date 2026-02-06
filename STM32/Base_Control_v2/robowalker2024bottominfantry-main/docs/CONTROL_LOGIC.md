# RoboWalker Bottom Infantry Control Logic Notes

This document summarizes the control logic, chassis kinematics, and PID usage for the STM32 project in this repo. It also lists executable vs separable code, required hardware, and the key callable APIs.

## Current implementation snapshot (chassis-only, PC UART2)
- Input: PC UART2 frame (0xAC header, 5 floats + key mask + switches, checksum) at 115200 baud (8-N-1).
- Scheduling: 1 ms PC parse + chassis target update, 2 ms chassis resolution/control, 1 ms CAN send + 5 ms Serialplot telemetry.
- Chassis: 4 mecanum, C620 current mode, outer PID on vx/vy/omega, wheel odom only (no IMU). Wheel direction mismatches (FR/RR wiring) are handled via a compile-time `Wheel_Direction` sign map when sampling encoder data and when issuing motor currents.

## Project layout and entry points
- Core entry point: `test/Core/Src/main.c` calls `Task_Init()` and `Task_Loop()`.
- Scheduler and callbacks: `test/User_File/5_Task/tsk_config_and_callback.cpp` wires CAN/UART interrupts to device callbacks and runs periodic control loops on TIM5.
- High level control: `test/User_File/4_Interaction/ita_robot.cpp` contains the mode logic and per subsystem control.
- Chassis kinematics and dynamics: `test/User_File/3_Chariot/1_Module/Chassis/crt_chassis.cpp`.
- PID algorithm: `test/User_File/1_Middleware/2_Algorithm/PID/alg_pid.cpp`.

## Executable control flow (what actually runs)
### Main loop
- `Task_Init()` initializes BSP power, CAN, UART, timers, watchdog, Serialplot, and all robot modules, then starts timers.
- `Task_Loop()` runs in the main while loop ~~and updates the referee UI~~ (currently empty stub).

### TIM5 1 ms scheduler
Executed in `Task1ms_TIM5_Callback()`:
- 1000 ms: `robot.TIM_1000ms_Alive_PeriodElapsedCallback()` (~~Referee, Manifold, Supercap alive checks~~ currently empty).
- 100 ms: `robot.TIM_100ms_Alive_PeriodElapsedCallback()` (~~DR16, Chassis, Gimbal, Booster, Posture~~ PC + Chassis alive checks).
- ~~100 ms: `robot.TIM_100ms_Calculate_Callback()` (power control PIDs for chassis power limits).~~
- 10 ms: `robot.TIM_10ms_Calculate_PeriodElapsedCallback()` (~~Manifold control + UART send, Supercap send~~ currently empty).
- 2 ms: `robot.TIM_2ms_Calculate_PeriodElapsedCallback()` (Chassis resolution + control loop).
- 1 ms: `robot.TIM_1ms_Calculate_Callback()` (~~status, supercap, chassis, gimbal, booster control, plus gimbal and posture updates~~ PC parse + chassis target update).
- Telemetry: Serialplot writes 6 channels each 5 ms in `Task1ms_TIM5_Callback()` (200 Hz).
- Driver layer each 1 ms: `TIM_1ms_CAN_PeriodElapsedCallback()`, `TIM_1ms_UART_PeriodElapsedCallback()`, `TIM_1ms_IWDG_PeriodElapsedCallback()`.

### Interrupt callbacks
- CAN FIFO callbacks in `drv_can.cpp` dispatch to `Device_CAN1_Callback()` ~~and `Device_CAN2_Callback()`~~.
- UART receive-to-idle callback in `drv_uart.cpp` dispatches to ~~`DR16_UART1_Callback()`, `Manifold_UART3_Callback()`, `Referee_UART6_Callback()`, `Chassis_AHRS_UART7_Callback()`, `Gimbal_AHRS_UART8_Callback()`~~ and now to `PC_UART2_Callback()` (falls through to Serialplot if not a PC frame).
- `Task100us_TIM4_Callback()` is currently empty (stub).

## Current data chain (PC UART2 -> CAN1 -> motors -> telemetry)
1. PC sends a 26-byte frame on UART2: header 0xAC + payload (5 floats: right_x/right_y/left_x/left_y/yaw, uint16 key mask, 2 switch bytes) + 8-bit checksum (sum of payload).
2. UART2 receive-to-idle DMA triggers `PC_UART2_Callback()`; `Class_PC::UART_RxCpltCallback()` parses the frame. Unmatched frames are passed to Serialplot parsing.
3. Each 1 ms, `Class_PC::TIM_1ms_Calculate_PeriodElapsedCallback()` updates key/switch edge states, then `Class_Robot::_Chassis_Control()` maps input to target `vx/vy` (m/s) and `omega` (rad/s) with slope planning.
4. Each 2 ms, `Class_Chassis::Self_Resolution()` computes odom (`vx/vy/omega`) from wheel speeds, then inverse kinematics computes target wheel omegas.
5. `PID_Velocity_X/Y/Omega` outputs are converted to per-wheel target current (with speed correction + resistance compensation).
6. `TIM_1ms_CAN_PeriodElapsedCallback()` sends CAN1 0x200 with 4 wheel currents. Motor feedback 0x201-0x204 updates wheel speed/power estimates.
7. Each 5 ms, Serialplot sends 6 float channels over UART2: target vx/vy/omega and now vx/vy/omega.

## Control logic (operator and subsystem behavior)
### Input sources
- ~~DR16: left stick for chassis translation, yaw channel for chassis rotation, right stick for gimbal.~~
- ~~Mouse and keyboard: gimbal fine aim, chassis WASD, mode toggles.~~
- ~~Manifold: auto aiming target angles and feedforward.~~
- ~~Referee: power, heat, and robot status constraints.~~
- PC UART2 frame: left_x/left_y for chassis translation, yaw for rotation; `tools/pc_control.py` maps WASD to left stick and Q/E to yaw.

### ~~Mode and status control (`Class_Robot::_Status_Control`)~~
~~Key behaviors:~~
- ~~Safety: if DR16 offline or left switch down, control is disabled.~~
- ~~Gyro mode: Q or E sets continuous spin (counterclockwise or clockwise). Releasing returns to follow mode.~~
- ~~Supercap accelerate: Shift enables acceleration, B enables burst acceleration.~~
- ~~Auto aim: right mouse enables Manifold aiming.~~
- ~~Manifold priority: V toggles armor vs rune, long left mouse hold forces armor.~~
- ~~With Ctrl+Shift held:~~
  - ~~Z reset MCU.~~
  - ~~C toggle supercap enable.~~
  - ~~Q toggle self color (also disables referee trust).~~
  - ~~X toggle referee trust.~~
  - ~~A toggle chassis type (HP vs POWER).~~
  - ~~S toggle booster type (BURST vs CD).~~
  - ~~E/D adjust robot level.~~
  - ~~R redraw UI.~~
- ~~Referee online updates self color and robot level from referee ID and level.~~

### Current mode and status control (chassis-only)
- Safety: if PC offline or left switch down, chassis control is disabled.
- Speed boost: Shift increases max chassis speed and omega (see `_Chassis_Control()`).
- No gyro/follow/supercap/referee/manifold modes in current build.

### ~~Supercap control (`Class_Robot::_Supercap_Control`)~~
- ~~Power limit for the supercap is set based on referee power limits or fallback tables.~~
- ~~Buffer energy, energy loop, and power compensation are set every tick.~~
- ~~Supercap is disabled if DR16 is offline, manually disabled, gimbal is offline, referee chassis power is offline, or buffer energy is too low.~~
- ~~Auto restart is allowed once buffer energy recovers above a threshold.~~

### Chassis control (`Class_Robot::_Chassis_Control`)
- ~~Power limit is derived from referee (if trusted) or fallback tables based on robot type and level.~~
- ~~If supercap is enabled and has normal energy, power limit is boosted (burst or normal acceleration).~~
- Power limit is fixed to 45.0f in current code (no referee/supercap inputs).
- PC offline or left switch down disables chassis control.
- Targets:
  - Translation from PC left stick (WASD in `tools/pc_control.py`), rotation from yaw (Q/E in `tools/pc_control.py`).
  - ~~Keyboard WASD adds full speed increments (when Ctrl is not pressed).~~
  - ~~Gyro mode adds constant rotation.~~
  - ~~Follow mode adds PID correction to align chassis yaw with gimbal yaw.~~
  - Yaw sign is inverted in code so Q = CCW and E = CW with the default PC script.
- Slope (ramp) control:
  - X/Y speeds are passed through slope planners using current chassis velocity (chassis frame).
  - Omega uses its own slope planner with chassis omega feedback.
- Chassis targets are set directly in chassis frame ~~after gimbal yaw feedforward compensation~~.

### ~~Gimbal control (`Class_Robot::_Gimbal_Control`)~~
- ~~DR16 offline or left switch down disables gimbal control.~~
- ~~Angle mode is used by default.~~
- ~~Right stick and mouse control yaw and pitch, with feedforward omega.~~
- ~~Auto aim (Manifold) overrides target angles and feedforward when enabled.~~
- ~~Chassis rotation feedforward is subtracted from yaw to maintain stability.~~

### ~~Booster control (`Class_Robot::_Booster_Control`)~~
- ~~A heat detector FSM estimates firing heat from friction wheel current if referee is unavailable.~~
- ~~Heat limits come from referee when trusted, otherwise from local tables.~~
- ~~DR16 offline or left switch down disables booster control.~~
- ~~Trigger logic:~~
  - ~~Right switch or mouse left triggers spot fire.~~
  - ~~Right switch down or long left mouse hold triggers auto fire.~~
  - ~~Referee heat limits adjust the maximum firing frequency.~~

## Chassis kinematics and dynamics
~~The chassis is a 4 wheel swerve drive with steer motors (GM6020) and wheel motors (C620). Key parameters:~~
- ~~Wheel radius: 0.058 m.~~
- ~~Wheel positions: 4 corners at 45, 135, 225, 315 degrees.~~
- ~~Distance to core: 0.207 m for each wheel.~~

Current chassis is a 4 wheel mecanum drive (C620 + M3508):
- Wheel radius: 0.0762 m.
- Half length: 0.1905 m.
- Half width: 0.3175 m.
- Wheel order: FL (ID1/0x201), FR (ID2/0x202), RL (ID3/0x203), RR (ID4/0x204).

### Forward kinematics (self resolution)
~~Computed in `Class_Chassis::Self_Resolution()`:~~
- ~~Vx = sum(omega_wheel * cos(steer_angle) * R) / 4~~
- ~~Vy = sum(omega_wheel * sin(steer_angle) * R) / 4~~
- ~~Omega = sum(omega_wheel * sin(steer_angle - wheel_azimuth) * R / distance) / 4~~
- ~~Pitch/roll from AHRS are used to compute slope direction vector.~~

Current (mecanum, using wheel omegas with direction signs `s_i`):
- Vx = R * (s_fl*w_fl + s_fr*w_fr + s_rl*w_rl + s_rr*w_rr) / 4
- Vy = R * (-s_fl*w_fl + s_fr*w_fr + s_rl*w_rl - s_rr*w_rr) / 4
- Omega = R * (-s_fl*w_fl + s_fr*w_fr - s_rl*w_rl + s_rr*w_rr) / (4 * (L + W))
- Pitch/roll are fixed to 0; slope direction is (0, 0, 1).

### Inverse kinematics
~~Computed in `Class_Chassis::Kinematics_Inverse_Resolution()`:~~
- ~~For each wheel i:~~
  - ~~vxi = Vx - Omega * dist * sin(azimuth_i)~~
  - ~~vyi = Vy + Omega * dist * cos(azimuth_i)~~
  - ~~wheel_speed = sqrt(vxi^2 + vyi^2) / R~~
  - ~~steer_angle = atan2(vyi, vxi)~~
- ~~Nearest transposition: if the steering change is more than 90 degrees, the wheel is flipped by PI and wheel speed sign is inverted to minimize steering rotation.~~

Current (mecanum, no steering):
- w_fl = (Vx - Vy - Omega * (L + W)) / R
- w_fr = (Vx + Vy + Omega * (L + W)) / R
- w_rl = (Vx + Vy - Omega * (L + W)) / R
- w_rr = (Vx - Vy + Omega * (L + W)) / R
  (direction signs are applied later when converting to motor commands)

### Dynamics inversion
~~Computed in `Class_Chassis::Dynamics_Inverse_Resolution()`:~~
- ~~PID outputs for Vx, Vy, Omega are treated as force and torque.~~
- ~~Per wheel force is computed in the wheel frame and converted to current.~~
- ~~Static and dynamic resistance compensation is applied based on wheel speed.~~

Current (mecanum):
- PID outputs for Vx/Vy/Omega are treated as force_x/force_y/torque.
- Per wheel current uses force contribution and speed error: `Target_Wheel_Current[i] = (force term) * R + Wheel_Speed_Limit_Factor * (Target_Wheel_Omega - Now_Omega)`.
- Dynamic resistance compensation is applied around `Wheel_Resistance_Omega_Threshold` using `Dynamic_Resistance_Wheel_Current[]`.
- Wheel direction signs are applied only when sampling `Now_Omega` and when commanding `Set_Target_Current()` so physical wiring differences do not skew control gains.

### Output and power limiting
- `Output_To_Motor()` sets wheel motors to current mode and runs motor PID loops (no steer motors).
- `_Power_Limit_Control()` uses per motor power estimates to scale wheel outputs; `Steer_Factor` remains 1.0 in this build.

## PID logic
### Algorithm (`Class_PID`)
- Error = Target - Now, with a configurable dead zone.
- P term = Kp * error.
- I term:
  - Optional variable speed integration (thresholds A and B).
  - Optional integral separation (only integrate below a threshold).
  - Optional integral clamp (`I_Out_Max`).
- D term:
  - Standard derivative on error, or derivative on measurement when D-first is enabled.
- F term = Kf * (Target - Pre_Target).
- Output is clamped by `Out_Max` when configured.

### Usage in this project
- Chassis: `PID_Velocity_X`, `PID_Velocity_Y`, `PID_Omega` regulate body velocity and yaw rate.
- ~~Chassis follow: `PID_Chassis_Follow` adds yaw correction to keep chassis aligned with gimbal.~~
- ~~Power control: `PID_Supercap_Chassis_Power` and `PID_Referee_Chassis_Power` adjust available chassis power.~~
- Motors: DJI motor drivers use cascaded loops (angle -> omega -> current).
- ~~GM6020 also supports a 2023 current control variant.~~

## Hardware dependencies
### MCU and peripherals
- STM32F427 (from startup and IOC configuration).
- ~~CAN1, CAN2 for motor and supercap buses.~~
- CAN1 for chassis wheel motors (0x201 to 0x204).
- ~~UART1/2/3/6/7/8 with DMA receive-to-idle for DR16, Serialplot, Manifold, Referee, and AHRS sensors.~~
- UART2 with DMA receive-to-idle for PC control input; UART2 IT TX for Serialplot telemetry.
- TIM4/TIM5 as software scheduler.
- IWDG watchdog.
- GPIO and BSP for board power and LEDs.

### External devices (by bus)
- CAN1:
  - ~~Booster driver motor (0x202).~~
  - ~~Booster friction motors (0x203, 0x204).~~
  - ~~Gimbal yaw/pitch motors (0x205, 0x206).~~
  - Chassis wheel motors (C620 + M3508) IDs 0x201 to 0x204.
- ~~CAN2:~~
  - ~~Chassis wheel motors (0x201 to 0x204).~~
  - ~~Chassis steer motors (0x205 to 0x208).~~
  - ~~Supercap (0x030).~~
- ~~UART1: DR16 remote control receiver.~~
- UART2: PC control + Serialplot.
- ~~UART3: Manifold (vision or auto aim compute unit).~~
- ~~UART6: Referee system.~~
- ~~UART7: Chassis AHRS (Wheeltec).~~
- ~~UART8: Gimbal AHRS (WIT).~~

## ~~Controller communication (DR16) - required code path~~
~~Minimum required pieces for DR16 to work:~~
- ~~`UART_Init(&huart1, DR16_UART1_Callback, 36)` in `Task_Init()`.~~
- ~~`drv_uart.cpp` receive-to-idle DMA handler (`HAL_UARTEx_RxEventCallback`) to dispatch the UART1 callback and re-arm DMA.~~
- ~~`DR16_UART1_Callback()` to call `Class_DR16::UART_RxCpltCallback()`.~~
- ~~`Class_DR16` parsing and its `TIM_100ms_Alive_PeriodElapsedCallback()` and `TIM_1ms_Calculate_PeriodElapsedCallback()` for status and edge detection.~~
- ~~Scheduler path from `Task1ms_TIM5_Callback()` so the above timers run.~~

## Controller communication (PC UART2) - required code path
Minimum required pieces for PC control to work (UART2 @ 115200 baud, 8-N-1):
- `UART_Init(&huart2, PC_UART2_Callback, SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH)` in `Task_Init()`.
- `drv_uart.cpp` receive-to-idle DMA handler (`HAL_UARTEx_RxEventCallback`) dispatches UART2 to `PC_UART2_Callback()` and re-arms DMA.
- `PC_UART2_Callback()` calls `Class_PC::UART_RxCpltCallback()`; if parsing fails, it forwards to Serialplot.
- `Class_PC::Parse_Frame()` validates header 0xAC + checksum and unpacks 5 floats + key mask + switches.
- `Class_PC::TIM_100ms_Alive_PeriodElapsedCallback()` and `Class_PC::TIM_1ms_Calculate_PeriodElapsedCallback()` run via `Task1ms_TIM5_Callback()`.

## Callable APIs (high level)
This is a condensed list of the most relevant APIs used by control logic:

- `Class_Robot` (in `ita_robot.h`)
  - `Init()`, `Loop()`
  - `TIM_1000ms_Alive_PeriodElapsedCallback()`
  - `TIM_100ms_Alive_PeriodElapsedCallback()`
  - ~~`TIM_100ms_Calculate_Callback()`~~
  - `TIM_10ms_Calculate_PeriodElapsedCallback()`
  - `TIM_2ms_Calculate_PeriodElapsedCallback()`
  - `TIM_1ms_Calculate_Callback()`

- `Class_Chassis` (in `crt_chassis.h`)
  - `Init()`
  - `Set_Chassis_Control_Type()`
  - `Set_Target_Velocity_X()`, `Set_Target_Velocity_Y()`, `Set_Target_Omega()`
  - `Set_Power_Limit_Max()`
  - `TIM_2ms_Resolution_PeriodElapsedCallback()`
  - `TIM_2ms_Control_PeriodElapsedCallback()`
  - `Get_Now_Velocity_X()`, `Get_Now_Velocity_Y()`, `Get_Now_Omega()`

- `Class_PC` (in `dvc_pc.h`)
  - `Init()`
  - `UART_RxCpltCallback()`
  - `TIM_100ms_Alive_PeriodElapsedCallback()`
  - `TIM_1ms_Calculate_PeriodElapsedCallback()`
  - `Get_*()` accessors for sticks, yaw, switches, and keyboard mask

- ~~`Class_Gimbal` (in `crt_gimbal.h`)~~
  - ~~`Init()`~~
  - ~~`Set_Gimbal_Control_Type()`~~
  - ~~`Set_Target_Yaw_Angle()`, `Set_Target_Pitch_Angle()`~~
  - ~~`TIM_1ms_Resolution_PeriodElapsedCallback()`~~
  - ~~`TIM_1ms_Control_PeriodElapsedCallback()`~~

- ~~`Class_Booster` (in `crt_booster.h`)~~
  - ~~`Init()`~~
  - ~~`Set_Booster_Control_Type()`~~
  - ~~`Set_Friction_Omega()` and fire mode setters~~
  - ~~`TIM_1ms_Calculate_PeriodElapsedCallback()`~~

- ~~`Class_DR16` (in `dvc_dr16.h`)~~
  - ~~`Init()`~~
  - ~~`UART_RxCpltCallback()`~~
  - ~~`TIM_100ms_Alive_PeriodElapsedCallback()`~~
  - ~~`TIM_1ms_Calculate_PeriodElapsedCallback()`~~
  - ~~`Get_*()` accessors for sticks, switches, mouse, keys, yaw~~

- `Class_PID` (in `alg_pid.h`)
  - `Init()`, `Set_Target()`, `Set_Now()`, `TIM_Calculate_PeriodElapsedCallback()`
  - `Set_K_P/I/D/F()`, `Set_I_Out_Max()`, `Set_Out_Max()`
  - `Get_Out()`

## Executable vs separable code
### Executable (tied to runtime scheduling and hardware)
- `test/Core/Src/main.c`
- `test/User_File/5_Task/tsk_config_and_callback.cpp`
- `test/User_File/4_Interaction/ita_robot.cpp`
- `test/User_File/3_Chariot/1_Module/*` ~~and `3_Chariot/2_Posture/*`~~
- Hardware drivers in `test/User_File/1_Middleware/1_Driver/*` (CAN, UART, TIM, WDG, BSP)

### Separable (reusable or portable modules)
- Algorithms: `test/User_File/1_Middleware/2_Algorithm/*` (PID, Slope, Filter, Timer, Queue).
- Device drivers: `test/User_File/2_Device/*` can be reused with different projects if the bus layer is adapted.
- UI / Serialplot: optional instrumentation that can be removed without affecting core control.

## Notes
- `Task100us_TIM4_Callback()` is empty, so TIM4 is not doing work yet.
- ~~`Dynamics_Inverse_Resolution()` has a TODO comment for slope and pressure effects.~~
- ~~Some posture math for Eigen matrices is currently commented out.~~
- IMU data is not used; pitch/roll are fixed to 0 and slope vector is (0, 0, 1).
