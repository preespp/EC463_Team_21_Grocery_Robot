# RoboWalker 2024 Bottom Infantry – Engineering Logbook

_Last updated: 2026-01-20_

This logbook consolidates the control architecture, firmware layout, communication protocols, toolchain, and notable debug sessions for the STM32-based bottom infantry chassis. It is intended as a “ready-to-hand” reference when onboarding new teammates, replicating the build, or integrating higher-level systems (e.g., Jetson ROS 2).

---

## 1. Project Overview

| Aspect | Description |
| --- | --- |
| MCU | STM32F427, Keil MDK target `test` |
| Chassis | 4-wheel mecanum (C620 + M3508), CAN1 bus IDs 0x201–0x204 |
| Host I/O | UART2 (115200 bps) for PC/Jetson control + Serialplot telemetry |
| Scheduler | TIM5 @ 1 kHz (alive checks, slope planning, CAN send); TIM4 @ 10 kHz (reserved) |
| Toolchain | Keil ARMCC 5.06 Update 6, CubeMX `.ioc`, Serialplot for telemetry |
| Repo layout | `test/Core` (Cube HAL), `test/User_File` (middleware & control), `docs` (specs) |

### Control Flow Snapshot
1. `main.c` → `Task_Init()` (BSP, CAN, UART, timers, watchdog, Serialplot, robot init).
2. TIM5 interrupt (`Task1ms_TIM5_Callback()`): alive counters (100 ms / 1 s), 10 ms logic, 2 ms chassis control, 1 ms PC parse + slope planning. Serialplot + CAN/UART/WDG services also run here.
3. Chassis loop (every 2 ms):
   - `Self_Resolution()` – wheel odometry (with wiring sign compensation).
   - `Kinematics_Inverse_Resolution()` – mecanum equations.
   - `Output_To_Dynamics()` – outer PID on vx/vy/omega.
   - `Dynamics_Inverse_Resolution()` – per-wheel current target.
   - `_Power_Limit_Control()` + CAN send.
4. PC UART2 input is parsed via DMA receive-to-idle. Valid frames update `Class_PC`; invalid ones fall back to Serialplot parser.

---

## 2. Key Modules & Algorithms

### 2.1 Class_PC (PC UART bridge)
- **Frame**: `0xAC` header + 5 floats (rightX/rightY/leftX/leftY/yaw) + uint16 keyboard mask + 2 switch bytes + checksum.
- **Timing**: frames expected at ≥10 Hz; alive watchdog disables chassis if no increment in 100 ms.
- **Edge detection**: `TIM_1ms_Calculate_PeriodElapsedCallback()` compares current vs previous mask/switch to generate DR16-style triggers.
- **Integration points**: `PC.Get_Left_X/Y`, `PC.Get_Yaw`, `Get_Keyboard_Key_*`, left switch gating in `_Chassis_Control()` (safety).

### 2.2 Class_Chassis
- **Geometry**: R = 0.0762 m, half-length 0.1905 m, half-width 0.3175 m.
- **Direction map**: `constexpr int8_t Wheel_Direction[4] = {+1, -1, +1, -1}` for FL/FR/RL/RR wiring.
- **Forward kinematics**: using signed omegas `s_i * w_i` to compute body velocity and yaw rate.
- **Inverse kinematics**: standard mecanum equations; direction signs applied only at motor command stage to avoid distorting dynamics/PIDs.
- **Dynamics inversion**: `force_x/y` and `torque` from outer PIDs → per-wheel current with slip correction (`Wheel_Speed_Limit_Factor`) + dynamic friction compensation.
- **Power limiting**: motor power estimation (from C620 feedback) allows dynamic scaling via `_Power_Limit_Control()`.

### 2.3 Motor Stack (C620 + M3508)
- `dvc_motor_dji.cpp` provides cascaded control: angle → omega → current (only current loop active in chassis mode).
- CAN commands: ID 0x200 (four int16 currents). Feedback frames 0x201–0x204 parsed to angle/omega/current/temperature, including power estimation for limit control.

### 2.4 PID & Slope
- `alg_pid`: configurable dead-zone, variable integration rate, derivative-on-measurement, feedforward term.
- `Class_Slope`: soft-start ramp for vx/vy/omega targets (configured in `Class_Robot::Init()`), smoothing PC commands.

---

## 3. Host Interfaces

### 3.1 UART2 (PC/Jetson Control)

| Parameter | Value |
| --- | --- |
| Baud | 115 200 (8-N-1) |
| Header | 0xAC |
| Payload | RightX, RightY, LeftX, LeftY, Yaw (float32) + Keyboard Mask (uint16) + Switch L/R (uint8) |
| Checksum | 8-bit sum over payload |
| Frame size | 26 bytes |

Keyboard mask (bit → action):

| Bit | Key | Action |
| --- | --- | --- |
|0|W|Left stick Y +1 (forward)|
|1|S|Left stick Y −1 (reverse)|
|2|A|Left stick X −1 (strafe left)|
|3|D|Left stick X +1 (strafe right)|
|4|Shift|Boost mode (higher velocity & omega limits)|
|5|Ctrl|Reserved|
|6|Q|Yaw −1 (CCW spin)|
|7|E|Yaw +1 (CW spin)|
|8‑15|R/F/G/Z/X/C/V/B|Reserved (future toggles)|

Switch encoding uses DR16 semantics (1=UP, 2=MID, 3=DOWN). Left switch DOWN or PC offline disables chassis output.

Serialplot telemetry (STM32 → host): 10 floats (target vx/vy/omega, measured vx/vy/omega, wheel omegas FL/FR/RL/RR). Format matches Serialplot binary protocol.

### 3.2 CAN1 (Wheel Motors)
- Bus speed: 1 Mbps.
- Command frame (ID 0x200): `[I_FL, I_FR, I_RL, I_RR]` as int16 currents (big-endian).
- Feedback frames (0x201–0x204): `[encoder, speed, current, temp, reserved]`.
- Direction signs already applied in firmware; host-level CAN tools may treat positive current as “chassis forward” for all wheels.

---

## 4. Toolchain & Utilities

| Tool | Purpose |
| --- | --- |
| Keil MDK (ARMCC 5.06u6) | Build + flash (`test.uvprojx` target). |
| STM32CubeMX | `test/test.ioc` as configuration reference. |
| Serialplot | UART2 telemetry visualization. |
| `tools/pc_control.py` | Python host script (pynput + pyserial) for keyboard teleoperation. Defaults: COM14, 115200, 50 Hz. |
| ROS 2 (planned) | Jetson integration—reuse UART2 frame format inside a ROS node. |

---

## 5. Debug & Experiment Log (selected entries)

| Date | Issue | Investigation | Resolution |
| --- | --- | --- | --- |
| 2026-01-19 | Right wheels spin backwards when moving forward. | Verified CAN IDs (0x202/0x204) physically reversed. Observed `Chassis_Control_Type_DISABLE` gating when PC offline. | Added `Wheel_Direction` map; first attempt applied signs multiple times causing speed mismatch. Final approach: apply sign only when sampling encoder (`Self_Resolution`) and when commanding currents (`Output_To_Motor`); keep core kinematics in uniform frame. |
| 2026-01-19 | Keil error “Cannot open file test\test.axf”. | Build log showed linking error `Undefined symbol Class_Chassis::Wheel_Direction`. | Added `constexpr int8_t Class_Chassis::Wheel_Direction[4];` definition in `crt_chassis.cpp`. |
| 2026-01-19 | Serial write timeout from `pc_control.py` on COM14. | Determined mismatched baud (MCU 1 Mbps, PC set default COM14). | Reduced USART2 baud to 115200 (CubeMX, HAL init, Python defaults) to match USB-UART dongle capability. |
| 2026-01-20 | Documentation gaps for ROS/Jetson integration. | Compiled `CONTROL_LOGIC.md` and `INTERFACES_PC_CAN.md`; added keyboard-to-action mapping, UART/CAN specs. | Created `LOGBOOK.md` + interface doc to support future ROS2 bridge. |

Future TODOs (tracked here for quick reference):
- Re-enable supercap/referee/manifold logic once hardware is available.
- Consider micro-ROS or native ROS2 bridge if Jetson ↔ MCU latency becomes critical.
- Expand Serialplot channels with battery/temperature telemetry, or replace with ROS2 diagnostics.

---

## 6. Integration Notes for Jetson ROS 2

1. **Serial node**: Use `rclpy`/`rclcpp` + `pyserial` to open `/dev/ttyUSBx` @ 115200. Publish `/cmd_vel` → frame builder, subscribe to `/chassis/odom` if feedback is implemented.
2. **Protocol reuse**: Keep frame layout identical to `pc_control.py` to avoid firmware changes.
3. **Safety**: Monitor `PC_Status` by adding a return frame or dedicating a Serialplot channel; stop sending commands if MCU reports disable.
4. **Testing**: Start with loopback on PC, then move to Jetson, verifying checksum and frame rate with logic analyzer or serial sniffer.

---

## 7. Repository Structure Cheat Sheet

```
├── test/
│   ├── Core/             # Cube HAL sources
│   ├── User_File/
│   │   ├── 1_Middleware/ # Drivers (CAN/UART/TIM/BSP), Algorithms (PID/Slope)
│   │   ├── 2_Device/     # Device abstractions (PC UART, DJI motors, Serialplot)
│   │   ├── 3_Chariot/    # Chassis modules
│   │   ├── 4_Interaction/# Robot orchestration
│   │   └── 5_Task/       # Scheduler, callbacks
│   └── MDK-ARM/          # Keil project files
├── tools/pc_control.py   # Host keyboard control script
└── docs/                 # CONTROL_LOGIC.md, INTERFACES_PC_CAN.md, LOGBOOK.md
```

---

## 8. Quick Reference (cheat table)

| Hotkey | Effect | Notes |
| --- | --- | --- |
| W / S | Move forward / backward | Continuous while held; filtered by slope planner. |
| A / D | Strafe left / right | Same magnitude as W/S. |
| Q / E | Spin CCW / CW | Maps to yaw command ±1. |
| Shift | Boost mode | `_Chassis_Control` raises target limits + slope rate. |
| Left switch DOWN | Disable chassis | Safety interlock. |
| PC frame loss >100 ms | Disable chassis | Alive timer triggers `UART_Reinit`. |

---

## 9. Contacts & Maintenance

| Area | Owner | Notes |
| --- | --- | --- |
| STM32 firmware | Team 21 embedded group | Keep `test` target synced with `Base_Control` if features diverge. |
| Jetson / ROS bridge | TBD | Follow UART & CAN specs; contributions welcome. |
| Documentation | Repo maintainers | Update `LOGBOOK.md` + interface docs when protocols or scheduling change. |

---

_End of logbook._
