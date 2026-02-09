# STM32 ⇄ Host (PC/Jetson) Interface Specification

This document captures the external I/O contracts used by the RoboWalker 2024 bottom infantry firmware: the UART2 “PC control” link and the CAN1 motor bus. It is derived from the current `test` target (January 2026 snapshot).

## UART2 (PC control + telemetry)
| Item | Value |
| --- | --- |
| Port | USART2 (STM32F427, PD5 TX / PD6 RX) |
| Baud | 115 200 bps |
| Framing | 8 data bits, no parity, 1 stop bit (8‑N‑1) |
| Flow control | None |
| Driver | DMA receive-to-idle, interrupt-driven TX |

Two logical streams share this link:

1. **PC → STM32 control frame** (`tools/pc_control.py`, Jetson/PC host)
2. **STM32 → PC telemetry frame** (Serialplot-compatible)

### PC → STM32 frame

```
Offset  Size  Type     Name
0       1     uint8    Header (0xAC)
1       4     float    Right stick X  (unused, reserved)
5       4     float    Right stick Y  (unused, reserved)
9       4     float    Left stick X   (strafe, -1..1)
13      4     float    Left stick Y   (forward, -1..1)
17      4     float    Yaw            (spin, -1..1)
21      2     uint16   Keyboard mask  (bitfield, see below)
23      1     uint8    Left switch    (DR16 encoding: 1=UP, 2=DOWN, 3=MID)
24      1     uint8    Right switch   (same as left)
25      1     uint8    Checksum = sum(bytes[1..24]) & 0xFF
```

- Total size: 26 bytes. Any other size is discarded.
- Frame rate: default 50 Hz (`--rate` in `pc_control.py`).
- Default host settings: COM14, 115 200 bps, overridable via CLI flags.

Keyboard mask bits follow the DR16 convention used in `dvc_pc.h`:

| Bit | Key | In-firmware action (Jan 2026 build) |
| --- | --- | --- |
|0|`W`|Set left stick Y = +1 ⇒ chassis forward|
|1|`S`|Set left stick Y = −1 ⇒ chassis reverse|
|2|`A`|Set left stick X = −1 ⇒ strafe left|
|3|`D`|Set left stick X = +1 ⇒ strafe right|
|4|`Shift`|Speed boost (`_Chassis_Control` raises velocity/omega limits)|
|5|`Ctrl`|Reserved (parsed but unused)|
|6|`Q`|Yaw command −1 ⇒ rotate CCW|
|7|`E`|Yaw command +1 ⇒ rotate CW|
|8|`R`|Reserved|
|9|`F`|Reserved|
|10|`G`|Reserved|
|11|`Z`|Reserved|
|12|`X`|Reserved|
|13|`C`|Reserved|
|14|`V`|Reserved|
|15|`B`|Reserved|

> Reserved keys still travel inside the frame (and keep DR16-compatible semantics) but the current firmware does not act on them. They remain available for future mode toggles (e.g., gyroscope, supercap, booster) without breaking the protocol.

**Host implementation tips**

- When integrating with ROS 2 on Jetson, reuse the frame format above—only the transport layer changes (wrap `pyserial`/`asio` inside a ROS 2 node).
- Keep the UART window focused (if using `pynput`) so key events propagate; otherwise no bits are set.
- Left switch down (`value == 2`) or loss of frames for >100 ms disables the chassis (`Chassis_Control_Type_DISABLE`).

### STM32 → PC telemetry frame (Serialplot)

The firmware reuses the Serialplot transport in `dvc_serialplot.cpp`. Every 5 ms (200 Hz) the following 6 floats are sent (little-endian):

| Channel | Signal |
| --- | --- |
|0|Target vx (m/s)|
|1|Target vy (m/s)|
|2|Target omega (rad/s)|
|3|Measured vx (m/s)|
|4|Measured vy (m/s)|
|5|Measured omega (rad/s)|

Current firmware frame format on UART2 TX:

- Header: `0xAB`
- Payload: 6 x `float32` (24 bytes)
- Checksum: 8-bit sum over payload (1 byte)

Total frame size: 26 bytes, i.e. about 52 kbps at 200 Hz (safe margin at 115200 bps). Hosts that don’t need telemetry may ignore UART2 TX entirely.

## CAN1 (chassis motors)

| Item | Value |
| --- | --- |
| Bus | CAN1 (1 Mbps, normal mode) |
| Command ID | 0x200 |
| Feedback IDs | 0x201 (FL), 0x202 (FR), 0x203 (RL), 0x204 (RR) |
| Motor type | DJI C620 + M3508 |

### STM32 → Motors (ID 0x200)

```
Byte 0~1 : Wheel FL target current (int16, big-endian)
Byte 2~3 : Wheel FR target current
Byte 4~5 : Wheel RL target current
Byte 6~7 : Wheel RR target current
```

- Units follow DJI ESC scaling (range ±16384 ≈ ±20 A).
- Sign already accounts for wiring via `Wheel_Direction`, so FR/RR currents may be negative for forward motion.
- Frames are refreshed every 1 ms (TIM5 scheduler).

### Motors → STM32 (IDs 0x201–0x204)

Each M3508/C620 reports an 8-byte status frame:

```
Byte 0~1 : Encoder (0..8191)
Byte 2~3 : Speed (int16, ±RPM)
Byte 4~5 : Current (int16, raw ADC units)
Byte 6   : Temperature (°C)
Byte 7   : Reserved
```

`dvc_motor_dji.cpp` converts these into angle (rad), omega (rad/s), current (A), and estimates instant power for power limiting. Encoder and speed values are multiplied by the wheel’s direction sign before feeding odometry, so the host does not need to compensate.

## Summary
- UART2 provides the primary host control ingress. Maintain the 26-byte frame and checksum if replacing the PC tool with Jetson/ROS 2.
- CAN1 drives the four chassis motors with ID 0x200 and reads back motor statuses via 0x201–0x204.
- Directional mismatches are abstracted in firmware (`Wheel_Direction`), so external tools can treat all wheels as having the same forward convention.
