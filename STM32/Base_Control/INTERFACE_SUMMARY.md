# Base_Control Interface Summary

This summary reflects the current Base_Control code configuration.

## UART / Serial Interfaces

| Purpose | Peripheral | MCU pins | Board macros | Baud | Notes |
| --- | --- | --- | --- | --- | --- |
| Serialplot telemetry (shared on command port) | USART2 | PD5 (TX), PD6 (RX) | `BoardA_Bluetooth_Tx_Pin`, `BoardA_Bluetooth_Rx_Pin` | 115200 | `Class_Serialplot` output now rides on the same link as host commands |
| Host UART (vx/vy/w + IMU) | USART2 | PD5 (TX), PD6 (RX) | `BoardA_Bluetooth_Tx_Pin`, `BoardA_Bluetooth_Rx_Pin` | 115200 | `Class_Host_UART` protocol (RX commands, TX telemetry); shares line with Serialplot frames |
| External IMU input (Wheeltec AHRS) | UART7 | PE8 (TX), PE7 (RX) | `BoardA_UART7_Tx_Pin`, `BoardA_UART7_Rx_Pin` | 921600 | Not used (external AHRS disabled) |

## CAN Buses

| Bus | Peripheral | MCU pins | Board macros | Bitrate | Usage |
| --- | --- | --- | --- | --- | --- |
| Chassis CAN bus | CAN1 | PD1 (TX), PD0 (RX) | `BoardA_CAN1_Tx_Pin`, `BoardA_CAN1_Rx_Pin` | 1 Mbps | DJI C620/M3508, Tx ID 0x200, Rx IDs 0x201-0x204 |
| Unused (available) | CAN2 | PB13 (TX), PB12 (RX) | `BoardA_CAN2_Tx_Pin`, `BoardA_CAN2_Rx_Pin` | 1 Mbps | MX_CAN2_Init runs, but CAN_Init is not called |

## IMU Data Output (to host)

- IMU data is forwarded to the host on USART2 using the `Class_Host_UART` TX frame (header 0x5AA5) every 10 ms.
- Fields: yaw, pitch, roll, omega_z plus chassis vx/vy/omega.
- USART2 Host protocol (little-endian, packed, CRC8 = sum of all previous bytes mod 256):
  - RX (PC -> board, vx/vy/w): `0xA55A` + `float vx` + `float vy` + `float w` + `uint8_t crc8` (15 bytes)
  - TX (board -> PC, IMU + chassis): `0x5AA5` + `float yaw` + `float pitch` + `float roll` + `float omega_z` + `float vx` + `float vy` + `float w` + `uint8_t crc8` (31 bytes)
- Serialplot telemetry frames (0xAB header, 6 floats + checksum) are also emitted on USART2 at 100 Hz; configure SerialPlot to read the same COM port used for commands.

## Internal IMU Hardware Interface (on-board)

- On-board IMU (MPU6500 + IST8310) is wired to SPI5 and used by the control loop:
  - SPI5 SCK PF7, MISO PF8, MOSI PF9
  - CS on PF6 (`BoardA_MPU6500_CS_Pin`)
  - IMU heater PWM on PB5 (`BoardA_PWM_MPU6500_Heater_Pin`)
- Driver: `Base_Control/User/2_Device/IMU/MPU6500/dvc_imu_mpu6500.*`

## Chassis Geometry (Mecanum)

- Geometry parameters are defined in `Base_Control/User/3_Chariot/1_Module/Chassis/crt_chassis.h`:
  - `Wheel_Radius` (wheel radius, meters)
  - `Wheelbase` (front-to-rear wheel center distance, meters)
  - `Wheeltrack` (left-to-right wheel center distance, meters)
  - `Wheel_Direction[4]` (per-wheel sign, wiring-dependent)
- Kinematics for mecanum uses `chassis_radius = (Wheelbase + Wheeltrack) * 0.5f` in `Base_Control/User/3_Chariot/1_Module/Chassis/crt_chassis.cpp`.
- For a rectangular mecanum chassis (wheels at the four corners), set `Wheelbase` and `Wheeltrack` to your measured dimensions.

## File References

- UART pin map: `Base_Control/Core/Inc/main.h`
- UART init/baud: `Base_Control/Core/Src/usart.c`
- CAN init/bitrate: `Base_Control/Core/Src/can.c`
- CAN send/receive flow: `Base_Control/User/1_Middleware/1_Driver/CAN/drv_can.cpp`
- Serialplot usage: `Base_Control/Core/Src/main.c`
- Host UART protocol: `Base_Control/User/2_Device/Host/dvc_host_uart.h`
