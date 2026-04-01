# Robot Perception

This package contains the ROS 2 side of the ultrasonic collision sensing system used by the grocery robot.

## Overview

Each ESP32 is responsible for one side of the robot and is marked with a white direction label:

- `F` = front
- `B` = back
- `R` = right
- `L` = left

Each ESP32 reads 2 ultrasonic sensors, then serves the two measured distances over I2C to the main computer. The ROS 2 node in this package reads those values and publishes a Boolean alert topic for that side.

## Sensor Wiring Per ESP32

Use the same wiring layout on every ESP32 board.

### Ultrasonic sensor channel 1

- `TRIG -> GPIO4`
- `ECHO -> GPIO16`

### Ultrasonic sensor channel 2

- `TRIG -> GPIO17`
- `ECHO -> GPIO34`

### I2C pins on every ESP32

- `SDA -> GPIO21`
- `SCL -> GPIO22`

### Power and ground

- Ultrasonic `VCC -> 5V`
- Ultrasonic `GND -> GND`
- ESP32 `GND` must be shared with the robot main controller/Raspberry Pi/Jetson I2C master ground

## Important Electrical Note

Most common ultrasonic sensors such as `HC-SR04` drive the `ECHO` pin at `5V`. The ESP32 GPIO pins are `3.3V` only.

- Do not connect a `5V` echo signal directly to the ESP32.
- Use a voltage divider or logic level shifter on each `ECHO` line before it reaches `GPIO16` or `GPIO34`.
- `GPIO34` is input-only on ESP32, which is fine because it is used only for `ECHO`.

## Per-Board Meaning of the Two Sensors

The firmware currently stores two readings as:

- `dist_left_m`
- `dist_right_m`

For each ESP32, interpret those as the left and right ultrasonic sensors relative to that board's facing direction.

Example:

- On the `F` board, the two sensors represent the front-left and front-right coverage
- On the `B` board, they represent back-left and back-right coverage
- On the `L` board, they represent left-front and left-back coverage
- On the `R` board, they represent right-front and right-back coverage

## ESP32 Firmware Expectations

The ESP32 ultrasonic I2C firmware is in [ESP32/ultrasonic_I2C/main/ultrasonic_i2c.c](/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/ESP32/ultrasonic_I2C/main/ultrasonic_i2c.c).

The firmware:

- measures both ultrasonic sensors
- packs them as 2 little-endian `float`s
- sends `8` bytes total over I2C
- uses `GPIO21` and `GPIO22` for I2C
- uses the GPIO pairs listed above for the two ultrasonic channels

If you change pin assignments in firmware, update this README and the source file together.

## Current ROS 2 Launch Mapping

The current launch file is [launch/ultrasonic_launch.py](/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_perception/launch/ultrasonic_launch.py).

It starts 4 ultrasonic nodes, one per ESP32:

- `front` at I2C address `0x09`
- `left` at I2C address `0x10`
- `right` at I2C address `0x11`
- `back` at I2C address `0x12`

You mentioned the ESP32 boards are already flashed with different addresses and assigned correctly in the launch file. The values above match the current checked-in launch file.

All nodes currently use:

- I2C bus: `/dev/i2c-7`
- polling rate: `100 Hz`
- alert threshold: `0.20 m`

## How To Run

From the ROS 2 workspace root:

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
colcon build --packages-select robot_perception
source install/setup.bash
ros2 launch robot_perception ultrasonic_launch.py
```

## What The Launch File Starts

The launch file starts the Python executable `distance_sensor` 4 times:

- `ultrasonic_front`
- `ultrasonic_left`
- `ultrasonic_right`
- `ultrasonic_back`

Each node reads one ESP32 over I2C and publishes a side-specific alert topic:

- `/front_alert`
- `/left_alert`
- `/right_alert`
- `/back_alert`

The node also logs the two raw distances it receives from that ESP32.

## Quick Checks

Before launching:

- confirm every ESP32 powers on
- confirm all boards share ground with the I2C master
- confirm the board labels `F`, `B`, `R`, and `L` match physical mounting direction
- confirm each board has the expected I2C address
- confirm `SDA` is on `GPIO21` and `SCL` is on `GPIO22`
- confirm each `ECHO` signal is level-shifted to `3.3V`

Helpful checks on the robot computer:

```bash
ls /dev/i2c-7
i2cdetect -y 7
ros2 topic list | grep alert
ros2 topic echo /front_alert
```

## Troubleshooting

- If `/dev/i2c-7` does not exist, the I2C bus is not enabled or the bus number is different on that computer.
- If an ESP32 does not appear in `i2cdetect`, check power, address configuration, SDA/SCL wiring, and shared ground.
- If the node logs `I2C read failed`, verify the address in the launch file matches the flashed ESP32 firmware.
- If readings are unstable, check the echo level shifting, ground connection, and sensor mounting angle.
- If alerts never trigger, verify the threshold in [launch/ultrasonic_launch.py](/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_perception/launch/ultrasonic_launch.py) and confirm the ESP32 is returning valid distances instead of timeout values.
