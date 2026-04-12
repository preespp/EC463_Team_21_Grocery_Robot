# robot_perception

`robot_perception` contains the ROS 2 ultrasonic collision-sensing stack for the grocery robot. It reads distance data from ESP32-based I2C sensor boards and publishes side-specific alert topics for use by navigation and safety logic.

## What This Package Is

This package is responsible for:

- reading ultrasonic distance data from ESP32 boards over I2C
- converting those readings into Boolean collision alerts
- launching the four directional ultrasonic readers used on the robot

## Tech Stack

- ROS 2 Humble
- Python ROS 2 nodes with `rclpy`
- `smbus2` / Linux I2C access
- ESP32 firmware integration over I2C

## Main Node

- `distance_sensor`: reads one ESP32 board and publishes a side-specific alert topic such as `/front_alert`

## Current Hardware Layout

Each ESP32 board covers one side of the robot:

- `F` = front
- `B` = back
- `L` = left
- `R` = right

Each ESP32 reads two ultrasonic sensors and returns two little-endian `float` values over I2C.

## Wiring Expectations

Ultrasonic sensor channels on each ESP32:

- channel 1: `TRIG -> GPIO4`, `ECHO -> GPIO16`
- channel 2: `TRIG -> GPIO17`, `ECHO -> GPIO34`

I2C pins on each ESP32:

- `SDA -> GPIO21`
- `SCL -> GPIO22`

Power:

- ultrasonic `VCC -> 5V`
- ultrasonic `GND -> GND`
- ESP32 ground must be shared with the robot computer / I2C master ground

## Important Electrical Note

Typical ultrasonic `ECHO` lines are `5V`, while ESP32 GPIO is `3.3V` only. Use a voltage divider or level shifter on each `ECHO` line before it reaches the ESP32.

## Current Launch Mapping

`launch/ultrasonic_launch.py` starts four `distance_sensor` nodes:

- `front` at `0x09`
- `left` at `0x10`
- `right` at `0x11`
- `back` at `0x12`

Current shared defaults:

- bus: `7`
- rate: `100.0`
- threshold: `0.20 m`

## Build

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_perception
source install/setup.bash
```

## How To Run

Run the full ultrasonic stack:

```bash
ros2 launch robot_perception ultrasonic_launch.py
```

Run one node manually:

```bash
ros2 run robot_perception distance_sensor --ros-args \
  -p bus:=7 \
  -p addr:=9 \
  -p rate:=100.0 \
  -p threshold_m:=0.20 \
  -p name:=front
```

## Published Topics

The launch file creates these alert topics:

- `/front_alert`
- `/left_alert`
- `/right_alert`
- `/back_alert`

## Quick Checks

```bash
ls /dev/i2c-7
i2cdetect -y 7
ros2 topic list | grep alert
ros2 topic echo /front_alert
```

## Troubleshooting

- If `/dev/i2c-7` is missing, confirm the Linux I2C bus is enabled and the bus number is correct.
- If a board does not show up in `i2cdetect`, check power, address, wiring, and shared ground.
- If the node logs I2C read failures, confirm the address in the launch file matches the firmware flashed onto the ESP32.
- If alerts never trigger, verify the threshold and make sure the ESP32 is returning valid distances.

## Related Firmware

The matching ESP32 firmware lives in:

- `ESP32/ultrasonic_I2C/main/ultrasonic_i2c.c`

If pin assignments or packet layout change in firmware, update this README and the ROS side together.
