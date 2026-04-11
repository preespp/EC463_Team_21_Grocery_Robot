# STM32 Firmware

This folder contains the STM32 firmware used for the mobile base motor-control side of GOFR.

## What It Contains

- `Base_Control_v2/robowalker2024bottominfantry-main/`: the current STM32 base-control project
- `docs/CONTROL_LOGIC.md`: control-logic notes for the mecanum chassis and motor pipeline
- `test/test.ioc`: STM32CubeMX / STM32CubeIDE project configuration for the checked-in build
- `tools/pc_control.py`: serial testing helper for host-side validation

## Current Role In The Robot

The STM32 side is used for:

- mecanum wheel motor control
- receiving velocity commands from the ROS navigation bridge
- returning odometry / telemetry used by the higher-level ROS 2 stack

## Typical Workflow

1. Open the STM32 project from:

```text
STM32/Base_Control_v2/robowalker2024bottominfantry-main/test/test.ioc
```

2. Build and flash it with STM32CubeIDE or a compatible STM32 toolchain.

3. Connect the controller to the robot computer so `robot_navigation/nav2_serial_bridge` can communicate with it over serial.

## Related Documentation

- [CONTROL_LOGIC.md](/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/STM32/Base_Control_v2/robowalker2024bottominfantry-main/docs/CONTROL_LOGIC.md)
