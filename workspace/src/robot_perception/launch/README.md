# Launch

This folder stores ROS 2 launch files for the `robot_perception` package.

## Files

- `ultrasonic_launch.py`: launches 4 ultrasonic I2C nodes for the `front`, `left`, `right`, and `back` ESP32 boards.

## Ultrasonic Launch

`ultrasonic_launch.py` starts the `distance_sensor` node 4 times with different I2C addresses:

- `front` -> `0x09`
- `left` -> `0x10`
- `right` -> `0x11`
- `back` -> `0x12`

Common parameters in the launch file:

- `bus = 7`
- `rate = 100.0`
- `threshold_m = 0.20`

Run it with:

```bash
ros2 launch robot_perception ultrasonic_launch.py
```

For full wiring instructions and setup notes, see [README.md](/Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace/src/robot_perception/README.md).
