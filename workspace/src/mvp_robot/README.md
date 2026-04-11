# mvp_robot

`mvp_robot` is the legacy minimum-viable-product package from the early project phase. It was used to prototype the first end-to-end robot behaviors before the codebase was split into the newer dedicated packages such as `robot_navigation`, `robot_vision`, `robot_perception`, `robot_manipulation`, and `robot_task_manager`.

## What This Package Is

This package collects older prototype nodes for:

- central orchestration
- mock UI input
- wheel and navigation experiments
- camera and distance-sensor experiments
- arm control experiments
- Gazebo bridge testing

Use this package mainly for historical reference, early demos, or small isolated tests. For the current production stack, prefer the newer dedicated packages in `workspace/src`.

## Tech Stack

- ROS 2 Humble
- Python ROS 2 nodes (`rclpy`)
- custom interfaces from `robot_interfaces`
- OpenCV / CV bridge for camera work
- simple launch files for mock integration tests

## Main Nodes

- `central`: early orchestration and mock product lookup
- `ui_input`: prototype UI input node
- `navigation`: prototype movement/navigation logic
- `wheel_motor`: wheel motor experiment node
- `distance_sensor`: legacy distance sensor node
- `camera_vision`: prototype camera node
- `arm_controller`: prototype arm controller
- `arm_to_gazebo`: bridge from ROS commands to Gazebo testing

## Launch Files

- `launch/mvp_mock.py`: starts `central`, `distance_sensor`, and `camera_vision`
- `launch/product_query_mock.py`: starts `central` and `ui_input`

## Build

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select mvp_robot robot_interfaces
source install/setup.bash
```

## How To Run

Run the small mock stack:

```bash
ros2 launch mvp_robot mvp_mock.py
```

Run the product-query mock flow:

```bash
ros2 launch mvp_robot product_query_mock.py
```

Run individual nodes:

```bash
ros2 run mvp_robot central
ros2 run mvp_robot ui_input
ros2 run mvp_robot navigation
ros2 run mvp_robot wheel_motor
ros2 run mvp_robot camera_vision
ros2 run mvp_robot distance_sensor
ros2 run mvp_robot arm_controller
ros2 run mvp_robot arm_to_gazebo
```

## Notes

- This package is older than the current split architecture, so some flows are prototype-only.
- If you are working on the current grocery robot stack, start with the package-specific READMEs in the newer packages instead of adding new work here.
