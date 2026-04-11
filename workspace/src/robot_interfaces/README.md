# robot_interfaces

`robot_interfaces` contains the shared ROS 2 messages, services, and actions used across the robot software stack. It is the contract package that lets navigation, manipulation, task management, perception, and external tools communicate using the same types.

## What This Package Is

This package defines the custom interfaces for:

- order intake
- semantic target lookup
- arm actions
- rack actions

Most other packages in `workspace/src` depend on this package.

## Tech Stack

- ROS 2 interface package
- `ament_cmake`
- `rosidl_default_generators`
- standard ROS message dependencies such as `geometry_msgs` and `std_msgs`

## Defined Interfaces

Messages:

- `msg/Order.msg`
- `msg/OrderItem.msg`

Services:

- `srv/NewOrder.srv`
- `srv/ResolveSemanticTarget.srv`

Actions:

- `action/PickArm.action`
- `action/MoveRack.action`

## Build

```bash
cd /Users/preejedi/Desktop/EC463_Team_21_Grocery_Robot/workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_interfaces
source install/setup.bash
```

## How To Use

Inspect the available interfaces:

```bash
ros2 interface show robot_interfaces/msg/Order
ros2 interface show robot_interfaces/msg/OrderItem
ros2 interface show robot_interfaces/srv/NewOrder
ros2 interface show robot_interfaces/srv/ResolveSemanticTarget
ros2 interface show robot_interfaces/action/PickArm
ros2 interface show robot_interfaces/action/MoveRack
```

Example service call:

```bash
ros2 service call /order/new robot_interfaces/srv/NewOrder "{order: {order_id: 1, role: customer, requester_id: test, items: [{product_id: TEST1, name: bottle, aisle: '0', rack: 0, shelf_level: 1, qty: 1, price: 0.0, stock: 1}]}}"
```

## Notes

- Build this package before packages that consume its custom actions, messages, or services.
- If you change an interface here, rebuild the dependent packages that use it.
