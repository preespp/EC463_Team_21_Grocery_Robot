#!/usr/bin/env python3

import argparse
import math
import sys
import time
from typing import Dict, List, Optional

import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RegisterValues, RobotInfo, TorqueEnable
from rclpy.node import Node
from sensor_msgs.msg import JointState


def clamp(value: float, lower: float, upper: float, margin: float = 0.05) -> float:
    safe_lower = lower + margin
    safe_upper = upper - margin
    if safe_lower > safe_upper:
        safe_lower = lower
        safe_upper = upper
    return max(safe_lower, min(value, safe_upper))


class VX300HardwareDiagnostics(Node):
    def __init__(self, robot_name: str, arm_group: str, test_delta: float, command_test: bool) -> None:
        super().__init__("vx300_hardware_diagnostics")
        namespace = f"/{robot_name.strip('/')}" if robot_name else ""
        self.robot_name = robot_name
        self.arm_group = arm_group
        self.test_delta = abs(test_delta)
        self.command_test = command_test
        self.latest_joint_state: Optional[JointState] = None

        self.group_publisher = self.create_publisher(
            JointGroupCommand,
            f"{namespace}/commands/joint_group",
            10,
        )
        self.info_client = self.create_client(
            RobotInfo,
            f"{namespace}/get_robot_info",
        )
        self.torque_client = self.create_client(
            TorqueEnable,
            f"{namespace}/torque_enable",
        )
        self.register_client = self.create_client(
            RegisterValues,
            f"{namespace}/get_motor_registers",
        )
        self.create_subscription(
            JointState,
            f"{namespace}/joint_states",
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def wait_for_services(self) -> None:
        clients = [
            ("get_robot_info", self.info_client),
            ("torque_enable", self.torque_client),
            ("get_motor_registers", self.register_client),
        ]
        for label, client in clients:
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {label} service...")

    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> JointState:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.latest_joint_state is not None:
                return self.latest_joint_state
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("Timed out waiting for joint state feedback from the arm.")

    def wait_for_command_subscriber(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.group_publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError("No subscriber detected on the joint_group command topic.")

    def get_arm_info(self) -> RobotInfo.Response:
        request = RobotInfo.Request()
        request.cmd_type = "group"
        request.name = self.arm_group

        future = self.info_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.done() or future.result() is None:
            raise RuntimeError("Failed to get robot info from xs_sdk.")
        return future.result()

    def torque_on_arm(self) -> None:
        request = TorqueEnable.Request()
        request.cmd_type = "group"
        request.name = self.arm_group
        request.enable = True

        future = self.torque_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.done():
            raise RuntimeError("Timed out while enabling arm torque.")

    def get_group_registers(self, register_name: str) -> List[int]:
        request = RegisterValues.Request()
        request.cmd_type = "group"
        request.name = self.arm_group
        request.reg = register_name
        request.value = 0

        future = self.register_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Failed to read register '{register_name}'.")
        return list(future.result().values)

    def arm_positions_from_joint_state(
        self,
        joint_state: JointState,
        joint_names: List[str],
    ) -> List[float]:
        joint_map: Dict[str, float] = dict(zip(joint_state.name, joint_state.position))
        missing = [name for name in joint_names if name not in joint_map]
        if missing:
            raise RuntimeError(
                "Joint state feedback is missing arm joints: %s" % ", ".join(missing)
            )
        return [joint_map[name] for name in joint_names]

    def publish_pose(self, positions: List[float]) -> None:
        message = JointGroupCommand()
        message.name = self.arm_group
        message.cmd = [float(position) for position in positions]
        self.group_publisher.publish(message)

    def wait_for_motion(
        self,
        joint_names: List[str],
        start_positions: List[float],
        timeout_sec: float = 4.0,
        threshold: float = 0.05,
    ) -> List[float]:
        deadline = time.monotonic() + timeout_sec
        latest_positions = start_positions

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is None:
                continue
            latest_positions = self.arm_positions_from_joint_state(
                self.latest_joint_state,
                joint_names,
            )
            if any(
                abs(current - start) >= threshold
                for current, start in zip(latest_positions, start_positions)
            ):
                return latest_positions

        return latest_positions

    def log_registers(self, joint_names: List[str], register_name: str, values: List[int]) -> None:
        if not values:
            self.get_logger().warn(
                f"{register_name}: no values returned by get_motor_registers."
            )
            return
        formatted = ", ".join(
            f"{joint_name}={value}" for joint_name, value in zip(joint_names, values)
        )
        self.get_logger().info(f"{register_name}: {formatted}")

    def run(self) -> int:
        self.wait_for_services()
        info = self.get_arm_info()
        self.wait_for_joint_state()
        self.wait_for_command_subscriber()

        self.get_logger().info(
            "Connected to arm group '%s' with joints: %s"
            % (self.arm_group, ", ".join(info.joint_names))
        )

        torque_values = self.get_group_registers("Torque_Enable")
        error_values = self.get_group_registers("Hardware_Error_Status")
        self.log_registers(info.joint_names, "Torque_Enable", torque_values)
        self.log_registers(info.joint_names, "Hardware_Error_Status", error_values)

        if any(value == 0 for value in torque_values):
            self.get_logger().warn("One or more arm joints are not torqued on.")

        if any(value != 0 for value in error_values):
            self.get_logger().error(
                "One or more arm joints report a non-zero hardware error status."
            )

        if not self.command_test:
            return 1 if any(value == 0 for value in torque_values) or any(
                value != 0 for value in error_values
            ) else 0

        self.torque_on_arm()
        torque_values_after = self.get_group_registers("Torque_Enable")
        self.log_registers(info.joint_names, "Torque_Enable_after_service", torque_values_after)

        starting_joint_state = self.wait_for_joint_state()
        start_positions = self.arm_positions_from_joint_state(
            starting_joint_state,
            list(info.joint_names),
        )
        goal_registers_before = self.get_group_registers("Goal_Position")
        self.log_registers(info.joint_names, "Goal_Position_before", goal_registers_before)

        target_positions = list(start_positions)
        if not target_positions:
            raise RuntimeError("Arm group does not contain any joints to test.")

        desired = clamp(
            start_positions[0] + self.test_delta,
            info.joint_lower_limits[0],
            info.joint_upper_limits[0],
        )
        if math.isclose(desired, start_positions[0], abs_tol=0.02):
            desired = clamp(
                start_positions[0] - self.test_delta,
                info.joint_lower_limits[0],
                info.joint_upper_limits[0],
            )
        target_positions[0] = desired

        self.get_logger().info(
            "Sending waist test command from %.3f rad to %.3f rad."
            % (start_positions[0], target_positions[0])
        )
        self.publish_pose(target_positions)
        time.sleep(0.5)

        goal_registers_after = self.get_group_registers("Goal_Position")
        self.log_registers(info.joint_names, "Goal_Position_after", goal_registers_after)

        final_positions = self.wait_for_motion(list(info.joint_names), start_positions)
        formatted_final = ", ".join(f"{position:.3f}" for position in final_positions)
        self.get_logger().info(f"Joint positions after command: [{formatted_final}]")

        position_deltas = [
            abs(final - start) for final, start in zip(final_positions, start_positions)
        ]
        moved = any(delta >= 0.05 for delta in position_deltas)
        goal_changed = goal_registers_after != goal_registers_before

        if moved:
            self.get_logger().info("Motion feedback changed. The arm accepted and executed the test.")
            return 0

        if any(value == 0 for value in torque_values_after):
            self.get_logger().error(
                "The waist test did not move and at least one joint still reports Torque_Enable=0."
            )
            return 1

        if any(value != 0 for value in error_values):
            self.get_logger().error(
                "The waist test did not move and the motors report hardware errors."
            )
            return 1

        if goal_changed:
            self.get_logger().error(
                "Goal_Position changed but joint feedback did not. ROS commands are reaching the "
                "motors, but the arm is not physically following them."
            )
            self.get_logger().error(
                "This usually points to motor torque/power problems, a hardware fault, or a "
                "mechanical lock."
            )
            return 1

        self.get_logger().error(
            "Joint feedback and Goal_Position both stayed unchanged after the command."
        )
        self.get_logger().error(
            "This points to a command-delivery problem between the ROS topic and xs_sdk."
        )
        return 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Diagnose why a real Interbotix VX300 arm is not moving."
    )
    parser.add_argument(
        "--robot-name",
        default="vx300s",
        help="Robot namespace used by xs_sdk. Defaults to vx300s.",
    )
    parser.add_argument(
        "--arm-group",
        default="arm",
        help="Joint group name from the Interbotix motor config. Defaults to arm.",
    )
    parser.add_argument(
        "--test-delta",
        type=float,
        default=0.15,
        help="Small waist motion [rad] used for the live motion test. Defaults to 0.15.",
    )
    parser.add_argument(
        "--skip-command-test",
        action="store_true",
        help="Only read torque and error registers without sending a motion command.",
    )
    args, _ = parser.parse_known_args()
    return args


def main() -> int:
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = VX300HardwareDiagnostics(
        robot_name=args.robot_name,
        arm_group=args.arm_group,
        test_delta=args.test_delta,
        command_test=not args.skip_command_test,
    )

    try:
        return node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
