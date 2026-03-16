#!/usr/bin/env python3

import argparse
import sys
import time
from typing import List

import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RobotInfo, TorqueEnable
from rclpy.node import Node


def clamp(value: float, lower: float, upper: float, margin: float = 0.05) -> float:
    safe_lower = lower + margin
    safe_upper = upper - margin
    if safe_lower > safe_upper:
        safe_lower = lower
        safe_upper = upper
    return max(safe_lower, min(value, safe_upper))


class QuickMoveClient(Node):
    def __init__(self, robot_name: str, arm_group: str) -> None:
        super().__init__("vx300_quick_move")
        namespace = f"/{robot_name.strip('/')}" if robot_name else ""
        self.arm_group = arm_group
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

    def get_arm_info(self) -> RobotInfo.Response:
        while rclpy.ok() and not self.info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Interbotix arm services...")

        request = RobotInfo.Request()
        request.cmd_type = "group"
        request.name = self.arm_group

        future = self.info_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.done() or future.result() is None:
            raise RuntimeError("Failed to get arm info from xs_sdk.")
        return future.result()

    def torque_on_arm(self) -> None:
        while rclpy.ok() and not self.torque_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for torque service...")

        request = TorqueEnable.Request()
        request.cmd_type = "group"
        request.name = self.arm_group
        request.enable = True

        future = self.torque_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if not future.done():
            raise RuntimeError("Timed out while enabling arm torque.")

        self.get_logger().info(f"Torque enabled for arm group '{self.arm_group}'.")

    def wait_for_command_subscriber(self, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.group_publisher.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(
            "No subscriber detected on the joint_group command topic. "
            "xs_sdk may not be ready to accept commands."
        )

    def send_arm_pose(self, label: str, positions: List[float], settle_time: float) -> None:
        message = JointGroupCommand()
        message.name = self.arm_group
        message.cmd = [float(position) for position in positions]

        formatted = ", ".join(f"{position:.3f}" for position in message.cmd)
        self.get_logger().info(f"{label}: [{formatted}]")
        self.group_publisher.publish(message)
        time.sleep(settle_time)


def build_sequence(info: RobotInfo.Response) -> List[tuple[str, List[float], float]]:
    joint_count = info.num_joints
    lower_limits = list(info.joint_lower_limits)
    upper_limits = list(info.joint_upper_limits)
    sleep_pose = list(info.joint_sleep_positions)

    home_pose = [0.0] * joint_count
    inspect_pose = [
        clamp(0.5 * sleep_pose[index], lower_limits[index], upper_limits[index])
        for index in range(joint_count)
    ]

    right_sweep_pose = inspect_pose.copy()
    left_sweep_pose = inspect_pose.copy()

    if joint_count > 0:
        right_sweep_pose[0] = clamp(0.35, lower_limits[0], upper_limits[0])
        left_sweep_pose[0] = clamp(-0.35, lower_limits[0], upper_limits[0])

    return [
        ("home_pose", home_pose, 3.5),
        ("inspect_pose", inspect_pose, 3.5),
        ("waist_sweep_right", right_sweep_pose, 2.5),
        ("waist_sweep_left", left_sweep_pose, 2.5),
        ("return_home", home_pose, 3.5),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a short, safe-ish quick-motion sequence to an Interbotix VX300 arm."
    )
    parser.add_argument(
        "--robot-name",
        default="vx300",
        help="Robot namespace used by xsarm_control.launch.py. Defaults to vx300.",
    )
    parser.add_argument(
        "--arm-group",
        default="arm",
        help="Joint group name from the Interbotix motor config. Defaults to arm.",
    )
    args, _ = parser.parse_known_args()
    return args


def main() -> int:
    args = parse_args()
    rclpy.init(args=sys.argv)
    node = QuickMoveClient(robot_name=args.robot_name, arm_group=args.arm_group)

    try:
        info = node.get_arm_info()
        node.get_logger().info(
            "Connected to arm group '%s' with joints: %s"
            % (args.arm_group, ", ".join(info.joint_names))
        )
        node.torque_on_arm()
        node.wait_for_command_subscriber()
        time.sleep(1.0)
        for label, positions, settle_time in build_sequence(info):
            node.send_arm_pose(label=label, positions=positions, settle_time=settle_time)
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
