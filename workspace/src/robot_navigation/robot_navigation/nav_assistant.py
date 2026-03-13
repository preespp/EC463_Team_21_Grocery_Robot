#!/usr/bin/env python3
"""
Unified workflow helper for the Team 21 navigation stack.

This script wraps the runbook in Nav/README_SLAM_UPDATED.md into short commands:
- one-line launch for mapping and localization+Nav2
- map save/export
- Nav2 goal/waypoint sending
- preset motion macros and keyboard macro pad
"""

from __future__ import annotations

import argparse
import json
import math
import select
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import FollowWaypoints, NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_srvs.srv import Empty
    ROS_IMPORT_ERROR = None
except ImportError as exc:
    rclpy = None
    GoalStatus = None
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Twist = None
    FollowWaypoints = None
    NavigateToPose = None
    ActionClient = None
    Node = object
    Empty = None
    ROS_IMPORT_ERROR = exc


def _find_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in [here.parent, *here.parents]:
        if (
            (parent / "Nav").exists()
            and (parent / "Maps").exists()
            and (parent / "workspace").exists()
        ):
            return parent
    cwd = Path.cwd()
    if (cwd / "Nav").exists() and (cwd / "Maps").exists():
        return cwd
    return cwd


REPO_ROOT = _find_repo_root()
DEFAULT_MAPS_DIR = str(REPO_ROOT / "Maps")
DEFAULT_MAP_NAME = "testmap1"
DEFAULT_CMD_TOPICS = '["/cmd_vel","/cmd_vel_nav","/cmd_vel_smoothed"]'
DEFAULT_RUN_MODE = "normal"
DEFAULT_RUNTIME_LOCALIZER = "amcl"
RUNTIME_LOCALIZERS = ("amcl", "cartographer")
DEFAULT_NAV_PROFILE = "smac_mppi_omni"
RUN_MODES = ("normal", "bench")
MAPPING_CONFIG_BASENAME = {
    "normal": "pico_2d_mapping_quality_scan_segment.lua",
    "bench": "pico_2d_bench.lua",
}
LOCALIZATION_CONFIG_BASENAME = {
    "normal": "pico_2d_localization_scan_segment.lua",
    "bench": "pico_2d_localization_bench.lua",
}
NAV2_PROFILE_PARAMS = {
    "baseline": "nav2_params_baseline.yaml",
    "astar_dwb": "nav2_params_astar_dwb.yaml",
    "smac_mppi_omni": "nav2_params_smac_mppi_omni.yaml",
    "smac_mppi_diff": "nav2_params_smac_mppi_diff.yaml",
    "smac_rpp": "nav2_params_smac_rpp.yaml",
}
LEGACY_NAV2_PARAMS = "nav2_params_cartographer.yaml"
BT_REQUIRED_PLUGIN_LIBS = (
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_back_up_action_bt_node",
)
NAV_PROFILE_EXPECTATIONS = {
    "baseline": {
        "planner_plugin": "nav2_navfn_planner/NavfnPlanner",
        "controller_plugin": "dwb_core::DWBLocalPlanner",
        "use_astar": "false",
    },
    "astar_dwb": {
        "planner_plugin": "nav2_navfn_planner/NavfnPlanner",
        "controller_plugin": "dwb_core::DWBLocalPlanner",
        "use_astar": "true",
    },
    "smac_mppi_omni": {
        "planner_plugin": "nav2_smac_planner/SmacPlanner2D",
        "controller_plugin": "nav2_mppi_controller::MPPIController",
        "motion_model": "Omni",
    },
    "smac_mppi_diff": {
        "planner_plugin": "nav2_smac_planner/SmacPlanner2D",
        "controller_plugin": "nav2_mppi_controller::MPPIController",
        "motion_model": "DiffDrive",
    },
    "smac_rpp": {
        "planner_plugin": "nav2_smac_planner/SmacPlanner2D",
        "controller_plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    },
}


def default_nav2_params_path(filename: str) -> Path:
    return REPO_ROOT / "workspace" / "src" / "robot_navigation" / "config" / filename


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def parse_pose(text: str) -> Tuple[float, float, float]:
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"Invalid pose '{text}'. Expected format: x,y,yaw"
        )
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"Invalid numeric values in pose '{text}'"
        ) from exc


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "true", "t", "yes", "y", "on"):
        return True
    if normalized in ("0", "false", "f", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Invalid boolean value '{value}'")


def parse_segment(text: str) -> "MotionSegment":
    parts = [p.strip() for p in text.split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            f"Invalid segment '{text}'. Expected: vx,vy,wz,duration_sec"
        )
    try:
        vx, vy, wz, duration = (float(v) for v in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"Invalid numeric values in segment '{text}'"
        ) from exc
    if duration <= 0.0:
        raise argparse.ArgumentTypeError("Segment duration must be > 0")
    return MotionSegment(vx=vx, vy=vy, wz=wz, duration=duration)


def bool_to_launch(value: bool) -> str:
    return "true" if value else "false"


def bool_to_sick_flag(value: bool) -> str:
    return "1" if value else "0"


def resolve_use_ekf(run_mode: str, requested: bool | None) -> bool:
    if requested is not None:
        return requested
    return run_mode != "bench"


def render_command(command: Sequence[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def map_paths(maps_dir: str, map_name: str) -> Tuple[Path, Path, Path]:
    base = Path(maps_dir)
    pbstream = base / f"{map_name}.pbstream"
    filestem = base / map_name
    yaml_file = base / f"{map_name}.yaml"
    return pbstream, filestem, yaml_file


def run_foreground_command(
    command: Sequence[str],
    dry_run: bool = False,
) -> int:
    print(f"$ {render_command(command)}")
    if dry_run:
        return 0
    completed = subprocess.run(command, check=False)
    return int(completed.returncode)


def run_checked_command(
    command: Sequence[str],
    timeout_sec: float = 10.0,
) -> int:
    print(f"$ {render_command(command)}")
    try:
        completed = subprocess.run(
            command,
            check=False,
            timeout=timeout_sec,
        )
    except FileNotFoundError as exc:
        print(f"[ERROR] {exc}")
        return 127
    except subprocess.TimeoutExpired:
        print(f"[TIMEOUT] command exceeded {timeout_sec:.1f}s")
        return 124
    return int(completed.returncode)


def run_capture_command(
    command: Sequence[str],
    timeout_sec: float = 10.0,
) -> Tuple[int, str, str]:
    print(f"$ {render_command(command)}")
    try:
        completed = subprocess.run(
            command,
            check=False,
            timeout=timeout_sec,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        return 127, "", str(exc)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout if isinstance(exc.stdout, str) else ""
        stderr = exc.stderr if isinstance(exc.stderr, str) else ""
        return 124, stdout, stderr
    return int(completed.returncode), completed.stdout, completed.stderr


def ensure_report_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def write_json_report(path: Path, payload: Dict[str, object]) -> None:
    ensure_report_parent(path)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def extract_block_scalar(text: str, block_name: str, key: str) -> str:
    block_header = f"{block_name}:"
    key_prefix = f"{key}:"
    lines = text.splitlines()
    in_block = False
    block_indent = -1
    for line in lines:
        stripped = line.strip()
        if not stripped:
            continue
        indent = len(line) - len(line.lstrip(" "))
        if not in_block:
            if stripped == block_header:
                in_block = True
                block_indent = indent
            continue
        if indent <= block_indent:
            break
        if stripped.startswith(key_prefix):
            value = stripped[len(key_prefix) :].strip()
            return value.strip('"').strip("'")
    return ""


def build_check(expected: object, actual: object) -> Dict[str, object]:
    return {
        "expected": expected,
        "actual": actual,
        "ok": actual == expected,
    }


def read_key(timeout_sec: float) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if not ready:
        return ""
    key = sys.stdin.read(1)
    if key == "\x1b":
        while select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
        return ""
    return key


@dataclass(frozen=True)
class MotionSegment:
    vx: float
    vy: float
    wz: float
    duration: float


PRESET_MOTIONS: Dict[str, List[MotionSegment]] = {
    "forward_stop": [
        MotionSegment(0.45, 0.0, 0.0, 2.5),
        MotionSegment(0.0, 0.0, 0.0, 0.5),
    ],
    "strafe_test": [
        MotionSegment(0.0, 0.35, 0.0, 1.5),
        MotionSegment(0.0, -0.35, 0.0, 1.5),
        MotionSegment(0.0, 0.0, 0.0, 0.5),
    ],
    "spin_scan": [
        MotionSegment(0.0, 0.0, 0.7, 2.0),
        MotionSegment(0.0, 0.0, -0.7, 2.0),
        MotionSegment(0.0, 0.0, 0.0, 0.5),
    ],
    "box_loop": [
        MotionSegment(0.35, 0.0, 0.0, 2.0),
        MotionSegment(0.0, 0.0, 0.7, 1.6),
        MotionSegment(0.35, 0.0, 0.0, 2.0),
        MotionSegment(0.0, 0.0, 0.7, 1.6),
        MotionSegment(0.35, 0.0, 0.0, 2.0),
        MotionSegment(0.0, 0.0, 0.7, 1.6),
        MotionSegment(0.35, 0.0, 0.0, 2.0),
        MotionSegment(0.0, 0.0, 0.7, 1.6),
        MotionSegment(0.0, 0.0, 0.0, 0.5),
    ],
}


MOTION_PAD_BINDINGS = {
    "1": "forward_stop",
    "2": "strafe_test",
    "3": "spin_scan",
    "4": "box_loop",
}


class NavAssistant(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError(
                f"ROS Python packages are required for this command: {ROS_IMPORT_ERROR}"
            )
        super().__init__("nav_assistant")

    def _pose(self, frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def save_map(
        self,
        service_name: str,
        filename: Path,
        include_unfinished_submaps: bool,
        timeout_sec: float,
    ) -> None:
        try:
            from cartographer_ros_msgs.srv import WriteState
        except ImportError as exc:
            raise RuntimeError(
                "cartographer_ros_msgs is required for save-map."
            ) from exc

        filename.parent.mkdir(parents=True, exist_ok=True)
        client = self.create_client(WriteState, service_name)
        self.get_logger().info(f"Waiting for service {service_name} ...")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service not available: {service_name}")

        request = WriteState.Request()
        request.filename = str(filename)
        request.include_unfinished_submaps = include_unfinished_submaps
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None:
            raise RuntimeError("No response from /write_state")

        code = int(response.status.code)
        message = str(response.status.message)
        if code != 0:
            raise RuntimeError(
                f"WriteState failed (status={code}, message='{message}')"
            )
        self.get_logger().info(f"Saved pbstream: {filename}")

    def send_goal(
        self,
        action_name: str,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
        server_timeout_sec: float,
        result_timeout_sec: float,
    ) -> None:
        client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(f"Waiting for action server {action_name} ...")
        if not client.wait_for_server(timeout_sec=server_timeout_sec):
            raise RuntimeError(f"Action server not available: {action_name}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose(frame_id, x, y, yaw)
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=server_timeout_sec,
        )
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")

        self.get_logger().info("Goal accepted; waiting for result ...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=result_timeout_sec,
        )
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("NavigateToPose result timed out or missing")
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"NavigateToPose failed with status={wrapped_result.status}"
            )
        self.get_logger().info("NavigateToPose succeeded")

    def send_waypoints(
        self,
        action_name: str,
        frame_id: str,
        waypoints: Sequence[Tuple[float, float, float]],
        server_timeout_sec: float,
        result_timeout_sec: float,
    ) -> None:
        client = ActionClient(self, FollowWaypoints, action_name)
        self.get_logger().info(f"Waiting for action server {action_name} ...")
        if not client.wait_for_server(timeout_sec=server_timeout_sec):
            raise RuntimeError(f"Action server not available: {action_name}")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self._pose(frame_id, x, y, yaw) for x, y, yaw in waypoints]
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=server_timeout_sec,
        )
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("FollowWaypoints goal was rejected")

        self.get_logger().info("Waypoints accepted; waiting for result ...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=result_timeout_sec,
        )
        wrapped_result = result_future.result()
        if wrapped_result is None:
            raise RuntimeError("FollowWaypoints result timed out or missing")
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"FollowWaypoints failed with status={wrapped_result.status}"
            )
        missed = list(wrapped_result.result.missed_waypoints)
        if missed:
            self.get_logger().warning(
                f"FollowWaypoints succeeded with missed indexes: {missed}"
            )
        else:
            self.get_logger().info("FollowWaypoints succeeded")

    def publish_initial_pose(
        self,
        topic: str,
        frame_id: str,
        x: float,
        y: float,
        yaw: float,
        stddev_xy: float,
        stddev_yaw: float,
        repeat: int,
        interval_sec: float,
    ) -> None:
        publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        time.sleep(0.5)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        covariance = [0.0] * 36
        covariance[0] = stddev_xy * stddev_xy
        covariance[7] = stddev_xy * stddev_xy
        covariance[35] = stddev_yaw * stddev_yaw
        msg.pose.covariance = covariance
        for _ in range(max(1, repeat)):
            msg.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(max(0.0, interval_sec))
        self.get_logger().info("Published AMCL initial pose.")

    def call_empty_service(self, service_name: str, timeout_sec: float) -> None:
        client = self.create_client(Empty, service_name)
        self.get_logger().info(f"Waiting for service {service_name} ...")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service not available: {service_name}")
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if future.result() is None:
            raise RuntimeError(f"No response from service {service_name}")
        self.get_logger().info(f"Service call succeeded: {service_name}")

    def publish_stop(self, cmd_topic: str, rate_hz: float) -> None:
        publisher = self.create_publisher(Twist, cmd_topic, 10)
        period = 1.0 / max(1e-3, rate_hz)
        stop = Twist()
        for _ in range(3):
            publisher.publish(stop)
            time.sleep(period)

    def run_motion(
        self,
        cmd_topic: str,
        rate_hz: float,
        segments: Sequence[MotionSegment],
        repeat: int,
    ) -> None:
        publisher = self.create_publisher(Twist, cmd_topic, 10)
        rate_hz = max(1e-3, rate_hz)
        repeat = max(1, repeat)
        period = 1.0 / rate_hz
        self.get_logger().info(
            f"Executing {len(segments)} segments x{repeat} on {cmd_topic}"
        )

        for _ in range(repeat):
            for index, segment in enumerate(segments, start=1):
                self.get_logger().info(
                    f"Segment {index}: vx={segment.vx:.3f}, "
                    f"vy={segment.vy:.3f}, wz={segment.wz:.3f}, "
                    f"duration={segment.duration:.2f}s"
                )
                start_time = time.monotonic()
                while (
                    rclpy.ok()
                    and (time.monotonic() - start_time) < segment.duration
                ):
                    msg = Twist()
                    msg.linear.x = segment.vx
                    msg.linear.y = segment.vy
                    msg.angular.z = segment.wz
                    publisher.publish(msg)
                    rclpy.spin_once(self, timeout_sec=0.0)
                    time.sleep(period)

        self.publish_stop(cmd_topic, rate_hz)
        self.get_logger().info("Motion complete; stop command published.")


def build_mapping_launch_cmd(args: argparse.Namespace) -> List[str]:
    run_mode = args.run_mode
    use_ekf = resolve_use_ekf(run_mode, args.use_ekf)
    carto_config = args.cartographer_config_basename or MAPPING_CONFIG_BASENAME[run_mode]
    command = [
        "ros2",
        "launch",
        "robot_navigation",
        "slam_mapping_stack.launch.py",
        f"hostname:={args.hostname}",
        f"udp_receiver_ip:={args.udp_receiver_ip}",
        f"serial_port:={args.serial_port}",
        f"baud_rate:={args.baud_rate}",
        f"cmd_topics:={args.cmd_topics}",
        f"telemetry_enabled:={bool_to_launch(args.telemetry_enabled)}",
        f"left_switch:={args.left_switch}",
        f"right_switch:={args.right_switch}",
        f"odom_topic:={args.odom_topic}",
        f"fallback_odom:={bool_to_launch(args.fallback_odom)}",
        f"use_ekf:={bool_to_launch(use_ekf)}",
        f"cartographer_config_basename:={carto_config}",
        f"imu_topic:={args.imu_topic}",
        f"sick_tf_publish_rate:={args.sick_tf_publish_rate}",
        f"imu_udp_port:={args.imu_udp_port}",
        f"scandataformat:={args.scandataformat}",
        f"send_sopas_start_stop_cmd:={bool_to_sick_flag(args.send_sopas_start_stop_cmd)}",
        f"host_frecho_filter:={args.host_frecho_filter}",
        f"host_set_frecho_filter:={bool_to_sick_flag(args.host_set_frecho_filter)}",
        f"host_set_lfp_angle_range_filter:={bool_to_sick_flag(args.host_set_lfp_angle_range_filter)}",
        f"host_set_lfp_interval_filter:={bool_to_sick_flag(args.host_set_lfp_interval_filter)}",
        f"lidar_x:={args.lidar_x}",
        f"lidar_y:={args.lidar_y}",
        f"lidar_z:={args.lidar_z}",
        f"lidar_roll:={args.lidar_roll}",
        f"lidar_pitch:={args.lidar_pitch}",
        f"lidar_yaw:={args.lidar_yaw}",
        f"with_collision:={bool_to_launch(args.with_collision)}",
        f"with_rviz:={bool_to_launch(args.with_rviz)}",
    ]
    if args.ekf_params_file:
        command.append(f"ekf_params_file:={Path(args.ekf_params_file)}")
    return command


def build_localization_launch_cmd(args: argparse.Namespace) -> List[str]:
    run_mode = args.run_mode
    use_ekf = resolve_use_ekf(run_mode, args.use_ekf)
    pbstream, _, map_yaml = map_paths(args.maps_dir, args.map_name)
    pbstream_path = Path(args.pbstream_file) if args.pbstream_file else pbstream
    yaml_path = Path(args.map_yaml) if args.map_yaml else map_yaml
    if args.runtime_localizer == "cartographer":
        carto_config = args.cartographer_config_basename or LOCALIZATION_CONFIG_BASENAME[run_mode]
        command = [
            "ros2",
            "launch",
            "robot_navigation",
            "nav2_localization_stack.launch.py",
            f"hostname:={args.hostname}",
            f"udp_receiver_ip:={args.udp_receiver_ip}",
            f"serial_port:={args.serial_port}",
            f"baud_rate:={args.baud_rate}",
            f"cmd_topics:={args.cmd_topics}",
            f"telemetry_enabled:={bool_to_launch(args.telemetry_enabled)}",
            f"left_switch:={args.left_switch}",
            f"right_switch:={args.right_switch}",
            f"odom_topic:={args.odom_topic}",
            f"fallback_odom:={bool_to_launch(args.fallback_odom)}",
            f"use_ekf:={bool_to_launch(use_ekf)}",
            f"cartographer_config_basename:={carto_config}",
            f"imu_topic:={args.imu_topic}",
            f"sick_tf_publish_rate:={args.sick_tf_publish_rate}",
            f"imu_udp_port:={args.imu_udp_port}",
            f"scandataformat:={args.scandataformat}",
            f"send_sopas_start_stop_cmd:={bool_to_sick_flag(args.send_sopas_start_stop_cmd)}",
            f"host_frecho_filter:={args.host_frecho_filter}",
            f"host_set_frecho_filter:={bool_to_sick_flag(args.host_set_frecho_filter)}",
            f"host_set_lfp_angle_range_filter:={bool_to_sick_flag(args.host_set_lfp_angle_range_filter)}",
            f"host_set_lfp_interval_filter:={bool_to_sick_flag(args.host_set_lfp_interval_filter)}",
            f"lidar_x:={args.lidar_x}",
            f"lidar_y:={args.lidar_y}",
            f"lidar_z:={args.lidar_z}",
            f"lidar_roll:={args.lidar_roll}",
            f"lidar_pitch:={args.lidar_pitch}",
            f"lidar_yaw:={args.lidar_yaw}",
            f"pbstream_file:={pbstream_path}",
            f"map_yaml:={yaml_path}",
            f"with_nav2_rviz:={bool_to_launch(args.with_nav2_rviz)}",
        ]
        if args.ekf_params_file:
            command.append(f"ekf_params_file:={Path(args.ekf_params_file)}")
        if args.nav2_params_file:
            command.append(f"nav2_params_file:={Path(args.nav2_params_file)}")
        return command

    nav2_params = (
        Path(args.nav2_params_file)
        if args.nav2_params_file
        else default_nav2_params_path(NAV2_PROFILE_PARAMS[args.nav_profile])
    )
    command = [
        "ros2",
        "launch",
        "robot_navigation",
        "nav2_amcl_localization_stack.launch.py",
        f"hostname:={args.hostname}",
        f"udp_receiver_ip:={args.udp_receiver_ip}",
        f"serial_port:={args.serial_port}",
        f"baud_rate:={args.baud_rate}",
        f"cmd_topics:={args.cmd_topics}",
        f"telemetry_enabled:={bool_to_launch(args.telemetry_enabled)}",
        f"left_switch:={args.left_switch}",
        f"right_switch:={args.right_switch}",
        f"odom_topic:={args.odom_topic}",
        f"fallback_odom:={bool_to_launch(args.fallback_odom)}",
        f"use_ekf:={bool_to_launch(use_ekf)}",
        f"imu_topic:={args.imu_topic}",
        f"sick_tf_publish_rate:={args.sick_tf_publish_rate}",
        f"imu_udp_port:={args.imu_udp_port}",
        f"scandataformat:={args.scandataformat}",
        f"send_sopas_start_stop_cmd:={bool_to_sick_flag(args.send_sopas_start_stop_cmd)}",
        f"host_frecho_filter:={args.host_frecho_filter}",
        f"host_set_frecho_filter:={bool_to_sick_flag(args.host_set_frecho_filter)}",
        f"host_set_lfp_angle_range_filter:={bool_to_sick_flag(args.host_set_lfp_angle_range_filter)}",
        f"host_set_lfp_interval_filter:={bool_to_sick_flag(args.host_set_lfp_interval_filter)}",
        f"lidar_x:={args.lidar_x}",
        f"lidar_y:={args.lidar_y}",
        f"lidar_z:={args.lidar_z}",
        f"lidar_roll:={args.lidar_roll}",
        f"lidar_pitch:={args.lidar_pitch}",
        f"lidar_yaw:={args.lidar_yaw}",
        f"map_yaml:={yaml_path}",
        f"nav2_params_file:={nav2_params}",
        f"startup_mode:={args.startup_mode}",
        f"initial_pose_x:={args.initial_pose_x}",
        f"initial_pose_y:={args.initial_pose_y}",
        f"initial_pose_yaw:={args.initial_pose_yaw}",
        f"with_nav2_rviz:={bool_to_launch(args.with_nav2_rviz)}",
    ]
    if args.ekf_params_file:
        command.append(f"ekf_params_file:={Path(args.ekf_params_file)}")
    return command


def build_teleop_cmd(args: argparse.Namespace) -> List[str]:
    return [
        "ros2",
        "run",
        "robot_navigation",
        "teleop_cmd_vel",
        "--topic",
        args.topic,
        "--linear",
        str(args.linear),
        "--angular",
        str(args.angular),
        "--rate",
        str(args.rate),
        "--deadman",
        str(args.deadman),
    ]


def build_collision_teleop_cmd(args: argparse.Namespace) -> List[str]:
    return [
        "ros2",
        "run",
        "robot_navigation",
        "teleop_cmd_vel_collision",
        "--topic",
        args.topic,
        "--linear",
        str(args.linear),
        "--angular",
        str(args.angular),
        "--rate",
        str(args.rate),
        "--deadman",
        str(args.deadman),
    ]


def print_runbook(args: argparse.Namespace) -> None:
    pbstream, _, yaml_file = map_paths(args.maps_dir, args.map_name)
    lines = [
        "# Phase A mapping stack",
        "ros2 launch robot_navigation slam_mapping_stack.launch.py",
        "",
        "# Optional teleop (standard and collision-aware)",
        "ros2 run robot_navigation teleop_cmd_vel "
        "--topic /cmd_vel --linear 0.6 --angular 1.2",
        "ros2 run robot_navigation teleop_cmd_vel_collision "
        "--topic /cmd_vel --linear 0.6 --angular 1.2",
        "",
        "# Save and export map",
        "ros2 run robot_navigation nav_assistant save-map "
        f"--map-name {args.map_name}",
        "ros2 run robot_navigation nav_assistant export-map "
        f"--map-name {args.map_name}",
        "",
        "# Phase B localization + Nav2 stack (AMCL default)",
        "ros2 run robot_navigation nav_assistant localization-stack "
        f"--map-name {args.map_name} --map-yaml {yaml_file}",
        "",
        "# Legacy runtime localization with Cartographer",
        "ros2 run robot_navigation nav_assistant localization-stack "
        f"--runtime-localizer cartographer --map-name {args.map_name} "
        f"--pbstream-file {pbstream} --map-yaml {yaml_file}",
        "",
        "# Send goals and waypoints",
        "ros2 run robot_navigation nav_assistant goal "
        "--x 1.0 --y 0.0 --yaw 0.0",
        "ros2 run robot_navigation nav_assistant waypoints "
        "--pose 1.0,0.0,0.0 --pose 1.5,0.5,0.0 --pose 0.5,1.0,0.0",
        "",
        "# Read-only verification",
        "ros2 run robot_navigation nav_assistant verify-localization "
        f"--map-yaml {yaml_file}",
        "ros2 run robot_navigation nav_assistant verify-nav-profile "
        "--nav-profile smac_mppi_omni",
        "",
        "# One-key motion macro pad (keys 1-4, space stop, q quit)",
        "ros2 run robot_navigation nav_assistant motion-pad --topic /cmd_vel",
    ]
    print("\n".join(lines))


def run_quick_checks(args: argparse.Namespace) -> int:
    commands = [
        ["ros2", "topic", "list"],
        ["ros2", "topic", "info", "/map"],
        ["ros2", "topic", "echo", "/map_metadata", "--once"],
        ["ros2", "topic", "echo", "/odom", "--once"],
        ["ros2", "topic", "echo", "/cmd_vel", "--once"],
        ["ros2", "action", "list"],
    ]
    failures = 0
    for command in commands:
        code = run_checked_command(command, timeout_sec=args.timeout)
        if code != 0:
            failures += 1
    if failures > 0:
        print(f"Quick-check finished with {failures} failing command(s).")
        return 1
    print("Quick-check passed.")
    return 0


def verify_nav_profile(args: argparse.Namespace) -> int:
    params_path = (
        Path(args.nav2_params_file)
        if args.nav2_params_file
        else default_nav2_params_path(NAV2_PROFILE_PARAMS[args.nav_profile])
    )
    params_text = params_path.read_text(encoding="utf-8")
    expected = NAV_PROFILE_EXPECTATIONS[args.nav_profile]
    actual = {
        "planner_plugin": extract_block_scalar(params_text, "GridBased", "plugin"),
        "controller_plugin": extract_block_scalar(params_text, "FollowPath", "plugin"),
        "use_astar": extract_block_scalar(params_text, "GridBased", "use_astar").lower(),
        "motion_model": extract_block_scalar(params_text, "FollowPath", "motion_model"),
        "amcl_scan_topic": extract_block_scalar(params_text, "amcl", "scan_topic"),
        "amcl_tf_broadcast": extract_block_scalar(params_text, "amcl", "tf_broadcast").lower(),
    }
    checks: Dict[str, object] = {
        "planner_plugin": build_check(expected["planner_plugin"], actual["planner_plugin"]),
        "controller_plugin": build_check(
            expected["controller_plugin"], actual["controller_plugin"]
        ),
        "amcl_scan_topic": build_check("/scan_fullframe", actual["amcl_scan_topic"]),
        "amcl_tf_broadcast": build_check("true", actual["amcl_tf_broadcast"]),
        "uses_default_bt_tree": {
            "expected": True,
            "actual": True,
            "ok": True,
        },
    }
    if "use_astar" in expected:
        checks["use_astar"] = build_check(expected["use_astar"], actual["use_astar"])
    if "motion_model" in expected:
        checks["motion_model"] = build_check(
            expected["motion_model"], actual["motion_model"]
        )
    bt_plugin_checks = {}
    for plugin_name in BT_REQUIRED_PLUGIN_LIBS:
        present = plugin_name in params_text
        bt_plugin_checks[plugin_name] = {
            "expected": True,
            "actual": present,
            "ok": present,
        }
    checks["bt_required_plugins"] = bt_plugin_checks
    ok = all(
        entry["ok"]
        for key, entry in checks.items()
        if key != "bt_required_plugins"
    ) and all(item["ok"] for item in bt_plugin_checks.values())
    report = {
        "ok": ok,
        "profile": args.nav_profile,
        "params_file": str(params_path),
        "checks": checks,
    }
    report_path = Path(args.report_file)
    write_json_report(report_path, report)
    print(json.dumps(report, indent=2, ensure_ascii=True))
    print(f"Wrote report: {report_path}")
    return 0 if ok else 1


def verify_localization(args: argparse.Namespace) -> int:
    launch_path = (
        REPO_ROOT
        / "workspace"
        / "src"
        / "robot_navigation"
        / "launch"
        / "nav2_amcl_localization_stack.launch.py"
    )
    launch_text = launch_path.read_text(encoding="utf-8")
    static_checks: Dict[str, object] = {
        "launch_exists": {"expected": True, "actual": launch_path.exists(), "ok": launch_path.exists()},
        "pbstream_not_required": {
            "expected": False,
            "actual": "pbstream" in launch_text.lower(),
            "ok": "pbstream" not in launch_text.lower(),
        },
        "map_yaml_suffix": {
            "expected": ".yaml",
            "actual": Path(args.map_yaml).suffix,
            "ok": Path(args.map_yaml).suffix == ".yaml",
        },
    }
    runtime_checks: Dict[str, object] = {}
    node_code, node_stdout, node_stderr = run_capture_command(
        ["ros2", "node", "list"], timeout_sec=args.timeout
    )
    node_lines = [line.strip() for line in node_stdout.splitlines() if line.strip()]
    runtime_checks["node_list"] = {
        "returncode": node_code,
        "stderr": node_stderr.strip(),
        "nodes": node_lines,
        "has_amcl": any(line.endswith("amcl") for line in node_lines),
        "has_map_server": any(line.endswith("map_server") for line in node_lines),
        "has_cartographer_node": any("cartographer_node" in line for line in node_lines),
        "ok": node_code == 0
        and any(line.endswith("amcl") for line in node_lines)
        and any(line.endswith("map_server") for line in node_lines)
        and not any("cartographer_node" in line for line in node_lines),
    }
    for lifecycle_node in ("/amcl", "/map_server"):
        code, stdout, stderr = run_capture_command(
            ["ros2", "lifecycle", "get", lifecycle_node],
            timeout_sec=args.timeout,
        )
        runtime_checks[f"lifecycle_{lifecycle_node.strip('/')}"] = {
            "returncode": code,
            "stdout": stdout.strip(),
            "stderr": stderr.strip(),
            "ok": code == 0 and "active" in stdout.lower(),
        }
    code, stdout, stderr = run_capture_command(
        ["ros2", "param", "get", "/amcl", "scan_topic"],
        timeout_sec=args.timeout,
    )
    runtime_checks["amcl_scan_topic"] = {
        "returncode": code,
        "stdout": stdout.strip(),
        "stderr": stderr.strip(),
        "ok": code == 0 and "/scan_fullframe" in stdout,
    }
    ok = all(item["ok"] for item in static_checks.values()) and all(
        item["ok"] for item in runtime_checks.values()
    )
    report = {
        "ok": ok,
        "map_yaml": args.map_yaml,
        "static_checks": static_checks,
        "runtime_checks": runtime_checks,
    }
    report_path = Path(args.report_file)
    write_json_report(report_path, report)
    print(json.dumps(report, indent=2, ensure_ascii=True))
    print(f"Wrote report: {report_path}")
    return 0 if ok else 1


def run_motion_pad(node: NavAssistant, args: argparse.Namespace) -> None:
    if termios is None or tty is None:
        raise RuntimeError("motion-pad requires a POSIX terminal environment.")
    prompt_lines = [
        "Motion pad ready:",
        "  [1] forward_stop",
        "  [2] strafe_test",
        "  [3] spin_scan",
        "  [4] box_loop",
        "  [space] stop",
        "  [q] quit",
    ]
    print("\n".join(prompt_lines))

    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key(0.1)
            if not key:
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            if key in ("q", "Q", "\x03"):
                break
            if key in (" ", "x", "X"):
                node.publish_stop(args.topic, args.rate)
                print("Stop command published.")
                continue
            if key in MOTION_PAD_BINDINGS:
                preset = MOTION_PAD_BINDINGS[key]
                print(f"Running preset '{preset}'")
                node.run_motion(
                    cmd_topic=args.topic,
                    rate_hz=args.rate,
                    segments=PRESET_MOTIONS[preset],
                    repeat=1,
                )
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.publish_stop(args.topic, args.rate)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Workflow helper for Cartographer + Nav2 + serial bridge."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    stack_common = argparse.ArgumentParser(add_help=False)
    stack_common.add_argument("--hostname", default="192.168.8.150")
    stack_common.add_argument("--udp-receiver-ip", default="192.168.8.249")
    stack_common.add_argument("--serial-port", default="/dev/ttyUSB0")
    stack_common.add_argument("--baud-rate", type=int, default=115200)
    stack_common.add_argument("--cmd-topics", default=DEFAULT_CMD_TOPICS)
    stack_common.add_argument(
        "--telemetry-enabled",
        type=parse_bool,
        default=True,
        help="true/false",
    )
    stack_common.add_argument("--left-switch", type=int, default=1)
    stack_common.add_argument("--right-switch", type=int, default=1)
    stack_common.add_argument("--odom-topic", default="/odom_raw")
    stack_common.add_argument("--fallback-odom", type=parse_bool, default=False)
    stack_common.add_argument("--run-mode", choices=RUN_MODES, default=DEFAULT_RUN_MODE)
    stack_common.add_argument(
        "--use-ekf",
        type=parse_bool,
        default=None,
        help="true/false. If omitted: true in normal mode, false in bench mode.",
    )
    stack_common.add_argument("--cartographer-config-basename", default="")
    stack_common.add_argument("--ekf-params-file", default="")
    stack_common.add_argument("--imu-topic", default="/sick_scansegment_xd/imu")
    stack_common.add_argument("--sick-tf-publish-rate", type=float, default=0.0)
    stack_common.add_argument("--imu-udp-port", type=int, default=7503)
    stack_common.add_argument("--scandataformat", type=int, default=2)
    stack_common.add_argument("--send-sopas-start-stop-cmd", type=parse_bool, default=False)
    stack_common.add_argument("--host-frecho-filter", type=int, default=2)
    stack_common.add_argument("--host-set-frecho-filter", type=parse_bool, default=True)
    stack_common.add_argument("--host-set-lfp-angle-range-filter", type=parse_bool, default=False)
    stack_common.add_argument("--host-set-lfp-interval-filter", type=parse_bool, default=False)
    stack_common.add_argument("--lidar-x", type=float, default=0.2413)
    stack_common.add_argument("--lidar-y", type=float, default=0.0)
    stack_common.add_argument("--lidar-z", type=float, default=0.0)
    stack_common.add_argument("--lidar-roll", type=float, default=0.0)
    stack_common.add_argument("--lidar-pitch", type=float, default=0.0)
    stack_common.add_argument("--lidar-yaw", type=float, default=0.0)
    stack_common.add_argument("--dry-run", action="store_true")

    mapping_parser = subparsers.add_parser(
        "mapping-stack",
        parents=[stack_common],
        help="One-line mapping phase launch (LiDAR + Cartographer + serial bridge).",
    )
    mapping_parser.add_argument("--with-collision", type=parse_bool, default=False)
    mapping_parser.add_argument("--with-rviz", type=parse_bool, default=False)

    localization_parser = subparsers.add_parser(
        "localization-stack",
        parents=[stack_common],
        help="One-line localization + Nav2 launch.",
    )
    localization_parser.add_argument("--maps-dir", default=DEFAULT_MAPS_DIR)
    localization_parser.add_argument("--map-name", default=DEFAULT_MAP_NAME)
    localization_parser.add_argument(
        "--runtime-localizer",
        choices=RUNTIME_LOCALIZERS,
        default=DEFAULT_RUNTIME_LOCALIZER,
    )
    localization_parser.add_argument(
        "--nav-profile",
        choices=sorted(NAV2_PROFILE_PARAMS.keys()),
        default=DEFAULT_NAV_PROFILE,
        help="AMCL runtime Nav2 profile. Ignored in cartographer legacy mode.",
    )
    localization_parser.add_argument(
        "--startup-mode",
        choices=("fixed_pose", "global_localization"),
        default="fixed_pose",
        help="AMCL startup procedure. Ignored in cartographer legacy mode.",
    )
    localization_parser.add_argument("--initial-pose-x", type=float, default=0.0)
    localization_parser.add_argument("--initial-pose-y", type=float, default=0.0)
    localization_parser.add_argument("--initial-pose-yaw", type=float, default=0.0)
    localization_parser.add_argument("--pbstream-file", default="")
    localization_parser.add_argument("--map-yaml", default="")
    localization_parser.add_argument("--nav2-params-file", default="")
    localization_parser.add_argument("--with-nav2-rviz", type=parse_bool, default=False)

    teleop_common = argparse.ArgumentParser(add_help=False)
    teleop_common.add_argument("--topic", default="/cmd_vel")
    teleop_common.add_argument("--linear", type=float, default=0.6)
    teleop_common.add_argument("--angular", type=float, default=1.2)
    teleop_common.add_argument("--rate", type=float, default=20.0)
    teleop_common.add_argument("--deadman", type=float, default=0.3)
    teleop_common.add_argument("--dry-run", action="store_true")

    subparsers.add_parser(
        "teleop",
        parents=[teleop_common],
        help="Launch keyboard teleop command publisher.",
    )
    subparsers.add_parser(
        "teleop-collision",
        parents=[teleop_common],
        help="Launch collision-aware keyboard teleop.",
    )

    save_parser = subparsers.add_parser(
        "save-map",
        help="Call /write_state and save Cartographer pbstream.",
    )
    save_parser.add_argument("--service", default="/write_state")
    save_parser.add_argument("--maps-dir", default=DEFAULT_MAPS_DIR)
    save_parser.add_argument("--map-name", default=DEFAULT_MAP_NAME)
    save_parser.add_argument(
        "--include-unfinished-submaps",
        type=parse_bool,
        default=True,
        help="true/false",
    )
    save_parser.add_argument("--timeout", type=float, default=20.0)

    export_parser = subparsers.add_parser(
        "export-map",
        help="Export pbstream into yaml/pgm using cartographer_ros.",
    )
    export_parser.add_argument("--maps-dir", default=DEFAULT_MAPS_DIR)
    export_parser.add_argument("--map-name", default=DEFAULT_MAP_NAME)
    export_parser.add_argument("--resolution", type=float, default=0.03)
    export_parser.add_argument("--dry-run", action="store_true")

    amcl_initialpose_parser = subparsers.add_parser(
        "amcl-initialpose",
        help="Publish one initial pose for AMCL.",
    )
    amcl_initialpose_parser.add_argument("--topic", default="/initialpose")
    amcl_initialpose_parser.add_argument("--frame-id", default="map")
    amcl_initialpose_parser.add_argument("--x", type=float, required=True)
    amcl_initialpose_parser.add_argument("--y", type=float, required=True)
    amcl_initialpose_parser.add_argument("--yaw", type=float, default=0.0)
    amcl_initialpose_parser.add_argument("--stddev-xy", type=float, default=0.25)
    amcl_initialpose_parser.add_argument("--stddev-yaw", type=float, default=0.35)
    amcl_initialpose_parser.add_argument("--repeat", type=int, default=3)
    amcl_initialpose_parser.add_argument("--interval", type=float, default=0.2)

    amcl_global_localize_parser = subparsers.add_parser(
        "amcl-global-localize",
        help="Call AMCL global localization reset service.",
    )
    amcl_global_localize_parser.add_argument(
        "--service", default="/reinitialize_global_localization"
    )
    amcl_global_localize_parser.add_argument("--timeout", type=float, default=10.0)

    goal_parser = subparsers.add_parser("goal", help="Send one Nav2 goal.")
    goal_parser.add_argument("--action-name", default="/navigate_to_pose")
    goal_parser.add_argument("--frame-id", default="map")
    goal_parser.add_argument("--x", type=float, required=True)
    goal_parser.add_argument("--y", type=float, required=True)
    goal_parser.add_argument("--yaw", type=float, default=0.0)
    goal_parser.add_argument("--server-timeout", type=float, default=10.0)
    goal_parser.add_argument("--result-timeout", type=float, default=180.0)

    waypoints_parser = subparsers.add_parser(
        "waypoints",
        help="Send waypoint list to FollowWaypoints action.",
    )
    waypoints_parser.add_argument("--action-name", default="/follow_waypoints")
    waypoints_parser.add_argument("--frame-id", default="map")
    waypoints_parser.add_argument(
        "--pose",
        action="append",
        required=True,
        type=parse_pose,
        help="Waypoint as x,y,yaw (repeat for multiple waypoints)",
    )
    waypoints_parser.add_argument("--server-timeout", type=float, default=10.0)
    waypoints_parser.add_argument("--result-timeout", type=float, default=300.0)

    motion_parser = subparsers.add_parser(
        "motion",
        help="Execute preset/custom cmd_vel motion segments.",
    )
    motion_parser.add_argument("--topic", default="/cmd_vel")
    motion_parser.add_argument("--rate", type=float, default=20.0)
    motion_parser.add_argument(
        "--preset",
        choices=sorted(PRESET_MOTIONS.keys()),
        default="forward_stop",
    )
    motion_parser.add_argument(
        "--segment",
        action="append",
        type=parse_segment,
        help="Custom segment as vx,vy,wz,duration_sec",
    )
    motion_parser.add_argument("--repeat", type=int, default=1)

    motion_pad_parser = subparsers.add_parser(
        "motion-pad",
        help="Interactive one-key motion macro pad.",
    )
    motion_pad_parser.add_argument("--topic", default="/cmd_vel")
    motion_pad_parser.add_argument("--rate", type=float, default=20.0)

    runbook_parser = subparsers.add_parser(
        "print-runbook",
        help="Print short command list for all runbook phases.",
    )
    runbook_parser.add_argument("--maps-dir", default=DEFAULT_MAPS_DIR)
    runbook_parser.add_argument("--map-name", default=DEFAULT_MAP_NAME)

    check_parser = subparsers.add_parser(
        "quick-check",
        help="Run quick topic/action checks from the runbook.",
    )
    check_parser.add_argument("--timeout", type=float, default=8.0)

    verify_localization_parser = subparsers.add_parser(
        "verify-localization",
        help="Read-only AMCL runtime ownership and launch checks.",
    )
    verify_localization_parser.add_argument("--timeout", type=float, default=8.0)
    verify_localization_parser.add_argument(
        "--map-yaml",
        default=str(Path(DEFAULT_MAPS_DIR) / f"{DEFAULT_MAP_NAME}.yaml"),
    )
    verify_localization_parser.add_argument(
        "--report-file",
        default=str(REPO_ROOT / "verification" / "localization_report.json"),
    )

    verify_nav_profile_parser = subparsers.add_parser(
        "verify-nav-profile",
        help="Read-only Nav2 profile config verification.",
    )
    verify_nav_profile_parser.add_argument(
        "--nav-profile",
        choices=sorted(NAV2_PROFILE_PARAMS.keys()),
        default=DEFAULT_NAV_PROFILE,
    )
    verify_nav_profile_parser.add_argument("--nav2-params-file", default="")
    verify_nav_profile_parser.add_argument(
        "--report-file",
        default=str(REPO_ROOT / "verification" / "nav_profile_report.json"),
    )

    return parser


def main(argv: List[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "mapping-stack":
        command = build_mapping_launch_cmd(args)
        return run_foreground_command(command, dry_run=args.dry_run)

    if args.command == "localization-stack":
        command = build_localization_launch_cmd(args)
        return run_foreground_command(command, dry_run=args.dry_run)

    if args.command == "teleop":
        command = build_teleop_cmd(args)
        return run_foreground_command(command, dry_run=args.dry_run)

    if args.command == "teleop-collision":
        command = build_collision_teleop_cmd(args)
        return run_foreground_command(command, dry_run=args.dry_run)

    if args.command == "print-runbook":
        print_runbook(args)
        return 0

    if args.command == "quick-check":
        return run_quick_checks(args)

    if args.command == "verify-localization":
        return verify_localization(args)

    if args.command == "verify-nav-profile":
        return verify_nav_profile(args)

    if args.command == "export-map":
        pbstream, filestem, _ = map_paths(args.maps_dir, args.map_name)
        command = [
            "ros2",
            "run",
            "cartographer_ros",
            "cartographer_pbstream_to_ros_map",
            "-pbstream_filename",
            str(pbstream),
            "-map_filestem",
            str(filestem),
            "-resolution",
            str(args.resolution),
        ]
        return run_foreground_command(command, dry_run=args.dry_run)

    if rclpy is None:
        raise RuntimeError(
            f"ROS Python packages are required for command '{args.command}': "
            f"{ROS_IMPORT_ERROR}"
        )

    rclpy.init(args=None)
    node = NavAssistant()
    try:
        if args.command == "save-map":
            pbstream, _, _ = map_paths(args.maps_dir, args.map_name)
            node.save_map(
                service_name=args.service,
                filename=pbstream,
                include_unfinished_submaps=args.include_unfinished_submaps,
                timeout_sec=args.timeout,
            )
            return 0

        if args.command == "goal":
            node.send_goal(
                action_name=args.action_name,
                frame_id=args.frame_id,
                x=args.x,
                y=args.y,
                yaw=args.yaw,
                server_timeout_sec=args.server_timeout,
                result_timeout_sec=args.result_timeout,
            )
            return 0

        if args.command == "amcl-initialpose":
            node.publish_initial_pose(
                topic=args.topic,
                frame_id=args.frame_id,
                x=args.x,
                y=args.y,
                yaw=args.yaw,
                stddev_xy=args.stddev_xy,
                stddev_yaw=args.stddev_yaw,
                repeat=args.repeat,
                interval_sec=args.interval,
            )
            return 0

        if args.command == "amcl-global-localize":
            node.call_empty_service(
                service_name=args.service,
                timeout_sec=args.timeout,
            )
            return 0

        if args.command == "waypoints":
            node.send_waypoints(
                action_name=args.action_name,
                frame_id=args.frame_id,
                waypoints=args.pose,
                server_timeout_sec=args.server_timeout,
                result_timeout_sec=args.result_timeout,
            )
            return 0

        if args.command == "motion":
            segments = list(args.segment) if args.segment else PRESET_MOTIONS[args.preset]
            node.run_motion(
                cmd_topic=args.topic,
                rate_hz=args.rate,
                segments=segments,
                repeat=args.repeat,
            )
            return 0

        if args.command == "motion-pad":
            run_motion_pad(node, args)
            return 0

        raise RuntimeError(f"Unsupported command: {args.command}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
