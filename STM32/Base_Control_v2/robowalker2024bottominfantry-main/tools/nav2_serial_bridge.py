#!/usr/bin/env python3
"""
ROS 2/NAV2 bridge for the RoboWalker 2024 base.

This node reuses the pc_control serial protocol so the Nav2 stack can drive the chassis.
It subscribes to /cmd_vel, streams normalized inputs over UART2, and turns the firmware's
Serialplot telemetry into nav_msgs/Odometry (+ TF) using encoder-based velocities.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import Iterable, List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile

try:
    from tf2_ros import TransformBroadcaster
except ImportError:  # pragma: no cover - tf2_ros should be present on most ROS 2 installs
    TransformBroadcaster = None

try:
    import serial
    from serial import SerialException
except ImportError as exc:  # pragma: no cover - surfaced during node startup
    raise SystemExit("pyserial is required: pip install pyserial") from exc


HEADER = 0xAC
COMMAND_STRUCT = struct.Struct("<5fHBB")
KEY_BITS = {
    "w": 0,
    "s": 1,
    "a": 2,
    "d": 3,
    "shift": 4,
    "ctrl": 5,
    "q": 6,
    "e": 7,
}


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(z: float) -> Tuple[float, float, float, float]:
    half = z * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class Nav2SerialBridge(Node):
    """ROS 2 node that bridges Nav2 commands to the STM32 base controller."""

    def __init__(self) -> None:
        super().__init__("pc_serial_bridge")

        # Command path configuration
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("send_rate", 50.0)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("cmd_timeout", 0.2)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_lateral_speed", 2.0)
        self.declare_parameter("max_yaw_speed", 3.0)
        self.declare_parameter("axis_deadband", 0.05)
        self.declare_parameter("force_shift", True)
        self.declare_parameter("force_ctrl", False)
        self.declare_parameter("left_switch", 1)
        self.declare_parameter("right_switch", 1)

        # Telemetry path configuration
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "world")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("telemetry_header", [0xAB])
        self.declare_parameter("telemetry_channels", 6)
        self.declare_parameter("telemetry_checksum", True)
        self.declare_parameter("telemetry_warn_period", 2.0)
        self.declare_parameter("telemetry_max_linear_abs", 6.0)
        self.declare_parameter("telemetry_max_angular_abs", 20.0)
        self.declare_parameter("telemetry_poll_rate", 500.0)
        self.declare_parameter("serial_read_chunk", 512)
        self.declare_parameter("pose_covariance_diagonal", [1e-3, 1e-3, 1e-2])
        self.declare_parameter("twist_covariance_diagonal", [1e-2, 1e-2, 1e-2])
        self.declare_parameter("fallback_odom", True)
        self.declare_parameter("fallback_timeout", 0.5)

        # Resolve configured values once to avoid repeated Parameter lookups
        self.serial_port = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.send_period = 1.0 / max(1e-3, float(self.get_parameter("send_rate").value))
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_lateral_speed = float(self.get_parameter("max_lateral_speed").value)
        self.max_yaw_speed = float(self.get_parameter("max_yaw_speed").value)
        self.deadband = float(self.get_parameter("axis_deadband").value)
        self.force_shift = bool(self.get_parameter("force_shift").value)
        self.force_ctrl = bool(self.get_parameter("force_ctrl").value)
        self.left_switch = int(self.get_parameter("left_switch").value)
        self.right_switch = int(self.get_parameter("right_switch").value)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        header_param = self.get_parameter("telemetry_header").value
        header_bytes = bytes(int(v) & 0xFF for v in header_param) or bytes([0xAB])
        self.telemetry_header = header_bytes
        self.telemetry_channels = int(self.get_parameter("telemetry_channels").value)
        if self.telemetry_channels <= 0:
            raise ValueError("telemetry_channels must be > 0")
        self.telemetry_checksum = bool(self.get_parameter("telemetry_checksum").value)
        self.telemetry_warn_period = float(self.get_parameter("telemetry_warn_period").value)
        self.telemetry_max_linear_abs = float(self.get_parameter("telemetry_max_linear_abs").value)
        self.telemetry_max_angular_abs = float(self.get_parameter("telemetry_max_angular_abs").value)
        poll_hz = float(self.get_parameter("telemetry_poll_rate").value)
        self.telemetry_period = 1.0 / max(1e-3, poll_hz)
        self.serial_read_chunk = int(self.get_parameter("serial_read_chunk").value)
        pose_cov_diag = self._resolve_diagonal(self.get_parameter("pose_covariance_diagonal").value)
        twist_cov_diag = self._resolve_diagonal(self.get_parameter("twist_covariance_diagonal").value)
        self.pose_covariance = self._covariance_from_diagonal(pose_cov_diag)
        self.twist_covariance = self._covariance_from_diagonal(twist_cov_diag)
        self.fallback_odom = bool(self.get_parameter("fallback_odom").value)
        self.fallback_timeout = float(self.get_parameter("fallback_timeout").value)

        self.telemetry_data_size = self.telemetry_channels * 4
        self.telemetry_frame_size = (
            len(self.telemetry_header)
            + self.telemetry_data_size
            + (1 if self.telemetry_checksum else 0)
        )
        self.telemetry_struct = struct.Struct("<{}f".format(self.telemetry_channels))
        self.command_header = bytes((HEADER,))
        self.command_payload_size = COMMAND_STRUCT.size
        self.command_frame_size = 1 + self.command_payload_size + 1

        qos = QoSProfile(depth=10)
        self.cmd_subscription = self.create_subscription(Twist, self.cmd_topic, self._cmd_callback, qos)
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, qos)

        self.tf_broadcaster = None
        if self.publish_tf:
            if TransformBroadcaster is None:
                self.get_logger().warning("tf2_ros not available; TF output disabled")
                self.publish_tf = False
            else:
                self.tf_broadcaster = TransformBroadcaster(self)

        self.serial_lock = threading.Lock()
        self.serial_conn: serial.Serial | None = None
        self.last_serial_attempt = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_cmd_time: float | None = None
        self.last_cmd_warn = 0.0
        self.last_telemetry_time: float | None = None
        self.last_checksum_warning = 0.0
        self.last_format_hint = 0.0
        self.last_outlier_warning = 0.0
        self.cmd_rx_count = 0
        self.cmd_tx_count = 0
        self.telemetry_ok_frames = 0
        self.telemetry_bad_checksum = 0
        self.command_echo_frames = 0
        self.telemetry_outlier_frames = 0

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.last_odom_stamp_ns: int | None = None

        self.telemetry_buffer = bytearray()

        self.command_timer = self.create_timer(self.send_period, self._send_frame)
        self.telemetry_timer = self.create_timer(self.telemetry_period, self._poll_serial)

        self._open_serial()
        self.get_logger().info(
            f"Nav2 serial bridge ready (port={self.serial_port}, baud={self.baud_rate}, "
            f"cmd_topic={self.cmd_topic}, odom_topic={self.odom_topic}, "
            f"frame={self.frame_id}->{self.child_frame_id}, publish_tf={self.publish_tf}, "
            f"telemetry_header={[hex(b) for b in self.telemetry_header]}, "
            f"telemetry_channels={self.telemetry_channels}, "
            f"telemetry_checksum={self.telemetry_checksum})"
        )

    # --------------------------------------------------------------------- #
    # Setup helpers
    # --------------------------------------------------------------------- #
    def _resolve_diagonal(self, values: Sequence[float]) -> Tuple[float, float, float]:
        result = list(values) if isinstance(values, Iterable) else []
        while len(result) < 3:
            result.append(0.0)
        return (float(result[0]), float(result[1]), float(result[2]))

    def _covariance_from_diagonal(self, diag: Tuple[float, float, float]) -> List[float]:
        cov = [0.0] * 36
        cov[0] = diag[0]
        cov[7] = diag[1]
        cov[35] = diag[2]
        return cov

    def _open_serial(self) -> None:
        now = time.monotonic()
        if now - self.last_serial_attempt < 1.0 and self.serial_conn is None:
            return
        self.last_serial_attempt = now
        try:
            conn = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0,
                write_timeout=0,
            )
        except SerialException as exc:
            self.get_logger().error(f"Unable to open {self.serial_port}: {exc}")
            self.serial_conn = None
            return

        with self.serial_lock:
            self.serial_conn = conn
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
        self.get_logger().info(f"Serial port {self.serial_port} opened")

    # --------------------------------------------------------------------- #
    # ROS callbacks
    # --------------------------------------------------------------------- #
    def _cmd_callback(self, msg: Twist) -> None:
        self.last_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.last_cmd_time = time.monotonic()
        self.cmd_rx_count += 1

    def _send_frame(self) -> None:
        conn = self.serial_conn
        if conn is None or not conn.is_open:
            self._open_serial()
            return

        now = time.monotonic()
        if self.last_cmd_time is None or (now - self.last_cmd_time) > self.cmd_timeout:
            linear_x = linear_y = angular_z = 0.0
            if (now - self.last_cmd_warn) >= 2.0:
                self.get_logger().warning(
                    "No recent /cmd_vel (or timed out); sending zero command frame. "
                    f"(cmd_rx={self.cmd_rx_count}, cmd_tx={self.cmd_tx_count})"
                )
                self.last_cmd_warn = now
        else:
            linear_x, linear_y, angular_z = self.last_cmd

        frame = self._build_command_frame(linear_x, linear_y, angular_z)

        try:
            with self.serial_lock:
                conn.write(frame)
                self.cmd_tx_count += 1
        except SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            self.serial_conn = None
            return

        self._maybe_publish_fallback_odom(linear_x, linear_y, angular_z)

    def _poll_serial(self) -> None:
        conn = self.serial_conn
        if conn is None or not conn.is_open:
            self._open_serial()
            return

        try:
            with self.serial_lock:
                chunk = conn.read(self.serial_read_chunk)
        except SerialException as exc:
            self.get_logger().error(f"Serial read failed: {exc}")
            self.serial_conn = None
            return

        if not chunk:
            return

        self.telemetry_buffer.extend(chunk)
        self._process_telemetry_buffer()

    # --------------------------------------------------------------------- #
    # Frame builders/parsers
    # --------------------------------------------------------------------- #
    def _build_command_frame(self, linear_x: float, linear_y: float, angular_z: float) -> bytes:
        left_y = clamp(linear_x / max(self.max_linear_speed, 1e-6), 1.0)
        left_x = clamp(-linear_y / max(self.max_lateral_speed, 1e-6), 1.0)
        yaw = clamp(-angular_z / max(self.max_yaw_speed, 1e-6), 1.0)
        key_mask = self._build_key_mask(left_x, left_y, yaw)
        payload = COMMAND_STRUCT.pack(
            0.0,  # right_x (unused)
            0.0,  # right_y (unused)
            left_x,
            left_y,
            yaw,
            key_mask,
            self.left_switch,
            self.right_switch,
        )
        checksum = sum(payload) & 0xFF
        return bytes((HEADER,)) + payload + bytes((checksum,))

    def _build_key_mask(self, left_x: float, left_y: float, yaw: float) -> int:
        mask = 0
        if left_y > self.deadband:
            mask |= 1 << KEY_BITS["w"]
        elif left_y < -self.deadband:
            mask |= 1 << KEY_BITS["s"]

        if left_x > self.deadband:
            mask |= 1 << KEY_BITS["d"]
        elif left_x < -self.deadband:
            mask |= 1 << KEY_BITS["a"]

        if yaw > self.deadband:
            mask |= 1 << KEY_BITS["e"]
        elif yaw < -self.deadband:
            mask |= 1 << KEY_BITS["q"]

        if self.force_shift:
            mask |= 1 << KEY_BITS["shift"]
        if self.force_ctrl:
            mask |= 1 << KEY_BITS["ctrl"]
        return mask

    def _process_telemetry_buffer(self) -> None:
        header = self.telemetry_header
        header_len = len(header)
        command_header = self.command_header
        while True:
            if len(self.telemetry_buffer) < min(self.telemetry_frame_size, self.command_frame_size):
                return

            telemetry_start = self.telemetry_buffer.find(header)
            command_start = -1
            if command_header != header:
                command_start = self.telemetry_buffer.find(command_header)

            # Telemetry header is at the current buffer start.
            if telemetry_start == 0:
                data_start = header_len
                data_end = data_start + self.telemetry_data_size
                frame_end = self.telemetry_frame_size
                frame = self.telemetry_buffer[:frame_end]
                data_bytes = frame[data_start:data_end]
                if len(data_bytes) != self.telemetry_data_size:
                    del self.telemetry_buffer[:1]
                    continue

                if self.telemetry_checksum:
                    checksum = frame[-1]
                    if (sum(data_bytes) & 0xFF) != checksum:
                        self.telemetry_bad_checksum += 1
                        now = time.monotonic()
                        if self.telemetry_warn_period >= 0.0 and (now - self.last_checksum_warning) >= self.telemetry_warn_period:
                            self.get_logger().warning(
                                "Telemetry checksum mismatch dropped frame "
                                f"(bad={self.telemetry_bad_checksum}, ok={self.telemetry_ok_frames}, "
                                f"echoed_cmd={self.command_echo_frames})"
                            )
                            if self.telemetry_ok_frames == 0 and self.telemetry_bad_checksum >= 50:
                                if (now - self.last_format_hint) >= 5.0:
                                    self.get_logger().warning(
                                        "No valid telemetry frame decoded yet. "
                                        "Likely mismatch: baud_rate (try 1000000), "
                                        "telemetry_channels (try 10), or checksum setting "
                                        "(try telemetry_checksum=False). Also verify the flashed firmware build."
                                    )
                                    self.last_format_hint = now
                            self.last_checksum_warning = now
                        # Resync on bad checksum by advancing one byte.
                        del self.telemetry_buffer[:1]
                        continue

                del self.telemetry_buffer[:frame_end]

                try:
                    values = self.telemetry_struct.unpack(data_bytes)
                except struct.error:
                    continue
                if not self._telemetry_plausible(values):
                    self.telemetry_outlier_frames += 1
                    now = time.monotonic()
                    if self.telemetry_warn_period >= 0.0 and (now - self.last_outlier_warning) >= self.telemetry_warn_period:
                        self.get_logger().warning(
                            "Telemetry outlier dropped "
                            f"(outlier={self.telemetry_outlier_frames}, bad_checksum={self.telemetry_bad_checksum}, "
                            f"ok={self.telemetry_ok_frames})"
                        )
                        self.last_outlier_warning = now
                    continue

                self.telemetry_ok_frames += 1
                self.last_telemetry_time = time.monotonic()
                self._handle_telemetry(values)
                continue

            # Some serial links echo host TX bytes back to RX. Drop echoed command frames.
            if command_start == 0:
                cmd_frame = self.telemetry_buffer[:self.command_frame_size]
                if len(cmd_frame) < self.command_frame_size:
                    return
                cmd_payload = cmd_frame[1:-1]
                cmd_checksum = cmd_frame[-1]
                if (sum(cmd_payload) & 0xFF) == cmd_checksum:
                    del self.telemetry_buffer[:self.command_frame_size]
                    self.command_echo_frames += 1
                    continue
                del self.telemetry_buffer[:1]
                continue

            # No valid sync byte at current start; seek the next potential frame boundary.
            candidates = [idx for idx in (telemetry_start, command_start) if idx > 0]
            if not candidates:
                self.telemetry_buffer.clear()
                return
            del self.telemetry_buffer[:min(candidates)]

    def _telemetry_plausible(self, values: Tuple[float, ...]) -> bool:
        for value in values:
            if not math.isfinite(value):
                return False

        if len(values) >= 6:
            target_vx, target_vy, target_omega = values[:3]
            now_vx, now_vy, now_omega = values[3:6]
            lin_limit = max(0.1, self.telemetry_max_linear_abs)
            ang_limit = max(0.1, self.telemetry_max_angular_abs)

            if abs(now_vx) > lin_limit or abs(now_vy) > lin_limit or abs(now_omega) > ang_limit:
                return False
            if abs(target_vx) > (2.0 * lin_limit) or abs(target_vy) > (2.0 * lin_limit) or abs(target_omega) > (2.0 * ang_limit):
                return False

        return True

    # --------------------------------------------------------------------- #
    # Odometry publishing
    # --------------------------------------------------------------------- #
    def _handle_telemetry(self, values: Tuple[float, ...]) -> None:
        if len(values) < 6:
            return

        target_vx, target_vy, target_omega = values[:3]
        now_vx, now_vy, now_omega = values[3:6]
        wheel_data = values[6:]

        now_stamp = self.get_clock().now()
        now_ns = now_stamp.nanoseconds
        self._update_pose(now_vx, now_vy, now_omega, now_ns)
        self._publish_odom(now_stamp, now_vx, now_vy, now_omega)

        # Optional debug output
        self.get_logger().debug(
            f"Cmd target(vx={target_vx:.3f}, vy={target_vy:.3f}, w={target_omega:.3f}) "
            f"now(vx={now_vx:.3f}, vy={now_vy:.3f}, w={now_omega:.3f}) "
            f"wheels={', '.join(f'{w:.1f}' for w in wheel_data)}"
        )

    def _update_pose(self, vx: float, vy: float, omega: float, now_ns: int) -> None:
        if self.last_odom_stamp_ns is not None:
            dt = (now_ns - self.last_odom_stamp_ns) * 1e-9
            if dt > 0.0:
                dx_body = vx * dt
                dy_body = vy * dt
                cos_yaw = math.cos(self.pose_yaw)
                sin_yaw = math.sin(self.pose_yaw)
                self.pose_x += cos_yaw * dx_body - sin_yaw * dy_body
                self.pose_y += sin_yaw * dx_body + cos_yaw * dy_body
                self.pose_yaw = normalize_angle(self.pose_yaw + omega * dt)
        self.last_odom_stamp_ns = now_ns

    def _publish_odom(self, stamp, vx: float, vy: float, omega: float) -> None:
        orientation = yaw_to_quaternion(self.pose_yaw)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]
        odom.pose.covariance = list(self.pose_covariance)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        odom.twist.covariance = list(self.twist_covariance)

        self.odom_publisher.publish(odom)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = odom.header.stamp
            transform.header.frame_id = self.frame_id
            transform.child_frame_id = self.child_frame_id
            transform.transform.translation.x = self.pose_x
            transform.transform.translation.y = self.pose_y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = orientation[0]
            transform.transform.rotation.y = orientation[1]
            transform.transform.rotation.z = orientation[2]
            transform.transform.rotation.w = orientation[3]
            self.tf_broadcaster.sendTransform(transform)

    def _maybe_publish_fallback_odom(self, vx: float, vy: float, omega: float) -> None:
        if not self.fallback_odom:
            return
        now = time.monotonic()
        if self.last_telemetry_time is not None and (now - self.last_telemetry_time) <= self.fallback_timeout:
            return

        now_stamp = self.get_clock().now()
        self._update_pose(vx, vy, omega, now_stamp.nanoseconds)
        self._publish_odom(now_stamp, vx, vy, omega)

    # --------------------------------------------------------------------- #
    # Shutdown
    # --------------------------------------------------------------------- #
    def destroy_node(self) -> bool:
        if self.serial_conn is not None:
            try:
                with self.serial_lock:
                    self.serial_conn.close()
            except SerialException:
                pass
            self.serial_conn = None
        return super().destroy_node()


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Nav2SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
