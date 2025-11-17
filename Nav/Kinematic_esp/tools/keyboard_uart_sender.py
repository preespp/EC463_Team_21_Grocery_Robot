#!/usr/bin/env python3
"""
Simple keyboard-driven UART command sender to emulate Jetson output.

Reads user input for vx vy w (m/s, m/s, rad/s) and continuously streams
JSON lines {"vx": ..., "vy": ..., "w": ...} to the specified serial port.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time

try:
    import serial  # type: ignore
except ImportError as exc:
    print("pyserial 未安装，请先运行 `pip install pyserial` 后再执行本脚本。", file=sys.stderr)
    sys.exit(1)


class CommandState:
    """Thread-safe structure storing the latest velocity command."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._cmd = {"vx": 0.0, "vy": 0.0, "w": 0.0}

    def update(self, vx: float, vy: float, w: float) -> None:
        with self._lock:
            self._cmd = {"vx": vx, "vy": vy, "w": w}

    def snapshot(self) -> dict[str, float]:
        with self._lock:
            return dict(self._cmd)


def tx_loop(ser: serial.SerialBase, state: CommandState, rate_hz: float, stop_event: threading.Event) -> None:
    """Background thread that periodically transmits the latest command."""
    period = 1.0 / rate_hz
    while not stop_event.is_set():
        payload = json.dumps(state.snapshot(), separators=(",", ":")) + "\n"
        try:
            ser.write(payload.encode("utf-8"))
            ser.flush()
        except serial.SerialException as exc:
            print(f"[TX] 写串口失败: {exc}")
            stop_event.set()
            break
        stop_event.wait(period)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="通过键盘输入模拟 Jetson -> ESP32 的 UART JSON 指令。")
    parser.add_argument("port", help="串口号，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    parser.add_argument("--rate", type=float, default=20.0, help="发送频率 Hz（默认 20Hz）")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        print(f"打开串口失败: {exc}", file=sys.stderr)
        return 1

    state = CommandState()
    stop_event = threading.Event()
    worker = threading.Thread(target=tx_loop, args=(ser, state, args.rate, stop_event), daemon=True)
    worker.start()

    print("键盘控制启动，连续向 ESP32 发送 JSON 行：{\"vx\":x,\"vy\":y,\"w\":z}")
    print("输入格式: <vx> <vy> <w>，单位 [m/s, m/s, rad/s]")
    print("输入 stop 将立即发送全零，输入 quit / q 退出程序。")

    try:
        while not stop_event.is_set():
            try:
                line = input("vx vy w > ").strip()
            except EOFError:
                break
            if not line:
                continue
            if line.lower() in {"q", "quit", "exit"}:
                break
            if line.lower() in {"s", "stop"}:
                state.update(0.0, 0.0, 0.0)
                print("发送：停止")
                continue
            parts = line.split()
            if len(parts) != 3:
                print("格式错误，请输入三个数字，例如: 0.2 0.0 0.1")
                continue
            try:
                vx, vy, w = (float(parts[0]), float(parts[1]), float(parts[2]))
            except ValueError:
                print("无法解析数字，请重试。")
                continue
            state.update(vx, vy, w)
            print(f"更新指令 vx={vx:.3f} m/s, vy={vy:.3f} m/s, w={w:.3f} rad/s")
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，准备退出...")
    finally:
        stop_event.set()
        worker.join(timeout=1.0)
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
