#!/usr/bin/env python3
"""
Interactive UART command sender for Jetson (Orin Nano / Xavier / etc.).

Reads command strings from stdin and forwards them to the ESP32 mecanum
controller over /dev/ttyTHS1 (default) using the format expected by the
firmware (e.g. "F0.2", "L0.1", "W-0.3", "S").
"""
import argparse
import sys
import time

import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send mecanum cmd_vel strings over UART.")
    parser.add_argument(
        "--port",
        default="/dev/ttyTHS1",
        help="UART device (default: /dev/ttyTHS1). Use /dev/ttyUSBx if using a USB-UART adapter.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate to match ESP32 firmware (default: 115200).",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Optional resend rate in Hz for the last command (default: 10 Hz).",
    )
    parser.add_argument(
        "--no-loop",
        action="store_true",
        help="Disable periodic re-send, send each line once only.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    print("Jetson UART command sender ready.")
    print("Format: F0.2 / B0.2 / L0.2 / R0.2 / W0.4 / S (stop). Press Ctrl+C to exit.")
    last_payload: bytes | None = None
    next_send_time = 0.0

    try:
        while True:
            # Periodically re-send last command if rate enabled
            now = time.monotonic()
            if (not args.no_loop) and last_payload and now >= next_send_time:
                ser.write(last_payload)
                ser.flush()
                next_send_time = now + (1.0 / args.rate)

            if sys.stdin in select.select([sys.stdin], [], [], 0.05)[0]:
                line = sys.stdin.readline()
                if line == "":
                    break  # EOF
                line = line.strip()
                if not line:
                    continue
                payload = (line + "\n").encode("utf-8")
                ser.write(payload)
                ser.flush()
                print(f"[TX] {line}")
                last_payload = payload
                next_send_time = time.monotonic() + (0.0 if args.no_loop else (1.0 / args.rate))
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    import select

    sys.exit(main())
