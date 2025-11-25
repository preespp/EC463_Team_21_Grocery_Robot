#!/usr/bin/env python3
"""
Simple Jetson I2C command sender for the ESP32 mecanum controller.

Writes packed little-endian floats {vx, vy, w} to the ESP32 I2C slave
address (default 0x46 on /dev/i2c-7).
"""
import argparse
import select
import struct
import sys
import time

from smbus2 import SMBus


def decode_cmd(line: str) -> tuple[float, float, float]:
    line = line.strip()
    if not line:
        return 0.0, 0.0, 0.0
    cmd = line[0].upper()
    value = 0.0
    if len(line) > 1:
        try:
            value = float(line[1:])
        except ValueError:
            value = 0.0

    vx = vy = w = 0.0
    if cmd == "F":
        vx = abs(value)
    elif cmd == "B":
        vx = -abs(value)
    elif cmd == "L":
        vy = abs(value)
    elif cmd == "R":
        vy = -abs(value)
    elif cmd == "W":
        w = value
    elif cmd == "S":
        pass
    else:
        print(f"[WARN] Unknown command '{cmd}', treating as stop.")
    return vx, vy, w


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send mecanum cmd_vel packets over Jetson I2C.")
    parser.add_argument("--bus", type=int, default=7, help="I2C bus number (default: 7)")
    parser.add_argument("--addr", type=lambda x: int(x, 0), default=0x46, help="Slave address (default: 0x46)")
    parser.add_argument("--rate", type=float, default=10.0, help="Repeat rate in Hz (default: 10)")
    parser.add_argument("--no-loop", action="store_true", help="Send each command once, no periodic resend")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        bus = SMBus(args.bus)
    except FileNotFoundError as exc:
        print(f"Cannot open /dev/i2c-{args.bus}: {exc}", file=sys.stderr)
        return 1

    print(f"I2C sender ready on /dev/i2c-{args.bus}, address 0x{args.addr:02X}")
    print("Commands: F0.2 / B0.2 / L0.2 / R0.2 / W0.4 / S")
    last_packet: bytes | None = None
    last_line: str | None = None
    next_send_time = 0.0

    try:
        while True:
            now = time.monotonic()
            if (not args.no_loop) and last_packet and now >= next_send_time:
                bus.write_i2c_block_data(args.addr, 0x00, list(last_packet))
                if last_line is not None:
                    vx, vy, w = decode_cmd(last_line)
                    print(f"[TX-loop] {last_line} -> vx={vx:.3f}, vy={vy:.3f}, w={w:.3f}")
                next_send_time = now + (1.0 / args.rate)

            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if sys.stdin in rlist:
                line = sys.stdin.readline()
                if line == "":
                    break
                line = line.strip()
                if not line:
                    continue
                vx, vy, w = decode_cmd(line)
                packet = struct.pack("<fff", vx, vy, w)
                bus.write_i2c_block_data(args.addr, 0x00, list(packet))
                print(f"[TX] {line} -> vx={vx:.3f}, vy={vy:.3f}, w={w:.3f}")
                last_packet = packet
                last_line = line
                next_send_time = time.monotonic() + (0.0 if args.no_loop else (1.0 / args.rate))
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
