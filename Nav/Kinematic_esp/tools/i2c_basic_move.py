#!/usr/bin/env python3
"""
Jetson WASD teleop over I2C for ESP32 mecanum controller.

Continuously writes packed little-endian floats {vx, vy, w} to the ESP32 I2C
slave address (default 0x46 on /dev/i2c-7), based on keyboard input:

  w/s : forward/backward   (vx)
  a/d : left/right strafe  (vy)
  q/e : rotate left/right  (w)
  space/x : stop
"""

import argparse
import select
import struct
import sys
import time
import termios
import tty

from smbus2 import SMBus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="WASD teleop: send mecanum cmd_vel packets over Jetson I2C."
    )
    parser.add_argument("--bus", type=int, default=7, help="I2C bus number (default: 7)")
    parser.add_argument(
        "--addr",
        type=lambda x: int(x, 0),
        default=0x46,
        help="Slave address (default: 0x46)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50.0,
        help="Send rate in Hz (default: 50)",
    )
    parser.add_argument(
        "--lin-speed",
        type=float,
        default=0.52,
        help="Linear speed (m/s) for WASD (default: 0.52)",
    )
    parser.add_argument(
        "--rot-speed",
        type=float,
        default=0.683,
        help="Angular speed (rad/s) for Q/E (default: 0.683)",
    )
    parser.add_argument(
        "--hold-timeout",
        type=float,
        default=3.0,
        help=(
            "Seconds without receiving a key before sending stop "
            "(vx=vy=w=0). Default: 5.0"
        ),
    )
    return parser.parse_args()


def get_key(timeout: float) -> str | None:
    """Read a single key with timeout (non-blocking-ish)."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if sys.stdin in rlist:
        ch = sys.stdin.read(1)
        return ch
    return None


def key_to_cmd(
    key: str,
    lin_speed: float,
    rot_speed: float,
) -> tuple[float, float, float] | None:
    """
    Map keys to (vx, vy, w).

    Returns:
      (vx, vy, w) if this key changes motion,
      None if the key should be ignored (no change).
    """
    k = key.lower()

    # Ctrl-C / Ctrl-D handling left to main (KeyboardInterrupt)
    if k == "w":
        return lin_speed, 0.0, 0.0
    if k == "s":
        return -lin_speed, 0.0, 0.0
    if k == "a":
        return 0.0, lin_speed, 0.0      # left strafe (vy > 0)
    if k == "d":
        return 0.0, -lin_speed, 0.0     # right strafe (vy < 0)
    if k == "q":
        return 0.0, 0.0, rot_speed      # rotate left
    if k == "e":
        return 0.0, 0.0, -rot_speed     # rotate right
    if k == " " or k == "x":
        return 0.0, 0.0, 0.0            # explicit stop

    # Any other key: ignore
    return None


def main() -> int:
    args = parse_args()

    try:
        bus = SMBus(args.bus)
    except FileNotFoundError as exc:
        print(f"Cannot open /dev/i2c-{args.bus}: {exc}", file=sys.stderr)
        return 1

    print(f"I2C sender ready on /dev/i2c-{args.bus}, address 0x{args.addr:02X}")
    print(
        "Controls:\n"
        "  w/s : forward/backward (vx)\n"
        "  a/d : left/right strafe (vy)\n"
        "  q/e : rotate left/right (w)\n"
        "  space or x : stop\n"
        "  Ctrl+C : quit\n"
    )

    # Current commanded velocities
    vx = vy = w = 0.0
    last_cmd_time = 0.0  # time of last motion-changing key
    period = 1.0 / args.rate

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)  # get single keypresses without waiting for ENTER

    try:
        try:
            while True:
                # Read key (if any) during this cycle
                key = get_key(period)
                now = time.monotonic()

                if key is not None:
                    # Handle Ctrl-C/Ctrl-D manually if needed
                    if key == "\x03":  # Ctrl-C
                        raise KeyboardInterrupt
                    if key == "\x04":  # Ctrl-D (EOF)
                        break

                    new_cmd = key_to_cmd(key, args.lin_speed, args.rot_speed)
                    if new_cmd is not None:
                        new_vx, new_vy, new_w = new_cmd
                        if (new_vx, new_vy, new_w) != (vx, vy, w):
                            vx, vy, w = new_vx, new_vy, new_w
                            last_cmd_time = now
                            print(
                                f"[KEY '{key}'] vx={vx:.3f}, vy={vy:.3f}, w={w:.3f}"
                            )

                # If no keys for a while, force stop
                if (vx != 0.0 or vy != 0.0 or w != 0.0) and (
                    now - last_cmd_time > args.hold_timeout
                ):
                    vx = vy = w = 0.0
                    print("[TIMEOUT] No key recently, sending stop.")

                # Build and send packet every loop
                packet = struct.pack("<fff", vx, vy, w)
                try:
                    bus.write_i2c_block_data(args.addr, 0x00, list(packet))
                except OSError as exc:
                    print(f"[I2C ERROR] {exc}", file=sys.stderr)

                # Optional: print periodic status
                # print(f"[TX] vx={vx:.3f}, vy={vy:.3f}, w={w:.3f}")

        except KeyboardInterrupt:
            print("\nStopping (Ctrl+C)...")

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        bus.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

