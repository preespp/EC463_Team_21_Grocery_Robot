#!/usr/bin/env python3
"""
Keyboard teleop for 5 servo joints over I2C (NO ROS2).

Sends 5 floats via I2C to ESP32:
    struct.pack("<fffff", j1, j2, j3, j4, j5)

Keys:
    1 / !  → servo1 -/+ 
    2 / @  → servo2 -/+ 
    3 / #  → servo3 -/+ 
    4 / $  → servo4 -/+ 
    5 / %  → servo5 -/+ 
    SPACE  → zero all
    q      → quit
"""

import struct
import time
import sys
import termios
import tty
import select
from smbus2 import SMBus

# ---------------------------
# Config
# ---------------------------
I2C_BUS = 7
I2C_ADDR = 0x08      # your ESP32 slave address
RATE_HZ = 30.0        # send rate
STEP = 15.0            # change per key press
MIN_ANGLE = 0.0
MAX_ANGLE = 180.0

def get_key(timeout):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    print(f"I2C Servo Teleop (/dev/i2c-{I2C_BUS}, addr 0x{I2C_ADDR:02X})")
    print("Controls:")
    print("   1 / ! → servo1 -/+")
    print("   2 / @ → servo2 -/+")
    print("   3 / # → servo3 -/+")
    print("   4 / $ → servo4 -/+")
    print("   5 / % → servo5 -/+")
    print("   SPACE → zero all")
    print("   q     → quit\n")

    joints = [90.0, 90.0, 90.0, 90.0, 90.0]   # start mid-position
    period = 1.0 / RATE_HZ

    try:
        bus = SMBBus(I2C_BUS)
    except FileNotFoundError:
        print(f"Error: Cannot open /dev/i2c-{I2C_BUS}")
        return

    # Setup non-blocking terminal
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    servo_keys = {
        "1": (0, -STEP), "!": (0, +STEP),
        "2": (1, -STEP), "@": (1, +STEP),
        "3": (2, -STEP), "#": (2, +STEP),
        "4": (3, -STEP), "$": (3, +STEP),
        "5": (4, -STEP), "%": (4, +STEP),
    }

    try:
        while True:
            key = get_key(period)

            if key is not None:
                if key == "q":
                    print("Exiting…")
                    break

                if key == " ":
                    joints = [0.0] * 5
                    print("[RESET] all → 0")
                elif key in servo_keys:
                    idx, delta = servo_keys[key]
                    joints[idx] += delta
                    joints[idx] = max(MIN_ANGLE, min(MAX_ANGLE, joints[idx]))
                    print(f"[JOINT {idx+1}] → {joints[idx]:.1f}")

            # Pack as 5 floats
            packet = struct.pack("<fffff", *joints)

            try:
                bus.write_i2c_block_data(I2C_ADDR, 0x00, list(packet))
            except OSError as e:
                print(f"[I2C ERROR] {e}")

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        bus.close()


if __name__ == "__main__":
    main()
