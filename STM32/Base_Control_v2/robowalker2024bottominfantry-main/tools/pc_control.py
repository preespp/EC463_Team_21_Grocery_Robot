#!/usr/bin/env python3
import argparse
import struct
import time

try:
    from pynput import keyboard
except ImportError as exc:
    raise SystemExit("pynput is required: pip install pynput") from exc

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

HEADER = 0xAC
PAYLOAD_SIZE = 24
FRAME_SIZE = 26

DR16_SWITCH_UP = 1

KEY_BITS = {
    "w": 0,
    "s": 1,
    "a": 2,
    "d": 3,
    "shift": 4,
    "ctrl": 5,
    "q": 6,
    "e": 7,
    "r": 8,
    "f": 9,
    "g": 10,
    "z": 11,
    "x": 12,
    "c": 13,
    "v": 14,
    "b": 15,
}

pressed = set()


def on_press(key):
    if key == keyboard.Key.esc:
        return False
    if key == keyboard.Key.shift or key == keyboard.Key.shift_r:
        pressed.add("shift")
        return
    if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
        pressed.add("ctrl")
        return
    if hasattr(key, "char") and key.char:
        pressed.add(key.char.lower())


def on_release(key):
    if key == keyboard.Key.shift or key == keyboard.Key.shift_r:
        pressed.discard("shift")
        return
    if key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
        pressed.discard("ctrl")
        return
    if hasattr(key, "char") and key.char:
        pressed.discard(key.char.lower())


def clamp(value, lo=-1.0, hi=1.0):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def build_frame():
    left_x = 0.0
    left_y = 0.0
    yaw = 0.0

    if "w" in pressed:
        left_y += 1.0
    if "s" in pressed:
        left_y -= 1.0
    if "a" in pressed:
        left_x -= 1.0
    if "d" in pressed:
        left_x += 1.0

    if "q" in pressed:
        yaw -= 1.0
    if "e" in pressed:
        yaw += 1.0

    left_x = clamp(left_x)
    left_y = clamp(left_y)
    yaw = clamp(yaw)

    key_mask = 0
    for name, bit in KEY_BITS.items():
        if name in pressed:
            key_mask |= (1 << bit)

    right_x = 0.0
    right_y = 0.0
    switch_left = DR16_SWITCH_UP
    switch_right = DR16_SWITCH_UP

    payload = struct.pack(
        "<5fHBB",
        right_x,
        right_y,
        left_x,
        left_y,
        yaw,
        key_mask,
        switch_left,
        switch_right,
    )
    checksum = sum(payload) & 0xFF
    return bytes([HEADER]) + payload + bytes([checksum])


def main():
    parser = argparse.ArgumentParser(description="PC control sender for chassis")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM5")
    parser.add_argument("--baud", type=int, default=1000000, help="Baud rate")
    parser.add_argument("--rate", type=float, default=50.0, help="Send rate (Hz)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0)
    period = 1.0 / args.rate

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while listener.is_alive():
            frame = build_frame()
            ser.write(frame)
            time.sleep(period)
    finally:
        ser.close()
        listener.stop()


if __name__ == "__main__":
    main()
