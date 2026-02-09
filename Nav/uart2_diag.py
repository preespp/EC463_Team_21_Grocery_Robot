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
TELEMETRY_HEADER = 0xAB
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
    if key in (keyboard.Key.shift, keyboard.Key.shift_r):
        pressed.add("shift")
        return
    if key in (keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
        pressed.add("ctrl")
        return
    if hasattr(key, "char") and key.char:
        pressed.add(key.char.lower())


def on_release(key):
    if key in (keyboard.Key.shift, keyboard.Key.shift_r):
        pressed.discard("shift")
        return
    if key in (keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
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
    # Follow the same control logic as tools/pc_control.py
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

    payload = struct.pack(
        "<5fHBB",
        0.0,  # right_x
        0.0,  # right_y
        left_x,
        left_y,
        yaw,
        key_mask,
        DR16_SWITCH_UP,
        DR16_SWITCH_UP,
    )
    checksum = sum(payload) & 0xFF
    return bytes((HEADER,)) + payload + bytes((checksum,))


def parse_telemetry(buffer, checksum_enabled=True):
    hdr = ok = bad = 0
    while True:
        i = buffer.find(bytes((TELEMETRY_HEADER,)))
        if i < 0:
            break
        hdr += 1
        if len(buffer) - i < FRAME_SIZE:
            if i > 0:
                del buffer[:i]
            break

        frame = buffer[i : i + FRAME_SIZE]
        del buffer[: i + FRAME_SIZE]

        if checksum_enabled:
            data = frame[1:25]
            if (sum(data) & 0xFF) != frame[25]:
                bad += 1
                continue

        ok += 1

    return hdr, ok, bad


def main():
    parser = argparse.ArgumentParser(
        description="UART2 teleop + telemetry diagnostics (PC frame protocol)"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--rate", type=float, default=50.0, help="Send rate (Hz)")
    parser.add_argument("--read-chunk", type=int, default=512, help="Bytes to read per cycle")
    parser.add_argument("--diag", action="store_true", help="Print telemetry stats every second")
    parser.add_argument(
        "--telemetry-checksum",
        dest="telemetry_checksum",
        action="store_true",
        default=True,
        help="Enable checksum check for telemetry stats (default: enabled)",
    )
    parser.add_argument(
        "--no-telemetry-checksum",
        dest="telemetry_checksum",
        action="store_false",
        help="Disable checksum check for telemetry stats",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Run seconds (0 = until ESC/Ctrl+C)")
    args = parser.parse_args()

    period = 1.0 / max(1e-3, args.rate)
    ser = serial.Serial(args.port, args.baud, timeout=0)

    print("Teleop ready: W/S/A/D/Q/E, Shift/Ctrl optional, ESC to quit.")
    print(f"port={args.port} baud={args.baud} rate={args.rate:.1f}Hz diag={args.diag}")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    buffer = bytearray()
    tx = rx = hdr = ok = bad = 0
    t0 = time.time()
    last = t0

    try:
        while listener.is_alive():
            if args.duration > 0.0 and (time.time() - t0) >= args.duration:
                break

            frame = build_frame()
            ser.write(frame)
            tx += 1

            chunk = ser.read(args.read_chunk)
            if chunk:
                rx += len(chunk)
                buffer.extend(chunk)
                h, o, b = parse_telemetry(buffer, checksum_enabled=args.telemetry_checksum)
                hdr += h
                ok += o
                bad += b

            now = time.time()
            if args.diag and (now - last) >= 1.0:
                keys = "".join(sorted(pressed)) if pressed else "-"
                print(f"tx={tx} rx_bytes={rx} hdr={hdr} ok={ok} bad={bad} keys={keys}")
                last = now

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        # Send one zero frame on exit
        pressed.clear()
        try:
            ser.write(build_frame())
        except Exception:
            pass
        ser.close()
        listener.stop()


if __name__ == "__main__":
    main()

