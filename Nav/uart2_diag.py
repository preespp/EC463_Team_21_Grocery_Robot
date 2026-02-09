#!/usr/bin/env python3
import argparse
import select
import struct
import sys
import time

if sys.platform.startswith("win"):
    termios = None
    tty = None
else:
    import termios
    import tty

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

PYNPUT_ERROR = None
try:
    from pynput import keyboard as pynput_keyboard
except Exception as exc:  # pragma: no cover - environment dependent
    pynput_keyboard = None
    PYNPUT_ERROR = exc

HEADER = 0xAC
TELEMETRY_HEADER = 0xAB
FRAME_SIZE = 26

DR16_SWITCH_UP = 1
DR16_SWITCH_DOWN = 2
DR16_SWITCH_MIDDLE = 3

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

MOVE_KEYS = ("w", "s", "a", "d", "q", "e")
pressed = set()
recent_key_time = {}


def clamp(value, lo=-1.0, hi=1.0):
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def on_press(key):
    if key == pynput_keyboard.Key.esc:
        return False
    if key in (pynput_keyboard.Key.shift, pynput_keyboard.Key.shift_r):
        pressed.add("shift")
        return
    if key in (pynput_keyboard.Key.ctrl_l, pynput_keyboard.Key.ctrl_r):
        pressed.add("ctrl")
        return
    if hasattr(key, "char") and key.char:
        pressed.add(key.char.lower())


def on_release(key):
    if key in (pynput_keyboard.Key.shift, pynput_keyboard.Key.shift_r):
        pressed.discard("shift")
        return
    if key in (pynput_keyboard.Key.ctrl_l, pynput_keyboard.Key.ctrl_r):
        pressed.discard("ctrl")
        return
    if hasattr(key, "char") and key.char:
        pressed.discard(key.char.lower())


def build_frame(left_switch, right_switch, force_shift=False, force_ctrl=False):
    # Same core logic as tools/pc_control.py
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
    if force_shift:
        key_mask |= 1 << KEY_BITS["shift"]
    if force_ctrl:
        key_mask |= 1 << KEY_BITS["ctrl"]

    payload = struct.pack(
        "<5fHBB",
        0.0,  # right_x
        0.0,  # right_y
        left_x,
        left_y,
        yaw,
        key_mask,
        int(left_switch) & 0xFF,
        int(right_switch) & 0xFF,
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


def clear_motion_keys():
    for k in MOVE_KEYS:
        pressed.discard(k)
        recent_key_time.pop(k, None)


def poll_stdin_keys(key_hold_s):
    stop = False
    now = time.monotonic()

    while select.select([sys.stdin], [], [], 0.0)[0]:
        ch = sys.stdin.read(1)
        if ch == "\x03":  # Ctrl+C
            stop = True
            break
        if ch == "\x1b":  # ESC
            stop = True
            break
        if ch in (" ", "x", "X"):
            clear_motion_keys()
            continue

        if not ch:
            continue
        key_name = ch.lower()
        if key_name in KEY_BITS:
            recent_key_time[key_name] = now
            # Uppercase implies Shift held by user.
            if ch.isupper():
                recent_key_time["shift"] = now

    # Convert recent key timestamps into "pressed" keys.
    now = time.monotonic()
    pressed.clear()
    for name, ts in list(recent_key_time.items()):
        if (now - ts) <= key_hold_s:
            pressed.add(name)
        else:
            del recent_key_time[name]

    return stop


def parse_switch(value):
    ivalue = int(value)
    if ivalue not in (DR16_SWITCH_UP, DR16_SWITCH_DOWN, DR16_SWITCH_MIDDLE):
        raise argparse.ArgumentTypeError("switch must be 1(UP), 2(DOWN), or 3(MIDDLE)")
    return ivalue


def main():
    parser = argparse.ArgumentParser(
        description="UART2 teleop + telemetry diagnostics using PC frame protocol"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--rate", type=float, default=50.0, help="Send rate (Hz)")
    parser.add_argument("--read-chunk", type=int, default=512, help="Bytes to read per cycle")
    parser.add_argument("--diag", action="store_true", help="Print telemetry stats every second")
    parser.add_argument(
        "--backend",
        choices=("auto", "pynput", "stdin"),
        default="auto",
        help="Keyboard backend: auto tries pynput first, then stdin",
    )
    parser.add_argument(
        "--key-hold",
        type=float,
        default=0.25,
        help="For stdin backend: key is considered pressed for this many seconds",
    )
    parser.add_argument("--left-switch", type=parse_switch, default=DR16_SWITCH_UP)
    parser.add_argument("--right-switch", type=parse_switch, default=DR16_SWITCH_UP)
    parser.add_argument("--force-shift", action="store_true", help="Always set Shift bit in key mask")
    parser.add_argument("--force-ctrl", action="store_true", help="Always set Ctrl bit in key mask")
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

    backend = args.backend
    if backend == "auto":
        backend = "pynput" if pynput_keyboard is not None else "stdin"
    if backend == "pynput" and pynput_keyboard is None:
        raise SystemExit(
            "pynput backend unavailable. Import error:\n"
            f"{PYNPUT_ERROR}\n"
            "Use --backend stdin (works over SSH/headless) or fix X display env."
        )
    if backend == "stdin":
        if not sys.stdin.isatty():
            raise SystemExit("stdin backend requires a TTY terminal")
        if termios is None or tty is None:
            raise SystemExit("stdin backend is unavailable on this platform")

    print("Teleop keys: W/S/A/D/Q/E, Space/X stop, ESC or Ctrl+C quit.")
    print(
        f"port={args.port} baud={args.baud} rate={args.rate:.1f}Hz "
        f"backend={backend} diag={args.diag}"
    )
    if backend == "stdin":
        print(f"stdin hold={args.key_hold:.2f}s (for key repeat based press/release)")

    listener = None
    stdin_settings = None
    if backend == "pynput":
        listener = pynput_keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
    else:
        stdin_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    buffer = bytearray()
    tx = rx = hdr = ok = bad = 0
    t0 = time.time()
    last = t0
    stop = False

    try:
        while not stop:
            if args.duration > 0.0 and (time.time() - t0) >= args.duration:
                break
            if listener is not None and not listener.is_alive():
                break

            if backend == "stdin":
                stop = poll_stdin_keys(args.key_hold)
                if stop:
                    break

            frame = build_frame(
                left_switch=args.left_switch,
                right_switch=args.right_switch,
                force_shift=args.force_shift,
                force_ctrl=args.force_ctrl,
            )
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
        pressed.clear()
        try:
            ser.write(
                build_frame(
                    left_switch=args.left_switch,
                    right_switch=args.right_switch,
                    force_shift=args.force_shift,
                    force_ctrl=args.force_ctrl,
                )
            )
        except Exception:
            pass
        ser.close()
        if listener is not None:
            listener.stop()
        if stdin_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, stdin_settings)


if __name__ == "__main__":
    main()
