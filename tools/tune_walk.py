"""
One-command PID trace of a Mini Pupper WALKING (Stanford trot gait).

This is the walk-time sibling of tune_servo.py. Instead of stepping one servo
low<->high, it runs the firmware `swalk` command: the ESP32 drives the real
Stanford trot gait for a few seconds and streams per-servo tracking CSV
(setpoint / actual / error / current / duty), framed like:

    #WALK_BEGIN secs=6 hz=50 vx=100.0 ids=2-3
    t_ms,set2,now2,err2,cur2,duty2,set3,now3,err3,cur3,duty3
    0,101.2,101.0,0.2,95,42.5,88.1,87.9,0.2,110,40.0
    ...
    #WALK_END

So you can tune kp/ki/kd against the actual walk motion (not just a step) for
as many servos as you like at once (default: id 2 and id 3).

Talks to the ESP32 directly over the USB serial (COM) port: enables CLI mode,
optionally sets gains on each traced servo, runs `swalk`, captures the CSV,
saves it, and calls plot_walk.py to make the PNG. No idf.py monitor, no
Ctrl-T/Ctrl-L, no copy-paste.

IMPORTANT: close `idf.py monitor` first - only one program can use the COM
port at a time.

Setup (once):
    pip install pyserial matplotlib

Usage:
    python tune_walk.py --port COM7
    python tune_walk.py --port COM7 --kp 40 --kd 12          # same gains, all ids
    python tune_walk.py --port COM7 --ids 2,3,5 --secs 8 --hz 50 --vx 120

Per-servo gains (override the global --kp/--ki/--kd for that id):
    python tune_walk.py --port COM7 --ids 2,3 \
        --set 2:kp=40,kd=12 --set 3:kp=55,kd=18
    # mix: global default for everyone, then tweak one id
    python tune_walk.py --port COM7 --ids 2,3 --kp 40 --kd 12 --set 3:kd=20

--set takes  id:key=val[,key=val...]  where key is kp, ki or kd. Repeat --set
once per servo. It's applied on top of the global --kp/--ki/--kd.

If you omit --port it lists the ports it can see so you can pick one.
Each run writes walk.log and walk_servo<ids>.png (overwritten).
"""

import argparse
import os
import subprocess
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not installed. Run:  pip install pyserial")


def pick_port(port):
    ports = list(list_ports.comports())
    if port:
        return port
    if not ports:
        sys.exit("No serial ports found. Is the robot plugged in?")
    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device}  ({p.description})")
    sys.exit("Re-run with --port <one of the above>, e.g. --port "
             + ports[0].device)


def parse_ids(text):
    ids = []
    for chunk in text.replace(",", " ").split():
        try:
            v = int(chunk)
        except ValueError:
            sys.exit(f"bad servo id: {chunk!r}")
        if not (1 <= v <= 12):
            sys.exit(f"servo id out of range 1-12: {v}")
        ids.append(v)
    if not ids:
        sys.exit("no servo ids given")
    return ids


KEY_MAP = {"kp": "kp_position", "ki": "ki_position", "kd": "kd_position"}


def parse_set(text):
    """Parse one --set value 'id:key=val,key=val' -> (id, {param_name: val})."""
    if ":" not in text:
        sys.exit(f"bad --set {text!r} (expected id:key=val, e.g. 2:kp=40,kd=12)")
    sid_str, rest = text.split(":", 1)
    try:
        sid = int(sid_str)
    except ValueError:
        sys.exit(f"bad servo id in --set: {sid_str!r}")
    gains = {}
    for pair in rest.replace(" ", "").split(","):
        if not pair:
            continue
        if "=" not in pair:
            sys.exit(f"bad gain {pair!r} in --set (expected key=val)")
        key, val = pair.split("=", 1)
        key = key.lower()
        if key not in KEY_MAP:
            sys.exit(f"unknown gain {key!r} in --set (use kp, ki or kd)")
        try:
            gains[KEY_MAP[key]] = float(val)
        except ValueError:
            sys.exit(f"bad value {val!r} for {key} in --set")
    return sid, gains


def send(ser, line, wait=0.3):
    ser.write((line + "\r\n").encode())
    ser.flush()
    time.sleep(wait)


def main():
    ap = argparse.ArgumentParser(
        description="One-shot Stanford-walk PID trace over serial")
    ap.add_argument("--port", help="serial port, e.g. COM7 (lists ports if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ids", default="2,3",
                    help="servo ids to trace, comma/space separated (default 2,3)")
    ap.add_argument("--secs", type=int, default=6, help="walk duration, seconds")
    ap.add_argument("--hz", type=int, default=50, help="log sample rate")
    ap.add_argument("--vx", type=float, default=100.0,
                    help="forward walk speed mm/s (max ~200)")
    ap.add_argument("--kp", type=float, help="global kp_position for every id")
    ap.add_argument("--ki", type=float, help="global ki_position for every id")
    ap.add_argument("--kd", type=float, help="global kd_position for every id")
    ap.add_argument("--set", action="append", default=[], dest="sets",
                    metavar="ID:key=val,...",
                    help="per-servo gains, e.g. --set 2:kp=40,kd=12 (repeatable)")
    ap.add_argument("--timeout", type=float, default=0.0,
                    help="max seconds to wait for #WALK_END (0 = secs + 8)")
    args = ap.parse_args()

    ids = parse_ids(args.ids)
    timeout = args.timeout if args.timeout > 0 else args.secs + 8

    # build per-servo gains: start from the global --kp/--ki/--kd, then let
    # each --set override for its own id.
    gains = {sid: {} for sid in ids}
    for name, val in (("kp_position", args.kp), ("ki_position", args.ki),
                      ("kd_position", args.kd)):
        if val is not None:
            for sid in ids:
                gains[sid][name] = val
    for entry in args.sets:
        sid, g = parse_set(entry)
        if sid not in gains:
            print(f"  note: --set id {sid} is not in --ids {ids}; adding it")
            gains[sid] = {}
            ids.append(sid)
        gains[sid].update(g)

    port = pick_port(args.port)
    print(f"Opening {port} @ {args.baud} ...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n"
                 "Is idf.py monitor still open? Close it first.")

    time.sleep(0.5)
    ser.reset_input_buffer()

    # 1) enable CLI mode (pauses the gait task so this run owns the SPI bus)
    send(ser, "cli on", 0.4)

    # 2) optional gains, per servo (globals already folded in above)
    for sid in ids:
        for name, val in gains[sid].items():
            send(ser, f"{sid} set {name} {val:g}", 0.2)
            print(f"  servo {sid}: {name} = {val:g}")

    # 3) run the walk trace and capture everything until #WALK_END
    cmd = f"swalk {args.secs} {args.hz} {args.vx:g} " + " ".join(str(i) for i in ids)
    print(f"  {cmd}")
    ser.reset_input_buffer()
    send(ser, cmd, 0.0)

    lines = []
    started = False
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw = ser.readline().decode(errors="replace").rstrip("\r\n")
        if not raw:
            continue
        if "#WALK_BEGIN" in raw:
            started = True
        if started:
            lines.append(raw)
        if "#WALK_END" in raw:
            break
    else:
        print("Timed out waiting for #WALK_END. Captured what I could.")

    # resume the gait so the robot isn't stuck in CLI mode
    send(ser, "cli off", 0.2)
    ser.close()

    if not any("#WALK_BEGIN" in l for l in lines):
        sys.exit("No walk data captured. Check the port and that the firmware "
                 "has the 'swalk' command flashed.")

    log = "walk.log"
    with open(log, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  captured {len(lines)} lines -> {log}")

    # 4) plot with the walk plotter (same folder as this script)
    plotter = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "plot_walk.py")
    subprocess.run([sys.executable, plotter, log], check=False)


if __name__ == "__main__":
    main()
