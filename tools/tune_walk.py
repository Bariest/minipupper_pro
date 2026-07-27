"""
One-command PID step test for a Mini Pupper servo.

Talks to the ESP32 directly over the USB serial (COM) port: enables CLI mode,
optionally sets gains, runs `sweep`, captures the CSV, saves it, and calls
plot_pid_tune.py to make the PNG. No idf.py monitor, no Ctrl-T/Ctrl-L, no
copy-paste.

IMPORTANT: close `idf.py monitor` first - only one program can use the COM
port at a time.

Setup (once):
    pip install pyserial matplotlib

Usage:
    python tune_servo.py --port COM7
    python tune_servo.py --port COM7 --kp 40 --kd 12
    python tune_servo.py --port COM7 --id 2 --low 100 --high 200 --cycles 3

If you omit --port it lists the ports it can see so you can pick one.
Each run writes tune_<id>.log and tune_<id>_servo<id>.png (overwritten).
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


def send(ser, line, wait=0.3):
    ser.write((line + "\r\n").encode())
    ser.flush()
    time.sleep(wait)


def main():
    ap = argparse.ArgumentParser(description="One-shot servo PID step test")
    ap.add_argument("--port", help="serial port, e.g. COM7 (lists ports if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--id", type=int, default=2)
    ap.add_argument("--low", type=float, default=100)
    ap.add_argument("--high", type=float, default=200)
    ap.add_argument("--hold", type=int, default=800)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--hz", type=int, default=50)
    ap.add_argument("--kp", type=float, help="set kp_position before the test")
    ap.add_argument("--ki", type=float, help="set ki_position before the test")
    ap.add_argument("--kd", type=float, help="set kd_position before the test")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="max seconds to wait for #SWEEP_END")
    args = ap.parse_args()

    port = pick_port(args.port)
    print(f"Opening {port} @ {args.baud} ...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n"
                 "Is idf.py monitor still open? Close it first.")

    time.sleep(0.5)
    ser.reset_input_buffer()

    # 1) enable CLI mode (pauses gait so it doesn't fight the SPI bus)
    send(ser, "cli on", 0.4)

    # 2) optional gains
    for name, val in (("kp_position", args.kp),
                      ("ki_position", args.ki),
                      ("kd_position", args.kd)):
        if val is not None:
            send(ser, f"{args.id} set {name} {val}", 0.3)
            print(f"  set {name} = {val}")

    # 3) run the sweep and capture everything until #SWEEP_END
    cmd = f"sweep {args.id} {args.low} {args.high} {args.hold} {args.cycles} {args.hz}"
    print(f"  {cmd}")
    ser.reset_input_buffer()
    send(ser, cmd, 0.0)

    lines = []
    started = False
    t0 = time.time()
    while time.time() - t0 < args.timeout:
        raw = ser.readline().decode(errors="replace").rstrip("\r\n")
        if not raw:
            continue
        if "#SWEEP_BEGIN" in raw:
            started = True
        if started:
            lines.append(raw)
        if "#SWEEP_END" in raw:
            break
    else:
        print("Timed out waiting for #SWEEP_END. Captured what I could.")

    ser.close()

    if not any("#SWEEP_BEGIN" in l for l in lines):
        sys.exit("No sweep data captured. Check the port and that the firmware "
                 "has the 'sweep' command flashed.")

    log = f"tune_{args.id}.log"
    with open(log, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  captured {len(lines)} lines -> {log}")

    # 4) plot with the existing plotter (same folder as this script)
    plotter = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "plot_pid_tune.py")
    subprocess.run([sys.executable, plotter, log], check=False)


if __name__ == "__main__":
    main()
