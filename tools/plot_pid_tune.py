"""
Plot a PID step-response captured from `idf.py monitor`.

The firmware has a serial command `sweep` (see main.c) that steps one servo
low<->high for N cycles and streams CSV over the console, framed like:

    #SWEEP_BEGIN id=2 low=100.0 high=200.0 hold_ms=800 cycles=3 hz=50
    t_ms,cmd_deg,set_deg,now_deg,err_deg,cur_mA,duty_pct
    0,200.0,100.4,101.2,-98.8,95,42.5
    20,200.0,180.1,178.9,-21.1,410,88.0
    ...
    #SWEEP_END

This script reads the monitor log, grabs the last sweep block, and plots the
commanded angle against the actual (measured) angle so you can judge
overshoot / rise / settling and pick the best kp/ki/kd.

How to capture the log, then plot:

    # 1) log the monitor session to a file
    idf.py monitor | tee sweep.log
    #    at the "> " prompt type:   cli on   <enter>
    #                               sweep    <enter>      (or: sweep 2 100 200 800 3 50)
    #    wait for #SWEEP_END, then Ctrl-] to quit the monitor

    # 2) plot it
    pip install matplotlib
    python plot_pid_tune.py sweep.log

Writes sweep_<id>_<timestamp>.png next to the log and prints the metrics.
Pass '-' as the filename to read from stdin.
"""

import os
import re
import sys
import time

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLOT = True
except ImportError:
    HAVE_PLOT = False

ANSI = re.compile(r"\x1b\[[0-9;]*m")            # color codes from the monitor
# a data row: 7 comma-separated fields, some (set/err/duty) may be blank
ROW = re.compile(r"^\s*(\d+),([-\d.]+),([-\d.]*),([-\d.]+),([-\d.]*),"
                 r"(-?\d*),([-\d.]*)\s*$")
HDR = re.compile(r"#SWEEP_BEGIN\s+id=(\d+)\s+low=([-\d.]+)\s+high=([-\d.]+)")


def clean(line):
    return ANSI.sub("", line).replace("\r", "").rstrip("\n")


def read_lines(path):
    if path == "-":
        return [clean(l) for l in sys.stdin]
    # Windows Explorer hides .txt; auto-add it if that's the real file
    if not os.path.exists(path) and os.path.exists(path + ".txt"):
        path = path + ".txt"
    if not os.path.exists(path):
        here = os.getcwd()
        cands = [f for f in os.listdir(".")
                 if f.lower().endswith((".log", ".txt")) or f.lower().startswith("log")]
        msg = f"File not found: {path}\n(looking in {here})"
        if cands:
            msg += "\nLog-like files I can see here: " + ", ".join(cands)
        else:
            msg += ("\nNo .log/.txt files here yet. Capture one first: in "
                    "`idf.py monitor` press Ctrl-T then Ctrl-L to start logging "
                    "to a file, run 'cli on' then 'sweep', wait for #SWEEP_END, "
                    "then pass that filename to this script.")
        raise SystemExit(msg)
    raw = open(path, "rb").read()
    # PowerShell redirection / Tee-Object writes UTF-16; the monitor logs UTF-8
    if raw[:2] in (b"\xff\xfe", b"\xfe\xff"):
        text = raw.decode("utf-16", errors="replace")
    else:
        text = raw.decode("utf-8", errors="replace")
    return [clean(l) for l in text.splitlines()]


def extract_last_block(lines):
    """Return (meta, rows). Uses the last #SWEEP_BEGIN..#SWEEP_END pair; if no
    markers are present, parses every data row it can find."""
    begins = [i for i, l in enumerate(lines) if "#SWEEP_BEGIN" in l]
    ends = [i for i, l in enumerate(lines) if "#SWEEP_END" in l]
    meta = {"id": "?", "low": None, "high": None}
    if begins:
        b = begins[-1]
        e = next((x for x in ends if x > b), len(lines))
        m = HDR.search(lines[b])
        if m:
            meta = {"id": m.group(1),
                    "low": float(m.group(2)), "high": float(m.group(3))}
        chunk = lines[b:e]
    else:
        chunk = lines

    rows = []
    for l in chunk:
        m = ROW.match(l)
        if not m:
            continue
        t, cmd, sset, now, err, cur, duty = m.groups()
        rows.append({
            "t": int(t) / 1000.0,
            "cmd": float(cmd),
            "set": float(sset) if sset else None,
            "now": float(now),
            "err": float(err) if err else None,
            "cur": float(cur) if cur else None,
            "duty": float(duty) if duty else None,
        })
    return meta, rows


def metrics(rows, low, high):
    """Overshoot / rise / settle for the first commanded low->high step."""
    if low is None or high is None or high == low:
        # infer from data: first row where cmd jumps up
        highs = sorted({r["cmd"] for r in rows})
        if len(highs) < 2:
            return None
        low, high = highs[0], highs[-1]
    # only the FIRST contiguous low->high step (not every up-cycle)
    seg = []
    started = False
    for r in rows:
        if r["cmd"] == high:
            started = True
            seg.append(r)
        elif started:
            break
    if not seg:
        return None
    t0 = seg[0]["t"]
    ts = [r["t"] - t0 for r in seg]
    ys = [r["now"] for r in seg]
    span = high - low
    peak = max(ys)
    overshoot = max(0.0, (peak - high) / span * 100.0)

    def cross(frac):
        tgt = low + span * frac
        for t, y in zip(ts, ys):
            if y >= tgt:
                return t
        return None

    t10, t90 = cross(0.10), cross(0.90)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
    band = abs(span) * 0.02
    settle = None
    for t, y in zip(ts, ys):
        if abs(y - high) > band:
            settle = t
    return {"low": low, "high": high, "overshoot": overshoot, "peak": peak,
            "rise": rise, "settle": settle, "ss_err": ys[-1] - high}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("usage: python plot_pid_tune.py <monitor.log>  (or '-' for stdin)")
        sys.exit(1)
    path = sys.argv[1]

    meta, rows = extract_last_block(read_lines(path))
    if not rows:
        print("No sweep data found. Did you run 'cli on' then 'sweep' in the "
              "monitor and capture the output?")
        sys.exit(1)
    print(f"Parsed {len(rows)} samples (servo {meta['id']}).")

    mt = metrics(rows, meta["low"], meta["high"])
    if mt:
        print(f"\nFirst {mt['low']:.0f}->{mt['high']:.0f} deg step:")
        print(f"  overshoot   : {mt['overshoot']:.1f} %  (peak {mt['peak']:.1f} deg)")
        print("  rise 10-90% : "
              + ("n/a" if mt["rise"] is None else f"{mt['rise'] * 1000:.0f} ms"))
        print("  settle ±2%  : "
              + ("n/a" if mt["settle"] is None else f"{mt['settle'] * 1000:.0f} ms"))
        print(f"  steady err  : {mt['ss_err']:+.2f} deg")

    if not HAVE_PLOT:
        print("\n(matplotlib not installed -> no PNG. pip install matplotlib)")
        return

    import math
    ts   = [r["t"] for r in rows]
    cmd  = [r["cmd"] for r in rows]
    now  = [r["now"] for r in rows]
    err  = [r["err"] if r["err"] is not None else r["cmd"] - r["now"] for r in rows]
    duty = [r["duty"] for r in rows]
    cur  = [r["cur"] for r in rows]
    has_duty = any(d is not None for d in duty)
    has_cur  = any(c is not None for c in cur)
    nan = float("nan")
    duty_p = [d if d is not None else nan for d in duty]
    cur_p  = [c if c is not None else nan for c in cur]

    n = 2 + (1 if (has_duty or has_cur) else 0)
    fig, axes = plt.subplots(n, 1, sharex=True, figsize=(11, 3.0 * n),
                             constrained_layout=True)
    ax_sp, ax_err = axes[0], axes[1]

    # panel 1 - setpoint vs actual
    ax_sp.plot(ts, cmd, drawstyle="steps-post", color="#888",
               linewidth=1.4, label="commanded (setpoint)")
    ax_sp.plot(ts, now, color="#1f77b4", linewidth=1.7, label="actual (trace)")
    ax_sp.set_ylabel("angle (deg)")
    ax_sp.set_title(f"Servo {meta['id']} step response (from idf.py monitor)")
    ax_sp.grid(True, alpha=0.3)
    ax_sp.legend(loc="best")
    if mt:
        rise = "n/a" if mt["rise"] is None else f"{mt['rise'] * 1000:.0f} ms"
        settle = "n/a" if mt["settle"] is None else f"{mt['settle'] * 1000:.0f} ms"
        ax_sp.text(0.012, 0.05,
                   f"overshoot: {mt['overshoot']:.1f} %\nrise 10-90%: {rise}\n"
                   f"settle (+/-2%): {settle}\nsteady err: {mt['ss_err']:+.2f} deg",
                   transform=ax_sp.transAxes, va="bottom", fontsize=9,
                   bbox=dict(boxstyle="round", fc="#fffbe6", ec="#ccc"))

    # panel 2 - error
    ax_err.plot(ts, err, color="tab:red", linewidth=1.2, label="error (deg)")
    ax_err.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    ax_err.set_ylabel("error (deg)")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="best")

    # panel 3 - PWM duty (+ current on a second axis)
    if n == 3:
        ax_pwm = axes[2]
        if has_duty:
            ax_pwm.plot(ts, duty_p, color="tab:green", linewidth=1.2,
                        label="PWM duty (%)")
            ax_pwm.set_ylabel("PWM duty (%)", color="tab:green")
            ax_pwm.tick_params(axis="y", labelcolor="tab:green")
        ax_pwm.axhline(0.0, color="black", linewidth=0.8, alpha=0.4)
        ax_pwm.grid(True, alpha=0.3)
        if has_cur:
            ax_c = ax_pwm.twinx()
            ax_c.plot(ts, cur_p, color="#d62728", alpha=0.5, linewidth=1.0,
                      label="current (mA)")
            ax_c.set_ylabel("current (mA)", color="#d62728")
            ax_c.tick_params(axis="y", labelcolor="#d62728")
        ax_pwm.legend(loc="upper left")

    axes[-1].set_xlabel("time (s)")

    # RMSE / peak error over the whole capture (matches the reference metrics)
    ev = [e for e in err if e is not None]
    rmse = math.sqrt(sum(e * e for e in ev) / len(ev)) if ev else 0.0
    peak = max((abs(e) for e in ev), default=0.0)
    fig.text(0.99, 0.004, f"RMSE={rmse:.3g} deg   |peak error|={peak:.3g} deg",
             ha="right", va="bottom", fontsize=9, alpha=0.85)

    # name the PNG after the log file and overwrite it each run, so you can
    # keep one image open and it refreshes in place (no timestamp pile-up).
    base = os.path.splitext(path)[0]
    if base.lower().endswith(".log"):
        base = base[:-4]
    out = base + f"_servo{meta['id']}.png"
    fig.savefig(out, dpi=130)
    print(f"\nsaved {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
