"""
Plot a Stanford-walk PID trace captured from the `swalk` serial command.

The firmware command `swalk` (see main.c) runs the Stanford trot gait and
streams one CSV row per sample, with a set/now/err/cur/duty column group per
servo, framed like:

    #WALK_BEGIN secs=6 hz=50 vx=100.0 ids=2-3
    t_ms,set2,now2,err2,cur2,duty2,set3,now3,err3,cur3,duty3
    0,101.2,101.0,0.2,95,42.5,88.1,87.9,0.2,110,40.0
    ...
    #WALK_END

This is the walk-time sibling of plot_pid_tune.py. For each servo it plots the
commanded (setpoint) angle against the actual (measured) angle so you can judge
how well the PID tracks the real gait, plus the error and PWM/current, and
prints tracking metrics (RMSE / peak error / mean current) per servo.

Usually you don't call this directly - tune_walk.py runs it for you. But you
can plot a saved log too:

    pip install matplotlib
    python plot_walk.py walk.log

Writes walk_servo<ids>.png next to the log. Pass '-' to read from stdin.
"""

import math
import os
import re
import sys

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLOT = True
except ImportError:
    HAVE_PLOT = False

ANSI = re.compile(r"\x1b\[[0-9;]*m")            # color codes from the monitor
HDR = re.compile(r"#WALK_BEGIN\s+secs=(\d+)\s+hz=(\d+)\s+vx=([-\d.]+)\s+ids=([\d\-]+)")


def clean(line):
    return ANSI.sub("", line).replace("\r", "").rstrip("\n")


def read_lines(path):
    if path == "-":
        return [clean(l) for l in sys.stdin]
    if not os.path.exists(path) and os.path.exists(path + ".txt"):
        path = path + ".txt"
    if not os.path.exists(path):
        here = os.getcwd()
        cands = [f for f in os.listdir(".")
                 if f.lower().endswith((".log", ".txt"))]
        msg = f"File not found: {path}\n(looking in {here})"
        if cands:
            msg += "\nLog-like files I can see here: " + ", ".join(cands)
        raise SystemExit(msg)
    raw = open(path, "rb").read()
    # PowerShell redirection / Tee-Object writes UTF-16; the monitor logs UTF-8
    if raw[:2] in (b"\xff\xfe", b"\xfe\xff"):
        text = raw.decode("utf-16", errors="replace")
    else:
        text = raw.decode("utf-8", errors="replace")
    return [clean(l) for l in text.splitlines()]


def num(s):
    s = s.strip()
    if s == "":
        return None
    try:
        return float(s)
    except ValueError:
        return None


def extract_last_block(lines):
    """Return (meta, header_cols, rows). Uses the last #WALK_BEGIN..#WALK_END
    pair; if markers are missing, parses everything after the header row."""
    begins = [i for i, l in enumerate(lines) if "#WALK_BEGIN" in l]
    ends = [i for i, l in enumerate(lines) if "#WALK_END" in l]
    meta = {"secs": None, "hz": None, "vx": None, "ids": None}

    if begins:
        b = begins[-1]
        e = next((x for x in ends if x > b), len(lines))
        m = HDR.search(lines[b])
        if m:
            meta = {"secs": int(m.group(1)), "hz": int(m.group(2)),
                    "vx": float(m.group(3)),
                    "ids": [int(x) for x in m.group(4).split("-")]}
        chunk = lines[b + 1:e]
    else:
        chunk = lines

    # find the CSV header row (starts with t_ms)
    header = None
    data_start = 0
    for i, l in enumerate(chunk):
        if l.strip().startswith("t_ms"):
            header = [c.strip() for c in l.split(",")]
            data_start = i + 1
            break
    if header is None:
        return meta, None, []

    rows = []
    for l in chunk[data_start:]:
        if not l or l.startswith("#") or "," not in l:
            continue
        parts = l.split(",")
        if not parts[0].strip().isdigit():
            continue
        # pad/truncate to header width
        if len(parts) < len(header):
            parts += [""] * (len(header) - len(parts))
        rows.append(parts[:len(header)])
    return meta, header, rows


def servo_series(header, rows, sid):
    """Pull the t + set/now/err/cur/duty series for one servo id."""
    def col(name):
        return header.index(name) if name in header else None

    ci = {k: col(f"{k}{sid}") for k in ("set", "now", "err", "cur", "duty")}
    t, sset, now, err, cur, duty = [], [], [], [], [], []
    for r in rows:
        tv = num(r[0])
        if tv is None:
            continue
        t.append(tv / 1000.0)
        sset.append(num(r[ci["set"]]) if ci["set"] is not None else None)
        now.append(num(r[ci["now"]]) if ci["now"] is not None else None)
        err.append(num(r[ci["err"]]) if ci["err"] is not None else None)
        cur.append(num(r[ci["cur"]]) if ci["cur"] is not None else None)
        duty.append(num(r[ci["duty"]]) if ci["duty"] is not None else None)
    return {"t": t, "set": sset, "now": now, "err": err,
            "cur": cur, "duty": duty}


def metrics(s):
    """Tracking metrics for a servo over the whole walk capture."""
    # prefer the firmware-reported error; else derive setpoint - actual
    ev = [e for e in s["err"] if e is not None]
    if not ev:
        ev = [(sp - nw) for sp, nw in zip(s["set"], s["now"])
              if sp is not None and nw is not None]
    rmse = math.sqrt(sum(e * e for e in ev) / len(ev)) if ev else 0.0
    peak = max((abs(e) for e in ev), default=0.0)
    cv = [c for c in s["cur"] if c is not None]
    mean_cur = sum(cv) / len(cv) if cv else 0.0
    return {"rmse": rmse, "peak": peak, "mean_cur": mean_cur}


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("usage: python plot_walk.py <walk.log>  (or '-' for stdin)")
        sys.exit(1)
    path = sys.argv[1]

    meta, header, rows = extract_last_block(read_lines(path))
    if not rows or header is None:
        print("No walk data found. Did you run 'cli on' then 'swalk' and "
              "capture the output?")
        sys.exit(1)

    # servo ids: from the header meta if present, else infer from columns
    if meta["ids"]:
        ids = meta["ids"]
    else:
        ids = sorted({int(m.group(1))
                      for c in header
                      for m in [re.match(r"set(\d+)", c)] if m})
    if not ids:
        print("Could not identify any servo columns (setN/nowN...).")
        sys.exit(1)

    print(f"Parsed {len(rows)} samples for servos {ids}"
          + (f" (vx={meta['vx']:g} mm/s)" if meta["vx"] is not None else ""))

    series = {sid: servo_series(header, rows, sid) for sid in ids}
    mets = {sid: metrics(series[sid]) for sid in ids}

    for sid in ids:
        m = mets[sid]
        print(f"\nServo {sid}:")
        print(f"  tracking RMSE : {m['rmse']:.2f} deg")
        print(f"  peak |error|  : {m['peak']:.2f} deg")
        print(f"  mean current  : {m['mean_cur']:.0f} mA")

    if not HAVE_PLOT:
        print("\n(matplotlib not installed -> no PNG. pip install matplotlib)")
        return

    nan = float("nan")
    ncol = len(ids)
    # 3 rows: setpoint-vs-actual, error, duty(+current). One column per servo.
    fig, axes = plt.subplots(3, ncol, sharex=True,
                             figsize=(6.0 * ncol, 8.5),
                             squeeze=False, constrained_layout=True)

    for j, sid in enumerate(ids):
        s = series[sid]
        ts = s["t"]
        setp = [v if v is not None else nan for v in s["set"]]
        now = [v if v is not None else nan for v in s["now"]]
        err = [v if v is not None else
               ((sp - nw) if (sp is not None and nw is not None) else nan)
               for v, sp, nw in zip(s["err"], s["set"], s["now"])]
        duty = [v if v is not None else nan for v in s["duty"]]
        cur = [v if v is not None else nan for v in s["cur"]]
        m = mets[sid]

        # row 0 - setpoint vs actual
        ax = axes[0][j]
        ax.plot(ts, setp, color="#888", linewidth=1.4, label="commanded (setpoint)")
        ax.plot(ts, now, color="#1f77b4", linewidth=1.6, label="actual (trace)")
        ax.set_title(f"Servo {sid} - walk step response")
        if j == 0:
            ax.set_ylabel("angle (deg)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        ax.text(0.012, 0.04,
                f"RMSE: {m['rmse']:.2f} deg\npeak |err|: {m['peak']:.2f} deg\n"
                f"mean cur: {m['mean_cur']:.0f} mA",
                transform=ax.transAxes, va="bottom", fontsize=8,
                bbox=dict(boxstyle="round", fc="#fffbe6", ec="#ccc"))

        # row 1 - error
        ax = axes[1][j]
        ax.plot(ts, err, color="tab:red", linewidth=1.1, label="error (deg)")
        ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        if j == 0:
            ax.set_ylabel("error (deg)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)

        # row 2 - PWM duty (+ current on a twin axis)
        ax = axes[2][j]
        if any(not math.isnan(d) for d in duty):
            ax.plot(ts, duty, color="tab:green", linewidth=1.1, label="PWM duty (%)")
            if j == 0:
                ax.set_ylabel("PWM duty (%)", color="tab:green")
            ax.tick_params(axis="y", labelcolor="tab:green")
        ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.4)
        ax.grid(True, alpha=0.3)
        if any(not math.isnan(c) for c in cur):
            axc = ax.twinx()
            axc.plot(ts, cur, color="#d62728", alpha=0.5, linewidth=1.0,
                     label="current (mA)")
            if j == ncol - 1:
                axc.set_ylabel("current (mA)", color="#d62728")
            axc.tick_params(axis="y", labelcolor="#d62728")
        ax.set_xlabel("time (s)")

    ttl = "Stanford walk PID trace"
    if meta["vx"] is not None:
        ttl += f"  (vx={meta['vx']:g} mm/s)"
    fig.suptitle(ttl, fontsize=13)

    base = os.path.splitext(path)[0]
    if base.lower().endswith(".log"):
        base = base[:-4]
    out = base + "_servo" + "-".join(str(i) for i in ids) + ".png"
    fig.savefig(out, dpi=130)
    print(f"\nsaved {os.path.abspath(out)}")


if __name__ == "__main__":
    main()
