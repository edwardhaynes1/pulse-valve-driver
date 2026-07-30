"""
PULSE-VALVE-LOG-PLOTTER  —  Overlay plot of a PULSE-VALVE-DRIVER session
=============================================================
Plots, on a shared time axis, for a single pulsing session:

  • Upstream pressure   (Keller, bar)          — top panel
  • Upstream temperature (Keller, °C)          — overlaid on top panel (right axis)
  • Vacuum chamber pressure (mbar, log scale)  — bottom panel
  • Valve-fire events                          — vertical markers on both panels

By default the whole log file is plotted. An optional --open-time filter can
restrict the plot to rows recorded at one valve open-time setting.

Usage
-----
  python PULSE-VALVE-LOG-PLOTTER.py [csv_file] [--open-time N] [--out plot.png]

  --open-time N   OPTIONAL: only plot rows where open_time_us == N
                  (default: plot the whole file)
  --out FILE      Save the figure to FILE (default: show interactively)
  --zoom          Crop the time axis to a margin around the pulses
  --margin S      Seconds of margin either side of pulses when --zoom is used

If no CSV file is given on the command line, a file-picker dialog opens so you
can choose which log to plot. If the dialog can't open (e.g. no display), a
numbered list of pvd-sensor_*.csv files in the current folder is offered
instead.
"""

import sys
import os
import glob
import argparse
import csv
from datetime import datetime

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.dates as mdates


def parse_ts(s):
    return datetime.fromisoformat(s)


def load(csv_path):
    with open(csv_path, newline="") as f:
        return list(csv.DictReader(f))


def to_float(s):
    try:
        return float(s)
    except (ValueError, TypeError):
        return None



def choose_csv_file():
    """Return a CSV path chosen by the user.

    Tries a graphical file-open dialog first (Tkinter). If that isn't
    available, falls back to a numbered list of pvd-sensor_*.csv files
    found in the current working directory.
    """
    # ── try a graphical dialog ─────────────────────────────────────────────
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        path = filedialog.askopenfilename(
            title="Select a pulse-valve log to plot",
            filetypes=[("PVD sensor logs", "pvd-sensor_*.csv"),
                       ("CSV files", "*.csv"),
                       ("All files", "*.*")],
        )
        root.destroy()
        if path:
            return path
        print("No file selected.")
        sys.exit(0)
    except Exception:
        pass  # fall through to text-mode picker

    # ── text-mode fallback ─────────────────────────────────────────────────
    candidates = sorted(glob.glob("pvd-sensor_*.csv")) or sorted(glob.glob("*.csv"))
    if not candidates:
        print("No CSV files found in the current folder.")
        print("Pass a file explicitly:  python PULSE-VALVE-LOG-PLOTTER.py <file.csv>")
        sys.exit(1)

    print("\nSelect a log file to plot:")
    for i, name in enumerate(candidates, 1):
        print(f"  {i:>2}. {name}")
    while True:
        choice = input(f"\nEnter number (1-{len(candidates)}), or q to quit: ").strip()
        if choice.lower() == "q":
            sys.exit(0)
        try:
            idx = int(choice)
            if 1 <= idx <= len(candidates):
                return candidates[idx - 1]
        except ValueError:
            pass
        print("  invalid choice")


def session_identity_str(rows):
    """One-line limiter + gas caption for a session, from the run-identity
    columns. Shows the actual setting; if it changed mid-session (the driver
    allows live cap/gas changes) it lists the states in the order they appeared,
    joined by →, instead of a bare 'mixed'. Returns '' when the columns are
    absent (logs predating the stamp) so nothing misleading is shown."""
    if not rows or "capillary_id_um" not in rows[0]:
        return ""

    def state(r):
        idd = (r.get("capillary_id_um") or "").strip()
        ln  = (r.get("capillary_length_mm") or "").strip()
        lab = (r.get("capillary_label") or "").strip()
        gas = (r.get("gas_species") or "").strip()
        if idd:
            cap = f"{idd} µm × {ln} mm" if ln else f"{idd} µm"
            if lab:
                cap += f" ({lab})"
        else:
            cap = "no limiter"
        return cap, (gas if gas else "gas n/a")

    # distinct (cap, gas) states in order of first appearance
    seen, order = set(), []
    for r in rows:
        cap, gas = state(r)
        if cap == "no limiter" and gas == "gas n/a":
            continue                       # wholly blank row — ignore
        key = (cap, gas)
        if key not in seen:
            seen.add(key)
            order.append(key)

    if not order:
        return ""
    if len(order) == 1:
        cap, gas = order[0]
        return f"limiter {cap}    ·    gas {gas}"
    if len(order) <= 3:
        return "limiter/gas changed:    " + "    →    ".join(
            f"{cap} · {gas}" for cap, gas in order)
    return (f"limiter/gas: {len(order)} distinct settings this session "
            f"— see per-row columns")


def main():
    ap = argparse.ArgumentParser(description="Overlay plot of a PVD session")
    ap.add_argument("csv", nargs="?", default=None,
                    help="pvd-sensor_*.csv file (if omitted, a picker opens)")
    ap.add_argument("--open-time", type=int, default=None,
                    help="OPTIONAL: only plot rows where open_time_us == N "
                         "(default: plot the whole file)")
    ap.add_argument("--out", default=None, help="save to file instead of showing")
    ap.add_argument("--zoom", action="store_true",
                    help="crop the time axis to a margin around the first/last "
                         "pulse, so idle time before/after is trimmed")
    ap.add_argument("--margin", type=float, default=30.0,
                    help="seconds of margin either side of pulses when --zoom "
                         "is used (default 30)")
    args = ap.parse_args()

    csv_path = args.csv if args.csv else choose_csv_file()
    print(f"Loading: {csv_path}")
    rows = load(csv_path)

    # ── Optional open-time filter (off by default: plot the whole file) ────
    if args.open_time is not None:
        rows = [r for r in rows if r.get("open_time_us") == str(args.open_time)]
        if not rows:
            print(f"No rows found with open_time_us == {args.open_time}.")
            sys.exit(1)
        section = f"open time = {args.open_time} µs"
    else:
        section = "full session"

    # ── Extract series ─────────────────────────────────────────────────────
    t         = [parse_ts(r["timestamp"]) for r in rows]
    up_p      = [to_float(r["keller_pressure_bar"]) for r in rows]
    up_t      = [to_float(r["keller_temperature_degC"]) for r in rows]
    vac       = [to_float(r["vacuum_chamber_mbar"]) for r in rows]

    # ── Extract pulse events (may be multiple per row, semicolon-sep) ──────
    pulse_times = []
    for r in rows:
        raw = r.get("pulse_timestamps_us", "")
        if raw:
            for ts in raw.split(";"):
                ts = ts.strip()
                if ts:
                    try:
                        pulse_times.append(parse_ts(ts))
                    except ValueError:
                        pass

    t0, t1 = t[0], t[-1]
    print(f"Plotting {section}: {len(rows)} rows, "
          f"{t0.strftime('%H:%M:%S')} → {t1.strftime('%H:%M:%S')}, "
          f"{len(pulse_times)} pulses")

    # ── Figure: two stacked panels sharing the x-axis ──────────────────────
    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(13, 8), sharex=True,
        gridspec_kw={"height_ratios": [1, 1], "hspace": 0.12},
    )
    _ident = session_identity_str(rows)
    _sub = f"\n{_ident}" if _ident else ""
    fig.suptitle(f"Pulse valve session — {section}{_sub}", fontsize=13, y=0.97)

    # Colours
    C_P   = "#1f5fd0"   # upstream pressure  (blue)
    C_T   = "#d9772b"   # upstream temp      (orange)
    C_VAC = "#111111"   # vacuum             (near-black)
    C_PLS = "#c0303090" # pulse markers      (translucent red)

    # ── TOP PANEL: upstream pressure (left) + temperature (right) ──────────
    ax_top.plot(t, up_p, color=C_P, lw=1.2, label="Upstream pressure (bar)")
    ax_top.set_ylabel("Upstream pressure (bar)", color=C_P)
    ax_top.tick_params(axis="y", labelcolor=C_P)
    ax_top.grid(True, alpha=0.25)

    ax_temp = ax_top.twinx()
    ax_temp.plot(t, up_t, color=C_T, lw=1.0, alpha=0.9,
                 label="Upstream temperature (°C)")
    ax_temp.set_ylabel("Upstream temperature (°C)", color=C_T)
    ax_temp.tick_params(axis="y", labelcolor=C_T)

    # ── BOTTOM PANEL: vacuum pressure (log) ────────────────────────────────
    ax_bot.plot(t, vac, color=C_VAC, lw=1.2, label="Vacuum chamber (mbar)")
    ax_bot.set_yscale("log")
    ax_bot.set_ylabel("Vacuum chamber (mbar)")
    ax_bot.grid(True, which="both", alpha=0.25)

    # ── Valve-fire markers on both panels ──────────────────────────────────
    # Drawn unlabeled; a single proxy handle is added to each legend below, so
    # the marker shows once per panel rather than once per line it crosses.
    for pt in pulse_times:
        for ax in (ax_top, ax_bot):
            ax.axvline(pt, color=C_PLS, lw=1.4)

    # ── Optional zoom: crop x-axis to a margin around the pulses ───────────
    if args.zoom and pulse_times:
        from datetime import timedelta
        lo = min(pulse_times) - timedelta(seconds=args.margin)
        hi = max(pulse_times) + timedelta(seconds=args.margin)
        ax_bot.set_xlim(lo, hi)
        print(f"Zoomed to {lo.strftime('%H:%M:%S')} → {hi.strftime('%H:%M:%S')} "
              f"({args.margin:g}s margin around pulses)")

    # ── X axis formatting ──────────────────────────────────────────────────
    ax_bot.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    ax_bot.set_xlabel("Time")
    fig.autofmt_xdate(rotation=0, ha="center")

    # ── Legends: built explicitly so the fire marker appears exactly once per
    #    panel, after the data series, with a self-explanatory label ──────────
    from matplotlib.lines import Line2D
    pulse_proxy = Line2D([0], [0], color=C_PLS, lw=1.4)
    pulse_label = (f"valve fired  (×{len(pulse_times)})"
                   if len(pulse_times) != 1 else "valve fired")

    top_lines, top_labels = [], []
    for ax in (ax_top, ax_temp):
        for ln, lb in zip(*ax.get_legend_handles_labels()):
            top_lines.append(ln); top_labels.append(lb)
    if pulse_times:
        top_lines.append(pulse_proxy); top_labels.append(pulse_label)
    ax_top.legend(top_lines, top_labels, loc="upper right",
                  fontsize=9, framealpha=0.9)

    bot_lines, bot_labels = ax_bot.get_legend_handles_labels()
    bot_lines, bot_labels = list(bot_lines), list(bot_labels)
    if pulse_times:
        bot_lines.append(pulse_proxy); bot_labels.append(pulse_label)
    ax_bot.legend(bot_lines, bot_labels, loc="upper right",
                  fontsize=9, framealpha=0.9)

    # subplots_adjust rather than tight_layout — twinx axes aren't tight-safe
    fig.subplots_adjust(left=0.08, right=0.92, top=0.93, bottom=0.08, hspace=0.12)

    if args.out:
        fig.savefig(args.out, dpi=150, bbox_inches="tight")
        print(f"Saved: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    matplotlib.rcParams["figure.dpi"] = 110
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        import traceback
        traceback.print_exc()
        input("\nPress Enter to close...")
