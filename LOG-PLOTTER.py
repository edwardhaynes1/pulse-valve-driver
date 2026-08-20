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
  --smooth N      Moving-average window (rows) for the upstream traces at plot
                  time; 0 = raw (default). The log is stored raw, so any
                  averaging is a plotting choice.

If no CSV file is given on the command line, a file-picker dialog opens so you
can choose which log to plot. If the dialog can't open (e.g. no display), a
numbered list of pvd-log_*.csv files in the current folder is offered
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


def smooth(vals, n):
    """Trailing moving average over a window of n rows, skipping None. Used at
    plot time only — the logged data is raw. A point with no usable samples in
    its window stays None, so gaps are preserved."""
    if not n or n <= 1:
        return list(vals)
    out, run = [], []
    for v in vals:
        run.append(v)
        if len(run) > n:
            run.pop(0)
        good = [x for x in run if x is not None]
        out.append(sum(good) / len(good) if good else None)
    return out



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
            filetypes=[("PVD session logs", "pvd-log_*.csv"),
                       ("PVD session logs (pre-Aug 2026)", "pvd-sensor_*.csv"),
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
    candidates = (sorted(glob.glob("pvd-log_*.csv"))
                  + sorted(glob.glob("pvd-sensor_*.csv"))   # pre-Aug 2026 name
                  or sorted(glob.glob("*.csv")))
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


def _cap_fmt(idd, ln):
    """Format one capillary, or 'none' when it is recorded as not fitted."""
    if not idd:
        return None                       # field absent — unknown
    try:
        if float(idd) <= 0:
            return "none"
    except ValueError:
        pass
    return f"{idd} µm × {ln} mm" if ln else f"{idd} µm"


def session_identity_str(rows):
    """One-line capillary + gas caption for a session.

    Reads the two-capillary columns written from August 2026 onward, and falls
    back to the single legacy `capillary_*` pair, which was the downstream
    capillary under its old name. If the setting changed mid-session — the
    driver allows live cap/gas changes — the states are listed in the order
    they appeared rather than collapsed to a bare 'mixed'. Returns '' only when
    no identity columns exist at all.
    """
    if not rows:
        return ""
    have_new = "downstream_cap_id_um" in rows[0]
    have_old = "capillary_id_um" in rows[0]
    if not (have_new or have_old):
        return ""

    def state(r):
        if have_new:
            d = _cap_fmt((r.get("downstream_cap_id_um") or "").strip(),
                         (r.get("downstream_cap_length_mm") or "").strip())
            u = _cap_fmt((r.get("upstream_cap_id_um") or "").strip(),
                         (r.get("upstream_cap_length_mm") or "").strip())
            if d is None and u is None:
                return None
            cap = f"down {d or 'n/a'}  ·  up {u or 'n/a'}"
        else:
            d = _cap_fmt((r.get("capillary_id_um") or "").strip(),
                         (r.get("capillary_length_mm") or "").strip())
            if d is None:
                return None
            # legacy logs recorded only the downstream capillary
            cap = f"down {d}  ·  up n/a"
        gas = (r.get("gas_species") or "").strip()
        return cap, (gas if gas else "gas n/a")

    seen, order = set(), []
    for r in rows:
        st = state(r)
        if st is None:
            continue
        if st not in seen:
            seen.add(st)
            order.append(st)

    if not order:
        return ""
    if len(order) == 1:
        cap, gas = order[0]
        return f"{cap}  ·  gas {gas}"
    if len(order) <= 3:
        return "capillary/gas changed:    " + "    →    ".join(
            f"{cap} · {gas}" for cap, gas in order)
    return (f"capillary/gas: {len(order)} distinct settings this session "
            f"— see per-row columns")


def main():
    ap = argparse.ArgumentParser(description="Overlay plot of a PVD session")
    ap.add_argument("csv", nargs="?", default=None,
                    help="pvd-log_*.csv file (if omitted, a picker opens)")
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
    ap.add_argument("--gauge-floor", type=float, default=1e-11, metavar="MBAR",
                    help="rows whose vacuum reading is below this are treated "
                         "as the gauge not yet reading and are dropped from the "
                         "vacuum trace (default 1e-11). The chamber never sees "
                         "1e-11, so such rows are an offline gauge, not data; "
                         "left in, they stretch the log axis over five empty "
                         "decades. Use 0 to keep everything.")
    ap.add_argument("--smooth", type=int, default=0, metavar="N",
                    help="moving-average window (in rows) applied at plot time "
                         "to the raw upstream traces; 0 = raw (default). The log "
                         "is stored raw, so smoothing is a plotting choice.")
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
    if args.smooth and args.smooth > 1:
        up_p = smooth(up_p, args.smooth)
        up_t = smooth(up_t, args.smooth)
        print(f"Upstream traces smoothed with a {args.smooth}-row moving average "
              f"(plot-time only; log is raw)")
    vac       = [to_float(r["vacuum_chamber_mbar"]) for r in rows]
    if args.gauge_floor and args.gauge_floor > 0:
        n_off = sum(1 for v in vac if v is not None and v < args.gauge_floor)
        if n_off:
            vac = [None if (v is not None and v < args.gauge_floor) else v
                   for v in vac]
            if n_off == len(vac):
                print(f"WARNING: every vacuum reading in this log is below "
                      f"{args.gauge_floor:g} mbar. The gauge was offline for the "
                      f"whole session — the vacuum panel will be empty. Check the "
                      f"gauge connection before trusting anything from this file.")
            else:
                print(f"Vacuum: {n_off} of {len(vac)} rows below "
                      f"{args.gauge_floor:g} mbar treated as gauge-not-reading "
                      f"and omitted (use --gauge-floor 0 to keep them)")

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
    _dur = (t1 - t0).total_seconds()
    _dur_s = (f"{_dur:.0f} s" if _dur < 120
              else f"{_dur/60:.1f} min" if _dur < 7200
              else f"{_dur/3600:.1f} h")
    print(f"Session started: {t0.strftime('%d/%m/%y %H:%M:%S')}")
    print(f"Plotting {section}: {len(rows)} rows, "
          f"{t0.strftime('%H:%M:%S')} → {t1.strftime('%H:%M:%S')} ({_dur_s}), "
          f"{len(pulse_times)} pulses")

    # ── Figure: two stacked panels sharing the x-axis ──────────────────────
    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1, figsize=(13, 8), sharex=True,
        gridspec_kw={"height_ratios": [1, 1], "hspace": 0.12},
    )
    _ident = session_identity_str(rows)
    if _ident:
        # A mid-session change lists every state, which easily overruns the
        # figure width. Break it at the arrows and shrink the type so the
        # whole caption stays inside the canvas.
        _ident = _ident.replace("    →    ", "\n   →   ")
    _sub = f"\n{_ident}" if _ident else ""
    # Start date and time, so a saved figure identifies its own session
    # without needing the filename.
    _sub += f"\n{t0.strftime('%d/%m/%y %H:%M:%S')} · {_dur_s}"
    _sub_lines = _sub.count("\n")
    fig.suptitle(f"Pulse valve session — {section}{_sub}",
                 fontsize=13 if _sub_lines <= 1 else 10, y=0.98)

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
    fig.subplots_adjust(left=0.08, right=0.92,
                        top=0.93 - 0.025 * max(0, _sub_lines - 1),
                        bottom=0.08, hspace=0.12)

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
