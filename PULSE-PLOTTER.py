"""
PULSE-VALVE-CAPTURE-PLOTTER  —  Single-shot plot of one high-rate capture
=========================================================================
Plots and re-analyses one pulse_*.csv written by the `c` command of
PULSE-VALVE-DRIVER: the chamber pressure through a single valve shot,
captured by hardware streaming at a few kHz.

Companion to PULSE-VALVE-LOG-PLOTTER, which plots a whole session at the
0.5 s logging cadence. This one plots a single 6-second window in detail.

  • Chamber pressure vs time, t = 0 at the fire command
  • Baseline, peak, dp, and time-to-peak marked
  • Rise (10–90 %) and decay-fit regions shaded
  • Optional log-axis plot showing the exponential fit behind tau

Everything is recomputed from the raw trace, so the chamber volume can be
changed after the fact without retaking any data:

    python PULSE-VALVE-CAPTURE-PLOTTER.py capture.csv --volume 15.586

Baseline estimators (--baseline)
--------------------------------
The baseline matters more than it looks. It is subtracted before the decay
fit takes logarithms, so an error in it does not merely shift the answer —
it bends the tail, and tau comes out wrong.

  drift   (default) a straight line fitted through the pre-trigger samples
          AND the far tail, evaluated at every time point. The far tail is
          many time constants after the shot, so the pulse has vanished
          there and it is a legitimate second anchor. This MODELS the
          background drift rather than averaging through it.

  mean    mean of the last --baseline-n samples before the fire.

  all     mean of every pre-trigger sample.

The two mean estimators trade one error against another: a short window is
noisy, a long window drags in drift, because the mean of a sloping window
sits at the value from the middle of that window, not at the fire. All three
are printed on every run so the choice is visible rather than implicit.

Usage
-----
  python PULSE-VALVE-CAPTURE-PLOTTER.py [csv_file] [options]

  --volume L        chamber volume in litres (default: value stored in file)
  --limit MBAR      in-chamber pressure limit (default 1e-6)
  --baseline MODE   drift | mean | all      (default drift)
  --baseline-n N    samples used by --baseline mean (default 125)
  --xlim T0 T1      time axis limits in seconds (default -0.1 0.7)
  --logfit          also draw the log-axis plot of the decay fit
  --out FILE        save to FILE instead of showing interactively

If no CSV file is given on the command line, a file-picker dialog opens so
you can choose which capture to plot. If the dialog can't open (e.g. no
display), a numbered list of pulse_*.csv files in the current folder is
offered instead.
"""

import sys
import os
import glob
import math
import argparse
import csv

import matplotlib
import matplotlib.pyplot as plt

GUARD_S = 0.05          # ignore this much just before the fire
TAIL_FRAC = 0.75        # far-tail anchor starts this far through the window
FIT_NOISE_K = 5.0       # stop the decay fit at this × the residual scatter
FIT_START_FRACTIONS = (0.7, 0.5, 0.35, 0.25, 0.15)
FIT_MIN_POINTS = 20


# ═══════════════════════════════════════════════════════════════════════════
# FILE SELECTION
# ═══════════════════════════════════════════════════════════════════════════

def choose_csv_file():
    """Return a capture CSV path chosen by the user.

    Tries a graphical file-open dialog first (Tkinter). If that isn't
    available, falls back to a numbered list of pulse_*.csv files found in
    the current working directory.
    """
    # ── try a graphical dialog ─────────────────────────────────────────────
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        path = filedialog.askopenfilename(
            title="Select a pulse capture to plot",
            filetypes=[("Pulse captures", "pulse_*.csv"),
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
    candidates = sorted(glob.glob("pulse_*.csv")) or \
                 sorted(glob.glob(os.path.join("logs", "pulse_*.csv"))) or \
                 sorted(glob.glob("*.csv"))
    if not candidates:
        print("No capture CSV files found in the current folder.")
        print("Pass a file explicitly:")
        print("  python PULSE-VALVE-CAPTURE-PLOTTER.py <pulse_file.csv>")
        sys.exit(1)

    print("\nSelect a capture to plot:")
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


# ═══════════════════════════════════════════════════════════════════════════
# FILE INPUT
# ═══════════════════════════════════════════════════════════════════════════

def read_capture(path):
    """Return (meta dict, times list, mbar list)."""
    meta, times, mbar = {}, [], []
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if not row:
                continue
            if row[0].startswith("#"):
                meta[row[0].lstrip("# ").strip()] = row[1] if len(row) > 1 else ""
            elif row[0] == "t_s":
                continue
            else:
                times.append(float(row[0]))
                mbar.append(float(row[1]))
    if len(times) < 20:
        print(f"{os.path.basename(path)} does not look like a pulse capture "
              f"(only {len(times)} data rows).")
        print("Session logs (pvd-sensor_*.csv) go to PULSE-VALVE-LOG-PLOTTER.")
        sys.exit(1)
    return meta, times, mbar


# ═══════════════════════════════════════════════════════════════════════════
# BASELINES
# ═══════════════════════════════════════════════════════════════════════════

def _linfit(pairs):
    """Least-squares (slope, intercept) for a list of (x, y)."""
    n = len(pairs)
    mx = sum(x for x, _ in pairs) / n
    my = sum(y for _, y in pairs) / n
    sxx = sum((x - mx) ** 2 for x, _ in pairs)
    sxy = sum((x - mx) * (y - my) for x, y in pairs)
    slope = sxy / sxx if sxx > 0 else 0.0
    return slope, my - slope * mx


def baselines(times, mbar, n_mean):
    """Compute all three baseline estimators.

    Each carries a callable b(t), so a sloping baseline and a flat one are
    handled identically downstream.
    """
    pre = [(t, p) for t, p in zip(times, mbar) if t < -GUARD_S]
    if not pre:
        print("Capture has no pre-trigger samples — cannot establish a baseline.")
        sys.exit(1)

    dt = times[1] - times[0]
    tail_start = max(times) * TAIL_FRAC
    tail = [(t, p) for t, p in zip(times, mbar) if t > tail_start]

    out = {}

    m_all = sum(p for _, p in pre) / len(pre)
    out["all"] = dict(fn=lambda t, m=m_all: m, n=len(pre), slope=0.0,
                      label=f"mean of {len(pre)} pre-trigger samples")

    seg = pre[-n_mean:]
    m_n = sum(p for _, p in seg) / len(seg)
    out["mean"] = dict(fn=lambda t, m=m_n: m, n=len(seg), slope=0.0,
                       label=f"mean of last {len(seg)} samples "
                             f"({len(seg)*dt:.2f} s)")

    anchor = pre + tail
    slope, icept = _linfit(anchor)
    out["drift"] = dict(fn=lambda t, s=slope, c=icept: s * t + c,
                        n=len(anchor), slope=slope,
                        label=f"linear fit, {len(pre)} pre + {len(tail)} tail")

    resid = [p - (slope * t + icept) for t, p in anchor]
    mr = sum(resid) / len(resid)
    out["_sd"] = math.sqrt(sum((r - mr) ** 2 for r in resid) / len(resid))
    out["_tail_start"] = tail_start
    return out


# ═══════════════════════════════════════════════════════════════════════════
# ANALYSIS
# ═══════════════════════════════════════════════════════════════════════════

def fit_decay(times, excess, floor):
    """Best single-exponential fit of the tail of *excess*. Dict or None.

    Several start points are tried and the best r-squared wins. The fit is
    only valid where pumping is the only process acting, and where that
    begins depends on how long gas keeps arriving through the capillary.
    """
    if not excess:
        return None
    i_peak = max(range(len(excess)), key=lambda i: excess[i])
    peak = excess[i_peak]
    if peak <= floor:
        return None

    best = None
    for frac in FIT_START_FRACTIONS:
        i0 = next((i for i in range(i_peak, len(excess))
                   if excess[i] <= frac * peak), None)
        if i0 is None:
            continue
        xs, ys = [], []
        for t, e in zip(times[i0:], excess[i0:]):
            if e <= floor:
                break
            xs.append(t)
            ys.append(math.log(e))
        if len(xs) < FIT_MIN_POINTS:
            continue
        slope, icept = _linfit(list(zip(xs, ys)))
        if slope >= 0:
            continue
        my = sum(ys) / len(ys)
        ss_res = sum((y - (slope * x + icept)) ** 2 for x, y in zip(xs, ys))
        ss_tot = sum((y - my) ** 2 for y in ys)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        cand = dict(tau=-1.0 / slope, r2=r2, n=len(xs), t0=xs[0], t1=xs[-1],
                    frac=frac, slope=slope, icept=icept)
        if best is None or r2 > best["r2"]:
            best = cand
    return best


def analyse(times, mbar, base_fn, sd, volume_l):
    i_fire = next(i for i, t in enumerate(times) if t >= 0)
    post_t = times[i_fire:]
    post_p = mbar[i_fire:]
    post_e = [p - base_fn(t) for t, p in zip(post_t, post_p)]

    i_pk = max(range(len(post_e)), key=lambda i: post_e[i])
    dp, t_peak, peak = post_e[i_pk], post_t[i_pk], post_p[i_pk]

    t10 = next((t for t, e in zip(post_t, post_e) if e >= 0.1 * dp), None)
    t90 = next((t for t, e in zip(post_t, post_e) if e >= 0.9 * dp), None)

    dt = times[1] - times[0]
    # The integral runs over the WHOLE post-fire trace, not just the tail:
    # every molecule the pump removes counts, whenever it happened to arrive.
    integral = sum(max(0.0, e) for e in post_e) * dt

    fit = fit_decay(post_t, post_e, FIT_NOISE_K * sd)
    a = dict(peak=peak, dp=dp, t_peak=t_peak, t10=t10, t90=t90,
             integral=integral, fit=fit, dt=dt, sd=sd, volume_l=volume_l,
             q_vdp=volume_l * dp, post_t=post_t, post_e=post_e)
    if fit:
        a["tau"] = fit["tau"]
        a["s_eff"] = volume_l / fit["tau"]
        a["q_int"] = a["s_eff"] * integral
    return a


# ═══════════════════════════════════════════════════════════════════════════
# PLOTTING
# ═══════════════════════════════════════════════════════════════════════════

C_TRACE = "#1f5fd0"
C_BASE = "#7a7a7a"
C_FIRE = "#c03030"
C_RISE = "#d9772b"
C_FIT = "#1d9e75"
C_ANN = "#4a4a4a"


def plot_trace(fig, ax, times, mbar, base_fn, a, meta, xlim, title_extra=""):
    scale = 1e-7

    if a["t10"] is not None and a["t90"] is not None:
        ax.axvspan(a["t10"], a["t90"], color=C_RISE, alpha=0.13, lw=0,
                   label=f"rise 10–90 %  ({(a['t90']-a['t10'])*1000:.0f} ms)")
    if a.get("fit"):
        ax.axvspan(a["fit"]["t0"], a["fit"]["t1"], color=C_FIT, alpha=0.13,
                   lw=0, label=f"decay fit  (τ = {a['tau']*1000:.1f} ms)")

    ax.plot(times, [p / scale for p in mbar], color=C_TRACE, lw=1.4, zorder=3)
    ax.plot(times, [base_fn(t) / scale for t in times], color=C_BASE,
            ls="--", lw=1.1, zorder=2,
            label=f"baseline  {base_fn(0.0):.4e} mbar at t = 0")

    ax.set_xlim(*xlim)
    vis = [p / scale for t, p in zip(times, mbar) if xlim[0] <= t <= xlim[1]]
    lo, hi = min(vis), max(vis)
    pad = 0.10 * (hi - lo)
    ax.set_ylim(lo - pad, hi + 1.9 * pad)
    span = ax.get_ylim()[1] - ax.get_ylim()[0]

    ax.axvline(0.0, color=C_FIRE, lw=1.4, zorder=4)
    ax.annotate("valve fires  (t = 0)", xy=(0, 0.30),
                xycoords=("data", "axes fraction"),
                xytext=(7, 0), textcoords="offset points",
                color=C_FIRE, fontsize=10)

    ax.plot([a["t_peak"]], [a["peak"] / scale], "o", ms=6, color=C_TRACE,
            mec="white", mew=1.4, zorder=5)
    ax.annotate(f"peak {a['peak']:.3e} mbar\nΔp = {a['dp']:.3e}",
                xy=(a["t_peak"], a["peak"] / scale),
                xytext=(16, 4), textcoords="offset points", fontsize=10,
                color="#111111",
                arrowprops=dict(arrowstyle="-", color=C_BASE, lw=0.8))

    y_arrow = a["peak"] / scale + 0.055 * span
    ax.annotate("", xy=(a["t_peak"], y_arrow), xytext=(0, y_arrow),
                arrowprops=dict(arrowstyle="<->", color=C_ANN, lw=1.0))
    ax.text(a["t_peak"] / 2, y_arrow + 0.015 * span,
            f"time to peak  {a['t_peak']*1000:.0f} ms",
            ha="center", fontsize=10, color=C_ANN)

    ax.set_xlabel("Time from valve fire (s)")
    ax.set_ylabel("Chamber pressure  (×10⁻⁷ mbar)")
    ax.grid(True, alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_title(f"Gyger pulse — open {meta.get('open_time_us','?')} µs — "
                 f"captured at {meta.get('capture_rate_hz','?')} Hz{title_extra}",
                 fontsize=12, loc="left")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)


def plot_logfit(fig, ax, a, title_extra=""):
    """Excess on a log axis with the fitted line — what the fit actually sees."""
    f = a["fit"]
    pts = [(t, e) for t, e in zip(a["post_t"], a["post_e"])
           if e > FIT_NOISE_K * a["sd"]]
    ax.semilogy([t for t, _ in pts], [e for _, e in pts],
                color=C_TRACE, lw=1.4, label="excess above baseline")
    ax.axvspan(f["t0"], f["t1"], color=C_FIT, alpha=0.13, lw=0,
               label="region fitted")
    xs = [f["t0"] - 0.15, f["t1"] + 0.15]
    ax.semilogy(xs, [math.exp(f["slope"] * x + f["icept"]) for x in xs],
                color=C_FIRE, ls="--", lw=1.5,
                label=f"fitted line: τ = {a['tau']*1000:.1f} ms, "
                      f"r² = {f['r2']:.4f}")
    ax.axhline(FIT_NOISE_K * a["sd"], color=C_BASE, ls=":", lw=1.0,
               label=f"noise floor ({FIT_NOISE_K:g} × sd)")

    ax.set_xlabel("Time from valve fire (s)")
    ax.set_ylabel("Excess above baseline (mbar)")
    ax.grid(True, which="both", alpha=0.25)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.set_title("An exponential is a straight line on a log axis" + title_extra,
                 fontsize=12, loc="left")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="Plot and re-analyse one PULSE-VALVE-DRIVER capture")
    ap.add_argument("csv", nargs="?", default=None,
                    help="pulse_*.csv file (if omitted, a picker opens)")
    ap.add_argument("--volume", type=float, default=None,
                    help="chamber volume in litres (default: value in the file)")
    ap.add_argument("--limit", type=float, default=1.0e-6,
                    help="in-chamber pressure limit, mbar (default 1e-6)")
    ap.add_argument("--baseline", choices=("drift", "mean", "all"),
                    default="drift")
    ap.add_argument("--baseline-n", type=int, default=125,
                    help="samples used by --baseline mean (default 125)")
    ap.add_argument("--xlim", type=float, nargs=2, default=(-0.10, 0.70),
                    metavar=("T0", "T1"),
                    help="time axis limits in seconds (default -0.1 0.7)")
    ap.add_argument("--logfit", action="store_true",
                    help="also draw the log-axis plot of the decay fit")
    ap.add_argument("--out", default=None,
                    help="save to file instead of showing interactively")
    args = ap.parse_args()

    csv_path = args.csv if args.csv else choose_csv_file()
    print(f"Loading: {csv_path}")

    meta, times, mbar = read_capture(csv_path)
    volume = args.volume if args.volume is not None else float(
        meta.get("chamber_volume_l", 10.0))
    dt = times[1] - times[0]

    bl = baselines(times, mbar, args.baseline_n)
    sd = bl["_sd"]

    print(f"  samples    {len(times)}  from {times[0]:+.2f} to {times[-1]:+.2f} s "
          f"({1/dt:.0f} Hz)")
    print(f"  open time  {meta.get('open_time_us','?')} µs")
    print(f"  scatter    {sd:.2e} mbar per point")
    print()
    print("Baseline estimators, value at t = 0:")
    for key in ("drift", "mean", "all"):
        b = bl[key]
        se = sd / math.sqrt(b["n"])
        extra = ""
        if key == "drift":
            se *= 2          # intercept at the edge of the fitted range
            extra = f", drift {b['slope']:+.2e} mbar/s"
        mark = "  <-- used" if key == args.baseline else ""
        print(f"  {key:6s} {b['fn'](0.0):.5e}  ± ~{se:.1e}   "
              f"[{b['label']}{extra}]{mark}")
    vals = [bl[k]["fn"](0.0) for k in ("drift", "mean", "all")]
    print(f"  spread between estimators: {max(vals)-min(vals):.2e} mbar")

    base_fn = bl[args.baseline]["fn"]
    a = analyse(times, mbar, base_fn, sd, volume)

    print()
    print(f"peak         {a['peak']:.4e} mbar  "
          f"({100*a['peak']/args.limit:.1f} % of the {args.limit:.0e} limit)")
    print(f"  Δp         {a['dp']:.4e} mbar  "
          f"({100*a['dp']/args.limit:.1f} % of the limit)")
    print(f"  t_peak     {a['t_peak']*1000:.0f} ms after the valve fires")
    if a["t10"] is not None and a["t90"] is not None:
        print(f"  rise 10–90 {(a['t90']-a['t10'])*1000:.0f} ms")
    print(f"  integral   {a['integral']:.4e} mbar·s  (whole post-fire trace)")

    if a.get("fit"):
        f = a["fit"]
        print()
        print(f"tau          {a['tau']*1000:.1f} ms  r² {f['r2']:.4f}  {f['n']} pts, "
              f"fit from {f['t0']*1000:.0f} to {f['t1']*1000:.0f} ms")
        print(f"  half-life  {a['tau']*math.log(2)*1000:.0f} ms   "
              f"back to baseline (5 τ) in {a['tau']*5*1000:.0f} ms")
        print(f"S_eff        {a['s_eff']:.1f} L/s  (= V / τ, V = {volume:g} L)")
        print(f"Q_pulse      {a['q_int']:.4e} mbar·L  = S_eff × integral")
        print(f"             {a['q_vdp']:.4e} mbar·L  = V × Δp")
        ratio = a["q_vdp"] / a["q_int"] if a["q_int"] else float("nan")
        print(f"  ratio      {ratio:.3f}  (V cancels here: this compares SHAPE only)")
        if a["t90"] is not None and a["t10"] is not None:
            tr = a["t90"] - a["t10"]
            if tr > 0.15 * a["tau"]:
                print(f"  the rise is {100*tr/a['tau']:.0f} % of τ, so the injection")
                print("  is not impulsive: V × Δp under-reads. Use the integral route.")
        if a["tau"] * 5 > bl["_tail_start"]:
            print(f"  WARNING: 5 τ ({a['tau']*5:.2f} s) reaches past the far-tail")
            print(f"  anchor at {bl['_tail_start']:.2f} s, so the drift baseline may")
            print("  be contaminated by the pulse. Re-run with --baseline mean.")
    else:
        print("\nNo usable decay fit: the peak excess is too close to the noise floor.")

    # ── Figure ────────────────────────────────────────────────────────────
    want_log = args.logfit and a.get("fit")
    if want_log:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 10))
    else:
        fig, ax1 = plt.subplots(figsize=(13, 6.5))
        ax2 = None

    plot_trace(fig, ax1, times, mbar, base_fn, a, meta, tuple(args.xlim),
               title_extra=f" — V = {volume:g} L")
    if ax2 is not None:
        plot_logfit(fig, ax2, a,
                    title_extra=f"  (open {meta.get('open_time_us','?')} µs)")

    fig.subplots_adjust(left=0.09, right=0.97, top=0.94, bottom=0.08, hspace=0.30)

    if args.out:
        fig.savefig(args.out, dpi=150, bbox_inches="tight")
        print(f"\nSaved: {args.out}")
    else:
        try:
            plt.show()
        except Exception:
            fallback = os.path.splitext(csv_path)[0] + ".png"
            fig.savefig(fallback, dpi=150, bbox_inches="tight")
            print(f"\nNo display available — saved: {fallback}")


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
