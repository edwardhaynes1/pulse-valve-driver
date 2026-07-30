"""
PULSE-PLOTTER  —  Plot and compare high-rate pulse captures
===========================================================
Plots and re-analyses pulse_*.csv files written by the `c` command of
PULSE-VALVE-DRIVER: the chamber pressure through a single valve shot,
captured by hardware streaming at a few kHz.

Companion to LOG-PLOTTER, which plots a whole session at the 0.5 s logging
cadence. This one plots individual shots in detail.

One capture:
  • Chamber pressure vs time, t = 0 at the fire command
  • Baseline, peak, dp and time-to-peak marked
  • Rise (10–90 %), fast-decay and slow-tail regions shaded
  • τ_fast, τ_slow, S_eff and Q_pulse shown in a summary box

Several captures:
  • All traces overlaid on a common time axis, aligned at the fire
  • Grouped and coloured by valve open time, with a bold group-mean trace
  • Annotations show the GROUP MEAN peak and time-to-peak, not one shot
  • Per-group statistics table: dp, t_peak, rise, τ_fast, τ_slow, S_eff,
    Q_pulse,
    each as mean ± standard deviation across the shots in that group

Traces are plotted as excess above each capture's OWN baseline, because
captures taken minutes apart sit on different backgrounds and only the
excess is comparable. Use --absolute to plot raw chamber pressure instead.

Shots the driver flagged as non-detections (`detected,0` in the header) are
excluded from group statistics and reported separately — averaging a real
pulse together with a shot that delivered nothing gives a meaningless
middle value.

Everything is recomputed from the raw trace, so the chamber volume can be
changed after the fact without retaking any data.

Two time constants
-------------------
The decay is fit twice. τ_fast is the chamber pumping down through the
capillary, and it is the one that sets throughput: S_eff = V / τ_fast.
τ_slow, when it can be resolved, is the slower fall left in the tail — on
this rig the hypothesis is gas still bleeding out of the capillary volume.

τ_slow is deliberately conservative: it is reported only when the tail
clears the noise floor AND comes out genuinely slower than τ_fast, and is
otherwise shown as "not resolved" rather than printing a number that is
really noise. Because the slow tail is small, the drift baseline can eat it
— its far-tail anchor assumes the pulse has vanished there, which a real
slow tail has not — so use --baseline mean when chasing τ_slow, and give it
a capture window several τ_slow long. Averaging many shots (the group mean)
is the surest way to lift the tail clear of the noise.

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
sits at the value from the middle of that window, not at the fire.

Usage
-----
  python PULSE-PLOTTER.py [csv_file ...] [options]

  python PULSE-PLOTTER.py                          pick files, multi-select
  python PULSE-PLOTTER.py cap.csv                  one capture, full detail
  python PULSE-PLOTTER.py logs/pulse_*.csv         overlay everything
  python PULSE-PLOTTER.py a.csv b.csv --volume 15.586

  --volume L        chamber volume in litres (default: value stored in file)
  --limit MBAR      in-chamber pressure limit (default 1e-6)
  --baseline MODE   drift | mean | all      (default drift)
  --baseline-n N    samples used by --baseline mean (default 125)
  --xlim T0 T1      time axis limits in seconds
                    (default: auto — extends to each trace's return to baseline)
  --absolute        plot raw chamber pressure instead of excess
  --no-individual   draw only the group-mean traces
  --logfit          add a log-axis panel of the decay
  --linear          use the linear excess/absolute view instead of the
                    default log-pressure axis (baseline → 10⁻⁶ limit)
  --out FILE        save to FILE instead of showing interactively

Shell globs are expanded internally, so `pulse_*.csv` works on Windows too.

If no file is given, a multi-select file-picker dialog opens. If the dialog
can't open (e.g. no display), a numbered list of pulse_*.csv files in the
current folder is offered instead; enter several numbers separated by
spaces, or `a` for all.
"""

import sys
import os
import glob
import math
import argparse
import csv

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter

GUARD_S = 0.05          # ignore this much just before the fire
TAIL_FRAC = 0.75        # far-tail anchor starts this far through the window
FIT_NOISE_K = 5.0       # stop the decay fit at this × the residual scatter
FIT_START_FRACTIONS = (0.7, 0.5, 0.35, 0.25, 0.15)
FIT_MIN_POINTS = 20
# Fit only the FAST component: stop once the excess falls below this fraction
# of the peak, so a slow wall-desorption tail can't drag τ (and hence S_eff)
# high. On a clean single-exponential shot this changes nothing — any segment
# of a straight log-decay gives the same slope — and on small shots the noise
# floor is reached first, so it only bites when there is a real slow tail.
FIT_FAST_FRAC = 0.2

# Slow tail (τ_slow): a second single-exponential fit over the window BELOW the
# fast region and ABOVE the noise floor. It only reports when that window holds
# enough points clear of the noise, fits a straight log-line well, and resolves
# as genuinely slower than τ_fast — otherwise "not resolved", so a noise-driven
# tail never masquerades as a measurement.
FIT_SLOW_MIN_POINTS = 25
FIT_SLOW_MIN_R2 = 0.80
FIT_SLOW_MIN_RATIO = 1.8   # τ_slow must exceed this × τ_fast to count as distinct
FIT_SLOW_SETTLE = 3.0      # start ≥ this × τ_fast past the peak, so fast is gone
FIT_SLOW_BREAK_N = 5       # end after this many consecutive points below the floor

# Auto time-window: how far right to draw so the return to baseline is shown.
RETURN_K = 3.0          # excess is "back to baseline" below this × scatter
TAIL_MARGIN_S = 0.15    # flat tail to keep past the return, minimum


# ═══════════════════════════════════════════════════════════════════════════
# FILE SELECTION
# ═══════════════════════════════════════════════════════════════════════════

def choose_csv_files():
    """Return a list of capture CSV paths chosen by the user.

    Tries a graphical multi-select dialog first (Tkinter). If that isn't
    available, falls back to a numbered list of pulse_*.csv files found in
    the current working directory, accepting several numbers at once.
    """
    # ── try a graphical dialog ─────────────────────────────────────────────
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        paths = filedialog.askopenfilenames(
            title="Select one or more pulse captures (Ctrl/Shift to multi-select)",
            filetypes=[("Pulse captures", "pulse_*.csv"),
                       ("CSV files", "*.csv"),
                       ("All files", "*.*")],
        )
        root.destroy()
        if paths:
            return list(paths)
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
        print("Pass files explicitly:")
        print("  python PULSE-PLOTTER.py <pulse_file.csv> [more.csv ...]")
        sys.exit(1)

    print("\nSelect one or more captures to plot:")
    for i, name in enumerate(candidates, 1):
        print(f"  {i:>2}. {name}")
    while True:
        raw = input(f"\nNumbers separated by spaces (1-{len(candidates)}), "
                    f"'a' for all, or q to quit: ").strip().lower()
        if raw == "q":
            sys.exit(0)
        if raw == "a":
            return candidates
        try:
            idx = [int(x) for x in raw.split()]
            if idx and all(1 <= i <= len(candidates) for i in idx):
                return [candidates[i - 1] for i in idx]
        except ValueError:
            pass
        print("  invalid choice")


def expand_paths(args_paths):
    """Expand shell globs internally — Windows does not do it for us."""
    out = []
    for p in args_paths:
        hits = sorted(glob.glob(p))
        if hits:
            out.extend(hits)
        elif os.path.exists(p):
            out.append(p)
        else:
            print(f"No file matches: {p}")
    if not out:
        sys.exit(1)
    # de-duplicate, preserve order
    seen, uniq = set(), []
    for p in out:
        rp = os.path.realpath(p)
        if rp not in seen:
            seen.add(rp)
            uniq.append(p)
    return uniq


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
        print("Session logs (pvd-sensor_*.csv) go to LOG-PLOTTER.")
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
    # Fit only where the fast component dominates: down to FIT_FAST_FRAC of the
    # peak, or the noise floor, whichever is higher.
    fit_floor = max(floor, FIT_FAST_FRAC * peak)
    for frac in FIT_START_FRACTIONS:
        i0 = next((i for i in range(i_peak, len(excess))
                   if excess[i] <= frac * peak), None)
        if i0 is None:
            continue
        xs, ys = [], []
        for t, e in zip(times[i0:], excess[i0:]):
            if e <= fit_floor:
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


def fit_slow(times, excess, i_peak, peak, floor, tau_fast, t_peak):
    """Single-exponential fit of the SLOW tail, below the fast region.

    Returns a dict shaped like fit_decay's, or None when the tail can't be
    trusted: too few points above the noise floor, a poor straight-line fit on
    the log axis, or a time constant not meaningfully slower than τ_fast (in
    which case the "tail" is just noise or leftover fast decay). The window runs
    from FIT_FAST_FRAC·peak down to the noise floor, starting only once the fast
    component has had FIT_SLOW_SETTLE τ_fast to die away.
    """
    if not tau_fast or tau_fast <= 0:
        return None
    lo = floor                        # noise floor (already FIT_NOISE_K · sd)
    hi = FIT_FAST_FRAC * peak         # top of the slow window (bottom of fast)
    if hi <= lo:                      # no gap between fast region and noise
        return None
    t_start = t_peak + FIT_SLOW_SETTLE * tau_fast

    xs, ys, below = [], [], 0
    for t, e in zip(times[i_peak:], excess[i_peak:]):
        if t < t_start or e >= hi:
            continue
        if e <= lo:                   # into the noise — end after a sustained run
            below += 1
            if below >= FIT_SLOW_BREAK_N:
                break
            continue
        below = 0
        xs.append(t)
        ys.append(math.log(e))

    if len(xs) < FIT_SLOW_MIN_POINTS:
        return None
    slope, icept = _linfit(list(zip(xs, ys)))
    if slope >= 0:
        return None
    tau = -1.0 / slope
    if tau < FIT_SLOW_MIN_RATIO * tau_fast:
        return None
    my = sum(ys) / len(ys)
    ss_res = sum((y - (slope * x + icept)) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    if r2 < FIT_SLOW_MIN_R2:
        return None
    return dict(tau=tau, r2=r2, n=len(xs), t0=xs[0], t1=xs[-1],
                slope=slope, icept=icept)


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
             q_vdp=volume_l * dp, post_t=post_t, post_e=post_e,
             tau_slow=None, fit_slow=None)
    if fit:
        a["tau"] = fit["tau"]        # τ_fast — the chamber-reset constant
        a["s_eff"] = volume_l / fit["tau"]
        a["q_int"] = a["s_eff"] * integral
        slow = fit_slow(post_t, post_e, i_pk, dp, FIT_NOISE_K * sd,
                        fit["tau"], t_peak)
        a["fit_slow"] = slow
        a["tau_slow"] = slow["tau"] if slow else None
    return a


# ═══════════════════════════════════════════════════════════════════════════
# LOADING AND GROUPING
# ═══════════════════════════════════════════════════════════════════════════

def load_one(path, args):
    """Read one capture, analyse it, and return a record dict."""
    meta, times, mbar = read_capture(path)
    volume = args.volume if args.volume is not None else float(
        meta.get("chamber_volume_l", 15.586))

    bl = baselines(times, mbar, args.baseline_n)
    base_fn = bl[args.baseline]["fn"]
    a = analyse(times, mbar, base_fn, bl["_sd"], volume)

    # The driver marks shots where no pulse was detectable. Trust that flag if
    # it is present; older captures predate it, so fall back to a check here.
    if "detected" in meta:
        detected = str(meta["detected"]).strip() not in ("0", "", "False")
    else:
        detected = a["dp"] > 10.0 * bl["_sd"]

    return dict(path=path, name=os.path.basename(path), meta=meta,
                times=times, mbar=mbar, base_fn=base_fn, sd=bl["_sd"],
                bl=bl, a=a, volume=volume, detected=detected,
                open_us=meta.get("open_time_us", "?"))


def group_by_open_time(records):
    """Group records by valve open time, ordered by that time."""
    groups = {}
    for r in records:
        groups.setdefault(r["open_us"], []).append(r)

    def sort_key(k):
        try:
            return (0, float(k))
        except (TypeError, ValueError):
            return (1, 0.0)

    return [(k, groups[k]) for k in sorted(groups, key=sort_key)]


def mean_sd(values):
    """Return (mean, sd, n). sd is None for fewer than two values."""
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None, 0
    m = sum(vals) / len(vals)
    if len(vals) < 2:
        return m, None, len(vals)
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))
    return m, sd, len(vals)


def fmt_ms(m, sd, n):
    if m is None:
        return "—"
    if sd is None:
        return f"{m*1000:.1f}" + (" (n=1)" if n == 1 else "")
    return f"{m*1000:.1f} ± {sd*1000:.1f}"


def fmt_sci(m, sd, n):
    if m is None:
        return "—"
    if sd is None:
        return f"{m:.3e}" + (" (n=1)" if n == 1 else "")
    return f"{m:.3e} ± {sd:.1e}"


def group_stats(recs):
    """Mean ± sd of every derived quantity across the detected shots."""
    good = [r for r in recs if r["detected"]]
    g = dict(n_total=len(recs), n_good=len(good),
             n_bad=len(recs) - len(good))
    if not good:
        return g
    pick = lambda key: [r["a"].get(key) for r in good]
    g["dp"] = mean_sd(pick("dp"))
    g["peak"] = mean_sd(pick("peak"))
    g["t_peak"] = mean_sd(pick("t_peak"))
    g["rise"] = mean_sd([(r["a"]["t90"] - r["a"]["t10"])
                         if (r["a"]["t10"] is not None and
                             r["a"]["t90"] is not None) else None
                         for r in good])
    g["integral"] = mean_sd(pick("integral"))
    g["tau"] = mean_sd(pick("tau"))
    g["tau_slow"] = mean_sd(pick("tau_slow"))
    g["s_eff"] = mean_sd(pick("s_eff"))
    g["q_int"] = mean_sd(pick("q_int"))
    g["base"] = mean_sd([r["base_fn"](0.0) for r in good])
    return g


def resample_mean(recs, use_excess):
    """Average several traces onto a common time grid.

    Captures may differ in sample rate or window length, so the grid runs
    over the overlap only, at the coarsest spacing present — averaging onto
    a finer grid than the data would invent resolution that isn't there.
    """
    good = [r for r in recs if r["detected"]]
    if len(good) < 2:
        return None, None
    t_lo = max(r["times"][0] for r in good)
    t_hi = min(r["times"][-1] for r in good)
    step = max(r["times"][1] - r["times"][0] for r in good)
    n = int((t_hi - t_lo) / step) + 1
    grid = [t_lo + i * step for i in range(n)]

    stacked = []
    for r in good:
        ts, ps = r["times"], r["mbar"]
        ys = [p - r["base_fn"](t) for t, p in zip(ts, ps)] if use_excess else ps
        out, j = [], 0
        for g in grid:
            while j < len(ts) - 2 and ts[j + 1] < g:
                j += 1
            t0, t1 = ts[j], ts[j + 1]
            f = 0.0 if t1 == t0 else (g - t0) / (t1 - t0)
            out.append(ys[j] + f * (ys[j + 1] - ys[j]))
        stacked.append(out)

    mean = [sum(col) / len(col) for col in zip(*stacked)]
    return grid, mean


# ═══════════════════════════════════════════════════════════════════════════
# PLOTTING
# ═══════════════════════════════════════════════════════════════════════════

C_BASE = "#7a7a7a"
C_FIRE = "#c03030"
C_RISE = "#d9772b"
C_FIT = "#1d9e75"
C_SLOW = "#9b59b6"
C_ANN = "#4a4a4a"

# Fixed order, so a given open time keeps its colour between runs.
PALETTE = ["#1f5fd0", "#eb6834", "#1baf7a", "#8e44c9", "#c9a227", "#c0392b"]

_SUP = {"-": "⁻", "0": "⁰", "1": "¹", "2": "²", "3": "³", "4": "⁴",
        "5": "⁵", "6": "⁶", "7": "⁷", "8": "⁸", "9": "⁹"}


def _sup(n):
    """Unicode superscript of an integer, e.g. -8 -> ⁻⁸."""
    return "".join(_SUP[c] for c in str(int(n)))


def axis_scale(values):
    """Pick a power-of-ten multiplier from the data's own magnitude.

    A fixed ×10⁻⁷ axis is misleading when the data is ×10⁻⁸ (an excess trace
    peaking at a few 10⁻⁸ ends up labelled in tenths under a 10⁻⁷ header, so
    the multiplier reads like a floor the data sits on). Sizing the multiplier
    to the largest visible value makes the tick numbers match what is plotted.
    Returns (scale, exponent).
    """
    vmax = max((abs(v) for v in values if v is not None), default=0.0)
    if vmax <= 0 or not math.isfinite(vmax):
        return 1e-7, -7
    exp = int(math.floor(math.log10(vmax)))
    return 10.0 ** exp, exp


def _mbar_fmt(v, _pos=None):
    """Tick label as an absolute pressure, e.g. 2.5×10⁻⁷ or 1.0×10⁻⁶."""
    if v <= 0:
        return ""
    exp = int(math.floor(math.log10(v * 1.0000001)))
    return f"{v / 10 ** exp:.1f}×10{_sup(exp)}"


def _cap_gas_str(meta):
    """Capillary geometry + gas from a capture header. Shows 'n/a' for captures
    that predate the identity stamp (fields absent), never implying a bore."""
    idd = str(meta.get("capillary_id_um", "")).strip()
    ln  = str(meta.get("capillary_length_mm", "")).strip()
    gas = str(meta.get("gas_species", "")).strip()
    if idd:
        cap = f"{idd} µm × {ln} mm" if ln else f"{idd} µm"
    else:
        cap = "capillary n/a"
    return f"{cap}  ·  {gas if gas else 'gas n/a'}"


def run_identity_str(meta):
    """Full one-line 'what produced this shot', incl. upstream pressure at fire."""
    up = str(meta.get("upstream_bar_at_fire", "")).strip()
    up_s = f"upstream {up} bar at fire" if up else "upstream n/a"
    return f"{_cap_gas_str(meta)}  ·  {up_s}"


def run_identity_for_records(records):
    """Collapse per-shot identity across many records into one caption line.
    Flags 'mixed' if limiter/gas varied; shows an upstream range when the shots
    spanned different pressures (the confound worth seeing at a glance)."""
    if not records:
        return ""
    idents = {_cap_gas_str(r["meta"]) for r in records}
    up_vals = []
    for r in records:
        up = str(r["meta"].get("upstream_bar_at_fire", "")).strip()
        if up:
            try:
                up_vals.append(float(up))
            except ValueError:
                pass
    if len(idents) != 1:
        return "mixed limiter / gas across shots — see per-shot headers"
    cg = idents.pop()
    if not up_vals:
        return f"{cg}  ·  upstream n/a"
    lo, hi = min(up_vals), max(up_vals)
    up_txt = (f"upstream {lo:.2f} bar at fire" if abs(hi - lo) < 5e-3
              else f"upstream {lo:.2f}–{hi:.2f} bar across shots")
    return f"{cg}  ·  {up_txt}"


def group_upstream_str(recs):
    """Compact upstream-pressure tag for a group's legend entry: a single value,
    or a range when the group's shots weren't at one pressure. Empty if unknown —
    so it simply doesn't appear on captures predating the stamp."""
    vals = []
    for r in recs:
        up = str(r["meta"].get("upstream_bar_at_fire", "")).strip()
        if up:
            try:
                vals.append(float(up))
            except ValueError:
                pass
    if not vals:
        return ""
    lo, hi = min(vals), max(vals)
    return (f"{lo:.2f} bar" if abs(hi - lo) < 5e-3
            else f"{lo:.2f}–{hi:.2f} bar")


def plot_single(ax, r, xlim, limit, title_extra=""):
    """Full-detail view of one capture: shaded regions and per-shot marks."""
    a, times, mbar, base_fn = r["a"], r["times"], r["mbar"], r["base_fn"]
    vis_p = [p for t, p in zip(times, mbar) if xlim[0] <= t <= xlim[1]] or mbar
    scale, exp = axis_scale(vis_p)

    if a["t10"] is not None and a["t90"] is not None:
        ax.axvspan(a["t10"], a["t90"], color=C_RISE, alpha=0.13, lw=0,
                   label=f"rise 10–90 %  ({(a['t90']-a['t10'])*1000:.0f} ms)")
    if a.get("fit"):
        ax.axvspan(a["fit"]["t0"], a["fit"]["t1"], color=C_FIT, alpha=0.13,
                   lw=0,
                   label=f"fast decay  (τ_fast = {a['tau']*1000:.1f} ms)")
    if a.get("fit_slow"):
        fs = a["fit_slow"]
        ax.axvspan(fs["t0"], fs["t1"], color=C_SLOW, alpha=0.13, lw=0,
                   label=f"slow tail  (τ_slow = {fs['tau']*1000:.0f} ms, "
                         f"r² {fs['r2']:.2f})")

    ax.plot(times, [p / scale for p in mbar], color=PALETTE[0], lw=1.4, zorder=3)
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

    ax.plot([a["t_peak"]], [a["peak"] / scale], "o", ms=6, color=PALETTE[0],
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

    # ── S_eff / Q_pulse summary box ──────────────────────────────────────
    # Computed in analyse() and otherwise only reaching the console; put the
    # headline throughput numbers on the figure too. Both Q routes are shown:
    # they agree only when the injection is impulsive, so seeing them side by
    # side is the cross-check.
    vol = r["volume"]
    if a.get("fit"):
        tau_line = f"τ_fast  {a['tau']*1000:.0f} ms"
        if a.get("tau_slow"):
            tau_line += f"     τ_slow  {a['tau_slow']*1000:.0f} ms"
        else:
            tau_line += "     τ_slow  not resolved"
        box = (f"{tau_line}\n"
               f"S_eff   {a['s_eff']:.1f} L/s   (V/τ_fast,  V = {vol:g} L)\n"
               f"Q_pulse {a['q_int']:.3e} mbar·L   (S_eff × ∫)\n"
               f"        {a['q_vdp']:.3e} mbar·L   (V × Δp)")
    else:
        box = (f"Q_pulse {a['q_vdp']:.3e} mbar·L   (V × Δp)\n"
               f"no decay fit — S_eff / integral route unavailable")
    ax.text(0.985, 0.04, box, transform=ax.transAxes, ha="right", va="bottom",
            fontsize=9, color="#111111", family="monospace", zorder=6,
            bbox=dict(boxstyle="round,pad=0.5", fc="white", ec=C_ANN,
                      lw=0.8, alpha=0.9))

    ax.set_xlabel("Time from valve fire (s)")
    ax.set_ylabel(f"Chamber pressure  (×10{_sup(exp)} mbar)")
    ax.grid(True, alpha=0.25)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.set_title(f"Gyger pulse — open {r['open_us']} µs — "
                 f"captured at {r['meta'].get('capture_rate_hz','?')} Hz"
                 f"{title_extra}\n{run_identity_str(r['meta'])}",
                 fontsize=12, loc="left")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)


def plot_overlay(ax, groups, stats, xlim, limit, use_excess,
                 show_individual=True, title_extra=""):
    """Overlay several captures, grouped and coloured by open time.

    Individual shots are drawn thin, the group mean bold. Annotations report
    the GROUP MEAN, since with several shots per setting no single trace is
    the answer.
    """
    # Size the axis multiplier to the data (pre-pass over visible values),
    # so an excess overlay in the 10⁻⁸ range is not labelled under a 10⁻⁷
    # header that reads like a floor.
    scan = []
    for key, recs in groups:
        for r in recs:
            if not r["detected"]:
                continue
            ys = ([p - r["base_fn"](t) for t, p in zip(r["times"], r["mbar"])]
                  if use_excess else r["mbar"])
            scan += [y for t, y in zip(r["times"], ys) if xlim[0] <= t <= xlim[1]]
    scale, exp = axis_scale(scan)
    kind = "Excess above baseline" if use_excess else "Chamber pressure"
    ylabel = f"{kind}  (×10{_sup(exp)} mbar)"

    lo_v, hi_v = [], []
    for gi, (key, recs) in enumerate(groups):
        colour = PALETTE[gi % len(PALETTE)]
        g = stats[key]

        if show_individual:
            for r in recs:
                if not r["detected"]:
                    continue
                ys = ([p - r["base_fn"](t) for t, p in zip(r["times"], r["mbar"])]
                      if use_excess else r["mbar"])
                ax.plot(r["times"], [y / scale for y in ys], color=colour,
                        lw=0.8, alpha=0.40, zorder=2)
                lo_v += [y / scale for t, y in zip(r["times"], ys)
                         if xlim[0] <= t <= xlim[1]]

        grid, mean = resample_mean(recs, use_excess)
        _up = group_upstream_str(recs)
        label = f"{key} µs  (n = {g['n_good']})" + (f"  @ {_up}" if _up else "")
        if g.get("dp") and g["dp"][0] is not None:
            m, sd, n = g["dp"]
            label += (f"   Δp = {m:.3e}" +
                      (f" ± {sd:.1e}" if sd is not None else ""))
        if grid:
            ax.plot(grid, [y / scale for y in mean], color=colour, lw=2.2,
                    zorder=4, label=label)
            hi_v += [y / scale for t, y in zip(grid, mean)
                     if xlim[0] <= t <= xlim[1]]
        else:
            r = next((x for x in recs if x["detected"]), None)
            if r is None:
                continue
            ys = ([p - r["base_fn"](t) for t, p in zip(r["times"], r["mbar"])]
                  if use_excess else r["mbar"])
            ax.plot(r["times"], [y / scale for y in ys], color=colour, lw=2.2,
                    zorder=4, label=label)
            hi_v += [y / scale for t, y in zip(r["times"], ys)
                     if xlim[0] <= t <= xlim[1]]

    ax.set_xlim(*xlim)
    allv = (lo_v + hi_v) or [0.0, 1.0]
    lo, hi = min(allv), max(allv)
    pad = 0.10 * (hi - lo) if hi > lo else 0.1
    ax.set_ylim(lo - pad, hi + 2.2 * pad)
    span = ax.get_ylim()[1] - ax.get_ylim()[0]

    if use_excess:
        ax.axhline(0.0, color=C_BASE, ls="--", lw=1.0, zorder=1)
        # The zero line IS the baseline. In excess mode that is not zero
        # pressure — the trace rests on a real background of ~10⁻⁷ mbar — so
        # label the line with its absolute value, otherwise "0" under the axis
        # multiplier reads as the pressure itself.
        bl_vals = [r["base_fn"](0.0) for _, recs in groups for r in recs
                   if r["detected"]]
        if bl_vals:
            bl_mean = sum(bl_vals) / len(bl_vals)
            spread = (max(bl_vals) - min(bl_vals)) if len(bl_vals) > 1 else 0.0
            note = f"0 = baseline  {bl_mean:.3e} mbar"
            if spread > 0.02 * bl_mean:
                note += f"  (shots vary ±{spread/2:.1e})"
            ax.annotate(note, xy=(0.985, 0.0),
                        xycoords=("axes fraction", "data"),
                        xytext=(0, 5), textcoords="offset points",
                        color=C_BASE, fontsize=9, ha="right", va="bottom",
                        zorder=5,
                        bbox=dict(boxstyle="round,pad=0.3", fc="white",
                                  ec="none", alpha=0.85))
    ax.axvline(0.0, color=C_FIRE, lw=1.4, zorder=3)
    ax.annotate("valve fires  (t = 0)", xy=(0, 0.06),
                xycoords=("data", "axes fraction"),
                xytext=(7, 0), textcoords="offset points",
                color=C_FIRE, fontsize=10)

    # mean peak marker and label per group
    for gi, (key, recs) in enumerate(groups):
        g = stats[key]
        if not g.get("dp") or g["dp"][0] is None:
            continue
        colour = PALETTE[gi % len(PALETTE)]
        tp = g["t_peak"][0]
        yv = (g["dp"][0] if use_excess else g["peak"][0]) / scale
        ax.plot([tp], [yv], "o", ms=7, color=colour, mec="white", mew=1.5,
                zorder=6)
        parts = [f"mean Δp {g['dp'][0]:.3e}"]
        if g["dp"][1] is not None:
            parts[0] += f" ± {g['dp'][1]:.1e}"
        parts.append(f"t_peak {g['t_peak'][0]*1000:.0f} ms")
        if g.get("tau") and g["tau"][0] is not None:
            parts.append(f"τ_fast {g['tau'][0]*1000:.0f} ms")
        if g.get("tau_slow") and g["tau_slow"][0] is not None:
            parts.append(f"τ_slow {g['tau_slow'][0]*1000:.0f} ms")
        if g.get("s_eff") and g["s_eff"][0] is not None:
            parts.append(f"S_eff {g['s_eff'][0]:.0f} L/s")
        if g.get("q_int") and g["q_int"][0] is not None:
            parts.append(f"Q_pulse {g['q_int'][0]:.2e} mbar·L")
        ax.annotate("\n".join(parts), xy=(tp, yv),
                    xytext=(18, 6 + 4 * gi), textcoords="offset points",
                    fontsize=9, color=colour,
                    arrowprops=dict(arrowstyle="-", color=colour, lw=0.8,
                                    alpha=0.6))

    ax.set_xlabel("Time from valve fire (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.25)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    n_all = sum(len(r) for _, r in groups)
    _ident = run_identity_for_records([r for _, recs in groups for r in recs])
    ax.set_title(f"Gyger pulse captures — {n_all} shots in "
                 f"{len(groups)} open-time groups{title_extra}\n{_ident}",
                 fontsize=12, loc="left")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)


def plot_logpanel(ax, groups, stats, use_excess):
    """Group-mean excess on a log axis — the view the decay fit works on."""
    for gi, (key, recs) in enumerate(groups):
        colour = PALETTE[gi % len(PALETTE)]
        grid, mean = resample_mean(recs, True)
        if grid is None:
            r = next((x for x in recs if x["detected"]), None)
            if r is None:
                continue
            grid = r["times"]
            mean = [p - r["base_fn"](t) for t, p in zip(r["times"], r["mbar"])]
        floor = FIT_NOISE_K * max(r["sd"] for r in recs)
        pts = [(t, y) for t, y in zip(grid, mean) if t >= 0 and y > floor]
        if len(pts) < 5:
            continue
        g = stats[key]
        lab = f"{key} µs"
        if g.get("tau") and g["tau"][0] is not None:
            lab += f"   τ_fast = {g['tau'][0]*1000:.1f} ms"
            if g["tau"][1] is not None:
                lab += f" ± {g['tau'][1]*1000:.1f}"
        if g.get("tau_slow") and g["tau_slow"][0] is not None:
            lab += f"   τ_slow = {g['tau_slow'][0]*1000:.0f} ms"
        ax.semilogy([t for t, _ in pts], [y for _, y in pts],
                    color=colour, lw=1.6, label=lab)
        ax.axhline(floor, color=C_BASE, ls=":", lw=0.8)

    ax.set_xlabel("Time from valve fire (s)")
    ax.set_ylabel("Excess above baseline (mbar)")
    ax.grid(True, which="both", alpha=0.25)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.set_title("Group means on a log axis — a single exponential is a "
                 "straight line", fontsize=12, loc="left")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)


def plot_logabs(ax, groups, stats, xlim, limit, title_extra=""):
    """Absolute chamber pressure on a log y-axis, from baseline to the limit.

    This is the headroom view. Unlike the excess plot, the y-axis reads TRUE
    pressure: the baseline sits at its own value (~10⁻⁷ mbar) where the excess
    plot's zero used to be, and the 10⁻⁶ limit is drawn in, so how much margin
    a pulse leaves is read straight off the axis. Note the decay is NOT a
    straight line here — that is the excess-log view (--logfit); this view is
    about absolute level and margin to the limit, not τ.
    """
    bl_vals = [r["base_fn"](0.0) for _, recs in groups
               for r in recs if r["detected"]]
    bl = sum(bl_vals) / len(bl_vals) if bl_vals else 3.0e-7
    t0, t1 = xlim
    vis = []                     # every plotted value in view, for a tight fit

    for gi, (key, recs) in enumerate(groups):
        colour = PALETTE[gi % len(PALETTE)]
        g = stats[key]
        for r in recs:
            if r["detected"]:
                ax.plot(r["times"], r["mbar"], color=colour, lw=0.8,
                        alpha=0.35, zorder=2)
                vis.extend(p for tt, p in zip(r["times"], r["mbar"])
                           if t0 <= tt <= t1)
        grid, mean = resample_mean(recs, use_excess=False)
        _up = group_upstream_str(recs)
        lbl = f"{key} µs  (n = {g['n_good']})" + (f"  @ {_up}" if _up else "")
        if grid:
            ax.plot(grid, mean, color=colour, lw=2.0, zorder=4, label=lbl)
        else:
            r = next((x for x in recs if x["detected"]), None)
            if r is not None:
                ax.plot(r["times"], r["mbar"], color=colour, lw=2.0,
                        zorder=4, label=lbl)
        if g.get("peak") and g["peak"][0] is not None:
            pk, tp = g["peak"][0], g["t_peak"][0]
            ax.plot([tp], [pk], "o", ms=7, color=colour, mec="white",
                    mew=1.5, zorder=6)
            # Group-mean stats, worked out across the selected shots (mean ± sd
            # where there is more than one). Placed in the empty upper area so
            # the tightened axis doesn't crop it; stacked/coloured per group.
            parts = [f"peak {pk:.3e} mbar  ({100*pk/limit:.0f}% of limit)"]
            if g.get("dp") and g["dp"][0] is not None:
                s = f"mean Δp {g['dp'][0]:.3e}"
                if g["dp"][1] is not None:
                    s += f" ± {g['dp'][1]:.1e}"
                parts.append(s)
            if g.get("t_peak") and g["t_peak"][0] is not None:
                parts.append(f"t_peak {g['t_peak'][0]*1000:.0f} ms")
            if g.get("tau") and g["tau"][0] is not None:
                parts.append(f"τ_fast {g['tau'][0]*1000:.0f} ms")
            if g.get("tau_slow") and g["tau_slow"][0] is not None:
                parts.append(f"τ_slow {g['tau_slow'][0]*1000:.0f} ms")
            if g.get("s_eff") and g["s_eff"][0] is not None:
                parts.append(f"S_eff {g['s_eff'][0]:.0f} L/s")
            if g.get("q_int") and g["q_int"][0] is not None:
                parts.append(f"Q_pulse {g['q_int'][0]:.2e} mbar·L")
            ax.annotate("\n".join(parts), xy=(tp, pk),
                        xytext=(12, -2 - 14 * gi), textcoords="offset points",
                        va="top", ha="left",
                        fontsize=9, color=colour, zorder=7,
                        bbox=dict(boxstyle="round,pad=0.4", fc="white",
                                  ec=colour, lw=0.8, alpha=0.9))

    # Tight y-range: hug the data — just under the noise floor to just over the
    # tallest peak — instead of padding out to fixed multiples of the baseline.
    if vis:
        ylo, yhi = min(vis) / 1.04, max(vis) * 1.06
    else:
        ylo, yhi = bl * 0.9, bl * 1.5

    ax.set_yscale("log")
    ax.set_xlim(*xlim)
    ax.set_ylim(ylo, yhi)

    # limit line only if it falls inside the (now zoomed) range
    if ylo <= limit <= yhi:
        ax.axhline(limit, color=C_FIRE, ls="-", lw=1.2, alpha=0.65, zorder=3)
        ax.annotate(f"limit  {limit:.0e} mbar", xy=(0.015, limit),
                    xycoords=("axes fraction", "data"), xytext=(0, -12),
                    textcoords="offset points", color=C_FIRE, fontsize=9,
                    va="top")
    ax.axhline(bl, color=C_BASE, ls="--", lw=1.1, zorder=3)
    ax.annotate(f"baseline  {bl:.3e} mbar", xy=(0.015, bl),
                xycoords=("axes fraction", "data"), xytext=(0, 4),
                textcoords="offset points", color=C_BASE, fontsize=9,
                va="bottom")
    ax.axvline(0.0, color=C_FIRE, lw=1.4, zorder=3)
    ax.annotate("valve fires  (t = 0)", xy=(0, 0.02),
                xycoords=("data", "axes fraction"), xytext=(7, 0),
                textcoords="offset points", color=C_FIRE, fontsize=10)

    # ticks: nice values in range; dense enough for a tight span, thinned wide
    mant = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2,
            3.4, 3.6, 3.8, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0, 8.0, 9.0]
    cand = sorted(m * 10 ** e for e in (-8, -7, -6, -5) for m in mant)
    ticks = [t for t in cand if ylo <= t <= yhi]
    if len(ticks) > 9:
        ticks = ticks[::(len(ticks) // 8 + 1)]
    if ylo <= limit <= yhi and limit not in ticks:
        ticks.append(limit)
    ax.set_yticks(sorted(ticks))
    ax.yaxis.set_major_formatter(FuncFormatter(_mbar_fmt))
    ax.yaxis.set_minor_formatter(NullFormatter())

    ax.set_xlabel("Time from valve fire (s)")
    ax.set_ylabel("Chamber pressure (mbar)")
    ax.grid(True, which="both", alpha=0.2)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    n_all = sum(len(r) for _, r in groups)
    _ident = run_identity_for_records([r for _, recs in groups for r in recs])
    ax.set_title(f"Gyger pulse — absolute pressure — "
                 f"{n_all} shot{'' if n_all == 1 else 's'}{title_extra}\n{_ident}",
                 fontsize=12, loc="left")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)


# ═══════════════════════════════════════════════════════════════════════════
# REPORTING
# ═══════════════════════════════════════════════════════════════════════════

def report_single(r, args):
    a, bl, sd = r["a"], r["bl"], r["sd"]
    dt = r["times"][1] - r["times"][0]
    print(f"  samples    {len(r['times'])}  from {r['times'][0]:+.2f} to "
          f"{r['times'][-1]:+.2f} s ({1/dt:.0f} Hz)")
    print(f"  open time  {r['open_us']} µs")
    print(f"  scatter    {sd:.2e} mbar per point")
    print()
    print("Baseline estimators, value at t = 0:")
    for key in ("drift", "mean", "all"):
        b = bl[key]
        se = sd / math.sqrt(b["n"])
        extra = ""
        if key == "drift":
            se *= 2
            extra = f", drift {b['slope']:+.2e} mbar/s"
        mark = "  <-- used" if key == args.baseline else ""
        print(f"  {key:6s} {b['fn'](0.0):.5e}  ± ~{se:.1e}   "
              f"[{b['label']}{extra}]{mark}")
    vals = [bl[k]["fn"](0.0) for k in ("drift", "mean", "all")]
    print(f"  spread between estimators: {max(vals)-min(vals):.2e} mbar")

    print()
    if not r["detected"]:
        print("*** NO PULSE DETECTED in this capture ***")
        print(f"  peak excess {a['dp']:.2e} is only {a['dp']/sd:.1f} × the "
              f"baseline scatter")
        print("  the numbers below describe noise, not a pulse")
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
        print(f"tau_fast     {a['tau']*1000:.1f} ms  r² {f['r2']:.4f}  "
              f"{f['n']} pts, fit from {f['t0']*1000:.0f} to "
              f"{f['t1']*1000:.0f} ms")
        print(f"  half-life  {a['tau']*math.log(2)*1000:.0f} ms   "
              f"back to baseline (5 τ_fast) in {a['tau']*5*1000:.0f} ms")
        if a.get("fit_slow"):
            fs = a["fit_slow"]
            print(f"tau_slow     {fs['tau']*1000:.0f} ms  r² {fs['r2']:.4f}  "
                  f"{fs['n']} pts, fit from {fs['t0']*1000:.0f} to "
                  f"{fs['t1']*1000:.0f} ms")
            print(f"  ratio      τ_slow/τ_fast = {fs['tau']/a['tau']:.1f}   "
                  f"(hypothesis: capillary emptying)")
            if args.baseline == "drift":
                print("  NOTE: τ_slow on the drift baseline is fragile — its "
                      "far-tail anchor")
                print("  assumes the pulse is gone there, but a real slow tail "
                      "is not, so drift")
                print("  can subtract part of it. Cross-check with --baseline "
                      "mean.")
        else:
            print("tau_slow     not resolved — the tail is at or below the "
                  "noise floor")
            print("  (need a bigger Δp, more shots averaged, or a longer "
                  "capture to pull")
            print("  it clear of the noise; --baseline mean helps too)")
        print(f"S_eff        {a['s_eff']:.1f} L/s  (= V / τ_fast, "
              f"V = {r['volume']:g} L)")
        print(f"Q_pulse      {a['q_int']:.4e} mbar·L  = S_eff × integral")
        print(f"             {a['q_vdp']:.4e} mbar·L  = V × Δp")
        ratio = a["q_vdp"] / a["q_int"] if a["q_int"] else float("nan")
        print(f"  ratio      {ratio:.3f}  (V cancels here: this compares "
              f"SHAPE only)")
        if a["t90"] is not None and a["t10"] is not None:
            tr = a["t90"] - a["t10"]
            if tr > 0.15 * a["tau"]:
                print(f"  the rise is {100*tr/a['tau']:.0f} % of τ, so the "
                      f"injection")
                print("  is not impulsive: V × Δp under-reads. Use the "
                      "integral route.")
        if a["tau"] * 5 > bl["_tail_start"]:
            print(f"  WARNING: 5 τ_fast ({a['tau']*5:.2f} s) reaches past the "
                  f"far-tail")
            print(f"  anchor at {bl['_tail_start']:.2f} s, so the drift "
                  f"baseline may")
            print("  be contaminated by the pulse. Re-run with --baseline mean.")
    else:
        print("\nNo usable decay fit: the peak excess is too close to the "
              "noise floor.")


def report_groups(groups, stats, args, volume):
    print()
    print(f"{'open':>7} {'n':>7} {'Δp (mbar)':>24} {'t_peak (ms)':>16} "
          f"{'rise (ms)':>16}")
    for key, recs in groups:
        g = stats[key]
        n = f"{g['n_good']}/{g['n_total']}"
        if not g.get("dp"):
            print(f"{key:>5} µs {n:>7}   no detected pulses")
            continue
        print(f"{key:>5} µs {n:>7} {fmt_sci(*g['dp']):>24} "
              f"{fmt_ms(*g['t_peak']):>16} {fmt_ms(*g['rise']):>16}")

    print()
    print(f"{'open':>7} {'tau_fast (ms)':>18} {'tau_slow (ms)':>16} "
          f"{'S_eff (L/s)':>16} {'Q_pulse (mbar·L)':>26}")
    for key, recs in groups:
        g = stats[key]
        if not g.get("tau") or g["tau"][0] is None:
            print(f"{key:>5} µs {'—':>18}")
            continue
        m, sd, n = g["s_eff"]
        s_eff = f"{m:.1f}" + (f" ± {sd:.1f}" if sd is not None else "")
        tau_slow = (fmt_ms(*g["tau_slow"])
                    if g.get("tau_slow") and g["tau_slow"][0] is not None
                    else "not resolved")
        print(f"{key:>5} µs {fmt_ms(*g['tau']):>18} {tau_slow:>16} "
              f"{s_eff:>16} {fmt_sci(*g['q_int']):>26}")
    print(f"\n  (S_eff and Q_pulse assume V = {volume:g} L;  S_eff = V/τ_fast)")

    for key, recs in groups:
        g = stats[key]
        if g["n_bad"]:
            print(f"\n  {key} µs: {g['n_bad']} of {g['n_total']} shots had no "
                  f"detectable pulse — excluded above")
            for r in recs:
                if not r["detected"]:
                    print(f"    {r['name']}")

    # τ_fast is a chamber property: it should not depend on the open time.
    taus = [(k, stats[k]["tau"][0]) for k, _ in groups
            if stats[k].get("tau") and stats[k]["tau"][0] is not None]
    if len(taus) > 1:
        lo = min(t for _, t in taus)
        hi = max(t for _, t in taus)
        if hi / lo > 1.15:
            print(f"\n  NOTE: τ_fast varies from {lo*1000:.0f} to "
                  f"{hi*1000:.0f} ms between groups.")
            print("  τ_fast is a property of the chamber, not the pulse, so "
                  "this is")
            print("  the fit degrading on the smaller shots rather than a real")
            print("  change. Take τ from the largest pulses.")


# ═══════════════════════════════════════════════════════════════════════════
# TIME WINDOW
# ═══════════════════════════════════════════════════════════════════════════

def auto_xlim(records, left=-0.10):
    """Right-hand limit that follows each trace out to its return to baseline.

    The capture stores several seconds of tail, but a fixed window throws it
    away and can stop before the pulse has come back — which is exactly what a
    hard 0.7 s cut does to a shot whose 5 τ runs to ~0.8 s. This walks each
    DETECTED trace to the last point still clearly above baseline noise, adds a
    margin (and covers 5 τ where a decay was fit so the whole pump-down shows),
    and takes the widest limit across all traces. Dead flat tail beyond that is
    trimmed so the pulse itself is never squashed into the corner.

    With no detected pulse to size against, the whole captured window is shown.
    """
    rights = []
    for r in records:
        if not r.get("detected", True):
            continue
        a, sd = r["a"], r["sd"]
        post_t, post_e = a.get("post_t") or [], a.get("post_e") or []
        if len(post_t) < 2:
            continue
        # Find the return to baseline on a SMOOTHED trace: over a 5 s tail a
        # per-sample threshold is tripped by isolated noise spikes and never
        # trims anything. Average the excess in ~50 ms blocks and test each
        # block mean against RETURN_K standard errors — a sustained decay stays
        # above, a lone spike averages away.
        dt = post_t[1] - post_t[0]
        w = max(1, int(round(0.05 / dt)))
        thr = RETURN_K * sd / math.sqrt(w)
        last_above = 0.0
        for b in range(len(post_e) // w):
            seg = post_e[b * w:(b + 1) * w]
            if sum(seg) / len(seg) > thr:
                last_above = post_t[b * w + w // 2]
        cover = 5.0 * a["tau"] if a.get("fit") else 0.0
        margin = max(TAIL_MARGIN_S, a["tau"]) if a.get("fit") else TAIL_MARGIN_S
        rights.append(min(post_t[-1], max(last_above, cover) + margin))

    ends = [r["times"][-1] for r in records if r["times"]]
    if not rights:                          # nothing detected — show it all
        return (left, max(ends) if ends else 0.70)
    right = min(max(ends), max(rights))     # never past the recorded data
    right = max(right, 0.30)                # keep a sensible minimum span
    return (left, right)


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="Plot and compare PULSE-VALVE-DRIVER captures")
    ap.add_argument("csv", nargs="*", default=None,
                    help="pulse_*.csv files (if omitted, a picker opens)")
    ap.add_argument("--volume", type=float, default=None,
                    help="chamber volume in litres (default: value in the file)")
    ap.add_argument("--limit", type=float, default=1.0e-6,
                    help="in-chamber pressure limit, mbar (default 1e-6)")
    ap.add_argument("--baseline", choices=("drift", "mean", "all"),
                    default="drift")
    ap.add_argument("--baseline-n", type=int, default=125,
                    help="samples used by --baseline mean (default 125)")
    ap.add_argument("--xlim", type=float, nargs=2, default=None,
                    metavar=("T0", "T1"),
                    help="time axis limits in seconds "
                         "(default: auto, follows the return to baseline)")
    ap.add_argument("--absolute", action="store_true",
                    help="plot raw chamber pressure instead of excess")
    ap.add_argument("--no-individual", action="store_true",
                    help="draw only the group-mean traces")
    ap.add_argument("--logfit", action="store_true",
                    help="add a log-axis panel of the decay")
    ap.add_argument("--linear", action="store_true",
                    help="use the linear excess/absolute view instead of the "
                         "default log-pressure axis")
    ap.add_argument("--out", default=None,
                    help="save to file instead of showing interactively")
    args = ap.parse_args()

    paths = expand_paths(args.csv) if args.csv else choose_csv_files()
    print(f"Loading {len(paths)} capture(s):")
    records = []
    for p in paths:
        print(f"  {os.path.basename(p)}")
        records.append(load_one(p, args))
    volume = records[0]["volume"]

    single = len(records) == 1
    xlim = tuple(args.xlim) if args.xlim else auto_xlim(records)

    if not args.linear:
        # DEFAULT: absolute pressure on a log axis, baseline → limit.
        if single:
            print()
            report_single(records[0], args)
        else:
            groups = group_by_open_time(records)
            stats = {k: group_stats(v) for k, v in groups}
            report_groups(groups, stats, args, volume)
        groups = group_by_open_time(records)
        stats = {k: group_stats(v) for k, v in groups}
        fig, ax = plt.subplots(1, 1, figsize=(13, 6.5))
        plot_logabs(ax, groups, stats, xlim, args.limit,
                    title_extra=f" — V = {volume:g} L")
    elif single:
        r = records[0]
        print()
        report_single(r, args)
        fig, axes = plt.subplots(2 if args.logfit and r["a"].get("fit") else 1,
                                 1, figsize=(13, 10 if args.logfit else 6.5))
        axes = axes if isinstance(axes, (list, tuple)) or hasattr(axes, "__len__") \
            else [axes]
        plot_single(axes[0], r, xlim, args.limit,
                    title_extra=f" — V = {volume:g} L")
        if len(axes) > 1:
            groups = group_by_open_time(records)
            stats = {k: group_stats(v) for k, v in groups}
            plot_logpanel(axes[1], groups, stats, not args.absolute)
    else:
        groups = group_by_open_time(records)
        stats = {k: group_stats(v) for k, v in groups}
        report_groups(groups, stats, args, volume)

        n_panels = 2 if args.logfit else 1
        fig, axes = plt.subplots(n_panels, 1,
                                 figsize=(13, 10 if args.logfit else 6.5))
        axes = axes if n_panels > 1 else [axes]
        plot_overlay(axes[0], groups, stats, xlim, args.limit,
                     use_excess=not args.absolute,
                     show_individual=not args.no_individual,
                     title_extra=f" — V = {volume:g} L")
        if n_panels > 1:
            plot_logpanel(axes[1], groups, stats, not args.absolute)

    fig.subplots_adjust(left=0.09, right=0.97, top=0.94, bottom=0.08,
                        hspace=0.30)

    if args.out:
        fig.savefig(args.out, dpi=150, bbox_inches="tight")
        print(f"\nSaved: {args.out}")
    else:
        try:
            plt.show()
        except Exception:
            fallback = os.path.splitext(paths[0])[0] + "_overlay.png"
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
