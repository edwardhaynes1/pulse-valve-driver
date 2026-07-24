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
  • Rise (10–90 %) and decay-fit regions shaded

Several captures:
  • All traces overlaid on a common time axis, aligned at the fire
  • Grouped and coloured by valve open time, with a bold group-mean trace
  • Annotations show the GROUP MEAN peak and time-to-peak, not one shot
  • Per-group statistics table: dp, t_peak, rise, tau, S_eff, Q_pulse,
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
  --xlim T0 T1      time axis limits in seconds (default -0.1 0.7)
  --absolute        plot raw chamber pressure instead of excess
  --no-individual   draw only the group-mean traces
  --logfit          add a log-axis panel of the decay
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

GUARD_S = 0.05          # ignore this much just before the fire
TAIL_FRAC = 0.75        # far-tail anchor starts this far through the window
FIT_NOISE_K = 5.0       # stop the decay fit at this × the residual scatter
FIT_START_FRACTIONS = (0.7, 0.5, 0.35, 0.25, 0.15)
FIT_MIN_POINTS = 20


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
# LOADING AND GROUPING
# ═══════════════════════════════════════════════════════════════════════════

def load_one(path, args):
    """Read one capture, analyse it, and return a record dict."""
    meta, times, mbar = read_capture(path)
    volume = args.volume if args.volume is not None else float(
        meta.get("chamber_volume_l", 10.0))

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
C_ANN = "#4a4a4a"

# Fixed order, so a given open time keeps its colour between runs.
PALETTE = ["#1f5fd0", "#eb6834", "#1baf7a", "#8e44c9", "#c9a227", "#c0392b"]


def plot_single(ax, r, xlim, limit, title_extra=""):
    """Full-detail view of one capture: shaded regions and per-shot marks."""
    scale = 1e-7
    a, times, mbar, base_fn = r["a"], r["times"], r["mbar"], r["base_fn"]

    if a["t10"] is not None and a["t90"] is not None:
        ax.axvspan(a["t10"], a["t90"], color=C_RISE, alpha=0.13, lw=0,
                   label=f"rise 10–90 %  ({(a['t90']-a['t10'])*1000:.0f} ms)")
    if a.get("fit"):
        ax.axvspan(a["fit"]["t0"], a["fit"]["t1"], color=C_FIT, alpha=0.13,
                   lw=0, label=f"decay fit  (τ = {a['tau']*1000:.1f} ms)")

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

    ax.set_xlabel("Time from valve fire (s)")
    ax.set_ylabel("Chamber pressure  (×10⁻⁷ mbar)")
    ax.grid(True, alpha=0.25)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.set_title(f"Gyger pulse — open {r['open_us']} µs — "
                 f"captured at {r['meta'].get('capture_rate_hz','?')} Hz"
                 f"{title_extra}", fontsize=12, loc="left")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)


def plot_overlay(ax, groups, stats, xlim, limit, use_excess,
                 show_individual=True, title_extra=""):
    """Overlay several captures, grouped and coloured by open time.

    Individual shots are drawn thin, the group mean bold. Annotations report
    the GROUP MEAN, since with several shots per setting no single trace is
    the answer.
    """
    scale = 1e-7
    ylabel = ("Excess above baseline  (×10⁻⁷ mbar)" if use_excess
              else "Chamber pressure  (×10⁻⁷ mbar)")

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
        label = f"{key} µs  (n = {g['n_good']})"
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
            parts.append(f"τ {g['tau'][0]*1000:.0f} ms")
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
    ax.set_title(f"Gyger pulse captures — {n_all} shots in "
                 f"{len(groups)} open-time groups{title_extra}",
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
            lab += f"   τ = {g['tau'][0]*1000:.1f} ms"
            if g["tau"][1] is not None:
                lab += f" ± {g['tau'][1]*1000:.1f}"
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
        print(f"tau          {a['tau']*1000:.1f} ms  r² {f['r2']:.4f}  "
              f"{f['n']} pts, fit from {f['t0']*1000:.0f} to "
              f"{f['t1']*1000:.0f} ms")
        print(f"  half-life  {a['tau']*math.log(2)*1000:.0f} ms   "
              f"back to baseline (5 τ) in {a['tau']*5*1000:.0f} ms")
        print(f"S_eff        {a['s_eff']:.1f} L/s  (= V / τ, V = {r['volume']:g} L)")
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
            print(f"  WARNING: 5 τ ({a['tau']*5:.2f} s) reaches past the "
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
    print(f"{'open':>7} {'tau (ms)':>18} {'S_eff (L/s)':>18} "
          f"{'Q_pulse (mbar·L)':>26}")
    for key, recs in groups:
        g = stats[key]
        if not g.get("tau") or g["tau"][0] is None:
            print(f"{key:>5} µs {'—':>18}")
            continue
        m, sd, n = g["s_eff"]
        s_eff = f"{m:.1f}" + (f" ± {sd:.1f}" if sd is not None else "")
        print(f"{key:>5} µs {fmt_ms(*g['tau']):>18} {s_eff:>18} "
              f"{fmt_sci(*g['q_int']):>26}")
    print(f"\n  (S_eff and Q_pulse assume V = {volume:g} L)")

    for key, recs in groups:
        g = stats[key]
        if g["n_bad"]:
            print(f"\n  {key} µs: {g['n_bad']} of {g['n_total']} shots had no "
                  f"detectable pulse — excluded above")
            for r in recs:
                if not r["detected"]:
                    print(f"    {r['name']}")

    # τ is a chamber property: it should not depend on the open time.
    taus = [(k, stats[k]["tau"][0]) for k, _ in groups
            if stats[k].get("tau") and stats[k]["tau"][0] is not None]
    if len(taus) > 1:
        lo = min(t for _, t in taus)
        hi = max(t for _, t in taus)
        if hi / lo > 1.15:
            print(f"\n  NOTE: τ varies from {lo*1000:.0f} to {hi*1000:.0f} ms "
                  f"between groups.")
            print("  τ is a property of the chamber, not the pulse, so this is")
            print("  the fit degrading on the smaller shots rather than a real")
            print("  change. Take τ from the largest pulses.")


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
    ap.add_argument("--xlim", type=float, nargs=2, default=(-0.10, 0.70),
                    metavar=("T0", "T1"),
                    help="time axis limits in seconds (default -0.1 0.7)")
    ap.add_argument("--absolute", action="store_true",
                    help="plot raw chamber pressure instead of excess")
    ap.add_argument("--no-individual", action="store_true",
                    help="draw only the group-mean traces")
    ap.add_argument("--logfit", action="store_true",
                    help="add a log-axis panel of the decay")
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

    if single:
        r = records[0]
        print()
        report_single(r, args)
        fig, axes = plt.subplots(2 if args.logfit and r["a"].get("fit") else 1,
                                 1, figsize=(13, 10 if args.logfit else 6.5))
        axes = axes if isinstance(axes, (list, tuple)) or hasattr(axes, "__len__") \
            else [axes]
        plot_single(axes[0], r, tuple(args.xlim), args.limit,
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
        plot_overlay(axes[0], groups, stats, tuple(args.xlim), args.limit,
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
