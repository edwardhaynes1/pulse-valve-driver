#!/usr/bin/env python3
"""
PAIRTEST_PLOTTER  —  Plot a paired-pulse test (pp) from PULSE-VALVE-DRIVER
=========================================================================
Re-analyses one pp run from its saved traces and plots Q2/Q1 (second dose /
first) against dt for every pair, with the mean and its 95 % confidence
interval at each dt, against the A expectation (Q2/Q1 = 1: the upstream
capillary is a brake, nothing is depleted).

The dose of each capture is the integral of its excess pressure, redone from
the trace (pulse_pp<id>_sNN.csv) rather than taken from the driver:
  • baseline: a straight line fitted to the pre-fire second (to 50 ms before
    the fire), so a slow drift from the previous capture is followed;
  • no clipping: noise below the baseline counts as negative, so it averages
    out instead of adding a positive bias that pulls Q2/Q1 towards 1;
  • the same window, fire to the shortest trace end, for every capture;
  • the tail beyond the window is extrapolated from an exponential fitted to
    its last second, when that tail is clean (the correction is reported).
If no traces are found, the driver's own integrals are used, with a note.

Verdict, from the shortest dt alone (no model):
  B  the second dose there is lower than the first by at least 5 %, and its
     95 % interval does not reach 1;
  A  otherwise.
The recovery Q2/Q1 = 1 - a*exp(-dt/tau) is fitted only with 6 or more dt
values, to estimate tau; it does not decide the verdict.
Results go to pvd-pairtest-reanalysed_<id>.csv next to the run's files.

The axis starts at the shortest dt the run could test (the ZC1 needs
dt >= 300 µs and dt >= open time + 100 µs); shorter dt is not plotted.
The capillary's refill time, t_fill = L² / D with D = d² p / (32 μ), comes
from the upstream capillary in the run's summary, the run's mean upstream
pressure and the gas's viscosity. It is drawn when it falls on the axis and
stated in the subtitle when it is shorter than the shortest testable dt:
depletion that refills that fast has recovered before the second pulse, so
the run cannot see it.

The run's details (capillaries, gas, open time, verdict) are read from
pvd-pairtest-summary_<id>.csv next to the chosen file, so nothing needs
editing between runs.

Usage
-----
  python PAIRTEST_PLOTTER.py                       a file picker opens
  python PAIRTEST_PLOTTER.py logs/pvd-pairtest_<id>.csv
  python PAIRTEST_PLOTTER.py ... --out plot.png    save there (default: next to the CSV)
  python PAIRTEST_PLOTTER.py ... --no-show         save only, no window

The picker starts in logs/ if there is one in the current folder. If no
dialog can open, a numbered list of pvd-pairtest_*.csv files is offered.
"""

import argparse
import csv
import glob
import math
import os
import sys

import matplotlib.pyplot as plt

# Dynamic viscosity at ~20 °C, Pa·s. Unknown gas: no refill time is drawn.
VISCOSITY_PA_S = {"N2": 1.76e-5, "AIR": 1.81e-5, "H2": 0.88e-5,
                  "HE": 1.96e-5, "AR": 2.23e-5}
ZC1_MIN_CYCLE_MS = 0.3        # ZC1 CYCLE time >= 300 µs ...
ZC1_GUARD_MS = 0.1            # ... and >= OPEN time + 100 µs

# Student t, two-sided 95 %, by number of pairs
T95 = {2: 12.71, 3: 4.30, 4: 3.18, 5: 2.78, 6: 2.57, 7: 2.45, 8: 2.36,
       9: 2.31, 10: 2.26, 12: 2.20, 15: 2.14, 20: 2.09}

INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BLUE, SURF = "#2a78d6", "#fcfcfb"
ENC = "latin-1"               # the driver's files are Windows-encoded

GUARD_S = 0.05                # baseline ends this long before the fire
TAIL_S = 1.0                  # last stretch of the window used for the tail fit
DROP_MIN = 0.05               # smallest drop at the shortest dt counted as B
FIT_MIN_DT = 6                # dt values needed before tau is fitted


# ── choosing the file ───────────────────────────────────────────────────────

def choose_file():
    """A file dialog, or a numbered list if no dialog can open."""
    start = "logs" if os.path.isdir("logs") else "."
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        path = filedialog.askopenfilename(
            title="Select a paired-pulse test to plot",
            initialdir=os.path.abspath(start),
            filetypes=[("PVD pair tests", "pvd-pairtest_*.csv"),
                       ("CSV files", "*.csv"), ("All files", "*.*")])
        root.destroy()
        if path:
            return path
        print("No file selected.")
        sys.exit(0)
    except Exception:
        pass

    candidates = sorted(glob.glob(os.path.join(start, "pvd-pairtest_*.csv")))
    if not candidates:
        print(f"No pvd-pairtest_*.csv files in {os.path.abspath(start)}.")
        print("Pass one explicitly:  python PAIRTEST_PLOTTER.py <file.csv>")
        sys.exit(1)
    print("\nSelect a paired-pulse test to plot:")
    for i, p in enumerate(candidates, 1):
        print(f"  {i:>2}. {os.path.basename(p)}")
    while True:
        c = input(f"\nEnter number (1-{len(candidates)}), or q to quit: ").strip()
        if c.lower() == "q":
            sys.exit(0)
        if c.isdigit() and 1 <= int(c) <= len(candidates):
            return candidates[int(c) - 1]
        print("  not a valid choice")


# ── reading the run ─────────────────────────────────────────────────────────

def run_id_of(path):
    name = os.path.splitext(os.path.basename(path))[0]
    return name.split("pvd-pairtest_", 1)[-1]


def read_summary(pairs_path):
    """Header values of pvd-pairtest-summary_<id>.csv next to the pairs file,
    or {} if it is not there."""
    path = os.path.join(os.path.dirname(pairs_path) or ".",
                        f"pvd-pairtest-summary_{run_id_of(pairs_path)}.csv")
    meta = {}
    if not os.path.exists(path):
        print(f"(no {os.path.basename(path)} — plotting without the run details)")
        return meta
    with open(path, encoding=ENC, newline="") as f:
        for row in csv.reader(f):
            if row and row[0].startswith("#"):
                meta[row[0].lstrip("# ").strip()] = row[1] if len(row) > 1 else ""
    return meta


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def read_pairs(path):
    with open(path, encoding=ENC, newline="") as f:
        rows = list(csv.DictReader(f))
    by_dt, ups, opens = {}, [], []
    for r in rows:
        dt, q = fnum(r.get("dt_actual_ms")), fnum(r.get("q2_over_q1"))
        if dt is None or q is None:
            continue
        by_dt.setdefault(dt, []).append(q)
        ups += [v for v in (fnum(r.get("upstream_bar_single")),
                            fnum(r.get("upstream_bar_pair"))) if v]
        if fnum(r.get("open_time_us")):
            opens.append(fnum(r.get("open_time_us")))
    if not by_dt:
        print(f"No pairs in {os.path.basename(path)}.")
        sys.exit(1)
    p_bar = sum(ups) / len(ups) if ups else None
    open_us = max(set(opens), key=opens.count) if opens else None
    return by_dt, p_bar, open_us, rows


def read_trace(path):
    """(header dict, times, mbar) of one capture; blank samples are None."""
    meta, t, p = {}, [], []
    with open(path, encoding=ENC, newline="") as f:
        for row in csv.reader(f):
            if not row:
                continue
            if row[0].startswith("#"):
                meta[row[0].lstrip("# ").strip()] = row[1] if len(row) > 1 else ""
            elif row[0] != "t_s":
                t.append(float(row[0]))
                p.append(fnum(row[1]) if len(row) > 1 else None)
    return meta, t, p


def _line(xs, ys):
    """Least-squares (slope, intercept)."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx if sxx > 0 else 0.0
    return b, my - b * mx


def trace_integral(t, p, t_end):
    """Excess-pressure integral (mbar*s) from the fire (t = 0) to t_end.

    Returns (integral, tail_extrapolated, note). The baseline is a line fitted
    to the pre-fire samples; nothing is clipped. The tail beyond t_end is
    added from an exponential fitted to the excess over the window's last
    TAIL_S, only if that excess is clearly above the noise and decaying."""
    pre = [(ti, pi) for ti, pi in zip(t, p) if ti <= -GUARD_S and pi is not None]
    if len(pre) < 20 or any(pi is None for ti, pi in zip(t, p) if 0 <= ti <= t_end):
        return None, 0.0, "trace has invalid or too few samples"
    b, a = _line([x for x, _ in pre], [y for _, y in pre])
    sd = math.sqrt(sum((y - (a + b * x)) ** 2 for x, y in pre) / (len(pre) - 2))
    post = [(ti, pi - (a + b * ti)) for ti, pi in zip(t, p) if 0 <= ti <= t_end]
    dt = (post[-1][0] - post[0][0]) / (len(post) - 1)
    integral = sum(e for _, e in post) * dt

    tail = [(ti, e) for ti, e in post if ti >= t_end - TAIL_S]
    extra, note = 0.0, ""
    if len(tail) > 10 and all(e > 3 * sd for _, e in tail):
        k, c = _line([x for x, _ in tail], [math.log(e) for _, e in tail])
        if k < 0:
            tau = -1.0 / k
            extra = math.exp(c + k * t_end) * tau
        else:
            note = "tail not decaying, not extrapolated"
    return integral + extra, extra, note


def reanalyse(pairs_path, rows):
    """Recompute every pair's Q2/Q1 from its two traces. Returns
    (by_dt, per_pair, info) or None if no traces are found."""
    folder = os.path.dirname(pairs_path) or "."
    files = {r["single_trace_file"] for r in rows} | {r["pair_trace_file"] for r in rows}
    traces = {}
    for name in files:
        path = os.path.join(folder, name)
        if os.path.exists(path):
            traces[name] = read_trace(path)
    if not traces:
        return None
    t_end = min(tr[1][-1] for tr in traces.values())
    integ, tails, notes = {}, [], []
    for name, (_, t, p) in traces.items():
        val, extra, note = trace_integral(t, p, t_end)
        integ[name] = val
        if val:
            tails.append(extra / val)
        if note:
            notes.append(f"{name}: {note}")

    by_dt, per_pair, missing = {}, [], 0
    for r in rows:
        i1, i2 = integ.get(r["single_trace_file"]), integ.get(r["pair_trace_file"])
        dt = fnum(r.get("dt_actual_ms"))
        if dt is None or not i1 or not i2 or i1 <= 0:
            missing += 1
            continue
        q = i2 / i1 - 1.0
        by_dt.setdefault(dt, []).append(q)
        per_pair.append((dt, r.get("repeat", ""), i1, i2, q, fnum(r.get("q2_over_q1"))))
    info = {"t_end": t_end, "n_traces": len(traces), "n_expected": len(files),
            "skipped_pairs": missing, "tail_mean": (sum(tails) / len(tails)) if tails else 0.0,
            "notes": notes}
    return by_dt, per_pair, info


def ci95(v):
    """(mean, sd, half-width of the 95 % confidence interval)."""
    n = len(v)
    m = sum(v) / n
    if n < 2:
        return m, 0.0, 0.0
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1))
    t = T95.get(n) or (1.96 if n > 20 else T95[min(T95, key=lambda k: abs(k - n))])
    return m, sd, t * sd / math.sqrt(n)


def verdict_from(stats):
    """A/B from the shortest dt: B if the second dose there is lower by at
    least DROP_MIN and its 95 % interval does not reach 1."""
    dt, n, m, sd, ci = stats[0]
    drop = 1.0 - m
    if m + ci < 1.0 and drop >= DROP_MIN:
        return "B", (f"B - reservoir: at the shortest dt ({dt:g} ms) the second dose is "
                     f"{100 * drop:.1f} ± {100 * ci:.1f} % lower than the first")
    if m + ci < 1.0:
        return "A", (f"A - brake: at the shortest dt ({dt:g} ms) the second dose is "
                     f"{100 * drop:.1f} ± {100 * ci:.1f} % lower, below the "
                     f"{100 * DROP_MIN:.0f} % counted as depletion")
    return "A", (f"A - brake: no drop at the shortest dt ({dt:g} ms): "
                 f"Q2/Q1 = {m:.3f} ± {ci:.3f}")


def fit_recovery(stats):
    """Q2/Q1 = 1 - a*exp(-dt/tau), weighted by 1/sd_mean², tau on a log grid.
    Returns (a, tau_ms) or None."""
    pts = [(dt, m, (sd / math.sqrt(n)) if n > 1 and sd > 0 else None)
           for dt, n, m, sd, ci in stats]
    weighted = all(se for _, _, se in pts)
    best = None
    for i in range(400):
        tau = 0.01 * (1e5 ** (i / 399))          # 0.01 ms .. 1000 ms
        sxx = sxy = 0.0
        for dt, m, se in pts:
            w = 1.0 / se ** 2 if weighted else 1.0
            x = math.exp(-dt / tau)
            sxx += w * x * x
            sxy += w * x * (1.0 - m)
        if sxx <= 0:
            continue
        a = sxy / sxx
        chi = sum((1.0 / se ** 2 if weighted else 1.0) * ((1.0 - m) - a * math.exp(-dt / tau)) ** 2
                  for dt, m, se in pts)
        if best is None or chi < best[0]:
            best = (chi, a, tau)
    return (best[1], best[2]) if best else None


def write_reanalysed(pairs_path, per_pair, stats, verdict, fit, info, used_traces):
    path = os.path.join(os.path.dirname(pairs_path) or ".",
                        f"pvd-pairtest-reanalysed_{run_id_of(pairs_path)}.csv")
    with open(path, "w", encoding=ENC, newline="") as f:
        w = csv.writer(f)
        method = ("from traces: sloped pre-fire baseline, unclipped, common window, "
                  "tail extrapolated" if used_traces else "driver integrals (no traces found)")
        for k, v in (("verdict", verdict), ("method", method),
                     ("window_end_s", f"{info.get('t_end', ''):g}" if info.get("t_end") else ""),
                     ("tail_correction_mean", f"{100 * info.get('tail_mean', 0):.2f} %"),
                     ("tau_fit", f"a = {fit[0]:.3f}, tau = {fit[1]:.3g} ms" if fit else
                      f"not fitted (needs {FIT_MIN_DT} or more dt values)")):
            w.writerow([f"# {k}", v])
        w.writerow(["dt_ms", "n_pairs", "q2_over_q1_mean", "q2_over_q1_sd", "q2_over_q1_ci95"])
        for dt, n, m, sd, ci in stats:
            w.writerow([f"{dt:g}", n, f"{m:.5f}", f"{sd:.5f}", f"{ci:.5f}"])
        w.writerow([])
        w.writerow(["dt_ms", "repeat", "integral_single_mbar_s", "integral_pair_mbar_s",
                    "q2_over_q1", "q2_over_q1_driver"])
        for dt, rep_, i1, i2, q, qd in per_pair:
            w.writerow([f"{dt:g}", rep_, f"{i1:.6e}" if i1 else "", f"{i2:.6e}" if i2 else "",
                        f"{q:.5f}", f"{qd:.5f}" if qd is not None else ""])
    return path


def refill_time_ms(d_um, l_mm, p_bar, gas):
    """t_fill = L² / D, D = d² p / (32 μ). None if anything is missing."""
    mu = VISCOSITY_PA_S.get((gas or "").strip().upper())
    if not (d_um and l_mm and p_bar and mu):
        return None
    d, L, p = d_um * 1e-6, l_mm * 1e-3, p_bar * 1e5
    return L * L / (d * d * p / (32 * mu)) * 1e3


def cap_text(d, l):
    return "none" if not d or not l else f"{d:g} µm × {l:g} mm"


# ── plotting ────────────────────────────────────────────────────────────────

def plot(pairs_path, out_path, show):
    meta = read_summary(pairs_path)
    by_dt_driver, p_bar, open_rows, rows = read_pairs(pairs_path)
    re = reanalyse(pairs_path, rows)
    if re is None:
        print("(no traces found next to the file — using the driver's integrals)")
    elif not re[0]:
        print(f"(found {re[2]['n_traces']} of {re[2]['n_expected']} traces, "
              f"no complete pair — using the driver's integrals)")
        re = None
    if re is None:
        by_dt, info = by_dt_driver, {}
        per_pair = [(fnum(r.get("dt_actual_ms")), r.get("repeat", ""),
                     fnum(r.get("integral_single_mbar_s")), fnum(r.get("integral_pair_mbar_s")),
                     fnum(r.get("q2_over_q1")), fnum(r.get("q2_over_q1")))
                    for r in rows if fnum(r.get("q2_over_q1")) is not None]
    else:
        by_dt, per_pair, info = re
        if info["n_traces"] < info["n_expected"]:
            print(f"(found {info['n_traces']} of {info['n_expected']} traces; "
                  f"{info['skipped_pairs']} pair(s) skipped)")
        for n_ in info["notes"][:5]:
            print(f"  note: {n_}")

    open_us = fnum(meta.get("open_time_us")) or open_rows
    gas = meta.get("gas_species", "")
    up_d, up_l = fnum(meta.get("upstream_cap_id_um")), fnum(meta.get("upstream_cap_length_mm"))
    dn_d, dn_l = fnum(meta.get("downstream_cap_id_um")), fnum(meta.get("downstream_cap_length_mm"))
    t_fill = refill_time_ms(up_d, up_l, p_bar, gas)
    dt_min = (max(ZC1_MIN_CYCLE_MS, open_us / 1000.0 + ZC1_GUARD_MS)
              if open_us else None)

    dts = sorted(by_dt)
    stats = [(dt, len(by_dt[dt])) + ci95(by_dt[dt]) for dt in dts]   # dt, n, m, sd, ci
    v_short, verdict = verdict_from(stats)
    fit = fit_recovery(stats) if len(dts) >= FIT_MIN_DT else None

    # The axis starts at the shortest testable dt: nothing below it can be measured.
    lo = min(dts[0], dt_min or dts[0]) / 1.6
    hi = dts[-1] * 2.0
    t_fill_on_axis = bool(t_fill and t_fill >= lo)
    allq = [q for v in by_dt.values() for q in v] + [1.0]
    span = max(allq) - min(allq)
    pad = max(0.05, 0.15 * span)
    y0, y1 = min(allq) - pad - max(0.05, 0.12 * span), max(allq) + pad

    plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2,
                         "axes.labelcolor": INK, "xtick.color": INK2,
                         "ytick.color": INK2})
    fig, ax = plt.subplots(figsize=(8, 4.8), facecolor=SURF)
    ax.set_facecolor(SURF)
    ax.set_xscale("log")
    ax.set_xlim(lo, hi)
    ax.set_ylim(y0, y1)

    if t_fill_on_axis:
        ax.axvline(t_fill, color=INK2, lw=1, ls=":", zorder=1)
        ax.text(t_fill * 1.08, y1 - 0.03 * (y1 - y0),
                f"capillary refill\ntime ≈ {t_fill:.2g} ms",
                color=INK2, fontsize=8.5, va="top")

    ax.axhline(1.0, color=INK2, lw=1.2, ls="--", zorder=1)
    # outside the plot on the right, where no data point can sit on it
    ax.annotate("A: no\ndepletion\n(Q₂ = Q₁)", xy=(1.0, 1.0),
                xycoords=("axes fraction", "data"), xytext=(6, 0),
                textcoords="offset points", color=INK2, fontsize=8.5,
                va="center", ha="left", annotation_clip=False)

    for dt in dts:
        v = by_dt[dt]
        ax.scatter([dt] * len(v), v, s=14, color=BLUE, alpha=0.35, lw=0, zorder=2)
    if fit:
        a_, tau_ = fit
        xs = [lo * (hi / lo) ** (i / 199) for i in range(200)]
        ax.plot(xs, [1 - a_ * math.exp(-x / tau_) for x in xs], color=BLUE,
                lw=1.5, alpha=0.6, zorder=1)
    for dt, n, m, sd, ci in stats:
        ax.errorbar(dt, m, yerr=ci, fmt="o", ms=8, color=BLUE, mec=SURF, mew=2,
                    elinewidth=2, capsize=4, zorder=3)
        # under its own error bar, so neighbouring dt values never crowd it
        low = min([m - ci] + by_dt[dt])
        ax.annotate(f"{m:.3f}\n± {ci:.3f}", (dt, low), xytext=(0, -5),
                    textcoords="offset points", color=INK, fontsize=8,
                    va="top", ha="center", linespacing=1.1,
                    bbox=dict(boxstyle="round,pad=0.15", fc=SURF, ec="none", alpha=0.85))

    ticks = dts                    # the refill-time line carries its own label
    ax.set_xticks(ticks)
    ax.set_xticklabels([f"{x:.2g}" if x < 1 else f"{x:g}" for x in ticks])
    ax.minorticks_off()
    ax.set_xlabel("Δt between the two pulses (ms)")
    ax.set_ylabel("Q₂ / Q₁  (second dose / first)")
    ax.grid(axis="y", color=GRID, lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)

    rid = run_id_of(pairs_path)
    title = f"Paired-pulse test {rid}"
    title += f" — verdict {verdict.split(':')[0].strip()}"
    n_pairs = sorted({s_[1] for s_ in stats})
    sub1 = " · ".join(x for x in (
        f"upstream {cap_text(up_d, up_l)}" if meta else "",
        f"downstream {cap_text(dn_d, dn_l)}" if meta else "",
        f"{gas} ≈ {p_bar:.2f} bar" if p_bar else gas,
        f"open {open_us:g} µs" if open_us else "") if x)
    sub2 = (f"{'/'.join(str(n) for n in n_pairs)} pairs per Δt: light dots are "
            f"single pairs, large dots the mean ± 95 % confidence interval")
    if re is not None:
        sub2 += (f"\nRe-analysed from traces: sloped baseline, unclipped, window 0–"
                 f"{info['t_end']:.2g} s, tail extrapolated (+{100 * info['tail_mean']:.1f} %)")
        if fit:
            sub2 += f" · fit: a = {fit[0]:.2f}, τ = {fit[1]:.2g} ms"
    else:
        sub2 += "\nDriver integrals (traces not found next to the file, not re-analysed)"
    if t_fill and not t_fill_on_axis:
        sub2 += (f"\nCapillary refill time ≈ {t_fill:.2g} ms is below the shortest testable "
                 f"Δt ({dt_min:g} ms), so capillary depletion would not show here")
    fig.text(0.02, 0.965, title, fontsize=11, color=INK, va="top")
    fig.text(0.02, 0.915, sub1 + "\n" + sub2, fontsize=8.5, color=INK2,
             va="top", linespacing=1.4)
    fig.tight_layout(rect=(0, 0, 0.92, 0.83 - 0.035 * sub2.count("\n")))

    fig.savefig(out_path, dpi=200, facecolor=SURF)
    print(f"Saved: {out_path}")
    drv = {dt: sum(v) / len(v) for dt, v in by_dt_driver.items()}
    for dt, n, m, sd, ci in stats:
        d_ = drv.get(dt)
        print(f"  dt {dt:>7g} ms   n {n}   Q2/Q1 {m:.3f} ± {ci:.3f} (95 %)"
              + (f"   [driver: {d_:.3f}]" if d_ is not None and re is not None else ""))
    if fit:
        print(f"  recovery fit: a = {fit[0]:.3f}, tau = {fit[1]:.3g} ms")
    out_csv = write_reanalysed(pairs_path, per_pair, stats, verdict, fit, info, re is not None)
    print(f"Saved: {out_csv}")
    if t_fill:
        print(f"  capillary refill time ≈ {t_fill:.2g} ms"
              + (f"; shortest dt {dts[0]:g} ms is {dts[0] / t_fill:.2g}× that"
                 if dts else ""))
    print(f"  verdict: {verdict}")
    if show:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description="Plot a PULSE-VALVE-DRIVER paired-pulse test")
    ap.add_argument("csv", nargs="?", help="pvd-pairtest_<id>.csv (omit to pick one)")
    ap.add_argument("--out", help="image to save (default: next to the CSV, same name, .png)")
    ap.add_argument("--no-show", action="store_true", help="save only, open no window")
    a = ap.parse_args()
    path = a.csv or choose_file()
    out = a.out or os.path.splitext(path)[0] + ".png"
    plot(path, out, show=not a.no_show)


if __name__ == "__main__":
    main()
