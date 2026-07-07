#!/usr/bin/env python3
"""
plot_pulse_log.py

Reads a pvd-sensor_*.csv log file (from PULSE-VALVE-DRIVER / GYGER-DRIVER)
and produces two diagnostic plots:

  1. <stem>_overview.png
     3-panel time series over the full run:
       - upstream (Keller) pressure
       - vacuum chamber pressure
       - cumulative pulse count
     with all pulse events marked.

  2. <stem>_burst_zoom.png
     2-panel zoom on the densest pulse burst in the file:
       - upstream pressure with individual pulses marked
       - vacuum chamber pressure with individual pulses marked
     Includes the measured pressure drop per pulse over that burst.

Usage:
    python plot_pulse_log.py [path_to_csv]

If no path is given, you'll be prompted for one.
Output PNGs are written next to the input CSV.
"""

import sys
from pathlib import Path

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as mdates


def get_csv_path() -> Path:
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
        if not path.exists():
            sys.exit(f"File not found: {path}")
        return path

    # No path given on the command line -> open a file picker dialog
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()
    root.attributes("-topmost", True)

    selected = filedialog.askopenfilename(
        title="Select pulse-valve driver CSV log",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
    )
    root.destroy()

    if not selected:
        sys.exit("No file selected.")

    return Path(selected)


def load_log(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["timestamp"] = pd.to_datetime(df["timestamp"])
    return df


def extract_pulse_times(df: pd.DataFrame) -> list[pd.Timestamp]:
    """Expand the semicolon-separated pulse_timestamps_us column into
    individual pulse event timestamps."""
    pulse_times = []
    pulse_rows = df[df["pulses_interval"] > 0]

    for _, row in pulse_rows.iterrows():
        ts_field = row.get("pulse_timestamps_us")
        if pd.notna(ts_field) and str(ts_field).strip():
            for ts in str(ts_field).split(";"):
                ts = ts.strip()
                if ts:
                    try:
                        pulse_times.append(pd.to_datetime(ts))
                    except (ValueError, TypeError):
                        pass
        else:
            # fall back to the row's own timestamp (one pulse assumed)
            pulse_times.append(row["timestamp"])

    return sorted(pulse_times)


def find_densest_burst(pulse_times: list[pd.Timestamp], gap_threshold_s: float = 2.0):
    """Split pulse_times into contiguous bursts (gaps <= gap_threshold_s)
    and return the burst with the most pulses."""
    if not pulse_times:
        return []

    bursts = [[pulse_times[0]]]
    for t in pulse_times[1:]:
        if (t - bursts[-1][-1]).total_seconds() <= gap_threshold_s:
            bursts[-1].append(t)
        else:
            bursts.append([t])

    # prefer bursts with more than one pulse; pick the largest
    multi_pulse_bursts = [b for b in bursts if len(b) > 1]
    candidates = multi_pulse_bursts if multi_pulse_bursts else bursts
    return max(candidates, key=len)


def plot_overview(df: pd.DataFrame, pulse_times: list[pd.Timestamp],
                   burst: list[pd.Timestamp], out_path: Path):
    keller = df.dropna(subset=["keller_pressure_bar"])
    vac = df.dropna(subset=["vacuum_chamber_mbar"])

    fig, axes = plt.subplots(
        3, 1, figsize=(12, 9), sharex=True,
        gridspec_kw={"height_ratios": [3, 3, 1]}
    )
    ax1, ax2, ax3 = axes

    ax1.plot(keller["timestamp"], keller["keller_pressure_bar"] * 1000,
             color="#1f4e79", lw=1.2)
    ax1.set_ylabel("Upstream pressure\n(mbar)")
    ax1.set_title(f"Pulse-valve driver log — {df['timestamp'].iloc[0]}")

    if not vac.empty:
        ax2.plot(vac["timestamp"], vac["vacuum_chamber_mbar"] * 1e3,
                 color="#a83232", lw=1.0)
    ax2.set_ylabel("Vacuum chamber\n(\u00d710\u207b\u00b3 mbar)")

    ax3.step(df["timestamp"], df["pulses_total"], where="post",
             color="black", lw=1.2)
    ax3.set_ylabel("Cumulative\npulses")
    ax3.set_xlabel("Time")

    # Mark all pulse events lightly, and the chosen burst window distinctly
    if len(burst) > 1:
        for ax in axes:
            ax.axvspan(burst[0], burst[-1], color="orange", alpha=0.15)
        ax1.text(0.99, 0.97,
                 f"{len(burst)}-pulse burst (highlighted)\nzoomed in second figure",
                 transform=ax1.transAxes, ha="right", va="top",
                 fontsize=9, color="darkorange")

    for ax in axes:
        for t in pulse_times:
            ax.axvline(t, color="orange", alpha=0.08, lw=0.6)

    ax3.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    fig.autofmt_xdate()
    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_burst_zoom(df: pd.DataFrame, burst: list[pd.Timestamp], out_path: Path):
    keller = df.dropna(subset=["keller_pressure_bar"])
    vac = df.dropna(subset=["vacuum_chamber_mbar"])

    if len(burst) < 2:
        print("No multi-pulse burst found — skipping zoom plot.")
        return

    burst_start, burst_end = burst[0], burst[-1]
    duration_s = (burst_end - burst_start).total_seconds()
    rate_hz = (len(burst) - 1) / duration_s if duration_s > 0 else float("nan")

    window_start = burst_start - pd.Timedelta(seconds=3)
    window_end = burst_end + pd.Timedelta(seconds=5)

    mk = (keller["timestamp"] >= window_start) & (keller["timestamp"] <= window_end)
    mv = (vac["timestamp"] >= window_start) & (vac["timestamp"] <= window_end)

    fig, (axa, axb) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    axa.plot(keller[mk]["timestamp"], keller[mk]["keller_pressure_bar"] * 1000,
             ".-", color="#1f4e79", ms=3, lw=0.8)
    for i, t in enumerate(burst):
        axa.axvline(t, color="darkorange", alpha=0.6, lw=1.2,
                     label="Pulse" if i == 0 else None)
    axa.legend(loc="upper right", fontsize=9, frameon=False)
    axa.set_ylabel("Upstream pressure (mbar)")

    open_times = df["open_time_us"].dropna().unique()
    open_time_str = ", ".join(str(int(v)) for v in open_times)
    axa.set_title(
        f"Burst: {len(burst)} pulses, {duration_s:.1f} s "
        f"(~{rate_hz:.1f} Hz), open_time_us = {open_time_str}"
    )

    if not vac.empty:
        axb.plot(vac[mv]["timestamp"], vac[mv]["vacuum_chamber_mbar"] * 1e3,
                 ".-", color="#a83232", ms=3, lw=0.8)
        for t in burst:
            axb.axvline(t, color="darkorange", alpha=0.6, lw=1.2)
    axb.set_ylabel("Vacuum chamber\n(\u00d710\u207b\u00b3 mbar)")
    axb.set_xlabel("Time")

    axb.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S.%f"))
    fig.autofmt_xdate()
    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    plt.close(fig)

    # Print pressure-drop summary
    pre = keller[keller["timestamp"] < burst_start]["keller_pressure_bar"]
    post = keller[keller["timestamp"] > burst_end]["keller_pressure_bar"]
    if len(pre) >= 1 and len(post) >= 1:
        p_before = pre.iloc[-min(5, len(pre)):].mean()
        p_after = post.iloc[:min(5, len(post))].mean()
        drop_mbar = (p_before - p_after) * 1000
        print(f"Pressure before burst: {p_before*1000:.3f} mbar")
        print(f"Pressure after burst:  {p_after*1000:.3f} mbar")
        print(f"Total drop over {len(burst)} pulses: {drop_mbar:.3f} mbar "
              f"({drop_mbar/len(burst)*1000:.2f} \u00b5bar/pulse)")


def main():
    csv_path = get_csv_path()
    df = load_log(csv_path)
    pulse_times = extract_pulse_times(df)
    burst = find_densest_burst(pulse_times)

    stem = csv_path.stem

    # Output goes to a sibling "plots" folder next to the CSV's folder,
    # e.g. .../pulse-valve-driver/logs/file.csv -> .../pulse-valve-driver/plots/
    # Falls back to a "plots" subfolder next to the CSV if no suitable
    # sibling directory structure is found.
    parent = csv_path.parent
    sibling_plots = parent.parent / "plots"
    if sibling_plots.is_dir():
        out_dir = sibling_plots
    else:
        out_dir = parent / "plots"
        out_dir.mkdir(exist_ok=True)

    overview_path = out_dir / f"{stem}_overview.png"
    zoom_path = out_dir / f"{stem}_burst_zoom.png"

    plot_overview(df, pulse_times, burst, overview_path)
    print(f"Wrote: {overview_path}")

    plot_burst_zoom(df, burst, zoom_path)
    if zoom_path.exists():
        print(f"Wrote: {zoom_path}")


if __name__ == "__main__":
    main()
