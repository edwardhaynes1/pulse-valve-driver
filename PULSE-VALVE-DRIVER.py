#!/usr/bin/env python3
"""
PULSE-VALVE-DRIVER.py
===============
Logs upstream pressure and temperature from a Keller PAA-23SX-H2 sensor,
and allows manual *and* automatic (independent) control of a ZC1 pulse valve controller.

This version presents a TWO-WINDOW GUI (Tkinter, standard library):
  • "Live Log"      — device status, live readouts, pressure strip chart,
                      pulse counters, and a scrolling event log.
  • "Valve Driver"  — fire single shots, set open time, and run high-rate
                      pulse captures. A capture streams the chamber gauge at
                      CAPTURE_RATE_HZ, fires the valve inside the window, and
                      reports the peak the gauge actually saw — the 10 Hz
                      monitoring loop cannot resolve a pulse transient and
                      under-reads the peak by an unknown amount.

Hardware
--------
  Keller PAA-23SX-H2   RS485/USB (K-114 adapter) — upstream P + T
  ZC1 controller       USB-RS232 — pulse valve control
  LabJack U3 (opt.)    vacuum chamber gauge on FIO2

CSV columns
-----------
  timestamp, keller_pressure_bar, keller_temperature_degC,
  vacuum_chamber_mbar, pulses_interval, pulses_total,
  drive_active, drive_rate_hz, open_time_us, vacuum_status
  (vacuum_chamber_mbar is blank when the gauge reading is not valid;
  vacuum_status says why)

Logging cadence
---------------
  One CSV row is written on a drift-free wall-clock cadence of
  LOG_INTERVAL_S seconds (first row written immediately at start).

Driver commands
---------------
  v                       fire one pulse (no capture)
  c [n]                   capture n pulses at high rate, with a summary after a run
  pp [dt ms ...]          paired-pulse test: is the upstream capillary a brake (A)
                          or a reservoir (B)? (default dt sweep: PAIR_DT_MS)
  pp stop                 end the paired-pulse test early and summarise
  t <µs>                  set open time
  cap <up|down> <id> <len>  record a capillary (bore µm, length mm)
  cap up none             record that no upstream capillary is fitted
  gas <species>           record the gas at the valve
  q                       quit

Capture outputs
---------------
  logs/pulse_<ts>.csv        one file per shot: full decimated trace, t = 0 at
                             fire, with peak / dp / integral / tau / S_eff / Q
                             in the header
  logs/pvd-captures_<ts>.csv one row per shot for the whole session — a manifest
                             indexing every capture (single or multi-shot, any
                             rate), so scatter / tau / regression work needs one
                             file, not dozens. Each row's trace_file points back
                             to its full per-shot trace.

Usage
-----
  python PULSE-VALVE-DRIVER.py
"""

import serial
import serial.tools.list_ports
import threading
import time
import csv
import os
import math
import random
import re
from collections import deque
from datetime import datetime, timedelta
from pathlib import Path

import tkinter as tk
from tkinter import font as tkfont

try:
    import u3
    LABJACK_AVAILABLE = True
except ImportError:
    LABJACK_AVAILABLE = False
    print("Warning: LabJackPython not installed — vacuum gauge disabled.")
    print("Install: pip install LabJackPython")

try:
    from keller_protocol import keller_protocol as kp
    KELLER_LIB_AVAILABLE = True
except ImportError:
    KELLER_LIB_AVAILABLE = False
    print("Warning: keller-protocol not installed — Keller sensor disabled.")
    print("Install: pip install keller-protocol")

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════
# LabJack vacuum gauge (FIO2, analogue) — Pfeiffer IKR 270 cold cathode gauge
# behind a voltage divider. Gauge characteristic (manual, Appendix A):
#   p [mbar] = 10 ** (1.25 * U_gauge - 12.75)   valid for U_gauge 1.96 … 8.6 V
# The LabJack sees V = U_gauge / VACUUM_DIVIDER_RATIO, so the slope per
# LabJack volt is 1.25 × ratio. The divider does not change the intercept.
# Same gauge, divider and conversion as the TE-valve driver.
LABJACK_FIO2_CHANNEL  = 2          # AIN2 (analogue input 2) = FIO2
VACUUM_DIVIDER_RATIO  = 3.235      # U_gauge / V_LabJack — calibrated 1.713 V ↔ 1.5e-6 mbar
VACUUM_GAUGE_SLOPE    = 1.25       # decades per gauge volt (IKR 270)
VACUUM_SLOPE          = VACUUM_GAUGE_SLOPE * VACUUM_DIVIDER_RATIO   # ≈ 4.044 decades per LabJack volt
VACUUM_INTERCEPT      = -12.75     # log10(p / mbar) at 0 V (IKR 270, mbar)
VACUUM_AIN_SPECIAL    = True       # read FIO2 on the U3-LV "special" 0-3.6 V range
                                   # (negative channel 32) instead of the normal
                                   # 0-2.44 V. With the 3.235 divider the gauge's full
                                   # 8.6 V maps to 2.66 V: the normal range clips it at
                                   # ≈ 1-2e-3 mbar, the special range doesn't.
                                   # Cost: ~2× coarser steps (≈ 1 % in pressure).
VACUUM_AIN_NEG        = 32 if VACUUM_AIN_SPECIAL else 31   # 31 = single-ended vs GND
# A reading outside the gauge's valid signal range is not a pressure: it is
# logged blank, shown as its status, and kept out of the chart and of capture
# analysis — never stored as a made-up number. Same ranges as the TE-valve driver.
VACUUM_GAUGE_ERROR_V  = 0.5        # gauge side: below = sensor error / no supply
VACUUM_GAUGE_MIN_V    = 1.96       # gauge side: below = underrange (<5e-11 mbar) or not ignited
VACUUM_GAUGE_MAX_V    = 8.6        # gauge side: above = overrange (>1e-2 mbar)
LABJACK_AIN_SAT_V     = 3.55 if VACUUM_AIN_SPECIAL else 2.40
                                   # at or above this the reading is CLIPPED, not a
                                   # pressure. None = don't check.
LABJACK_SAMPLE_HZ     = 10         # internal sampling rate

KELLER_BAUD        = 9600
KELLER_ADDRESS     = 250        # bus address 0xFA — confirmed for this unit
KELLER_PORT        = None       # None = auto-detect
KELLER_TIMEOUT     = 0.3
KELLER_ECHO        = True       # K-114 adapter echoes TX
KELLER_POLL_HZ     = 4          # Keller read rate
FIRE_SNAPSHOT_N    = 5          # recent Keller samples median-ed for the at-fire
                                # upstream snapshot. A median rejects the ±1 LSB
                                # quantisation toggle and one-off spikes without
                                # the lag a mean would add over the slow drift.
ZC1_BAUD           = 38400
ZC1_PORT           = None       # None = auto-detect
LOG_INTERVAL_S     = 0.5        # seconds between logged rows (drift-free)
DEFAULT_OPEN_US    = 1600       # µs — matches current ZC1 setting

DEFAULT_DRIVE_HZ   = 1.0        # automatic pulse repetition rate (Hz)
DRIVE_HZ_MIN       = 0.01       # clamp: slowest auto-drive rate
DRIVE_HZ_MAX       = 200.0      # clamp: fastest auto-drive rate

CHART_SECONDS      = 120        # pressure strip-chart window (s)
DISPLAY_SMOOTH_S   = 1.0        # strip-chart display smoothing, in SECONDS (display
                                # only; the log stays raw). Applied to all three
                                # charts, so each smooths over the same real time
                                # regardless of its sample rate. 0 = no smoothing.
LOG_DIR            = str(Path(__file__).parent / "logs")

# ─── Pulse capture ────────────────────────────────────────────────────────────
# The 10 Hz monitoring loop samples the chamber gauge every 100 ms. A pulse
# transient decays with tau = V/S_eff, of order 100 ms, so at 10 Hz the recorded
# "peak" is whatever the sampler happened to land on — systematically low, by an
# unknown amount. Capture streams AIN2 in hardware at CAPTURE_RATE_HZ, fires the
# valve at a known point inside the window, and reports the peak the gauge
# actually saw.
#
# The chamber gauge's own response is ~10 ms above 1e-6 mbar, so 5 kHz
# oversamples the instrument by ~50x. That is deliberate: the surplus is spent
# on noise averaging in the decimation step, not on bandwidth.
# The U3 will not sustain every scan rate at every resolution, and it signals
# a configuration it cannot meet by erroring on every packet rather than by
# refusing outright. So the rate is probed at the first capture: the ladder is
# tried fastest-first and the first clean rate is cached for the session.
CAPTURE_RATE_LADDER = (5000, 2500, 2000, 1000, 500)   # Hz, fastest first
CAPTURE_RESOLUTION  = 0        # U3 stream resolution index (0 = best noise)
CAPTURE_PRE_S       = 1.0      # baseline recorded before the pulse (s)
CAPTURE_POST_S      = 5.0      # transient + tail recorded after the pulse (s)
CAPTURE_DECIMATE    = 10       # samples averaged per analysis point
CAPTURE_SETTLE_S    = 3.0      # pause between shots in a multi-shot run (s)
STREAM_SELFTEST_S   = 0.75     # sustained self-test span per rate rung (s)

# ─── Paired-pulse test (pp) ───────────────────────────────────────────────────
# Does gas flow freely through the upstream capillary during a pulse (A, the
# capillary as a brake), or is only the gas stored near the valve released (B,
# the capillary as a reservoir that then refills)? Fire two pulses dt apart in
# one capture and compare the gas they deliver with a single shot taken just
# before: Q2/Q1 = I_pair / I_single - 1, with I the integral of the excess
# pressure. In A the second dose equals the first at every dt; in B it is
# smaller at short dt and recovers with the refill time tau. The pair is timed
# by the ZC1 itself (shot series: G = 2 shots, CYCLE time C = dt), not by the
# serial link, so dt is exact to the ZC1's µs timer.
PAIR_DT_MS          = (1, 2, 3, 5, 10, 20, 50, 100)   # default dt sweep (ms)
PAIR_REPEATS        = 8        # pairs averaged per dt, each with its own single reference
PAIR_DT_MAX_MS      = 1000.0   # longest dt accepted (the pair must sit well inside the window)
PAIR_FIT_TAU_S      = (1e-4, 10.0)   # tau search range for the recovery fit (s)
PAIR_A_MIN          = 0.05     # smallest drop in the second dose counted as depletion
                               # (B); below it, or within 2 SE of zero, it is A

CHAMBER_VOLUME_L    = 15.586   # geometric estimate (5 combined cylinders); consistent
                               # with measured tau ~161 ms if pump line is DN63+.
                               # Pending final pump-line-diameter check vs Cornelia's 5 L.
CHAMBER_LIMIT_MBAR  = 1.0e-6   # the in-chamber limit being demonstrated against

# ─── Run identity (what physically produced these captures) ────────────────────
# The capillaries and the gas are NOT auto-detectable — they change only when
# hardware is swapped, and nothing in the rig reports them. They are stamped
# into every capture header and every session row so a file is self-describing
# and re-analysable without the lab notebook. The catch: a stale value here is
# WORSE than none, because it mislabels data confidently. Defences: the driver
# echoes the identity in a banner at startup, keeps it in the status hint at all
# times, and lets it be changed at runtime (cap / gas commands) so a mid-session
# swap can be recorded without editing this file and relaunching.
#
# Two capillaries are recorded independently:
#   downstream — the conductance limiter after the valve, into the chamber. This
#                is the one that sets chamber conductance and shapes the decay.
#                The Gyger is never run without a capillary, so this is always
#                fitted and both values must be positive.
#   upstream   — the feed capillary before the valve, on the reservoir side.
#                0 records "none fitted" and is displayed as such.
#
# Upstream *pressure* is deliberately NOT here: it is read live from the Keller
# and stamped at the fire instant, so it can never go stale. (Distinct from the
# upstream capillary geometry above, which is a fixed hardware fact.)
DEFAULT_DOWNSTREAM_CAP_ID_UM  = 50.0    # downstream limiter bore, micrometres
DEFAULT_DOWNSTREAM_CAP_LEN_MM = 100.0   # downstream limiter length, millimetres
DEFAULT_UPSTREAM_CAP_ID_UM    = 0.0     # 0 = no upstream capillary fitted
DEFAULT_UPSTREAM_CAP_LEN_MM   = 0.0     # 0 = no upstream capillary fitted
DEFAULT_GAS_SPECIES      = "N2"    # gas at the valve. RECORDED ONLY — the gauge conversion
                                   # still assumes N2/air; apply the species factor
                                   # (H2~2.4, He~5.9, Ar~0.8) downstream, not here.

FIT_NOISE_K            = 5.0   # stop the decay fit at this x baseline noise
FIT_MIN_DYNAMIC_RANGE  = 20.0  # need peak excess this far above the noise floor
FIT_START_FRACTIONS    = (0.7, 0.5, 0.35, 0.25, 0.15)  # candidate fit starts
FIT_MIN_POINTS         = 20    # minimum points for a candidate fit to count

# --- Pulse detection ---------------------------------------------------------
# A capture always produces numbers. Without a test for whether anything
# actually happened, the maximum of a flat noisy trace gets reported as a
# "peak" - the max of ~1000 normal samples sits ~3 sd above the mean, which
# on a 5e-7 baseline looks like a plausible small pulse. Two independent
# tests must pass before a shot is called a detection.
DETECT_PEAK_K    = 10.0   # peak excess must exceed this x baseline sd
DETECT_WINDOW_S  = 0.5    # averaging window just after the fire
DETECT_T_STAT    = 6.0    # window mean must exceed this many standard errors
FLAT_TRACE_V     = 1e-5   # below this spread in volts the input is not live
# ═══════════════════════════════════════════════════════════════════════════════

os.makedirs(LOG_DIR, exist_ok=True)


def _make_log_path():
    """Return a writable CSV path, retrying with a unique suffix if locked."""
    base = datetime.now().strftime('%Y%m%d_%H%M%S')
    for attempt in range(100):
        suffix = "" if attempt == 0 else f"_{attempt}"
        path = os.path.join(LOG_DIR, f"pvd-log_{base}{suffix}.csv")
        try:
            fh = open(path, 'x', newline='')
            fh.close()
            return path
        except (PermissionError, FileExistsError):
            continue
    return f"pvd-log_{base}.csv"


LOG_FILE = _make_log_path()

# ─── Capture manifest ─────────────────────────────────────────────────────────
# One summary row per shot — single or multi-shot, any rate — appended to a
# single per-session file. The bulky traces stay in their own per-shot
# pulse_*.csv files; this manifest is the index across them, so all of a
# session's captures sit in one table for scatter / tau / regression work
# without opening dozens of files. `trace_file` on each row points back to the
# full trace. The column set is FIXED (below): the detected, not-detected,
# flat-trace and invalid-gauge cases each carry a different subset of fields, so a fixed schema
# with blanks for the missing ones keeps every row aligned. Named to match the
# session log's stamp so the pair is obvious: pvd-log_<ts> / pvd-captures_<ts>.
MANIFEST_FILE = os.path.join(
    LOG_DIR,
    "pvd-captures_" + os.path.basename(LOG_FILE)[len("pvd-log_"):])

MANIFEST_COLUMNS = [
    'capture_group_id', 'capture_shot_index', 'capture_group_size',
    'detected', 'flat_trace', 'gauge_invalid', 'invalid_samples',
    'capture_rate_hz', 'analysis_rate_hz',
    'open_time_us', 'zc1_ack', 'shots_per_fire', 'pair_dt_us',
    'pairtest_dt_ms', 'pairtest_repeat', 'pairtest_role',
    'downstream_cap_id_um', 'downstream_cap_length_mm',
    'upstream_cap_id_um', 'upstream_cap_length_mm',
    'gas_species',
    'upstream_bar_at_fire', 'upstream_degC_at_fire',
    'p_base_mbar', 'baseline_sd_mbar',
    'peak_mbar', 'dp_mbar', 't_peak_s', 't_rise_10_90_s',
    'peak_snr', 'detect_t_stat', 'dp_upper_bound_mbar',
    'integral_mbar_s',
    'tau_s', 'r2', 's_eff_l_s', 'chamber_volume_l',
    'q_pulse_mbar_l_integral', 'q_pulse_mbar_l_v_dp',
    'samples_missed', 'trace_file',
]

_manifest_lock = threading.Lock()   # captures are serialised on the LabJack
                                    # thread already; this is belt-and-braces.


def append_manifest(meta: dict, trace_path):
    """Append one summary row for a capture to MANIFEST_FILE.

    Called for every shot that produced a file (detected, not-detected,
    flat-trace and invalid-gauge alike), so the manifest is a complete record
    of the session's captures — a non-detection is a real result (an upper bound), not nothing.
    Uses the fixed MANIFEST_COLUMNS: unknown keys in *meta* are dropped and
    missing ones written blank, so the four outcome shapes all append cleanly.
    The header is written only when the file is first created."""
    row = dict(meta)
    row['trace_file'] = os.path.basename(trace_path) if trace_path else ''
    with _manifest_lock:
        try:
            new = (not os.path.exists(MANIFEST_FILE)
                   or os.path.getsize(MANIFEST_FILE) == 0)
            with open(MANIFEST_FILE, 'a', newline='') as f:
                w = csv.DictWriter(f, fieldnames=MANIFEST_COLUMNS,
                                   extrasaction='ignore', restval='')
                if new:
                    w.writeheader()
                w.writerow(row)
                f.flush()
        except Exception as e:
            log_event(f"MANIFEST: could not append row ({e})")


# ─── Shared state ─────────────────────────────────────────────────────────────
_lock        = threading.Lock()        # protects _state / _events / _chart
_zc1_lock    = threading.Lock()        # serialises ALL access to the ZC1 port
_stop        = threading.Event()
_drive_on    = threading.Event()       # set => automatic valve drive is running
_state       = dict(
    keller_pressure_samples     = [],    # accumulated between log rows, then averaged
    keller_temperature_samples  = [],    # accumulated between log rows, then averaged
    vacuum_chamber_mbar         = None,  # latest single reading, None if not valid
    vacuum_status               = None,  # None = valid, else why the reading is not
    pulse_interval              = 0,     # pulses fired since last log row (reset each row)
    pulses_total                = 0,
    open_time_us                = DEFAULT_OPEN_US,
    drive_rate_hz               = DEFAULT_DRIVE_HZ,
    # Run identity — what produced the captures. Runtime-settable (cap / gas).
    downstream_cap_id_um        = DEFAULT_DOWNSTREAM_CAP_ID_UM,
    downstream_cap_len_mm       = DEFAULT_DOWNSTREAM_CAP_LEN_MM,
    upstream_cap_id_um          = DEFAULT_UPSTREAM_CAP_ID_UM,
    upstream_cap_len_mm         = DEFAULT_UPSTREAM_CAP_LEN_MM,
    gas_species                 = DEFAULT_GAS_SPECIES,
    # Pulse event log: list of (iso_timestamp, open_us, ack_ok) since last row
    pulse_events                = [],
)
_events      = deque(maxlen=200)                          # (timestamp, text)
_chart       = deque(maxlen=CHART_SECONDS * KELLER_POLL_HZ)     # recent upstream pressures
_temp_chart  = deque(maxlen=CHART_SECONDS * KELLER_POLL_HZ)     # recent upstream temperatures
_keller_p_recent = deque(maxlen=FIRE_SNAPSHOT_N)   # last few upstream pressures — for the fire snapshot
_keller_t_recent = deque(maxlen=FIRE_SNAPSHOT_N)   # last few upstream temperatures — for the fire snapshot
_keller_log_pending = []   # (ts, p, t) per Keller poll — drained by the logger so every raw sample is written (zero loss)
_vac_chart   = deque(maxlen=CHART_SECONDS * LABJACK_SAMPLE_HZ)  # recent vacuum readings
_zc1_serial  = None
_zc1_ok      = False
_keller_ok   = False
_labjack_ok  = False

_capture_request = threading.Event()   # GUI -> LabJack thread: capture now
_capture_busy    = False               # True while a capture is running
_capture_pending = 0                   # shots still queued in a multi-shot run
_capture_runs    = []                  # per-shot summary dicts, this session
_stream_rate     = None                # probed once, then cached for the session
_summary_window  = 0                   # shots in the current multi-shot run
_capture_group_id    = None            # shared id for the shots of one `c n` run
_capture_group_size  = 0               # shots requested in the current `c n` run
_capture_group_index = 0               # shots started so far in the current run
_pairtest        = None                # dict while a paired-pulse test (pp) runs
_pairtest_cancel = threading.Event()   # GUI -> LabJack thread: end the pp test early


def log_event(text: str):
    """Record a timestamped event for the GUI log (and echo to console)."""
    stamp = datetime.now().strftime('%H:%M:%S')
    with _lock:
        _events.append((stamp, text))
    print(f"[{stamp}] {text}")


# ═══════════════════════════════════════════════════════════════════════════════
# PORT FILTERING
# ═══════════════════════════════════════════════════════════════════════════════

def _usb_serial_ports():
    """Return non-Bluetooth COM ports, FTDI/USB-serial first."""
    real, other = [], []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').upper()
        hwid = (p.hwid or '').upper()
        if 'BLUETOOTH' in desc or 'BLUETOOTH' in hwid or 'BTH' in hwid:
            continue
        if not hwid or hwid == 'N/A':
            continue
        if 'FTDI' in desc or 'FTDI' in hwid or 'USB SERIAL' in desc or 'VID_0403' in hwid:
            real.append(p)
        else:
            other.append(p)
    return real + other


# ═══════════════════════════════════════════════════════════════════════════════
# KELLER  — pressure / temperature
# ═══════════════════════════════════════════════════════════════════════════════

def _probe_keller(port: str):
    if not KELLER_LIB_AVAILABLE:
        return None
    try:
        bus = kp.KellerProtocol(
            port=port, baud_rate=KELLER_BAUD,
            timeout=KELLER_TIMEOUT, echo=KELLER_ECHO,
        )
        fw = bus.f48(KELLER_ADDRESS)
        p1 = bus.f73(KELLER_ADDRESS, 1)
        if p1 is not None and -2.0 < p1 < 50.0:
            print(f"  [Keller] {port}: init OK (fw {fw}), P1={p1:.3f} bar")
            return bus
    except Exception:
        pass
    return None


def _detect_keller_bus():
    if KELLER_PORT is not None:
        bus = _probe_keller(KELLER_PORT)
        if bus is not None:
            print(f"  [Keller] connected on {KELLER_PORT}")
            return KELLER_PORT, bus
        print(f"  [Keller] no response on configured port {KELLER_PORT}")
        return None, None
    for port_info in _usb_serial_ports():
        port = port_info.device
        bus = _probe_keller(port)
        if bus is not None:
            print(f"  [Keller] detected on {port}")
            return port, bus
    return None, None


def keller_thread(initial_port: str, initial_bus):
    """Keller read loop with automatic reconnection.

    On any read error the state is immediately nulled (so the GUI shows '---'
    rather than silently repeating the last good value) and the thread scans
    for the sensor again before resuming.
    """
    global _keller_ok
    interval = 1.0 / KELLER_POLL_HZ
    port, bus = initial_port, initial_bus

    while not _stop.is_set():
        # ── read loop — exits on error ─────────────────────────────────────
        try:
            while not _stop.is_set():
                t0 = time.time()
                p1   = bus.f73(KELLER_ADDRESS, 1)
                tob1 = bus.f73(KELLER_ADDRESS, 4)
                ts_s = datetime.now().isoformat(timespec='milliseconds')
                with _lock:
                    if p1 is not None:
                        _state['keller_pressure_samples'].append(round(p1, 4))
                        _keller_p_recent.append(p1)
                        _chart.append(p1)
                    if tob1 is not None:
                        _state['keller_temperature_samples'].append(round(tob1, 2))
                        _keller_t_recent.append(tob1)
                        _temp_chart.append(tob1)
                    if p1 is not None or tob1 is not None:
                        # One entry per poll, timestamped at read time, so the
                        # logger can write every raw sample regardless of its
                        # flush cadence — zero loss, no thread-drift dependence.
                        _keller_log_pending.append((
                            ts_s,
                            round(p1, 4) if p1 is not None else None,
                            round(tob1, 2) if tob1 is not None else None))
                _stop.wait(timeout=max(0.0, interval - (time.time() - t0)))

        except Exception as e:
            log_event(f"Keller read error: {e} — reconnecting…")

        # ── null state immediately so GUI shows '---' ──────────────────────
        _keller_ok = False
        with _lock:
            _state['keller_pressure_samples']    = []
            _state['keller_temperature_samples'] = []
            _keller_p_recent.clear()
            _keller_t_recent.clear()

        if _stop.is_set():
            break

        # ── scan for the sensor again ──────────────────────────────────────
        _stop.wait(timeout=5.0)
        new_port, new_bus = _detect_keller_bus()
        if new_bus is not None:
            port, bus = new_port, new_bus
            _keller_ok = True
            log_event(f"Keller reconnected · {port}")
        else:
            log_event("Keller not found — will retry")


# ═══════════════════════════════════════════════════════════════════════════════
# LABJACK U3 — VACUUM CHAMBER PRESSURE (FIO2)
# ═══════════════════════════════════════════════════════════════════════════════

_SAT_MBAR = (10.0 ** (VACUUM_SLOPE * LABJACK_AIN_SAT_V + VACUUM_INTERCEPT)
             if LABJACK_AIN_SAT_V is not None else None)
VAC_ERROR       = "sensor error / no supply"
VAC_UNDER       = "underrange or not yet ignited"
VAC_OVER        = "overrange (>1e-2 mbar)"
VAC_SATURATED   = (f"LabJack input saturated (>~{_SAT_MBAR:.0e} mbar)"
                   if _SAT_MBAR and _SAT_MBAR < 1e-2 else "LabJack input saturated")


def _vacuum_gauge_status(volts: float):
    """Classify a LabJack voltage against the IKR 270 signal ranges.
    Returns None if the reading is valid, otherwise one of the VAC_* strings."""
    if LABJACK_AIN_SAT_V is not None and volts >= LABJACK_AIN_SAT_V:
        return VAC_SATURATED                    # clipped at the LabJack input
    u = volts * VACUUM_DIVIDER_RATIO            # back to gauge-side volts
    if u < VACUUM_GAUGE_ERROR_V:
        return VAC_ERROR
    if u < VACUUM_GAUGE_MIN_V:
        return VAC_UNDER
    if u > VACUUM_GAUGE_MAX_V:
        return VAC_OVER
    return None


def _voltage_to_vacuum_mbar(volts: float):
    """LabJack voltage → chamber pressure in mbar, or None if the reading is
    not valid (see _vacuum_gauge_status)."""
    if _vacuum_gauge_status(volts) is not None:
        return None
    return 10.0 ** (VACUUM_SLOPE * volts + VACUUM_INTERCEPT)


def labjack_thread():
    global _labjack_ok
    if not LABJACK_AVAILABLE:
        return
    interval = 1.0 / LABJACK_SAMPLE_HZ
    while not _stop.is_set():
        # ── connect / configure ────────────────────────────────────────────
        try:
            lj = u3.U3()
            # A capture that died before streamStop() leaves the stream running
            # in the U3 itself, and that state outlives our process. Every later
            # AIN() then fails with STREAM_IS_ACTIVE (48), which this thread
            # reads as a disconnect — so it reconnects, fails again, and spins.
            # Relaunching the app cannot fix it because the state is in the
            # device. Clear it defensively on every connect; the exception is
            # the normal case, when nothing was streaming.
            try:
                lj.streamStop()
                log_event("LabJack: cleared a stale stream left by an earlier capture")
            except Exception:
                pass
            lj.getCalibrationData()
            lj.configIO(FIOAnalog=0x04)   # FIO2 as analogue input (vacuum gauge)
            _labjack_ok = True
            log_event("LabJack connected  FIO2 analog "
                      f"({'0-3.6 V special' if VACUUM_AIN_SPECIAL else '0-2.44 V'} range)")
        except Exception as e:
            _labjack_ok = False
            log_event(f"LabJack connect failed: {e} — retry in 5 s")
            _stop.wait(timeout=5)
            continue   # retry the outer loop

        # ── read loop — breaks on error to trigger reconnect ───────────────
        vac_status = "startup"   # last gauge status, so changes are logged once
        try:
            while not _stop.is_set():
                t0 = time.time()

                # ── capture ────────────────────────────────────────────────
                # Streaming takes exclusive control of the U3, so it has to run
                # on this thread. Monitoring pauses for the window and resumes
                # automatically. A multi-shot run re-arms itself here.
                global _capture_busy, _capture_pending, _capture_group_index
                global _pairtest
                pt = _pairtest
                if pt is not None and not _capture_request.is_set():
                    if _pairtest_cancel.is_set() or not pt['plan']:
                        _pairtest_finish(pt, cancelled=_pairtest_cancel.is_set())
                        _pairtest = None
                        _pairtest_cancel.clear()
                        continue
                    if time.time() >= pt['next_t']:
                        step = pt['plan'].popleft()
                        _capture_busy = True
                        try:
                            _pairtest_step(lj, pt, step)
                        except Exception as e:
                            log_event(f"PAIR TEST: capture failed ({e}) — "
                                      f"test stopped, reconnecting device")
                            _capture_busy = False
                            _pairtest_finish(pt, cancelled=True)
                            _pairtest = None
                            break
                        finally:
                            _capture_busy = False
                        # Monitoring resumes between captures; the next one
                        # waits for the chamber to settle.
                        pt['next_t'] = time.time() + CAPTURE_SETTLE_S
                        continue

                if _capture_request.is_set():
                    _capture_request.clear()
                    _capture_busy = True
                    _capture_group_index += 1
                    try:
                        _run_capture(
                            lj,
                            shot_index=_capture_group_index,
                            group_id=_capture_group_id,
                            group_size=_capture_group_size)
                    except Exception as e:
                        log_event(f"CAPTURE failed ({e}) — reconnecting device")
                        _capture_busy = False
                        _capture_pending = 0
                        break
                    finally:
                        _capture_busy = False
                    if _capture_pending > 0:
                        _capture_pending -= 1
                        if _capture_pending > 0:
                            _stop.wait(timeout=CAPTURE_SETTLE_S)
                            _capture_request.set()
                        else:
                            _summarise_runs(_summary_window)
                    continue

                try:
                    raw  = lj.getAIN(LABJACK_FIO2_CHANNEL, VACUUM_AIN_NEG)
                    mbar = _voltage_to_vacuum_mbar(raw)
                    status = _vacuum_gauge_status(raw)
                    if status != vac_status:
                        if status is not None:
                            log_event(f"Vacuum gauge {status} — "
                                      f"{raw:.3f} V at LabJack, "
                                      f"{raw * VACUUM_DIVIDER_RATIO:.2f} V at gauge")
                        elif vac_status != "startup":
                            log_event("Vacuum gauge back in range")
                        vac_status = status
                    with _lock:
                        _state['vacuum_chamber_mbar'] = mbar
                        _state['vacuum_status'] = status
                        _vac_chart.append(mbar)
                except Exception as e:
                    log_event(f"LabJack read error: {e}")
                    break   # drop to finally → reconnect
                _stop.wait(timeout=max(0.0, interval - (time.time() - t0)))
        finally:
            _labjack_ok = False
            try:
                lj.close()
            except Exception:
                pass
            if not _stop.is_set():
                log_event("LabJack disconnected — attempting reconnect")
                # Without a pause this loop reconnects, fails, and logs again
                # within milliseconds — hundreds of identical lines a second,
                # which floods the event log and starves the GUI. A short wait
                # costs nothing on a transient fault and keeps the app usable
                # on a persistent one.
                _stop.wait(timeout=2.0)


# ═══════════════════════════════════════════════════════════════════════════════
# ZC1 DETECTION AND CONTROL
# ═══════════════════════════════════════════════════════════════════════════════
#
# Protocol notes (from Betriebsanleitung ZC1 Rev. 2.02, §6):
#
#   Typ A — execution command (e.g. 'V' = fire single shot)
#     Send:    V
#     Receive: V  LF  CR  >
#     The ZC1 echoes the command byte immediately, then sends LF CR after the
#     shot completes, then '>' to signal ready. All three must arrive before
#     the ACK is considered good. '>' is the definitive ready-prompt.
#
#   Typ B — parameter set (e.g. "1600B" = set open-time to 1600 µs)
#     Send:    1 6 0 0 B
#     Receive: 1 6 0 0 B  LF  CR  >
#     Each character is echoed in turn; '>' ends the transaction.
#
#   Typ C — parameter read (e.g. 'b' = read OPEN-Time)
#     Send:    b
#     Receive: . b 0 0 0 0 1 6 0 0  LF  CR  >
#     Used here as a safe connection health-check (no mechanical action).
#
#   Boot behaviour:
#     After power-up the ZC1 resumes whatever mode was last saved to EEPROM
#     (Terminal or one of the SPS modes F0–F5). ESC switches SPS → Terminal.
#     In SPS mode the ZC1 ignores 'V' entirely, so we must guarantee Terminal
#     mode on every (re)connect.
#
# Timeout strategy:
#   The ZC1 echoes each character immediately. After a 'V' the shot runs for
#   open_us µs then the ZC1 sends LF CR >. We read with a 200 ms hard timeout
#   — enough for any open-time up to ~5 s — and declare ACK only if '>' arrives.
#   If '>' is absent we treat it as a comms failure, not just "no ACK", so the
#   reconnect supervisor can take over.

_ZC1_TIMEOUT   = 0.20   # seconds — serial read timeout (covers shot + response)
_ZC1_INIT_WAIT = 0.15   # seconds — pause after ESC before flushing


def _zc1_read_until_prompt(ser: serial.Serial, timeout_s: float = _ZC1_TIMEOUT) -> str:
    """Read from *ser* until the '>' prompt or timeout.  Returns decoded string."""
    buf = bytearray()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            if b'>' in buf:
                break
        else:
            time.sleep(0.005)
    return buf.decode('ascii', errors='ignore')


def _zc1_init(ser: serial.Serial) -> bool:
    """Put ZC1 into a known Terminal-mode state and verify comms.

    Steps:
      1. Send ESC — switches SPS → Terminal mode (safe to send in either mode).
      2. Wait briefly and flush any banner text the ZC1 may emit.
      3. Send 'b' (Typ-C: read OPEN-Time) — a safe no-action health check.
      4. Verify that '>' appears in the response (ready prompt).

    Returns True on success, False if the device does not respond correctly.
    """
    try:
        ser.reset_input_buffer()
        ser.write(b'\x1b')                          # ESC → Terminal mode
        time.sleep(_ZC1_INIT_WAIT)
        ser.reset_input_buffer()                    # discard any mode-switch banner
        ser.write(b'b')                             # read OPEN-Time (Typ C, harmless)
        reply = _zc1_read_until_prompt(ser, timeout_s=0.5)
        return '>' in reply
    except Exception:
        return False


def _probe_zc1(port: str):
    """Open *port* and attempt ZC1 initialisation.  Returns serial.Serial or None."""
    try:
        s = serial.Serial(
            port, ZC1_BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=_ZC1_TIMEOUT,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        if _zc1_init(s):
            print(f"  [ZC1]    detected on {port}")
            return s
        s.close()
    except Exception:
        pass
    return None


def _detect_zc1():
    """Scan available ports for the ZC1.  Returns (port_name, serial.Serial) or
    (None, None) if not found."""
    candidates = []
    if ZC1_PORT is not None:
        candidates = [type('P', (), {'device': ZC1_PORT})()]
    else:
        candidates = _usb_serial_ports()
    for port_info in candidates:
        port = port_info.device
        s = _probe_zc1(port)
        if s is not None:
            return port, s
    return None, None


def zc1_set_open_time(ser: serial.Serial, open_us: int):
    """Send open-time parameter to ZC1 (Typ-B command '<value>B').
    Waits for '>' prompt.  Returns (clamped_value, raw_reply)."""
    open_us = max(50, min(5_000_000, int(open_us)))
    cmd = f"{open_us}B".encode('ascii')
    with _zc1_lock:
        try:
            ser.reset_input_buffer()
            ser.write(cmd)
            reply = _zc1_read_until_prompt(ser)
        except Exception as e:
            reply = f"ERROR:{e}"
    return open_us, reply


def _zc1_read_param(ser: serial.Serial, letter: str):
    """Read one ZC1 parameter (Typ-C, lower-case letter, e.g. 'g' = number of
    shots, 'c' = CYCLE time). The reply is '.g00000002' then LF CR '>'.
    Returns the integer, or None if the reply could not be parsed. Call with
    _zc1_lock held."""
    ser.reset_input_buffer()
    ser.write(letter.encode('ascii'))
    reply = _zc1_read_until_prompt(ser)
    m = re.search(r'\.' + letter + r'(\d+)', reply)
    return int(m.group(1)) if m else None


def _zc1_write_param(ser: serial.Serial, value: int, letter: str) -> str:
    """Set one ZC1 parameter (Typ-B, '<value><LETTER>'). Call with _zc1_lock held."""
    ser.reset_input_buffer()
    ser.write(f"{int(value)}{letter}".encode('ascii'))
    return _zc1_read_until_prompt(ser)


def zc1_arm_pair(ser: serial.Serial, dt_us: int):
    """Set the ZC1 shot series to exactly two shots dt_us apart: G = 2, C = dt_us.

    Both are read back. The ZC1 raises C to OPEN + 100 µs if it is shorter, so
    the CYCLE time actually in force is returned and is what gets recorded.
    Returns (ok, cycle_us, message). ok is False unless G reads back as 2 —
    G powers up at its saved value (100 by default), and 'Q' with G = 100
    would fire 100 pulses."""
    with _zc1_lock:
        try:
            _zc1_write_param(ser, dt_us, 'C')
            _zc1_write_param(ser, 2, 'G')
            c = _zc1_read_param(ser, 'c')
            g = _zc1_read_param(ser, 'g')
        except Exception as e:
            return False, None, f"ZC1 error: {e}"
    if g != 2:
        return False, c, f"number of shots reads back {g!r}, not 2 — not fired"
    if c is None:
        return False, None, "CYCLE time could not be read back — not fired"
    return True, c, ""


def zc1_fire_pair(ser: serial.Serial, cycle_us: int, mark=None):
    """Fire the armed two-shot series ('Q'). G is read back once more first,
    and nothing is fired unless it is still 2. *mark* is called just before
    'Q' is sent, to timestamp the fire. The ZC1 returns '>' only once the
    series is done, so the wait allows for it. Returns (ok, message)."""
    wait_s = _ZC1_TIMEOUT + 2 * cycle_us / 1e6
    with _zc1_lock:
        try:
            g = _zc1_read_param(ser, 'g')
            if g != 2:
                return False, f"number of shots reads back {g!r}, not 2 — not fired"
            if mark is not None:
                mark()
            ser.reset_input_buffer()
            ser.write(b'Q')
            reply = _zc1_read_until_prompt(ser, timeout_s=wait_s)
            return ('>' in reply), ("" if '>' in reply else "no ZC1 prompt after Q")
        except Exception as e:
            return False, f"ZC1 error: {e}"


def zc1_fire_pulse(ser: serial.Serial) -> bool:
    """Fire one pulse (Typ-A 'V' command).  Returns True if '>' prompt received.

    The ZC1 protocol for 'V':
      Send:    V
      Receive: V  LF  CR  >
    We read until '>' with a 200 ms timeout.  Absence of '>' means the ZC1
    did not complete the transaction — caller should treat this as a fault.
    """
    with _zc1_lock:
        try:
            ser.reset_input_buffer()
            ser.write(b'V')
            reply = _zc1_read_until_prompt(ser)
            return '>' in reply
        except Exception:
            return False


def fire_one():
    """Fire a single pulse, update counters, and record a timestamped pulse event."""
    global _zc1_ok
    ok = False
    if _zc1_serial is not None and _zc1_ok:
        try:
            ok = zc1_fire_pulse(_zc1_serial)
            if not ok:
                # '>' never arrived — mark offline so supervisor reconnects
                _zc1_ok = False
                log_event("ZC1 no prompt — marking offline for reconnect")
        except Exception as e:
            log_event(f"ZC1 fire error: {e}")
            _zc1_ok = False
    ts = datetime.now().isoformat(timespec='microseconds')
    with _lock:
        _state['pulse_interval'] += 1
        _state['pulses_total']   += 1
        ot = _state['open_time_us']
        _state['pulse_events'].append((ts, ot, ok))
    return ok, ot


def fire_pair(cycle_us: int, mark=None):
    """Fire the armed two-shot series and record both pulses. Returns
    (ok, open_us, message). Nothing is fired unless the ZC1 confirms G = 2."""
    global _zc1_ok
    ok, msg = False, "no device"
    if _zc1_serial is not None and _zc1_ok:
        ok, msg = zc1_fire_pair(_zc1_serial, cycle_us, mark=mark)
    t0 = datetime.now()
    with _lock:
        ot = _state['open_time_us']
        if ok:
            _state['pulse_interval'] += 2
            _state['pulses_total']   += 2
            for k in range(2):
                ts = (t0 + timedelta(microseconds=k * cycle_us)).isoformat(
                    timespec='microseconds')
                _state['pulse_events'].append((ts, ot, True))
    return ok, ot, msg


# ═══════════════════════════════════════════════════════════════════════════════
# PULSE CAPTURE  — hardware-streamed transient, valve fired inside the window
# ═══════════════════════════════════════════════════════════════════════════════
#
# Sequence, all driven from the LabJack thread because streaming needs exclusive
# control of the U3:
#
#   [-- CAPTURE_PRE_S baseline --][FIRE][-- CAPTURE_POST_S transient + tail --]
#
# The valve is fired from a short-lived helper thread. This is not cosmetic:
# fire_one() blocks until the ZC1 returns its '>' prompt, up to 200 ms, and the
# U3's stream buffer would overflow if the main capture loop stopped draining
# packets for that long.
#
# What comes out:
#   peak            — the highest pressure the gauge actually saw
#   dp              — peak above the pre-pulse baseline
#   integral        — area under (p - baseline), for the throughput calculation
#   tau, S_eff      — from the decay tail, if it is clean enough to fit
#   Q_pulse         — gas per shot, by two independent routes (see below)

def _stream_selftest(lj, rate):
    """Configure and briefly run the stream at *rate*; True if it is clean.

    The U3 rejects a stream configuration it cannot sustain by flagging an
    error on every packet rather than by raising, so the only reliable check
    is to start it and look. Doing this *before* arming the valve matters: a
    failure discovered mid-capture would mean a pulse fired into a capture
    that then had to be retried.
    """
    try:
        lj.streamConfig(NumChannels=1,
                        PChannels=[LABJACK_FIO2_CHANNEL],
                        NChannels=[VACUUM_AIN_NEG],   # same range as monitoring
                        Resolution=CAPTURE_RESOLUTION,
                        ScanFrequency=int(rate))
        lj.streamStart()
        try:
            # Stream for a sustained span, not just two packets. A rate the U3
            # can *start* is not necessarily one it can *hold*: two packets
            # clear in a few ms, far too short to see a slow FIFO overflow. If
            # the self-test is that brief it will bless a rate that then stalls
            # partway through the real 6 s capture — which is exactly the hang.
            # Watch the whole window and reject on any errored *or* missed
            # sample, so the chosen rate survives a full capture.
            deadline = time.time() + STREAM_SELFTEST_S
            saw_data = False
            for packet in lj.streamData():
                if time.time() > deadline:
                    break
                if packet is None:
                    continue
                if packet.get('errors'):
                    return False
                if packet.get('missed'):
                    return False
                if packet.get(f'AIN{LABJACK_FIO2_CHANNEL}'):
                    saw_data = True
            return saw_data
        finally:
            try:
                lj.streamStop()
            except Exception:
                pass
    except Exception:
        try:
            lj.streamStop()
        except Exception:
            pass
        return False
    return False


def _choose_stream_rate(lj):
    """Find the fastest rate in the ladder the U3 will actually stream.

    Probed once per session and cached. The gauge's own ~10 ms response means
    anything above a few hundred Hz is oversampling the instrument; the surplus
    is spent on noise averaging in the decimation step, so dropping a rung of
    the ladder costs very little.
    """
    global _stream_rate
    if _stream_rate is not None:
        return _stream_rate
    for rate in CAPTURE_RATE_LADDER:
        if _stream_selftest(lj, rate):
            _stream_rate = rate
            log_event(f"CAPTURE: stream rate {rate:g} Hz "
                      f"(analysis {rate/CAPTURE_DECIMATE:g} Hz)")
            return rate
        log_event(f"CAPTURE: {rate:g} Hz rejected by the U3 — trying slower")
    return None


def _smooth_display(vals, n):
    """Trailing moving average for DISPLAY ONLY. Never applied to logged or
    captured data — smoothing there would blunt real features. Same length as
    the input; each point averages the valid values among the last n samples.
    A sample that is None (not a valid reading) stays None, so it is never
    painted over by its neighbours."""
    if n <= 1 or len(vals) < 2:
        return list(vals)
    out, run = [], []
    for v in vals:
        run.append(v)
        if len(run) > n:
            run.pop(0)
        good = [x for x in run if x is not None]
        out.append(sum(good) / len(good) if v is not None else None)
    return out


def _median(vals):
    """Median of a short sequence; None if empty. Used for the fire-instant
    upstream snapshot: it sits on the true level between quantisation steps and
    rejects one-off spikes, without the lag a running mean would add over the
    reservoir's slow drift."""
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return None
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


def _capture_stream_fire(lj, rate, pair_cycle_us=None):
    """Stream AIN2 at *rate*, firing one pulse after CAPTURE_PRE_S — or, with
    *pair_cycle_us*, the armed two-shot ZC1 series (see zc1_arm_pair).

    Returns (volts, fire_index, fire_ok, open_us, missed, up_bar, up_degC,
    fire_msg).
    """
    pre_n   = int(rate * CAPTURE_PRE_S)
    total_n = int(rate * (CAPTURE_PRE_S + CAPTURE_POST_S))

    volts, missed = [], 0
    fire_idx, fired = None, {}

    def _fire():
        # Snapshot the upstream state at the fire instant, before fire_one()
        # blocks on the ZC1 prompt. A short median of the most recent Keller
        # samples sits on the true level between quantisation steps, rather than
        # whichever side of the ±1 LSB toggle a single reading landed on.
        with _lock:
            p = _median(_keller_p_recent)
            t = _median(_keller_t_recent)
        fired['up_bar']  = round(p, 4) if p is not None else None
        fired['up_degC'] = round(t, 2) if t is not None else None
        if pair_cycle_us is None:
            fired['ok'], fired['ot'] = fire_one()
        else:
            # G is re-checked before 'Q', a few ms after the thread starts, so
            # the fire is timestamped at the moment 'Q' is sent.
            def _mark():
                fired['idx'] = len(volts)
            fired['ok'], fired['ot'], fired['msg'] = fire_pair(
                pair_cycle_us, mark=_mark)

    lj.streamConfig(NumChannels=1,
                    PChannels=[LABJACK_FIO2_CHANNEL],
                    NChannels=[VACUUM_AIN_NEG],
                    Resolution=CAPTURE_RESOLUTION,
                    ScanFrequency=int(rate))
    window_s = CAPTURE_PRE_S + CAPTURE_POST_S
    deadline = time.time() + window_s + 3.0   # hard ceiling on the whole stream
    stalled  = False
    fire_thread = None
    try:
        lj.streamStart()
        for packet in lj.streamData():
            # A rate the U3 can start is not always one it can sustain. If the
            # stream stalls mid-capture, streamData() keeps yielding empty /
            # recovery packets and len(volts) never reaches total_n, so this
            # loop spins forever and the whole capture hangs. Bail on a
            # wall-clock deadline (or on shutdown) so _capture_busy is always
            # cleared and the app can recover instead of freezing.
            if _stop.is_set() or time.time() > deadline:
                stalled = len(volts) < total_n
                break
            if packet is None:
                continue
            if packet.get('errors'):
                raise RuntimeError(
                    f"{packet['errors']} bad packets at {rate:g} Hz")
            missed += packet.get('missed', 0) or 0
            chunk = packet.get(f'AIN{LABJACK_FIO2_CHANNEL}')
            if chunk:
                volts.extend(chunk)

            if fire_idx is None and len(volts) >= pre_n:
                fire_idx = len(volts)
                fire_thread = threading.Thread(target=_fire, daemon=True)
                fire_thread.start()

            if len(volts) >= total_n:
                break
    finally:
        try:
            lj.streamStop()
        except Exception:
            pass
        if fire_thread is not None:
            fire_thread.join(timeout=1.0)

    if stalled:
        log_event(f"CAPTURE: stream stalled at {len(volts)}/{total_n} "
                  f"samples @ {rate:g} Hz — aborted (would have hung)")

    fire_idx = fired.get('idx', fire_idx)
    return (volts[:total_n], fire_idx,
            fired.get('ok', False), fired.get('ot'), missed,
            fired.get('up_bar'), fired.get('up_degC'), fired.get('msg', ''))


def _decimate(values, factor):
    """Block-average *values* by *factor*, discarding any short final block."""
    return [sum(values[i:i + factor]) / factor
            for i in range(0, len(values) - factor + 1, factor)]


def _fit_decay(times, mbar, p_base, floor):
    """Least-squares fit of ln(p - p_base) vs t over the usable tail.

    Returns (tau, r2, n_points, t_start, start_frac) or None.

    Three details that matter.

    The base pressure is subtracted before taking logarithms: as p approaches
    p_base the raw tail flattens, which reads as a falsely long tau.

    The cutoff is set by baseline *noise*, not baseline *magnitude*. Because
    p_base is measured from a second of pre-trigger data it is known far more
    precisely than its own value, so the excess can be followed well below
    p_base. Cutting off at a multiple of p_base instead would throw away most
    of the decay, and at a base of 1e-7 with a peak of 4e-7 would leave
    nothing to fit at all.

    Where the fit *starts* is not a free choice. If gas is still arriving —
    which it is whenever a conductance limiter stretches the injection over a
    time comparable to the chamber's own decay — the early tail falls more
    slowly than the chamber alone would, and a fit that includes it reports a
    tau that is too long. Start too late instead and the signal has decayed
    into the noise. So rather than fix a start point, several are tried and
    the one with the best r-squared wins: a single exponential fits properly
    only over the region where the chamber is the only thing acting.
    """
    if not mbar:
        return None
    i_peak = max(range(len(mbar)), key=lambda i: mbar[i])
    peak_excess = mbar[i_peak] - p_base
    if peak_excess <= floor * FIT_MIN_DYNAMIC_RANGE:
        return None

    best = None
    for start_frac in FIT_START_FRACTIONS:
        i_start = None
        for i in range(i_peak, len(mbar)):
            if (mbar[i] - p_base) <= start_frac * peak_excess:
                i_start = i
                break
        if i_start is None:
            continue

        xs, ys = [], []
        for t, p in zip(times[i_start:], mbar[i_start:]):
            excess = p - p_base
            if excess <= floor:
                break
            xs.append(t)
            ys.append(math.log(excess))
        if len(xs) < FIT_MIN_POINTS:
            continue

        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
        if sxx <= 0 or sxy >= 0:
            continue
        slope = sxy / sxx
        intercept = my - slope * mx
        ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
        ss_tot = sum((y - my) ** 2 for y in ys)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

        if best is None or r2 > best[1]:
            best = (-1.0 / slope, r2, n, xs[0], start_frac)
    return best


def _cap_fmt(idd, ln, compact=False):
    """Human-readable capillary geometry, e.g. '130 µm × 100 mm', or 'none' when
    not fitted (id or length <= 0). *compact* drops the spaces for the hint."""
    if idd is None or ln is None or idd <= 0 or ln <= 0:
        return "none"
    return f"{idd:g}µm×{ln:g}mm" if compact else f"{idd:g} µm × {ln:g} mm"


def _run_conditions(up_bar, up_degC):
    """Fields describing what produced a capture: both capillary geometries, gas,
    and the upstream *pressure/temperature* at the fire instant. Merged into every
    capture header (all outcomes) so a file is self-describing and comparable
    standalone. An unfitted upstream capillary is recorded as 0 (its 'none'
    state), so the column is always present and machine-readable."""
    with _lock:
        d_id = _state['downstream_cap_id_um']; d_ln = _state['downstream_cap_len_mm']
        u_id = _state['upstream_cap_id_um'];   u_ln = _state['upstream_cap_len_mm']
        gas  = _state['gas_species']
    return {
        'downstream_cap_id_um':     f"{d_id:g}",
        'downstream_cap_length_mm': f"{d_ln:g}",
        'upstream_cap_id_um':       f"{u_id:g}",
        'upstream_cap_length_mm':   f"{u_ln:g}",
        'gas_species':              gas,
        'upstream_bar_at_fire':  f"{up_bar:.4f}"  if up_bar  is not None else '',
        'upstream_degC_at_fire': f"{up_degC:.2f}" if up_degC is not None else '',
    }


def _capture_group_meta(group_id, shot_index, group_size):
    """Fields that tie the shots of one multi-shot `c n` run together, so files
    from the same capture are identifiable. For a single shot the group is of
    size 1. `capture_group_id` is shared by every shot in the run; the files
    also share it as a filename prefix."""
    return {
        'capture_group_id':   group_id,
        'capture_shot_index': shot_index,
        'capture_group_size': group_size,
    }


def _write_capture_csv(times, mbar, meta, stem=None):
    """Write one capture to its own CSV; return the path. `stem` sets the
    filename (without extension); defaults to a fresh timestamp."""
    if stem is None:
        stem = f"pulse_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    path = os.path.join(LOG_DIR, f"{stem}.csv")
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        for k, v in meta.items():
            w.writerow([f"# {k}", v])
        w.writerow(['t_s', 'mbar'])          # t = 0 at the fire command
        for t, p in zip(times, mbar):
            # blank = not a valid gauge reading
            w.writerow([f"{t:.5f}", f"{p:.6e}" if p is not None else ''])
    return path


def _run_capture(lj, shot_index=1, group_id=None, group_size=1,
                 pair_dt_us=None, extra=None):
    """Capture one pulse, analyse it, save it, and report to the event log.

    shot_index / group_id / group_size tie the shots of one `c n` run together;
    they are stamped into the header and shared as a filename prefix.
    pair_dt_us fires a ZC1 two-shot series dt apart instead of one pulse (the
    paired-pulse test); extra is stamped into the header and manifest.

    Returns the shot's summary dict (with 'integral' in mbar*s) for a detected
    or not-detected shot, None if there is nothing to analyse."""
    if group_id is None:
        group_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    tag  = "" if group_size <= 1 else f" [{shot_index}/{group_size}]"
    stem = (f"pulse_{group_id}_s{shot_index:02d}"
            if group_size > 1 else f"pulse_{group_id}")
    grp  = _capture_group_meta(group_id, shot_index, group_size)
    grp.update(extra or {})

    cycle_us = None
    if pair_dt_us is not None:
        if not (_zc1_serial is not None and _zc1_ok):
            log_event(f"CAPTURE{tag}: ZC1 offline — pair not fired")
            return None
        ok, cycle_us, msg = zc1_arm_pair(_zc1_serial, pair_dt_us)
        if not ok:
            log_event(f"CAPTURE{tag}: {msg}")
            return None
        if cycle_us != pair_dt_us:
            log_event(f"CAPTURE{tag}: ZC1 CYCLE time is {cycle_us} µs, "
                      f"not the {pair_dt_us} µs asked for — recorded as is")
        grp.update({'shots_per_fire': 2, 'pair_dt_us': cycle_us})
    else:
        grp.setdefault('shots_per_fire', 1)

    rate = _choose_stream_rate(lj)
    if rate is None:
        log_event(f"CAPTURE{tag}: the U3 would not stream at any rate in the "
                  f"ladder — capture unavailable")
        return None

    log_event(f"CAPTURE{tag}: streaming "
              f"{CAPTURE_PRE_S + CAPTURE_POST_S:g} s @ {rate:g} Hz")

    volts, fire_idx, fire_ok, open_us, missed, up_bar, up_degC, fire_msg = \
        _capture_stream_fire(lj, rate, pair_cycle_us=cycle_us)
    if missed:
        log_event(f"CAPTURE{tag}: {missed} samples missed by the stream")
    if fire_idx is None or len(volts) < CAPTURE_DECIMATE * 20:
        log_event(f"CAPTURE{tag}: capture failed - too few samples")
        return None
    if cycle_us is not None and not fire_ok:
        # A pair that did not fire would be analysed as a pair that delivered
        # nothing — a false "fully depleted" result. Drop it.
        log_event(f"CAPTURE{tag}: pair not fired ({fire_msg}) — discarded")
        return None
    if not fire_ok:
        log_event(f"CAPTURE{tag}: WARNING - no ZC1 ack; valve may not have fired")

    dt    = CAPTURE_DECIMATE / rate
    v_dec = _decimate(volts, CAPTURE_DECIMATE)
    i_fire = fire_idx // CAPTURE_DECIMATE
    times  = [(i - i_fire) * dt for i in range(len(v_dec))]

    # ---- is every reading a valid pressure? ---------------------------------
    # Judged on the raw samples, so a clipped peak cannot hide inside a block
    # average. A block with any invalid sample is written blank, and the shot
    # is not analysed: a baseline, peak or tau built on readings that are not
    # pressures would be a made-up number.
    raw_status = [_vacuum_gauge_status(v) for v in volts]
    bad = {}
    for st_ in raw_status:
        if st_ is not None:
            bad[st_] = bad.get(st_, 0) + 1
    if bad:
        mbar = []
        for i, v in enumerate(v_dec):
            blk = raw_status[i * CAPTURE_DECIMATE:(i + 1) * CAPTURE_DECIMATE]
            mbar.append(None if any(blk) else _voltage_to_vacuum_mbar(v))
        n_bad = sum(bad.values())
        log_event(f"CAPTURE{tag}: *** GAUGE READING NOT VALID *** "
                  f"{n_bad} of {len(volts)} samples - "
                  + "; ".join(f"{k}: {n}" for k, n in bad.items()))
        log_event(f"CAPTURE{tag}: no analysis attempted")
        bad_meta = {'detected': 0, 'gauge_invalid': 1,
                    'invalid_samples': n_bad,
                    'capture_rate_hz': rate,
                    'open_time_us': open_us,
                    'zc1_ack': int(bool(fire_ok)),
                    **grp,
                    **_run_conditions(up_bar, up_degC),
                    'samples_missed': missed}
        path = _write_capture_csv(times, mbar, bad_meta, stem=stem)
        append_manifest(bad_meta, path)
        log_event(f"CAPTURE{tag}: {os.path.basename(path)}")
        return None
    mbar = [_voltage_to_vacuum_mbar(v) for v in v_dec]

    # Baseline from the pre-trigger region, ending a little before the fire so
    # nothing from the rise leaks into it.
    guard = max(1, int(0.05 / dt))
    pre = mbar[:max(1, i_fire - guard)]
    p_base = sum(pre) / len(pre)
    # Scatter on the pre-trigger baseline sets how far down the decay can be
    # followed. p_base itself is averaged over ~1 s, so it is known far better
    # than any single point.
    sd_pre = (math.sqrt(sum((p - p_base) ** 2 for p in pre) / len(pre))
              if len(pre) > 1 else p_base * 0.01)
    fit_floor = FIT_NOISE_K * sd_pre

    post = mbar[i_fire:]
    i_pk_rel = max(range(len(post)), key=lambda i: post[i])
    peak = post[i_pk_rel]
    t_peak = times[i_fire + i_pk_rel]
    dp = peak - p_base

    # ---- did anything actually happen? --------------------------------------
    # Test 1: is the raw input live at all? A disconnected or stuck gauge gives
    # a near-constant voltage, which would otherwise be analysed as a baseline
    # with a tiny "pulse" on it.
    v_lo, v_hi = min(volts), max(volts)
    trace_live = (v_hi - v_lo) > FLAT_TRACE_V

    # Test 2: peak against baseline noise. On its own this is weak - the
    # maximum of N noisy samples grows with N - so it is a necessary condition,
    # not a sufficient one.
    peak_snr = dp / sd_pre if sd_pre > 0 else 0.0

    # Test 3: the sensitive one. Average the window just after the fire and ask
    # how many standard errors it sits above the baseline. Averaging n points
    # shrinks the noise by sqrt(n), so a real pulse scores in the hundreds
    # while noise scores about 1.
    n_win = max(5, min(len(post), int(DETECT_WINDOW_S / dt)))
    win_mean = sum(post[:n_win]) / n_win
    se = sd_pre / math.sqrt(n_win) if sd_pre > 0 else 0.0
    t_stat = (win_mean - p_base) / se if se > 0 else 0.0

    detected = (trace_live
                and peak_snr >= DETECT_PEAK_K
                and t_stat >= DETECT_T_STAT)

    # Integral of the excess over the post-fire window, in mbar*s.
    integral = sum(max(0.0, p - p_base) for p in post) * dt

    fit = _fit_decay(times[i_fire:], post, p_base, fit_floor)
    tau = s_eff = r2 = None
    fit_from = None
    if fit:
        tau, r2, npts, t_fit0, start_frac = fit
        s_eff = CHAMBER_VOLUME_L / tau
        fit_from = (t_fit0, start_frac)

    # Rise time. For an impulsive injection this is set by the gauge (~10 ms).
    # Anything much longer means gas is still arriving well after the valve
    # shut - which is exactly what a conductance limiter is for, but it breaks
    # the V*dp route to Q_pulse, because not all the gas is ever in the
    # chamber at once.
    t10 = t90 = None
    for t, p in zip(times[i_fire:], post):
        if t10 is None and p >= p_base + 0.1 * dp:
            t10 = t
        if t90 is None and p >= p_base + 0.9 * dp:
            t90 = t
            break
    t_rise = (t90 - t10) if (t10 is not None and t90 is not None) else None

    # Gas per pulse, two independent routes:
    #   Q = V * dp          needs the volume, and the true peak
    #   Q = S_eff * integral needs S_eff, but survives a rounded-off peak
    # They lean on different estimates, so agreement is a real cross-check.
    q_peak = CHAMBER_VOLUME_L * dp                      # mbar*L
    q_int  = s_eff * integral if s_eff else None        # mbar*L

    # ---- reporting ----------------------------------------------------------
    if not trace_live:
        log_event(f"CAPTURE{tag}: *** GAUGE INPUT NOT CHANGING *** "
                  f"({v_lo:.4f} to {v_hi:.4f} V across the whole window)")
        log_event(f"CAPTURE{tag}: check the FIO2 wiring and the gauge - "
                  f"no analysis attempted")
        flat_meta = {'detected': 0, 'flat_trace': 1,
                     'capture_rate_hz': rate,
                     'open_time_us': open_us,
                     'zc1_ack': int(bool(fire_ok)),
                     **grp,
                     **_run_conditions(up_bar, up_degC)}
        path = _write_capture_csv(times, mbar, flat_meta, stem=stem)
        append_manifest(flat_meta, path)
        return None

    if not detected:
        log_event(f"CAPTURE{tag}: *** NO PULSE DETECTED ***")
        log_event(f"CAPTURE{tag}: peak excess {dp:.2e} = {peak_snr:.1f} x noise "
                  f"(need {DETECT_PEAK_K:g}), window t = {t_stat:.1f} "
                  f"(need {DETECT_T_STAT:g})")
        # An upper bound is still a real result: whatever the valve delivered,
        # it was smaller than this.
        ub = DETECT_PEAK_K * sd_pre
        log_event(f"CAPTURE{tag}: upper bound - any dp was below {ub:.2e} mbar "
                  f"({100*ub/CHAMBER_LIMIT_MBAR:.2f} % of limit)")
        if not fire_ok:
            log_event(f"CAPTURE{tag}: the ZC1 did not acknowledge - "
                      f"the valve probably never fired")
        else:
            log_event(f"CAPTURE{tag}: ZC1 acknowledged, so the valve actuated "
                      f"but delivered no measurable gas - check upstream "
                      f"pressure and open time against the threshold")
        meta = {
            'detected': 0,
            'capture_rate_hz': rate,
            'analysis_rate_hz': 1.0 / dt,
            'open_time_us': open_us,
            'zc1_ack': int(bool(fire_ok)),
            **grp,
            **_run_conditions(up_bar, up_degC),
            'p_base_mbar': f"{p_base:.6e}",
            'baseline_sd_mbar': f"{sd_pre:.6e}",
            'peak_snr': f"{peak_snr:.2f}",
            'detect_t_stat': f"{t_stat:.2f}",
            'dp_upper_bound_mbar': f"{ub:.6e}",
            'integral_mbar_s': f"{integral:.6e}",
            'samples_missed': missed,
        }
        path = _write_capture_csv(times, mbar, meta, stem=stem)
        append_manifest(meta, path)
        log_event(f"CAPTURE{tag}: {os.path.basename(path)}")
        run = dict(meta, path=path, peak=None, tau=None, detected=False,
                   integral=integral)
        _capture_runs.append(run)
        return run

    pct = 100.0 * peak / CHAMBER_LIMIT_MBAR
    log_event(f"CAPTURE{tag}: peak {peak:.3e} mbar  "
              f"({pct:.1f}% of {CHAMBER_LIMIT_MBAR:.0e} limit)  "
              f"SNR {peak_snr:.0f}")
    log_event(f"CAPTURE{tag}: base {p_base:.3e}  dp {dp:.3e}  "
              f"t_peak {t_peak*1000:.0f} ms")
    if t_rise is not None:
        log_event(f"CAPTURE{tag}: rise 10-90% {t_rise*1000:.0f} ms "
                  f"(valve open {open_us} us)")
    if tau:
        log_event(f"CAPTURE{tag}: tau {tau*1000:.1f} ms (r2 {r2:.4f}, "
                  f"fit from {fit_from[0]*1000:.0f} ms)  S_eff {s_eff:.1f} L/s")
        log_event(f"CAPTURE{tag}: Q_pulse {q_int:.3e} mbar.L (integral)  "
                  f"{q_peak:.3e} (V.dp)")
        if t_rise is not None and t_rise > 0.15 * tau:
            log_event(f"CAPTURE{tag}: rise is {100*t_rise/tau:.0f}% of tau - "
                      f"injection is not impulsive, trust the integral route")
        elif q_int > 0 and not (0.5 < q_peak / q_int < 2.0):
            log_event(f"CAPTURE{tag}: routes disagree by "
                      f"{q_peak/q_int:.1f}x - check V and S_eff")
    else:
        log_event(f"CAPTURE{tag}: pulse detected but no clean decay to fit")
        log_event(f"CAPTURE{tag}: peak excess {dp:.2e} vs noise floor "
                  f"{fit_floor:.2e} - too little range for tau; "
                  f"peak and dp are still measured")
    if peak >= CHAMBER_LIMIT_MBAR:
        log_event(f"CAPTURE{tag}: *** PEAK EXCEEDED THE LIMIT ***")
    if t_peak < 3 * dt:
        log_event(f"CAPTURE{tag}: peak within {t_peak*1000:.0f} ms of fire - "
                  f"rise may be gauge-limited, treat peak as a lower bound")

    meta = {
        'detected': 1,
        'peak_snr': f"{peak_snr:.1f}",
        'detect_t_stat': f"{t_stat:.1f}",
        'capture_rate_hz': rate,
        'analysis_rate_hz': 1.0 / dt,
        'open_time_us': open_us,
        'zc1_ack': int(bool(fire_ok)),
        **grp,
        **_run_conditions(up_bar, up_degC),
        'p_base_mbar': f"{p_base:.6e}",
        'baseline_sd_mbar': f"{sd_pre:.6e}",
        'peak_mbar': f"{peak:.6e}",
        'dp_mbar': f"{dp:.6e}",
        't_peak_s': f"{t_peak:.4f}",
        't_rise_10_90_s': f"{t_rise:.4f}" if t_rise is not None else '',
        'fit_start_s': f"{fit_from[0]:.4f}" if fit_from else '',
        'fit_start_frac': fit_from[1] if fit_from else '',
        'integral_mbar_s': f"{integral:.6e}",
        'tau_s': f"{tau:.6f}" if tau else '',
        'r2': f"{r2:.5f}" if r2 else '',
        'chamber_volume_l': CHAMBER_VOLUME_L,
        's_eff_l_s': f"{s_eff:.3f}" if s_eff else '',
        'q_pulse_mbar_l_integral': f"{q_int:.6e}" if q_int else '',
        'q_pulse_mbar_l_v_dp': f"{q_peak:.6e}",
        'samples_missed': missed,
    }
    path = _write_capture_csv(times, mbar, meta, stem=stem)
    append_manifest(meta, path)
    log_event(f"CAPTURE{tag}: {os.path.basename(path)}")

    run = dict(meta)
    run['path'] = path
    run['peak'] = peak
    run['tau'] = tau
    run['detected'] = True
    run['integral'] = integral
    _capture_runs.append(run)
    return run


# ═══════════════════════════════════════════════════════════════════════════════
# PAIRED-PULSE TEST (pp) — capillary as brake (A) or reservoir (B)?
# ═══════════════════════════════════════════════════════════════════════════════
# Plan: PAIR_REPEATS rounds; each round visits every dt once, in shuffled
# order, as [single shot, pair dt apart]. The single is the pair's reference,
# taken seconds before it, so upstream drift cancels in the ratio; shuffling
# keeps drift over the run from masquerading as recovery with dt. The plan runs
# on the LabJack thread one capture at a time, with monitoring between them.
#
#   Q2/Q1 = I_pair / I_single - 1          (I = integral of the excess, mbar*s)
#
# S_eff and the chamber volume cancel, so the ratio needs no calibration. The
# mean ratio per dt is fitted with  Q2/Q1 = 1 - a * exp(-dt / tau):
#   a ~ 0       A: the capillary is a brake, gas flows through it freely
#   a > 0       B: the second shot finds the reservoir near the valve depleted;
#               tau is its refill time through the upstream capillary.

PAIRTEST_COLUMNS = [
    'pairtest_dt_ms', 'dt_actual_ms', 'repeat', 'open_time_us',
    'upstream_bar_single', 'upstream_bar_pair',
    'integral_single_mbar_s', 'integral_pair_mbar_s', 'q2_over_q1',
    'single_trace_file', 'pair_trace_file',
]


def _pairtest_new(dts_ms, open_us):
    """Build a paired-pulse test: the plan, the per-pair CSV and its state."""
    tid = datetime.now().strftime('%Y%m%d_%H%M%S')
    plan = deque()
    for k in range(1, PAIR_REPEATS + 1):
        order = list(dts_ms)
        random.shuffle(order)
        for dt in order:
            plan.append({'dt_ms': dt, 'k': k, 'role': 'single'})
            plan.append({'dt_ms': dt, 'k': k, 'role': 'pair'})
    path = os.path.join(LOG_DIR, f"pvd-pairtest_{tid}.csv")
    with open(path, 'w', newline='') as f:
        csv.writer(f).writerow(PAIRTEST_COLUMNS)
    with _lock:
        ident = {k: _state[k] for k in (
            'upstream_cap_id_um', 'upstream_cap_len_mm',
            'downstream_cap_id_um', 'downstream_cap_len_mm', 'gas_species')}
    return {'id': tid, 'plan': plan, 'n_total': len(plan), 'done': 0,
            'refs': {}, 'rows': [], 'next_t': 0.0, 'csv': path,
            'dts_ms': list(dts_ms), 'open_us': open_us, 'ident': ident}


def _pairtest_step(lj, pt, step):
    """Run one capture of the plan; after a pair, work out its Q2/Q1."""
    pt['done'] += 1
    dt_ms, k, role = step['dt_ms'], step['k'], step['role']
    run = _run_capture(
        lj, shot_index=pt['done'], group_id=f"pp{pt['id']}",
        group_size=pt['n_total'],
        pair_dt_us=(int(round(dt_ms * 1000)) if role == 'pair' else None),
        extra={'pairtest_dt_ms': f"{dt_ms:g}", 'pairtest_repeat': k,
               'pairtest_role': role})
    if role == 'single':
        pt['refs'][(dt_ms, k)] = run
        return
    ref = pt['refs'].pop((dt_ms, k), None)
    if (run is None or ref is None or not ref.get('detected')
            or not ref.get('integral') or ref['integral'] <= 0):
        log_event(f"PAIR TEST: dt {dt_ms:g} ms, repeat {k}: no usable "
                  f"pair or single reference — skipped")
        return
    ratio = run['integral'] / ref['integral'] - 1.0
    dt_act = float(run.get('pair_dt_us') or dt_ms * 1000) / 1000.0
    row = {
        'pairtest_dt_ms': f"{dt_ms:g}", 'dt_actual_ms': f"{dt_act:g}",
        'repeat': k, 'open_time_us': run.get('open_time_us', ''),
        'upstream_bar_single': ref.get('upstream_bar_at_fire', ''),
        'upstream_bar_pair': run.get('upstream_bar_at_fire', ''),
        'integral_single_mbar_s': f"{ref['integral']:.6e}",
        'integral_pair_mbar_s': f"{run['integral']:.6e}",
        'q2_over_q1': f"{ratio:.5f}",
        'single_trace_file': os.path.basename(ref.get('path', '')),
        'pair_trace_file': os.path.basename(run.get('path', '')),
    }
    pt['rows'].append(dict(row, _dt=dt_act, _ratio=ratio))
    try:
        with open(pt['csv'], 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=PAIRTEST_COLUMNS).writerow(row)
    except Exception as e:
        log_event(f"PAIR TEST: could not append to {os.path.basename(pt['csv'])} ({e})")
    log_event(f"PAIR TEST [{pt['done']}/{pt['n_total']}]: dt {dt_act:g} ms  "
              f"Q2/Q1 = {ratio:.3f}")


def _fit_recovery(points):
    """Fit Q2/Q1 = 1 - a*exp(-dt/tau) to [(dt_s, mean, se), ...].

    tau is searched on a log grid over PAIR_FIT_TAU_S; for each tau the best a
    is linear least squares, weighted by 1/se² when every point has an se.
    Returns (a, a_se, tau_s) or None. a_se is scaled up by the fit's reduced
    chi-squared when the scatter is larger than the error bars say."""
    if len(points) < 2:
        return None
    weighted = all(se and se > 0 for _, _, se in points)
    lo, hi = PAIR_FIT_TAU_S
    best = None
    for i in range(400):
        tau = lo * (hi / lo) ** (i / 399)
        sxx = sxy = 0.0
        for dt, m, se in points:
            w = 1.0 / se ** 2 if weighted else 1.0
            x = math.exp(-dt / tau)
            sxx += w * x * x
            sxy += w * x * (1.0 - m)
        if sxx <= 0:
            continue
        a = sxy / sxx
        chi = sum((1.0 / se ** 2 if weighted else 1.0)
                  * ((1.0 - m) - a * math.exp(-dt / tau)) ** 2
                  for dt, m, se in points)
        if best is None or chi < best[0]:
            best = (chi, a, tau, sxx)
    if best is None:
        return None
    chi, a, tau, sxx = best
    dof = len(points) - 2
    red = chi / dof if dof > 0 else 1.0
    a_se = math.sqrt(1.0 / sxx) * (math.sqrt(red) if (red > 1 or not weighted) else 1.0)
    return a, a_se, tau


def _pairtest_finish(pt, cancelled=False):
    """Average the pairs per dt, fit the recovery and give the A/B verdict, in
    the event log and in pvd-pairtest-summary_<id>.csv."""
    rows = pt['rows']
    head = "PAIR TEST: stopped early — " if cancelled else "PAIR TEST: done — "
    log_event(f"{head}{len(rows)} usable pairs of {pt['n_total'] // 2} planned")
    by_dt = {}
    for r in rows:
        by_dt.setdefault(r['_dt'], []).append(r['_ratio'])
    table = []
    for dt in sorted(by_dt):
        v = by_dt[dt]
        n = len(v)
        m = sum(v) / n
        sd = math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1)) if n > 1 else None
        se = sd / math.sqrt(n) if sd is not None else None
        table.append((dt, n, m, sd, se))
        log_event(f"PAIR TEST: dt {dt:>7g} ms  n {n}  Q2/Q1 {m:.3f}"
                  + (f" ± {se:.3f} (SE)" if se is not None else ""))

    fit = _fit_recovery([(dt / 1000.0, m, se) for dt, n, m, sd, se in table])
    if fit is None or len(table) < 3:
        verdict = "too few dt values with usable pairs for a verdict"
        fit_s = ""
    else:
        a, a_se, tau = fit
        fit_s = (f"Q2/Q1 = 1 - a*exp(-dt/tau): a = {a:.3f} ± {a_se:.3f}, "
                 f"tau = {tau * 1000:.3g} ms")
        dts_s = [dt / 1000.0 for dt, *_ in table]
        if a <= max(2 * a_se, PAIR_A_MIN):
            verdict = (f"A - brake: the second dose drops by no more than "
                       f"{100 * max(2 * a_se, PAIR_A_MIN):.0f} % at short dt, so gas "
                       f"flows through the capillary during the pulse")
        else:
            verdict = (f"B - reservoir: the second dose drops by {100 * a:.0f} % at "
                       f"short dt and recovers with tau = {tau * 1000:.3g} ms "
                       f"(refill of the gas near the valve)")
            if tau < 0.5 * min(dts_s) or tau > 2 * max(dts_s):
                verdict += " - tau lies outside the dt range, extend the sweep"
    log_event(f"PAIR TEST: {fit_s}" if fit_s else "PAIR TEST: no fit")
    log_event(f"PAIR TEST: verdict {verdict}")
    log_event("PAIR TEST: control - repeat with 'cap up none'; a drop there "
              "comes from the valve or the ZC1, not from the capillary")

    path = os.path.join(LOG_DIR, f"pvd-pairtest-summary_{pt['id']}.csv")
    try:
        ident = pt['ident']
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            for k, v in (('verdict', verdict), ('fit', fit_s),
                         ('cancelled', int(cancelled)),
                         ('open_time_us', pt['open_us']),
                         ('pairs_per_dt_planned', PAIR_REPEATS),
                         ('upstream_cap_id_um', ident['upstream_cap_id_um']),
                         ('upstream_cap_length_mm', ident['upstream_cap_len_mm']),
                         ('downstream_cap_id_um', ident['downstream_cap_id_um']),
                         ('downstream_cap_length_mm', ident['downstream_cap_len_mm']),
                         ('gas_species', ident['gas_species']),
                         ('pairs_file', os.path.basename(pt['csv']))):
                w.writerow([f"# {k}", v])
            w.writerow(['dt_ms', 'n_pairs', 'q2_over_q1_mean',
                        'q2_over_q1_sd', 'q2_over_q1_se'])
            for dt, n, m, sd, se in table:
                w.writerow([f"{dt:g}", n, f"{m:.5f}",
                            f"{sd:.5f}" if sd is not None else '',
                            f"{se:.5f}" if se is not None else ''])
        log_event(f"PAIR TEST: {os.path.basename(path)}")
    except Exception as e:
        log_event(f"PAIR TEST: could not write the summary ({e})")


def _summarise_runs(n_last):
    """Log mean and spread of the last *n_last* captures.

    Non-detections are counted but excluded from the statistics - averaging a
    real pulse together with a shot that delivered nothing would report a
    meaningless middle value.
    """
    runs = _capture_runs[-n_last:]
    good = [r for r in runs if r.get('detected')]
    n_bad = len(runs) - len(good)
    if n_bad:
        log_event(f"SUMMARY: {n_bad} of {len(runs)} shots had NO DETECTABLE "
                  f"PULSE - excluded from the statistics below")
    if not good:
        log_event("SUMMARY: no shots delivered a measurable pulse")
        return
    peaks = [r['peak'] for r in good]
    if len(peaks) < 2:
        return
    mean = sum(peaks) / len(peaks)
    sd = math.sqrt(sum((p - mean) ** 2 for p in peaks) / (len(peaks) - 1))
    log_event(f"SUMMARY: {len(peaks)} shots  peak {mean:.3e} +/- {sd:.1e} mbar "
              f"({100*sd/mean:.1f}%)  max {max(peaks):.3e}")
    taus = [r['tau'] for r in good if r['tau']]
    if len(taus) >= 2:
        tm = sum(taus) / len(taus)
        tsd = math.sqrt(sum((t - tm) ** 2 for t in taus) / (len(taus) - 1))
        log_event(f"SUMMARY: tau {tm*1000:.1f} +/- {tsd*1000:.1f} ms  "
                  f"S_eff {CHAMBER_VOLUME_L/tm:.1f} L/s")
    worst = max(peaks)
    log_event(f"SUMMARY: worst-case peak is "
              f"{100*worst/CHAMBER_LIMIT_MBAR:.1f}% of the "
              f"{CHAMBER_LIMIT_MBAR:.0e} limit")


# ═══════════════════════════════════════════════════════════════════════════════
# ZC1 SUPERVISOR THREAD  — reconnects automatically on disconnect / fault
# ═══════════════════════════════════════════════════════════════════════════════

def zc1_supervisor_thread():
    """Watches _zc1_ok.  When False, scans for the ZC1, (re)connects, and
    restores _zc1_serial so the rest of the programme can use it without
    knowing a reconnect happened.

    On reconnect it also re-applies the current open_time_us so the ZC1
    EEPROM state matches what the driver believes is set.
    """
    global _zc1_serial, _zc1_ok
    _retry_delay = 5.0   # seconds between scan attempts

    while not _stop.is_set():
        if _zc1_ok:
            _stop.wait(timeout=1.0)
            continue

        # Close stale handle if present
        with _zc1_lock:
            if _zc1_serial is not None:
                try:
                    _zc1_serial.close()
                except Exception:
                    pass
                _zc1_serial = None

        log_event("ZC1 offline — scanning for device…")
        port, ser = _detect_zc1()

        if ser is None:
            _stop.wait(timeout=_retry_delay)
            continue

        # Re-apply the open-time the driver currently knows about
        with _lock:
            current_ot = _state['open_time_us']
        zc1_set_open_time(ser, current_ot)

        _zc1_serial = ser
        _zc1_ok     = True
        log_event(f"ZC1 online · {port}  (open={current_ot} µs)")
        _stop.wait(timeout=1.0)


# ═══════════════════════════════════════════════════════════════════════════════
# VALVE DRIVE THREAD  — automatic, independent of the logging cadence
# ═══════════════════════════════════════════════════════════════════════════════

def valve_drive_thread():
    next_t = None
    while not _stop.is_set():
        if not _drive_on.is_set():
            _drive_on.wait(timeout=0.2)
            next_t = None
            continue
        with _lock:
            rate = _state['drive_rate_hz']
        period = 1.0 / rate if rate > 0 else 1.0
        now = time.time()
        if next_t is None:
            next_t = now
        fire_one()
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            _stop.wait(timeout=sleep_for)
        else:
            next_t = time.time()


# ═══════════════════════════════════════════════════════════════════════════════
# CSV LOGGING THREAD  — drift-free 0.5 s cadence, immediate first row
#
# One output file per session: pvd-log_<ts>.csv
#
# The logger flushes on a fixed cadence, but writes one row per raw Keller
# sample buffered since the last flush, so no upstream sample is dropped. Pulse
# events that occurred in the window are embedded (semicolon-separated, with
# their own µs timestamps) on the last row of the flush so everything needed for
# analysis is in a single file.
#
# Columns:
#   timestamp, keller_pressure_bar (raw), keller_temperature_degC (raw),
#   n_keller_samples, vacuum_chamber_mbar, pulses_interval, pulses_total,
#   open_time_us, pulse_timestamps_us, pulse_open_times_us, pulse_acks,
#   downstream_cap_id_um, downstream_cap_length_mm,
#   upstream_cap_id_um, upstream_cap_length_mm, gas_species, vacuum_status
#
# vacuum_chamber_mbar is blank when the gauge reading is not valid (outside
# the IKR 270's signal range, or the LabJack input saturated); vacuum_status
# then says why.
# ═══════════════════════════════════════════════════════════════════════════════

def logger_thread():
    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)

        # Each periodic row covers one LOG_INTERVAL_S window.
        # pulse_timestamp_us / pulse_open_us / pulse_ack: comma-separated values
        # when one or more pulses fired in the interval, empty otherwise.
        writer.writerow([
            'timestamp',
            'keller_pressure_bar',     # raw sample value for this row (blank if no sample)
            'keller_temperature_degC', # raw sample value for this row (blank if no sample)
            'n_keller_samples',        # 1 = this row is a raw sample; 0 = interval with no sample
            'vacuum_chamber_mbar',
            'pulses_interval',
            'pulses_total',
            'open_time_us',
            'pulse_timestamps_us',     # semicolon-separated microsecond timestamps
            'pulse_open_times_us',     # semicolon-separated open times
            'pulse_acks',              # semicolon-separated 1/0 ack flags
            'downstream_cap_id_um',     # downstream limiter bore in force for this row
            'downstream_cap_length_mm', # downstream limiter length in force for this row
            'upstream_cap_id_um',       # upstream capillary bore (0 = none)
            'upstream_cap_length_mm',   # upstream capillary length (0 = none)
            'gas_species',              # gas in force for this row
            'vacuum_status',            # blank = valid reading, else why vacuum_chamber_mbar is blank
        ])
        f.flush()

        start = time.time()
        n = 0
        while not _stop.is_set():
            with _lock:
                # Drain every buffered Keller sample — each becomes its own row,
                # so no sample is dropped whatever the flush cadence.
                pending = _keller_log_pending[:]
                _keller_log_pending.clear()
                # Bound the GUI-readout buffers (they feed the live mean only).
                _state['keller_pressure_samples']    = []
                _state['keller_temperature_samples'] = []

                vac    = _state['vacuum_chamber_mbar']
                vac_st = _state['vacuum_status'] or ''
                p_int  = _state['pulse_interval']
                p_tot  = _state['pulses_total']
                ot     = _state['open_time_us']
                d_id   = _state['downstream_cap_id_um']
                d_ln   = _state['downstream_cap_len_mm']
                u_id   = _state['upstream_cap_id_um']
                u_ln   = _state['upstream_cap_len_mm']
                gas    = _state['gas_species']
                _state['pulse_interval'] = 0

                events = _state['pulse_events']
                _state['pulse_events'] = []

            if events:
                pulse_ts   = ';'.join(e[0] for e in events)
                pulse_ots  = ';'.join(str(e[1]) for e in events)
                pulse_acks = ';'.join('1' if e[2] else '0' for e in events)
            else:
                pulse_ts = pulse_ots = pulse_acks = ''

            d_id_s, d_ln_s = f"{d_id:g}", f"{d_ln:g}"
            u_id_s, u_ln_s = f"{u_id:g}", f"{u_ln:g}"

            def _row(ts, p, t, n_k, with_pulses):
                # Pulses carry their own µs timestamps, so attaching the interval's
                # pulse list to a single row (not every row) keeps them counted once.
                return [
                    ts, p, t, n_k, vac,
                    (p_int if with_pulses else 0), p_tot, ot,
                    (pulse_ts   if with_pulses else ''),
                    (pulse_ots  if with_pulses else ''),
                    (pulse_acks if with_pulses else ''),
                    d_id_s, d_ln_s, u_id_s, u_ln_s, gas, vac_st,
                ]

            if pending:
                last = len(pending) - 1
                for i, (sts, sp, st) in enumerate(pending):
                    writer.writerow(_row(sts, sp, st, 1, with_pulses=(i == last)))
            else:
                # No Keller sample this interval (e.g. sensor offline) — still emit
                # one row so vacuum and any pulses are logged.
                ts = datetime.now().isoformat(timespec='milliseconds')
                writer.writerow(_row(ts, None, None, 0, with_pulses=True))
            f.flush()

            n += 1
            target = start + n * LOG_INTERVAL_S
            sleep_for = target - time.time()
            if sleep_for < 0:
                n = max(n, math.ceil((time.time() - start) / LOG_INTERVAL_S))
                target = start + n * LOG_INTERVAL_S
                sleep_for = max(0.0, target - time.time())
            _stop.wait(timeout=sleep_for)


# ═══════════════════════════════════════════════════════════════════════════════
# GUI
# ═══════════════════════════════════════════════════════════════════════════════
BG     = "#0c0c0c"
TEXT   = "#d0d0d0"
BRIGHT = "#ffffff"
DIM    = "#505050"
BORDER = "#2a2a2a"
WARN   = "#ff4040"
GRID   = "#1a1a1a"
VAC_LINE = "#2f8cff"   # vacuum chamber pressure (blue): trace and readout, as in the TE-valve driver


class PVDGui:
    """
    Window 1 — Live Log    : text readouts + pressure chart
    Window 2 — Valve Driver: terminal prompt, type commands
                              v        fire one pulse
                              d        toggle auto-drive
                              r <Hz>   set drive rate
                              t <µs>   set open time
                              pp [dt]  paired-pulse test (A brake / B reservoir)
                              q        quit
    """

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("PVD — Live Log")
        self.root.configure(bg=BG)
        self.root.geometry("640x800")
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown)

        M = ("Consolas", "Menlo", "Courier New", "DejaVu Sans Mono", "monospace")
        self.f = self._pick_font(M, 11)

        self._build_log_window()
        self._build_driver_window()
        self.root.after(150, self._poll)

    def _pick_font(self, families, size, weight="normal"):
        available = set(tkfont.families())
        fam = next((f for f in families if f in available), families[-1])
        return tkfont.Font(family=fam, size=size, weight=weight)

    # ── window 1: live log ──────────────────────────────────────────────────
    def _build_log_window(self):
        outer = tk.Frame(self.root, bg=BG)
        outer.pack(fill="both", expand=True, padx=8, pady=8)

        # status block (redrawn each poll)
        self.status_text = tk.Text(outer, bg=BG, fg=TEXT, font=self.f,
                                   height=8, bd=0, highlightthickness=0,
                                   state="disabled", wrap="none", cursor="arrow")
        self.status_text.pack(fill="x")
        self.status_text.tag_config("bright", foreground=BRIGHT)
        self.status_text.tag_config("dim",    foreground=DIM)
        self.status_text.tag_config("ok",     foreground=BRIGHT)
        self.status_text.tag_config("err",    foreground=WARN)
        self.status_text.tag_config("vac",    foreground=VAC_LINE)

        # charts
        tk.Label(outer, text=f"─── upstream pressure (bar)  last {CHART_SECONDS}s",
                 font=self.f, fg=DIM, bg=BG, anchor="w").pack(fill="x")
        self.canvas = tk.Canvas(outer, bg=BG, height=100,
                                highlightbackground=BORDER, highlightthickness=1)
        self.canvas.pack(fill="both", expand=True)

        tk.Label(outer, text=f"─── upstream temperature (°C)  last {CHART_SECONDS}s",
                 font=self.f, fg=DIM, bg=BG, anchor="w").pack(fill="x")
        self.temp_canvas = tk.Canvas(outer, bg=BG, height=100,
                                     highlightbackground=BORDER, highlightthickness=1)
        self.temp_canvas.pack(fill="both", expand=True)

        tk.Label(outer, text=f"─── vacuum chamber (mbar, log)  last {CHART_SECONDS}s",
                 font=self.f, fg=DIM, bg=BG, anchor="w").pack(fill="x")
        self.vac_canvas = tk.Canvas(outer, bg=BG, height=100,
                                    highlightbackground=BORDER, highlightthickness=1)
        self.vac_canvas.pack(fill="both", expand=True)

        # event log
        tk.Label(outer, text="─── event log",
                 font=self.f, fg=DIM, bg=BG, anchor="w").pack(fill="x")
        self.logtext = tk.Text(outer, bg=BG, fg=TEXT, font=self.f,
                               height=6, bd=0, highlightthickness=0,
                               state="disabled", wrap="none", cursor="arrow")
        self.logtext.pack(fill="both", expand=True)

        tk.Label(outer, text=f"─── log: {LOG_FILE}",
                 font=self.f, fg=DIM, bg=BG, anchor="w").pack(fill="x")

    # ── window 2: driver terminal ───────────────────────────────────────────
    def _build_driver_window(self):
        win = tk.Toplevel(self.root)
        win.title("PVD — Valve Driver")
        win.configure(bg=BG)
        win.protocol("WM_DELETE_WINDOW", self.shutdown)
        win.geometry("+730+80")
        self.drive_win = win

        outer = tk.Frame(win, bg=BG)
        outer.pack(fill="both", expand=True, padx=8, pady=8)

        # scrolling output
        self.cmd_text = tk.Text(outer, bg=BG, fg=TEXT, font=self.f,
                                height=20, bd=0, highlightthickness=0,
                                state="disabled", wrap="none", cursor="arrow")
        self.cmd_text.pack(fill="both", expand=True)
        self.cmd_text.tag_config("bright", foreground=BRIGHT)
        self.cmd_text.tag_config("dim",    foreground=DIM)
        self.cmd_text.tag_config("ok",     foreground=BRIGHT)
        self.cmd_text.tag_config("err",    foreground=WARN)

        # status hint
        self.hint_var = tk.StringVar()
        tk.Label(outer, textvariable=self.hint_var, font=self.f, fg=DIM,
                 bg=BG, anchor="w").pack(fill="x")

        # command input
        inrow = tk.Frame(outer, bg=BG)
        inrow.pack(fill="x", pady=(4, 0))
        tk.Label(inrow, text=">", font=self.f, fg=BRIGHT, bg=BG).pack(side="left")
        self.cmd_entry = tk.Entry(inrow, font=self.f, bg=BG, fg=BRIGHT,
                                  insertbackground=BRIGHT, relief="flat", bd=0,
                                  highlightbackground=BORDER, highlightthickness=1)
        self.cmd_entry.pack(side="left", fill="x", expand=True,
                            ipady=4, ipadx=4, padx=(4, 0))
        self.cmd_entry.bind("<Return>", lambda e: self._on_cmd())
        self.cmd_entry.focus_set()

        self._driver_print("PVD — Valve Driver")
        self._driver_print("Pulse Valve Driver · ZC1 controller")
        self._driver_print("")
        self._driver_print("  v              fire one pulse (no capture)")
        self._driver_print("  c [n]          capture n pulses at high rate")
        self._driver_print("  pp [dt ms …]   paired-pulse test: capillary as brake (A) or reservoir (B)")
        self._driver_print("  pp stop        end the paired-pulse test and summarise")
        self._driver_print("  t <µs>         set open time")
        self._driver_print("  cap <up|down> <id> <len>  set a capillary (bore µm, length mm)")
        self._driver_print("  cap up none    record that no upstream capillary is fitted")
        self._driver_print("  gas <species>  set gas (N2, H2, He, Ar…)")
        self._driver_print("  q              quit")
        self._driver_print("")
        self._print_identity_banner()
        self._update_hint()

    def _print_identity_banner(self):
        """Prominent, visually distinct restatement of what will be stamped into
        every capture and row. This is the staleness defence: if the hardware was
        swapped and this is wrong, it is impossible to miss. Correct it with the
        cap / gas commands before capturing."""
        with _lock:
            d_id = _state['downstream_cap_id_um']; d_ln = _state['downstream_cap_len_mm']
            u_id = _state['upstream_cap_id_um'];   u_ln = _state['upstream_cap_len_mm']
            gas  = _state['gas_species']
        self._driver_print("  ┄┄┄ RUN IDENTITY — check before capturing ┄┄┄", "ok")
        self._driver_print(f"    downstream cap: {_cap_fmt(d_id, d_ln)}", "ok")
        self._driver_print(f"    upstream cap:   {_cap_fmt(u_id, u_ln)}", "ok")
        self._driver_print(f"    gas:            {gas}", "ok")
        self._driver_print("    upstream P:     read live from the Keller, stamped at fire", "dim")
        self._driver_print("    wrong?  cap <up|down> <id_µm> <len_mm>   /   gas <species>", "dim")
        self._driver_print("  ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄", "ok")

    def _driver_print(self, line, tag=None):
        self.cmd_text.configure(state="normal")
        if tag:
            self.cmd_text.insert("end", line + "\n", tag)
        else:
            self.cmd_text.insert("end", line + "\n")
        self.cmd_text.see("end")
        self.cmd_text.configure(state="disabled")

    def _update_hint(self):
        with _lock:
            ot   = _state['open_time_us']
            d_id = _state['downstream_cap_id_um']; d_ln = _state['downstream_cap_len_mm']
            u_id = _state['upstream_cap_id_um'];   u_ln = _state['upstream_cap_len_mm']
            gas  = _state['gas_species']
        ident = (f"D:{_cap_fmt(d_id, d_ln, compact=True)} "
                 f"U:{_cap_fmt(u_id, u_ln, compact=True)} · {gas}")
        pt = _pairtest
        if pt is not None:
            self.hint_var.set(f"{ident}  |  open={ot} µs  |  PAIR TEST "
                              f"{pt['done']}/{pt['n_total']}  (pp stop to end)")
        elif _capture_busy:
            self.hint_var.set(f"{ident}  |  open={ot} µs  |  CAPTURING…")
        elif _capture_pending > 0:
            self.hint_var.set(f"{ident}  |  open={ot} µs  |  {_capture_pending} shot(s) queued")
        else:
            self.hint_var.set(f"{ident}  |  open={ot} µs  | v  c[n]  pp  t  cap  gas  q")

    def _on_cmd(self):
        raw = self.cmd_entry.get().strip()
        self.cmd_entry.delete(0, "end")
        if not raw:
            return
        self._driver_print(f"> {raw}", "dim")
        parts     = raw.lower().split()
        raw_parts = raw.split()          # original case, for free-text arguments
        cmd   = parts[0]

        if cmd == "q":
            self.shutdown()

        elif cmd == "pp":
            self._pairtest_cmd(parts[1:])

        elif cmd == "v":
            if _pairtest is not None:
                self._driver_print("  pair test running — 'pp stop' first", "err")
                return
            ok, ot = fire_one()
            status = "OK" if ok else ("no ACK" if _zc1_ok else "no device")
            self._driver_print(f"  pulse fired  {ot} µs  {status}",
                               "ok" if ok else "err")
            log_event(f"pulse fired  {ot} µs  {status}")

        elif cmd == "c":
            global _capture_pending, _summary_window, _capture_group_id, \
                   _capture_group_size, _capture_group_index
            if _capture_busy or _capture_request.is_set() or _pairtest is not None:
                self._driver_print("  capture already running", "err")
                return
            if not _labjack_ok:
                self._driver_print("  no LabJack — capture unavailable", "err")
                return
            n = 1
            if len(parts) > 1:
                try:
                    n = max(1, min(50, int(parts[1])))
                except ValueError:
                    self._driver_print("  usage: c [n]  (1 – 50 shots)", "err")
                    return
            _capture_pending = n
            _summary_window  = n
            _capture_group_id    = datetime.now().strftime('%Y%m%d_%H%M%S')
            _capture_group_size  = n
            _capture_group_index = 0
            _capture_request.set()
            window = CAPTURE_PRE_S + CAPTURE_POST_S
            total  = n * window + (n - 1) * CAPTURE_SETTLE_S
            self._driver_print(
                f"  capture armed — {n} shot(s), ~{total:.0f} s total", "ok")
            self._driver_print(
                "  the valve fires automatically inside each window", "dim")

        elif cmd in ("d", "r"):
            self._driver_print(
                "  auto-drive is disabled — use 'v' for single pulses", "dim")

        elif cmd == "t":
            if len(parts) > 1:
                self._set_open(parts[1])
            else:
                self._driver_print("  usage: t <µs>  (50 – 5 000 000)")

        elif cmd == "cap":
            # cap <up|down> <id_µm> <len_mm>   |   cap up none
            UP   = ("up", "u", "upstream")
            DOWN = ("down", "d", "downstream")
            if len(parts) == 3 and parts[1] in UP and parts[2] == "none":
                self._clear_upstream_cap()
            elif len(parts) == 4 and parts[1] in (UP + DOWN):
                side = "up" if parts[1] in UP else "down"
                self._set_capillary(side, parts[2], parts[3])
            else:
                self._driver_print(
                    "  usage: cap <up|down> <id_µm> <len_mm>   |   cap up none", "err")

        elif cmd == "gas":
            if len(raw_parts) == 2:
                self._set_gas(raw_parts[1])
            else:
                self._driver_print("  usage: gas <species>  (e.g. N2, H2, He, Ar)", "err")

        else:
            self._driver_print(f"  unknown: '{cmd}'", "err")

    def _pairtest_cmd(self, args):
        """pp [dt_ms ...] starts the paired-pulse test; pp stop ends it early
        (the pairs so far are still summarised)."""
        global _pairtest
        if args and args[0] == "stop":
            if _pairtest is None:
                self._driver_print("  no pair test running", "dim")
            else:
                _pairtest_cancel.set()
                self._driver_print("  pair test stops after the current capture", "ok")
            return
        if _pairtest is not None:
            self._driver_print("  pair test already running — 'pp stop' to end it", "err")
            return
        if _capture_busy or _capture_request.is_set() or _capture_pending > 0:
            self._driver_print("  capture running — wait for it to finish", "err")
            return
        if not _labjack_ok:
            self._driver_print("  no LabJack — pair test unavailable", "err")
            return
        if not _zc1_ok:
            self._driver_print("  no ZC1 — pair test unavailable", "err")
            return
        try:
            dts = [float(a) for a in args] if args else list(PAIR_DT_MS)
        except ValueError:
            self._driver_print("  usage: pp [dt_ms ...]   e.g. pp 1 2 5 10 20 50", "err")
            return
        with _lock:
            ot = _state['open_time_us']
        # The ZC1's CYCLE time is at least 300 µs and at least OPEN + 100 µs.
        dt_min = max(0.3, (ot + 100) / 1000.0)
        bad = [d for d in dts if not (dt_min <= d <= PAIR_DT_MAX_MS)]
        dts = sorted(set(d for d in dts if dt_min <= d <= PAIR_DT_MAX_MS))
        if bad:
            self._driver_print(
                f"  dropped {', '.join(f'{d:g}' for d in bad)} ms — dt must be "
                f"{dt_min:g} to {PAIR_DT_MAX_MS:g} ms at open {ot} µs", "err")
        if not dts:
            self._driver_print("  no valid dt left — nothing to run", "err")
            return
        pt = _pairtest_new(dts, ot)
        _pairtest = pt
        n = pt['n_total']
        mins = n * (CAPTURE_PRE_S + CAPTURE_POST_S + CAPTURE_SETTLE_S) / 60.0
        self._driver_print(
            f"  pair test armed — dt {', '.join(f'{d:g}' for d in dts)} ms, "
            f"{PAIR_REPEATS} pairs each, every pair with its own single "
            f"reference: {n} captures, ~{mins:.0f} min", "ok")
        self._driver_print("  'pp stop' ends it early; the pairs so far are summarised", "dim")
        log_event(f"PAIR TEST: started — dt {', '.join(f'{d:g}' for d in dts)} ms, "
                  f"{PAIR_REPEATS} pairs each, open {ot} µs ({n} captures)")
        self._update_hint()

    def _set_rate(self, s):
        try:
            val = float(s)
        except ValueError:
            self._driver_print("  invalid — must be a number", "err")
            return
        val = max(DRIVE_HZ_MIN, min(DRIVE_HZ_MAX, val))
        with _lock:
            _state['drive_rate_hz'] = val
        note = "  (live)" if _drive_on.is_set() else ""
        self._driver_print(
            f"  drive rate → {val:g} Hz  ({1000.0/val:.1f} ms){note}", "ok")
        log_event(f"drive rate  →  {val:g} Hz  ({1000.0/val:.1f} ms){note}")
        self._update_hint()

    def _set_open(self, s):
        try:
            val = int(float(s))
        except ValueError:
            self._driver_print("  invalid — must be an integer", "err")
            return
        if _zc1_ok:
            new_val, _ = zc1_set_open_time(_zc1_serial, val)
        else:
            new_val = max(50, min(5_000_000, val))
        with _lock:
            _state['open_time_us'] = new_val
        suffix = "" if _zc1_ok else "  (ZC1 offline)"
        self._driver_print(f"  open time → {new_val} µs{suffix}", "ok")
        log_event(f"open time  →  {new_val} µs{suffix}")
        self._update_hint()

    def _set_capillary(self, side, id_um, len_mm):
        """Record a capillary now installed on *side* ('up' or 'down'). Stamped
        into every subsequent capture and session row. Both values must be
        positive; to record that no upstream capillary is fitted use
        'cap up none' instead. The downstream capillary cannot be cleared — the
        Gyger is never run without one."""
        try:
            idd = float(id_um)
            ln  = float(len_mm)
        except (ValueError, TypeError):
            self._driver_print("  invalid — id and length must be numbers", "err")
            return
        if idd <= 0 or ln <= 0:
            self._driver_print(
                "  invalid — id and length must be > 0 "
                "(use 'cap up none' to clear the upstream capillary)", "err")
            return
        id_key = 'upstream_cap_id_um'  if side == "up" else 'downstream_cap_id_um'
        ln_key = 'upstream_cap_len_mm' if side == "up" else 'downstream_cap_len_mm'
        label  = "upstream" if side == "up" else "downstream"
        with _lock:
            _state[id_key] = idd
            _state[ln_key] = ln
        desc = _cap_fmt(idd, ln)
        self._driver_print(f"  {label} capillary → {desc}", "ok")
        log_event(f"{label} capillary  →  {desc}")
        self._update_hint()

    def _clear_upstream_cap(self):
        """Record that no upstream capillary is fitted (both values → 0). Only
        the upstream side can be cleared; the downstream capillary is always
        present on this rig."""
        with _lock:
            _state['upstream_cap_id_um']  = 0.0
            _state['upstream_cap_len_mm'] = 0.0
        self._driver_print("  upstream capillary → none", "ok")
        log_event("upstream capillary  →  none")
        self._update_hint()

    def _set_gas(self, species):
        """Record the gas at the valve. RECORDED ONLY — the gauge conversion
        still assumes N2/air; apply the species factor downstream."""
        species = species.strip()
        if not species:
            self._driver_print("  invalid — give a species, e.g. gas N2", "err")
            return
        with _lock:
            _state['gas_species'] = species
        self._driver_print(f"  gas → {species}   (gauge still reads N2-equivalent)", "ok")
        log_event(f"gas  →  {species}")
        self._update_hint()

    # ── chart drawing ───────────────────────────────────────────────────────
    def _draw_chart(self, canvas, data, fmt="{:.3f}", log=False, color=BRIGHT):
        """Draw a strip chart on *canvas*.

        data : sequence of values (oldest → newest)
        fmt  : format string for the y-axis tick labels
        log  : if True, plot log10 of the data (for vacuum pressure, which
               spans orders of magnitude). Non-positive values are skipped.
        color: colour of the trace
        """
        c = canvas
        c.delete("all")
        w = c.winfo_width() or 580
        h = c.winfo_height() or 100
        pad_l, pad_r, pad_y = 80, 8, 6

        for i in range(1, 4):
            y = pad_y + (h - 2 * pad_y) * i / 4
            c.create_line(pad_l, y, w - pad_r, y, fill=GRID)

        if len(data) < 2:
            c.create_text(w / 2, h / 2, text="waiting for data",
                          fill=DIM, font=self.f)
            return

        if log:
            plot_vals = [math.log10(v) for v in data if v is not None and v > 0]
        else:
            plot_vals = [v for v in data if v is not None]
        if len(plot_vals) < 2:
            c.create_text(w / 2, h / 2, text="waiting for data",
                          fill=DIM, font=self.f)
            return

        lo, hi = min(plot_vals), max(plot_vals)
        if hi - lo < 1e-9:
            lo -= 0.5
            hi += 0.5
        span = hi - lo
        n = len(plot_vals)

        for i in range(5):
            frac = i / 4
            y = (h - pad_y) - (h - 2 * pad_y) * frac
            plot_val = lo + span * frac
            real_val = (10 ** plot_val) if log else plot_val
            c.create_text(pad_l - 4, y, text=fmt.format(real_val),
                          fill=DIM, font=self.f, anchor="e")

        pts = []
        for i, v in enumerate(plot_vals):
            x = pad_l + (w - pad_l - pad_r) * i / (n - 1)
            y = (h - pad_y) - (h - 2 * pad_y) * (v - lo) / span
            pts.extend((x, y))
        c.create_line(*pts, fill=color, width=1)

    # ── polling loop ────────────────────────────────────────────────────────
    def _poll(self):
        """GUI refresh tick. Reschedules itself unconditionally.

        _poll_once does the drawing. It is wrapped because this callback is the
        only thing that keeps the GUI alive: if it raises, Tk drops the chain
        and the window stops updating for good, which looks exactly like a
        freeze even though the acquisition threads are still running. A drawing
        fault should cost one frame, not the whole session.
        """
        try:
            self._poll_once()
        except Exception as e:
            log_event(f"GUI refresh error (continuing): {e}")
        if not _stop.is_set():
            self.root.after(150, self._poll)

    def _poll_once(self):
        with _lock:
            p_samp = _state['keller_pressure_samples']
            t_samp = _state['keller_temperature_samples']
            p      = (sum(p_samp) / len(p_samp)) if p_samp else None
            t      = (sum(t_samp) / len(t_samp)) if t_samp else None
            vac    = _state['vacuum_chamber_mbar']
            vac_st = _state['vacuum_status']
            ptot   = _state['pulses_total']
            pint   = _state['pulse_interval']
            rate   = _state['drive_rate_hz']
            ot     = _state['open_time_us']
            chart      = list(_chart)
            temp_chart = list(_temp_chart)
            vac_chart  = list(_vac_chart)
            events = list(_events)
        driving = _drive_on.is_set()

        # Display smoothing over the same real time on every chart (their sample
        # rates differ), plus a matching smoothed value for the vacuum readout so
        # the number stops chasing gauge noise. Display only — the log is raw.
        n_k = max(1, round(DISPLAY_SMOOTH_S * KELLER_POLL_HZ))
        n_v = max(1, round(DISPLAY_SMOOTH_S * LABJACK_SAMPLE_HZ))
        vac_disp = (_smooth_display(vac_chart[-n_v:], n_v)[-1]
                    if vac_chart else vac)

        p_s  = f"{p:.4f} bar"    if p   is not None else "---"
        t_s  = f"{t:.1f} °C"     if t   is not None else "---"
        if vac_disp is not None:
            v_s = f"{vac_disp:.2e} mbar"
        else:
            v_s = f"--- ({vac_st})" if vac_st else "---"
        d_s  = f"ON  {rate:g} Hz" if driving else "off"

        st = self.status_text
        st.configure(state="normal")
        st.delete("1.0", "end")
        st.insert("end", "PULSE-VALVE-DRIVER\n", "bright")
        for lbl, ok, avail in (
            (f"[KELLER:{'OK' if _keller_ok else '--'}]",  _keller_ok,  True),
            (f"[VACUUM:{'OK' if _labjack_ok else '--'}]", _labjack_ok, LABJACK_AVAILABLE),
            (f"[ZC1:{'OK' if _zc1_ok else '--'}]",        _zc1_ok,     True),
        ):
            tag = "ok" if ok else ("dim" if not avail else "err")
            st.insert("end", lbl + "  ", tag)
        st.insert("end", "\n\n")
        st.insert("end", "PRESSURE     ", "dim") ; st.insert("end", p_s + "\n",
                  "bright" if p   is not None else "dim")
        st.insert("end", "TEMPERATURE  ", "dim") ; st.insert("end", t_s + "\n",
                  "bright" if t   is not None else "dim")
        st.insert("end", "VACUUM       ", "dim") ; st.insert("end", v_s + "\n",
                  "vac" if vac_disp is not None
                  else ("err" if vac_st else "dim"))
        st.insert("end", "\n")
        st.insert("end", "PULSES       ", "dim")
        st.insert("end", f"total={ptot}  interval={pint}\n", "bright")
        st.insert("end", "OPEN TIME    ", "dim")
        st.insert("end", f"{ot} µs\n", "bright")
        st.insert("end", "CAPTURE      ", "dim")
        if _capture_busy:
            st.insert("end", "running…\n", "bright")
        elif _capture_runs:
            last = _capture_runs[-1]
            pk = last['peak']
            # A shot with no detectable pulse records peak = None. Formatting it
            # as a number raises, and because _poll reschedules itself at the
            # end, the exception stops the GUI updating for good.
            if pk is None:
                st.insert("end", "last shot — no pulse detected\n", "err")
            else:
                pct = 100.0 * pk / CHAMBER_LIMIT_MBAR
                st.insert("end",
                          f"last peak {pk:.3e} mbar  ({pct:.0f}% of limit)\n",
                          "err" if pk >= CHAMBER_LIMIT_MBAR else "bright")
        else:
            st.insert("end", "none yet\n", "dim")
        st.configure(state="disabled")

        self._draw_chart(self.canvas,      _smooth_display(chart, n_k),      fmt="{:.3f}")
        self._draw_chart(self.temp_canvas, _smooth_display(temp_chart, n_k), fmt="{:.1f}")
        self._draw_chart(self.vac_canvas,  _smooth_display(vac_chart, n_v),  fmt="{:.3e}", log=True,
                         color=VAC_LINE)

        text = "\n".join(f"> {s}  {m}" for s, m in events[-200:])
        if getattr(self, "_last_log_text", None) != text:
            self._last_log_text = text
            self.logtext.configure(state="normal")
            self.logtext.delete("1.0", "end")
            self.logtext.insert("1.0", text)
            self.logtext.see("end")
            self.logtext.configure(state="disabled")

        self._update_hint()

    # ── lifecycle ───────────────────────────────────────────────────────────
    def shutdown(self):
        _drive_on.clear()
        _stop.set()
        try:
            if _zc1_serial and _zc1_serial.is_open:
                _zc1_serial.close()
        except Exception:
            pass
        print(f"Log saved: {LOG_FILE}")
        try:
            self.root.destroy()
        except Exception:
            pass

    def run(self):
        self.root.mainloop()


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    global _zc1_serial, _zc1_ok, _keller_ok

    print("\nPULSE-VALVE-DRIVER — startup")
    print("─" * 50)

    print("Scanning for Keller sensor...")
    keller_port, keller_bus = _detect_keller_bus()
    _keller_ok = keller_bus is not None
    if not _keller_ok:
        print("  [Keller] NOT FOUND — pressure/temperature will be blank.")

    print("Scanning for ZC1 controller...")
    zc1_port, _zc1_serial = _detect_zc1()
    _zc1_ok = _zc1_serial is not None
    if _zc1_ok:
        zc1_set_open_time(_zc1_serial, DEFAULT_OPEN_US)
        print(f"  [ZC1]    connected on {zc1_port}")
    else:
        print("  [ZC1]    NOT FOUND — supervisor will keep scanning.")

    print(f"\nLogging to: {LOG_FILE}")
    print(f"Capture manifest: {MANIFEST_FILE}")
    print(f"Log cadence: every {LOG_INTERVAL_S:g} s (drift-free)")
    print("─" * 50)

    # ── Start worker threads ───────────────────────────────────────────────
    if keller_bus is not None:
        threading.Thread(target=keller_thread, args=(keller_port, keller_bus),
                         daemon=True).start()
    if LABJACK_AVAILABLE:
        threading.Thread(target=labjack_thread, daemon=True).start()
    threading.Thread(target=valve_drive_thread,   daemon=True).start()
    threading.Thread(target=logger_thread,         daemon=True).start()
    threading.Thread(target=zc1_supervisor_thread, daemon=True).start()

    log_event("System started")
    if _keller_ok:
        log_event(f"Keller online · {keller_port}")
    if _zc1_ok:
        log_event(f"ZC1 online · {zc1_port}")

    # ── GUI runs on the main thread ────────────────────────────────────────
    PVDGui().run()


if __name__ == '__main__':
    main()
