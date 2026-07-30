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
  drive_active, drive_rate_hz, open_time_us

Logging cadence
---------------
  One CSV row is written on a drift-free wall-clock cadence of
  LOG_INTERVAL_S seconds (first row written immediately at start).

Driver commands
---------------
  v          fire one pulse (no capture)
  c [n]      capture n pulses at high rate, with a summary after a multi-shot run
  t <µs>     set open time
  q          quit

Capture outputs
---------------
  logs/pulse_<ts>.csv   one file per shot: full decimated trace, t = 0 at fire,
                        with peak / dp / integral / tau / S_eff / Q in the header

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
from collections import deque
from datetime import datetime
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
LABJACK_FIO2_CHANNEL  = 2          # AIN2 = FIO2
VACUUM_SLOPE          = 5.389       # log-linear calibration slope
VACUUM_INTERCEPT      = -11.329     # log-linear calibration intercept
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

CHAMBER_VOLUME_L    = 15.586   # geometric estimate (5 combined cylinders); consistent
                               # with measured tau ~161 ms if pump line is DN63+.
                               # Pending final pump-line-diameter check vs Cornelia's 5 L.
CHAMBER_LIMIT_MBAR  = 1.0e-6   # the in-chamber limit being demonstrated against

# ─── Run identity (what physically produced these captures) ────────────────────
# The conductance limiter and the gas are NOT auto-detectable — they change only
# when hardware is swapped, and nothing in the rig reports them. They are stamped
# into every capture header and every session row so a file is self-describing
# and re-analysable without the lab notebook. The catch: a stale value here is
# WORSE than none, because it mislabels data confidently. Defences: the driver
# echoes the identity in a banner at startup, keeps it in the status hint at all
# times, and lets it be changed at runtime (cap / gas commands) so a mid-session
# swap can be recorded without editing this file and relaunching.
#
# Upstream pressure is deliberately NOT here: it is read live from the Keller and
# stamped at the fire instant, so it can never go stale.
DEFAULT_CAPILLARY_ID_UM  = 50.0    # limiter bore, micrometres
DEFAULT_CAPILLARY_LEN_MM = 100.0   # limiter length, millimetres
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
        path = os.path.join(LOG_DIR, f"pvd-sensor_{base}{suffix}.csv")
        try:
            fh = open(path, 'x', newline='')
            fh.close()
            return path
        except (PermissionError, FileExistsError):
            continue
    return f"pvd-sensor_{base}.csv"


LOG_FILE = _make_log_path()

# ─── Shared state ─────────────────────────────────────────────────────────────
_lock        = threading.Lock()        # protects _state / _events / _chart
_zc1_lock    = threading.Lock()        # serialises ALL access to the ZC1 port
_stop        = threading.Event()
_drive_on    = threading.Event()       # set => automatic valve drive is running
_state       = dict(
    keller_pressure_samples     = [],    # accumulated between log rows, then averaged
    keller_temperature_samples  = [],    # accumulated between log rows, then averaged
    vacuum_chamber_mbar         = None,  # latest single reading
    pulse_interval              = 0,     # pulses fired since last log row (reset each row)
    pulses_total                = 0,
    open_time_us                = DEFAULT_OPEN_US,
    drive_rate_hz               = DEFAULT_DRIVE_HZ,
    # Run identity — what produced the captures. Runtime-settable (cap / gas).
    capillary_id_um             = DEFAULT_CAPILLARY_ID_UM,
    capillary_len_mm            = DEFAULT_CAPILLARY_LEN_MM,
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

def _voltage_to_vacuum_mbar(volts: float) -> float:
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
            lj.getCalibrationData()
            lj.configIO(FIOAnalog=0x04)   # FIO2 as analogue input (vacuum gauge)
            _labjack_ok = True
            log_event("LabJack connected  FIO2 analog")
        except Exception as e:
            _labjack_ok = False
            log_event(f"LabJack connect failed: {e} — retry in 5 s")
            _stop.wait(timeout=5)
            continue   # retry the outer loop

        # ── read loop — breaks on error to trigger reconnect ───────────────
        try:
            while not _stop.is_set():
                t0 = time.time()

                # ── capture ────────────────────────────────────────────────
                # Streaming takes exclusive control of the U3, so it has to run
                # on this thread. Monitoring pauses for the window and resumes
                # automatically. A multi-shot run re-arms itself here.
                global _capture_busy, _capture_pending, _capture_group_index
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
                    raw  = lj.getAIN(LABJACK_FIO2_CHANNEL)
                    mbar = _voltage_to_vacuum_mbar(raw)
                    with _lock:
                        _state['vacuum_chamber_mbar'] = mbar
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
                        NChannels=[31],            # 31 = single-ended vs GND
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
    the input; each point averages up to the last n samples."""
    if n <= 1 or len(vals) < 2:
        return list(vals)
    out, run = [], []
    for v in vals:
        run.append(v)
        if len(run) > n:
            run.pop(0)
        out.append(sum(run) / len(run))
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


def _capture_stream_fire(lj, rate):
    """Stream AIN2 at *rate*, firing one pulse after CAPTURE_PRE_S.

    Returns (volts, fire_index, fire_ok, open_us, missed, up_bar, up_degC).
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
        fired['ok'], fired['ot'] = fire_one()

    lj.streamConfig(NumChannels=1,
                    PChannels=[LABJACK_FIO2_CHANNEL],
                    NChannels=[31],
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

    return (volts[:total_n], fire_idx,
            fired.get('ok', False), fired.get('ot'), missed,
            fired.get('up_bar'), fired.get('up_degC'))


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


def _capillary_str():
    """Human-readable capillary geometry, e.g. '50 µm × 100 mm'."""
    with _lock:
        idd = _state['capillary_id_um']
        ln  = _state['capillary_len_mm']
    return f"{idd:g} µm × {ln:g} mm"


def _run_conditions(up_bar, up_degC):
    """Fields describing what produced a capture: capillary geometry, gas, and
    the upstream state at the fire instant. Merged into every capture header
    (all outcomes) so a file is self-describing and comparable standalone."""
    with _lock:
        idd = _state['capillary_id_um']
        ln  = _state['capillary_len_mm']
        gas = _state['gas_species']
    return {
        'capillary_id_um':       f"{idd:g}",
        'capillary_length_mm':   f"{ln:g}",
        'gas_species':           gas,
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
            w.writerow([f"{t:.5f}", f"{p:.6e}"])
    return path


def _run_capture(lj, shot_index=1, group_id=None, group_size=1):
    """Capture one pulse, analyse it, save it, and report to the event log.

    shot_index / group_id / group_size tie the shots of one `c n` run together;
    they are stamped into the header and shared as a filename prefix."""
    if group_id is None:
        group_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    tag  = "" if group_size <= 1 else f" [{shot_index}/{group_size}]"
    stem = (f"pulse_{group_id}_s{shot_index:02d}"
            if group_size > 1 else f"pulse_{group_id}")
    grp  = _capture_group_meta(group_id, shot_index, group_size)

    rate = _choose_stream_rate(lj)
    if rate is None:
        log_event(f"CAPTURE{tag}: the U3 would not stream at any rate in the "
                  f"ladder — capture unavailable")
        return None

    log_event(f"CAPTURE{tag}: streaming "
              f"{CAPTURE_PRE_S + CAPTURE_POST_S:g} s @ {rate:g} Hz")

    volts, fire_idx, fire_ok, open_us, missed, up_bar, up_degC = \
        _capture_stream_fire(lj, rate)
    if missed:
        log_event(f"CAPTURE{tag}: {missed} samples missed by the stream")
    if fire_idx is None or len(volts) < CAPTURE_DECIMATE * 20:
        log_event(f"CAPTURE{tag}: capture failed - too few samples")
        return None
    if not fire_ok:
        log_event(f"CAPTURE{tag}: WARNING - no ZC1 ack; valve may not have fired")

    dt    = CAPTURE_DECIMATE / rate
    v_dec = _decimate(volts, CAPTURE_DECIMATE)
    mbar  = [_voltage_to_vacuum_mbar(v) for v in v_dec]
    i_fire = fire_idx // CAPTURE_DECIMATE
    times  = [(i - i_fire) * dt for i in range(len(v_dec))]

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
        _write_capture_csv(times, mbar, {'detected': 0, 'flat_trace': 1,
                                         'capture_rate_hz': rate,
                                         'open_time_us': open_us,
                                         'zc1_ack': int(bool(fire_ok)),
                                         **grp,
                                         **_run_conditions(up_bar, up_degC)},
                           stem=stem)
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
            'samples_missed': missed,
        }
        path = _write_capture_csv(times, mbar, meta, stem=stem)
        log_event(f"CAPTURE{tag}: {os.path.basename(path)}")
        _capture_runs.append(dict(meta, path=path, peak=None, tau=None,
                                  detected=False))
        return None

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
    log_event(f"CAPTURE{tag}: {os.path.basename(path)}")

    run = dict(meta)
    run['path'] = path
    run['peak'] = peak
    run['tau'] = tau
    run['detected'] = True
    _capture_runs.append(run)
    return run


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
# One output file per session: pvd-sensor_<ts>.csv
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
#   capillary_id_um, capillary_length_mm, gas_species
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
            'capillary_id_um',         # limiter bore in force for this row
            'capillary_length_mm',     # limiter length in force for this row
            'gas_species',             # gas in force for this row
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
                p_int  = _state['pulse_interval']
                p_tot  = _state['pulses_total']
                ot     = _state['open_time_us']
                cap_id = _state['capillary_id_um']
                cap_ln = _state['capillary_len_mm']
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

            cap_s, ln_s = f"{cap_id:g}", f"{cap_ln:g}"

            def _row(ts, p, t, n_k, with_pulses):
                # Pulses carry their own µs timestamps, so attaching the interval's
                # pulse list to a single row (not every row) keeps them counted once.
                return [
                    ts, p, t, n_k, vac,
                    (p_int if with_pulses else 0), p_tot, ot,
                    (pulse_ts   if with_pulses else ''),
                    (pulse_ots  if with_pulses else ''),
                    (pulse_acks if with_pulses else ''),
                    cap_s, ln_s, gas,
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


class PVDGui:
    """
    Window 1 — Live Log    : text readouts + pressure chart
    Window 2 — Valve Driver: terminal prompt, type commands
                              v        fire one pulse
                              d        toggle auto-drive
                              r <Hz>   set drive rate
                              t <µs>   set open time
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
        self._driver_print("  t <µs>         set open time")
        self._driver_print("  cap <id> <len>  set capillary (bore µm, length mm)")
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
            gas = _state['gas_species']
        self._driver_print("  ┄┄┄ RUN IDENTITY — check before capturing ┄┄┄", "ok")
        self._driver_print(f"    capillary: {_capillary_str()}", "ok")
        self._driver_print(f"    gas:       {gas}", "ok")
        self._driver_print("    upstream:  read live from the Keller, stamped at fire", "dim")
        self._driver_print("    wrong?  cap <id_µm> <len_mm>   /   gas <species>", "dim")
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
            ot  = _state['open_time_us']
            idd = _state['capillary_id_um']
            ln  = _state['capillary_len_mm']
            gas = _state['gas_species']
        cap = f"{idd:g}µm×{ln:g}mm"
        ident = f"{cap} · {gas}"
        if _capture_busy:
            self.hint_var.set(f"{ident}  |  open={ot} µs  |  CAPTURING…")
        elif _capture_pending > 0:
            self.hint_var.set(f"{ident}  |  open={ot} µs  |  {_capture_pending} shot(s) queued")
        else:
            self.hint_var.set(f"{ident}  |  open={ot} µs  | v  c[n]  t  cap  gas  q")

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

        elif cmd == "v":
            ok, ot = fire_one()
            status = "OK" if ok else ("no ACK" if _zc1_ok else "no device")
            self._driver_print(f"  pulse fired  {ot} µs  {status}",
                               "ok" if ok else "err")
            log_event(f"pulse fired  {ot} µs  {status}")

        elif cmd == "c":
            global _capture_pending, _summary_window, _capture_group_id, \
                   _capture_group_size, _capture_group_index
            if _capture_busy or _capture_request.is_set():
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
            # cap <id_µm> <len_mm>
            if len(parts) == 3:
                self._set_capillary(parts[1], parts[2])
            else:
                self._driver_print("  usage: cap <id_µm> <len_mm>", "err")

        elif cmd == "gas":
            if len(raw_parts) == 2:
                self._set_gas(raw_parts[1])
            else:
                self._driver_print("  usage: gas <species>  (e.g. N2, H2, He, Ar)", "err")

        else:
            self._driver_print(f"  unknown: '{cmd}'", "err")

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

    def _set_capillary(self, id_um, len_mm):
        """Record the capillary now installed. Stamped into every subsequent
        capture and session row. The rig always has a capillary after the
        valve, so both values must be positive."""
        try:
            idd = float(id_um)
            ln  = float(len_mm)
        except (ValueError, TypeError):
            self._driver_print("  invalid — id and length must be numbers", "err")
            return
        if idd <= 0 or ln <= 0:
            self._driver_print("  invalid — id and length must be > 0", "err")
            return
        with _lock:
            _state['capillary_id_um']  = idd
            _state['capillary_len_mm'] = ln
        desc = _capillary_str()
        self._driver_print(f"  capillary → {desc}", "ok")
        log_event(f"capillary  →  {desc}")
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
    def _draw_chart(self, canvas, data, fmt="{:.3f}", log=False):
        """Draw a strip chart on *canvas*.

        data : sequence of values (oldest → newest)
        fmt  : format string for the y-axis tick labels
        log  : if True, plot log10 of the data (for vacuum pressure, which
               spans orders of magnitude). Non-positive values are skipped.
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
        c.create_line(*pts, fill=BRIGHT, width=1)

    # ── polling loop ────────────────────────────────────────────────────────
    def _poll(self):
        with _lock:
            p_samp = _state['keller_pressure_samples']
            t_samp = _state['keller_temperature_samples']
            p      = (sum(p_samp) / len(p_samp)) if p_samp else None
            t      = (sum(t_samp) / len(t_samp)) if t_samp else None
            vac    = _state['vacuum_chamber_mbar']
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
        vac_disp = (sum(vac_chart[-n_v:]) / len(vac_chart[-n_v:])) if vac_chart else vac

        p_s  = f"{p:.4f} bar"    if p   is not None else "---"
        t_s  = f"{t:.1f} °C"     if t   is not None else "---"
        v_s  = f"{vac_disp:.2e} mbar" if vac_disp is not None else "---"
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
                  "bright" if vac_disp is not None else "dim")
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
            pct = 100.0 * pk / CHAMBER_LIMIT_MBAR
            st.insert("end",
                      f"last peak {pk:.3e} mbar  ({pct:.0f}% of limit)\n",
                      "err" if pk >= CHAMBER_LIMIT_MBAR else "bright")
        else:
            st.insert("end", "none yet\n", "dim")
        st.configure(state="disabled")

        self._draw_chart(self.canvas,      _smooth_display(chart, n_k),      fmt="{:.3f}")
        self._draw_chart(self.temp_canvas, _smooth_display(temp_chart, n_k), fmt="{:.1f}")
        self._draw_chart(self.vac_canvas,  _smooth_display(vac_chart, n_v),  fmt="{:.3e}", log=True)

        text = "\n".join(f"> {s}  {m}" for s, m in events[-200:])
        if getattr(self, "_last_log_text", None) != text:
            self._last_log_text = text
            self.logtext.configure(state="normal")
            self.logtext.delete("1.0", "end")
            self.logtext.insert("1.0", text)
            self.logtext.see("end")
            self.logtext.configure(state="disabled")

        self._update_hint()

        if not _stop.is_set():
            self.root.after(150, self._poll)

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
