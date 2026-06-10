#!/usr/bin/env python3
"""
GYGER-DRIVER.py
===============
Logs upstream pressure and temperature from a Keller PAA-23SX-H2 sensor,
and allows manual *and* automatic (independent) control of a Gyger ZC1
valve controller (SMLD 300GR).

This version presents a TWO-WINDOW GUI (Tkinter, standard library):
  • "Live Log"      — device status, live readouts, pressure strip chart,
                      pulse counters, and a scrolling event log.
  • "Valve Driver"  — fire single shots, set open time, and start/stop an
                      automatic valve drive at a configurable rate. The
                      valve runs on its own thread, independent of logging.

Hardware
--------
  Keller PAA-23SX-H2   RS485/USB (K-114 adapter) — upstream P + T
  Gyger ZC1            USB-RS232 — pulse valve control
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

Usage
-----
  python GYGER-DRIVER.py
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
ZC1_BAUD           = 38400
ZC1_PORT           = None       # None = auto-detect
LOG_INTERVAL_S     = 0.5        # seconds between logged rows (drift-free)
DEFAULT_OPEN_US    = 1600       # µs — matches current ZC1 setting

DEFAULT_DRIVE_HZ   = 1.0        # automatic pulse repetition rate (Hz)
DRIVE_HZ_MIN       = 0.01       # clamp: slowest auto-drive rate
DRIVE_HZ_MAX       = 200.0      # clamp: fastest auto-drive rate

CHART_SECONDS      = 120        # pressure strip-chart window (s)
LOG_DIR            = str(Path(__file__).parent / "logs")
# ═══════════════════════════════════════════════════════════════════════════════

os.makedirs(LOG_DIR, exist_ok=True)


def _make_log_path():
    """Return a writable CSV path, retrying with a unique suffix if locked."""
    base = datetime.now().strftime('%Y%m%d_%H%M%S')
    for attempt in range(100):
        suffix = "" if attempt == 0 else f"_{attempt}"
        path = os.path.join(LOG_DIR, f"gyger_drive_{base}{suffix}.csv")
        try:
            fh = open(path, 'x', newline='')
            fh.close()
            return path
        except (PermissionError, FileExistsError):
            continue
    return f"gyger_drive_{base}.csv"


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
    # Pulse event log: list of (iso_timestamp, open_us, ack_ok) since last row
    pulse_events                = [],
)
_events      = deque(maxlen=200)                          # (timestamp, text)
_chart       = deque(maxlen=CHART_SECONDS * KELLER_POLL_HZ)  # recent pressures
_zc1_serial  = None
_zc1_ok      = False
_keller_ok   = False
_labjack_ok  = False


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
                with _lock:
                    if p1 is not None:
                        _state['keller_pressure_samples'].append(round(p1, 4))
                        _chart.append(p1)
                    if tob1 is not None:
                        _state['keller_temperature_samples'].append(round(tob1, 2))
                _stop.wait(timeout=max(0.0, interval - (time.time() - t0)))

        except Exception as e:
            log_event(f"Keller read error: {e} — reconnecting…")

        # ── null state immediately so GUI shows '---' ──────────────────────
        _keller_ok = False
        with _lock:
            _state['keller_pressure_samples']    = []
            _state['keller_temperature_samples'] = []

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
                try:
                    raw  = lj.getAIN(LABJACK_FIO2_CHANNEL)
                    mbar = _voltage_to_vacuum_mbar(raw)
                    with _lock:
                        _state['vacuum_chamber_mbar'] = round(mbar, 6)
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
# Two output files per session:
#   gyger_drive_<ts>.csv       — periodic sensor rows (0.5 s cadence)
#   gyger_pulses_<ts>.csv      — one row per pulse, microsecond timestamp
#
# Sensor row columns:
#   timestamp, keller_pressure_bar (mean), keller_temperature_degC (mean),
#   vacuum_chamber_mbar, pulses_interval, pulses_total, open_time_us,
#   n_keller_samples
#
# Pulse log columns:
#   timestamp_us, open_time_us, ack_ok
# ═══════════════════════════════════════════════════════════════════════════════

PULSE_LOG_FILE = LOG_FILE.replace('gyger_drive_', 'gyger_pulses_')


def logger_thread():
    with open(LOG_FILE, 'a', newline='') as sf,          open(PULSE_LOG_FILE, 'a', newline='') as pf:

        sensor_writer = csv.writer(sf)
        pulse_writer  = csv.writer(pf)

        sensor_writer.writerow([
            'timestamp',
            'keller_pressure_bar',      # mean over interval (None if no samples)
            'keller_temperature_degC',  # mean over interval (None if no samples)
            'n_keller_samples',         # how many Keller samples were averaged
            'vacuum_chamber_mbar',
            'pulses_interval',
            'pulses_total',
            'open_time_us',
        ])
        pulse_writer.writerow(['timestamp_us', 'open_time_us', 'ack_ok'])
        sf.flush()
        pf.flush()

        start = time.time()
        n = 0
        while not _stop.is_set():
            with _lock:
                p_samp = _state['keller_pressure_samples']
                t_samp = _state['keller_temperature_samples']
                p_mean = round(sum(p_samp) / len(p_samp), 4) if p_samp else None
                t_mean = round(sum(t_samp) / len(t_samp), 2) if t_samp else None
                n_k    = len(p_samp)
                _state['keller_pressure_samples']    = []
                _state['keller_temperature_samples'] = []

                vac    = _state['vacuum_chamber_mbar']
                p_int  = _state['pulse_interval']
                p_tot  = _state['pulses_total']
                ot     = _state['open_time_us']
                _state['pulse_interval'] = 0

                events = _state['pulse_events']
                _state['pulse_events'] = []

            ts = datetime.now().isoformat(timespec='milliseconds')
            sensor_writer.writerow([ts, p_mean, t_mean, n_k, vac, p_int, p_tot, ot])
            sf.flush()

            for evt_ts, evt_ot, evt_ok in events:
                pulse_writer.writerow([evt_ts, evt_ot, 1 if evt_ok else 0])
            if events:
                pf.flush()

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


class GygerGUI:
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
        self.root.title("GYGER-DRIVER  —  Live Log")
        self.root.configure(bg=BG)
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

        # chart
        tk.Label(outer, text=f"─── pressure  last {CHART_SECONDS}s",
                 font=self.f, fg=DIM, bg=BG, anchor="w").pack(fill="x")
        self.canvas = tk.Canvas(outer, bg=BG, height=120,
                                highlightbackground=BORDER, highlightthickness=1)
        self.canvas.pack(fill="both", expand=True)

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
        win.title("GYGER-DRIVER  —  Valve Driver")
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

        self._driver_print("GYGER-DRIVER  —  Valve Driver")
        self._driver_print("SMLD 300GR · Gyger ZC1")
        self._driver_print("")
        self._driver_print("  v          fire one pulse")
        self._driver_print("  t <µs>     set open time")
        self._driver_print("  q          quit")
        self._driver_print("")
        self._update_hint()

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
            ot = _state['open_time_us']
        self.hint_var.set(f"open={ot} µs  | v  t <µs>  q")

    def _on_cmd(self):
        raw = self.cmd_entry.get().strip()
        self.cmd_entry.delete(0, "end")
        if not raw:
            return
        self._driver_print(f"> {raw}", "dim")
        parts = raw.lower().split()
        cmd   = parts[0]

        if cmd == "q":
            self.shutdown()

        elif cmd == "v":
            ok, ot = fire_one()
            status = "OK" if ok else ("no ACK" if _zc1_ok else "no device")
            self._driver_print(f"  pulse fired  {ot} µs  {status}",
                               "ok" if ok else "err")
            log_event(f"pulse fired  {ot} µs  {status}")

        elif cmd in ("d", "r"):
            self._driver_print(
                "  auto-drive is disabled — use 'v' for single pulses", "dim")

        elif cmd == "t":
            if len(parts) > 1:
                self._set_open(parts[1])
            else:
                self._driver_print("  usage: t <µs>  (50 – 5 000 000)")

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

    # ── chart drawing ───────────────────────────────────────────────────────
    def _draw_chart(self, data):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width() or 580
        h = c.winfo_height() or 120
        pad_l, pad_r, pad_y = 52, 8, 6
        for i in range(1, 4):
            y = pad_y + (h - 2 * pad_y) * i / 4
            c.create_line(pad_l, y, w - pad_r, y, fill=GRID)
        if len(data) < 2:
            c.create_text(w / 2, h / 2, text="waiting for data",
                          fill=DIM, font=self.f)
            return
        lo, hi = min(data), max(data)
        if hi - lo < 1e-9:
            lo -= 0.5
            hi += 0.5
        span = hi - lo
        n = len(data)
        for i in range(5):
            frac = i / 4
            y = (h - pad_y) - (h - 2 * pad_y) * frac
            c.create_text(pad_l - 4, y, text=f"{lo + span * frac:.3f}",
                          fill=DIM, font=self.f, anchor="e")
        pts = []
        for i, v in enumerate(data):
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
            chart  = list(_chart)
            events = list(_events)
        driving = _drive_on.is_set()

        p_s  = f"{p:.4f} bar"    if p   is not None else "---"
        t_s  = f"{t:.1f} °C"     if t   is not None else "---"
        v_s  = f"{vac:.2e} mbar" if vac is not None else "---"
        d_s  = f"ON  {rate:g} Hz" if driving else "off"

        st = self.status_text
        st.configure(state="normal")
        st.delete("1.0", "end")
        st.insert("end", "GYGER-DRIVER\n", "bright")
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
                  "bright" if vac is not None else "dim")
        st.insert("end", "\n")
        st.insert("end", "PULSES       ", "dim")
        st.insert("end", f"total={ptot}  interval={pint}\n", "bright")
        st.insert("end", "OPEN TIME    ", "dim")
        st.insert("end", f"{ot} µs\n", "bright")
        st.configure(state="disabled")

        self._draw_chart(chart)

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
        print(f"Sensor log: {LOG_FILE}")
        print(f"Pulse log:  {PULSE_LOG_FILE}")
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

    print("\nGYGER-DRIVER — startup")
    print("─" * 50)

    print("Scanning for Keller sensor...")
    keller_port, keller_bus = _detect_keller_bus()
    _keller_ok = keller_bus is not None
    if not _keller_ok:
        print("  [Keller] NOT FOUND — pressure/temperature will be blank.")

    print("Scanning for Gyger ZC1...")
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
    GygerGUI().run()


if __name__ == '__main__':
    main()
