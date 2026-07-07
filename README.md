# PULSE-VALVE-DRIVER

Python driver for serial pulse valve systems with real-time pressure and temperature logging.

Designed for gas handling characterisation work: fire single calibrated pulses through a pulse valve, log upstream pressure, upstream temperature, and vacuum chamber pressure, and export timestamped CSV data for post-processing.

---

## Experiment setup

![Experiment setup](docs/experiment-setup.png)

N₂ supply → isolation valve → **pressurised volume** (upstream pressure sensor) → 
isolation valve → **pulse valve** → vacuum chamber (downstream pressure sensor + turbopump).

---

## Features

- Single-shot pulse valve control via ZC1 controller over RS-232/USB
- Real-time upstream pressure and temperature logging (Keller PAA series)
- Vacuum chamber pressure logging via LabJack U3 (log-linear gauge)
- Single timestamped CSV with per-pulse events embedded inline (microsecond resolution)
- Auto-detection of serial devices on any COM port, with automatic reconnection if a device disconnects mid-session
- Tkinter GUI: live readouts, three synchronised strip charts (upstream pressure, upstream temperature, vacuum chamber), and a valve driver terminal
- Companion tools for plotting logs and diagnosing the vacuum gauge

---

## Hardware

| Device | Interface | Role |
|---|---|---|
| Keller PAA-23SX-H2 | RS-485 / USB (K-114 adapter) | Upstream pressure + temperature |
| ZC1 controller | RS-232 / USB | Pulse valve driver |
| LabJack U3 (optional) | USB | Vacuum chamber gauge on FIO2 |

---

## Requirements

```
pip install pyserial keller-protocol LabJackPython matplotlib
```

LabJackPython is optional — the driver runs without it if no vacuum gauge is connected. `matplotlib` is only needed for the log plotter, not the driver itself.

---

## Usage

```bash
python PULSE-VALVE-DRIVER.py
```

### Valve driver commands

| Command | Action |
|---|---|
| `v` | Fire one pulse |
| `t <µs>` | Set open time (50 – 5 000 000 µs) |
| `q` | Quit |

The open time in force is shown in the driver window and recorded in every CSV row, so pulses fired at different open times are distinguishable in the log.

---

## Output

One CSV file is written to a `logs/` folder on each run:

**`pvd-sensor_<timestamp>.csv`** — sensor readings at the configured cadence, with pulse events embedded in the row covering the interval they occurred in.

| Column | Description |
|---|---|
| `timestamp` | ISO 8601 |
| `keller_pressure_bar` | Mean upstream pressure over interval (blank if no samples) |
| `keller_temperature_degC` | Mean upstream temperature over interval (blank if no samples) |
| `n_keller_samples` | Number of Keller samples averaged for the row |
| `vacuum_chamber_mbar` | Vacuum gauge reading |
| `pulses_interval` | Pulses fired since last row |
| `pulses_total` | Cumulative pulse count |
| `open_time_us` | Valve open-time setting |
| `pulse_timestamps_us` | Semicolon-separated microsecond timestamps of any pulses in this interval (blank if none) |
| `pulse_open_times_us` | Semicolon-separated open times for those pulses |
| `pulse_acks` | Semicolon-separated ACK flags (1 = ZC1 acknowledged, 0 = no response) |

Rows with no pulse in their interval leave the three `pulse_*` columns blank, so filtering to pulse rows is simply a matter of selecting rows where `pulse_timestamps_us` is non-empty.

---

## Companion tools

**`PULSE-VALVE-LOG-PLOTTER.py`** — overlay plot of a session. Run with no arguments to open a file picker, or pass a CSV directly. Plots upstream pressure and temperature (top panel, twin axes), vacuum chamber pressure (bottom panel, log scale), and pulse events as vertical markers across both.

```bash
python PULSE-VALVE-LOG-PLOTTER.py                 # pick a file, plot whole session
python PULSE-VALVE-LOG-PLOTTER.py log.csv --zoom  # crop to around the pulses
python PULSE-VALVE-LOG-PLOTTER.py log.csv --open-time 90   # optional: one open-time only
```

**`VACUUM-DIAG.py`** — standalone diagnostic for the vacuum gauge on FIO2. Prints raw voltage and converted pressure, and flags ADC saturation or a floating input.

---

## Configuration

Edit the `CONFIGURATION` block at the top of `PULSE-VALVE-DRIVER.py`:

```python
KELLER_PORT     = None      # None = auto-detect, or e.g. "COM15"
ZC1_PORT        = None      # None = auto-detect, or e.g. "COM18"
DEFAULT_OPEN_US = 1600      # default valve open time (µs)
LOG_INTERVAL_S  = 0.5       # sensor log cadence (s)
KELLER_POLL_HZ  = 4         # Keller sampling rate (Hz)
```

---

## Vacuum gauge calibration

The LabJack FIO2 input uses a log-linear conversion:

```
log10(P / mbar) = 5.389 × V − 11.329
```

The LabJack U3 single-ended ADC saturates at ~2.44 V, corresponding to ~1 mbar. Readings above this pressure will be pegged. See `VACUUM-DIAG.py` for a standalone diagnostic.

---

## License

MIT License — see [LICENSE](LICENSE)
