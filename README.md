# PULSE-VALVE-DRIVER

Python driver for serial pulse valve systems with real-time pressure and temperature logging.

Designed for gas handling characterisation work: fire single calibrated pulses through a pulse valve, log upstream pressure and vacuum chamber pressure, and export timestamped CSV data for post-processing.

---

## Experiment setup

![Experiment setup](docs/experiment-setup.png)

---

## Features

- Single-shot pulse valve control via ZC1 controller over RS-232/USB
- Real-time upstream pressure and temperature logging (Keller PAA series)
- Vacuum chamber pressure logging via LabJack U3 (log-linear gauge)
- Dual CSV export: periodic sensor rows and per-pulse event log with microsecond timestamps
- Auto-detection of serial devices on any COM port with reconnection on disconnect
- Two-window Tkinter GUI: live readouts, pressure strip chart, and valve driver terminal

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
pip install pyserial keller-protocol LabJackPython
```

LabJackPython is optional — the driver runs without it if no vacuum gauge is connected.

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

---

## Output

Two CSV files are written to a `logs/` folder on each run:

**`pvd-sensor_<timestamp>.csv`** — sensor readings at 0.5 s cadence

| Column | Description |
|---|---|
| `timestamp` | ISO 8601 |
| `keller_pressure_bar` | Mean upstream pressure over interval |
| `keller_temperature_degC` | Mean temperature over interval |
| `n_keller_samples` | Number of samples averaged |
| `vacuum_chamber_mbar` | Vacuum gauge reading |
| `pulses_interval` | Pulses fired since last row |
| `pulses_total` | Cumulative pulse count |
| `open_time_us` | Valve open time setting |

**`pvd-pulses_<timestamp>.csv`** — one row per pulse

| Column | Description |
|---|---|
| `timestamp_us` | Microsecond-resolution ISO 8601 |
| `open_time_us` | Open time used for this pulse |
| `ack_ok` | 1 if ZC1 acknowledged, 0 if no response |

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
