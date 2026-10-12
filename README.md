# Toyota Tacoma 2019 · 2TR-FE OBD-II Live Monitor

Real-time engine monitor + anomaly flagger for the **2019 Toyota Tacoma 2.7L 2TR-FE** (and similar 2TR-FE trucks).

Runs on a Linux laptop with a USB OBD adapter (tested with **Vgate vLinker FS USB**). Ships with:

- **Terminal TUI** (Rich) — sparklines, thresholds, baseline deltas, session summary
- **Web dashboard** (`--web`) — 1980s TRD tri-color cluster gauges in the browser
- **Demo mode** (`--demo`) — no adapter required; full simulated drive cycle

US units throughout: **°F**, **mph**, **inHg**, **gal/h**.

---

## ⚠️ Oil pressure is not available over OBD-II

The 2TR-FE does **not** report quantitative oil pressure (PSI).

- Factory FSM (hot): ≥ ~4.3 psi idle · 23–75 psi @ 3,000 RPM
- For real pressure, install a **mechanical gauge**

The app surfaces this limitation on every start and in the web UI.

---

## Features

### Terminal + shared engine
- Live metrics with color thresholds and ASCII sparklines
- Baseline capture (`b`) for delta-from-good tracking
- Smart flags: elevated/critical fuel trims, timing retard, AFR voltage out-of-range, low voltage, hot coolant/cat, low fuel, high load at low RPM
- **Pinned CRIT alerts** — last CRITs stay visible until you clear flags (won’t scroll off or get pruned)
- Wide-format CSV logging (one full snapshot row per interval)
- Session summary on exit (min/max, flag counts, last AFR voltage)
- Stable under live data (RLock + defensive callbacks — no freeze when alerts fire)
- Double Ctrl-C force-exits if the serial link is wedged

### Metrics watched (when the ECU supports them)

**Default = FULL set.** Use `--lite` if the adapter stalls under load.

| Group | PIDs |
|-------|------|
| Core | RPM, speed, coolant, intake, timing, load, throttle, MAF, STFT, LTFT, voltage, fuel status, **O2_S1_WR_VOLTAGE** (upstream AFR), O2_B1S2 (downstream) |
| Extra (full) | Ambient, absolute load, relative throttle, accelerator, MAP, baro, fuel level, fuel rate, run time, commanded equiv ratio, cat temp B1S1 |

Unsupported PIDs are skipped — no crashes.

### Upstream AFR (wideband) note
The 2TR-FE uses a **wideband Air-Fuel Ratio sensor** upstream, not a classic narrowband O2.  
We therefore query **PID 0x24 (`O2_S1_WR_VOLTAGE`)** instead of `O2_B1S1`.  
Downstream remains the narrowband `O2_B1S2`.  
There is no “switch count” logic for the wideband sensor (it reports continuous voltage / lambda). A soft out-of-range check still runs when warm.

### Fuel rate / ambient / fuel level
Standard PIDs for ambient air temp (0x46), fuel level (0x2F), and fuel rate (0x5E) are **usually not published** by the 2TR-FE over generic Mode 01.  
When fuel rate is missing we **estimate** it from MAF ÷ AFR (using commanded lambda when available) so instant/avg MPG still works. The dashboard labels estimated values as `gph (est)`.

### Transmission fluid temp (Mode 22)
Custom Toyota Mode 22 PIDs (header `701`):
- **TFT_PAN** — `22 1627` — transmission pan temperature (°F)
- **TFT_CONV** — `22 1628` — post-converter temperature (°F)

Polled every ~3 s (header switch is heavier than Mode 01). Same formulas the Torque/Fusion community uses on 2016–2021 Tacomas. If the adapter or TCM doesn’t answer, those tiles stay blank — no crash.

### Web dashboard (`tacoma_dashboard.html`)
- **Cluster view** — circular RPM/speed gauges, side tiles, mini strip, system strip
- **Metrics view** — full table with min/max/sparklines
- **80s TRD tri-color** — red · orange · yellow on black, Toyota mark in header
- **Peaks** — max RPM/speed/coolant/cat, lowest voltage, instant MPG
- **Session summary** — samples, flags, last AFR voltage, avg MPG, log rows
- **Pinned CRIT** — stays at top of Alerts until Clear Flags
- **Save Log (CSV)** — download in-browser session log (`S` key); includes `active_flags` + `pinned_crit`
- **Autosave every 5 min** — toggle in footer
- **Sound alerts** — beep on new warn/crit (toggleable)
- **Gauge glow** — pulse on hot coolant, low voltage, timing retard, high RPM
- **Keyboard** — `S` save · `P` pause · `B` baseline
- Works standalone (client demo) or fed live from `python tacoma_monitor.py --web`

---

## Hardware

1. **USB OBD-II adapter** (recommended):
   - **Vgate vLinker FS USB** — solid on Linux; shows up as FTDI (`0403:6015`) → often `/dev/ttyUSB4` or similar
   - Also fine: OBDLink SX/EX, quality ELM327 (real chip)
   - Avoid the cheapest fake “v2.1” clones
2. **Cable** — 2019 Tacoma OBD port is under the dash, left of the steering column
3. **Key ON or engine running** — the adapter needs 12V from the truck; laptop-only = “no connection to car”
4. **Linux permissions**
   ```bash
   sudo usermod -a -G dialout $USER
   # log out and back in (or reboot)
   ```

### Finding the port (vLinker)
```bash
lsusb | grep -i '0403:6015\|10c4:ea60'   # FTDI or CP210x
dmesg | tail -20                          # look for ttyUSBx attach
ls -l /dev/ttyUSB* /dev/serial/by-id/
python3 -m serial.tools.list_ports -v
```

If the process hangs and won’t die:
```bash
pkill -9 -f tacoma_monitor.py
fuser -k /dev/ttyUSB4          # use your actual port
```

---

## Install

```bash
cd tacoma-obd-monitor   # or wherever you keep the files
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Keep these files together:

```
tacoma_monitor.py
tacoma_dashboard.html
README.md
requirements.txt
```

---

## Run

```bash
# Demo only (terminal)
python tacoma_monitor.py --demo

# Demo + fancy web gauges
python tacoma_monitor.py --demo --web
# → open http://127.0.0.1:8765/

# Live truck (FULL PID set)
python tacoma_monitor.py --port /dev/ttyUSB4 --web --log

# Lighter query load if the adapter ever stalls
python tacoma_monitor.py --port /dev/ttyUSB4 --web --lite

# Explicit log path
python tacoma_monitor.py --port /dev/ttyUSB4 --web --log-file drive_test.csv
```

| Flag | Meaning |
|------|---------|
| `--demo` | Simulated 2TR-FE data (cold → warm-up → looping drive cycle) |
| `--web` | Start browser dashboard on port 8765 |
| `--web-port N` | Custom web port |
| `--log` / `--log-file` | Terminal-side CSV logging |
| `--lite` | CORE PIDs only (fewer queries) |
| `-p /dev/ttyUSB4` | Force serial port |

### Terminal keys (type + Enter)
| Key | Action |
|-----|--------|
| `b` | Capture baseline |
| `l` | Toggle CSV logging |
| `c` | Clear flags **and pinned CRITs** |
| `h` | Help |
| `q` | Quit (session summary) |

Ctrl-C once = clean stop · twice = force exit.

### Browser keys
| Key | Action |
|-----|--------|
| `S` | Save log CSV |
| `P` | Pause / resume |
| `B` | Capture baseline |

---

## Demo timeline

| Time | Phase |
|------|--------|
| 0:00–0:30 | Cold idle |
| 0:30–1:30 | Warm-up → closed loop |
| 1:30+ | Drive cycle (cruise / accel / pull / coast) — **loops forever** |

Demo does **not** exit on its own. Quit with `q` or Ctrl-C.

---

## Logging

**Terminal CSV** (`--log`): wide rows from Python (watched PIDs + active flags).

**Browser CSV** (Save Log / Autosave): wide rows from the page, including:
- `inst_mpg`
- `active_flags` — recent alerts at that sample
- `pinned_crit` — any CRIT still pinned

---

## What the flags mean (high level)

| Flag | Meaning |
|------|---------|
| **Fuel trims** | Sustained high +/− often = vacuum leak, MAF, exhaust leak, fuel delivery |
| **Timing retard** | Big negative under load = knock activity, fuel, carbon, sensor |
| **AFR voltage out of range** | Wideband upstream voltage far outside normal ~1.5–4.5 V window |
| **Voltage** | Charging system / battery |
| **Coolant / cat temp** | Thermal issues |
| **High load @ low RPM** | Possible lugging (INFO) |

CRIT alerts stay **pinned** until you clear them so a brief spike isn’t lost mid-drive.

---

## Notes for 2TR-FE owners

- Warm idle timing is typically **5–15° BTDC**
- LTFT near **±10%** is usually fine; watch for drift over multiple cycles
- Upstream is a wideband AFR sensor (PID 0x24) — expect continuous voltage near ~3 V at stoich, not 0.1–0.9 V switching
- Absolute load + timing together give a better picture under real driving
- This adapter is for **monitoring / diagnosis**, not flashing performance maps on Toyota ECUs

---

## Safety

- Diagnostic aid only — not a substitute for a mechanical oil gauge or a proper scan tool
- Eyes on the road; mount any display safely
- Some jurisdictions regulate devices while driving — use responsibly

---

## Credits

- [python-obd](https://github.com/brendan-w/python-OBD)
- [Rich](https://github.com/Textualize/rich)
- TacomaWorld / Toyota FSM notes on 2TR-FE oil pressure and PID coverage
- SAE J1979 standard PIDs

---

Drive safe, Bullet.
