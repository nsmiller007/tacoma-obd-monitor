#!/usr/bin/env python3
"""
Toyota Tacoma 2019 · 2TR-FE (2.7L I4) · OBD-II Live Monitor

Real-time engine monitor + anomaly flagger for the 2019 Tacoma 2TR-FE
(and similar 2TR-FE trucks).

Terminal mode (Rich TUI):
  - Live metrics, sparklines, color thresholds, baseline deltas
  - Smart flags: fuel trims, timing retard, stuck upstream O2, voltage,
    coolant/cat temp, low fuel
  - Wide-format CSV logging + session summary on exit

Web mode (--web):
  - Serves tacoma_dashboard.html with 80s TRD tri-color cluster gauges
  - JSON API at /api/data for live + demo feeds
  - Browser features: MPG estimate, peaks, session summary, CSV save,
    autosave, sound alerts, gauge glow, keyboard shortcuts

US units throughout (°F, mph, inHg, gal/h).
Oil pressure is intentionally called out as unavailable via OBD-II on the 2TR-FE.
See README.md for full usage notes.
"""

import argparse
import csv
import json
import os
import select
import signal
import sys
import threading
import time
from collections import defaultdict, deque
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import obd
from obd import OBDCommand, Unit
from obd.protocols import ECU
from obd.utils import bytes_to_int
from rich import box
from rich.console import Console, Group, RenderableType
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.align import Align

import config as cfgmod
from config import log, make_log_path
from demo import demo_loop
from web_dashboard import start_web_server

# -----------------------------------------------------------------------------
# Configuration / Tuning
# -----------------------------------------------------------------------------

# Core PIDs — always watched (keep this list short; ELM/vLinker stalls if flooded)
# Note: 2TR-FE upstream is a wideband AFR sensor → use O2_S1_WR_VOLTAGE (PID 0x24)
# instead of the classic narrowband O2_B1S1 (PID 0x14). Downstream stays O2_B1S2.
CORE_PIDS: List[str] = [
    "RPM",
    "SPEED",
    "COOLANT_TEMP",
    "INTAKE_TEMP",
    "TIMING_ADVANCE",
    "ENGINE_LOAD",
    "THROTTLE_POS",
    "MAF",
    "SHORT_FUEL_TRIM_1",
    "LONG_FUEL_TRIM_1",
    "CONTROL_MODULE_VOLTAGE",
    "FUEL_STATUS",
    "O2_S1_WR_VOLTAGE",
    "O2_B1S2",
]

# Extra PIDs — watched only if --full is passed (can overwhelm some adapters)
EXTRA_PIDS: List[str] = [
    "AMBIENT_AIR_TEMP",
    "ABSOLUTE_LOAD",
    "RELATIVE_THROTTLE_POS",
    "ACCELERATOR_POS_D",
    "INTAKE_PRESSURE",
    "BAROMETRIC_PRESSURE",
    "FUEL_LEVEL",
    "FUEL_RATE",
    "RUN_TIME",
    "COMMANDED_EQUIV_RATIO",
    "CATALYST_TEMP_B1S1",
]

# Toyota Mode 22 transmission temps (3rd-gen Tacoma / AC60E)
# Header 701, Mode 22, PID 1627 (pan) / 1628 (converter)
# Equation returns °F directly: (A*459/255)+(B*1.6/255)-40
# Polled slowly — header switch is heavier than Mode 01 watches.
TFT_POLL_INTERVAL = 3.0  # seconds between Mode 22 queries

def _decode_toyota_tft(messages):
    """Decode 2-byte Toyota TFT response into °F."""
    try:
        d = messages[0].data
        # Response format typically: [mode_echo, pid_hi, pid_lo, A, B, ...]
        # After python-obd framing, data often starts at mode byte.
        if len(d) >= 5:
            A = d[-2]
            B = d[-1]
        elif len(d) >= 2:
            A = d[0]
            B = d[1]
        else:
            return None
        temp_f = (A * 459.0 / 255.0) + (B * 1.6 / 255.0) - 40.0
        return temp_f * Unit.fahrenheit
    except Exception:
        log.debug("format fallback", exc_info=True)
        return None

CMD_TFT_PAN = OBDCommand(
    "TFT_PAN",
    "Trans Fluid Pan Temp",
    b"221627",
    0,  # unknown length
    _decode_toyota_tft,
    ECU.ALL,
    False,
    header=b"701",
)
CMD_TFT_CONV = OBDCommand(
    "TFT_CONV",
    "Trans Fluid Converter Temp",
    b"221628",
    0,
    _decode_toyota_tft,
    ECU.ALL,
    False,
    header=b"701",
)

# Default set used for display/logging; start_watching uses CORE unless full=True
INTERESTED: List[str] = CORE_PIDS + EXTRA_PIDS

# Display labels and formatting hints
METRIC_META: Dict[str, Dict[str, Any]] = {
    "RPM":               {"label": "Engine RPM",          "unit": "rpm",   "fmt": ".0f", "min": 0,   "max": 5500, "warn_low": 550, "warn_high": 4500},
    "SPEED":             {"label": "Vehicle Speed",       "unit": "mph",   "fmt": ".0f", "min": 0,   "max": 100},
    "COOLANT_TEMP":      {"label": "Coolant Temp",        "unit": "°F",    "fmt": ".0f", "min": -40, "max": 260,  "warn_low": 160, "warn_high": 220},
    "INTAKE_TEMP":       {"label": "Intake Air Temp",     "unit": "°F",    "fmt": ".0f", "min": -40, "max": 180,  "warn_high": 130},
    "AMBIENT_AIR_TEMP":  {"label": "Ambient Air Temp",    "unit": "°F",    "fmt": ".0f", "min": -40, "max": 140},
    "TIMING_ADVANCE":    {"label": "Ignition Timing",     "unit": "°",     "fmt": ".1f", "min": -20, "max": 60,   "warn_low": 0},
    "ENGINE_LOAD":       {"label": "Engine Load",         "unit": "%",     "fmt": ".1f", "min": 0,   "max": 100},
    "ABSOLUTE_LOAD":     {"label": "Absolute Load",       "unit": "%",     "fmt": ".1f", "min": 0,   "max": 100},
    "THROTTLE_POS":      {"label": "Throttle Position",   "unit": "%",     "fmt": ".1f", "min": 0,   "max": 100},
    "RELATIVE_THROTTLE_POS": {"label": "Relative Throttle", "unit": "%",   "fmt": ".1f", "min": 0,   "max": 100},
    "ACCELERATOR_POS_D": {"label": "Accelerator Pedal D", "unit": "%",     "fmt": ".1f", "min": 0,   "max": 100},
    "MAF":               {"label": "Mass Air Flow",       "unit": "g/s",   "fmt": ".2f", "min": 0,   "max": 120},
    "INTAKE_PRESSURE":   {"label": "Intake MAP",          "unit": "kPa",   "fmt": ".1f", "min": 0,   "max": 110},
    "BAROMETRIC_PRESSURE": {"label": "Barometric Press",  "unit": "inHg",  "fmt": ".2f", "min": 20,  "max": 32},
    "SHORT_FUEL_TRIM_1": {"label": "ST Fuel Trim B1",     "unit": "%",     "fmt": ".1f", "min": -25, "max": 25,   "warn_low": -12, "warn_high": 12},
    "LONG_FUEL_TRIM_1":  {"label": "LT Fuel Trim B1",     "unit": "%",     "fmt": ".1f", "min": -25, "max": 25,   "warn_low": -10, "warn_high": 10},
    "CONTROL_MODULE_VOLTAGE": {"label": "Module Voltage", "unit": "V",     "fmt": ".2f", "min": 8,   "max": 16,   "warn_low": 13.0, "warn_high": 15.0},
    "FUEL_STATUS":       {"label": "Fuel System",         "unit": "",      "fmt": "s"},
    "FUEL_LEVEL":        {"label": "Fuel Level",          "unit": "%",     "fmt": ".0f", "min": 0,   "max": 100,  "warn_low": 15},
    "FUEL_RATE":         {"label": "Fuel Rate (or est.)", "unit": "gph",   "fmt": ".2f", "min": 0,   "max": 10},
    "RUN_TIME":          {"label": "Engine Run Time",     "unit": "s",     "fmt": ".0f", "min": 0,   "max": 86400},
    "O2_S1_WR_VOLTAGE":  {"label": "AFR Up (WR V)",       "unit": "V",     "fmt": ".3f", "min": 0,   "max": 5.0},
    "O2_B1S2":           {"label": "O2 B1S2 (downstream)","unit": "V",     "fmt": ".3f", "min": 0,   "max": 1.2},
    "COMMANDED_EQUIV_RATIO": {"label": "Cmd Equiv Ratio", "unit": "",      "fmt": ".3f", "min": 0.5, "max": 1.5, "warn_low": 0.9, "warn_high": 1.1},
    "CATALYST_TEMP_B1S1": {"label": "Cat Temp B1S1",     "unit": "°F",    "fmt": ".0f", "min": 0,   "max": 1650, "warn_high": 1560},
    "TFT_PAN":            {"label": "Trans Pan Temp",     "unit": "°F",    "fmt": ".0f", "min": 0,   "max": 300,  "warn_high": 220, "crit_high": 250},
    "TFT_CONV":           {"label": "Trans Conv Temp",    "unit": "°F",    "fmt": ".0f", "min": 0,   "max": 300,  "warn_high": 230, "crit_high": 260},
}

# 2TR-FE reference notes (from FSM + community data)
OIL_PRESSURE_NOTE = (
    "OIL PRESSURE: NOT AVAILABLE as PSI via OBD-II on 2TR-FE.\n"
    "Only a low-pressure switch (if exposed) or no data. "
    "FSM (hot): ≥4.3 psi idle / 23-75 psi @ 3,000 RPM. "
    "Install a mechanical gauge for real monitoring."
)

TIMING_NOTE = (
    "Normal idle timing (warm, no load): typically 5–15° BTDC.\n"
    "Advances with RPM/load. Large retard under load often = knock sensor activity or issue."
)

# How many historical samples to keep for deltas / sparklines (per metric)
HISTORY_LEN = 40
# Minimum samples before we trust "change" calculations
MIN_HISTORY_FOR_DELTA = 6

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def _get_obd_command(name: str) -> Optional[obd.OBDCommand]:
    """Return the obd command object by its common name, or None."""
    if hasattr(obd.commands, name):
        return getattr(obd.commands, name)
    return None

def _format_fuel_status(value: Any) -> str:
    """Turn the (status1, status2) tuple into a short readable string."""
    if value is None:
        return "—"
    if isinstance(value, (list, tuple)) and len(value) >= 1:
        primary = value[0] or ""
        # Shorten the long official strings
        mapping = {
            "Open loop due to insufficient engine temperature": "Open (cold)",
            "Closed loop, using oxygen sensor feedback to determine fuel mix": "Closed loop",
            "Open loop due to engine load OR fuel cut due to deceleration": "Open (load/decel)",
            "Open loop due to system failure": "Open (fault)",
            "Closed loop, using at least one oxygen sensor but there is a fault in the feedback system": "Closed (O2 fault)",
        }
        short = mapping.get(primary, primary[:28] if primary else "—")
        return short
    return str(value)[:40]


def _format_value(name: str, value: Any) -> str:
    """Human friendly string for a value."""
    if value is None:
        return "—"
    if name == "FUEL_STATUS":
        return _format_fuel_status(value)
    meta = METRIC_META.get(name, {})
    fmt = meta.get("fmt", "")
    unit = meta.get("unit", "")
    try:
        if isinstance(value, (int, float)):
            if fmt == ".0f":
                s = f"{value:.0f}"
            elif fmt == ".1f":
                s = f"{value:.1f}"
            elif fmt == ".2f":
                s = f"{value:.2f}"
            elif fmt == ".3f":
                s = f"{value:.3f}"
            else:
                s = str(value)
            return f"{s} {unit}".strip()
        if hasattr(value, "magnitude"):  # pint Quantity
            # Avoid unit conversion headaches for things without matching units
            try:
                q = value.to(unit) if unit else value
                mag = q.magnitude
            except Exception:
                mag = value.magnitude
            if fmt == ".0f":
                return f"{mag:.0f} {unit}".strip()
            if fmt == ".1f":
                return f"{mag:.1f} {unit}".strip()
            if fmt == ".2f":
                return f"{mag:.2f} {unit}".strip()
            if fmt == ".3f":
                return f"{mag:.3f} {unit}".strip()
            return f"{mag} {unit}".strip()
        return str(value)
    except Exception:
        log.debug("format fallback", exc_info=True)
        return str(value)

def _sparkline(values: List[float], width: int = 18) -> str:
    """Very simple ascii sparkline from a list of numeric values."""
    if not values or len(values) < 2:
        return " " * width
    vals = [v for v in values if isinstance(v, (int, float))]
    if len(vals) < 2:
        return " " * width
    lo, hi = min(vals), max(vals)
    if hi - lo < 1e-9:
        return "▁" * width
    chars = "▁▂▃▄▅▆▇█"
    out = []
    step = (len(vals) - 1) / (width - 1) if width > 1 else 0
    for i in range(width):
        idx = int(round(i * step))
        idx = max(0, min(idx, len(vals) - 1))
        v = vals[idx]
        norm = (v - lo) / (hi - lo)
        cidx = int(norm * (len(chars) - 1))
        out.append(chars[max(0, min(cidx, len(chars) - 1))])
    return "".join(out)

def _color_for_value(name: str, val: Any, baseline: Optional[float] = None) -> str:
    """Return rich color name based on thresholds + optional baseline delta."""
    if val is None or not isinstance(val, (int, float)):
        return "white"
    meta = METRIC_META.get(name, {})
    low = meta.get("warn_low")
    high = meta.get("warn_high")

    # Fuel trim special: symmetric and important
    if name in ("SHORT_FUEL_TRIM_1", "LONG_FUEL_TRIM_1"):
        if abs(val) >= 18:
            return "red"
        if abs(val) >= 12:
            return "yellow"
        if abs(val) >= 8:
            return "cyan"
        return "green"

    if low is not None and val <= low:
        return "red" if val < (low - 5) else "yellow"
    if high is not None and val >= high:
        return "red" if val > (high + 5) else "yellow"

    # Voltage special
    if name == "CONTROL_MODULE_VOLTAGE":
        if val < 13.0:
            return "red"
        if val < 13.5:
            return "yellow"
        return "green"

    # Timing: low/negative under load is often bad, but at idle low positive is normal
    if name == "TIMING_ADVANCE":
        if val < 2:
            return "yellow"
        if val < -3:
            return "red"
        return "green"

    # Coolant (°F)
    if name == "COOLANT_TEMP":
        if val > 226:       # ~108°C
            return "red"
        if val > 215:       # ~102°C
            return "yellow"
        return "green"

    # Commanded equivalence ratio (1.0 = stoich)
    if name == "COMMANDED_EQUIV_RATIO":
        if val < 0.85 or val > 1.15:
            return "red"
        if val < 0.92 or val > 1.08:
            return "yellow"
        return "green"

    # Catalyst temp (°F)
    if name == "CATALYST_TEMP_B1S1":
        if val > 1616:      # ~880°C
            return "red"
        if val > 1508:      # ~820°C
            return "yellow"
        return "green"

    # Default: green if we have no strong opinion
    return "green"

def _severity_color(level: str) -> str:
    return {"CRIT": "red", "WARN": "yellow", "INFO": "cyan"}.get(level, "white")

# -----------------------------------------------------------------------------
# Main Monitor Class
# -----------------------------------------------------------------------------

class TacomaMonitor:
    def __init__(self, port: Optional[str] = None, log_path: Optional[str] = None,
                 poll_interval: float = 0.5, demo: bool = False, lite_pids: bool = False,
                 config: Optional[Dict[str, Any]] = None):
        self.cfg = config or cfgmod.load_config()
        self.th: Dict[str, Any] = self.cfg["thresholds"]
        self.log_dir: str = self.cfg.get("log_dir", "logs")
        self.port = port
        self.conn: Optional[obd.OBD] = None
        self.poll_interval = poll_interval
        self.demo = demo
        self.lite_pids = lite_pids  # True = CORE only; False = full INTERESTED set

        self.data: Dict[str, Any] = {}
        self.history: Dict[str, deque] = defaultdict(lambda: deque(maxlen=HISTORY_LEN))
        self.baseline: Dict[str, Any] = {}
        self.flags: List[Tuple[float, str, str]] = []  # (ts, level, msg)
        self.flag_cooldown: Dict[str, float] = {}  # simple spam control per message key

        self.running = False
        self.log_path = log_path
        self.log_file = None
        self.log_writer = None
        self._log_lock = threading.Lock()
        self._last_log_ts = 0.0
        self.log_interval = max(1.0, poll_interval)  # full snapshot every ~1s (or poll rate)

        # O2 / AFR tracker
        self.o2_last_val: Optional[float] = None
        self.o2_last_cross_ts: float = 0.0
        self.o2_switch_count: int = 0

        # Fuel-rate source tracking (2TR-FE rarely publishes PID 0x5E)
        self._fuel_rate_from_ecu = False
        self._fuel_rate_calculated = False

        # Mode 22 TFT polling
        self._last_tft_poll = 0.0
        self._tft_ok = False  # True once we get a valid response

        # Session statistics for the exit summary
        self.stats: Dict[str, Dict[str, float]] = defaultdict(lambda: {"min": float("inf"), "max": float("-inf")})
        self.flag_counts: Dict[str, int] = defaultdict(int)  # level -> count

        # RLock: _on_response holds the lock and may call _quick_flag_check,
        # which also needs it — a plain Lock deadlocks the whole monitor when
        # any alert fires (exactly the hang you saw).
        self._lock = threading.RLock()
        self._last_flag_prune = 0.0
        self.session_start = time.time()
        self.samples = 0

        self.console = Console()
        self._input_thread: Optional[threading.Thread] = None
        self._live: Optional[Live] = None
        self._demo_thread: Optional[threading.Thread] = None
        self._demo_t0 = time.time()

        # Reconnect watchdog state
        self.conn_state = "DEMO" if demo else "INIT"   # INIT / CONNECTED / STALE / RECONNECTING / DISCONNECTED
        self.reconnect_count = 0
        self.last_response_ts = 0.0
        self._watchdog_thread: Optional[threading.Thread] = None
        self._reconnecting = False

    # ---------------- Connection & Watching ----------------

    def connect(self) -> bool:
        # Always surface the oil pressure limitation immediately — this is the #1 thing users asked for.
        self._print_oil_warning()

        if self.demo:
            self.console.print("[bold magenta]DEMO MODE[/bold magenta] — feeding realistic 2TR-FE simulation data")
            self.console.print("[dim]No adapter required. All UI, flags, logging, and summary features work normally.[/dim]")
            return True

        self.console.print("[bold]Connecting to OBD-II adapter...[/bold]")
        ok = self._open_connection(verbose=True)
        self.conn_state = "CONNECTED" if ok else "DISCONNECTED"
        if ok:
            self.last_response_ts = time.time()
        return ok

    def _open_connection(self, verbose: bool = True) -> bool:
        try:
            # Async supports watch()/start()/stop() used by the live dashboard.
            # fast=False is more reliable on many USB ELM clones.
            self.conn = obd.Async(
                portstr=self.port,
                baudrate=None,          # auto
                protocol=None,          # auto (CAN for your truck)
                fast=False,
                timeout=1.0,            # give ELM more time — reduces stalls
                check_voltage=False,    # skip AT RV hang on some adapters
            )
        except Exception as e:
            log.exception("OBD connection error")
            if verbose:
                self.console.print(f"[red]Connection error:[/red] {e}")
            return False

        status = self.conn.status()

        def _port_label(c) -> str:
            # python-obd: port_name may be a method, property, or missing
            pn = getattr(c, "port_name", None)
            if callable(pn):
                try:
                    return str(pn())
                except Exception:
                    log.exception("suppressed error")
            if isinstance(pn, str) and pn:
                return pn
            p = getattr(c, "port", None)
            if p is not None and not callable(p):
                return str(getattr(p, "device", p))
            return self.port or "?"

        port_name = _port_label(self.conn)
        if status == obd.OBDStatus.NOT_CONNECTED:
            if not verbose:
                return False
            self.console.print("[red]Failed to connect.[/red] Check: adapter in truck OBD port, key ON/engine running, dialout group, port not in use.")
            self.console.print("Try:  ls /dev/ttyUSB* /dev/ttyACM* ; python -m serial.tools.list_ports")
            try:
                import serial.tools.list_ports as lp
                ports = [str(p.device) for p in lp.comports()]
                if ports:
                    self.console.print(f"Detected ports: {', '.join(ports)}")
            except Exception:
                log.exception("suppressed error")
            return False

        if not verbose:
            return status == obd.OBDStatus.CAR_CONNECTED
        proto = getattr(getattr(self.conn, "protocol", None), "name", None) or "unknown"
        self.console.print(f"[green]Connected[/green] via {port_name}  |  Protocol: {proto}  |  Status: {status}")

        # Show what we can actually query
        supported = []
        for name in INTERESTED:
            cmd = _get_obd_command(name)
            if cmd and cmd in self.conn.supported_commands:
                supported.append(name)
        self.console.print(f"Supported of interest: [cyan]{', '.join(supported) or 'none (will still try)'}[/cyan]")

        return True

    def _print_oil_warning(self):
        self.console.print(Panel.fit(
            Text(OIL_PRESSURE_NOTE, style="bold yellow"),
            title="[red]OIL PRESSURE — READ THIS[/red]",
            border_style="red",
            box=box.HEAVY,
        ))

    def start_watching(self):
        if self.demo:
            self._demo_thread = threading.Thread(target=self._demo_loop, daemon=True)
            self._demo_thread.start()
            self.console.print("[green]Demo data generator started[/green]")
            return

        if not self.conn:
            return
        if hasattr(self.conn, "stop"):
            try:
                self.conn.stop()
            except Exception:
                log.exception("suppressed error")

        # Default: full INTERESTED set. --lite uses CORE only if adapter stalls.
        watch_names = CORE_PIDS if getattr(self, "lite_pids", False) else INTERESTED
        watched = 0
        for name in watch_names:
            cmd = _get_obd_command(name)
            if cmd and cmd in self.conn.supported_commands:
                def make_cb(n: str):
                    def cb(response: obd.OBDResponse):
                        try:
                            self._on_response(n, response)
                        except Exception:
                            log.exception("suppressed error")  # never let a bad PID kill the async thread
                    return cb
                try:
                    self.conn.watch(cmd, callback=make_cb(name))
                    watched += 1
                except Exception:
                    log.exception("suppressed error")

        # Space out queries so the vLinker doesn't wedge
        if hasattr(self.conn, "delay_commands"):
            try:
                self.conn.delay_commands = True
            except Exception:
                log.exception("suppressed error")

        self.conn.start()
        mode = "LITE/CORE" if getattr(self, "lite_pids", False) else "FULL"
        self.console.print(
            f"[green]Live watch started[/green] ({watched} PIDs, {mode} set). "
            f"[dim]Use --lite if the adapter stalls.[/dim]"
        )

    # ---------------- Reconnect watchdog ----------------

    def _watchdog_loop(self):
        """Detect a dropped adapter or stale data and reconnect with exponential backoff."""
        rc = self.cfg.get("reconnect", {})
        stale_s = float(rc.get("stale_seconds", 5.0))
        b0 = float(rc.get("backoff_initial", 2.0))
        bmax = float(rc.get("backoff_max", 60.0))
        while self.running:
            time.sleep(1.0)
            if not self.running or self.demo:
                continue
            problem = None
            try:
                if self.conn is None:
                    problem = "no connection"
                elif self.conn.status() != obd.OBDStatus.CAR_CONNECTED:
                    problem = f"status {self.conn.status()}"
                elif self.last_response_ts and time.time() - self.last_response_ts > stale_s:
                    problem = f"no OBD data for {time.time() - self.last_response_ts:.0f}s"
                    self.conn_state = "STALE"
            except Exception:
                log.exception("watchdog status check failed")
                problem = "status check error"
            if not problem:
                continue
            log.warning("Adapter problem (%s); reconnecting", problem)
            self.add_flag("WARN", f"OBD link lost ({problem}) — reconnecting")
            delay = b0
            self._reconnecting = True
            while self.running:
                self.conn_state = "RECONNECTING"
                self._close_conn()
                if self._open_connection(verbose=False):
                    self.reconnect_count += 1
                    self.last_response_ts = time.time()
                    try:
                        self.start_watching()
                        self.conn_state = "CONNECTED"
                        log.info("Reconnected (#%d)", self.reconnect_count)
                        self.add_flag("INFO", f"OBD reconnected (#{self.reconnect_count})")
                        break
                    except Exception:
                        log.exception("re-watch after reconnect failed")
                self.conn_state = "DISCONNECTED"
                log.info("Reconnect failed; retrying in %.0fs", delay)
                end = time.time() + delay
                while self.running and time.time() < end:
                    time.sleep(0.25)
                delay = min(delay * 2, bmax)
            self._reconnecting = False

    def _close_conn(self):
        c = self.conn
        self.conn = None
        if not c:
            return
        for m in ("stop", "close"):
            try:
                if hasattr(c, m):
                    getattr(c, m)()
            except Exception:
                log.exception("conn.%s() failed", m)

    def _demo_loop(self):
        demo_loop(self)

    def _on_response(self, name: str, response: obd.OBDResponse):
        self.last_response_ts = time.time()
        if response.is_null() or response.value is None:
            return
        val = response.value
        # Normalize pint quantities to plain float where sensible for history
        numeric = None
        if hasattr(val, "magnitude"):
            numeric = float(val.magnitude)
        elif isinstance(val, (int, float)):
            numeric = float(val)

        # Convert to US customary units for display / thresholds / logging
        # (python-obd returns temps in °C, speed in km/h, baro in kPa, fuel rate in L/h)
        if numeric is not None:
            if name in ("COOLANT_TEMP", "INTAKE_TEMP", "CATALYST_TEMP_B1S1", "AMBIENT_AIR_TEMP"):
                numeric = numeric * 9.0 / 5.0 + 32.0          # °C → °F
                val = numeric
            elif name == "SPEED":
                numeric = numeric * 0.621371                   # km/h → mph
                val = numeric
            elif name == "BAROMETRIC_PRESSURE":
                numeric = numeric * 0.2953                     # kPa → inHg
                val = numeric
            elif name == "FUEL_RATE":
                numeric = numeric * 0.264172                   # L/h → gal/h
                val = numeric
                # Real ECU fuel-rate PID responded — never overwrite with estimate
                self._fuel_rate_from_ecu = True
                self._fuel_rate_calculated = False

        ts = time.time()
        with self._lock:
            self.data[name] = val
            if numeric is not None:
                self.history[name].append((ts, numeric))
                # Update session min/max
                st = self.stats[name]
                if numeric < st["min"]:
                    st["min"] = numeric
                if numeric > st["max"]:
                    st["max"] = numeric
            self.samples += 1

            # Track upstream AFR voltage (wideband does not "switch" like narrowband).
            # Keep last value for display / future lambda-deviation checks.
            if name == "O2_S1_WR_VOLTAGE" and numeric is not None:
                self.o2_last_val = numeric
                # No switch-count logic — WR sensors report continuous lambda/voltage.

            # When ECU does not publish FUEL_RATE (common on 2TR-FE), estimate it
            # from MAF + commanded lambda so MPG tiles still work.
            if name in ("MAF", "COMMANDED_EQUIV_RATIO") and numeric is not None:
                self._maybe_calc_fuel_rate()

            # Immediate light flagging (full analysis also runs in render)
            self._quick_flag_check(name, val, numeric)

    def _maybe_calc_fuel_rate(self) -> None:
        """Estimate fuel rate (gal/h) from MAF when ECU does not publish FUEL_RATE.

        Formula:
          AFR ≈ 14.7 * lambda   (lambda from COMMANDED_EQUIV_RATIO, default 1.0)
          fuel_g/s = MAF_g/s / AFR
          fuel_gph = (fuel_g/s * 3600) / 2835   (≈ gasoline density g/gal)

        Skips calculation if a real ECU FUEL_RATE PID response has been seen.
        """
        if getattr(self, "_fuel_rate_from_ecu", False):
            return

        maf = self.data.get("MAF")
        if maf is None:
            return
        maf_gps = float(maf.magnitude) if hasattr(maf, "magnitude") else float(maf)
        if maf_gps <= 0.05:
            return

        lambda_val = 1.0
        eq = self.data.get("COMMANDED_EQUIV_RATIO")
        if eq is not None:
            try:
                lambda_val = float(eq.magnitude) if hasattr(eq, "magnitude") else float(eq)
                if lambda_val < 0.5 or lambda_val > 2.0:
                    lambda_val = 1.0
            except (TypeError, ValueError):
                lambda_val = 1.0

        afr = 14.7 * lambda_val
        fuel_gps = maf_gps / afr
        # ~2835 g per US gallon of gasoline
        fuel_gph = (fuel_gps * 3600.0) / 2835.0
        fuel_gph = max(0.0, min(25.0, fuel_gph))  # clamp to sane range

        ts = time.time()
        self.data["FUEL_RATE"] = fuel_gph
        self.history["FUEL_RATE"].append((ts, fuel_gph))
        st = self.stats["FUEL_RATE"]
        if fuel_gph < st["min"]:
            st["min"] = fuel_gph
        if fuel_gph > st["max"]:
            st["max"] = fuel_gph
        self._fuel_rate_calculated = True

    def _poll_tft(self) -> None:
        """Query Toyota Mode 22 transmission fluid temps (slow path)."""
        if self.demo or self.conn is None or self._reconnecting:
            return
        now = time.time()
        if now - self._last_tft_poll < TFT_POLL_INTERVAL:
            return
        self._last_tft_poll = now

        for name, cmd in (("TFT_PAN", CMD_TFT_PAN), ("TFT_CONV", CMD_TFT_CONV)):
            try:
                resp = self.conn.query(cmd, force=True)
                if resp is None or resp.is_null() or resp.value is None:
                    continue
                val = resp.value
                numeric = float(val.magnitude) if hasattr(val, "magnitude") else float(val)
                # Sanity clamp — garbage responses sometimes appear on wrong header
                if numeric < -20 or numeric > 350:
                    continue
                with self._lock:
                    self.data[name] = numeric
                    self.history[name].append((now, numeric))
                    st = self.stats[name]
                    if numeric < st["min"]:
                        st["min"] = numeric
                    if numeric > st["max"]:
                        st["max"] = numeric
                    self._tft_ok = True
                # Temp flags
                meta = METRIC_META.get(name, {})
                if numeric >= meta.get("crit_high", 260):
                    self.add_flag("CRIT", f"{meta.get('label', name)} {numeric:.0f}°F — critically hot")
                elif numeric >= meta.get("warn_high", 230):
                    self.add_flag("WARN", f"{meta.get('label', name)} {numeric:.0f}°F — running hot")
            except Exception:
                log.exception("suppressed error")  # Mode 22 can fail quietly on some adapters / ignition states

    def _write_snapshot(self):
        """Write one clean wide-row CSV snapshot of all current metrics."""
        if not self.log_writer:
            return
        try:
            ts = time.time()
            with self._lock:
                data = dict(self.data)
                flags = list(self.flags)

            row = {
                "timestamp": datetime.fromtimestamp(ts).isoformat(timespec="milliseconds"),
            }
            log_keys = list(INTERESTED) + ["TFT_PAN", "TFT_CONV"]
            for name in log_keys:
                val = data.get(name)
                if val is None:
                    row[name] = ""
                elif hasattr(val, "magnitude"):
                    row[name] = f"{val.magnitude:.4f}"
                elif isinstance(val, (int, float)):
                    row[name] = f"{val:.4f}"
                else:
                    row[name] = str(val)

            if flags:
                recent = "; ".join(f[2] for f in flags[-3:])
                row["active_flags"] = recent[:200]
            else:
                row["active_flags"] = ""

            with self._log_lock:
                self.log_writer.writerow(row)
                self.log_file.flush()
        except Exception:
            log.exception("suppressed error")  # never let logging kill the monitor

    # ---------------- Flagging Logic ----------------

    def _quick_flag_check(self, name: str, val: Any, numeric: Optional[float]):
        """Lightweight immediate checks to populate flags quickly."""
        th = self.th
        now = time.time()
        if numeric is None:
            return

        key = None
        level = "WARN"
        msg = None

        if name == "LONG_FUEL_TRIM_1":
            if abs(numeric) >= th["ltft_crit"]:
                key = "ltft_crit"
                level = "CRIT"
                msg = f"LTFT {numeric:+.1f}% — strong indication of problem (vacuum leak, MAF, exhaust, fuel delivery)"
            elif abs(numeric) >= th["ltft_warn"]:
                key = "ltft_warn"
                msg = f"LTFT {numeric:+.1f}% — elevated, watch for drift"

        elif name == "SHORT_FUEL_TRIM_1":
            if abs(numeric) >= th["stft_crit"]:
                key = "stft_crit"
                level = "CRIT"
                msg = f"STFT {numeric:+.1f}% — large correction happening right now"

        elif name == "TIMING_ADVANCE":
            # Very low/negative timing is often bad, especially if load is high
            load = self.data.get("ENGINE_LOAD")
            load_val = load.magnitude if hasattr(load, "magnitude") else (load if isinstance(load, (int,float)) else 0)
            if numeric < th["timing_retard"]:
                key = "timing_retard"
                level = "CRIT" if load_val > th["timing_retard_crit_load"] else "WARN"
                msg = f"Timing {numeric:.1f}° — significant retard (knock? bad fuel? sensor?)"
            elif numeric < th["timing_low"] and load_val > th["timing_low_load"]:
                key = "timing_low_load"
                msg = f"Timing only {numeric:.1f}° at {load_val:.0f}% load"

        elif name == "COOLANT_TEMP":
            if numeric > th["coolant_overheat"]:       # ~110°C
                key = "ect_overheat"
                level = "CRIT"
                msg = f"Coolant {numeric:.0f}°F — overheating risk"
            elif numeric > th["coolant_hot"]:     # ~103°C
                key = "ect_hot"
                msg = f"Coolant {numeric:.0f}°F — running hot"

        elif name == "CONTROL_MODULE_VOLTAGE":
            if numeric < th["voltage_warn"]:
                key = "voltage_low"
                level = "CRIT" if numeric < th["voltage_crit"] else "WARN"
                msg = f"System voltage {numeric:.2f}V — charging or battery issue?"

        elif name == "FUEL_LEVEL":
            if numeric <= th["fuel_crit"]:
                key = "fuel_crit"
                level = "CRIT"
                msg = f"Fuel level {numeric:.0f}% — critically low"
            elif numeric <= th["fuel_low"]:
                key = "fuel_low"
                msg = f"Fuel level {numeric:.0f}% — getting low"

        if key and msg:
            # Cooldown to avoid spam
            last = self.flag_cooldown.get(key, 0)
            if now - last > th["flag_cooldown_s"]:   # seconds
                self.flag_cooldown[key] = now
                with self._lock:
                    self.flags.append((now, level, msg))
                    self.flag_counts[level] += 1
                    if level == "CRIT":
                        if not hasattr(self, "pinned_crit"):
                            self.pinned_crit = []
                        self.pinned_crit.append((now, msg))
                        self.pinned_crit = self.pinned_crit[-5:]
                    if len(self.flags) > 25:
                        self.flags = self.flags[-20:]

    def analyze_and_flag(self):
        """Deeper / trend-based analysis. Called from the render path."""
        th = self.th
        now = time.time()
        with self._lock:
            current_flags = []
            # Copy so we can work
            hist = {k: list(v) for k, v in self.history.items()}
            data = dict(self.data)
            base = dict(self.baseline)

        # Sustained fuel trim deviation (using recent history avg)
        for trim_name in ("LONG_FUEL_TRIM_1", "SHORT_FUEL_TRIM_1"):
            h = hist.get(trim_name, [])
            if len(h) >= MIN_HISTORY_FOR_DELTA:
                recent = [x[1] for x in h[-12:]]
                avg = sum(recent) / len(recent)
                if abs(avg) >= th["trim_avg_warn"]:
                    level = "CRIT" if abs(avg) >= th["trim_avg_crit"] else "WARN"
                    current_flags.append((now, level,
                        f"{trim_name} avg {avg:+.1f}% over last samples — persistent correction"))

        # Timing change vs baseline or vs recent
        timing = data.get("TIMING_ADVANCE")
        if timing is not None:
            tval = timing.magnitude if hasattr(timing, "magnitude") else timing
            if "TIMING_ADVANCE" in base:
                bval = base["TIMING_ADVANCE"]
                if isinstance(bval, (int, float)) and abs(tval - bval) > th["timing_baseline_delta"]:
                    current_flags.append((now, "WARN",
                        f"Timing {tval:.1f}° vs baseline {bval:.1f}° (Δ {tval-bval:+.1f}°)"))

            # Sudden change detector
            th_hist = hist.get("TIMING_ADVANCE", [])
            if len(th_hist) >= 8:
                recent = [x[1] for x in th_hist[-8:]]
                delta = recent[-1] - recent[0]
                if abs(delta) > th["timing_sudden_delta"]:
                    current_flags.append((now, "WARN",
                        f"Timing moved {delta:+.1f}° in recent window"))

        # Load vs RPM sanity (very rough)
        rpm = data.get("RPM")
        load = data.get("ENGINE_LOAD")
        if rpm and load:
            r = rpm.magnitude if hasattr(rpm, "magnitude") else rpm
            l = load.magnitude if hasattr(load, "magnitude") else load
            # Only flag while actually moving — idle/creep can show high calc load
            if th["lug_rpm_min"] < r < th["lug_rpm_max"] and l > th["lug_load"]:
                current_flags.append((now, "INFO", "Very high load at low RPM — lugging?"))

        # Upstream AFR (wideband) — no narrowband "switch" logic.
        # Optional soft check: if we have a reading and it is way outside
        # the normal Toyota AFR voltage window while warm, flag it.
        coolant = data.get("COOLANT_TEMP")
        rpm_val = None
        if rpm:
            rpm_val = rpm.magnitude if hasattr(rpm, "magnitude") else rpm
        cool_val = None
        if coolant:
            cool_val = coolant.magnitude if hasattr(coolant, "magnitude") else coolant

        if (cool_val is not None and cool_val > th["afr_check_coolant_min"] and
                rpm_val is not None and rpm_val > th["afr_check_rpm_min"] and
                self.o2_last_val is not None and self.samples > th["afr_check_min_samples"]):
            # Typical Toyota AFR sensor voltage around stoich is roughly 2.5–3.8 V.
            # Outside ~1.5–4.5 V for a sustained period is suspicious.
            v = self.o2_last_val
            if v < th["afr_volt_low"] or v > th["afr_volt_high"]:
                current_flags.append((now, "WARN",
                    f"AFR upstream voltage out of range ({v:.3f}V) — check sensor/wiring"))

        # Catalyst temp sanity (optional extra)
        cat = data.get("CATALYST_TEMP_B1S1")
        if cat is not None:
            cval = cat.magnitude if hasattr(cat, "magnitude") else cat
            if isinstance(cval, (int, float)) and cval > th["catalyst_hot"]:   # ~850°C
                current_flags.append((now, "WARN", f"Catalyst temp {cval:.0f}°F — running very hot"))

        # Merge into main flag list (dedup similar recent messages)
        with self._lock:
            if not hasattr(self, "pinned_crit"):
                self.pinned_crit = []  # (ts, msg) — survives prune until user clears
            for f in current_flags:
                # simple dedup: don't add identical message if one is very recent
                if not any(abs(f[0] - ff[0]) < 12 and f[2] == ff[2] for ff in self.flags):
                    self.flags.append(f)
                    self.flag_counts[f[1]] += 1
                    if f[1] == "CRIT":
                        # Pin last few CRITs so they don't scroll off / get pruned
                        self.pinned_crit.append((f[0], f[2]))
                        self.pinned_crit = self.pinned_crit[-5:]
            # prune old (but never drop pinned CRITs from the visible list)
            if now - self._last_flag_prune > 5:
                cutoff = now - 180  # keep last 3 minutes of flags
                pinned_msgs = {m for _, m in self.pinned_crit}
                self.flags = [
                    f for f in self.flags
                    if f[0] > cutoff or (f[1] == "CRIT" and f[2] in pinned_msgs)
                ]
                self._last_flag_prune = now
                if len(self.flags) > 30:
                    # Prefer keeping CRITs when trimming
                    crits = [f for f in self.flags if f[1] == "CRIT"]
                    rest = [f for f in self.flags if f[1] != "CRIT"]
                    self.flags = crits[-10:] + rest[-(25 - min(10, len(crits))):]

    # ---------------- Baseline & Logging Control ----------------

    def set_baseline(self):
        with self._lock:
            snap = {}
            for k, v in self.data.items():
                if hasattr(v, "magnitude"):
                    snap[k] = float(v.magnitude)
                elif isinstance(v, (int, float)):
                    snap[k] = float(v)
            self.baseline = snap
        self.console.print("[bold green]Baseline captured[/bold green] from current readings.")
        self.add_flag("INFO", "Baseline set by user")

    def add_flag(self, level: str, msg: str):
        with self._lock:
            self.flags.append((time.time(), level, msg))
            self.flag_counts[level] += 1

    def toggle_logging(self):
        if self.log_file:
            self.stop_logging()
            self.console.print("[yellow]Logging stopped[/yellow]")
            self.add_flag("INFO", "CSV logging stopped")
        else:
            self.start_logging()
            if self.log_file:
                self.console.print(f"[green]Logging to[/green] {self.log_path}")
                self.add_flag("INFO", "CSV logging started")

    def start_logging(self):
        if self.log_file:
            return
        path = self.log_path or make_log_path(self.log_dir)
        self.log_path = path
        try:
            self.log_file = open(path, "w", newline="")
            # Wide format: one row per snapshot with every metric as its own column
            fieldnames = ["timestamp"] + INTERESTED + ["TFT_PAN", "TFT_CONV", "active_flags"]
            self.log_writer = csv.DictWriter(self.log_file, fieldnames=fieldnames)
            self.log_writer.writeheader()
            self.console.print(f"[green]CSV logging started (snapshot mode):[/green] {path}")
        except Exception as e:
            log.exception("Failed to start CSV log %s", path)
            self.console.print(f"[red]Failed to start log:[/red] {e}")
            self.log_file = None
            self.log_writer = None

    def stop_logging(self):
        with self._log_lock:
            if self.log_file:
                try:
                    self.log_file.flush()
                    self.log_file.close()
                except Exception:
                    log.exception("suppressed error")
            self.log_file = None
            self.log_writer = None

    # ---------------- Rendering (Rich) ----------------

    def _build_header(self) -> RenderableType:
        elapsed = str(timedelta(seconds=int(time.time() - self.session_start)))
        if self.demo:
            status = "DEMO MODE"
            status_style = "bold yellow"
        else:
            port = self.port or "?"
            if self.conn:
                pn = getattr(self.conn, "port_name", None)
                if callable(pn):
                    try:
                        port = str(pn())
                    except Exception:
                        log.exception("suppressed error")
                elif isinstance(pn, str) and pn:
                    port = pn
            status = f"LIVE · {port} · {self.conn_state}"
            if self.reconnect_count:
                status += f" (reconnects: {self.reconnect_count})"
            status_style = "bold green"
        title = Text.assemble(
            ("Toyota Tacoma ", "bold white"),
            ("2019", "bold red"),
            ("  ·  ", "dim"),
            ("2TR-FE 2.7L I4", "bold yellow"),
            ("  ·  OBD-II Monitor  ·  ", "dim"),
            (elapsed, "bold orange1"),
            ("  ·  ", "dim"),
            (f"{self.samples} samples", "dim"),
        )
        return Panel(
            Text.assemble((status, status_style), "   ", title),
            style="bold white on black",
            border_style="red",
            box=box.HEAVY,
        )

    def _build_oil_panel(self) -> RenderableType:
        with self._lock:
            # Try to surface any switch-like status if the adapter ever gives us something
            oil_switch = self.data.get("OIL_PRESSURE_SWITCH") or self.data.get("OIL_PRESSURE") or None
            oil_str = _format_value("OIL_PRESSURE_SWITCH", oil_switch) if oil_switch else "UNAVAILABLE (see note)"
        txt = Text()
        txt.append(OIL_PRESSURE_NOTE + "\n", style="yellow")
        if oil_switch:
            txt.append(f"Reported switch value: {oil_str}\n", style="bold")
        txt.append("Real pressure requires mechanical gauge.", style="bold red")
        return Panel(txt, title="[red]Oil Pressure (2TR-FE Limitation)[/red]", border_style="red", box=box.ROUNDED)

    def _build_timing_panel(self) -> RenderableType:
        with self._lock:
            timing = self.data.get("TIMING_ADVANCE")
            timing_str = _format_value("TIMING_ADVANCE", timing) if timing is not None else "—"
            load = self.data.get("ENGINE_LOAD")
            load_str = _format_value("ENGINE_LOAD", load) if load is not None else "—"

            hist = list(self.history.get("TIMING_ADVANCE", []))
            spark = _sparkline([x[1] for x in hist]) if hist else ""

        body = Text()
        body.append(f"Current: {timing_str}    Load: {load_str}\n", style="bold")
        body.append(f"Trend: {spark}\n", style="cyan")
        body.append(TIMING_NOTE, style="dim")
        return Panel(body, title="Ignition Timing Advance", border_style="blue", box=box.ROUNDED)

    def _build_metrics_table(self) -> RenderableType:
        table = Table(
            show_header=True,
            header_style="bold cyan",
            box=box.SIMPLE,
            padding=(0, 1),
        )
        table.add_column("Metric", style="white")
        table.add_column("Value", justify="right")
        table.add_column("Min", justify="right", style="dim")
        table.add_column("Max", justify="right", style="dim")
        table.add_column("ΔBase", justify="right")
        table.add_column("Trend", style="cyan")

        with self._lock:
            data = dict(self.data)
            hist = {k: list(v) for k, v in self.history.items()}
            base = dict(self.baseline)

        for name in INTERESTED:
            if name not in METRIC_META:
                continue
            meta = METRIC_META[name]
            val = data.get(name)
            val_str = _format_value(name, val)
            color = _color_for_value(name, val.magnitude if hasattr(val, "magnitude") else val)

            h = hist.get(name, [])
            nums = [x[1] for x in h]
            minv = f"{min(nums):.1f}" if nums else "—"
            maxv = f"{max(nums):.1f}" if nums else "—"

            # delta from baseline
            delta = "—"
            if name in base:
                b = base[name]
                v = val.magnitude if hasattr(val, "magnitude") else val
                if isinstance(v, (int, float)) and isinstance(b, (int, float)):
                    d = v - b
                    delta = f"{d:+.1f}"

            spark = _sparkline(nums, width=14) if len(nums) >= 3 else ""

            table.add_row(
                meta["label"],
                Text(val_str, style=color),
                minv,
                maxv,
                delta,
                spark,
            )
        return table

    def _build_alerts_panel(self) -> RenderableType:
        # Rate-limit heavy analysis so render never blocks the ELM thread
        now = time.time()
        if now - getattr(self, "_last_analyze_ts", 0) > 1.5:
            try:
                self.analyze_and_flag()
            except Exception:
                log.exception("suppressed error")
            self._last_analyze_ts = now
        with self._lock:
            flags = list(self.flags)

        if not flags:
            content = Text("No active flags. All monitored parameters within expected bands so far.", style="green")
        else:
            t = Table(show_header=False, box=box.SIMPLE, padding=(0, 0))
            for ts, level, msg in sorted(flags, key=lambda x: -x[0])[:12]:
                age = int(time.time() - ts)
                t.add_row(
                    Text(f"{level:4}", style=_severity_color(level)),
                    Text(f"{msg} ", style="white"),
                    Text(f"({age}s ago)", style="dim"),
                )
            content = t

        return Panel(content, title="Alerts & Flags (changes + thresholds)", border_style="yellow", box=box.ROUNDED)

    def _build_footer(self) -> RenderableType:
        cmds = "b=baseline  l=toggle log  c=clear flags  q=quit  Ctrl-C also quits"
        log_status = f"Logging: {'ON' if self.log_file else 'OFF'}   |   Link: {self.conn_state}"
        return Text(f"{log_status}   |   {cmds}", style="dim")

    def get_renderable(self) -> RenderableType:
        try:
            header = self._build_header()
            oil = self._build_oil_panel()
            timing = self._build_timing_panel()
            metrics = self._build_metrics_table()
            alerts = self._build_alerts_panel()
            footer = self._build_footer()

            # Layout: top header, then two columns (oil+timing | metrics), then alerts, footer
            left = Group(oil, timing)
            right = metrics

            top = Table.grid(expand=True)
            top.add_row(left, right)

            return Group(
                header,
                top,
                alerts,
                footer,
            )
        except Exception as e:
            log.exception("render error")
            return Panel(f"Dashboard render error: {e}", border_style="red")

    # ---------------- Input Handling (Linux-friendly) ----------------

    def _handle_input(self, line: str):
        line = line.strip().lower()
        if not line:
            return
        if line in ("q", "quit", "exit"):
            self.stop()
        elif line in ("b", "baseline"):
            self.set_baseline()
        elif line in ("l", "log"):
            self.toggle_logging()
        elif line in ("c", "clear"):
            with self._lock:
                self.flags.clear()
                self.pinned_crit = []
            self.console.print("[dim]Flags cleared (including pinned CRITs)[/dim]")
        elif line in ("h", "help", "?"):
            self.console.print(Panel(
                "b / baseline   — capture current readings as your personal baseline\n"
                "l / log        — start or stop CSV logging\n"
                "c / clear      — clear the alerts panel\n"
                "q / quit       — exit cleanly\n"
                "Any other key shows this help.",
                title="Commands (type + Enter)",
            ))
        else:
            self.console.print("[dim]Unknown command. Type h for help.[/dim]")

    def _input_loop(self):
        """Non-blocking-ish input on Linux using select."""
        while self.running:
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.15)
                if r:
                    line = sys.stdin.readline()
                    if line:
                        self._handle_input(line)
            except Exception:
                # stdin issues (e.g. when piped) — just sleep
                log.debug("stdin read failed", exc_info=True)
                time.sleep(0.3)

    # ---------------- Main Run Loop ----------------

    def run(self):
        if not self.connect():
            return 1

        self.running = True

        # Optional auto-log from CLI
        if self.log_path:
            self.start_logging()

        self.start_watching()

        if not self.demo:
            self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True, name="obd-watchdog")
            self._watchdog_thread.start()

        # Start input thread (best effort)
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        # Live dashboard — update() pattern; screen=False is more stable on some terminals
        try:
            initial = self.get_renderable()
        except Exception:
            initial = Text("Starting…")

        # screen=False is more stable / Ctrl-C friendly than alternate-screen mode
        self._live = Live(
            initial,
            console=self.console,
            refresh_per_second=2,
            screen=False,
            transient=False,
            auto_refresh=False,  # we drive updates ourselves
        )

        try:
            with self._live:
                while self.running:
                    try:
                        # Slow Mode 22 TFT poll (header switch — keep off the hot path)
                        self._poll_tft()
                        # Periodic full-row CSV snapshot
                        if self.log_writer:
                            now = time.time()
                            if now - self._last_log_ts >= self.log_interval:
                                self._write_snapshot()
                                self._last_log_ts = now
                        # Refresh the dashboard with current data
                        self._live.update(self.get_renderable())
                        self._live.refresh()
                    except KeyboardInterrupt:
                        raise
                    except Exception as e:
                        # Never let a single bad frame kill the session
                        log.exception("render loop hiccup")
                        try:
                            self.console.print(f"[dim]render hiccup: {e}[/dim]")
                        except Exception:
                            log.exception("suppressed error")
                    time.sleep(0.25)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            try:
                self.console.print(f"[red]Live UI error:[/red] {e}")
            except Exception:
                log.exception("suppressed error")
        finally:
            self.stop()
        return 0

    def _print_session_summary(self):
        """Pretty exit report of what we saw this run."""
        elapsed = timedelta(seconds=int(time.time() - self.session_start))
        lines = []
        lines.append(f"Session length : {elapsed}")
        lines.append(f"Samples        : {self.samples}")
        lines.append(f"AFR last V     : {self.o2_last_val:.3f}" if self.o2_last_val is not None else "AFR last V     : —")

        # Flag tally
        if self.flag_counts:
            parts = [f"{lvl}:{cnt}" for lvl, cnt in sorted(self.flag_counts.items())]
            lines.append(f"Flags raised   : {', '.join(parts)}")
        else:
            lines.append("Flags raised   : none")

        # Key min/max
        interesting = [
            ("COOLANT_TEMP", "Coolant"),
            ("INTAKE_TEMP", "Intake"),
            ("TIMING_ADVANCE", "Timing"),
            ("LONG_FUEL_TRIM_1", "LTFT"),
            ("SHORT_FUEL_TRIM_1", "STFT"),
            ("CONTROL_MODULE_VOLTAGE", "Voltage"),
            ("CATALYST_TEMP_B1S1", "Cat Temp"),
            ("TFT_PAN", "Trans Pan"),
            ("TFT_CONV", "Trans Conv"),
            ("ABSOLUTE_LOAD", "Abs Load"),
            ("MAF", "MAF"),
        ]
        lines.append("")
        lines.append("Key ranges this session:")
        for key, label in interesting:
            st = self.stats.get(key)
            if st and st["min"] != float("inf"):
                unit = METRIC_META.get(key, {}).get("unit", "")
                fmt = METRIC_META.get(key, {}).get("fmt", ".1f")
                try:
                    mn = format(st["min"], fmt)
                    mx = format(st["max"], fmt)
                    lines.append(f"  {label:12}  {mn} → {mx} {unit}".rstrip())
                except Exception:
                    lines.append(f"  {label:12}  {st['min']:.2f} → {st['max']:.2f}")

        if self.log_path and self.log_file is None:  # already closed
            lines.append("")
            lines.append(f"Log written to : {self.log_path}")

        body = "\n".join(lines)
        self.console.print()
        self.console.print(Panel(
            body,
            title="[bold red]Session Summary[/bold red] [yellow]· Toyota Tacoma 2019 2TR-FE[/yellow]",
            border_style="cyan",
            box=box.ROUNDED,
        ))
        self.console.print("[bold]Monitor stopped. Drive safe, Bullet.[/bold]\n")

    def stop(self):
        self.running = False
        if getattr(self, "_stopped", False):
            return
        self._stopped = True
        if self.conn:
            try:
                if hasattr(self.conn, "stop"):
                    self.conn.stop()
            except Exception:
                log.exception("suppressed error")
            try:
                if hasattr(self.conn, "close"):
                    self.conn.close()
            except Exception:
                log.exception("suppressed error")
        self.stop_logging()
        try:
            n = cfgmod.cleanup_empty_logs(self.log_dir)
            if n:
                log.info("Removed %d empty/header-only CSV(s)", n)
        except Exception:
            log.exception("log cleanup failed")
        if self._live:
            try:
                self._live.stop()
            except Exception:
                log.exception("suppressed error")
        try:
            self._print_session_summary()
        except Exception:
            self.console.print("\n[bold]Monitor stopped. Drive safe.[/bold]")

    def add_manual_flag_for_test(self, msg: str):
        """Helper for development/testing."""
        self.add_flag("INFO", msg)

    def get_snapshot(self) -> Dict[str, Any]:
        """Return current state as a JSON-serializable dict for the web dashboard."""
        with self._lock:
            data = {}
            for name, val in self.data.items():
                if hasattr(val, "magnitude"):
                    data[name] = float(val.magnitude)
                elif isinstance(val, (int, float)):
                    data[name] = float(val)
                elif isinstance(val, (list, tuple)):
                    data[name] = [str(x) for x in val]
                else:
                    data[name] = str(val) if val is not None else None

            hist = {}
            for k, dq in self.history.items():
                hist[k] = [v for _, v in list(dq)[-40:]]

            flags = [
                {"ts": ts, "level": level, "msg": msg, "age": int(time.time() - ts)}
                for ts, level, msg in list(self.flags)[-15:]
            ]
            pinned = [
                {"ts": ts, "msg": msg, "age": int(time.time() - ts)}
                for ts, msg in list(getattr(self, "pinned_crit", []))[-5:]
            ]

            stats_out = {}
            for k, st in self.stats.items():
                if st["min"] != float("inf"):
                    stats_out[k] = {"min": st["min"], "max": st["max"]}

        return {
            "demo": self.demo,
            "conn_state": self.conn_state,
            "reconnect_count": self.reconnect_count,
            "samples": self.samples,
            "elapsed": int(time.time() - self.session_start),
            "afr_last_v": self.o2_last_val,
            "fuel_rate_calculated": getattr(self, "_fuel_rate_calculated", False),
            "data": data,
            "history": hist,
            "flags": flags,
            "pinned_crit": pinned,
            "stats": stats_out,
            "baseline": {k: float(v) for k, v in self.baseline.items()} if self.baseline else {},
        }


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Toyota Tacoma 2019 · 2TR-FE OBD-II live monitor + TRD web dashboard"
    )
    parser.add_argument("--config", default=None,
                        help="Path to config.json (default: config.json next to the script)")
    parser.add_argument("--port", "-p", default=None,
                        help="Serial port (e.g. /dev/ttyUSB0). Overrides config serial_port. Omit for auto-detect.")
    parser.add_argument("--log", "-l", action="store_true",
                        help="Start CSV logging immediately (file named with timestamp in log_dir).")
    parser.add_argument("--log-file", default=None,
                        help="Explicit path for CSV log instead of auto name.")
    parser.add_argument("--log-dir", default=None,
                        help="Directory for CSV logs and errors.log (overrides config log_dir)")
    parser.add_argument("--retention-days", type=float, default=None,
                        help="Gzip CSVs older than this many days on startup (overrides config; 0=off)")
    parser.add_argument("--interval", "-i", type=float, default=None,
                        help="Approximate poll / display interval (seconds). Overrides config (default 0.5)")
    parser.add_argument("--demo", action="store_true",
                        help="Simulated 2TR-FE data — no adapter needed")
    parser.add_argument("--lite", action="store_true",
                        help="Watch CORE PIDs only (fewer queries; use if adapter stalls)")
    parser.add_argument("--web", action="store_true",
                        help="Start local TRD cluster web dashboard (browser gauges + CSV save)")
    parser.add_argument("--web-port", type=int, default=None,
                        help="Port for the web dashboard (overrides config web_port, default 8765)")
    parser.add_argument("--no-color", action="store_true", help="Disable rich colors (rarely needed)")
    args = parser.parse_args()

    cfg = cfgmod.load_config(args.config)
    if args.port is not None:
        cfg["serial_port"] = args.port
    if args.web_port is not None:
        cfg["web_port"] = args.web_port
    if args.interval is not None:
        cfg["poll_interval"] = args.interval
    if args.log_dir is not None:
        cfg["log_dir"] = args.log_dir
    if args.retention_days is not None:
        cfg["log_retention_days"] = args.retention_days

    cfgmod.setup_error_logging(cfg["log_dir"])
    log.info("Starting tacoma_monitor (demo=%s)", args.demo)
    cfgmod.migrate_root_csvs(cfg["log_dir"])
    try:
        n = cfgmod.compress_old_logs(cfg["log_dir"], cfg.get("log_retention_days", 60))
        if n:
            log.info("Gzipped %d old CSV log(s)", n)
    except Exception:
        log.exception("log retention failed")

    if args.no_color:
        os.environ["NO_COLOR"] = "1"

    log_path = args.log_file
    if args.log and not log_path:
        log_path = make_log_path(cfg["log_dir"])

    monitor = TacomaMonitor(
        port=cfg.get("serial_port"),
        log_path=log_path,
        poll_interval=float(cfg["poll_interval"]),
        demo=args.demo,
        lite_pids=args.lite,
        config=cfg,
    )

    # Ctrl-C: first try clean stop; second hit forces exit
    _sig_hits = {"n": 0}
    def _sigint(sig, frame):
        _sig_hits["n"] += 1
        if _sig_hits["n"] >= 2:
            os._exit(1)
        monitor.running = False
        try:
            monitor.stop()
        except Exception:
            log.exception("stop() from signal failed")
        # Don't block forever if serial is wedged
        threading.Timer(2.0, lambda: os._exit(0)).start()
    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    if args.web:
        try:
            start_web_server(monitor, port=int(cfg["web_port"]))
        except Exception as e:
            log.exception("web server failed to start")
            monitor.console.print(f"[red]Web dashboard failed to start:[/red] {e}")

    sys.exit(monitor.run())


if __name__ == "__main__":
    main()
