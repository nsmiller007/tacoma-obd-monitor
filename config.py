"""Config loading, log-path helpers, log retention and error logging for tacoma_monitor."""
import copy
import gzip
import json
import logging
import logging.handlers
import shutil
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

BASE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = BASE_DIR / "config.json"

DEFAULTS: Dict[str, Any] = {
    "serial_port": None,           # null = auto-detect
    "web_port": 8765,
    "poll_interval": 0.5,
    "log_dir": "logs",
    "log_retention_days": 60,      # CSVs older than this are gzipped on startup (0 = off)
    "reconnect": {
        "stale_seconds": 5.0,
        "backoff_initial": 2.0,
        "backoff_max": 60.0,
    },
    "thresholds": {
        "ltft_warn": 12, "ltft_crit": 18,
        "stft_crit": 20,
        "trim_avg_warn": 15, "trim_avg_crit": 18,
        "timing_retard": -2, "timing_low": 3,
        "timing_retard_crit_load": 40, "timing_low_load": 30,
        "timing_baseline_delta": 12, "timing_sudden_delta": 10,
        "coolant_hot": 217, "coolant_overheat": 230,
        "voltage_warn": 12.8, "voltage_crit": 12.0,
        "fuel_low": 15, "fuel_crit": 10,
        "lug_rpm_min": 800, "lug_rpm_max": 2000, "lug_load": 85,
        "afr_check_coolant_min": 167, "afr_check_rpm_min": 600, "afr_check_min_samples": 40,
        "afr_volt_low": 1.2, "afr_volt_high": 5.0,
        "catalyst_hot": 1560,
        "flag_cooldown_s": 8.0,
    },
}

log = logging.getLogger("tacoma")


def _merge(base: Dict[str, Any], over: Dict[str, Any]) -> Dict[str, Any]:
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


def load_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load config.json over DEFAULTS. Missing/invalid file -> defaults."""
    cfg = copy.deepcopy(DEFAULTS)
    p = Path(path) if path else DEFAULT_CONFIG_PATH
    if p.exists():
        try:
            with open(p, "r", encoding="utf-8") as f:
                _merge(cfg, json.load(f))
        except Exception:
            log.exception("Failed to read config %s; using defaults", p)
    return cfg


def resolve_log_dir(log_dir: str) -> Path:
    d = Path(log_dir).expanduser()
    if not d.is_absolute():
        d = BASE_DIR / d
    d.mkdir(parents=True, exist_ok=True)
    return d


def make_log_path(log_dir: str = "logs") -> str:
    """The single place CSV log names are built."""
    return str(resolve_log_dir(log_dir) / f"tacoma_obd_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")


def setup_error_logging(log_dir: str = "logs") -> None:
    if any(getattr(h, "_tacoma", False) for h in log.handlers):
        return
    h = logging.handlers.RotatingFileHandler(
        resolve_log_dir(log_dir) / "errors.log", maxBytes=1_000_000, backupCount=3, encoding="utf-8")
    h._tacoma = True
    h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(threadName)s %(message)s"))
    log.addHandler(h)
    log.setLevel(logging.INFO)
    log.propagate = False


def migrate_root_csvs(log_dir: str = "logs") -> None:
    d = resolve_log_dir(log_dir)
    for f in BASE_DIR.glob("tacoma_obd_*.csv"):
        try:
            shutil.move(str(f), str(d / f.name))
        except Exception:
            log.exception("Could not move %s into %s", f, d)


def compress_old_logs(log_dir: str = "logs", days: float = 60) -> int:
    if not days or days <= 0:
        return 0
    cutoff = time.time() - days * 86400
    n = 0
    for f in resolve_log_dir(log_dir).glob("*.csv"):
        try:
            if f.stat().st_mtime < cutoff:
                gz = f.with_suffix(".csv.gz")
                with open(f, "rb") as src, gzip.open(gz, "wb") as dst:
                    shutil.copyfileobj(src, dst)
                shutil.copystat(f, gz)
                f.unlink()
                n += 1
        except Exception:
            log.exception("Failed to gzip %s", f)
    return n


def cleanup_empty_logs(log_dir: str = "logs") -> int:
    """Delete zero-byte and header-only CSVs."""
    n = 0
    for f in resolve_log_dir(log_dir).glob("*.csv"):
        try:
            if f.stat().st_size == 0:
                f.unlink(); n += 1
                continue
            with open(f, "r", encoding="utf-8", errors="replace") as fh:
                lines = [ln for ln, _ in zip(fh, range(2)) if ln.strip()]
            if len(lines) <= 1:
                f.unlink(); n += 1
        except Exception:
            log.exception("Failed to clean %s", f)
    return n
