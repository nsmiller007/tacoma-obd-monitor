"""Demo-mode data generator (no adapter). Takes the monitor as an argument -> no import cycles."""
import logging
import math
import random
import time

log = logging.getLogger("tacoma")


def demo_loop(monitor):
    """Generate realistic 2TR-FE style values that evolve over time."""

    while monitor.running:
        try:
            demo_tick(monitor, math, random)
        except Exception:
            log.exception("demo tick failed")  # never let demo data kill the monitor
        time.sleep(0.35)

def demo_tick(monitor, math, random):
    t = time.time() - monitor._demo_t0
    # Phases: 0-30s cold idle, 30-90s warm-up, then mixed driving
    phase = "cold" if t < 30 else ("warmup" if t < 90 else "drive")

    # Base values by phase
    if phase == "cold":
        rpm = 850 + random.uniform(-30, 40)
        speed = 0
        coolant = 35 + t * 0.9 + random.uniform(-1, 1)
        intake = 28 + random.uniform(-2, 2)
        load = 18 + random.uniform(-3, 4)
        throttle = 8 + random.uniform(-1, 2)
        timing = 8 + random.uniform(-1, 2)
        stft = random.uniform(-3, 5)
        ltft = random.uniform(0, 4)
        fuel_status = ("Open loop due to insufficient engine temperature", "")
        # Wideband AFR voltage (Toyota-style ~3.0–3.5 V around stoich)
        o2_up = 3.25 + 0.08 * math.sin(t * 0.8) + random.uniform(-0.05, 0.05)
    elif phase == "warmup":
        rpm = 780 + random.uniform(-20, 30)
        speed = 0
        coolant = min(92, 55 + (t - 30) * 0.7 + random.uniform(-1, 1))
        intake = 32 + random.uniform(-2, 2)
        load = 16 + random.uniform(-2, 3)
        throttle = 7 + random.uniform(-1, 1)
        timing = 10 + random.uniform(-1, 2)
        stft = random.uniform(-4, 4)
        ltft = random.uniform(1, 6)
        fuel_status = ("Closed loop, using oxygen sensor feedback to determine fuel mix", "")
        o2_up = 3.30 + 0.12 * math.sin(t * 1.4) + random.uniform(-0.06, 0.06)
    else:
        # Simulated drive cycle with some variation
        cycle = (t - 90) % 60
        if cycle < 15:          # light cruise
            rpm = 1800 + random.uniform(-80, 100)
            speed = 55 + random.uniform(-4, 6)
            load = 28 + random.uniform(-4, 6)
            throttle = 18 + random.uniform(-2, 3)
            timing = 22 + random.uniform(-2, 3)
        elif cycle < 30:        # acceleration
            rpm = 2800 + random.uniform(-150, 200)
            speed = 70 + random.uniform(-5, 8)
            load = 55 + random.uniform(-6, 10)
            throttle = 42 + random.uniform(-4, 6)
            timing = 14 + random.uniform(-3, 4)
        elif cycle < 45:        # harder pull
            rpm = 3400 + random.uniform(-100, 150)
            speed = 85 + random.uniform(-5, 7)
            load = 72 + random.uniform(-5, 8)
            throttle = 58 + random.uniform(-4, 5)
            timing = 8 + random.uniform(-4, 3)
        else:                   # deceleration / coast
            rpm = 1400 + random.uniform(-60, 80)
            speed = 48 + random.uniform(-6, 5)
            load = 12 + random.uniform(-3, 4)
            throttle = 5 + random.uniform(-1, 2)
            timing = 18 + random.uniform(-2, 3)

        coolant = 91 + random.uniform(-1.5, 2.5)
        intake = 38 + random.uniform(-3, 4)
        stft = random.uniform(-6, 7)
        ltft = 3.5 + random.uniform(-2, 3)
        fuel_status = ("Closed loop, using oxygen sensor feedback to determine fuel mix", "")
        o2_up = 3.28 + 0.15 * math.sin(t * 2.0) + random.uniform(-0.08, 0.08)

    # Shared derived values
    maf = max(2.0, (rpm / 1000) * (load / 30) * 4.2 + random.uniform(-1, 1.5))
    abs_load = min(95, load * 1.15 + random.uniform(-3, 4))
    voltage = 14.1 + random.uniform(-0.15, 0.18)
    o2_down = 0.65 + random.uniform(-0.08, 0.08)
    equiv = 1.0 + random.uniform(-0.02, 0.02)
    cat_temp = min(780, 120 + (coolant - 30) * 6.5 + (load * 3) + random.uniform(-15, 20))

    # Occasionally inject a mild anomaly so flags can fire
    if phase == "drive" and random.random() < 0.04:
        ltft = random.choice([14.5, -13.0, 16.0])
    if phase == "drive" and random.random() < 0.03:
        timing = -1.5 + random.uniform(-1, 1)

    # Demo values in native OBD units (converted in _on_response)
    ambient_c = 22 + random.uniform(-3, 5)                 # ~72°F-ish Oklahoma day
    baro_kpa = 98 + random.uniform(-2, 2)                  # ~29 inHg
    fuel_level = max(8, 62 - t * 0.02 + random.uniform(-1, 1))
    run_time = max(0, int(t))
    map_kpa = max(20, 101 - load * 0.55 + random.uniform(-3, 3))
    rel_throttle = max(0, throttle * 0.85 + random.uniform(-2, 2))
    accel_pedal = max(0, throttle * 0.95 + random.uniform(-3, 3))
    # rough L/h from load/rpm for demo
    fuel_rate_lph = max(0.4, (rpm / 1000) * (load / 40) * 2.8 + random.uniform(-0.2, 0.3))

    # Feed through the same path as real responses
    fake = {
        "RPM": rpm,
        "SPEED": speed,
        "COOLANT_TEMP": coolant,
        "INTAKE_TEMP": intake,
        "AMBIENT_AIR_TEMP": ambient_c,
        "TIMING_ADVANCE": timing,
        "ENGINE_LOAD": load,
        "ABSOLUTE_LOAD": abs_load,
        "THROTTLE_POS": throttle,
        "RELATIVE_THROTTLE_POS": rel_throttle,
        "ACCELERATOR_POS_D": accel_pedal,
        "MAF": maf,
        "INTAKE_PRESSURE": map_kpa,
        "BAROMETRIC_PRESSURE": baro_kpa,
        "SHORT_FUEL_TRIM_1": stft,
        "LONG_FUEL_TRIM_1": ltft,
        "CONTROL_MODULE_VOLTAGE": voltage,
        "FUEL_STATUS": fuel_status,
        "FUEL_LEVEL": fuel_level,
        "FUEL_RATE": fuel_rate_lph,
        "RUN_TIME": run_time,
        "O2_S1_WR_VOLTAGE": max(1.5, min(4.5, o2_up)),
        "O2_B1S2": max(0.1, min(0.9, o2_down)),
        "COMMANDED_EQUIV_RATIO": equiv,
        "CATALYST_TEMP_B1S1": cat_temp,
    }

    for name, val in fake.items():
        class FakeResp:
            def __init__(self, v):
                self.value = v
            def is_null(self):
                return False
        monitor._on_response(name, FakeResp(val))

    # Demo TFT (Mode 22 path is skipped in demo — inject directly)
    tft_pan = 175 + random.uniform(-5, 12) + (load * 0.15)
    tft_conv = tft_pan + random.uniform(5, 25)
    with monitor._lock:
        monitor.data["TFT_PAN"] = tft_pan
        monitor.data["TFT_CONV"] = tft_conv
        monitor.history["TFT_PAN"].append((time.time(), tft_pan))
        monitor.history["TFT_CONV"].append((time.time(), tft_conv))
        monitor._tft_ok = True

