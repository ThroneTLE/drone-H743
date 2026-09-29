"""DShot-only current pairing shared by displays and offline modelling."""
from __future__ import annotations
import math


def valid_current(value, age_ms, *, max_age_ms=1000.0):
    return (value is not None and age_ms is not None
            and math.isfinite(value) and value >= 0
            and math.isfinite(age_ms) and 0 <= age_ms <= max_age_ms)


def dshot_total_current(upper_a, lower_a, upper_age_ms, lower_age_ms,
                        *, max_age_ms=1000.0, max_skew_ms=50.0):
    if not (valid_current(upper_a, upper_age_ms, max_age_ms=max_age_ms)
            and valid_current(lower_a, lower_age_ms, max_age_ms=max_age_ms)):
        return None
    if abs(upper_age_ms-lower_age_ms) > max_skew_ms:
        return None
    return float(upper_a)+float(lower_a)


def dshot_input_power(voltage_v, voltage_age_ms, upper_a, lower_a,
                      upper_age_ms, lower_age_ms, *, max_age_ms=1000.0,
                      max_voltage_age_ms=250.0, max_skew_ms=50.0):
    current = dshot_total_current(upper_a, lower_a, upper_age_ms, lower_age_ms,
                                 max_age_ms=max_age_ms, max_skew_ms=max_skew_ms)
    if (current is None or voltage_v is None or voltage_age_ms is None
            or not math.isfinite(voltage_v) or voltage_v <= 0
            or not math.isfinite(voltage_age_ms)
            or not 0 <= voltage_age_ms <= max_voltage_age_ms):
        return None
    if max(voltage_age_ms, upper_age_ms, lower_age_ms)-min(
            voltage_age_ms, upper_age_ms, lower_age_ms) > max_skew_ms:
        return None
    return float(voltage_v)*current
