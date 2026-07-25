from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal

from tools.attitude_ident_pid import analyze


def test_closed_loop_attitude_ident_script_fits_synthetic_prbs(tmp_path: Path) -> None:
    fs = 100.0
    duration_s = 25.0
    t = np.arange(0.0, duration_s, 1.0 / fs)
    bit_s = 0.25
    bits = ((np.floor(t / bit_s).astype(int) * 1103515245 + 12345) >> 3) & 1
    u = np.where(bits != 0, math.radians(0.8), -math.radians(0.8))

    wn = 2.0 * math.pi * 0.55
    zeta = 1.05
    sys = signal.TransferFunction([wn * wn], [1.0, 2.0 * zeta * wn, wn * wn])
    _, y, _ = signal.lsim(sys, U=u, T=t)

    df = pd.DataFrame(
        {
            "timestamp_us": (t * 1.0e6).astype(np.int64),
            "sequence": np.arange(len(t)),
            "ident_att_active": 1,
            "ident_att_axis": 1,
            "ident_att_mode": 1,
            "ident_att_signal_rad": u,
            "roll_deg": np.rad2deg(y),
            "pitch_deg": 0.0,
            "motor_output_reason_name": "stabilized_mix",
            "throttle_over_20": 1,
            "ctrl_protection_flags": 0,
            "ctrl_tilt_out_rad_0": 0.0,
            "ctrl_tilt_out_rad_1": 0.0,
        }
    )
    csv_path = tmp_path / "synthetic_ident.csv"
    df.to_csv(csv_path, index=False)

    fits = analyze(csv_path, inertia_kg_m2=0.051, target_zeta=1.1)

    assert len(fits) == 1
    fit = fits[0]
    assert fit.axis == "roll"
    assert fit.fit_pct > 85.0
    assert fit.wn_rad_s == pytest_approx_rel(wn, rel=0.18)
    assert fit.suggested_kr > 0.0
    assert fit.param_kp < 0.0
    assert fit.param_kd < 0.0


def pytest_approx_rel(value: float, rel: float):
    import pytest

    return pytest.approx(value, rel=rel)
