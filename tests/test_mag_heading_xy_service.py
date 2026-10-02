"""Host-compiled near-level heading service on the recovered board capture."""

from __future__ import annotations

import csv
import ctypes
import math
import shutil
import statistics
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "tests/fixtures/mag_heading/static_new_96.csv"


class Config(ctypes.Structure):
    _fields_ = [
        ("bias_x_mgauss", ctypes.c_float),
        ("bias_y_mgauss", ctypes.c_float),
        ("radius_xy_mgauss", ctypes.c_float),
        ("generation", ctypes.c_uint32),
        ("frame_contract_version", ctypes.c_uint32),
        ("axis_verified", ctypes.c_uint8),
        ("enabled", ctypes.c_uint8),
    ]


class Input(ctypes.Structure):
    _fields_ = [
        ("timestamp_us", ctypes.c_uint64),
        ("raw_flu_mgauss", ctypes.c_float * 3),
        ("roll_deg", ctypes.c_float),
        ("pitch_deg", ctypes.c_float),
        ("yaw_deg", ctypes.c_float),
    ]


class State(ctypes.Structure):
    _fields_ = [
        ("last_timestamp_us", ctypes.c_uint64),
        ("generation", ctypes.c_uint32),
        ("z_window_mgauss", ctypes.c_float * 10),
        ("last_z_mean_mgauss", ctypes.c_float),
        ("xy_rotation_rad", ctypes.c_float),
        ("z_count", ctypes.c_uint8),
        ("z_next", ctypes.c_uint8),
        ("heading_aligned", ctypes.c_uint8),
        ("ready_count", ctypes.c_uint32),
        ("tilt_count", ctypes.c_uint32),
    ]


@pytest.fixture(scope="module")
def service(tmp_path_factory: pytest.TempPathFactory):
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc unavailable")
    out = tmp_path_factory.mktemp("mag_heading") / "mag_heading.dll"
    command = [
        gcc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-shared",
        "-I",
        str(ROOT / "Services/Inc"),
        str(ROOT / "Services/Src/svc_mag_heading.c"),
        "-o",
        str(out),
        "-lm",
    ]
    subprocess.run(command, check=True, capture_output=True, text=True)
    lib = ctypes.CDLL(str(out))
    lib.SVC_MAG_HeadingUpdate.argtypes = [
        ctypes.POINTER(Config), ctypes.POINTER(State),
        ctypes.POINTER(Input), ctypes.POINTER(ctypes.c_float),
    ]
    lib.SVC_MAG_HeadingUpdate.restype = ctypes.c_int
    lib.SVC_MAG_HeadingConfigValid.argtypes = [ctypes.POINTER(Config)]
    lib.SVC_MAG_HeadingConfigValid.restype = ctypes.c_uint8
    return lib


def sample(index: int, x: float, y: float, z: float, *, roll: float = 0.0,
           yaw: float = 0.0) -> Input:
    return Input(
        timestamp_us=(index + 1) * 50_000,
        raw_flu_mgauss=(ctypes.c_float * 3)(x, y, z),
        roll_deg=roll,
        pitch_deg=0.0,
        yaw_deg=yaw,
    )


def test_real_capture_preserves_xy_direction_and_feeds_no_z(service) -> None:
    """输出只给方向：XY 按对齐旋转后归一到固定场强，Z 恒为 0。

    2026-09-30 台架：原来喂 10 点 Z 均值（含大硬铁偏移，FLU +567 mG），融合库按 2.6° 静态倾斜
    做倾斜补偿，把它投影进航向，每次重新对齐后 yaw 慢摆约 5°（data/analysis/mag-fit/2026-09-30）。
    """
    with CAPTURE.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 96  # sha256: 73d6f01c243c5e66... real recovered sample
    config = Config(-3.7253387, 135.43175, 268.29245, 1, 0, 1, 1)
    state = State()
    out = (ctypes.c_float * 3)()
    corrected_z: list[float] = []
    heading_deg: list[float] = []
    initial = rows[9]
    initial_angle = math.atan2(
        float(initial["y_flu_mgauss"]) - config.bias_y_mgauss,
        float(initial["x_flu_mgauss"]) - config.bias_x_mgauss,
    )
    c = math.cos(-initial_angle)
    s = math.sin(-initial_angle)
    for index, row in enumerate(rows):
        x, y, z = (float(row[key]) for key in
                   ("x_flu_mgauss", "y_flu_mgauss", "z_flu_mgauss"))
        result = service.SVC_MAG_HeadingUpdate(
            ctypes.byref(config), ctypes.byref(state),
            ctypes.byref(sample(index, x, y, z)), out,
        )
        if index < 9:
            assert result == 4  # warmup
        else:
            assert result == 5  # one fresh input per observation
            assert math.hypot(out[0], out[1]) == pytest.approx(500.0, abs=1e-3)
            corrected_z.append(out[2])
            heading_deg.append(math.degrees(math.atan2(out[1], out[0])))
    assert state.ready_count == 87
    assert corrected_z == [0.0] * 87
    # Z 均值仍然算，只作 MAGXY? 诊断显示。
    assert math.isfinite(state.last_z_mean_mgauss) and state.last_z_mean_mgauss != 0.0
    reference_heading = [
        math.degrees(math.atan2(
            s * (float(row["x_flu_mgauss"]) - config.bias_x_mgauss)
            + c * (float(row["y_flu_mgauss"]) - config.bias_y_mgauss),
            c * (float(row["x_flu_mgauss"]) - config.bias_x_mgauss)
            - s * (float(row["y_flu_mgauss"]) - config.bias_y_mgauss),
        ))
        for row in rows[9:]
    ]
    assert heading_deg == pytest.approx(reference_heading, abs=1e-5)
    assert statistics.stdev(heading_deg) < 0.5


def test_initial_relative_heading_matches_current_yaw(service) -> None:
    config = Config(-3.7253387, 135.43175, 268.29245, 3, 0, 1, 1)
    state = State()
    out = (ctypes.c_float * 3)()
    for index in range(10):
        result = service.SVC_MAG_HeadingUpdate(
            ctypes.byref(config), ctypes.byref(state),
            ctypes.byref(sample(index, 265.0, 194.0, 700.0, yaw=37.0)), out,
        )
    assert result == 5
    assert state.heading_aligned == 1
    assert math.degrees(math.atan2(out[1], out[0])) == pytest.approx(-37.0, abs=0.001)


def test_default_disabled_repeated_tilt_and_generation_reset(service) -> None:
    config = Config(-3.7253387, 135.43175, 268.29245, 1, 0, 0, 0)
    state = State()
    out = (ctypes.c_float * 3)(999.0, 999.0, 999.0)
    first = sample(0, 265.0, 194.0, 750.0)
    assert service.SVC_MAG_HeadingUpdate(
        ctypes.byref(config), ctypes.byref(state), ctypes.byref(first), out
    ) == 0
    assert out[0] == 999.0
    config.enabled = 1
    assert service.SVC_MAG_HeadingUpdate(
        ctypes.byref(config), ctypes.byref(state), ctypes.byref(first), out
    ) == 4
    assert service.SVC_MAG_HeadingUpdate(
        ctypes.byref(config), ctypes.byref(state), ctypes.byref(first), out
    ) == 1
    tilted = sample(1, 265.0, 194.0, 630.0, roll=6.0)
    assert service.SVC_MAG_HeadingUpdate(
        ctypes.byref(config), ctypes.byref(state), ctypes.byref(tilted), out
    ) == 3
    assert state.z_count == 0
    for index in range(2, 12):
        result = service.SVC_MAG_HeadingUpdate(
            ctypes.byref(config), ctypes.byref(state),
            ctypes.byref(sample(index, 265.0, 194.0, 630.0)), out,
        )
    assert result == 5
    config.generation = 2
    assert service.SVC_MAG_HeadingUpdate(
        ctypes.byref(config), ctypes.byref(state),
        ctypes.byref(sample(12, 265.0, 194.0, 630.0)), out,
    ) == 4
    assert state.z_count == 1


def test_nonfinite_configuration_and_sample_fail_closed(service) -> None:
    config = Config(float("nan"), 135.0, 268.0, 1, 0, 1, 1)
    state = State()
    out = (ctypes.c_float * 3)(999.0, 999.0, 999.0)
    assert service.SVC_MAG_HeadingConfigValid(ctypes.byref(config)) == 0
    assert service.SVC_MAG_HeadingUpdate(
        ctypes.byref(config), ctypes.byref(state),
        ctypes.byref(sample(0, 265.0, 194.0, 630.0)), out,
    ) == 2
    assert out[0] == 999.0
    config.bias_x_mgauss = -3.725
    assert service.SVC_MAG_HeadingUpdate(
        ctypes.byref(config), ctypes.byref(state),
        ctypes.byref(sample(1, float("nan"), 194.0, 630.0)), out,
    ) == 2
    assert state.z_count == 0
