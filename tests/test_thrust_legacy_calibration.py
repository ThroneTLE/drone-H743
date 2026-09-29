"""Original pressure calibration stays identical in measured acquisition."""
import threading
from types import SimpleNamespace

import pytest

from tools.thrust_bench.acquisition import AcquisitionEngine
from tools.thrust_bench.legacy_calibration import LegacyCalibration, LegacyLoadCell, legacy_grams
from tools.thrust_bench.scale import LoadCell, ScaleCalibration


@pytest.mark.parametrize("raw,expected", [(-100, -100), (100, 0), (200, 50), (300, 100), (400, 250), (500, 400), (600, 550)])
def test_original_piecewise_interpolation_and_extrapolation(raw, expected):
    points = [(500, 400), (100, 0), (300, 100)]
    calibration = LegacyCalibration.from_points(points)
    assert calibration.grams(raw) == expected
    assert legacy_grams(raw, points) == expected
    points.clear()
    assert calibration.grams(raw) == expected  # session calibration is immutable


def test_original_single_point_ratio_and_metadata():
    calibration = LegacyCalibration.from_points([(2190, 2181)])
    assert calibration.grams(2190) == 2181
    assert calibration.grams(1095) == 1090.5
    assert calibration.to_dict()["single_point_assumes_zero_origin"] is True
    assert legacy_grams(5, []) is None


@pytest.mark.parametrize("points", [[], [(0, 0)], [(0, 100)], [(100, 0)], [(1, float('nan'))], [(1, 2), (1, 3)], [(1, 2), (3, 2)]])
def test_unusable_calibration_is_not_turned_into_zero_weight(points):
    with pytest.raises(ValueError):
        LegacyCalibration.from_points(points)


def test_tare_subtracts_calibrated_weight_not_raw_in_a_different_segment():
    cal = LegacyCalibration.from_points([(100, 0), (300, 100), (500, 400)])
    cell = LegacyLoadCell(None, 4, cal, register=6)
    assert cell.grams_from_raw(400) == 250
    cell.read_raw = lambda: 200
    assert cell.tare(2) == 200
    assert cell.grams_from_raw(400) == 200
    assert cell.grams_from_raw(200) == 0
    assert cell.register == 6
    assert cal.points == ((100, 0), (300, 100), (500, 400))


@pytest.mark.parametrize("legacy", [False, True])
def test_real_acquisition_uses_one_raw_sample_and_cells_calibration(legacy):
    if legacy:
        cell = LegacyLoadCell(None, 1, LegacyCalibration.from_points([(100, 0), (300, 100), (500, 400)]))
        expected = 250
    else:
        cell = LoadCell(None, 1, ScaleCalibration(points=[(100, 0), (500, 200)]).fit())
        expected = 150
    reads = []
    cell.read_raw = lambda: reads.append(400) or 400
    rows = []
    shutdown = threading.Event()
    def record(**data):
        rows.append(data)
        shutdown.set()
    # Execute the production worker for one iteration without devices/threads.
    engine = SimpleNamespace(_observation_lock=threading.Lock(), _shutdown=shutdown, clock=lambda: 12.5, load_cell=cell,
                             store=SimpleNamespace(raw_scale=record), _latest_scale=None)
    AcquisitionEngine._scale_loop(engine)
    assert reads == [400]
    assert engine._latest_scale == (expected, 12.5)
    assert rows == [{"host_time_s": 12.5, "raw": 400, "grams": expected}]
