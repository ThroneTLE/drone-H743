"""Tests for :mod:`tools.mag_cal_fit`.

All data here is synthetic (Fibonacci-sphere direction sampling plus a
closed-form, hand-built rotation). There is no captured QMC5883L data in
this repository -- the board's magnetometer has never been sampled for
calibration -- so these tests only prove the algebra is self-consistent on
data with known ground truth. They are not evidence that a real sensor
would fit well; see ``tools/mag_cal_fit.py``'s module docstring and the
delivery notes for this work order.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from tools import mag_cal_fit
from tools.mag_cal_fit import (
    DEFAULT_THRESHOLDS,
    MagCalFitError,
    MagCalFitThresholds,
    apply_mag_calibration,
    evaluate_coverage,
    fit_mag_calibration,
)


# ---------------------------------------------------------------------------
# Synthetic data helpers
# ---------------------------------------------------------------------------


def _rotation_matrix(axis: tuple[float, float, float], angle_rad: float) -> np.ndarray:
    """Rodrigues' rotation formula; avoids a SciPy dependency in tests."""

    axis_array = np.asarray(axis, dtype=float)
    axis_array = axis_array / np.linalg.norm(axis_array)
    x, y, z = axis_array
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    outer = np.outer(axis_array, axis_array)
    return (
        math.cos(angle_rad) * np.eye(3)
        + math.sin(angle_rad) * cross
        + (1.0 - math.cos(angle_rad)) * outer
    )


def _fibonacci_sphere(count: int) -> np.ndarray:
    """Near-uniform unit direction vectors covering the full sphere."""

    indices = np.arange(count, dtype=float) + 0.5
    phi = np.arccos(1.0 - 2.0 * indices / count)
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    theta = golden_angle * indices
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)
    return np.column_stack([x, y, z])


def _volume_preserving_symmetric_matrix() -> np.ndarray:
    """A fixed symmetric positive-definite matrix with det == 1.

    Eigenvalues (1.2, 1.0, 1/1.2) multiply to exactly 1, matching the
    fit's own volume-preserving convention (see module docstring), and the
    rotation gives off-diagonal terms so the test exercises full 3x3
    recovery, not just a diagonal special case.
    """

    rotation = _rotation_matrix((1.0, 1.0, 1.0), math.radians(25.0))
    eigenvalues = np.diag([1.2, 1.0, 1.0 / 1.2])
    return rotation @ eigenvalues @ rotation.T


def _synthesize_samples(
    *,
    directions: np.ndarray,
    bias_true: tuple[float, float, float],
    matrix_true: np.ndarray,
    radius_mgauss: float,
    noise_sigma_mgauss: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Invert ``corrected = M @ (raw - b)`` to build noisy raw samples."""

    corrected = directions * radius_mgauss
    matrix_inverse = np.linalg.inv(matrix_true)
    raw = (matrix_inverse @ corrected.T).T + np.asarray(bias_true)
    raw += rng.normal(scale=noise_sigma_mgauss, size=raw.shape)
    return raw


def _single_axis_wobble_samples(
    *, count: int, rng: np.random.Generator
) -> np.ndarray:
    """Samples from spinning (mostly) about one body axis: bad coverage.

    A pure yaw-only spin traces an exact circle (a 2-D subspace), which
    would be rejected by the coplanar/collinear degeneracy check rather
    than exercising the coverage judgement. Adding a few degrees of
    roll/pitch wobble -- an unsteady hand, not a perfect bench fixture --
    keeps the point cloud genuinely 3-D (passes the rank check) while
    still never letting the Z-body-axis field component change sign, so
    :func:`evaluate_coverage` must be the thing that rejects it.
    """

    field_world = np.array([250.0, 0.0, 300.0])
    thetas = np.linspace(0.0, 4.0 * math.pi, count, endpoint=False)
    samples = np.empty((count, 3), dtype=float)
    for index, theta in enumerate(thetas):
        wobble_x = math.radians(rng.normal(scale=6.0))
        wobble_y = math.radians(rng.normal(scale=6.0))
        wobble = _rotation_matrix((1.0, 0.0, 0.0), wobble_x) @ _rotation_matrix(
            (0.0, 1.0, 0.0), wobble_y
        )
        spin = _rotation_matrix((0.0, 0.0, 1.0), theta)
        rotation = spin @ wobble
        samples[index] = rotation.T @ field_world
    samples += rng.normal(scale=1.5, size=samples.shape)
    return samples


# ---------------------------------------------------------------------------
# Closed-loop synthetic recovery
# ---------------------------------------------------------------------------


def test_synthetic_closed_loop_recovers_known_hard_and_soft_iron() -> None:
    rng = np.random.default_rng(20260920)
    bias_true = (30.0, -15.0, 50.0)
    matrix_true = _volume_preserving_symmetric_matrix()
    directions = _fibonacci_sphere(400)
    raw = _synthesize_samples(
        directions=directions,
        bias_true=bias_true,
        matrix_true=matrix_true,
        radius_mgauss=500.0,
        noise_sigma_mgauss=1.5,
        rng=rng,
    )

    result = fit_mag_calibration(raw)

    fitted_bias = np.asarray(result.hard_iron_offset_mgauss)
    fitted_matrix = np.asarray(result.soft_iron_matrix)
    assert np.allclose(fitted_bias, bias_true, atol=3.0)
    assert np.allclose(fitted_matrix, matrix_true, atol=0.05)
    assert result.determinant == pytest.approx(1.0, abs=0.02)
    assert result.determinant > 0.0
    assert result.fitted_radius_mgauss == pytest.approx(500.0, rel=0.02)

    # Corrected samples must land on (approximately) one sphere: same
    # magnitude regardless of which direction the sample came from.
    corrected = np.asarray(apply_mag_calibration(raw, result.hard_iron_offset_mgauss, result.soft_iron_matrix))
    norms = np.linalg.norm(corrected, axis=1)
    assert float(np.std(norms)) < 5.0
    assert float(np.std(norms)) / float(np.mean(norms)) < 0.02


def test_pure_hard_iron_does_not_overfit_a_spurious_soft_iron_matrix() -> None:
    rng = np.random.default_rng(4)
    bias_true = (40.0, -20.0, 10.0)
    matrix_true = np.eye(3)
    directions = _fibonacci_sphere(350)
    raw = _synthesize_samples(
        directions=directions,
        bias_true=bias_true,
        matrix_true=matrix_true,
        radius_mgauss=480.0,
        noise_sigma_mgauss=1.5,
        rng=rng,
    )

    result = fit_mag_calibration(raw)

    fitted_bias = np.asarray(result.hard_iron_offset_mgauss)
    fitted_matrix = np.asarray(result.soft_iron_matrix)
    assert np.allclose(fitted_bias, bias_true, atol=3.0)
    assert np.allclose(fitted_matrix, np.eye(3), atol=0.05)
    off_diagonal = fitted_matrix - np.diag(np.diag(fitted_matrix))
    assert float(np.max(np.abs(off_diagonal))) < 0.03


# ---------------------------------------------------------------------------
# Coverage judgement
# ---------------------------------------------------------------------------


def test_full_sphere_coverage_passes() -> None:
    directions = _fibonacci_sphere(300)
    raw = directions * 400.0 + np.array([10.0, -5.0, 20.0])
    report = evaluate_coverage(raw)
    assert report.passed
    assert report.occupied_octants == 8


def test_single_axis_rotation_coverage_is_explicitly_rejected() -> None:
    rng = np.random.default_rng(7)
    raw = _single_axis_wobble_samples(count=300, rng=rng)

    report = evaluate_coverage(raw)
    assert not report.passed
    # The samples are confined to a thin band around one great circle, so
    # the direction second-moment matrix collapses one eigenvalue toward
    # zero -- that is the signal expected to catch this pattern (see
    # evaluate_coverage's docstring for why octant occupancy alone cannot
    # be trusted here).
    assert report.min_eigen_fraction < DEFAULT_THRESHOLDS.min_eigen_fraction
    assert any("anisotropic" in finding for finding in report.findings)

    with pytest.raises(MagCalFitError, match="insufficient sphere coverage"):
        fit_mag_calibration(raw)


# ---------------------------------------------------------------------------
# Degenerate inputs: each must fail with its own readable message
# ---------------------------------------------------------------------------


def test_rejects_too_few_samples() -> None:
    raw = _fibonacci_sphere(5) * 400.0
    with pytest.raises(MagCalFitError, match="at least"):
        fit_mag_calibration(raw)


def test_rejects_all_identical_points() -> None:
    raw = np.tile(np.array([10.0, 20.0, 30.0]), (200, 1))
    with pytest.raises(MagCalFitError, match="same point"):
        fit_mag_calibration(raw)


def test_rejects_coplanar_samples() -> None:
    rng = np.random.default_rng(11)
    xy = rng.uniform(-300.0, 300.0, size=(300, 2))
    raw = np.column_stack([xy, np.zeros(300)])
    with pytest.raises(MagCalFitError, match="coplanar"):
        fit_mag_calibration(raw)


def test_rejects_collinear_samples() -> None:
    t = np.linspace(-300.0, 300.0, 200)
    raw = np.column_stack([t, t * 0.5, t * -0.25])
    with pytest.raises(MagCalFitError, match="coplanar"):
        fit_mag_calibration(raw)


def test_rejects_nan_input() -> None:
    raw = _fibonacci_sphere(200) * 400.0
    raw[10, 1] = float("nan")
    with pytest.raises(MagCalFitError, match="NaN"):
        fit_mag_calibration(raw)


def test_rejects_wrong_shape_input() -> None:
    with pytest.raises(MagCalFitError, match="3-value"):
        fit_mag_calibration([[1.0, 2.0], [3.0, 4.0]])


def test_rejects_empty_input() -> None:
    with pytest.raises(MagCalFitError):
        fit_mag_calibration([])


# ---------------------------------------------------------------------------
# Determinant / mirroring guard
# ---------------------------------------------------------------------------


def test_fitted_determinant_is_always_positive_across_seeds() -> None:
    for seed in range(5):
        rng = np.random.default_rng(seed)
        bias_true = tuple(rng.uniform(-50.0, 50.0, size=3).tolist())
        directions = _fibonacci_sphere(300)
        raw = _synthesize_samples(
            directions=directions,
            bias_true=bias_true,
            matrix_true=np.eye(3),
            radius_mgauss=450.0,
            noise_sigma_mgauss=2.0,
            rng=rng,
        )
        result = fit_mag_calibration(raw)
        assert result.determinant > 0.0


# ---------------------------------------------------------------------------
# apply_mag_calibration helper
# ---------------------------------------------------------------------------


def test_apply_mag_calibration_matches_contract_formula() -> None:
    raw = [(10.0, 20.0, 30.0), (40.0, 50.0, 60.0)]
    corrected = apply_mag_calibration(raw, (0.0, 0.0, 0.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
    assert corrected == [(10.0, 20.0, 30.0), (40.0, 50.0, 60.0)]

    offset_corrected = apply_mag_calibration(
        raw, (10.0, 10.0, 10.0), ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    )
    assert offset_corrected == [(0.0, 10.0, 20.0), (30.0, 40.0, 50.0)]


def test_apply_mag_calibration_rejects_bad_matrix_shape() -> None:
    with pytest.raises(MagCalFitError, match="3x3"):
        apply_mag_calibration([(1.0, 2.0, 3.0)], (0.0, 0.0, 0.0), ((1.0, 0.0), (0.0, 1.0)))


def test_apply_mag_calibration_rejects_malformed_bias_and_matrix_as_mag_cal_fit_error() -> None:
    # Every rejection from this module must surface as MagCalFitError, not a
    # bare numpy TypeError/ValueError, so a caller that only catches
    # MagCalFitError (per the module's own contract) never crashes instead.
    with pytest.raises(MagCalFitError):
        apply_mag_calibration([(1.0, 2.0, 3.0)], ("not-a-number", 0.0, 0.0), np.eye(3).tolist())
    with pytest.raises(MagCalFitError):
        apply_mag_calibration([(1.0, 2.0, 3.0)], (0.0, 0.0), np.eye(3).tolist())
    with pytest.raises(MagCalFitError):
        apply_mag_calibration(
            [(1.0, 2.0, 3.0)],
            (0.0, 0.0, 0.0),
            ((1.0, 0.0, 0.0), (0.0, 1.0), (0.0, 0.0, 1.0)),
        )
    with pytest.raises(MagCalFitError):
        apply_mag_calibration(
            [(1.0, 2.0, 3.0)],
            (0.0, 0.0, 0.0),
            ((1.0, 0.0, 0.0), (0.0, float("nan"), 0.0), (0.0, 0.0, 1.0)),
        )


# ---------------------------------------------------------------------------
# Threshold validation and NumPy guard
# ---------------------------------------------------------------------------


def test_thresholds_reject_invalid_values() -> None:
    with pytest.raises(ValueError):
        MagCalFitThresholds(min_samples=1)
    with pytest.raises(ValueError):
        MagCalFitThresholds(octant_min_occupied=9)
    with pytest.raises(ValueError):
        MagCalFitThresholds(min_eigen_fraction=0.0)
    with pytest.raises(ValueError):
        MagCalFitThresholds(max_condition_number=1.0)


def test_default_thresholds_are_the_module_default() -> None:
    assert DEFAULT_THRESHOLDS == MagCalFitThresholds()


def test_module_import_survives_without_numpy_but_fit_is_explicit(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(mag_cal_fit, "_np", None)
    with pytest.raises(mag_cal_fit.NumpyRequiredError, match="pip install numpy"):
        fit_mag_calibration([[1.0, 2.0, 3.0]] * 200)


def test_fit_problems_flag_radius_outside_firmware_gate_and_noisy_capture() -> None:
    """Verdict mirrors the firmware gate; a quarter-scale field is unusable."""

    root = Path(__file__).resolve().parents[1]
    header = (root / "Driver" / "Inc" / "drv_mag_calibration.h").read_text(encoding="utf-8")
    assert "DRV_MAG_FIELD_MIN_MGAUSS 200.0f" in header
    assert "DRV_MAG_FIELD_MAX_MGAUSS 800.0f" in header
    assert mag_cal_fit.FIRMWARE_FIELD_MIN_MGAUSS == 200.0
    assert mag_cal_fit.FIRMWARE_FIELD_MAX_MGAUSS == 800.0

    directions = _fibonacci_sphere(300)
    rng = np.random.default_rng(7)

    def fit(radius: float, noise: float):
        return fit_mag_calibration(_synthesize_samples(
            directions=directions, bias_true=(120.0, -60.0, 40.0),
            matrix_true=np.eye(3), radius_mgauss=radius,
            noise_sigma_mgauss=noise, rng=rng))

    assert mag_cal_fit.fit_problems(fit(500.0, 2.0)) == ()
    quarter = mag_cal_fit.fit_problems(fit(125.0, 0.5))
    assert len(quarter) == 1 and "拟合半径" in quarter[0]
    noisy = mag_cal_fit.fit_problems(fit(500.0, 30.0))
    assert len(noisy) == 1 and "RMS" in noisy[0]
