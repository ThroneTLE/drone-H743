"""Host-only magnetometer hard-/soft-iron ellipsoid fit (no UI, no transport).

This module answers one narrow question: given a batch of raw magnetometer
samples (mgauss) captured while the airframe was rotated through many
attitudes, what hard-iron offset ``b`` and soft-iron matrix ``M`` make
``corrected = M @ (raw - b)`` land on a sphere?

It knows nothing about I2C, QMC5883L registers, transport framing, or the
FLU axis mapping from ``drv_frame_contract.h``.  Callers are responsible for
capturing every sample in one consistent coordinate frame (chip-native axes
or already FRD->FLU-rotated axes -- this module's algebra is rotation
covariant, so either works as long as the frame used here matches the frame
``M``/``b`` will later be applied to on the firmware side).  Fitting the
axis mapping itself is a separate, Driver-layer task; this module never
guesses or corrects axis order/sign.

Model (matches the firmware-side contract exactly, see
``.agents/skills/drone-h743-project/references/flu-coordinate-contract.md``
routing and the magnetometer work order C3)::

    corrected = M @ (raw - b)

``b`` is a hard-iron offset in mgauss; ``M`` is a dimensionless 3x3 soft-iron
matrix with ``det(M) > 0`` (a mirrored solution would flip heading and is
rejected outright, never returned).  An uncalibrated firmware record uses
``b = (0, 0, 0)`` and ``M = I`` -- that identity default lives on the
firmware/Services side, not here; this module only ever returns a fit or
raises.

The fit itself is the standard algebraic ellipsoid fit: solve the
homogeneous general quadric ``v^T A v + 2 b_lin^T v + c = 0`` in the
least-squares sense (smallest right singular vector of the design matrix),
complete the square to recover the ellipsoid centre and shape matrix, then
take the symmetric positive-definite square root of the (rescaled) shape
matrix as ``M``.  Using the symmetric square root -- rather than an
arbitrary matrix square root with a rotation folded in -- is a deliberate
choice: it is the only member of the "M with M^T M equal to the required
shape" family that carries no arbitrary orientation, so it is the
unique, reproducible answer, and its eigenvalues are the (necessarily
positive) shape eigenvalues, which also guarantees ``det(M) > 0`` (in fact
``det(M) == 1`` by construction: this fit intentionally puts all volume
scale into the reported ``fitted_radius_mgauss`` rather than into ``M``, so
a well-calibrated sensor gets ``M`` close to the identity instead of a
scaled identity).

No fitted numbers here have been checked against a real magnetometer
capture -- the on-board QMC5883L has never been sampled for this purpose
(see the work order this module was written for). Every numeric claim this
module's own tests make is about synthetic data only; do not read a passing
test as "the fit is flight-quality," only as "the algebra is self-consistent."

NumPy is the only third-party dependency, matching ``tools/imu_metrology.py``.
Importing this module never requires NumPy; only the numerical entry points
(``evaluate_coverage``, ``fit_mag_calibration``) do, and raise
:class:`NumpyRequiredError` with an actionable message if it is missing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

try:  # Keep import-time usable on a minimal host install.
    import numpy as _np
except ImportError:  # pragma: no cover - exercised by monkeypatch in tests.
    _np = None

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]

NUMPY_AVAILABLE = _np is not None

# "All the same point" is a distance check in raw mgauss units; real sensor
# noise/quantization is always orders of magnitude above this.
_COINCIDENT_POINT_EPS_MGAUSS = 1.0e-6
# Below this fraction, the smallest/largest singular value ratio of the
# centred raw cloud means the data is (numerically) coplanar or collinear:
# there is no third independent direction to anchor a 3-D ellipsoid.
_COPLANAR_SINGULAR_RATIO_MIN = 0.02
# Eigenvalues of the recovered ellipsoid shape matrix must clear this floor
# (relative to the fitted radius) to be treated as numerically meaningful;
# below it the "ellipsoid" is indistinguishable from a degenerate quadric.
_MIN_RELATIVE_EIGENVALUE = 1.0e-9


class NumpyRequiredError(RuntimeError):
    """Raised when a numerical fit is requested without NumPy installed."""


class MagCalFitError(ValueError):
    """Raised when raw samples are degenerate, insufficient, or don't cover
    enough of the sphere to trust an ellipsoid fit.

    This is always an explicit rejection, never a best-effort/garbage
    result -- callers must treat this as "do not calibrate," not as
    "retry with a lower bar."
    """


def _require_numpy() -> Any:
    if _np is None:
        raise NumpyRequiredError(
            "magnetometer ellipsoid fit requires NumPy; install it with "
            "'python -m pip install numpy'."
        )
    return _np


def _finite(value: Any, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _strict_int(value: Any, *, name: str, minimum: int = 0) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be an integer")
    if value < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    return value


@dataclass(frozen=True)
class MagCalFitThresholds:
    """Auditable acceptance limits for coverage and fit quality.

    Defaults are deliberately generous on physical soft-iron severity
    (``max_condition_number``) and strict on the things that make a fit
    numerically meaningless (coverage, degeneracy) -- see module docstring
    for why ``M`` close to identity is expected for a clean sensor.
    """

    min_samples: int = 150
    # Sphere-direction coverage: an octant (sign combination of the
    # bias-corrected direction vector) counts as "occupied" only once it
    # holds at least this many samples, so a handful of outlier points
    # cannot fake coverage.
    octant_min_samples: int = 5
    octant_min_occupied: int = 6  # out of 8
    # Smallest eigenvalue of the normalized second-moment matrix of unit
    # direction vectors, as a fraction of the isotropic value (1/3).  1.0
    # means perfectly uniform sphere coverage; values near 0 mean the
    # samples are confined to (close to) a 2-D band, e.g. rotation about a
    # single body axis with the external field's sign along that axis
    # never changing.
    min_eigen_fraction: float = 0.20
    max_condition_number: float = 10.0

    def __post_init__(self) -> None:
        _strict_int(self.min_samples, name="min_samples", minimum=10)
        _strict_int(self.octant_min_samples, name="octant_min_samples", minimum=1)
        occupied = _strict_int(
            self.octant_min_occupied, name="octant_min_occupied", minimum=1
        )
        if occupied > 8:
            raise ValueError("octant_min_occupied must be <= 8")
        fraction = _finite(self.min_eigen_fraction, name="min_eigen_fraction")
        if not 0.0 < fraction <= 1.0:
            raise ValueError("min_eigen_fraction must be in (0, 1]")
        condition = _finite(self.max_condition_number, name="max_condition_number")
        if condition <= 1.0:
            raise ValueError("max_condition_number must be > 1")


DEFAULT_THRESHOLDS = MagCalFitThresholds()


@dataclass(frozen=True)
class CoverageReport:
    """Diagnostic sphere-coverage judgement, independent of the ellipsoid fit."""

    sample_count: int
    occupied_octants: int
    octant_counts: tuple[int, int, int, int, int, int, int, int]
    min_eigen_fraction: float
    passed: bool
    findings: tuple[str, ...]


@dataclass(frozen=True)
class MagCalFitResult:
    """Immutable ellipsoid-fit result; never an apply/persist command."""

    hard_iron_offset_mgauss: Vec3
    soft_iron_matrix: Mat3
    determinant: float
    condition_number: float
    fitted_radius_mgauss: float
    corrected_rms_mgauss: float
    corrected_max_error_mgauss: float
    sample_count: int
    coverage: CoverageReport
    findings: tuple[str, ...]


def _as_vec3(value: Any) -> Vec3:
    return (float(value[0]), float(value[1]), float(value[2]))


def _as_mat3(value: Any) -> Mat3:
    return (_as_vec3(value[0]), _as_vec3(value[1]), _as_vec3(value[2]))


def _validate_raw_samples(raw_mgauss: Sequence[Sequence[float]]) -> Any:
    """Convert input to a float ndarray and reject shape/NaN problems.

    Deliberately does not check sample count or 3-D spread here: callers
    that want a tailored "too few samples" vs. "coplanar" message call
    ``_check_population`` / ``_check_three_dimensional_spread`` themselves
    in a fixed order, so each degenerate-input case gets its own readable
    error instead of one generic linear-algebra exception.
    """

    np = _require_numpy()
    if isinstance(raw_mgauss, (str, bytes, bytearray)):
        raise TypeError("raw_mgauss must be a sequence of 3-value samples, not text")
    try:
        array = np.asarray(raw_mgauss, dtype=float)
    except (TypeError, ValueError) as exc:
        raise MagCalFitError(
            "raw_mgauss must be convertible to an N x 3 array of numbers"
        ) from exc
    if array.ndim != 2 or array.shape[1] != 3 or array.shape[0] == 0:
        raise MagCalFitError(
            "raw_mgauss must be a non-empty sequence of 3-value samples "
            f"(got shape {array.shape})"
        )
    if not np.all(np.isfinite(array)):
        raise MagCalFitError("raw_mgauss contains NaN/Inf; cannot fit an ellipsoid")
    return array


def _check_population(array: Any, thresholds: MagCalFitThresholds) -> None:
    if array.shape[0] < thresholds.min_samples:
        raise MagCalFitError(
            f"only {array.shape[0]} samples, need at least "
            f"{thresholds.min_samples} for a stable ellipsoid fit"
        )


def _check_three_dimensional_spread(array: Any) -> None:
    np = _require_numpy()
    extent = np.ptp(array, axis=0)
    if float(np.max(extent)) < _COINCIDENT_POINT_EPS_MGAUSS:
        raise MagCalFitError(
            "all samples are the same point; cannot determine an ellipsoid "
            "orientation or scale from a single reading"
        )
    centered = array - np.mean(array, axis=0)
    singular_values = np.linalg.svd(centered, compute_uv=False)
    largest = float(singular_values[0])
    smallest = float(singular_values[-1])
    ratio = smallest / largest if largest > 0.0 else 0.0
    if ratio < _COPLANAR_SINGULAR_RATIO_MIN:
        raise MagCalFitError(
            "samples are (nearly) coplanar or collinear "
            f"(smallest/largest spread singular value ratio={ratio:.5f}); "
            "there is no independent third direction to fit a 3-D ellipsoid"
        )


_OCTANT_BIT_X = 4
_OCTANT_BIT_Y = 2
_OCTANT_BIT_Z = 1


def _validate_and_prepare(
    raw_mgauss: Sequence[Sequence[float]], thresholds: MagCalFitThresholds
) -> Any:
    """Shared front door: shape/finite/population/degeneracy checks, once.

    Both :func:`evaluate_coverage` and :func:`fit_mag_calibration` need the
    exact same up-front validation; routing both through here means a large
    capture only pays for the O(n) checks (including the spread SVD) a
    single time per fit, instead of once per function in the call chain.
    """

    array = _validate_raw_samples(raw_mgauss)
    _check_population(array, thresholds)
    _check_three_dimensional_spread(array)
    return array


def evaluate_coverage(
    raw_mgauss: Sequence[Sequence[float]],
    thresholds: MagCalFitThresholds = DEFAULT_THRESHOLDS,
) -> CoverageReport:
    """Judge whether ``raw_mgauss`` explores enough of the sphere to fit.

    Two independent, complementary signals, both required to pass. Both use
    directions relative to the cheap axis-aligned bounding-box centre
    (``(max + min) / 2``) rather than the full ellipsoid fit, because this
    function is meant to run *before* attempting the (more expensive, and
    fit-dependent) ellipsoid solve:

    1. **Octant occupancy** -- bucket each bias-corrected sample by the sign
       of its (x, y, z) direction (8 octants) and require most of them to
       hold a minimum number of samples. This is the simple, inspectable
       judgement, but it is deliberately not trusted alone: a narrow band of
       samples still straddles both signs of every axis relative to *its own*
       bounding-box centre, so a single-axis rotation with any wobble at all
       can still touch all 8 octants.
    2. **Direction isotropy** -- the smallest eigenvalue of the normalized
       second-moment matrix of unit direction vectors, as a fraction of the
       isotropic value (1/3). This is the signal that actually catches a
       single-axis rotation: the sample directions are confined to a thin
       band (a near-2-D subspace of the direction sphere), which collapses
       one eigenvalue toward zero regardless of how the octant boundaries
       happen to fall. Octant occupancy remains useful for the complementary
       failure mode -- e.g. samples clustered inside only two or three
       octants with large but very unevenly-distributed variance.

    Both checks must pass; either one failing is reported in ``findings``.
    """

    array = _validate_and_prepare(raw_mgauss, thresholds)
    return _evaluate_coverage_from_array(array, thresholds)


def _evaluate_coverage_from_array(array: Any, thresholds: MagCalFitThresholds) -> CoverageReport:
    """Coverage judgement on an already-validated array; see :func:`evaluate_coverage`."""

    np = _require_numpy()
    center_estimate = (np.max(array, axis=0) + np.min(array, axis=0)) / 2.0
    delta = array - center_estimate
    norms = np.linalg.norm(delta, axis=1)
    valid = norms > _COINCIDENT_POINT_EPS_MGAUSS
    findings: list[str] = []
    if not np.any(valid):
        # _check_three_dimensional_spread already guards the exact-coincident
        # case; this remains defensive against pathological bounding boxes.
        raise MagCalFitError(
            "no sample is distinguishable from the bounding-box centre; "
            "cannot evaluate direction coverage"
        )
    directions = delta[valid] / norms[valid, None]

    octant_counts = [0] * 8
    for direction in directions:
        index = (
            (_OCTANT_BIT_X if direction[0] >= 0.0 else 0)
            | (_OCTANT_BIT_Y if direction[1] >= 0.0 else 0)
            | (_OCTANT_BIT_Z if direction[2] >= 0.0 else 0)
        )
        octant_counts[index] += 1
    occupied_octants = sum(
        1 for count in octant_counts if count >= thresholds.octant_min_samples
    )

    moment = (directions.T @ directions) / directions.shape[0]
    eigenvalues = np.linalg.eigvalsh(moment)
    min_eigen_fraction = float(np.min(eigenvalues)) / (1.0 / 3.0)

    passed = True
    if occupied_octants < thresholds.octant_min_occupied:
        passed = False
        findings.append(
            f"only {occupied_octants}/8 direction octants have >= "
            f"{thresholds.octant_min_samples} samples (need >= "
            f"{thresholds.octant_min_occupied}); rotate through more distinct "
            "attitudes, not just one axis"
        )
    if min_eigen_fraction < thresholds.min_eigen_fraction:
        passed = False
        findings.append(
            f"direction distribution is anisotropic (min_eigen_fraction="
            f"{min_eigen_fraction:.4f}, need >= {thresholds.min_eigen_fraction:.2f}); "
            "samples cluster near a band instead of covering the sphere"
        )
    if not findings:
        findings.append("coverage sufficient for an ellipsoid fit")

    return CoverageReport(
        sample_count=int(array.shape[0]),
        occupied_octants=occupied_octants,
        octant_counts=tuple(octant_counts),  # type: ignore[arg-type]
        min_eigen_fraction=min_eigen_fraction,
        passed=passed,
        findings=tuple(findings),
    )


def _solve_quadric(scaled: Any) -> tuple[Any, Any, float]:
    """Least-squares fit of ``v^T A v + 2 b^T v + c = 0`` to ``scaled`` points.

    Returns ``(A, b, c)`` oriented so that ``A`` is meant to be positive
    definite (sign of the whole homogeneous solution is arbitrary; the
    caller resolves it once the ellipsoid centre/radius are known).
    """

    np = _require_numpy()
    x, y, z = scaled[:, 0], scaled[:, 1], scaled[:, 2]
    design = np.column_stack(
        [
            x * x,
            y * y,
            z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            2.0 * x,
            2.0 * y,
            2.0 * z,
            np.ones_like(x),
        ]
    )
    _, _, vt = np.linalg.svd(design, full_matrices=False)
    p = vt[-1]
    a11, a22, a33, a12, a13, a23, bx, by, bz, c = p
    matrix_a = np.array(
        [[a11, a12, a13], [a12, a22, a23], [a13, a23, a33]], dtype=float
    )
    b_lin = np.array([bx, by, bz], dtype=float)
    return matrix_a, b_lin, float(c)


def fit_mag_calibration(
    raw_mgauss: Sequence[Sequence[float]],
    thresholds: MagCalFitThresholds = DEFAULT_THRESHOLDS,
) -> MagCalFitResult:
    """Fit hard-iron offset ``b`` and soft-iron matrix ``M`` from raw samples.

    Raises :class:`MagCalFitError` for anything that would make the fit
    meaningless: too few samples, degenerate geometry (coincident/coplanar
    points), insufficient sphere coverage, a singular or non-positive-definite
    quadric, or (defensively) a non-positive determinant.  There is no
    best-effort fallback -- every rejection means "do not calibrate," never
    "use this anyway."
    """

    np = _require_numpy()
    array = _validate_and_prepare(raw_mgauss, thresholds)
    coverage = _evaluate_coverage_from_array(array, thresholds)
    if not coverage.passed:
        raise MagCalFitError(
            "insufficient sphere coverage, refusing to fit: "
            + "; ".join(coverage.findings)
        )

    # Fit in a rescaled, ~O(1)-magnitude coordinate to keep the design
    # matrix's x^2 / xy / x / 1 columns comparably conditioned; mgauss
    # magnitudes (hundreds) would otherwise dominate the constant column by
    # several orders of magnitude and destabilise the SVD.
    centroid = np.mean(array, axis=0)
    scale = float(np.mean(np.linalg.norm(array - centroid, axis=1)))
    if scale <= _COINCIDENT_POINT_EPS_MGAUSS:
        raise MagCalFitError("samples have no meaningful spread to scale by")
    scaled = array / scale

    matrix_a, b_lin, c = _solve_quadric(scaled)
    try:
        center = np.linalg.solve(matrix_a, -b_lin)
    except np.linalg.LinAlgError as exc:
        raise MagCalFitError(
            "quadric coefficient matrix is singular; samples may be "
            "coplanar/collinear or otherwise fail to pin down an ellipsoid"
        ) from exc

    # Complete the square: (v - center)^T A (v - center) = k. The overall
    # sign of the homogeneous solution is arbitrary (p and -p describe the
    # same surface), so flip to the branch where k > 0.
    k = -(float(center @ matrix_a @ center) + 2.0 * float(b_lin @ center) + c)
    if k < 0.0:
        matrix_a = -matrix_a
        k = -k
    if k <= _MIN_RELATIVE_EIGENVALUE:
        raise MagCalFitError(
            "fit degenerated to a non-ellipsoid quadric (zero or negative "
            "radial extent); samples likely lack 3-D coverage"
        )

    normalized = matrix_a / k
    eigenvalues, eigenvectors = np.linalg.eigh(normalized)
    if float(np.min(eigenvalues)) <= _MIN_RELATIVE_EIGENVALUE:
        raise MagCalFitError(
            "fitted quadric is not positive definite (not a closed ellipsoid); "
            "samples likely lack 3-D coverage or contain gross outliers"
        )

    # Radius convention: pick the scale that makes the *shape-normalizing*
    # matrix volume-preserving (det == 1), so a sensor whose raw samples
    # already lie on a sphere fits M close to the identity instead of a
    # scaled identity -- see module docstring.
    radius_scaled = float(np.prod(eigenvalues)) ** (-1.0 / 6.0)
    sqrt_eigenvalues = np.sqrt(eigenvalues)
    m_scaled = radius_scaled * (eigenvectors @ np.diag(sqrt_eigenvalues) @ eigenvectors.T)

    bias = center * scale
    matrix_m = m_scaled  # dimensionless: the 1/scale and *scale cancel, see docstring derivation
    fitted_radius = radius_scaled * scale

    determinant = float(np.linalg.det(matrix_m))
    if determinant <= 0.0:
        raise MagCalFitError(
            f"fitted soft-iron matrix has non-positive determinant ({determinant:.6g}); "
            "refusing a mirrored solution"
        )
    condition_number = float(np.max(sqrt_eigenvalues) / np.min(sqrt_eigenvalues))
    if condition_number > thresholds.max_condition_number:
        raise MagCalFitError(
            f"fitted ellipsoid is too eccentric (condition={condition_number:.3f}, "
            f"limit={thresholds.max_condition_number:.1f}); refusing an unreliable fit"
        )

    corrected = (matrix_m @ (array - bias).T).T
    corrected_norms = np.linalg.norm(corrected, axis=1)
    residual = corrected_norms - fitted_radius
    rms = float(math.sqrt(float(np.mean(residual * residual))))
    max_error = float(np.max(np.abs(residual)))

    findings: list[str] = [
        f"fitted_radius={fitted_radius:.2f} mgauss, corrected_rms={rms:.3f} mgauss, "
        f"corrected_max_error={max_error:.3f} mgauss",
    ]

    return MagCalFitResult(
        hard_iron_offset_mgauss=_as_vec3(bias),
        soft_iron_matrix=_as_mat3(matrix_m),
        determinant=determinant,
        condition_number=condition_number,
        fitted_radius_mgauss=fitted_radius,
        corrected_rms_mgauss=rms,
        corrected_max_error_mgauss=max_error,
        sample_count=int(array.shape[0]),
        coverage=coverage,
        findings=tuple(findings),
    )


def apply_mag_calibration(
    raw_mgauss: Sequence[Sequence[float]],
    hard_iron_offset_mgauss: Vec3,
    soft_iron_matrix: Mat3,
) -> list[Vec3]:
    """Apply ``corrected = M @ (raw - b)``; a pure preview/test helper.

    This mirrors the firmware-side formula exactly (see module docstring)
    so host-side previews and tests can check a candidate without depending
    on a target. It performs no fitting and no validation beyond shape.
    """

    np = _require_numpy()
    array = _validate_raw_samples(raw_mgauss)
    # Every rejection from this module is a MagCalFitError (see module and
    # class docstrings) -- so malformed bias/matrix arguments are translated
    # here too, not left to leak a bare numpy TypeError/ValueError.
    try:
        bias = np.asarray(hard_iron_offset_mgauss, dtype=float)
    except (TypeError, ValueError) as exc:
        raise MagCalFitError(
            "hard_iron_offset_mgauss must be convertible to three numbers"
        ) from exc
    if bias.shape != (3,) or not np.all(np.isfinite(bias)):
        raise MagCalFitError("hard_iron_offset_mgauss must contain exactly three finite values")
    try:
        matrix = np.asarray(soft_iron_matrix, dtype=float)
    except (TypeError, ValueError) as exc:
        raise MagCalFitError(
            "soft_iron_matrix must be convertible to a 3x3 array of numbers"
        ) from exc
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise MagCalFitError("soft_iron_matrix must be a finite 3x3 matrix")
    corrected = (matrix @ (array - bias).T).T
    return [_as_vec3(row) for row in corrected]


# The firmware fusion gate (Driver/Inc/drv_mag_calibration.h,
# DRV_MAG_FIELD_MIN/MAX_MGAUSS): a corrected field outside it is rejected on
# every tick, so a fit whose radius lands outside can never be used.
FIRMWARE_FIELD_MIN_MGAUSS = 200.0
FIRMWARE_FIELD_MAX_MGAUSS = 800.0
# ArduPilot's default COMPASS_CAL_FIT: residual RMS above this means the
# capture was disturbed (nearby steel, moving cables) and should be redone.
MAX_USABLE_RMS_MGAUSS = 16.0


def fit_problems(result: MagCalFitResult) -> tuple[str, ...]:
    """Reasons a successful fit should still not be used; empty means usable."""

    problems: list[str] = []
    radius = result.fitted_radius_mgauss
    if not FIRMWARE_FIELD_MIN_MGAUSS <= radius <= FIRMWARE_FIELD_MAX_MGAUSS:
        problems.append(
            f"拟合半径 {radius:.0f} mG 不在飞控接受的 "
            f"{FIRMWARE_FIELD_MIN_MGAUSS:.0f}–{FIRMWARE_FIELD_MAX_MGAUSS:.0f} mG 内"
            "（地磁约 450–600 mG），融合会一直拒收；多半是固件换算不对")
    if result.corrected_rms_mgauss > MAX_USABLE_RMS_MGAUSS:
        problems.append(
            f"校正后 RMS {result.corrected_rms_mgauss:.1f} mG 超过 "
            f"{MAX_USABLE_RMS_MGAUSS:.0f} mG，采样时有干扰，远离铁件和电脑后重采")
    return tuple(problems)


__all__ = [
    "FIRMWARE_FIELD_MAX_MGAUSS",
    "FIRMWARE_FIELD_MIN_MGAUSS",
    "MAX_USABLE_RMS_MGAUSS",
    "fit_problems",
    "DEFAULT_THRESHOLDS",
    "CoverageReport",
    "MagCalFitError",
    "MagCalFitResult",
    "MagCalFitThresholds",
    "Mat3",
    "NumpyRequiredError",
    "NUMPY_AVAILABLE",
    "Vec3",
    "apply_mag_calibration",
    "evaluate_coverage",
    "fit_mag_calibration",
]
