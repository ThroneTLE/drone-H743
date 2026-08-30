"""R-F5 seam 5 telemetry/log frame contract."""

# Author ruling 2026-08-30: seam 5 is the same shape as seam 3/4 -- pin the
# contract and name the frame, do not migrate the representation.
#
# The residue spec section 10 names is real and this module pins it rather than
# hiding it:
#
#   * app_stabilizer.c logs ctx->roll_control / pitch_control / yaw_control,
#     which are canonical FLU Euler angles after seams 0/1.
#   * tools/flight_log_rerun_replay.py builds its rotation with
#     rpy_body_to_local_down(), which assumes X-forward / Y-right / Z-down.
#     FLU and FRD disagree on pitch and yaw sign for the same number, so the
#     replay renders those two channels under the older convention.
#   * The log record carries no frame identifier, version or firmware CRC, so
#     a reader cannot tell which convention a given file was recorded under.
#
# Fixing this properly means adding frame provenance to the log format, which
# is an observable format change and needs its own author-approved item.  What
# seam 5 may do now is make the assumption explicit and freeze the geometry, so
# that neither the replay silently switches convention nor historical FRD logs
# get reinterpreted as FLU (spec section 10: 历史数据永不重释义).

from __future__ import annotations

from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
REPLAY = ROOT / "tools" / "flight_log_rerun_replay.py"
CONTRACT = ROOT / "Driver" / "Inc" / "drv_frame_contract.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
LOG_HEADER = ROOT / "App" / "Inc" / "app_flight_log.h"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_replay_geometry_is_frozen() -> None:
    """Rendering must stay bit-identical; this seam changes no geometry."""
    source = read(REPLAY)
    # Yaw-pitch-roll rotation, third row, unchanged.
    assert "[-sp, cp * sr, cp * cr]," in source
    # Local(X fwd, Y right, Z down) -> Rerun(X right, Y fwd, Z up), unchanged.
    assert "x_forward, y_right, z_down = list(point)" in source
    assert "return [float(y_right), float(x_forward), float(-z_down)]" in source


def test_replay_does_not_reinterpret_history_as_flu() -> None:
    """No unconditional FLU conversion may appear in the replay path.

    Historical logs predating the V0 candidate were recorded with FRD attitude
    from the Fusion NED path.  Converting every file would corrupt exactly the
    evidence spec section 10 protects.
    """
    source = read(REPLAY)
    assert "y_left" not in source
    assert "FluToFrd" not in source
    assert "DRV_FRAME_FrdToFlu" not in source


def test_log_record_still_carries_no_frame_provenance() -> None:
    """Pin the gap, so adding provenance forces a revisit of this seam.

    If this ever fails it is good news: it means the log format gained a frame
    identifier, and the replay may finally choose its convention per file
    instead of assuming one.
    """
    header = read(LOG_HEADER)
    for field in ("frame", "orientation", "contract_version", "firmware_crc32"):
        assert field not in header


def test_logged_attitude_source_is_pinned() -> None:
    """The replay's mismatch is only diagnosable if the source stays pinned."""
    source = read(STABILIZER)
    assert "flog_snapshot.roll_deg = ctx->roll_control;" in source
    assert "flog_snapshot.pitch_deg = ctx->pitch_control;" in source
    assert "flog_snapshot.yaw_deg = ctx->yaw_control;" in source


def test_telemetry_seam_cannot_be_declared_done() -> None:
    """Seam 5 stays unmigrated until the log format carries its frame."""
    contract = read(CONTRACT)
    assert "#define DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK               0U" in contract


@pytest.mark.xfail(
    strict=True,
    reason="R-F5 red test: flight_log_rerun_replay.py states its frame only in "
    "two inline comments and never declares that the firmware now logs FLU, "
    "that logs carry no frame identifier, or that history must not be "
    "reinterpreted; cleared by the seam 5 implementation commit",
)
def test_replay_declares_its_frame_contract() -> None:
    source = read(REPLAY)
    head = source[: source.index("from __future__")]
    assert "X前/Y右/Z下" in head
    assert "FLU" in head
    assert "历史数据永不重释义" in head
    assert "帧内未记录坐标系" in head
