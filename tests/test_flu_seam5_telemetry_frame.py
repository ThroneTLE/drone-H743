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


def test_replay_converts_to_flu_only_on_recorded_provenance() -> None:
    """R-F5b made the FLU conversion conditional; it must never be blanket.

    Historical logs predating the V0 candidate were recorded with FRD attitude
    from the Fusion NED path and carry no provenance.  Converting every file
    would corrupt exactly the evidence spec section 10 protects, so the
    conversion has to sit behind the recorded frame identity and the
    no-provenance branch has to land on legacy.
    """
    source = read(REPLAY)
    # The conversion exists, but only inside the canonical_flu branch.
    assert "def flu_to_local_down_angles(" in source
    assert 'if attitude_frame == ATTITUDE_FRAME_FLU:' in source
    # Absent or inconsistent provenance must fall back to legacy, not to FLU.
    assert 'return ATTITUDE_FRAME_LEGACY, "未标注坐标系（无溯源列）' in source
    assert "ATTITUDE_FRAME_LEGACY = \"legacy_frd\"" in source


def test_log_record_now_carries_frame_provenance() -> None:
    """The seam 5 sentinel fired as designed and was re-adjudicated by R-F5b.

    Its predecessor asserted the log carried *no* frame provenance, and said
    that failing would be good news.  R-F5b added the provenance, so the
    contract is restated here: the snapshot must carry the frame identity that
    lets a reader pick a convention per file.  Placement, version migration and
    the v7 read-back guarantee are locked by
    tests/test_flight_log_frame_provenance.py.
    """
    header = read(LOG_HEADER)
    assert "uint8_t frame_orientation_code;" in header
    assert "uint8_t frame_contract_version;" in header
    assert "uint32_t calibration_generation;" in header


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


def test_replay_declares_its_frame_contract() -> None:
    source = read(REPLAY)
    head = source[: source.index("from __future__")]
    assert "X前/Y右/Z下" in head
    assert "FLU" in head
    assert "历史数据永不重释义" in head
    # R-F5b: the declaration must now also state how the convention is chosen
    # and what happens when a file carries no provenance.
    assert "resolve_attitude_frame" in head
    assert "没有溯源就一律按 legacy FRD 渲染" in head
