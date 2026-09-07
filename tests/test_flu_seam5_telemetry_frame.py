"""R-F5 seam 5 telemetry/log frame contract."""

# Author ruling 2026-09-03: live status telemetry is an external observation
# boundary and must publish canonical body FLU even while the controller keeps
# its legacy X-forward/Y-right representation.  Schema v3 carries the frame and
# contract version, and app_telem_port performs the explicit adapter.
#
# The remaining seam-5 residue is still real and this module pins it rather
# than hiding it:
#
#   * app_stabilizer.c logs ctx->roll_control / pitch_control / yaw_control,
#     which are canonical FLU Euler angles after seams 0/1.
#   * tools/flight_log_rerun_replay.py builds its rotation with
#     rpy_body_to_local_down(), which assumes X-forward / Y-right / Z-down.
#     FLU and FRD disagree on pitch and yaw sign for the same number, so the
#     replay renders those two channels under the older convention.
#   * R-F5b added per-file provenance, but historical logs legitimately have
#     none and must remain legacy FRD.
#   * The author explicitly set the global mask before props-off validation on
#     2026-09-06.  That override is not evidence that seam 5 or M6 passed.

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPLAY = ROOT / "tools" / "flight_log_rerun_replay.py"
CONTRACT = ROOT / "Driver" / "Inc" / "drv_frame_contract.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
LOG_HEADER = ROOT / "App" / "Inc" / "app_flight_log.h"
TELEMETRY_PORT = ROOT / "App" / "Src" / "app_telem_port.c"
FRAME_REFERENCE = (ROOT / ".agents" / "skills" / "drone-h743-project" /
                   "references" / "flu-coordinate-contract.md")
PIPELINE = ROOT / "PIPELINE.md"


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


def test_live_nav_telemetry_adapts_controller_legacy_xy_to_body_flu() -> None:
    """状态监视的 X/Y 必须遵守 X前、Y左，不能泄漏控制器的 Y右口径。"""
    source = read(TELEMETRY_PORT)
    assert '#include "drv_frame_contract.h"' in source
    assert source.count("DRV_FRAME_FrdToFlu(") >= 2
    assert "vofa_data[APP_TELEM_CH_VEL_EST_X] = velocity_flu.x;" in source
    assert "vofa_data[APP_TELEM_CH_VEL_EST_Y] = velocity_flu.y;" in source
    assert "vofa_data[APP_TELEM_CH_POS_EST_X] = position_flu.x;" in source
    assert "vofa_data[APP_TELEM_CH_POS_EST_Y] = position_flu.y;" in source
    assert "APP_TELEM_CH_VEL_EST_Y] = vofa_debug.vel_est_m_s[1]" not in source
    assert "APP_TELEM_CH_POS_EST_Y] = vofa_debug.pos_est_m[1]" not in source


def test_telemetry_mask_override_does_not_claim_physical_acceptance() -> None:
    """The author may arm for validation without fabricating seam-5 evidence."""
    contract = read(CONTRACT)
    done_match = re.search(
        r"^#define\s+DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK\s+(0x[0-9A-Fa-f]+|\d+)U\s*$",
        contract,
        re.MULTILINE,
    )
    assert done_match is not None
    required_match = re.search(
        r"^#define\s+DRV_FRAME_RUNTIME_MIGRATION_REQUIRED_MASK\s+(0x[0-9A-Fa-f]+|\d+)U\s*$",
        contract,
        re.MULTILINE,
    )
    assert required_match is not None
    done_mask = int(done_match.group(1), 0)
    required_mask = int(required_match.group(1), 0)
    assert (done_mask & (1 << 5)) != 0
    assert done_mask == required_mask
    reference = " ".join(read(FRAME_REFERENCE).replace("**", "").split())
    pipeline = read(PIPELINE)
    assert "Author-directed props-off validation override" in reference
    assert "not flight release" in reference
    assert "先置 `DONE_MASK=0x3FU` 再手动验证" in pipeline
    assert "不等于物理验收或解冻 M7" in pipeline


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
