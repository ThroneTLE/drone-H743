"""`tools/panel_lib/mag_protocol.py`: pure decode of MAGCAL/MAGFRAME/`MAG?` lines.

Two fixture-sourcing strategies, matched to what D5-3 (decoupling-spec) requires
("测试夹具必须锚定固件真实发出的格式") and to what's practical for each family:

* MAGCAL/MAGFRAME: compiled and executed from the real
  `App/Src/app_cmd_magcal.c` + `Driver/Src/drv_mag_calibration.c`, the same
  "stub the boundary, run the real module" technique already proven in
  `tests/test_magcal_command_contract.py` (this file does not re-test firmware
  *behaviour* -- that is that file's job -- only that our host-side decoder
  reads its real output correctly).
* `MAG ok=...` (`App/Src/app_mag.c::APP_MAG_Report`): compiling that unit pulls
  in the BSP/HAL magnetometer stack (`drv_mag.h` drags in `I2C_HandleTypeDef`),
  which is disproportionate machinery for one report line. Anchored instead the
  way `tools/panel_qa/fixtures.py` anchors its GPS/TELEM fixtures: the key
  *names* are extracted straight from the source format string and asserted
  equal before a hand-built line using those same keys is fed to the decoder.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import pytest

from tools.panel_lib import mag_protocol

ROOT = Path(__file__).resolve().parents[1]

# ------------------------------------------------------------------ MAGCAL/MAGFRAME 真实固件转录

FAKE_MAIN_H = r"""
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
#endif
"""

FAKE_CMSIS_OS2_H = r"""
#ifndef FAKE_CMSIS_OS2_H
#define FAKE_CMSIS_OS2_H
typedef void *osSemaphoreId_t;
typedef void *osMessageQueueId_t;
typedef void *osMutexId_t;
typedef void *osThreadId_t;
#endif
"""

# Same stub boundary as tests/test_magcal_command_contract.py (proven to link
# and run against the real app_cmd_magcal.c); the only behavioural difference
# is the fusion-status stub returns a fixed non-zero pattern instead of an
# all-zero struct, so the decoder is exercised against 1/true values too, and
# `run()` never resets the log -- the whole session transcript is dumped once
# at the end, delimited by an "@@@ <command>" marker line per command.
HARNESS = r"""
#include "app_magcal.h"
#include "app_magxy.h"
#include "app_control_internal.h"
#include "app_stabilizer.h"
#include "drv_frame_contract.h"
#include "drv_mag_calibration.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char log_buf[16384];
static size_t log_len;

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;
    int written;

    va_start(args, format);
    written = vsnprintf(log_buf + log_len, sizeof(log_buf) - log_len, format, args);
    va_end(args);
    if (written > 0) {
        log_len += (size_t)written;
        if (log_len >= sizeof(log_buf)) { log_len = sizeof(log_buf) - 1U; }
    }
}

uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    char *end;
    unsigned long parsed;
    if ((text == NULL) || (text[0] == '\0')) { return 0U; }
    parsed = strtoul(text, &end, 10);
    if (*end != '\0') { return 0U; }
    *value = (uint32_t)parsed;
    return 1U;
}

uint8_t app_control_parse_f32(const char *text, float *value)
{
    char *end;
    float parsed;
    if ((text == NULL) || (text[0] == '\0')) { return 0U; }
    parsed = strtof(text, &end);
    if ((*end != '\0') || !isfinite(parsed)) { return 0U; }
    *value = parsed;
    return 1U;
}

static uint8_t stub_armed;
uint8_t APP_Stabilizer_IsArmed(void) { return stub_armed; }

static uint8_t stub_commit_status;
uint8_t app_control_internal_commit_config_persist(void) { return stub_commit_status; }

static int critical_depth;
uint32_t BSP_Critical_Enter(void) { return (uint32_t)critical_depth++; }
void BSP_Critical_Exit(uint32_t state) { critical_depth = (int)state; }

void APP_Stabilizer_GetMagFusionStatus(APP_Stabilizer_MagFusionStatus *out)
{
    if (out != NULL) {
        out->subsystem_enabled = 1U;
        out->field_rejected = 0U;
        out->used = 1U;
        out->ignored = 0U;
        out->recovery = 1U;
        out->error_deg = 3.75f;
    }
}

uint8_t APP_MagXY_HandleCommand(char **tokens, uint32_t count)
{
    (void)tokens; (void)count;
    return 0U;
}

static uint8_t run(const char *line)
{
    static char buffer[128];
    char *tokens[10];
    uint32_t count = 0U;
    char *cursor;
    size_t n = strlen(log_buf);

    n += (size_t)snprintf(log_buf + n, sizeof(log_buf) - n, "@@@ %s\n", line);
    log_len = n;

    strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    cursor = buffer;
    while ((*cursor != '\0') && (count < 10U)) {
        while (*cursor == ' ') { *cursor++ = '\0'; }
        if (*cursor == '\0') { break; }
        tokens[count++] = cursor;
        while ((*cursor != '\0') && (*cursor != ' ')) { cursor++; }
    }
    return app_control_handle_magcal(tokens, count);
}

int main(void)
{
    run("MAGCAL?");                                            /* #0 */
    run("MAGCAL SET BIAS 1.500000 -2.250000 0.125000");
    run("MAGCAL SET MATRIX 0 1.100000 0.010000 -0.020000");
    run("MAGCAL SET MATRIX 1 0.020000 0.950000 0.030000");
    run("MAGCAL SET MATRIX 2 -0.010000 0.040000 1.050000");
    run("MAGCAL APPLY");                                       /* #0: succeeds */
    run("MAGCAL?");                                            /* #1: post-apply */
    run("MAGFRAME VERIFY CONFIRM");                            /* #0: succeeds */
    run("MAGFRAME?");                                          /* #0 */
    stub_commit_status = 0U;
    run("MAGCAL COMMIT");
    run("MAGCAL?");                                            /* #2: post-commit */
    run("MAGCAL SET BIAS 1 2");           /* bad usage: wrong arg count */
    run("MAGCAL BOGUS");                  /* unknown subcommand */
    run("MAGFRAME VERIFY");               /* missing CONFIRM */
    run("MAGCAL CLEAR");
    run("MAGCAL APPLY");                                       /* #1: incomplete draft after CLEAR */
    stub_armed = 1U;
    run("MAGCAL SET BIAS 5 5 5");         /* armed blocks writes */
    run("MAGFRAME VERIFY CONFIRM");                            /* #1: armed_blocked */
    stub_armed = 0U;

    fputs(log_buf, stdout);
    return 0;
}
"""


@pytest.fixture(scope="module")
def transcript() -> list[tuple[str, list[str]]]:
    """`[(command, [reply lines]), ...]` in session order, from real compiled firmware.

    A list of `(command, lines)` pairs rather than a dict, because several
    commands (`MAGCAL?`, `MAGCAL APPLY`, `MAGFRAME VERIFY CONFIRM`) are sent
    more than once in the session with different outcomes each time -- a dict
    keyed by command text would silently merge repeats. Use `block()` below to
    pick a specific occurrence.
    """
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    with tempfile.TemporaryDirectory() as raw_tmp:
        tmp_path = Path(raw_tmp)
        fakes = tmp_path / "fakes"
        fakes.mkdir()
        (fakes / "main.h").write_text(FAKE_MAIN_H, encoding="ascii")
        (fakes / "cmsis_os2.h").write_text(FAKE_CMSIS_OS2_H, encoding="ascii")
        harness = tmp_path / "mag_protocol_harness.c"
        executable = tmp_path / "mag_protocol_harness.exe"
        harness.write_text(HARNESS, encoding="ascii")
        result = subprocess.run(
            [
                compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
                f"-I{fakes}",
                f"-I{ROOT / 'App' / 'Inc'}",
                f"-I{ROOT / 'Driver' / 'Inc'}",
                f"-I{ROOT / 'BSP' / 'Inc'}",
                f"-I{ROOT / 'Services' / 'Inc'}",
                str(ROOT / "App" / "Src" / "app_cmd_magcal.c"),
                str(ROOT / "Driver" / "Src" / "drv_mag_calibration.c"),
                str(harness), "-lm", "-o", str(executable),
            ],
            capture_output=True, text=True,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        run_result = subprocess.run([str(executable)], capture_output=True, text=True)
        assert run_result.returncode == 0, run_result.stdout + run_result.stderr
        output = run_result.stdout

    session: list[tuple[str, list[str]]] = []
    current_lines: list[str] | None = None
    for raw_line in output.replace("\r\n", "\n").split("\n"):
        if raw_line.startswith("@@@ "):
            current_lines = []
            session.append((raw_line[len("@@@ "):], current_lines))
            continue
        if raw_line and current_lines is not None:
            current_lines.append(raw_line)
    return session


def block(transcript: list[tuple[str, list[str]]], command: str, occurrence: int = 0) -> list[str]:
    """The reply lines for the `occurrence`-th (0-indexed) time `command` was sent."""
    matches = [lines for cmd, lines in transcript if cmd == command]
    assert len(matches) > occurrence, (command, occurrence, [c for c, _ in transcript])
    return matches[occurrence]


def only(lines: list[str], predicate) -> str:
    matches = [line for line in lines if predicate(line)]
    assert len(matches) == 1, (matches, lines)
    return matches[0]


# ------------------------------------------------------------------ MAGCAL 状态/零偏/矩阵/草稿/融合


def test_initial_status_is_the_factory_default(transcript):
    lines = block(transcript, "MAGCAL?", 0)
    status = mag_protocol.parse_magcal_status(only(lines, lambda l: l.startswith("MAGCAL calibrated=")))
    assert status.calibrated is False
    assert status.axis_verified == mag_protocol.AXIS_UNVERIFIED
    assert status.axis_effective == mag_protocol.AXIS_UNVERIFIED
    assert status.dirty is False
    # Factory-default contract_stored is 0 (never verified); it need not equal
    # whatever DRV_FRAME_CONTRACT_VERSION currently is, and axis_verified is
    # already 0 in this state regardless -- contract_mismatch is informational
    # here, not a second independent gate.
    assert status.contract_stored == 0

    matrix = mag_protocol.parse_magcal_matrix(only(lines, lambda l: l.startswith("MAGCAL matrix ")))
    assert matrix.rows == ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    draft = mag_protocol.parse_magcal_draft(only(lines, lambda l: l.startswith("MAGCAL draft ")))
    assert draft == mag_protocol.MagCalDraft(False, (False, False, False))
    assert draft.complete is False


def test_set_bias_and_matrix_acks_are_decoded(transcript):
    bias_ack_line = block(transcript, "MAGCAL SET BIAS 1.500000 -2.250000 0.125000", 0)[0]
    assert mag_protocol.parse_magcal_event(bias_ack_line) == mag_protocol.MagCalEvent(state="draft_bias_set")

    row0_ack_line = block(transcript, "MAGCAL SET MATRIX 0 1.100000 0.010000 -0.020000", 0)[0]
    row0_ack = mag_protocol.parse_magcal_event(row0_ack_line)
    assert row0_ack == mag_protocol.MagCalEvent(state="draft_matrix_row", row=0)


def test_applied_status_reports_the_fitted_bias_and_matrix(transcript):
    apply_ack = mag_protocol.parse_magcal_event(block(transcript, "MAGCAL APPLY", 0)[0])
    assert apply_ack.state == "applied_ram"

    lines = block(transcript, "MAGCAL?", 1)  # post-APPLY
    status = mag_protocol.parse_magcal_status(only(lines, lambda l: l.startswith("MAGCAL calibrated=")))
    assert status.calibrated is True
    assert status.dirty is True

    bias = mag_protocol.parse_magcal_bias(only(lines, lambda l: l.startswith("MAGCAL bias_mgauss=")))
    bx, by, bz = bias.bias_mgauss
    assert bx == pytest.approx(1.5, abs=1e-3)
    assert by == pytest.approx(-2.25, abs=1e-3)
    assert bz == pytest.approx(0.125, abs=1e-3)

    matrix = mag_protocol.parse_magcal_matrix(only(lines, lambda l: l.startswith("MAGCAL matrix ")))
    assert matrix.rows[0] == pytest.approx((1.1, 0.01, -0.02), abs=1e-4)
    assert matrix.rows[2] == pytest.approx((-0.01, 0.04, 1.05), abs=1e-4)

    draft = mag_protocol.parse_magcal_draft(only(lines, lambda l: l.startswith("MAGCAL draft ")))
    assert draft.complete is False  # APPLY clears the draft


def test_fusion_status_line_reports_the_stubbed_pattern(transcript):
    lines = block(transcript, "MAGCAL?", 0)
    fusion = mag_protocol.parse_magcal_fusion(only(lines, lambda l: l.startswith("MAGCAL fusion ")))
    assert fusion.subsystem_enabled is True
    assert fusion.used is True
    assert fusion.participating is True
    assert fusion.field_rejected is False
    assert fusion.recovery is True
    assert fusion.error_deg == pytest.approx(3.75, abs=1e-2)


def test_magframe_verify_confirm_flips_axis_verified(transcript):
    ack = mag_protocol.parse_magframe_event(block(transcript, "MAGFRAME VERIFY CONFIRM", 0)[0])
    assert ack.state == "verified"

    lines = block(transcript, "MAGFRAME?", 0)
    frame = mag_protocol.parse_magframe_status(only(lines, lambda l: l.startswith("MAGFRAME axis_verified=")))
    assert frame.axis_verified == mag_protocol.AXIS_VERIFIED
    assert frame.axis_effective_verified is True
    assert frame.derivation == mag_protocol.MAGFRAME_DERIVATION_DEFAULT_UNVERIFIED


def test_commit_then_status_shows_dirty_cleared(transcript):
    ack = mag_protocol.parse_magcal_event(block(transcript, "MAGCAL COMMIT", 0)[0])
    assert ack.state == "committed"
    lines = block(transcript, "MAGCAL?", 2)  # post-commit
    status = mag_protocol.parse_magcal_status(only(lines, lambda l: l.startswith("MAGCAL calibrated=")))
    assert status.dirty is False


def test_bad_usage_and_unknown_subcommand_are_not_decoded_as_data(transcript):
    # These are "ERR ..." lines -- out of mag_protocol's dispatch table by
    # design (the page treats them as opaque usage-error text, see mag_cal.py).
    bad_bias = block(transcript, "MAGCAL SET BIAS 1 2", 0)[0]
    bogus = block(transcript, "MAGCAL BOGUS", 0)[0]
    missing_confirm = block(transcript, "MAGFRAME VERIFY", 0)[0]
    assert bad_bias.startswith("ERR usage MAGCAL SET BIAS")
    assert bogus.startswith("ERR usage MAGCAL SET|APPLY|COMMIT|CLEAR")
    assert missing_confirm.startswith("ERR usage MAGFRAME VERIFY CONFIRM")
    for line in (bad_bias, bogus, missing_confirm):
        assert mag_protocol.decode_line(line) is None


def test_clear_resets_calibrated_and_draft_stays_incomplete(transcript):
    ack = mag_protocol.parse_magcal_event(block(transcript, "MAGCAL CLEAR", 0)[0])
    assert ack.state == "cleared_ram"
    apply_after_clear = mag_protocol.parse_magcal_event(block(transcript, "MAGCAL APPLY", 1)[0])
    assert apply_after_clear.state == "apply_rejected"
    assert apply_after_clear.reason == "incomplete_draft"
    assert "草稿未凑齐" in apply_after_clear.reason_text


def test_armed_blocks_writes_for_both_magcal_and_magframe(transcript):
    bias_blocked = mag_protocol.parse_magcal_event(block(transcript, "MAGCAL SET BIAS 5 5 5", 0)[0])
    assert bias_blocked.state == "armed_blocked"
    verify_blocked = mag_protocol.parse_magframe_event(block(transcript, "MAGFRAME VERIFY CONFIRM", 1)[0])
    assert verify_blocked.state == "armed_blocked"


# ------------------------------------------------------------------ MAG 原始读数（源码锚定夹具）

_KEY_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=")


def _mag_report_format_keys() -> tuple[str, ...]:
    """从 `App/Src/app_mag.c` 的真实格式串里抽出 `key=` 名字，保持源码顺序。

    和 `tools/panel_qa/fixtures.py::firmware_format_keys()` 同一个技术，
    本地重做一份而不是 import 共享版本：那个模块的可改范围不属于本任务。
    """
    text = (ROOT / "App" / "Src" / "app_mag.c").read_text(encoding="utf-8")
    match = re.search(r'"MAG ok=[^"]*"', text)
    assert match is not None, "APP_MAG_Report 的 MAG ok=... 格式串未找到"
    return tuple(_KEY_RE.findall(match.group(0)))


def test_mag_report_fixture_keys_match_the_real_format_string():
    assert _mag_report_format_keys() == (
        "ok", "init", "st", "type", "addr", "who", "n", "raw", "mgauss",
    )


def test_parse_mag_raw_sample_reads_a_source_anchored_fixture_line():
    keys = _mag_report_format_keys()
    assert {"ok", "raw", "mgauss"} <= set(keys)
    line = (
        "MAG ok=1 init=0 st=0 type=QMC5883L addr=0x0D who=0xFF n=42 "
        "raw=111,-222,333 mgauss=145,-260,398\r\n"
    )
    sample = mag_protocol.parse_mag_raw_sample(line)
    assert sample.ok is True
    assert sample.raw_counts == (111, -222, 333)
    assert sample.chip_mgauss == (145.0, -260.0, 398.0)


def test_parse_mag_raw_sample_decodes_not_ok_samples_too():
    line = "MAG ok=0 init=1 st=2 type=NONE addr=0x00 who=0x00 n=0 raw=0,0,0 mgauss=0,0,0\r\n"
    sample = mag_protocol.parse_mag_raw_sample(line)
    assert sample.ok is False


# ------------------------------------------------------------------ 芯片 -> FLU 默认旋转


def test_chip_to_flu_default_matches_the_svc_mag_derivation():
    # Services/Src/svc_mag.c::SVC_MAG_DefaultRotation() is SVC_MAG_ROTATION_NONE
    # (chip == FRD), then Driver/Inc/drv_frame_contract.h::DRV_FRAME_FrdToFlu
    # negates y and z. Verified against both source files, not just the
    # comment restating it.
    assert mag_protocol.chip_to_flu_default_mgauss((100.0, -50.0, 25.0)) == (100.0, 50.0, -25.0)
    assert mag_protocol.chip_to_flu_default_mgauss((0.0, 0.0, 0.0)) == (0.0, 0.0, 0.0)


# ------------------------------------------------------------------ decode_line 派发


def test_decode_line_returns_none_for_unrelated_text():
    for line in ("PONG", "OK", "CFG generation=3", "PARAM idx=0 value=1.0", "", "MAGIC"):
        assert mag_protocol.decode_line(line) is None


def test_decode_line_dispatches_every_known_shape():
    cases = {
        "MAGCAL calibrated=1 axis_verified=1 axis_effective=1 contract_stored=2 contract_live=2 dirty=0":
            mag_protocol.MagCalStatus,
        "MAGCAL bias_mgauss=1.000,2.000,3.000": mag_protocol.MagCalBias,
        "MAGCAL matrix row0=1.0000,0.0000,0.0000 row1=0.0000,1.0000,0.0000 row2=0.0000,0.0000,1.0000":
            mag_protocol.MagCalMatrix,
        "MAGCAL draft bias_set=1 row_set=1,0,0": mag_protocol.MagCalDraft,
        "MAGCAL fusion subsystem_enabled=1 used=0 field_rejected=0 ignored=0 recovery=0 error_deg=0.00":
            mag_protocol.MagFusionStatus,
        "MAGCAL state=applied_ram": mag_protocol.MagCalEvent,
        "MAGFRAME axis_verified=1 axis_effective=1 contract_stored=2 contract_live=2 "
        "derivation=svc_mag_default_rotation_unverified": mag_protocol.MagFrameStatus,
        "MAGFRAME state=verified": mag_protocol.MagFrameEvent,
        "MAG ok=1 init=0 st=0 type=QMC5883L addr=0x0D who=0xFF n=1 raw=1,2,3 mgauss=1,2,3":
            mag_protocol.MagRawSample,
    }
    for line, expected_type in cases.items():
        decoded = mag_protocol.decode_line(line)
        assert isinstance(decoded, expected_type), (line, decoded)


# ------------------------------------------------------------------ 严格校验：非法/缺字段/矛盾回包


@pytest.mark.parametrize("line", [
    "MAGCAL calibrated=2 axis_verified=0 axis_effective=0 contract_stored=0 contract_live=0 dirty=0",
    "MAGCAL calibrated=1 axis_verified=5 axis_effective=0 contract_stored=0 contract_live=0 dirty=0",
    "MAGCAL calibrated=1 axis_verified=0 axis_effective=0 contract_stored=0 dirty=0",  # missing contract_live
    "MAGCAL calibrated=x axis_verified=0 axis_effective=0 contract_stored=0 contract_live=0 dirty=0",
])
def test_magcal_status_rejects_invalid_or_incomplete_fields(line):
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_status(line)


def test_magcal_bias_rejects_nan_and_wrong_arity():
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_bias("MAGCAL bias_mgauss=nan,0.0,0.0")
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_bias("MAGCAL bias_mgauss=1.0,2.0")
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_bias("MAGCAL calibrated=1")  # wrong line kind entirely


def test_magcal_matrix_rejects_missing_row():
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_matrix(
            "MAGCAL matrix row0=1,0,0 row1=0,1,0")  # row2 missing


def test_magcal_draft_rejects_malformed_row_set():
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_draft("MAGCAL draft bias_set=1 row_set=1,0")
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_draft("MAGCAL draft bias_set=1 row_set=1,0,9")


def test_magcal_fusion_rejects_infinite_error_deg():
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_fusion(
            "MAGCAL fusion subsystem_enabled=1 used=1 field_rejected=0 ignored=0 "
            "recovery=0 error_deg=inf")


def test_magframe_status_rejects_unexpected_derivation_text():
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magframe_status(
            "MAGFRAME axis_verified=1 axis_effective=1 contract_stored=1 contract_live=1 "
            "derivation=something_else")


def test_magcal_event_rejects_out_of_range_row():
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_magcal_event("MAGCAL state=draft_matrix_row row=3")


def test_mag_raw_sample_rejects_two_component_raw():
    with pytest.raises(mag_protocol.MagProtocolError):
        mag_protocol.parse_mag_raw_sample("MAG ok=1 raw=1,2 mgauss=1,2,3")


def test_wrong_line_kind_is_rejected_by_every_specific_parser():
    unrelated = "OK"
    parsers = (
        mag_protocol.parse_magcal_status, mag_protocol.parse_magcal_bias,
        mag_protocol.parse_magcal_matrix, mag_protocol.parse_magcal_draft,
        mag_protocol.parse_magcal_fusion, mag_protocol.parse_magcal_event,
        mag_protocol.parse_magframe_status, mag_protocol.parse_magframe_event,
        mag_protocol.parse_mag_raw_sample,
    )
    for parser in parsers:
        with pytest.raises(mag_protocol.MagProtocolError):
            parser(unrelated)
