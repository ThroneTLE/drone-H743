"""飞行日志记录频率子分频（默认 N=2，62.5 Hz）的源码契约。"""
from __future__ import annotations

import re
from pathlib import Path

from tools.panel_lib import ai_bridge_policy as policy

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


STAB = read("App/Src/app_stabilizer.c")
FLOG = read("App/Src/app_flight_log.c")
HDR = read("App/Inc/app_flight_log.h")
CMD = read("App/Src/app_cmd_flograte.c")


def test_skipped_ticks_never_call_observe() -> None:
    assert "if (((ctx->flight_log_divider & 0x03U) == 0U) && (stabilizer_flight_log_subdiv_due(ctx) != 0U)) {" in STAB
    assert STAB.count("stabilizer_flight_log_subdiv_due(ctx)") == 1
    assert STAB.count("APP_FlightLog_Observe(") == 1
    assert STAB.index("stabilizer_flight_log_subdiv_due(ctx) != 0U") < STAB.index("APP_FlightLog_Observe(")
    assert "ctx->flight_log_divider = (uint8_t)((uint32_t)(ctx->flight_log_divider + 1U) & 0x03U);" in STAB
    assert "uint8_t flight_log_subdiv_count;" in STAB


def test_defaults_and_range() -> None:
    assert re.search(r"#define APP_FLIGHT_LOG_RATE_HZ\s+125U", HDR)
    assert re.search(r"#define APP_FLIGHT_LOG_SUBDIV_DEFAULT\s+2U", HDR)
    assert re.search(r"#define APP_FLIGHT_LOG_SUBDIV_MAX\s+5U", HDR)
    setter = FLOG.split("uint8_t APP_FlightLog_SetSubdiv")[1].split("\n}\n")[0]
    assert "subdiv < 1U" in setter and "APP_FLIGHT_LOG_SUBDIV_MAX" in setter
    assert "recording" in setter and "export_active" in setter and "export_pending" in setter


def test_tail_follows_rate_and_header_records_actual_rate() -> None:
    assert "#define STABILIZER_FLIGHT_LOG_TAIL_RECORDS" in STAB
    assert "STABILIZER_FLIGHT_LOG_TAIL_RECORDS / APP_FlightLog_GetSubdiv()" in STAB
    assert "ctx->flight_log_tail_records = stabilizer_flight_log_tail_records();" in STAB
    assert "header->log_rate_hz = APP_FLIGHT_LOG_RATE_HZ / (uint32_t)flight_log_subdiv;" in FLOG


def test_command_registered_integer_only_and_allowed() -> None:
    assert "app_control_handle_flograte(tokens, count)" in read("App/Src/app_cmd_fallback.c")
    assert "uint8_t app_control_handle_flograte(char **tokens, uint32_t count);" in read(
        "App/Inc/app_control_internal.h")
    assert "App/Src/app_cmd_flograte.c" in read("CMakeLists.txt")
    assert "FLOGRATE" not in read("App/Src/app_control.c")
    assert "FLOGRATE" not in read("tools/drone_tcp_panel.py")
    assert "%f" not in CMD and "%g" not in CMD
    for text in ("state=rejected reason=recording", "state=rejected reason=range", " state=set",
                 "rate_hz_x10=", "window_s=", "capacity="):
        assert text in CMD
    assert policy.classify("FLOGRATE 2").kind == policy.ALLOW
    assert policy.classify("FLOGRATE?").kind == policy.ALLOW
