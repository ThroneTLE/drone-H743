"""飞行日志导出：只导最近一次、USB 导出提速、队列/预擦、FLOG? 实际频率的契约与主机侧行为。

固件没有宿主机可编译的入口（依赖 FreeRTOS/HAL），所以固件侧用源码契约；
主机侧（flight_log_receive 与地面站接收页）用假传输喂数据做行为测试。
"""
from __future__ import annotations

import io
import json
import re
import runpy
import sys
import time
import zlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import flight_log_receive as flog
from tools import project_paths
from tools.panel_lib import ai_bridge_policy as policy
from tools.panel_lib import log_export_scope

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


FLOG = read("App/Src/app_flight_log.c")
HDR = read("App/Inc/app_flight_log.h")
CMD = read("App/Src/app_cmd_flogdump.c")
FIXTURE = runpy.run_path(str(Path(__file__).with_name("test_flight_log_receive.py")))


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    return source[start:source.index("\n}\n", start)]


# ---------------------------------------------------------------- 命令与范围（固件源码契约）

def test_flogdump_is_a_new_command_word_outside_app_control() -> None:
    assert "app_control_handle_flogdump(tokens, count)" in read("App/Src/app_cmd_fallback.c")
    assert "uint8_t app_control_handle_flogdump(char **tokens, uint32_t count);" in read(
        "App/Inc/app_control_internal.h")
    assert "App/Src/app_cmd_flogdump.c" in read("CMakeLists.txt")
    assert "FLOGDUMP" not in read("App/Src/app_control.c")
    assert "FLOGDUMP" not in read("tools/drone_tcp_panel.py")
    assert 'strcmp(tokens[1], "LAST") == 0' in CMD and 'strcmp(tokens[1], "ALL") == 0' in CMD
    assert "APP_FlightLog_StartDumpScope(scope)" in CMD
    assert "FLOG ERROR start %s" in CMD
    assert "%f" not in CMD and "%g" not in CMD


def test_scope_api_reuses_the_existing_export_state_machine() -> None:
    assert "APP_FLIGHT_LOG_EXPORT_SCOPE_ALL = 0" in HDR
    assert "APP_FLIGHT_LOG_EXPORT_SCOPE_LAST_RUN = 1" in HDR
    assert "APP_FlightLogCommandStatus APP_FlightLog_StartDumpScope(APP_FlightLogExportScope scope);" in HDR
    # 旧入口 = 全量，行为不变
    assert "return APP_FlightLog_StartDumpScope(APP_FLIGHT_LOG_EXPORT_SCOPE_ALL);" in FLOG
    start = function_body(FLOG, "static APP_FlightLogCommandStatus flight_log_start_export_from_background")
    # total / sectors / 起始扇区位置三者必须同源于 first_pos，才与实际导出范围一致
    assert "flight_log_find_last_run_first_pos()" in start
    assert "(flight_log_status.used_sectors - first_pos) * APP_FLASH_SERVICE_SECTOR_SIZE" in start
    assert "flight_log_export_sector_pos = first_pos;" in start
    assert "(unsigned long)(flight_log_status.used_sectors - first_pos)" in start
    assert "scope=%s" in start and '"last" : "all"' in start


def test_run_boundary_is_marked_in_sector_header_without_layout_change() -> None:
    assert "#define APP_FLIGHT_LOG_VERSION            12U" in FLOG
    assert "_Static_assert(sizeof(APP_FlightLogRecord) == 844U" in FLOG
    assert "sizeof(APP_FlightLogSectorHeader) == APP_FLIGHT_LOG_SECTOR_HEADER_SIZE" in FLOG
    assert "header->reserved[3] = (flight_log_new_run_pending != 0U) ? 1U : 0U;" in FLOG
    observe = function_body(FLOG, "void APP_FlightLog_Observe")
    # 录制 0->1 的边沿才算新一次记录
    edge = "if (flight_log_status.recording == 0U) {\n        flight_log_new_run_pending = 1U;"
    assert observe.index(edge) < observe.index("flight_log_status.recording = 1U;")
    write = function_body(FLOG, "static uint8_t flight_log_write_records")
    assert "(flight_log_new_run_pending != 0U) && (flight_log_status.sector_open != 0U)" in write
    opened = function_body(FLOG, "static uint8_t flight_log_open_sector")
    assert "flight_log_run_start_seq = flight_log_status.sector_seq;" in opened
    assert opened.index("flight_log_run_start_seq = flight_log_status.sector_seq;") \
        < opened.index("flight_log_status.sector_seq++;")


def test_last_run_search_stops_on_marker_gap_or_session_change() -> None:
    body = function_body(FLOG, "static uint32_t flight_log_find_last_run_first_pos")
    assert "(header.reserved[0] == 0xD5U) && (header.reserved[3] == 1U)" in body
    assert "flight_log_sector_order_seq[pos - 1U] + 1U" in body            # sector_seq 不连续
    assert "header.session_id != session" in body                          # 旧固件写的日志没有标记，按会话分界
    assert "flight_log_run_start_valid" in body                            # 本次上电已知起点则不必读 SD
    assert "return 0U;" in body                                            # 没有扇区 -> 空范围，与全量一致


def test_testfill_resets_run_tracking() -> None:
    fill = function_body(FLOG, "APP_FlightLogCommandStatus APP_FlightLog_TestFill")
    assert "flight_log_run_start_valid = 0U;" in fill
    assert "flight_log_new_run_pending = 0U;" in fill


def test_ai_bridge_does_not_allow_export_commands() -> None:
    for text in ("FLOGDUMP LAST", "FLOGDUMP ALL", "FLOG DUMP"):
        assert policy.classify(text).kind != policy.ALLOW


# ---------------------------------------------------------------- USB 提速（固件源码契约）

def test_usb_export_sends_multiple_blocks_per_service_period_within_budget() -> None:
    found = re.search(r"#define APP_FLIGHT_LOG_USB_EXPORT_BUDGET_MS (\d+)U", FLOG)
    assert found is not None
    assert 1 <= int(found.group(1)) <= 5
    step = function_body(FLOG, "static void flight_log_export_step")
    assert "while (flight_log_export_block_step() != 0U)" in step
    assert "APP_FLIGHT_LOG_USB_EXPORT_BUDGET_MS" in step
    # UART 路径：每周期一块，不进循环，块间隔与负载上限不变
    assert "(void)flight_log_export_block_step();\n        return;" in step
    assert "#define APP_FLIGHT_LOG_EXPORT_BLOCK_GAP_MS 40U" in FLOG
    assert "#define APP_FLIGHT_LOG_UART_EXPORT_PAYLOAD_MAX 48U" in FLOG
    assert "#define APP_FLIGHT_LOG_USB_EXPORT_PAYLOAD_MAX 1024U" in FLOG
    block = function_body(FLOG, "static uint8_t flight_log_export_block_step")
    # USB 忙/写失败：本块不推进、本周期结束，下周期重试，不丢块
    assert "APP_FLIGHT_LOG_USB_TX_TIMEOUT_MS) == 0U) {\n            return 0U;" in block
    assert block.rstrip().endswith("return 1U;")


def test_background_task_does_not_idle_wait_while_usb_export_runs() -> None:
    wait = function_body(FLOG, "uint32_t APP_FlightLog_BackgroundWaitMs")
    assert "APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY" in wait and "return 0U;" in wait
    assert "return APP_FLIGHT_LOG_BACKGROUND_BUSY_WAIT_MS;" in wait        # 有整批待写：只让出 1 个节拍
    assert "return idle_ms;" in wait
    background = read("App/Src/app_background.c")
    assert "APP_FlightLog_BackgroundWaitMs(APP_FLIGHT_LOG_BACKGROUND_IDLE_MS)" in background
    assert "osWaitForever" not in background
    assert re.search(r"#define APP_FLIGHT_LOG_BACKGROUND_BUSY_WAIT_MS [1-9]U", FLOG)  # 不为 0：同级任务不饿死


def test_crc32_table_version_matches_zlib() -> None:
    body = function_body(FLOG, "static uint32_t flight_log_crc32")
    table_text = body[body.index("{"):body.index("};")]
    table = [int(x, 16) for x in re.findall(r"0x([0-9A-F]{8})UL", table_text)]
    assert len(table) == 16

    def crc(data: bytes) -> int:
        value = 0xFFFFFFFF
        for byte in data:
            value ^= byte
            value = (value >> 4) ^ table[value & 15]
            value = (value >> 4) ^ table[value & 15]
        return ~value & 0xFFFFFFFF

    for blob in (b"", b"123456789", bytes(range(256)) * 5, b"\xff" * 1024):
        assert crc(blob) == zlib.crc32(blob)


# ---------------------------------------------------------------- dropped：队列与预擦

def test_queue_is_128_in_d1_ram_and_prewarm_runs_only_when_idle() -> None:
    assert "#define APP_FLIGHT_LOG_QUEUE_CAPACITY     128U" in FLOG
    assert ("APP_FLIGHT_LOG_RAM_D1_NOINIT\n"
            "static APP_FlightLogRecord flight_log_queue[APP_FLIGHT_LOG_QUEUE_CAPACITY];") in FLOG
    warm = function_body(FLOG, "static void flight_log_prewarm_step")
    for guard in ("flight_log_status.recording != 0U", "flight_log_queue_count != 0U",
                  "flight_log_flush_requested != 0U", "flight_log_export_pending != 0U",
                  "flight_log_status.export_active != 0U"):
        assert guard in warm
    assert "APP_FirmwareIdentity_GetCrc32()" in warm                       # 首次开扇区的整镜像 CRC 提前算
    assert "flight_log_preerase_tried_sector == sector" in warm            # 同一目标只试一次
    assert "flight_log_block_is_blank(sector)" in warm                     # 已擦的块不重复擦
    background = function_body(FLOG, "void APP_FlightLog_BackgroundStep")
    assert background.rstrip().endswith("flight_log_prewarm_step();")
    # open_sector 仍保留兜底擦除（预擦失败/未赶上时）
    assert "if (flight_log_prepare_erase(sector) == 0U) {" in function_body(
        FLOG, "static uint8_t flight_log_open_sector")


# ---------------------------------------------------------------- FLOG? 的 rate 字段

def test_flog_status_rate_is_actual_rate_matching_sector_header() -> None:
    control = read("App/Src/app_control.c")
    # 计算挪进日志模块（app_control.c 不许变长：test_hover_thrust_est 体积闸门）。
    assert "(unsigned int)APP_FlightLog_RateHz()," in control
    assert "(unsigned int)APP_FLIGHT_LOG_RATE_HZ," not in control
    assert "return APP_FLIGHT_LOG_RATE_HZ / (uint32_t)APP_FlightLog_GetSubdiv();" in FLOG
    assert "header->log_rate_hz = APP_FLIGHT_LOG_RATE_HZ / (uint32_t)flight_log_subdiv;" in FLOG
    # 125/N 取整：N=2 -> 62，与扇区头一致
    assert [125 // n for n in range(1, 6)] == [125, 62, 41, 31, 25]


# ---------------------------------------------------------------- 主机：范围、回退、速度

class ScriptedPort:
    """按写入的命令喂对应字节流的假串口。responses: 命令 -> 回应字节。"""

    def __init__(self, responses: dict[bytes, bytes]) -> None:
        self.responses = responses
        self.writes: list[bytes] = []
        self._rx = io.BytesIO(b"")

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        reply = self.responses.get(data)
        if reply is not None:
            rest = self._rx.read()
            self._rx = io.BytesIO(rest + reply)
        return len(data)

    def read(self, length: int) -> bytes:
        return self._rx.read(length)

    def readline(self) -> bytes:
        return self._rx.readline()


def sector_image(sector_count: int) -> bytes:
    sectors = []
    for _ in range(sector_count):
        image = bytearray(b"\xff" * flog.SECTOR_SIZE)
        image[:flog.SECTOR_HEADER_SIZE] = FIXTURE["make_sector_header"](
            version=9, record_size=flog.V9_RECORD_SIZE,
            params_struct=flog.V9_PARAMS_STRUCT, param_names=flog.V9_PARAM_NAMES)
        record = FIXTURE["make_record"]()
        image[flog.SECTOR_HEADER_SIZE:flog.SECTOR_HEADER_SIZE + len(record)] = record
        sectors.append(bytes(image))
    return b"".join(sectors)


def export_stream(payload: bytes, scope: str) -> bytes:
    total = len(payload)
    begin = (f"FLOG BEGIN version=1 block_magic=0x31424C46 transport=usbcdc encoding=binary "
             f"total={total} sectors={total // flog.SECTOR_SIZE} sector_size=4096 header_size=256 "
             f"record_size={flog.V9_RECORD_SIZE} log_rate=62 baud=57600 session=123 payload=1024 "
             f"scope={scope}\r\n").encode()
    end = f"FLOG END reason=done sent={total} total={total}\r\n".encode()
    return begin + FIXTURE["make_export_blocks"](payload) + end


def test_default_receive_requests_last_run_and_reports_scope(tmp_path) -> None:
    payload = sector_image(3)           # 环形区里可能有更多扇区；固件只发最近一次的 3 个
    port = ScriptedPort({flog.LAST_RUN_COMMAND: export_stream(payload, "last")})

    result = flog.receive_dump(port, tmp_path)

    assert port.writes[-1] == b"FLOGDUMP LAST\r\n"
    assert b"FLOG DUMP\r\n" not in port.writes
    assert result.total_bytes == 3 * flog.SECTOR_SIZE
    assert result.sectors == 3 and result.records == 3 and result.complete
    meta = json.loads(result.meta_path.read_text(encoding="utf-8"))
    assert meta["scope"] == "last" and meta["requested_scope"] == "last"
    assert meta["total_bytes"] == meta["good_bytes"] == 3 * flog.SECTOR_SIZE


def test_all_runs_option_uses_the_legacy_full_dump_command(tmp_path) -> None:
    payload = sector_image(5)
    port = ScriptedPort({flog.ALL_RUNS_COMMAND: export_stream(payload, "all")})

    result = flog.receive_dump(port, tmp_path, all_runs=True)

    assert port.writes[-1] == b"FLOG DUMP\r\n"
    assert b"FLOGDUMP LAST\r\n" not in port.writes
    assert result.sectors == 5 and result.complete
    assert json.loads(result.meta_path.read_text(encoding="utf-8"))["requested_scope"] == "all"


def test_old_firmware_falls_back_to_full_dump(tmp_path) -> None:
    payload = sector_image(2)
    logs: list[str] = []
    port = ScriptedPort({
        flog.LAST_RUN_COMMAND: b"ERR unknown cmd FLOGDUMP\r\n",
        flog.ALL_RUNS_COMMAND: export_stream(payload, "all").replace(b" scope=all", b""),  # 旧固件无 scope 字段
    })

    result = flog.receive_dump(port, tmp_path, log=logs.append)

    assert port.writes[-2:] == [b"FLOGDUMP LAST\r\n", b"FLOG DUMP\r\n"]
    assert result.complete and result.sectors == 2
    assert any("回退" in line for line in logs)
    meta = json.loads(result.meta_path.read_text(encoding="utf-8"))
    assert meta["scope"] == "all" and meta["requested_scope"] == "last"


def test_real_firmware_error_is_not_mistaken_for_old_firmware(tmp_path) -> None:
    port = ScriptedPort({flog.LAST_RUN_COMMAND: b"FLOG ERROR start recording\r\n"})
    with pytest.raises(flog.FlightLogError, match="start recording"):
        flog.receive_dump(port, tmp_path)
    assert b"FLOG DUMP\r\n" not in port.writes


def test_begin_wait_is_extended_only_when_asked() -> None:
    class SlowPort:
        def __init__(self) -> None:
            self.calls = 0

        def readline(self) -> bytes:
            self.calls += 1
            if self.calls < 4:
                return b""          # 固件还在往回读扇区头
            return b"FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1 scope=last\r\n"

    with pytest.raises(flog.FlightLogError, match="timeout waiting for FLOG BEGIN"):
        flog.read_until_begin(SlowPort())
    assert flog.read_until_begin(SlowPort(), wait_s=5.0).fields["scope"] == "last"


def test_cli_defaults_to_last_run_and_all_flag(monkeypatch, tmp_path) -> None:
    seen: list[bool] = []

    class FakeSerial:
        def __init__(self, *args, **kwargs) -> None:
            pass

        def __enter__(self):
            return self

        def __exit__(self, *exc) -> None:
            return None

    def fake_receive(port, output_dir, all_runs=False, **kwargs):
        seen.append(all_runs)
        return SimpleNamespace(records=0, total_bytes=0, good_bytes=0, missing_bytes=0,
                               bin_path=Path("a"), csv_path=Path("b"), meta_path=Path("c"))

    monkeypatch.setattr(flog, "serial", SimpleNamespace(Serial=FakeSerial))
    monkeypatch.setattr(flog, "receive_dump", fake_receive)
    for argv, expected in ((["x", "--port", "COM9", "--out-dir", str(tmp_path)], False),
                           (["x", "--port", "COM9", "--out-dir", str(tmp_path), "--all"], True)):
        monkeypatch.setattr(sys, "argv", argv)
        assert flog.main() == 0
        assert seen[-1] is expected


def test_host_decodes_a_megabyte_usb_stream_fast_enough_for_300_kib_per_second(tmp_path) -> None:
    """固件目标 >=300 KiB/s；主机解码不能成为瓶颈（假传输一次喂完，计纯解码耗时）。"""
    payload = sector_image(256)         # 1 MiB
    port = ScriptedPort({flog.LAST_RUN_COMMAND: export_stream(payload, "last")})
    started = time.perf_counter()
    result = flog.receive_dump(port, tmp_path)
    elapsed = time.perf_counter() - started

    assert result.complete and result.total_bytes == len(payload)
    assert len(payload) / elapsed / 1024 > 300


# ---------------------------------------------------------------- 地面站：选项与持久化

def test_scope_option_defaults_to_last_and_persists_in_panel_state(monkeypatch, tmp_path) -> None:
    state_path = tmp_path / "panel_state.json"
    monkeypatch.setattr(project_paths, "PANEL_STATE_PATH", state_path)
    panel = SimpleNamespace(_panel_state={"transport": "serial"})

    assert log_export_scope.load_export_all(panel) is False
    assert log_export_scope.save_export_all(True, panel) is True
    assert json.loads(state_path.read_text(encoding="utf-8")) == {"flight_log_export_all": True}
    assert panel._panel_state["flight_log_export_all"] is True          # 面板整份写回时不会冲掉
    assert log_export_scope.load_export_all(SimpleNamespace()) is True  # 新会话从文件读到
    state_path.write_text(json.dumps({"transport": "serial", "flight_log_export_all": "yes"}), encoding="utf-8")
    assert log_export_scope.load_export_all(SimpleNamespace()) is False  # 非布尔值视为默认
    log_export_scope.save_export_all(False, SimpleNamespace())
    assert json.loads(state_path.read_text(encoding="utf-8")) == {
        "transport": "serial", "flight_log_export_all": False}          # 读-改-写：别的键保留


def test_receiver_view_passes_scope_to_receive_dump() -> None:
    view = read("tools/panel_lib/log_receive_view.py")
    assert "log_export_scope.load_export_all(panel)" in view
    assert "log_export_scope.save_export_all(self.export_all_var.get(), self.panel)" in view
    assert "all_runs = bool(self.export_all_var.get())" in view
    assert "all_runs=all_runs)" in view
    assert "导出全部（默认只导最近一次）" in view
