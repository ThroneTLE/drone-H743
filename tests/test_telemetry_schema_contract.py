"""Contract tests for the VOFA telemetry channel schema (``TELEM?``).

The schema exists so the ground station never has to hard-code "float #7 is
``roll_rate_kd``". Before this table, that mapping lived in three places at once
(the fill code in ``freertos.c``, a comment above ``VOFA_task``, and the ground
station project file) with nothing keeping them in sync -- and it had already
drifted: the old comment omitted ``[22] vel_z_kd``.

Three properties matter and are easy to regress:

1. The wire schema must enumerate every channel exactly once, in fill order.
2. The schema hash must change whenever any channel metadata changes, because
   the ground station uses it to decide whether to rebuild its dashboard.
3. A schema reply must not overrun the 32-deep UART TX queue, which drops the
   *oldest* message when full -- losing a channel line silently mis-binds a plot.

The first two are checked by compiling the real module for the host and running
it, so these assert the shipped C behaviour rather than a Python re-model.
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]

HARNESS = """
#include "app_telemetry.h"

#include <stdarg.h>
#include <stdio.h>

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    (void)vprintf(format, args);
    va_end(args);
}

int main(void)
{
    uint32_t from = 0U;

    APP_Telemetry_ReportHeader();
    while (from < APP_Telemetry_ChannelCount()) {
        APP_Telemetry_ReportPage(from);
        from += APP_TELEM_PAGE_SIZE;
    }
    return 0;
}
"""


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def build_and_run(tmp_path: Path, telemetry_source: str | None = None) -> list[str]:
    """Compile the real app_telemetry.c for the host and return its output lines."""
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    tmp_path.mkdir(parents=True, exist_ok=True)
    source = tmp_path / "app_telemetry.c"
    source.write_text(
        telemetry_source
        if telemetry_source is not None
        else read("App/Src/app_telemetry.c"),
        encoding="utf-8",
    )

    harness = tmp_path / "harness.c"
    harness.write_text(HARNESS, encoding="ascii")

    executable = tmp_path / "telem.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(source),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
    )

    result = subprocess.run([str(executable)], check=True, capture_output=True)
    text = result.stdout.decode("ascii")
    return [line.strip("\r") for line in text.split("\n") if line.strip()]


def parse_kv(line: str) -> dict[str, str]:
    return dict(item.split("=", 1) for item in line.split() if "=" in item)


@pytest.fixture(scope="module")
def schema_lines(tmp_path_factory) -> list[str]:
    return build_and_run(tmp_path_factory.mktemp("telem"))


def test_header_reports_version_count_rate_and_hash(schema_lines: list[str]) -> None:
    header = schema_lines[0]
    assert header.startswith("TELEM ver="), header

    fields = parse_kv(header)
    # v3：状态监视的轴向数据统一为机体 FLU，并携带契约版本。
    assert fields["ver"] == "4"
    assert fields["frame"] == "body_flu"
    assert fields["contract"] == "1"
    assert int(fields["n"]) > 0
    # 25 ms send period -> 40 Hz. Both come from APP_TELEM_PERIOD_MS.
    assert int(fields["rate"]) == 40
    assert int(fields["page"]) > 0
    # The ground station keys its dashboard cache on this; it must be 8 hex digits.
    assert re.fullmatch(r"[0-9A-F]{8}", fields["hash"]), fields["hash"]


def test_every_channel_reported_exactly_once_in_fill_order(
    schema_lines: list[str],
) -> None:
    header = parse_kv(schema_lines[0])
    channels = [parse_kv(line) for line in schema_lines if line.startswith("TELEM CH ")]

    indices = [int(c["idx"]) for c in channels]
    assert indices == list(range(int(header["n"]))), (
        "channel indices must be contiguous, ascending and complete"
    )


# The float order on the wire as it shipped before the channel table existed,
# transcribed from the pre-refactor `vofa_data[N] = ...` assignments. Every
# existing host decoder (tools/vofa_serial_capture.py, drone_tcp_panel.py, the
# recorded flight logs) is indexed by these positions, so the table may only
# ever APPEND. Reordering or inserting here silently mis-binds every plot and
# invalidates historical captures.
HISTORICAL_WIRE_ORDER = [
    "roll", "pitch", "yaw", "flow_height", "uptime", "vel_est_x", "vel_est_y",
    "reserved_7", "reserved_8", "reserved_9", "reserved_10",
    "pos_x_kp", "pos_y_kp", "vel_x_kd", "vel_y_kd",
    "pos_est_x", "pos_est_y", "vel_loop_enable",
    "reserved_18", "reserved_19", "pos_z_kp", "reserved_21", "vel_z_kd",
    "fusion_acc_err", "fusion_acc_ignored", "fusion_acc_recovery",
    "fusion_acc_corrections", "fusion_acc_norm_rej",
]


def test_wire_order_is_unchanged_from_the_pre_table_layout(
    schema_lines: list[str],
) -> None:
    channels = [parse_kv(line) for line in schema_lines if line.startswith("TELEM CH ")]
    names = [c["name"] for c in sorted(channels, key=lambda c: int(c["idx"]))]

    # Prefix comparison, so appending new channels stays legal.
    assert names[: len(HISTORICAL_WIRE_ORDER)] == HISTORICAL_WIRE_ORDER
    assert len(names) >= len(HISTORICAL_WIRE_ORDER)


def test_channel_fields_are_wire_safe(schema_lines: list[str]) -> None:
    channels = [parse_kv(line) for line in schema_lines if line.startswith("TELEM CH ")]
    assert channels

    names = [c["name"] for c in channels]
    assert len(set(names)) == len(names), "channel names must be unique"

    for channel in channels:
        # The reply is whitespace-tokenised key=value, so no field may contain
        # a space or the ground station's parser silently mis-splits the line.
        for key in ("name", "unit", "grp", "param"):
            assert " " not in channel[key]
            assert channel[key], f"{key} must not be empty"
        assert float(channel["min"]) < float(channel["max"]), channel["name"]


def test_page_footers_chain_and_terminate(schema_lines: list[str]) -> None:
    header = parse_kv(schema_lines[0])
    total = int(header["n"])
    page_size = int(header["page"])

    footers = [parse_kv(line) for line in schema_lines if line.startswith("TELEM PAGE ")]
    assert footers, "each page must end with a footer so the client can chain requests"

    expected_from = 0
    for footer in footers:
        assert int(footer["from"]) == expected_from
        count = int(footer["count"])
        assert 0 < count <= page_size
        expected_from += count

    assert expected_from == total
    # Only the final footer terminates the walk.
    assert [int(f["next"]) for f in footers[:-1]] == [
        int(f["from"]) + int(f["count"]) for f in footers[:-1]
    ]
    assert int(footers[-1]["next"]) == -1


def test_page_reply_cannot_overrun_the_tx_queue(schema_lines: list[str]) -> None:
    queue_depth = int(
        re.search(
            r"uartTxQueueHandle\s*=\s*osMessageQueueNew\s*\(\s*(\d+)",
            read("Core/Src/freertos.c"),
        ).group(1)
    )

    header = parse_kv(schema_lines[0])
    # One reply = page_size channel lines + one footer. QueueText drops the
    # OLDEST entry when the queue is full, so a reply must leave ample headroom
    # for concurrent traffic (heartbeats, ACKs) rather than merely fit.
    assert int(header["page"]) + 1 <= queue_depth // 2


def test_schema_hash_changes_when_channel_metadata_changes(tmp_path: Path) -> None:
    baseline = build_and_run(tmp_path / "base")

    source = read("App/Src/app_telemetry.c")
    assert '{"roll",' in source
    mutated = source.replace('{"roll",', '{"rollX",', 1)
    assert mutated != source

    changed = build_and_run(tmp_path / "mutated", telemetry_source=mutated)

    base_hash = parse_kv(baseline[0])["hash"]
    changed_hash = parse_kv(changed[0])["hash"]
    assert base_hash != changed_hash, (
        "renaming a channel must change the schema hash, otherwise the ground "
        "station keeps a stale dashboard after a firmware update"
    )


def test_schema_hash_covers_frame_provenance(tmp_path: Path) -> None:
    baseline = build_and_run(tmp_path / "frame_base")
    source = read("App/Src/app_telemetry.c")
    assert "APP_TELEM_BODY_FRAME_NAME" in source
    frame_mutated = source.replace(
        '#define APP_TELEM_BODY_FRAME_NAME "body_flu"',
        '#define APP_TELEM_BODY_FRAME_NAME "body_frd"',
        1,
    )
    assert frame_mutated != source
    contract_mutated = source.replace(
        "#define APP_TELEM_FRAME_CONTRACT_VERSION DRV_FRAME_CONTRACT_VERSION",
        "#define APP_TELEM_FRAME_CONTRACT_VERSION 2U",
        1,
    )
    assert contract_mutated != source

    changed_frame = build_and_run(
        tmp_path / "frame_mutated", telemetry_source=frame_mutated
    )
    changed_contract = build_and_run(
        tmp_path / "contract_mutated", telemetry_source=contract_mutated
    )
    assert parse_kv(changed_frame[0])["frame"] == "body_frd"
    assert parse_kv(changed_contract[0])["contract"] == "2"
    hashes = {
        parse_kv(baseline[0])["hash"],
        parse_kv(changed_frame[0])["hash"],
        parse_kv(changed_contract[0])["hash"],
    }
    assert len(hashes) == 3


PARAM_CHANNELS = {
    "pos_x_kp": "coax.pos_x_kp",
    "pos_y_kp": "coax.pos_y_kp",
    "vel_x_kd": "coax.vel_x_kd",
    "vel_y_kd": "coax.vel_y_kd",
    "vel_loop_enable": "coax.vel_loop_enable",
    "pos_z_kp": "coax.pos_z_kp",
    "vel_z_kd": "coax.vel_z_kd",
    # v2 追加的真名增益（通道 64 起）。上面 13 条是单环 PD 时代的换算 alias，
    # 保留只为不重排历史通道号；调参真正该动的是下面这些。
    # 注意 `roll_rate_kd`（alias，其实是角速度环的 P）与 `rate_roll_kd`（真正的
    # 角速度环 D）是两条不同的通道，名字只差词序。
    "rate_roll_kp": "coax.rate_roll_kp",
    "rate_pitch_kp": "coax.rate_pitch_kp",
    "rate_yaw_kp": "coax.rate_yaw_kp",
    "rate_roll_ki": "coax.rate_roll_ki",
    "rate_pitch_ki": "coax.rate_pitch_ki",
    "rate_yaw_ki": "coax.rate_yaw_ki",
    "rate_roll_kd": "coax.rate_roll_kd",
    "rate_pitch_kd": "coax.rate_pitch_kd",
    "rate_yaw_kd": "coax.rate_yaw_kd",
    "att_roll_kp": "coax.att_roll_kp",
    "att_pitch_kp": "coax.att_pitch_kp",
    "att_yaw_kp": "coax.att_yaw_kp",
    "vel_x_kp": "coax.vel_x_kp",
    "vel_y_kp": "coax.vel_y_kp",
    "vel_z_kp": "coax.vel_z_kp",
    "vel_x_ki": "coax.vel_x_ki",
    "vel_y_ki": "coax.vel_y_ki",
    "vel_z_ki": "coax.vel_z_ki",
    "angular_accel_lpf": "coax.angular_accel_lpf_cutoff_rad_s",
    "accel_lpf": "coax.accel_lpf_cutoff_hz",
}


def test_gain_channels_advertise_the_parameter_they_echo(
    schema_lines: list[str],
) -> None:
    """滑块由 param 数据驱动生成，所以 param 必须是真实的参数键。

    上位机不再自己维护一张"哪条通道是哪个增益"的表——那张表就是上一版
    通道映射漂移的老路。
    """
    channels = {
        parse_kv(line)["name"]: parse_kv(line)
        for line in schema_lines
        if line.startswith("TELEM CH ")
    }

    for name, param in PARAM_CHANNELS.items():
        assert channels[name]["param"] == param, name
        assert channels[name]["grp"] == "gain", name

    for name, channel in channels.items():
        if name not in PARAM_CHANNELS:
            assert channel["param"] == "-", name


def test_schema_hash_changes_when_a_param_binding_changes(tmp_path: Path) -> None:
    """param 变了 hash 必须变：否则滑块会静静地绑到错的参数上。"""
    baseline = build_and_run(tmp_path / "param_base")

    source = read("App/Src/app_telemetry.c")
    assert '"coax.rate_roll_kp"' in source
    mutated = source.replace('"coax.rate_roll_kp"}', '"coax.rate_pitch_kp"}', 1)
    assert mutated != source

    changed = build_and_run(tmp_path / "param_mutated", telemetry_source=mutated)
    assert parse_kv(baseline[0])["hash"] != parse_kv(changed[0])["hash"]


def test_schema_hash_is_stable_across_runs(tmp_path: Path) -> None:
    first = build_and_run(tmp_path / "a")
    second = build_and_run(tmp_path / "b")
    assert parse_kv(first[0])["hash"] == parse_kv(second[0])["hash"]


def test_fill_code_indexes_by_channel_enum(schema_lines: list[str]) -> None:
    # R-T1-1：填充代码从 Core/Src/freertos.c 的 VOFA_task 搬到遥测流的 Port 实现。
    source = read("App/Src/app_telem_port.c")
    task = source[source.index("uint8_t APP_TelemStream_PortSample") :]

    # A bare numeric index is exactly how the mapping drifted before; the enum
    # is what ties the fill order to the advertised schema.
    bare = re.findall(r"vofa_data\[\s*\d+\s*\]", task)
    assert not bare, f"use APP_TELEM_CH_* instead of numeric indices: {bare}"

    used = set(re.findall(r"vofa_data\[(APP_TELEM_CH_\w+)\]", task))
    vector_bases = set(
        re.findall(r"vofa_data\[(APP_TELEM_CH_CTRL_\w+) \+ axis\]", task)
    )
    channels = [parse_kv(line) for line in schema_lines if line.startswith("TELEM CH ")]
    assert len(used) + (3 * len(vector_bases)) == len(channels), (
        "every advertised channel must be written by the fill code"
    )


def test_frame_length_and_period_derive_from_the_channel_table() -> None:
    port = read("App/Src/app_telem_port.c")
    stream = read("App/Src/app_telem_stream.c")

    # If either of these is re-hardcoded, adding a channel silently truncates
    # the frame or de-syncs the advertised rate.
    assert "(values == NULL) || (count != (uint32_t)APP_TELEM_CH_COUNT)" in port
    assert "app_telem_stream.rate_hz           = APP_TELEM_RATE_HZ;" in stream
    # 掩码 v2 是变长 64/128 位，表长超过 128 必须先升帧版本而不是悄悄加一条。
    assert ("_Static_assert((int)APP_TELEM_CH_COUNT <= 128,"
            in read("App/Inc/app_telemetry.h"))


def test_channel_table_covers_every_enum_id() -> None:
    header = read("App/Inc/app_telemetry.h")
    table = read("App/Src/app_telemetry.c")

    enum_body = header[header.index("typedef enum {") : header.index("} APP_TelemChannelId")]
    enum_ids = re.findall(r"(APP_TELEM_CH_\w+)", enum_body)
    enum_ids = [name for name in enum_ids if name != "APP_TELEM_CH_COUNT"]

    # Skip past the array declaration itself ("...[APP_TELEM_CH_COUNT] = {").
    marker = "app_telem_channels[APP_TELEM_CH_COUNT] = {"
    body = table[table.index(marker) + len(marker) :]
    designated = re.findall(r"\[(APP_TELEM_CH_\w+)\]\s*=\s*\{", body)
    assert designated == enum_ids, (
        "the table must use designated initialisers for every enum id, in order"
    )


def test_telem_is_reachable_from_the_command_dispatcher() -> None:
    source = read("App/Src/app_control.c")
    dispatch = source[source.index("static void app_control_dispatch_tokens(char **tokens, uint32_t count, uint8_t emit_ack)\n{") :]

    assert '"TELEM?"' in dispatch
    assert "app_control_handle_telem" in dispatch
