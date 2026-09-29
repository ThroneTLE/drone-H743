"""按位置绑定的遗留通道名表必须跟着固件 schema 走。

`tools/vofa_serial_capture.py` 与 `tools/synex_config_builder.py` 各有一张
`CHANNEL_NAMES`：它们服务的是 VOFA JustFloat 定长 28 float 帧，帧里**没有标签**，
"第 N 个 float 是谁"全靠这两张表的**下标**。掩码帧（`app_telem_frame.h`）把这条
约定搬进了每一帧，但这两个工具走的是 jf 后端，仍然吃老约定。

于是有一类改动会**安静失败**：把固件通道表里某个退役槽位重新启用（R-PWR-1 就是
这么干的——`reserved_7/8` 变成了 `batt_v`/`batt_i`），而这两张表还写着 `Reserved_7`。
工具不报错、不崩溃，只是从此把电压标成"保留"落进 CSV，或在 Synex 里画出一条叫
Reserved_7 的电压曲线。等有人拿这份 CSV 去做分析，错的是结论不是程序。

本文件钉住的就是这一条：**退役与在役必须两边同时成立**。
它不要求名字逐字相等——三张表历史上就是三套命名风格（`Roll_deg` / `roll_deg` /
`roll`），强行统一属于另一件事，不在这里做。
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def tool_channel_names(path: str) -> list[str]:
    """取出 `CHANNEL_NAMES = [...]` 里的字面量，跳过注释行。

    注释里出现引号是常事（中文引号对同样会被朴素正则捞走），所以先按行剔掉
    `#` 之后的内容再匹配——否则一条解释性注释就能让这张表凭空多出两项，
    而错位的表恰恰是本文件要防的东西。
    """
    source = read(path)
    start = source.index("CHANNEL_NAMES = [")
    block = source[start + len("CHANNEL_NAMES = ["):]
    block = block[:block.index("\n]")]
    names: list[str] = []
    for line in block.splitlines():
        code = line.split("#", 1)[0]
        names.extend(re.findall(r'"([^"]+)"', code))
    return names


def firmware_channel_slots() -> list[str]:
    """固件通道枚举名（去掉 `APP_TELEM_CH_` 前缀，不含 `COUNT`）。"""
    header = read("App/Inc/app_telemetry.h")
    body = re.search(
        r"typedef enum\s*\{(.*?)\}\s*APP_TelemChannel", header, re.S
    ).group(1)
    return [
        name for name in re.findall(r"APP_TELEM_CH_(\w+)", body) if name != "COUNT"
    ]


TOOL_TABLES = [
    ("tools/vofa_serial_capture.py", r"reserved_(\d+)"),
    ("tools/synex_config_builder.py", r"Reserved_(\d+)"),
]


@pytest.mark.parametrize("tool_path,reserved_pattern", TOOL_TABLES)
def test_retired_slots_agree_with_the_firmware_table(
    tool_path: str, reserved_pattern: str
) -> None:
    """工具表里叫 Reserved_N 的槽位，固件那边必须也还是退役的；反之亦然。"""
    firmware = firmware_channel_slots()
    tool = tool_channel_names(tool_path)

    assert tool, f"{tool_path} 的 CHANNEL_NAMES 解析为空，解析器或表结构变了"
    assert len(tool) <= len(firmware), (
        f"{tool_path} 有 {len(tool)} 路，比固件通道表的 {len(firmware)} 路还长；"
        "按位置绑定的表不可能比它所绑定的表更长"
    )

    for index, tool_name in enumerate(tool):
        firmware_retired = "RESERVED" in firmware[index]
        match = re.fullmatch(reserved_pattern, tool_name)
        tool_retired = match is not None

        assert tool_retired == firmware_retired, (
            f"{tool_path} 第 {index} 路是 {tool_name!r}，固件第 {index} 路是 "
            f"APP_TELEM_CH_{firmware[index]}。一边退役一边在役 —— 这张表按下标"
            "绑定，错位不会报错，只会让分析结论悄悄用错通道。"
            "重新启用退役槽位时，固件表与这两张工具表必须同一提交里一起改。"
        )

        if tool_retired:
            assert firmware[index] == f"RESERVED_{match.group(1)}", (
                f"{tool_path} 第 {index} 路叫 {tool_name!r}，但固件第 {index} 路是 "
                f"APP_TELEM_CH_{firmware[index]} —— 退役编号本身就对不上，"
                "说明这张表整体错位了一格以上"
            )


def test_the_power_channels_reached_both_legacy_tables() -> None:
    """R-PWR-1 的回归钉：7/8 号槽位在三处都不再是 reserved。

    上面那条通用规则已经覆盖了这个场景，这里再单钉一次具体编号，是因为
    通用规则只保证"两边一致"——如果有人把固件改回 reserved 而工具表跟着改回去，
    通用规则仍然绿。而这两路是有实机接线的真实传感器，退回 reserved 属于功能丢失。
    """
    firmware = firmware_channel_slots()
    assert firmware[7] == "BATT_V"
    assert firmware[8] == "BATT_I"

    assert tool_channel_names("tools/vofa_serial_capture.py")[7:9] == [
        "batt_v",
        "batt_i",
    ]
    assert tool_channel_names("tools/synex_config_builder.py")[7:9] == [
        "Batt_V",
        "Batt_I",
    ]
