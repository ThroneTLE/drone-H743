"""上位机能生成的最长命令行必须放得进飞控两条命令链路的行缓冲。

2026-09-27 实机：舵机单独模式的 SYSID EXC 正好 128 个字符，USB 行缓冲原为 128（最多 127 个字符），
整行被悄悄丢掉，页面只看到"等待回显超时"。本文件钉住两件事：
1. 页面能发出的最坏 EXC 行（每个字段取固件允许的最宽写法，再加舵机单独的 servo_tilt_mrad）
   比 USB 与维护口（蓝牙）两处行缓冲都短；
2. USB 超长行整行丢弃并回 ERR，剩余部分不会被当成下一条命令。
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from sysid.excitation import Excitation  # noqa: E402


def _define(path: str, name: str) -> int:
    source = (ROOT / path).read_text(encoding="utf-8")
    match = re.search(rf"#define\s+{name}\s+(\d+)U?", source)
    assert match is not None, f"{path} 缺 {name}"
    return int(match.group(1))


def _worst_exc_line() -> str:
    # 每个数取固件边界内字符数最多的写法（amp 的 :g 最多 6 位有效数字）。
    spec = Excitation(profile=3, amplitude_rad_s=4.99999, duration_ms=30000, hold_ms=30000,
                      repeat=20, ramp_ms=30000, chirp_f0_hz=0.123456, chirp_f1_hz=99.9999,
                      prbs_bit_ms=99999, prbs_seed=4294967295)
    return spec.command() + " servo_tilt_mrad=262"


def test_the_longest_page_command_fits_both_command_links():
    line = _worst_exc_line()
    usb = _define("App/Src/app_usb_cdc.c", "APP_USB_CDC_LINE_SIZE")
    maint = _define("App/Src/app_maint_uart.c", "APP_MAINT_UART_LINE_SIZE")
    # 缓冲含结尾 0：可用字符数 = SIZE − 1。
    assert len(line) <= usb - 1, (len(line), usb)
    assert len(line) <= maint - 1, (len(line), maint)


def test_the_servo_exc_line_that_was_dropped_on_2026_09_27_now_fits_usb():
    line = ("SYSID EXC profile=doublet amp=0.31 dur_ms=8000 hold_ms=250 repeat=16 ramp_ms=150 "
            "f0=0.3 f1=6 bit_ms=40 seed=1 servo_tilt_mrad=87")
    assert len(line) == 128
    assert len(line) <= _define("App/Src/app_usb_cdc.c", "APP_USB_CDC_LINE_SIZE") - 1


def test_usb_overlong_lines_are_rejected_out_loud_and_whole():
    source = (ROOT / "App/Src/app_usb_cdc.c").read_text(encoding="utf-8")
    step = source.split("void APP_USB_CDC_Task_Step(void)")[1].split("\n}\n")[0]
    assert 'APP_Control_QueueText("ERR line too long max=%u\\r\\n"' in step
    # 超长后到行尾之前的字节一律丢弃（否则尾巴会被当成新命令执行）。
    assert step.index("if (app_usb_cdc_line_overflow != 0U) {\n            continue;") < \
        step.index("app_usb_cdc_line[app_usb_cdc_line_used++] = (char)byte;")
    assert "app_usb_cdc_line_overflow = 1U;" in step
    init = source.split("void APP_USB_CDC_Init(void)")[1].split("\n}\n")[0]
    assert "app_usb_cdc_line_overflow = 0U;" in init


def test_the_longest_page_command_fits_the_command_parser():
    """USB 收进来之后还要过 APP_Control_ProcessLine：行缓冲与拆段上限也得装得下（原为 128 字节 / 10 段）。"""
    line = _worst_exc_line()
    max_line = _define("App/Src/app_control.c", "APP_CONTROL_MAX_LINE")
    max_tokens = _define("App/Inc/app_control_internal.h", "APP_CONTROL_MAX_TOKENS")
    assert len(line) <= max_line - 1, (len(line), max_line)
    assert len(line.split()) <= max_tokens, (len(line.split()), max_tokens)
    assert len(line.split()) == 13           # SYSID EXC + 11 个键值：原来 10 段会丢掉末尾三个


def test_the_command_parser_rejects_overflow_out_loud():
    core = (ROOT / "App/Src/app_control_core.c").read_text(encoding="utf-8")
    split = core.split("uint32_t app_control_split_line(")[1].split("\n}\n")[0]
    assert 'APP_Control_QueueText("ERR line too long max=%u\\r\\n"' in split
    assert 'APP_Control_QueueText("ERR too many tokens max=%u\\r\\n"' in split
    assert "count >= max_tokens" in split
    process = (ROOT / "App/Src/app_control.c").read_text(encoding="utf-8").split(
        "void APP_Control_ProcessLine(const char *line)")[1].split("\n}\n")[0]
    assert "char *tokens[APP_CONTROL_MAX_TOKENS + 1U];" in process
    assert "app_control_split_line(line, buffer" in process
    assert "snprintf(buffer" not in process, "不能再静默截断"
