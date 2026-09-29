"""Host contract for DShot special-command frame encoding (commands 1..47).

DShot 特殊命令帧编码的宿主侧契约（R-DSHOT command tier，1..47）。

背景：飞控要给 AM32 电调发命令 13（Extended DShot Telemetry enable）才能拿到
逐路电流回传。油门入口 `DRV_DShot_Encode` / `DRV_DShotTelem_EncodeRequest`
明确拒绝 1..47——那是安全属性，命令 7/8/20/21 会反转电机转向、命令 12 会写
电调 Flash，绝不能让油门路径的一次算错悄悄变成一条特殊命令。所以命令帧走
`DRV_DShot_EncodeCommand` / `DRV_DShotTelem_EncodeCommand` 这两个独立入口。

判据分两层：
  1. 命令 13 的两档帧值**手算**钉死成字面量（不从代码反推期望值，见下方
     两个 `*_matches_hand_derivation` 测试的函数体注释）。
  2. 1..47 的穷举用一份独立的 Python 参考实现交叉验证，外加边界拒绝、
     telemetry 位、解码回路、以及"油门入口没有被顺手放宽"的回归守卫。
"""
from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# 两个状态枚举的前两个值都是 OK=0 / INVALID=1（DRV_DShotStatus 与
# DRV_DShotTelemStatus 各自定义，数值恰好一致，共用一组常量）。
OK, INVALID = 0, 1


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the DShot command-encode contract")
    build = tmp_path_factory.mktemp("dshot-cmd")
    out = build / ("cmd.dll" if os.name == "nt" else "cmd.so")
    result = subprocess.run(
        [
            compiler, "-shared", "-fPIC", "-std=c11", "-O2",
            "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(ROOT / "Driver/Inc"),
            str(ROOT / "Driver/Src/drv_dshot.c"),
            str(ROOT / "Driver/Src/drv_dshot_telemetry.c"),
            "-o", str(out),
        ],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    handle.DRV_DShot_Encode.argtypes = [
        ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint16)]
    handle.DRV_DShot_EncodeCommand.argtypes = [
        ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint16)]
    handle.DRV_DShotTelem_EncodeRequest.argtypes = [
        ctypes.c_uint16, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint16)]
    handle.DRV_DShotTelem_EncodeCommand.argtypes = [
        ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint16)]
    return handle


# ---------------------------------------------------------------- 手算基准帧（命令 13）


def test_command_13_unidirectional_frame_matches_hand_derivation(lib):
    """手算过程（不取代码输出当期望值）：

    command = 13 = 0b00000001101 (11 bit)
    telemetry 位固定为 1（特殊命令帧的判别特征）
    value12 = (command << 1) | 1 = (13 << 1) | 1 = 26 | 1 = 27 = 0x01B
    packet(前 12 bit，crc 位暂为 0) = value12 << 4 = 27 << 4 = 432 = 0x01B0

    value12=0x01B 拆成三个 nibble：nibble0=0xB, nibble1=0x1, nibble2=0x0
    crc = nibble0 ^ nibble1 ^ nibble2 = 0xB ^ 0x1 ^ 0x0
        = 1011b ^ 0001b = 1010b = 0xA

    frame = packet | crc = 0x01B0 | 0x0A = 0x01BA (十进制 442)

    校验位: frame>>5 = 442>>5 = 13 = command；(frame>>4)&1 = 1 = telemetry。
    """
    out = ctypes.c_uint16()
    assert lib.DRV_DShot_EncodeCommand(13, ctypes.byref(out)) == OK
    assert out.value == 0x01BA


def test_command_13_bidirectional_frame_matches_hand_derivation(lib):
    """手算过程（不取代码输出当期望值）：

    与单向共享同一个 value12 = 27 = 0x01B，因此 packet 前 12 bit 同样是
    0x01B0（value 与 telemetry 位的编码规则两档完全一致，唯一差别在校验）。

    双向校验 = 单向校验取反：crc_bidi = ~crc_uni & 0xF = ~0xA & 0xF
        = ~1010b & 1111b = 0101b = 0x5

    frame = 0x01B0 | 0x5 = 0x01B5（十进制 437）

    0xA（单向）与 0x5（双向）逐位互补：1010 vs 0101，符合"载荷相同，
    4 bit 校验取反"的双向规则。
    """
    out = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeCommand(13, ctypes.byref(out)) == OK
    assert out.value == 0x01B5


def test_command_13_frames_differ_between_the_two_tiers(lib):
    """两档帧必须不同：高 12 bit(value+telemetry) 一致，低 4 bit crc 互补。"""
    uni = ctypes.c_uint16()
    bidi = ctypes.c_uint16()
    assert lib.DRV_DShot_EncodeCommand(13, ctypes.byref(uni)) == OK
    assert lib.DRV_DShotTelem_EncodeCommand(13, ctypes.byref(bidi)) == OK
    assert uni.value != bidi.value
    assert (uni.value >> 4) == (bidi.value >> 4), "value+telemetry 的 12 bit 必须一致"
    assert (uni.value & 0xF) == (~bidi.value) & 0xF, "4 bit 校验必须互为反码"


# ------------------------------------------------------- 穷举交叉验证（独立参考实现）


def _expected_command_frame(command: int, invert_crc: bool) -> int:
    """命令帧的独立 Python 参考实现，用于 1..47 的穷举交叉验证。

    与被测 C 代码依据同一份协议描述编写，因此不是"手算基准"（那部分见上面
    两个 *_matches_hand_derivation 测试），而是用**第二份独立实现**去接住
    "代码在个别 command 值上抄错一位"这类手算基准帧覆盖不到的错误。
    """
    value12 = (command << 1) | 1
    packet = value12 << 4
    crc = (value12 ^ (value12 >> 4) ^ (value12 >> 8)) & 0xF
    if invert_crc:
        crc = (~crc) & 0xF
    return packet | crc


@pytest.mark.parametrize("command", range(1, 48))
def test_unidirectional_command_matches_independent_reference(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShot_EncodeCommand(command, ctypes.byref(out)) == OK
    assert out.value == _expected_command_frame(command, invert_crc=False)


@pytest.mark.parametrize("command", range(1, 48))
def test_bidirectional_command_matches_independent_reference(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeCommand(command, ctypes.byref(out)) == OK
    assert out.value == _expected_command_frame(command, invert_crc=True)


# ---------------------------------------------------------------- 边界拒绝 / 接受


@pytest.mark.parametrize("command", [0, 48, 2047])
def test_unidirectional_command_rejects_throttle_range(lib, command):
    """0 和 48..2047 是油门，命令入口必须原样拒绝，不能静默当命令编。"""
    out = ctypes.c_uint16()
    assert lib.DRV_DShot_EncodeCommand(command, ctypes.byref(out)) == INVALID


@pytest.mark.parametrize("command", [1, 47])
def test_unidirectional_command_accepts_boundary_values(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShot_EncodeCommand(command, ctypes.byref(out)) == OK


@pytest.mark.parametrize("command", [0, 48, 2047])
def test_bidirectional_command_rejects_throttle_range(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeCommand(command, ctypes.byref(out)) == INVALID


@pytest.mark.parametrize("command", [1, 47])
def test_bidirectional_command_accepts_boundary_values(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeCommand(command, ctypes.byref(out)) == OK


def test_unidirectional_command_rejects_null_out(lib):
    assert lib.DRV_DShot_EncodeCommand(13, None) == INVALID


def test_bidirectional_command_rejects_null_out(lib):
    assert lib.DRV_DShotTelem_EncodeCommand(13, None) == INVALID


# ---------------------------------------------------------------- 回归守卫：油门入口没被放宽


def test_throttle_encoder_still_rejects_command_13_regression_guard(lib):
    """防止有人为了让命令 13 能发出去，顺手放宽了油门入口的校验范围。"""
    out = ctypes.c_uint16()
    assert lib.DRV_DShot_Encode(13, ctypes.byref(out)) == INVALID


def test_telemetry_request_still_rejects_command_13_regression_guard(lib):
    out = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeRequest(13, 0, ctypes.byref(out)) == INVALID


# ---------------------------------------------------------------- telemetry 位


@pytest.mark.parametrize("command", [1, 13, 47])
def test_unidirectional_command_frame_sets_telemetry_bit(lib, command):
    """特殊命令帧必须置位 telemetry（bit4），否则电调会当成极低油门执行。"""
    out = ctypes.c_uint16()
    assert lib.DRV_DShot_EncodeCommand(command, ctypes.byref(out)) == OK
    assert (out.value >> 4) & 1 == 1


@pytest.mark.parametrize("command", [1, 13, 47])
def test_bidirectional_command_frame_sets_telemetry_bit(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeCommand(command, ctypes.byref(out)) == OK
    assert (out.value >> 4) & 1 == 1


# ---------------------------------------------------------------- 解码回路


@pytest.mark.parametrize("command", range(1, 48))
def test_unidirectional_command_value_field_decodes_back(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShot_EncodeCommand(command, ctypes.byref(out)) == OK
    assert (out.value >> 5) == command


@pytest.mark.parametrize("command", range(1, 48))
def test_bidirectional_command_value_field_decodes_back(lib, command):
    out = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeCommand(command, ctypes.byref(out)) == OK
    assert (out.value >> 5) == command
