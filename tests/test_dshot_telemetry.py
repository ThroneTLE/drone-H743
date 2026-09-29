"""双向 DShot300 回传帧解码的宿主侧契约。

判据不是"我写的解码器自洽"，而是**独立的 Python 编码器造出线上码流，真实 C 解码器
必须还原出同一个数**。GCR 表、差分还原顺序、校验取反、指数尾数这四处只要错一处，
下面的往返就对不上。实机上这四种错都表现为"偶尔读到离谱转速"，波形是看不出来的。

这里不模拟 DMA 与寄存器：捕获时刻怎么来的属于 BSP，不属于协议。
"""
from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# nibble -> 五位组。与 C 侧同源但**独立书写**，写错会立刻被往返测试抓到。
GCR_ENCODE = [
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
]

TIMER_CLOCK_HZ = 120_000_000
DSHOT_BIT_RATE = 300_000
# 回传波特率 = 300k * 5/4 = 375k -> 120MHz / 375k = 320 tick/bit
TICKS_PER_BIT = 320


class TelemValue(ctypes.Structure):
    _fields_ = [
        ("period_us", ctypes.c_uint16),
        ("erpm", ctypes.c_uint32),
        ("not_spinning", ctypes.c_uint8),
        ("kind", ctypes.c_uint8),
        ("edt_type", ctypes.c_uint8),
        ("edt_value", ctypes.c_uint8),
    ]


OK, INVALID, BAD_EDGES, BAD_GCR, BAD_CRC = range(5)


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the DShot telemetry contract")
    build = tmp_path_factory.mktemp("dshot-telem")
    out = build / ("telem.dll" if os.name == "nt" else "telem.so")
    result = subprocess.run(
        [
            compiler, "-shared", "-fPIC", "-std=c11", "-O2",
            "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(ROOT / "Driver/Inc"),
            str(ROOT / "Driver/Src/drv_dshot_telemetry.c"),
            str(ROOT / "Driver/Src/drv_dshot.c"),
            "-o", str(out),
        ],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    handle.DRV_DShotTelem_EncodeRequest.argtypes = [
        ctypes.c_uint16, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint16)]
    handle.DRV_DShotTelem_BitsFromEdges.argtypes = [
        ctypes.POINTER(ctypes.c_uint16), ctypes.c_size_t, ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint32)]
    handle.DRV_DShotTelem_DecodeRaw.argtypes = [
        ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint16)]
    handle.DRV_DShotTelem_ValueFromPayload.argtypes = [
        ctypes.c_uint16, ctypes.POINTER(TelemValue)]
    handle.DRV_DShotTelem_Decode.argtypes = [
        ctypes.POINTER(ctypes.c_uint16), ctypes.c_size_t, ctypes.c_uint32,
        ctypes.POINTER(TelemValue)]
    handle.DRV_DShotTelem_MechanicalRpm.argtypes = [ctypes.c_uint32, ctypes.c_uint8]
    handle.DRV_DShotTelem_MechanicalRpm.restype = ctypes.c_uint32
    handle.DRV_DShotTelem_TicksPerBitQ8.argtypes = [
        ctypes.c_uint32, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32)]
    handle.DRV_DShot_Encode.argtypes = [ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint16)]
    return handle


# ---------------------------------------------------------------- Python 编码器


def crc_nibble(payload12: int) -> int:
    """回传帧的 4 bit 校验：全 16 bit 折叠异或后低 4 bit 必须是 0xF。"""
    for crc in range(16):
        value = (payload12 << 4) | crc
        folded = value ^ (value >> 8)
        folded ^= folded >> 4
        if (folded & 0xF) == 0xF:
            return crc
    raise AssertionError(f"no crc satisfies the fold for payload {payload12:#05x}")


def wire_bits(payload12: int) -> int:
    """12 bit 载荷 -> 21 bit 线上码流（MSB 先发，起始位为 0）。"""
    value16 = (payload12 << 4) | crc_nibble(payload12)
    gcr = 0
    for quintet in range(4):
        nibble = (value16 >> (4 * quintet)) & 0xF
        gcr |= GCR_ENCODE[nibble] << (5 * quintet)
    # 解码侧做的是 gcr = raw ^ (raw >> 1)；这里求它的逆，起始位 r20 固定为 0。
    raw = 0
    previous = 0
    for index in range(19, -1, -1):
        bit = ((gcr >> index) & 1) ^ previous
        raw |= bit << index
        previous = bit
    return raw & 0x1FFFFF


def edges_from_bits(raw21: int, ticks_per_bit: int = TICKS_PER_BIT,
                    start: int = 1000, trailing_idle: bool = True) -> list[int]:
    """21 bit 码流 -> 定时器捕获时刻序列（含首个下降沿）。"""
    bits = [(raw21 >> shift) & 1 for shift in range(20, -1, -1)]
    assert bits[0] == 0, "起始位必须是低电平"
    edges = [start & 0xFFFF]
    level = 0
    for index in range(1, 21):
        if bits[index] != level:
            edges.append((start + index * ticks_per_bit) & 0xFFFF)
            level = bits[index]
    if trailing_idle and level == 0:
        # 末位是低电平时，线路必然还有一次回空闲高电平的上升沿。
        edges.append((start + 21 * ticks_per_bit) & 0xFFFF)
    return edges


def payload_for_period(period_us: int) -> int:
    """周期 -> `eee mmmmmmmmm`，取能精确表示该周期的最小指数。"""
    for exponent in range(8):
        if period_us % (1 << exponent) == 0:
            mantissa = period_us >> exponent
            if mantissa < 512:
                return (exponent << 9) | mantissa
    raise AssertionError(f"{period_us} us is not representable")


def decode(lib, edges: list[int], ticks_per_bit_q8: int = TICKS_PER_BIT << 8):
    buffer = (ctypes.c_uint16 * len(edges))(*edges)
    value = TelemValue()
    status = lib.DRV_DShotTelem_Decode(buffer, len(edges), ticks_per_bit_q8,
                                       ctypes.byref(value))
    return status, value


# ---------------------------------------------------------------- 往返


@pytest.mark.parametrize("period_us", [26, 100, 200, 416, 1000, 4096, 32768])
def test_wire_roundtrip_recovers_the_period(lib, period_us):
    payload = payload_for_period(period_us)
    status, value = decode(lib, edges_from_bits(wire_bits(payload)))
    assert status == OK
    assert value.not_spinning == 0
    assert value.period_us == period_us
    assert value.erpm == 60_000_000 // period_us


def test_roundtrip_holds_without_the_trailing_idle_edge(lib):
    """末位是高电平时没有收尾上升沿；补齐逻辑必须自己把剩余位填成高。"""
    payload = payload_for_period(100)
    with_idle = edges_from_bits(wire_bits(payload), trailing_idle=True)
    without_idle = edges_from_bits(wire_bits(payload), trailing_idle=False)
    for edges in (with_idle, without_idle):
        status, value = decode(lib, edges)
        assert status == OK
        assert value.period_us == 100


def test_capture_counter_wraparound_is_not_an_error(lib):
    """16 位捕获值在帧中间回绕是常态，不能当成坏帧。"""
    payload = payload_for_period(200)
    edges = edges_from_bits(wire_bits(payload), start=0xFF00)
    status, value = decode(lib, edges)
    assert status == OK
    assert value.period_us == 200


def test_not_spinning_is_a_valid_reply_not_a_decode_failure(lib):
    status, value = decode(lib, edges_from_bits(wire_bits(0x0FFF)))
    assert status == OK
    assert value.not_spinning == 1
    assert value.erpm == 0
    assert value.period_us == 0


@pytest.mark.parametrize("edt_type", [0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E])
def test_edt_is_classified_without_refreshing_or_fabricating_erpm(lib, edt_type):
    value = TelemValue()
    payload = (edt_type << 8) | 37
    assert lib.DRV_DShotTelem_ValueFromPayload(payload, ctypes.byref(value)) == OK
    assert value.kind == 1
    assert value.edt_type == edt_type
    assert value.edt_value == 37
    assert value.erpm == 0
    assert value.not_spinning == 0


def test_zero_amp_edt_is_not_misreported_as_not_spinning(lib):
    value = TelemValue()
    assert lib.DRV_DShotTelem_ValueFromPayload(0x0600, ctypes.byref(value)) == OK
    assert (value.kind, value.edt_type, value.edt_value) == (1, 0x06, 0)
    assert value.not_spinning == 0


# ---------------------------------------------------------------- 拒绝


def test_corrupted_payload_is_rejected_rather_than_returning_a_plausible_rpm(lib):
    """翻掉一个数据位之后，绝不允许安静地返回一个看着正常的转速。"""
    payload = payload_for_period(100)
    raw = wire_bits(payload)
    rejected = 0
    for bit in range(20):
        status, value = decode(lib, edges_from_bits(raw ^ (1 << bit)))
        if status == OK:
            # 侥幸仍然合法时，至少不能等于原值
            assert value.period_us != 100, f"bit {bit} 翻转后仍解出原周期"
        else:
            assert status in (BAD_GCR, BAD_CRC, BAD_EDGES)
            rejected += 1
    assert rejected >= 16, f"只挡住了 {rejected}/20 个单比特错误"


def test_glitch_shorter_than_one_bit_is_rejected(lib):
    payload = payload_for_period(100)
    edges = edges_from_bits(wire_bits(payload))
    edges.insert(2, edges[1] + 4)  # 4 tick = 1/80 个位宽
    status, _ = decode(lib, edges)
    assert status == BAD_EDGES


def test_too_few_edges_is_rejected(lib):
    buffer = (ctypes.c_uint16 * 1)(1000)
    value = TelemValue()
    status = lib.DRV_DShotTelem_Decode(buffer, 1, TICKS_PER_BIT << 8,
                                       ctypes.byref(value))
    assert status == INVALID


# ---------------------------------------------------------------- 请求帧


@pytest.mark.parametrize("throttle", [0, 48, 1000, 2047])
def test_bidirectional_crc_is_the_complement_of_the_unidirectional_one(lib, throttle):
    """双向与单向的唯一数字差别就是校验取反——直接和真实单向编码器对拍。"""
    bidi = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeRequest(throttle, 0, ctypes.byref(bidi)) == OK
    uni = ctypes.c_uint16()
    assert lib.DRV_DShot_Encode(throttle, ctypes.byref(uni)) == 0

    assert (bidi.value >> 4) == (uni.value >> 4), "12 bit 载荷必须一致"
    assert (bidi.value & 0xF) == (~uni.value) & 0xF, "校验必须互为反码"


def test_encode_request_rejects_the_reserved_command_range(lib):
    out = ctypes.c_uint16()
    for throttle in (1, 47, 2048):
        assert lib.DRV_DShotTelem_EncodeRequest(throttle, 0, ctypes.byref(out)) == INVALID


def test_telemetry_request_bit_lands_in_bit_four(lib):
    plain = ctypes.c_uint16()
    extended = ctypes.c_uint16()
    assert lib.DRV_DShotTelem_EncodeRequest(1000, 0, ctypes.byref(plain)) == OK
    assert lib.DRV_DShotTelem_EncodeRequest(1000, 1, ctypes.byref(extended)) == OK
    assert (plain.value >> 5) == (extended.value >> 5), "油门位不受影响"
    assert ((plain.value >> 4) & 1) == 0
    assert ((extended.value >> 4) & 1) == 1


# ---------------------------------------------------------------- 时钟与单位


def test_ticks_per_bit_follows_the_five_over_four_telemetry_rate(lib):
    out = ctypes.c_uint32()
    assert lib.DRV_DShotTelem_TicksPerBitQ8(
        TIMER_CLOCK_HZ, DSHOT_BIT_RATE, ctypes.byref(out)) == OK
    assert out.value == TICKS_PER_BIT << 8


def test_mechanical_rpm_divides_by_pole_pairs_and_refuses_to_guess(lib):
    assert lib.DRV_DShotTelem_MechanicalRpm(60_000, 7) == 60_000 // 7
    assert lib.DRV_DShotTelem_MechanicalRpm(60_000, 0) == 0


# ---------------------------------------------------------------- GCR 字母表

# 下面三条检查 GCR 表本身，**不经过我的解码器**，所以不会被"编解码两边犯同一个错"
# 掩盖掉。但要说清楚它们能证到哪一步：
#
#   能证：选用的 16 个五位组构成的字母表是对的（游程受限、恰好 16/32 合法）。
#   不能证：nibble -> 五位组的**对应关系**。把表里两项对调，下面三条仍然全绿，
#           因为用到的五位组集合没变。
#
# 对应关系用的是经典 GCR(4,5) 标准表（0->11001, 1->11011, ... F->01111）。
# 真要钉死它只需要**一帧真实电调回包**：对应关系若错一位，4 bit 校验会以 15/16
# 的概率直接失败。这条留给台架验证，不在这里假装已经证过。


def test_gcr_alphabet_is_run_length_limited():
    """GCR 存在的理由就是限制游程，让接收端能恢复位时钟。"""
    def integrate(gcr20: int) -> int:
        raw, previous = 0, 0
        for index in range(19, -1, -1):
            bit = ((gcr20 >> index) & 1) ^ previous
            raw |= bit << index
            previous = bit
        return raw

    def max_run(raw21: int) -> int:
        bits = [(raw21 >> shift) & 1 for shift in range(20, -1, -1)]
        best = current = 1
        for index in range(1, 21):
            current = current + 1 if bits[index] == bits[index - 1] else 1
            best = max(best, current)
        return best

    worst = 0
    for n0 in range(16):
        for n1 in range(16):
            for n2 in range(16):
                for n3 in range(16):
                    word = (GCR_ENCODE[n0] | (GCR_ENCODE[n1] << 5) |
                            (GCR_ENCODE[n2] << 10) | (GCR_ENCODE[n3] << 15))
                    worst = max(worst, max_run(integrate(word)))
    assert worst == 3, f"线上最长同电平游程为 {worst}，GCR 字母表不对"


def test_alphabet_has_exactly_sixteen_of_thirty_two_codes():
    assert len(set(GCR_ENCODE)) == 16
    assert all(0 <= code < 32 for code in GCR_ENCODE)


def test_every_invalid_quintet_is_rejected(lib):
    """剩下 16 个五位组必须全部报错，不许安静地映射成某个 nibble。"""
    invalid = sorted(set(range(32)) - set(GCR_ENCODE))
    assert len(invalid) == 16
    payload = ctypes.c_uint16()
    for code in invalid:
        # 低位放非法五位组，其余三组用合法值填充
        gcr = code | (GCR_ENCODE[0] << 5) | (GCR_ENCODE[0] << 10) | (GCR_ENCODE[0] << 15)
        raw, previous = 0, 0
        for index in range(19, -1, -1):
            bit = ((gcr >> index) & 1) ^ previous
            raw |= bit << index
            previous = bit
        status = lib.DRV_DShotTelem_DecodeRaw(raw, ctypes.byref(payload))
        assert status == BAD_GCR, f"五位组 {code:#04x} 没有被拒绝"


# ---------------------------------------------------------------- 分层


def test_module_is_hardware_independent():
    import re

    source = (ROOT / "Driver/Src/drv_dshot_telemetry.c").read_text(encoding="utf-8")
    header = (ROOT / "Driver/Inc/drv_dshot_telemetry.h").read_text(encoding="utf-8")
    includes = re.findall(r'^#include\s+[<"]([^>"\n]+)', source + "\n" + header, re.M)
    assert set(includes) == {"drv_dshot_telemetry.h", "stdint.h", "stddef.h"}
    assert "HAL_" not in source
    assert "htim" not in source
    assert "Driver/Src/drv_dshot_telemetry.c" in (
        ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
