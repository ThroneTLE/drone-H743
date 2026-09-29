"""双向 DShot300 bitbang 收发层的宿主侧契约。

背景：本板双向档最初用 DMA burst 写 TIM1->DMAR 加"发完把通道翻成输入捕获"，
实机验证帧值、极性、协议档位都核对过，电调仍然完全不响应——不转、也不回传。
参考实现明确记载 burst DMA 与双向 DShot 不兼容，于是改走 bitbang：一条 DMA
流按固定节拍写 GPIOE->BSRR（发送）/ 读 GPIOE->IDR（接收），PE9/PE11 两路
电调信号脚同在 GPIOE，天然可以共用同一条 DMA 流同时驱动/采样两路。

本文件不模拟 DMA、寄存器或采样时钟——那些属于 BSP。这里只钉两件纯数据变换
的事：发送侧"数字帧 -> 每子槽一个 BSRR 值"占空比对不对、空闲电平对不对、
两路独不独立；接收侧"过采样 IDR 序列 -> 21 bit 原始码流"在找不到边沿时
是否老实报错，以及在真实抖动下还原不还原得回原值。

判据同 test_dshot_telemetry.py 的思路：不是"我写的实现自洽"，而是用独立的
Python 编码器（wire_bits/GCR 表，与该文件同源但独立写）造出一段"电调回传"，
再喂给真实 C 解码链路核对。
"""
from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------- 常量与布局

SLOTS_PER_BIT = 8
FRAME_BITS = 16
TAIL_SLOTS = 8
FRAME_WORDS = FRAME_BITS * SLOTS_PER_BIT + TAIL_SLOTS  # 136，须与头文件宏一致

DSHOT_OK, DSHOT_INVALID = range(2)
TELEM_OK, TELEM_INVALID, TELEM_BAD_EDGES, TELEM_BAD_GCR, TELEM_BAD_CRC = range(5)

# 任意选的两路掩码，只要满足"set 落低 16 位、reset 落高 16 位、两路不重叠"
# 这个 BSP 会遵守的约定即可——本模块不认识引脚号，测试也不必用真实的 PE9/PE11。
CH0_SET, CH0_RESET = 0x00000001, 0x00010000
CH1_SET, CH1_RESET = 0x00000004, 0x00040000


class BitbangPins(ctypes.Structure):
    _fields_ = [("set_mask", ctypes.c_uint32), ("reset_mask", ctypes.c_uint32)]


class TelemValue(ctypes.Structure):
    _fields_ = [
        ("period_us", ctypes.c_uint16),
        ("erpm", ctypes.c_uint32),
        ("not_spinning", ctypes.c_uint8),
        ("kind", ctypes.c_uint8),
        ("edt_type", ctypes.c_uint8),
        ("edt_value", ctypes.c_uint8),
    ]


PINS0 = BitbangPins(CH0_SET, CH0_RESET)
PINS1 = BitbangPins(CH1_SET, CH1_RESET)


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the DShot bitbang contract")
    build = tmp_path_factory.mktemp("dshot-bitbang")
    out = build / ("bitbang.dll" if os.name == "nt" else "bitbang.so")
    result = subprocess.run(
        [
            compiler, "-shared", "-fPIC", "-std=c11", "-O2",
            "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-I", str(ROOT / "Driver/Inc"),
            str(ROOT / "Driver/Src/drv_dshot_bitbang.c"),
            str(ROOT / "Driver/Src/drv_dshot_telemetry.c"),
            "-o", str(out),
        ],
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    handle.DRV_DShotBitbang_BuildFrame.argtypes = [
        ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(BitbangPins), ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t]
    handle.DRV_DShotBitbang_BitsFromSamples.argtypes = [
        ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t, ctypes.c_uint32,
        ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32)]
    handle.DRV_DShotTelem_DecodeRaw.argtypes = [
        ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint16)]
    handle.DRV_DShotTelem_ValueFromPayload.argtypes = [
        ctypes.c_uint16, ctypes.POINTER(TelemValue)]
    return handle


# ---------------------------------------------------------------- 发送侧帮手


def build_frame(lib, packet, pins, inverted, capacity=FRAME_WORDS, sentinel=0xDEADBEEF):
    buf = (ctypes.c_uint32 * capacity)(*([sentinel] * capacity))
    packet_arr = (ctypes.c_uint16 * 2)(*packet) if packet is not None else None
    pins_arr = (BitbangPins * 2)(*pins) if pins is not None else None
    status = lib.DRV_DShotBitbang_BuildFrame(packet_arr, pins_arr, inverted, buf, capacity)
    return status, list(buf)


def channel_level(word: int, pins: BitbangPins) -> int:
    """从一个 BSRR 字里解出某一路当前是高还是低。

    顺带把"同一路同一个字里不许同时 set 和 reset"这条 BSRR 语义钉在这里——
    凡是调用这个函数的测试，都在隐式核对这一条。
    """
    is_set = (word & pins.set_mask) != 0
    is_reset = (word & pins.reset_mask) != 0
    assert not (is_set and is_reset), f"word {word:#010x} 同一路同时出现了 set 和 reset"
    assert is_set or is_reset, f"word {word:#010x} 这一路既没 set 也没 reset"
    return 1 if is_set else 0


def decode_channel_word(words, bit_index: int, pins: BitbangPins, inverted: int) -> int:
    """从第 bit_index 个 8 子槽分组里解出数据位的值。

    子槽 4 恰好能区分 duty=3（bit=0，子槽 3/4/5 都已回到空闲）与 duty=6
    （bit=1，子槽 3/4/5 仍是数据脉冲）——用它而不是子槽 0，是因为子槽 0
    无论 bit 是 0 还是 1 都必然是脉冲电平，分不出两种情况。
    """
    pulse_level = 0 if inverted else 1
    slot4 = words[bit_index * SLOTS_PER_BIT + 4]
    return 1 if channel_level(slot4, pins) == pulse_level else 0


def decode_channel_bits(words, pins: BitbangPins, inverted: int) -> int:
    value = 0
    for bit_index in range(FRAME_BITS):
        value = (value << 1) | decode_channel_word(words, bit_index, pins, inverted)
    return value


# ---------------------------------------------------------------- 1. 子槽占空精确


# 手算对照表（子槽 0..7，1=高电平，0=低电平）：
#   inverted=0（单向，空闲低、数据拉高）：bit=0 -> 1,1,1,0,0,0,0,0（高3低5）
#                                        bit=1 -> 1,1,1,1,1,1,0,0（高6低2）
#   inverted=1（双向，空闲高、数据拉低）：bit=0 -> 0,0,0,1,1,1,1,1（低3高5）
#                                        bit=1 -> 0,0,0,0,0,0,1,1（低6高2）
HAND_TABLE = {
    (0, 0): [1, 1, 1, 0, 0, 0, 0, 0],
    (0, 1): [1, 1, 1, 1, 1, 1, 0, 0],
    (1, 0): [0, 0, 0, 1, 1, 1, 1, 1],
    (1, 1): [0, 0, 0, 0, 0, 0, 1, 1],
}


@pytest.mark.parametrize("inverted", [0, 1])
@pytest.mark.parametrize("bit_value", [0, 1])
def test_subslot_duty_matches_the_hand_computed_table(lib, inverted, bit_value):
    """逐位逐槽断言，不只看子槽里高/低电平的总数。"""
    packet_word = 0xFFFF if bit_value else 0x0000
    status, words = build_frame(lib, (packet_word, packet_word), (PINS0, PINS1), inverted)
    assert status == DSHOT_OK
    expected = HAND_TABLE[(inverted, bit_value)]

    for bit_index in range(FRAME_BITS):
        group = words[bit_index * SLOTS_PER_BIT:(bit_index + 1) * SLOTS_PER_BIT]
        got_ch0 = [channel_level(w, PINS0) for w in group]
        got_ch1 = [channel_level(w, PINS1) for w in group]
        assert got_ch0 == expected, f"ch0 第 {bit_index} 位子槽错了: {got_ch0} != {expected}"
        assert got_ch1 == expected, f"ch1 第 {bit_index} 位子槽错了: {got_ch1} != {expected}"


# ---------------------------------------------------------------- 2. 空闲电平


@pytest.mark.parametrize("inverted", [0, 1])
def test_idle_level_holds_through_the_tail_slots(lib, inverted):
    """空闲电平错了，电调根本进不了双向模式（2026-09-21 实机就卡在这类问题上）。

    TAIL 的 8 个子槽是本函数唯一真正"写出空闲电平"的地方（首槽之前的空闲
    由上一帧的 TAIL 提供，不在本次输出范围内），所以直接钉在这里。
    """
    status, words = build_frame(lib, (0x5555, 0x2AAA), (PINS0, PINS1), inverted)
    assert status == DSHOT_OK
    idle_level = 1 if inverted else 0

    tail = words[FRAME_BITS * SLOTS_PER_BIT:]
    assert len(tail) == TAIL_SLOTS
    for w in tail:
        assert channel_level(w, PINS0) == idle_level
        assert channel_level(w, PINS1) == idle_level

    # 反向 sanity：数据段第一个子槽必然是脉冲电平，不会是空闲电平，
    # 否则上面的断言会因为"本来就一直是这个电平"而失去意义。
    assert channel_level(words[0], PINS0) != idle_level
    assert channel_level(words[0], PINS1) != idle_level


# ---------------------------------------------------------------- 3. 两路独立且同步


def test_channels_are_independent_and_bit_synchronous(lib):
    packet_a = (0xACE1, 0x1234)
    packet_b = (0xACE1, 0x5A3C)  # ch0 相同，ch1 换掉

    status_a, words_a = build_frame(lib, packet_a, (PINS0, PINS1), inverted=1)
    status_b, words_b = build_frame(lib, packet_b, (PINS0, PINS1), inverted=1)
    assert status_a == DSHOT_OK and status_b == DSHOT_OK

    # 每一个 BSRR 字都天然同步（两路写在同一个字里），先核对各自解出来的
    # 16 bit 就是各自传入的 packet。
    assert decode_channel_bits(words_a, PINS0, inverted=1) == packet_a[0]
    assert decode_channel_bits(words_a, PINS1, inverted=1) == packet_a[1]
    assert decode_channel_bits(words_b, PINS1, inverted=1) == packet_b[1]

    # 换掉 ch2 的 packet 不改变 ch1 的任何一个字。
    ch0_levels_a = [channel_level(w, PINS0) for w in words_a]
    ch0_levels_b = [channel_level(w, PINS0) for w in words_b]
    assert ch0_levels_a == ch0_levels_b, "改 ch2 的 packet 不该动 ch1 的任何一位"


# ---------------------------------------------------------------- 4. BSRR 语义


def test_bsrr_set_bits_stay_low_and_reset_bits_stay_high(lib):
    """拉高只能出现在低 16 位、拉低只能出现在高 16 位，两路都不许错混。"""
    for inverted in (0, 1):
        status, words = build_frame(lib, (0xB6DA, 0x2C97), (PINS0, PINS1), inverted)
        assert status == DSHOT_OK
        for w in words:
            assert (w & 0x0000FFFF) & (CH0_RESET | CH1_RESET) == 0, "低 16 位混进了 reset 掩码"
            assert (w & 0xFFFF0000) & (CH0_SET | CH1_SET) == 0, "高 16 位混进了 set 掩码"
            # channel_level 内部会断言"同一路不许同时 set 和 reset"；两路都跑一遍。
            channel_level(w, PINS0)
            channel_level(w, PINS1)


# ---------------------------------------------------------------- 5. 容量与空指针


def test_insufficient_capacity_is_rejected_and_writes_nothing(lib):
    sentinel = 0xDEADBEEF
    status, buf = build_frame(lib, (0x0100, 0x0200), (PINS0, PINS1), 1,
                              capacity=FRAME_WORDS - 1, sentinel=sentinel)
    assert status == DSHOT_INVALID
    assert buf == [sentinel] * (FRAME_WORDS - 1)


def test_null_packet_is_rejected_and_writes_nothing(lib):
    sentinel = 0xDEADBEEF
    status, buf = build_frame(lib, None, (PINS0, PINS1), 1, sentinel=sentinel)
    assert status == DSHOT_INVALID
    assert buf == [sentinel] * FRAME_WORDS


def test_null_pins_is_rejected_and_writes_nothing(lib):
    sentinel = 0xDEADBEEF
    status, buf = build_frame(lib, (0x0100, 0x0200), None, 1, sentinel=sentinel)
    assert status == DSHOT_INVALID
    assert buf == [sentinel] * FRAME_WORDS


def test_null_out_is_rejected(lib):
    packet_arr = (ctypes.c_uint16 * 2)(0x0100, 0x0200)
    pins_arr = (BitbangPins * 2)(PINS0, PINS1)
    status = lib.DRV_DShotBitbang_BuildFrame(packet_arr, pins_arr, 1, None, FRAME_WORDS)
    assert status == DSHOT_INVALID


def test_extra_capacity_is_accepted_but_only_frame_words_are_written(lib):
    """契约写的是"只写 DRV_DSHOT_BB_FRAME_WORDS 个字"，不是"至少写这么多"。"""
    sentinel = 0xDEADBEEF
    status, buf = build_frame(lib, (0x0100, 0x0200), (PINS0, PINS1), 1,
                              capacity=FRAME_WORDS + 4, sentinel=sentinel)
    assert status == DSHOT_OK
    assert all(w != sentinel for w in buf[:FRAME_WORDS])
    assert buf[FRAME_WORDS:] == [sentinel] * 4


# ---------------------------------------------------------------- 接收侧帮手


# nibble -> 五位组，与 drv_dshot_telemetry.c 同源但独立书写（同 test_dshot_telemetry.py）。
GCR_ENCODE = [
    0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
    0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F,
]


def crc_nibble(payload12: int) -> int:
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
    raw = 0
    previous = 0
    for index in range(19, -1, -1):
        bit = ((gcr >> index) & 1) ^ previous
        raw |= bit << index
        previous = bit
    return raw & 0x1FFFFF


def payload_for_period(period_us: int) -> int:
    for exponent in range(8):
        if period_us % (1 << exponent) == 0:
            mantissa = period_us >> exponent
            if mantissa < 512:
                return (exponent << 9) | mantissa
    raise AssertionError(f"{period_us} us is not representable")


def samples_from_bits(raw21: int, channel_mask: int, samples_per_bit: int,
                      jitter: list[int] | None = None,
                      idle_lead: int = 4, idle_tail: int | None = None) -> list[int]:
    """21 bit 线上码流 -> 过采样 IDR 序列（每个采样一个整字）。

    `idle_lead` 个采样模拟"触发采样前线路已经在空闲高电平"；`idle_tail`
    默认留 3 个位宽，模拟真实捕获缓冲在帧尾之后仍有余量——不留够会让
    "缓冲区恰好在帧尾截断"和"毛刺"混在一起，两者在这个函数里必须分得开。
    `jitter[i]` 给第 i 位单独叠加/减少若干个采样，用来测抖动容差。
    """
    bits = [(raw21 >> shift) & 1 for shift in range(20, -1, -1)]
    assert bits[0] == 0, "起始位必须是低电平"
    if idle_tail is None:
        idle_tail = samples_per_bit * 3
    samples = [channel_mask] * idle_lead
    for index, bit in enumerate(bits):
        n = samples_per_bit + (jitter[index] if jitter is not None else 0)
        assert n >= 1, "单个位的采样数不能被抖动削成 0"
        level = channel_mask if bit else 0
        samples.extend([level] * n)
    samples.extend([channel_mask] * idle_tail)
    return samples


def bits_from_samples(lib, samples: list[int], channel_mask: int, samples_per_bit_q8: int):
    arr = (ctypes.c_uint32 * len(samples))(*samples)
    raw21 = ctypes.c_uint32()
    status = lib.DRV_DShotBitbang_BitsFromSamples(
        arr, len(samples), channel_mask, samples_per_bit_q8, ctypes.byref(raw21))
    return status, raw21.value


def decode_value(lib, raw21: int):
    payload = ctypes.c_uint16()
    status = lib.DRV_DShotTelem_DecodeRaw(raw21, ctypes.byref(payload))
    if status != TELEM_OK:
        return status, None
    value = TelemValue()
    status2 = lib.DRV_DShotTelem_ValueFromPayload(payload.value, ctypes.byref(value))
    if status2 != TELEM_OK:
        return status2, None
    return TELEM_OK, value


CHANNEL_MASK = 0x00000200  # 任意选的一位，象征某路电调的采样位
SAMPLES_PER_BIT = 8
SAMPLES_PER_BIT_Q8 = SAMPLES_PER_BIT << 8


# ---------------------------------------------------------------- 6. 往返


@pytest.mark.parametrize("period_us", [26, 100, 4096])
def test_roundtrip_recovers_a_normal_erpm_with_reasonable_jitter(lib, period_us):
    """每位采样数 ±1 的合理抖动（奇偶位交替加减），模拟真实过采样时钟抖动。"""
    raw21 = wire_bits(payload_for_period(period_us))
    jitter = [1 if (i % 2 == 0) else -1 for i in range(21)]
    samples = samples_from_bits(raw21, CHANNEL_MASK, SAMPLES_PER_BIT, jitter=jitter)

    status, raw21_out = bits_from_samples(lib, samples, CHANNEL_MASK, SAMPLES_PER_BIT_Q8)
    assert status == TELEM_OK
    assert raw21_out == raw21

    status, value = decode_value(lib, raw21_out)
    assert status == TELEM_OK
    assert value.not_spinning == 0
    assert value.period_us == period_us
    assert value.erpm == 60_000_000 // period_us


def test_roundtrip_recovers_the_not_spinning_sentinel(lib):
    raw21 = wire_bits(0x0FFF)
    samples = samples_from_bits(raw21, CHANNEL_MASK, SAMPLES_PER_BIT)

    status, raw21_out = bits_from_samples(lib, samples, CHANNEL_MASK, SAMPLES_PER_BIT_Q8)
    assert status == TELEM_OK

    status, value = decode_value(lib, raw21_out)
    assert status == TELEM_OK
    assert value.not_spinning == 1
    assert value.erpm == 0
    assert value.period_us == 0


@pytest.mark.parametrize("edt_type,edt_value", [(0x06, 37), (0x04, 100)])
def test_roundtrip_recovers_an_edt_frame(lib, edt_type, edt_value):
    payload = (edt_type << 8) | edt_value
    raw21 = wire_bits(payload)
    samples = samples_from_bits(raw21, CHANNEL_MASK, SAMPLES_PER_BIT)

    status, raw21_out = bits_from_samples(lib, samples, CHANNEL_MASK, SAMPLES_PER_BIT_Q8)
    assert status == TELEM_OK

    status, value = decode_value(lib, raw21_out)
    assert status == TELEM_OK
    assert value.kind == 1
    assert value.edt_type == edt_type
    assert value.edt_value == edt_value
    assert value.not_spinning == 0


# ---------------------------------------------------------------- 7. 没有下降沿


def test_all_high_samples_report_bad_edges(lib):
    """一片高电平（电调没回话/idle）绝不能被解成一串合法的 0。"""
    samples = [CHANNEL_MASK] * 40
    status, _ = bits_from_samples(lib, samples, CHANNEL_MASK, SAMPLES_PER_BIT_Q8)
    assert status == TELEM_BAD_EDGES


def test_all_low_samples_report_bad_edges(lib):
    """全程低电平说明没等到真正的空闲期就开始采样，同样不能硬解。"""
    samples = [0] * 40
    status, _ = bits_from_samples(lib, samples, CHANNEL_MASK, SAMPLES_PER_BIT_Q8)
    assert status == TELEM_BAD_EDGES


def test_a_single_edge_with_no_further_transitions_reports_bad_edges(lib):
    """起始下降沿之后再没有任何翻转：只证明了 1 个 bit，不能补成合法的 21 bit。"""
    samples = [CHANNEL_MASK] * 10 + [0] * 30
    status, _ = bits_from_samples(lib, samples, CHANNEL_MASK, SAMPLES_PER_BIT_Q8)
    assert status == TELEM_BAD_EDGES


def test_a_glitch_shorter_than_one_bit_is_rejected(lib):
    """起始下降沿后 1 个采样就跳回高电平：比一个位宽还短，只可能是毛刺。"""
    samples = [CHANNEL_MASK] * 10 + [0] + [CHANNEL_MASK] * 30
    status, _ = bits_from_samples(lib, samples, CHANNEL_MASK, SAMPLES_PER_BIT_Q8)
    assert status == TELEM_BAD_EDGES


# ---------------------------------------------------------------- 8. 过采样容差


TOLERANCE_NOMINAL = 20  # 15% 正好是 3 个采样的整数，边界不会被四舍五入抹掉
TOLERANCE_Q8 = TOLERANCE_NOMINAL << 8


@pytest.mark.parametrize("period_us", [100, 416])
def test_decoding_tolerates_plus_minus_15_percent_per_bit_jitter(lib, period_us):
    raw21 = wire_bits(payload_for_period(period_us))
    jitter = [3 if (i % 2 == 0) else -3 for i in range(21)]  # 正好 ±15%
    samples = samples_from_bits(raw21, CHANNEL_MASK, TOLERANCE_NOMINAL, jitter=jitter)

    status, raw21_out = bits_from_samples(lib, samples, CHANNEL_MASK, TOLERANCE_Q8)
    assert status == TELEM_OK
    assert raw21_out == raw21


def test_decoding_fails_clearly_once_jitter_exceeds_tolerance(lib):
    raw21 = wire_bits(payload_for_period(200))
    # 把第 5 位的采样数拉大 5 倍位宽，远超 ±15% 容差，足够让这一位的游程
    # 被算成多出好几个位，后面的位全部错位。
    jitter = [0] * 21
    jitter[5] = TOLERANCE_NOMINAL * 5
    samples = samples_from_bits(raw21, CHANNEL_MASK, TOLERANCE_NOMINAL, jitter=jitter)

    status, raw21_out = bits_from_samples(lib, samples, CHANNEL_MASK, TOLERANCE_Q8)
    if status == TELEM_OK:
        assert raw21_out != raw21, "严重超差的抖动不该巧合解出原始码流"
        decode_status, value = decode_value(lib, raw21_out)
        if decode_status == TELEM_OK:
            assert value.period_us != 200, "严重超差的抖动不该巧合解出原周期"
    else:
        assert status == TELEM_BAD_EDGES


# ---------------------------------------------------------------- 分层


def test_module_is_hardware_independent():
    """保持与 drv_dshot_telemetry.c 同样的纯净度：无 HAL、无寄存器、无全局可变状态。"""
    import re

    source = (ROOT / "Driver/Src/drv_dshot_bitbang.c").read_text(encoding="utf-8")
    header = (ROOT / "Driver/Inc/drv_dshot_bitbang.h").read_text(encoding="utf-8")
    includes = re.findall(r'^#include\s+[<"]([^>"\n]+)', source + "\n" + header, re.M)
    assert set(includes) == {
        "drv_dshot_bitbang.h", "drv_dshot.h", "drv_dshot_telemetry.h",
        "stddef.h", "stdint.h",
    }
    assert "HAL_" not in source
    assert "htim" not in source
