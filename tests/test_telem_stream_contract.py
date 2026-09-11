"""R-T1-1：遥测流 v2（固件侧）契约测试。

三部分：

  1. **黄金向量**：宿主 gcc 真编译 `app_telem_frame.c` + `app_proto.c`，让固件
     自己的编码器把一组给定的 mask/values 编出来，逐字节与本文件里写死的期望
     比对。这份期望同时是 R-T1-2 上位机解码器的输入（`tests/golden/`），两端
     不允许各写各的"看起来一样"。
  2. **策略**：同样在宿主上编译真实的 `app_telem_stream.c`，配一份 C 装置实现
     Port 层，验证脏位、全量刷新、出口选择、USB 拔出关流、导出互斥、超限 ERR。
     这些判定不能用 Python 重写一遍模型来"验证"——那验的是模型不是固件。
  3. **搬家**：`freertos.c` 里不许再留通道装配，`app_control.c` 不许变长。
"""

from __future__ import annotations

import re
import shutil
import struct
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
GOLDEN_DIR = ROOT / "tests" / "golden"
GOLDEN_FRAMES = GOLDEN_DIR / "telem_frames_v2.bin"

APP_INC = ROOT / "App" / "Inc"
APP_SRC = ROOT / "App" / "Src"
DRIVER_INC = ROOT / "Driver" / "Inc"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ------------------------------------------------------------------ 黄金向量


# 刻意在这里重新写一遍而不是从 panel_lib 里 import：黄金向量测的就是"两个
# 独立实现产出同一串字节"，共用常量会让两边一起写错时静默通过。
TELEM_FRAME_FLAG_WIDE_MASK = 0x0002


def proto_crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def expected_frame(
    *,
    count: int,
    seq: int,
    schema: int,
    t_us: int,
    dt_us: int,
    flags: int,
    mask: int,
    values: list[float],
) -> bytes:
    """规划文档 §2.2 的 payload 布局，在这里独立地重搭一遍。

    刻意不复用固件的任何代码：黄金向量的意义就是"两个独立实现产出同一串
    字节"。如果这里也去调固件的编码器，两边一起写错也测不出来。

    v2 的掩码是变长的，宽窄**由掩码内容唯一决定**（高 64 位非零即宽），
    调用方给的 WIDE_MASK 位一律丢弃。这条在这里也独立重写一遍，正是为了
    钉住"帧里写的宽度"与"实际写了几个字节"出自同一个判断。
    """
    wide = mask >> 64 != 0
    flags = flags & ~TELEM_FRAME_FLAG_WIDE_MASK
    if wide:
        flags |= TELEM_FRAME_FLAG_WIDE_MASK
    payload = struct.pack(
        "<BBHIIHHQ", 2, count, seq, schema, t_us, dt_us, flags, mask & ((1 << 64) - 1)
    )
    if wide:
        payload += struct.pack("<Q", mask >> 64)
    payload += b"".join(struct.pack("<f", value) for value in values)
    body = struct.pack("<BHH", 0, 0x2230, len(payload)) + payload
    return b"$X>" + body + bytes([proto_crc8_dvb_s2(body)])


# (count, seq, schema, t_us, dt_us, flags, mask, values)
GOLDEN_CASES = [
    # 单通道，最小帧。
    (1, 0, 0x11223344, 0, 0, 0, 0x0000000000000001, [1.5]),
    # 三个不相邻通道：值必须按通道索引升序排，不是按用户勾选顺序。
    (1, 7, 0xDEADBEEF, 0x01020304, 0, 0, 0x0000000000000015, [-1.0, 0.0, 2.5]),
    # 全量刷新帧：flags bit0 置位。
    (1, 65535, 0x00000001, 0xFFFFFFFF, 0, 1, 0x000000000000000F,
     [1.0, 2.0, 3.0, 4.0]),
    # 高位通道（bit 27）也要能编出来——mask 是 u64，不是 u32。
    (1, 1, 0xA5A5A5A5, 12345, 0, 0, 0x0000000008000000, [3.25]),
    # count>1 的批量帧（R-T2 才会用，格式在 R-T1 就钉死）。
    (2, 3, 0x5A5A5A5A, 1000, 500, 0, 0x0000000000000003,
     [1.0, 2.0, 3.0, 4.0]),
    # 边界：bit 63 仍是窄掩码（24 字节头）。差一位就变宽，所以两边都要钉。
    (1, 11, 0x0BADF00D, 777, 0, 0, 1 << 63, [6.5]),
    # bit 64：最小的宽掩码帧，头 32 字节，flags 自动带上 WIDE_MASK。
    (1, 12, 0x0BADF00D, 778, 0, 0, 1 << 64, [7.5]),
    # 跨越 64 位边界：低半与高半都有通道，值仍按全局通道索引升序排。
    (1, 13, 0xC0FFEE00, 779, 0, 1, (1 << 0) | (1 << 63) | (1 << 64) | (1 << 127),
     [1.0, 2.0, 3.0, 4.0]),
]


def build_golden_bytes() -> bytes:
    """把全部黄金向量串成一条字节流，供两端共用。"""
    blob = b""
    for count, seq, schema, t_us, dt_us, flags, mask, values in GOLDEN_CASES:
        blob += expected_frame(
            count=count, seq=seq, schema=schema, t_us=t_us, dt_us=dt_us,
            flags=flags, mask=mask, values=values,
        )
    return blob


ENCODER_HARNESS = r"""
#include "app_telem_frame.h"

#include <stdio.h>
#include <string.h>

/* 写到 argv[1] 指定的文件而不是 stdout：Windows 上 stdout 是文本模式，
   会把 0x0A 翻成 0x0D 0x0A，黄金向量当场就被改写了。 */
static FILE *out_file;

static int emit(uint8_t count, uint16_t seq, uint32_t schema, uint32_t t_us,
                uint16_t dt_us, uint16_t flags, uint64_t mask_lo,
                uint64_t mask_hi, const float *values, uint32_t value_count)
{
    APP_TelemFrameDesc desc;
    uint8_t frame[APP_TELEM_FRAME_OVERHEAD + APP_TELEM_FRAME_MAX_PAYLOAD];
    uint16_t length = 0U;

    desc.count = count;
    desc.seq = seq;
    desc.schema = schema;
    desc.t_us = t_us;
    desc.dt_us = dt_us;
    desc.flags = flags;
    desc.mask.lo = mask_lo;
    desc.mask.hi = mask_hi;

    if (APP_TelemFrame_Encode(&desc, values, value_count,
                              APP_TELEM_FRAME_MAX_PAYLOAD, frame,
                              (uint16_t)sizeof(frame), &length)
        != APP_TELEM_FRAME_OK) {
        return 1;
    }
    if (fwrite(frame, 1U, length, out_file) != length) {
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) { return 2; }
    out_file = fopen(argv[1], "wb");
    if (out_file == NULL) { return 3; }

    /* --- 不变量：popcount / 头长 / 长度公式 --- */
    {
        APP_TelemMask m_zero = {0ULL, 0ULL};
        APP_TelemMask m_lo_full = {0xFFFFFFFFFFFFFFFFULL, 0ULL};
        APP_TelemMask m_all = {0xFFFFFFFFFFFFFFFFULL, 0xFFFFFFFFFFFFFFFFULL};
        APP_TelemMask m_bit0 = {1ULL, 0ULL};
        APP_TelemMask m_lo4 = {0x0FULL, 0ULL};
        APP_TelemMask m_lo2 = {0x03ULL, 0ULL};
        APP_TelemMask m_bit63 = {1ULL << 63, 0ULL};
        APP_TelemMask m_bit64 = {0ULL, 1ULL};

        if (APP_TelemFrame_PopCount(m_zero) != 0U) { return 10; }
        if (APP_TelemFrame_PopCount(m_lo_full) != 64U) { return 11; }
        if (APP_TelemFrame_PopCount(m_all) != 128U) { return 16; }
        if (APP_TelemFrame_PayloadLength(1U, m_zero) != 0U) { return 12; }
        if (APP_TelemFrame_PayloadLength(0U, m_bit0) != 0U) { return 13; }
        if (APP_TelemFrame_PayloadLength(1U, m_lo4) != 24U + 16U) { return 14; }
        if (APP_TelemFrame_PayloadLength(2U, m_lo2) != 24U + 16U) { return 15; }
        /* 宽窄只看高半：bit63 仍窄，bit64 立刻变宽。 */
        if (APP_TelemFrame_HeaderBytes(m_bit63) != 24U) { return 17; }
        if (APP_TelemFrame_HeaderBytes(m_bit64) != 32U) { return 18; }
        if (APP_TelemFrame_PayloadLength(1U, m_bit64) != 32U + 4U) { return 19; }
        /* 位读写在 64 位边界两侧都要成立。 */
        if (APP_TelemMask_Test(m_bit63, 63U) == 0U) { return 50; }
        if (APP_TelemMask_Test(m_bit63, 64U) != 0U) { return 51; }
        if (APP_TelemMask_Test(m_bit64, 64U) == 0U) { return 52; }
        if (APP_TelemMask_Test(m_bit64, 63U) != 0U) { return 53; }
        if (APP_TelemMask_IsWide(m_bit63) != 0U) { return 54; }
        if (APP_TelemMask_IsWide(m_bit64) == 0U) { return 55; }
        if (APP_TelemMask_IsEmpty(APP_TelemMask_AndNot(m_bit64, m_all)) == 0U) { return 56; }
    }

    /* --- 值个数与 mask 对不上必须整帧拒绝，不许截断 --- */
    {
        APP_TelemFrameDesc desc;
        uint8_t frame[64];
        uint16_t length = 0U;
        float values[3] = {1.0f, 2.0f, 3.0f};

        memset(&desc, 0, sizeof(desc));
        desc.count = 1U;
        desc.mask.lo = 0x07ULL;
        if (APP_TelemFrame_Encode(&desc, values, 2U, 256U, frame,
                                  (uint16_t)sizeof(frame), &length)
            != APP_TELEM_FRAME_ERR_ARGS) {
            return 20;
        }
        if (length != 0U) { return 21; }
    }

    /* --- 超出出口上限必须报 TOO_LARGE，不许截断 --- */
    {
        APP_TelemFrameDesc desc;
        uint8_t frame[APP_TELEM_FRAME_OVERHEAD + APP_TELEM_FRAME_MAX_PAYLOAD];
        uint16_t length = 0U;
        float values[8];
        uint32_t i;

        for (i = 0U; i < 8U; ++i) { values[i] = (float)i; }
        memset(&desc, 0, sizeof(desc));
        desc.count = 1U;
        desc.mask.lo = 0xFFULL;   /* 8 路 -> payload 56 B */
        if (APP_TelemFrame_Encode(&desc, values, 8U, 55U, frame,
                                  (uint16_t)sizeof(frame), &length)
            != APP_TELEM_FRAME_ERR_TOO_LARGE) {
            return 30;
        }
        if (APP_TelemFrame_Encode(&desc, values, 8U, 56U, frame,
                                  (uint16_t)sizeof(frame), &length)
            != APP_TELEM_FRAME_OK) {
            return 31;
        }
    }

    /* --- 黄金向量：顺序与 tests/test_telem_stream_contract.py 的 GOLDEN_CASES 一致 --- */
    {
        float a[1] = {1.5f};
        float b[3] = {-1.0f, 0.0f, 2.5f};
        float c[4] = {1.0f, 2.0f, 3.0f, 4.0f};
        float d[1] = {3.25f};
        float e[4] = {1.0f, 2.0f, 3.0f, 4.0f};
        float f[1] = {6.5f};
        float g[1] = {7.5f};
        float h[4] = {1.0f, 2.0f, 3.0f, 4.0f};

        if (emit(1U, 0U, 0x11223344UL, 0UL, 0U, 0U, 0x0000000000000001ULL, 0ULL, a, 1U)) { return 40; }
        if (emit(1U, 7U, 0xDEADBEEFUL, 0x01020304UL, 0U, 0U, 0x0000000000000015ULL, 0ULL, b, 3U)) { return 41; }
        if (emit(1U, 65535U, 0x00000001UL, 0xFFFFFFFFUL, 0U, 1U, 0x000000000000000FULL, 0ULL, c, 4U)) { return 42; }
        if (emit(1U, 1U, 0xA5A5A5A5UL, 12345UL, 0U, 0U, 0x0000000008000000ULL, 0ULL, d, 1U)) { return 43; }
        if (emit(2U, 3U, 0x5A5A5A5AUL, 1000UL, 500U, 0U, 0x0000000000000003ULL, 0ULL, e, 4U)) { return 44; }
        if (emit(1U, 11U, 0x0BADF00DUL, 777UL, 0U, 0U, 1ULL << 63, 0ULL, f, 1U)) { return 45; }
        if (emit(1U, 12U, 0x0BADF00DUL, 778UL, 0U, 0U, 0ULL, 1ULL, g, 1U)) { return 46; }
        if (emit(1U, 13U, 0xC0FFEE00UL, 779UL, 0U, 1U,
                 1ULL | (1ULL << 63), 1ULL | (1ULL << 63), h, 4U)) { return 47; }
    }

    (void)fclose(out_file);
    return 0;
}
"""


def compile_harness(tmp_path: Path, harness: str, sources: list[Path]) -> Path:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness_path = tmp_path / "harness.c"
    harness_path.write_text(harness, encoding="utf-8")
    executable = tmp_path / "harness.exe"

    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{APP_INC}",
            f"-I{DRIVER_INC}",
            *[str(source) for source in sources],
            str(harness_path), "-o", str(executable),
        ],
        check=True, capture_output=True, text=True,
    )
    return executable


def compile_and_run(tmp_path: Path, harness: str, sources: list[Path]) -> str:
    executable = compile_harness(tmp_path, harness, sources)
    result = subprocess.run([str(executable)], check=True, capture_output=True)
    return result.stdout.decode("utf-8")


@pytest.fixture(scope="module")
def encoder_output(tmp_path_factory) -> bytes:
    tmp_path = tmp_path_factory.mktemp("telem_frame")
    executable = compile_harness(
        tmp_path,
        ENCODER_HARNESS,
        [APP_SRC / "app_telem_frame.c", APP_SRC / "app_proto.c"],
    )
    blob_path = tmp_path / "frames.bin"
    subprocess.run(
        [str(executable), str(blob_path)], check=True, capture_output=True
    )
    return blob_path.read_bytes()


def test_encoder_matches_the_golden_vectors_byte_for_byte(encoder_output: bytes) -> None:
    assert encoder_output == build_golden_bytes()


def test_golden_vector_file_is_in_sync_with_the_firmware(encoder_output: bytes) -> None:
    """落盘的黄金向量就是固件跑出来的字节，R-T1-2 的解码器直接吃这份文件。"""
    assert GOLDEN_FRAMES.exists(), f"missing golden vectors: {GOLDEN_FRAMES}"
    assert GOLDEN_FRAMES.read_bytes() == encoder_output


def test_golden_frames_are_self_describing() -> None:
    """帧自己就说得清"这一段是谁"：长度由 popcount 推出来并与 $X 的 len 一致。"""
    blob = build_golden_bytes()
    offset = 0
    seen = 0
    while offset < len(blob):
        assert blob[offset:offset + 2] == b"$X"
        payload_length = blob[offset + 6] | (blob[offset + 7] << 8)
        payload = blob[offset + 8:offset + 8 + payload_length]
        count = payload[1]
        flags = struct.unpack_from("<H", payload, 14)[0]
        # 头长要能只从 flags 推出来：接收端在读掩码之前还不知道数据从哪儿开始，
        # 只能靠这一位。用长度反推宽度就成了循环论证，正是这里要排除的。
        wide = bool(flags & TELEM_FRAME_FLAG_WIDE_MASK)
        header = 32 if wide else 24
        mask = struct.unpack_from("<Q", payload, 16)[0]
        if wide:
            mask |= struct.unpack_from("<Q", payload, 24)[0] << 64
        assert payload_length == header + 4 * count * bin(mask).count("1")
        # 宽窄由内容决定：高半为空却发了宽掩码，就是白白多占 8 字节。
        assert wide == (mask >> 64 != 0)
        offset += 9 + payload_length
        seen += 1
    assert seen == len(GOLDEN_CASES)


# ------------------------------------------------------------------ 流策略


STREAM_HARNESS = r"""
#include "app_telem_stream.h"
#include "app_telemetry.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

/* ------------------ Port 层装置：目标板那份在 app_telem_port.c ------------- */

static uint32_t port_now_us;
static uint8_t  port_exports_busy;
static uint8_t  port_usb_ready = 1U;
static uint8_t  port_sample_available = 1U;
static float    port_values[APP_TELEM_CH_COUNT];
static uint16_t port_max_payload = 247U;

static uint32_t port_uart_frames;
static uint32_t port_usb_frames;
static uint32_t port_bt_frames;
/*
 * 每帧发送要花多久（微秒）。蓝牙那条出口是阻塞的 UART 写，84 字节 / 115200
 * 大约 7.3 ms —— 这段时间**真实存在**，装置必须能模拟，否则测不出
 * "实际周期 = 睡眠 + 干活" 这个缺陷。
 */
static uint32_t port_send_cost_us;
static uint32_t port_jf_frames;
static uint8_t  port_uart_fails;
static uint8_t  port_last_frame[512];
static uint16_t port_last_length;
static char     port_last_reply[256];
static uint32_t port_reply_count;

uint32_t APP_TelemStream_PortNowUs(void) { return port_now_us; }

void APP_TelemStream_PortDelayMs(uint32_t ms)
{
    port_now_us += (ms * 1000U);
}

uint8_t APP_TelemStream_PortServiceExports(void) { return port_exports_busy; }
uint8_t APP_TelemStream_PortUsbReady(void) { return port_usb_ready; }

uint8_t APP_TelemStream_PortSample(float *values, uint32_t count)
{
    if (port_sample_available == 0U) { return 0U; }
    memcpy(values, port_values, count * sizeof(float));
    return 1U;
}

static void port_capture(const uint8_t *frame, uint16_t length)
{
    port_last_length = (length < sizeof(port_last_frame))
                           ? length : (uint16_t)sizeof(port_last_frame);
    memcpy(port_last_frame, frame, port_last_length);
}

uint8_t APP_TelemStream_PortSendUart(const uint8_t *frame, uint16_t length)
{
    if (port_uart_fails != 0U) { return 0U; }
    port_capture(frame, length);
    port_uart_frames++;
    return 1U;
}

uint8_t APP_TelemStream_PortSendUsb(const uint8_t *frame, uint16_t length)
{
    port_capture(frame, length);
    port_usb_frames++;
    return 1U;
}

/*
 * 每帧发送要花多久（微秒）。蓝牙那条出口是阻塞的 UART 写，84 字节 / 115200
 * 大约 7.3 ms —— 这段时间是**真实存在**的，装置必须能模拟它，否则测不出
 * "周期 = 睡眠 + 干活" 这个缺陷。
 */
uint8_t APP_TelemStream_PortSendBt(const uint8_t *frame, uint16_t length)
{
    /* 板载蓝牙（UART8）出口。与数传同档的容量判据，见 PortMaxPayload。 */
    port_capture(frame, length);
    port_bt_frames++;
    port_now_us += port_send_cost_us;
    return 1U;
}

uint8_t APP_TelemStream_PortSendJustFloat(const float *values, uint32_t count)
{
    (void)values; (void)count;
    port_jf_frames++;
    return 1U;
}

uint16_t APP_TelemStream_PortMaxPayload(APP_TelemSink sink)
{
    (void)sink;
    return port_max_payload;
}

void APP_TelemStream_PortReply(const char *text)
{
    port_reply_count++;
    snprintf(port_last_reply, sizeof(port_last_reply), "%s", text);
}

/* app_telemetry.c 的回包出口，本装置只需要它能链接。 */
void APP_Control_QueueText(const char *format, ...) { (void)format; }

/* ------------------------------ 工具 ------------------------------------- */

static uint16_t frame_flags(void);

static APP_TelemMask frame_mask(void)
{
    APP_TelemMask mask = {0ULL, 0ULL};
    uint32_t i;
    /* payload 从 $X 头之后第 8 字节开始，mask 在 payload 偏移 16。 */
    for (i = 0U; i < 8U; ++i) {
        mask.lo |= ((uint64_t)port_last_frame[8 + 16 + i]) << (8U * i);
    }
    /* 宽掩码的高半紧跟其后；窄帧里那 8 个字节是数据，绝不能当掩码读。 */
    if ((frame_flags() & APP_TELEM_FRAME_FLAG_WIDE_MASK) != 0U) {
        for (i = 0U; i < 8U; ++i) {
            mask.hi |= ((uint64_t)port_last_frame[8 + 24 + i]) << (8U * i);
        }
    }
    return mask;
}

static uint16_t frame_flags(void)
{
    return (uint16_t)(port_last_frame[8 + 14] |
                      ((uint16_t)port_last_frame[8 + 15] << 8));
}

static APP_TelemMask param_mask(void)
{
    APP_TelemMask mask = {0ULL, 0ULL};
    uint32_t i;
    for (i = 0U; i < (uint32_t)APP_TELEM_CH_COUNT; ++i) {
        if (APP_Telemetry_ChannelHasParam(i) != 0U) {
            mask = APP_TelemMask_Or(mask, APP_TelemMask_FromBit(i));
        }
    }
    return mask;
}

static int mask_equal(APP_TelemMask a, APP_TelemMask b)
{
    return ((a.lo == b.lo) && (a.hi == b.hi)) ? 1 : 0;
}

static void reset_world(void)
{
    memset(port_values, 0, sizeof(port_values));
    port_now_us = 0U;
    port_exports_busy = 0U;
    port_usb_ready = 1U;
    port_sample_available = 1U;
    port_max_payload = 247U;
    port_uart_frames = 0U;
    port_usb_frames = 0U;
    port_bt_frames = 0U;
    port_send_cost_us = 0U;
    port_jf_frames = 0U;
    port_uart_fails = 0U;
    port_last_length = 0U;
    port_reply_count = 0U;
    APP_TelemStream_Reset();
}

/* --------------------------- 测试 ---------------------------------------- */

static int test_default_mask_and_steady_state_frame(void)
{
    APP_TelemMask steady;

    reset_world();
    /* 默认掩码里必须有全部参数通道，否则全量刷新帧喂不出滑块初值。 */
    CHECK(mask_equal(APP_TelemMask_And(APP_TelemStream_DefaultMask(), param_mask()),
                     param_mask()), 100);

    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    CHECK(APP_TelemStream_SetRefresh(0U) == APP_TELEM_STREAM_OK, 101);
    CHECK(APP_TelemStream_SetActive(1U) == APP_TELEM_STREAM_OK, 102);

    /* 首帧：影子还没建立，参数通道全部当成"变了"，所以带上。 */
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 1U, 103);
    CHECK(mask_equal(APP_TelemMask_And(frame_mask(), param_mask()), param_mask()), 104);

    /* 第二帧起参数不变就不再回显——这正是 JustFloat 每帧都在浪费的 2560 B/s。 */
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 2U, 105);
    steady = frame_mask();
    CHECK(APP_TelemMask_IsEmpty(APP_TelemMask_And(steady, param_mask())) != 0U, 106);
    /* 12 路实时通道 -> 9 + 24 + 48 = 81 B。带宽表就按这个数算。 */
    CHECK(port_last_length == 81U, 107);
    return 0;
}

static int test_only_the_changed_parameter_is_echoed(void)
{
    APP_TelemMask roll_bit = APP_TelemMask_FromBit((uint32_t)APP_TELEM_CH_RATE_ROLL_KP);
    APP_TelemMask pitch_bit = APP_TelemMask_FromBit((uint32_t)APP_TELEM_CH_RATE_PITCH_KP);

    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetRefresh(0U);
    (void)APP_TelemStream_SetActive(1U);
    APP_TelemStream_Tick();   /* 首帧建影子 */
    APP_TelemStream_Tick();   /* 稳态帧 */
    CHECK(APP_TelemMask_IsEmpty(APP_TelemMask_And(frame_mask(), param_mask())) != 0U, 200);

    port_values[APP_TELEM_CH_RATE_ROLL_KP] = 1.25f;
    APP_TelemStream_Tick();
    CHECK(mask_equal(APP_TelemMask_And(frame_mask(), param_mask()), roll_bit), 201);
    CHECK(APP_TelemMask_IsEmpty(APP_TelemMask_And(frame_mask(), pitch_bit)) != 0U, 202);

    /* 只回显一帧，下一帧就不再带了。 */
    APP_TelemStream_Tick();
    CHECK(APP_TelemMask_IsEmpty(APP_TelemMask_And(frame_mask(), param_mask())) != 0U, 203);

    /* 值没变就不置位，哪怕重复写同一个数。 */
    port_values[APP_TELEM_CH_RATE_ROLL_KP] = 1.25f;
    APP_TelemStream_Tick();
    CHECK(APP_TelemMask_IsEmpty(APP_TelemMask_And(frame_mask(), param_mask())) != 0U, 204);
    return 0;
}

static int test_send_failure_keeps_the_dirty_bit(void)
{
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetRefresh(0U);
    (void)APP_TelemStream_SetActive(1U);
    APP_TelemStream_Tick();
    APP_TelemStream_Tick();

    port_values[APP_TELEM_CH_VEL_Z_KD] = 9.0f;
    port_uart_fails = 1U;
    APP_TelemStream_Tick();      /* 发失败 */
    port_uart_fails = 0U;
    APP_TelemStream_Tick();      /* 必须补发 */
    CHECK(APP_TelemMask_Test(frame_mask(), (uint32_t)APP_TELEM_CH_VEL_Z_KD) != 0U, 250);
    return 0;
}

static int test_refresh_period(void)
{
    uint32_t tick;

    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    CHECK(APP_TelemStream_SetRefresh(1U) == APP_TELEM_STREAM_OK, 300);
    (void)APP_TelemStream_SetActive(1U);

    /* 40 Hz -> 25 ms/拍，1 s = 第 40 拍是全量帧。 */
    for (tick = 1U; tick <= 39U; ++tick) {
        APP_TelemStream_Tick();
        if (tick > 1U) {
            /*
             * 稳态帧必须两位全 0：FULL_REFRESH 不用说，WIDE_MASK 也必须是 0。
             * 实时通道全在低 64 位，稳态帧因此只发 8 字节掩码；一旦哪个高通道
             * 漏进了稳态掩码，每帧就白涨 8 B，40 Hz 下 312 B/s，默认配置会被
             * 顶过数传 60% 带宽门限——那种回归在波形上完全看不出来。
             */
            CHECK(frame_flags() == 0U, 301);
        }
    }
    APP_TelemStream_Tick();
    /* 全量帧带上真名增益（通道 >= 64），所以它是宽掩码帧。 */
    CHECK((frame_flags() & APP_TELEM_FRAME_FLAG_FULL_REFRESH) != 0U, 302);
    CHECK((frame_flags() & APP_TELEM_FRAME_FLAG_WIDE_MASK) != 0U, 304);
    CHECK(mask_equal(frame_mask(), APP_TelemStream_DefaultMask()), 303);

    /* REFRESH 0 之后永远不再出现全量帧。 */
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetRefresh(0U);
    (void)APP_TelemStream_SetActive(1U);
    for (tick = 0U; tick < 200U; ++tick) {
        APP_TelemStream_Tick();
        /*
         * 这里只断言 FULL_REFRESH，不再断言整个 flags 为 0：复位后的第一帧会把
         * 全部参数当脏值带上，其中含通道 >= 64，那一帧合法地是宽掩码帧。
         * 稳态帧的"两位全 0"由上面的 301 负责。
         */
        CHECK((frame_flags() & APP_TELEM_FRAME_FLAG_FULL_REFRESH) == 0U, 310);
    }
    return 0;
}

static int test_sink_selection(void)
{
    reset_world();

    /* auto：跟随 STREAM on 那条命令的来源链路。 */
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_USB);
    (void)APP_TelemStream_SetActive(1U);
    CHECK(APP_TelemStream_ActiveSink() == APP_TELEM_SINK_USB, 400);
    APP_TelemStream_Tick();
    CHECK(port_usb_frames == 1U, 401);
    CHECK(port_uart_frames == 0U, 402);

    /* 开流之后再有数传命令进来，不许把跑着的流甩过去。 */
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    APP_TelemStream_Tick();
    CHECK(port_usb_frames == 2U, 403);
    CHECK(port_uart_frames == 0U, 404);

    /* 显式 uart 覆盖 auto。 */
    CHECK(APP_TelemStream_SetSink(APP_TELEM_SINK_UART) == APP_TELEM_STREAM_OK, 405);
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 1U, 406);

    /* 反过来：从数传开的流走 uart。 */
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetActive(1U);
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 1U, 410);
    CHECK(port_usb_frames == 0U, 411);
    return 0;
}

static int test_usb_unplug_stops_the_stream(void)
{
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_USB);
    (void)APP_TelemStream_SetActive(1U);
    APP_TelemStream_Tick();
    CHECK(port_usb_frames == 1U, 500);

    port_usb_ready = 0U;
    APP_TelemStream_Tick();
    CHECK(port_usb_frames == 1U, 501);
    /* 不是"这一拍没发出去"，是链路没了：必须关流，别让上位机对着 stream=1 猜。 */
    CHECK(vofaStreamActive == 0U, 502);

    /* USB 不在时显式开流要当场拒绝，不能悄悄置上然后永远不发。 */
    CHECK(APP_TelemStream_SetActive(1U) == APP_TELEM_STREAM_ERR_SINK, 503);
    CHECK(vofaStreamActive == 0U, 504);
    return 0;
}

static int test_exports_own_the_link(void)
{
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetActive(1U);
    port_exports_busy = 1U;
    APP_TelemStream_Tick();
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 0U, 600);
    CHECK(port_usb_frames == 0U, 601);
    port_exports_busy = 0U;
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 1U, 602);
    return 0;
}

static int test_no_new_sample_means_no_frame(void)
{
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetActive(1U);
    port_sample_available = 0U;
    APP_TelemStream_Tick();
    /* 宁可不发，也不把上一拍的旧值再推一遍冒充新数据。 */
    CHECK(port_uart_frames == 0U, 650);
    return 0;
}

static int test_limits_are_rejected_not_truncated(void)
{
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);

    CHECK(APP_TelemStream_SetRate(0U) == APP_TELEM_STREAM_ERR_RANGE, 700);
    CHECK(APP_TelemStream_SetRate(41U) == APP_TELEM_STREAM_ERR_RANGE, 701);
    CHECK(APP_TelemStream_SetRate(40U) == APP_TELEM_STREAM_OK, 702);
    CHECK(APP_TelemStream_SetRate(1U) == APP_TELEM_STREAM_OK, 703);

    CHECK(APP_TelemStream_SetMask(APP_TelemMask_Zero()) == APP_TELEM_STREAM_ERR_MASK, 704);
    /* R-S5-1 fills the final u64 slot; bit 63 is now a valid channel. */
    CHECK(APP_TelemStream_SetMask(APP_TelemMask_FromBit(63U)) == APP_TELEM_STREAM_OK, 705);
    /* 高半通道（v2 宽掩码）同样必须被接受，且表外位仍要拒。 */
    CHECK(APP_TelemStream_SetMask(APP_TelemMask_FromBit(64U)) == APP_TELEM_STREAM_OK, 709);
    CHECK(APP_TelemStream_SetMask(APP_TelemMask_FromBit(127U))
          == APP_TELEM_STREAM_ERR_MASK, 710);
    CHECK(APP_TelemStream_SetRefresh(61U) == APP_TELEM_STREAM_ERR_RANGE, 706);

    /* 出口装不下的配置：报 ERR，而且一帧都不发。 */
    port_max_payload = 40U;   /* 只放得下 4 路 */
    {
        APP_TelemMask m8 = {0xFFULL, 0ULL};
        APP_TelemMask m4 = {0x0FULL, 0ULL};
        CHECK(APP_TelemStream_SetMask(m8) == APP_TELEM_STREAM_ERR_TOO_LARGE, 707);
        CHECK(APP_TelemStream_SetMask(m4) == APP_TELEM_STREAM_OK, 708);
    }

    /* 配置期漏掉的情况由发送期兜底：可观测（回一条 ERR + drop 计数）。 */
    (void)APP_TelemStream_SetActive(1U);
    port_max_payload = 20U;
    port_reply_count = 0U;
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 0U, 709);
    CHECK(port_reply_count == 1U, 710);
    CHECK(strstr(port_last_reply, "ERR telem frame too large") != NULL, 711);
    /* 但不许 40 Hz 逐帧刷屏，把命令回复挤出 uartTxQueue。 */
    APP_TelemStream_Tick();
    APP_TelemStream_Tick();
    CHECK(port_reply_count == 1U, 712);
    return 0;
}

static int test_justfloat_format_stays_available_for_synex(void)
{
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    CHECK(APP_TelemStream_SetFormat(APP_TELEM_FORMAT_JF) == APP_TELEM_STREAM_OK, 800);
    (void)APP_TelemStream_SetActive(1U);
    APP_TelemStream_Tick();
    CHECK(port_jf_frames == 1U, 801);
    CHECK(port_uart_frames == 0U, 802);

    /* jf 只有数传那条路认；USB 出口必须当场拒绝。 */
    CHECK(APP_TelemStream_SetSink(APP_TELEM_SINK_USB) == APP_TELEM_STREAM_ERR_SINK, 803);
    return 0;
}

static int test_stream_defaults_to_off_and_survives_reinit(void)
{
    reset_world();
    /* 流配置只存 RAM，上电一律关：调参用的流不该自己跑起来占满数传。 */
    CHECK(vofaStreamActive == 0U, 900);
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 0U, 901);
    CHECK(port_usb_frames == 0U, 902);
    return 0;
}

static int test_the_period_absorbs_the_send_time(void)
{
    uint32_t first_us;
    uint32_t last_us;
    uint32_t ticks = 20U;
    uint32_t i;
    uint32_t measured_us;

    /*
     * 缺陷：`PortDelayMs(period)` 打头、干活在后，实际周期 = 周期 + 干活时间。
     * USB 上干活只有几十微秒，看不出来；蓝牙那条出口是阻塞的 UART 写
     * （84 字节 / 115200 ≈ 7.3 ms），于是 40 Hz 设下去实测只跑出 28.3 Hz，
     * 而链路带宽才用了五分之一——卡的不是链路，是排程。
     *
     * 这里把发送开销做成 7.3 ms 喂进去，断言**实际周期仍等于标称周期**。
     * 没有这条，同样的回归下次还会被当成"蓝牙模块不行"。
     */
    reset_world();
    port_send_cost_us = 7300U;
    CHECK(APP_TelemStream_SetRate(40U) == APP_TELEM_STREAM_OK, 700);
    CHECK(APP_TelemStream_SetSink(APP_TELEM_SINK_BT) == APP_TELEM_STREAM_OK, 701);
    CHECK(APP_TelemStream_SetRefresh(0U) == APP_TELEM_STREAM_OK, 702);
    CHECK(APP_TelemStream_SetActive(1U) == APP_TELEM_STREAM_OK, 703);

    /* 先走一拍让基准对齐，再量后面若干拍。 */
    APP_TelemStream_Tick();
    first_us = port_now_us;
    for (i = 0U; i < ticks; ++i) {
        APP_TelemStream_Tick();
    }
    last_us = port_now_us;

    CHECK(port_bt_frames == (ticks + 1U), 704);

    measured_us = (last_us - first_us) / ticks;
    /*
     * 标称 25000 us。允许 ±1 ms 的取整余量（睡眠只能按毫秒下发）；
     * 修复前这里会是 32300 us，差得远不止余量。
     */
    CHECK(measured_us >= 24000U, 705);
    CHECK(measured_us <= 26000U, 706);

    port_send_cost_us = 0U;
    return 0;
}

int main(void)
{
    int rc;

    rc = test_default_mask_and_steady_state_frame(); if (rc) { return rc; }
    rc = test_only_the_changed_parameter_is_echoed(); if (rc) { return rc; }
    rc = test_send_failure_keeps_the_dirty_bit(); if (rc) { return rc; }
    rc = test_refresh_period(); if (rc) { return rc; }
    rc = test_sink_selection(); if (rc) { return rc; }
    rc = test_usb_unplug_stops_the_stream(); if (rc) { return rc; }
    rc = test_exports_own_the_link(); if (rc) { return rc; }
    rc = test_no_new_sample_means_no_frame(); if (rc) { return rc; }
    rc = test_limits_are_rejected_not_truncated(); if (rc) { return rc; }
    rc = test_justfloat_format_stays_available_for_synex(); if (rc) { return rc; }
    rc = test_stream_defaults_to_off_and_survives_reinit(); if (rc) { return rc; }
    rc = test_the_period_absorbs_the_send_time(); if (rc) { return rc; }

    printf("telem stream harness ok\n");
    return 0;
}
"""


def test_stream_policy_on_host_gcc(tmp_path) -> None:
    output = compile_and_run(
        tmp_path,
        STREAM_HARNESS,
        [
            APP_SRC / "app_telem_stream.c",
            APP_SRC / "app_telem_frame.c",
            APP_SRC / "app_proto.c",
            APP_SRC / "app_telemetry.c",
        ],
    )
    assert "telem stream harness ok" in output


# ------------------------------------------------------------------ 搬家


def test_freertos_no_longer_assembles_telemetry_channels() -> None:
    """通道装配整段离开 CubeMX 文件；USER CODE 段只剩一行调用。"""
    source = read("Core/Src/freertos.c")
    task = source[source.index("void VOFA_task") :]

    assert "vofa_data[APP_TELEM_CH_" not in task
    assert "APP_TelemStream_Tick();" in task
    assert "volatile uint8_t vofaStreamActive" not in source
    assert "VOFA_DATA_SIZE" not in source
    assert "IMU_CAPTURE_EXPORT_BLOCKS_PER_TICK" not in source


def test_channel_assembly_moved_verbatim_into_the_port_module() -> None:
    """搬家不是重写：逐条赋值语句必须与搬家前一字不差。"""
    port = read("App/Src/app_telem_port.c")
    for statement in (
        "vofa_data[APP_TELEM_CH_ROLL] = msg.roll_deg;",
        "vofa_data[APP_TELEM_CH_TIME] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;",
        'vofa_data[APP_TELEM_CH_RESERVED_7] = 0.0f;',
        'vofa_data[APP_TELEM_CH_RESERVED_9] = 0.0f;',
        "vofa_data[APP_TELEM_CH_FUSION_ACC_NORM_REJECTED] = (float)msg.fusion_accel_norm_rejected;",
    ):
        assert statement in port, statement
    # 搬家后唯一允许的改动：删掉旧代码对四个角度增益的显示取反。遥测回显与
    # `PARAM?`（app_control_param_to_ui_value，现为恒等）必须同口径——审核实机
    # 复核见 test_coax_ctrl_contract.test_vofa_exports_compact_slider_parameter_feedback。
    assert "= -vofa_data[" not in port


def test_app_control_did_not_grow() -> None:
    """TELEM 命令族整族迁出，那个 4000 行的文件只许变短。"""
    control = read("App/Src/app_control.c")
    assert "static void app_control_handle_telem" not in control
    assert "app_control_handle_telem(tokens, count);" in control
    assert "app_control_handle_telem" in read("App/Src/app_cmd_telem.c")


def test_proto_builder_is_enabled_but_the_parser_is_not() -> None:
    header = read("App/Inc/app_proto.h")
    source = read("App/Src/app_proto.c")

    assert "#define APP_PROTO_MSG_TELEM_FRAME" in header
    assert "0x2230U" in header
    # 成帧器出了 #if 0，解析器仍在里面。
    assert source.index("uint8_t APP_Proto_BuildFrame(") < source.index(
        "#if 0 /* 解析器"
    )
    assert "APP_Proto_ConsumeByte" not in header.split("/* Parser side remains disabled")[0]


def test_function_id_is_registered_on_both_ends() -> None:
    """历史教训：0x1022/0x1023 曾被两端各自定义过一次。"""
    firmware = read("App/Inc/app_proto.h")
    assert "#define APP_PROTO_MSG_TELEM_FRAME" in firmware
    assert "0x2230U" in firmware
    assert "PROTO_MSG_TELEM_FRAME = 0x2230" in read("tools/panel_lib/proto.py")


def test_new_modules_are_registered_in_the_build() -> None:
    cmake = read("CMakeLists.txt")
    for source in (
        "App/Src/app_telem_frame.c",
        "App/Src/app_telem_port.c",
        "App/Src/app_telem_stream.c",
        "App/Src/app_cmd_telem.c",
    ):
        assert source in cmake, source


def test_stream_never_runs_in_the_control_loop_context() -> None:
    """USB 写是阻塞的；它只允许出现在遥测任务的 Port 实现里。"""
    assert "APP_USB_CDC_Write" not in read("App/Src/app_telem_stream.c")
    assert "APP_USB_CDC_Write" not in read("App/Src/app_vofa.c")
    assert "APP_USB_CDC_Write" in read("App/Src/app_telem_port.c")


# ------------------------------------------------------------------ 带宽


def test_default_configuration_fits_the_telemetry_link() -> None:
    """数传 57600 baud = 5760 B/s；类别模式规定默认配置不得超过 60%。

    通道数**从源码数出来**，不写死。写死的话，往通道表里加一条参数通道时这条
    门限会继续绿着，而实机上全量刷新帧已经变长了——那正是 v2 加 20 路真名增益
    时差点撞上的事（余量只剩 76 B/s）。

    两种帧：
      * 稳态帧 —— 只有实时通道，全部在低 64 位，所以是 24 字节窄掩码头；
      * 全量刷新帧 —— 每 refresh 秒一帧，带上全部参数通道，含通道 >= 64，
        所以是 32 字节宽掩码头，并且替换当拍的稳态帧（不是额外多发一帧）。
    """
    stream_src = read("App/Src/app_telem_stream.c")
    table_src = read("App/Src/app_telemetry.c")

    default_block = stream_src.split("app_telem_stream_default_channels[] = {", 1)[1]
    default_block = default_block.split("};", 1)[0]
    realtime = len(re.findall(r"APP_TELEM_CH_\w+", default_block))

    table_block = table_src.split("app_telem_channels[APP_TELEM_CH_COUNT] = {", 1)[1]
    table_block = table_block.split("\n};", 1)[0]
    params = len(re.findall(r'"(coax\.[\w.]+)"', table_block))

    assert realtime == 12
    assert params == 27, "27 current parameters; retired slots have no binding"

    steady_bytes = 9 + 24 + 4 * realtime
    refresh_bytes = 9 + 32 + 4 * (realtime + params)
    assert steady_bytes == 81
    assert refresh_bytes == 197

    total = 39 * steady_bytes + refresh_bytes
    assert total == 3356
    assert total <= 0.60 * 5760
    # 对照：改造前的 JustFloat 定长帧。
    assert 116 * 40 == 4640


def test_the_full_refresh_frame_still_fits_one_uart_message() -> None:
    """全量刷新帧要塞进一条 APP_UART_TxMessage，否则数传上整条流直接配不起来。

    UART 出口上限 = APP_UART_TX_TEXT_SIZE - $X 成帧开销。加参数通道最先撞的是
    这堵墙，不是带宽门限——SetMask 会返回 ERR_TOO_LARGE，症状是"面板一打开
    调参页，流就停了"。
    """
    messages = read("App/Inc/app_messages.h")
    tx_text_size = int(
        re.search(r"#define APP_UART_TX_TEXT_SIZE\s+(\d+)U", messages).group(1)
    )
    frame_header = read("App/Inc/app_telem_frame.h")
    overhead = int(
        re.search(r"#define APP_TELEM_FRAME_OVERHEAD\s+(\d+)U", frame_header).group(1)
    )

    max_payload = tx_text_size - overhead
    refresh_payload = 32 + 4 * (12 + 33)
    assert refresh_payload == 212
    assert refresh_payload <= max_payload, (
        f"全量刷新帧 {refresh_payload} B 放不进 UART 出口上限 {max_payload} B"
    )



def test_the_telemetry_task_is_not_starved_at_the_bottom_priority() -> None:
    """遥测任务必须在 BelowNormal，不能回到 Low。

    2026-09-11 实测：Low 比 messageTask / backgroundTask（都是 BelowNormal）
    还低，于是只有在它们和 1 kHz 的 Stabilizer 全都让出 CPU 时才轮得到——而这
    条件并不成立。`RTOS?` 报 TELEM state=1(Ready)，任务内第一条语句的计数器
    从上电起一直是 0：**一整拍都没跑过**，表现为 stream=1 却一个字节都不来。

    这种失败特别难查，因为每一层看起来都正常：命令回 OK、状态说 stream=1、
    链路也通。所以把优先级钉住，并要求 `.ioc` 与生成代码一致——只改一边的话
    下次 CubeMX 重新生成就会把它悄悄打回 Low。
    """
    freertos = (ROOT / "Core" / "Src" / "freertos.c").read_text(encoding="utf-8")
    ioc = (ROOT / "drone-H743.ioc").read_text(encoding="utf-8")

    attrs = freertos.split("VOFA_Task_attributes = {", 1)[1].split("};", 1)[0]
    assert ".priority = (osPriority_t) osPriorityBelowNormal," in attrs
    assert ".priority = (osPriority_t) osPriorityLow," not in attrs

    # .ioc 的任务表：优先级 16 = BelowNormal（Low 是 8）。
    assert "VOFA_Task,16,512,VOFA_task" in ioc
    assert "VOFA_Task,8,512,VOFA_task" not in ioc


def test_rtos_report_exposes_every_task_that_can_stall() -> None:
    """`RTOS?` 必须报出每个会卡住的任务，含调度状态。

    上面那个缺陷排查了很久，正是因为遥测任务既不在任务栈报告里，也没有调度
    状态可看——分不清它是在跑、阻塞着、还是根本没被调度，而这三种情况的
    下一步完全不同。
    """
    system = (ROOT / "App" / "Src" / "app_cmd_system.c").read_text(encoding="utf-8")

    for task in ("STABILIZER", "SENSOR", "MSG", "UART", "BACKGROUND", "TELEM"):
        assert f'app_control_report_task_stack("{task}"' in system, task
    assert "eTaskGetState((TaskHandle_t)handle)" in system
    assert "state=%u" in system
