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

import shutil
import struct
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
GOLDEN_DIR = ROOT / "tests" / "golden"
GOLDEN_FRAMES = GOLDEN_DIR / "telem_frames_v1.bin"

APP_INC = ROOT / "App" / "Inc"
APP_SRC = ROOT / "App" / "Src"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ------------------------------------------------------------------ 黄金向量


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
    """
    payload = struct.pack(
        "<BBHIIHHQ", 1, count, seq, schema, t_us, dt_us, flags, mask
    ) + b"".join(struct.pack("<f", value) for value in values)
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
                uint16_t dt_us, uint16_t flags, uint64_t mask,
                const float *values, uint32_t value_count)
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
    desc.mask = mask;

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

    /* --- 不变量：popcount / 长度公式 --- */
    if (APP_TelemFrame_PopCount(0ULL) != 0U) { return 10; }
    if (APP_TelemFrame_PopCount(0xFFFFFFFFFFFFFFFFULL) != 64U) { return 11; }
    if (APP_TelemFrame_PayloadLength(1U, 0ULL) != 0U) { return 12; }
    if (APP_TelemFrame_PayloadLength(0U, 1ULL) != 0U) { return 13; }
    if (APP_TelemFrame_PayloadLength(1U, 0x0FULL) != 24U + 16U) { return 14; }
    if (APP_TelemFrame_PayloadLength(2U, 0x03ULL) != 24U + 16U) { return 15; }

    /* --- 值个数与 mask 对不上必须整帧拒绝，不许截断 --- */
    {
        APP_TelemFrameDesc desc;
        uint8_t frame[64];
        uint16_t length = 0U;
        float values[3] = {1.0f, 2.0f, 3.0f};

        memset(&desc, 0, sizeof(desc));
        desc.count = 1U;
        desc.mask = 0x07ULL;
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
        desc.mask = 0xFFULL;   /* 8 路 -> payload 56 B */
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

        if (emit(1U, 0U, 0x11223344UL, 0UL, 0U, 0U, 0x0000000000000001ULL, a, 1U)) { return 40; }
        if (emit(1U, 7U, 0xDEADBEEFUL, 0x01020304UL, 0U, 0U, 0x0000000000000015ULL, b, 3U)) { return 41; }
        if (emit(1U, 65535U, 0x00000001UL, 0xFFFFFFFFUL, 0U, 1U, 0x000000000000000FULL, c, 4U)) { return 42; }
        if (emit(1U, 1U, 0xA5A5A5A5UL, 12345UL, 0U, 0U, 0x0000000008000000ULL, d, 1U)) { return 43; }
        if (emit(2U, 3U, 0x5A5A5A5AUL, 1000UL, 500U, 0U, 0x0000000000000003ULL, e, 4U)) { return 44; }
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
        mask = struct.unpack_from("<Q", payload, 16)[0]
        assert payload_length == 24 + 4 * count * bin(mask).count("1")
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

static uint64_t frame_mask(void)
{
    uint64_t mask = 0ULL;
    uint32_t i;
    /* payload 从 $X 头之后第 8 字节开始，mask 在 payload 偏移 16。 */
    for (i = 0U; i < 8U; ++i) {
        mask |= ((uint64_t)port_last_frame[8 + 16 + i]) << (8U * i);
    }
    return mask;
}

static uint16_t frame_flags(void)
{
    return (uint16_t)(port_last_frame[8 + 14] |
                      ((uint16_t)port_last_frame[8 + 15] << 8));
}

static uint64_t param_mask(void)
{
    uint64_t mask = 0ULL;
    uint32_t i;
    for (i = 0U; i < (uint32_t)APP_TELEM_CH_COUNT; ++i) {
        if (APP_Telemetry_ChannelHasParam(i) != 0U) { mask |= (1ULL << i); }
    }
    return mask;
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
    port_jf_frames = 0U;
    port_uart_fails = 0U;
    port_last_length = 0U;
    port_reply_count = 0U;
    APP_TelemStream_Reset();
}

/* --------------------------- 测试 ---------------------------------------- */

static int test_default_mask_and_steady_state_frame(void)
{
    uint64_t steady;

    reset_world();
    /* 默认掩码里必须有全部参数通道，否则全量刷新帧喂不出滑块初值。 */
    CHECK((APP_TelemStream_DefaultMask() & param_mask()) == param_mask(), 100);

    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    CHECK(APP_TelemStream_SetRefresh(0U) == APP_TELEM_STREAM_OK, 101);
    CHECK(APP_TelemStream_SetActive(1U) == APP_TELEM_STREAM_OK, 102);

    /* 首帧：影子还没建立，参数通道全部当成"变了"，所以带上。 */
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 1U, 103);
    CHECK((frame_mask() & param_mask()) == param_mask(), 104);

    /* 第二帧起参数不变就不再回显——这正是 JustFloat 每帧都在浪费的 2560 B/s。 */
    APP_TelemStream_Tick();
    CHECK(port_uart_frames == 2U, 105);
    steady = frame_mask();
    CHECK((steady & param_mask()) == 0ULL, 106);
    /* 12 路实时通道 -> 9 + 24 + 48 = 81 B。带宽表就按这个数算。 */
    CHECK(port_last_length == 81U, 107);
    return 0;
}

static int test_only_the_changed_parameter_is_echoed(void)
{
    uint64_t roll_bit = (1ULL << (uint32_t)APP_TELEM_CH_ROLL_RATE_KD);
    uint64_t pitch_bit = (1ULL << (uint32_t)APP_TELEM_CH_PITCH_RATE_KD);

    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetRefresh(0U);
    (void)APP_TelemStream_SetActive(1U);
    APP_TelemStream_Tick();   /* 首帧建影子 */
    APP_TelemStream_Tick();   /* 稳态帧 */
    CHECK((frame_mask() & param_mask()) == 0ULL, 200);

    port_values[APP_TELEM_CH_ROLL_RATE_KD] = 1.25f;
    APP_TelemStream_Tick();
    CHECK((frame_mask() & param_mask()) == roll_bit, 201);
    CHECK((frame_mask() & pitch_bit) == 0ULL, 202);

    /* 只回显一帧，下一帧就不再带了。 */
    APP_TelemStream_Tick();
    CHECK((frame_mask() & param_mask()) == 0ULL, 203);

    /* 值没变就不置位，哪怕重复写同一个数。 */
    port_values[APP_TELEM_CH_ROLL_RATE_KD] = 1.25f;
    APP_TelemStream_Tick();
    CHECK((frame_mask() & param_mask()) == 0ULL, 204);
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
    CHECK((frame_mask() & (1ULL << (uint32_t)APP_TELEM_CH_VEL_Z_KD)) != 0ULL, 250);
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
            CHECK(frame_flags() == 0U, 301);
        }
    }
    APP_TelemStream_Tick();
    CHECK(frame_flags() == APP_TELEM_FRAME_FLAG_FULL_REFRESH, 302);
    CHECK(frame_mask() == APP_TelemStream_DefaultMask(), 303);

    /* REFRESH 0 之后永远不再出现全量帧。 */
    reset_world();
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_UART);
    (void)APP_TelemStream_SetRefresh(0U);
    (void)APP_TelemStream_SetActive(1U);
    for (tick = 0U; tick < 200U; ++tick) {
        APP_TelemStream_Tick();
        CHECK(frame_flags() == 0U, 310);
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

    CHECK(APP_TelemStream_SetMask(0ULL) == APP_TELEM_STREAM_ERR_MASK, 704);
    /* 表外通道：拒绝，不做"忽略高位"的宽容处理。 */
    CHECK(APP_TelemStream_SetMask(1ULL << 63) == APP_TELEM_STREAM_ERR_MASK, 705);
    CHECK(APP_TelemStream_SetRefresh(61U) == APP_TELEM_STREAM_ERR_RANGE, 706);

    /* 出口装不下的配置：报 ERR，而且一帧都不发。 */
    port_max_payload = 40U;   /* 只放得下 4 路 */
    CHECK(APP_TelemStream_SetMask(0xFFULL) == APP_TELEM_STREAM_ERR_TOO_LARGE, 707);
    CHECK(APP_TelemStream_SetMask(0x0FULL) == APP_TELEM_STREAM_OK, 708);

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
        '(void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[APP_TELEM_CH_ROLL_RATE_KD]);',
        '(void)DRV_COAX_CTRL_GetParam("coax.yaw_angle_kp", &vofa_data[APP_TELEM_CH_YAW_ANGLE_KP]);',
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

    稳态 12 路 = 81 B/帧 × 40 Hz，1 Hz 的全量刷新帧替换当拍的稳态帧。
    """
    steady_bytes = 9 + 24 + 4 * 12
    refresh_bytes = 9 + 24 + 4 * 26
    assert steady_bytes == 81
    total = 39 * steady_bytes + refresh_bytes
    assert total == 3296
    assert total <= 0.60 * 5760
    # 对照：改造前的 JustFloat 定长帧。
    assert 116 * 40 == 4640
