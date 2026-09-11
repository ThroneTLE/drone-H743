"""`drv_tx_ring` 的宿主侧单测，外加"它必须留在 PC 上可测的形态"这条约束。

这个环形队列是蓝牙出口从阻塞发送改成 DMA 发送时唯一会算错的部分：回绕、
满与空的区分、整包要么全进要么不进。这三件事一点硬件都不需要，所以它被单独
拆成一个不含 HAL、不含 RTOS 的 Driver 模块——否则只能靠烧录去验证索引算得对不对。

harness 用宿主 gcc **真编译固件那份 `drv_tx_ring.c`**，不是在 Python 里重写一遍
模型：重写的话验的是模型不是固件。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER_INC = ROOT / "Driver" / "Inc"
DRIVER_SRC = ROOT / "Driver" / "Src"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


RING_HARNESS = r"""
#include "drv_tx_ring.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

static uint8_t storage[16];
static DRV_TxRing ring;

static void reset_ring(void)
{
    memset(storage, 0, sizeof(storage));
    DRV_TxRing_Init(&ring, storage, sizeof(storage));
}

/* Drain the ring through the same two-step path the DMA engine uses. */
static uint32_t drain(uint8_t *out, uint32_t max)
{
    uint32_t total = 0U;

    for (;;) {
        const uint8_t *chunk = NULL;
        uint32_t length = DRV_TxRing_PeekContiguous(&ring, &chunk);

        if ((length == 0U) || (total + length > max)) { break; }
        memcpy(&out[total], chunk, length);
        total += length;
        DRV_TxRing_Release(&ring, length);
    }
    return total;
}

static int test_empty_ring_yields_nothing(void)
{
    const uint8_t *chunk = (const uint8_t *)0x1234;

    reset_ring();
    CHECK(DRV_TxRing_Used(&ring) == 0U, 100);
    CHECK(DRV_TxRing_Free(&ring) == sizeof(storage), 101);
    CHECK(DRV_TxRing_PeekContiguous(&ring, &chunk) == 0U, 102);
    return 0;
}

static int test_push_is_all_or_nothing(void)
{
    /*
     * 这条是整个模块存在的理由。出口上跑的是 $X 帧，写进去半帧会让上位机把
     * 后续字节当帧体解析，一路错到下一个偶然出现的 0x24 0x58——比丢掉整帧糟得多。
     */
    uint8_t big[20];

    reset_ring();
    memset(big, 0xAB, sizeof(big));
    CHECK(DRV_TxRing_Push(&ring, big, sizeof(big)) == 0U, 110);
    CHECK(DRV_TxRing_Used(&ring) == 0U, 111);     /* 一个字节都没写进去 */

    CHECK(DRV_TxRing_Push(&ring, big, 10U) == 1U, 112);
    CHECK(DRV_TxRing_Used(&ring) == 10U, 113);
    /* 剩 6 个位置，塞 7 个要整包拒绝，不能只写 6 个。 */
    CHECK(DRV_TxRing_Push(&ring, big, 7U) == 0U, 114);
    CHECK(DRV_TxRing_Used(&ring) == 10U, 115);
    /* 正好塞满是允许的——差一个字节的容量判据是最经典的错法。 */
    CHECK(DRV_TxRing_Push(&ring, big, 6U) == 1U, 116);
    CHECK(DRV_TxRing_Used(&ring) == sizeof(storage), 117);
    CHECK(DRV_TxRing_Free(&ring) == 0U, 118);
    return 0;
}

static int test_full_and_empty_are_distinguishable(void)
{
    uint8_t data[16];
    uint8_t out[16];
    const uint8_t *chunk = NULL;

    reset_ring();
    for (uint32_t i = 0U; i < sizeof(data); ++i) { data[i] = (uint8_t)i; }

    /* 写满：head == tail，但 used 说满。靠 head/tail 相等判空的实现会在这里翻车。 */
    CHECK(DRV_TxRing_Push(&ring, data, sizeof(data)) == 1U, 120);
    CHECK(DRV_TxRing_PeekContiguous(&ring, &chunk) == sizeof(data), 121);
    CHECK(drain(out, sizeof(out)) == sizeof(data), 122);
    CHECK(memcmp(out, data, sizeof(data)) == 0, 123);
    /* 排空之后 head == tail 仍然成立，这次必须判空。 */
    CHECK(DRV_TxRing_Used(&ring) == 0U, 124);
    CHECK(DRV_TxRing_PeekContiguous(&ring, &chunk) == 0U, 125);
    return 0;
}

static int test_a_wrapped_push_comes_back_in_order(void)
{
    /*
     * 回绕后一次只能给出连续的一段，剩下的下次再给——DMA 一次只搬一段连续内存。
     * 但两段拼起来必须是原始顺序，否则线上的字节流就乱了序。
     */
    uint8_t first[12] = {0};
    uint8_t data[10];
    uint8_t out[10];
    const uint8_t *chunk = NULL;

    reset_ring();
    CHECK(DRV_TxRing_Push(&ring, first, sizeof(first)) == 1U, 130);
    DRV_TxRing_Release(&ring, sizeof(first));    /* tail=12, head=12 */

    for (uint32_t i = 0U; i < sizeof(data); ++i) { data[i] = (uint8_t)(0x40U + i); }
    CHECK(DRV_TxRing_Push(&ring, data, sizeof(data)) == 1U, 131);
    CHECK(DRV_TxRing_Used(&ring) == sizeof(data), 132);

    /* 第一段只到缓冲末尾为止：12..15，共 4 字节。 */
    CHECK(DRV_TxRing_PeekContiguous(&ring, &chunk) == 4U, 133);
    CHECK(memcmp(chunk, data, 4U) == 0, 134);

    CHECK(drain(out, sizeof(out)) == sizeof(data), 135);
    CHECK(memcmp(out, data, sizeof(data)) == 0, 136);
    return 0;
}

static int test_release_never_walks_past_what_was_queued(void)
{
    /*
     * 发送出错时上层会把"在途长度"退掉，而那个长度有可能比队列里剩下的还大
     * （例如出错与完成中断撞在一起）。截断掉，不能让 used 下溢成 40 亿。
     */
    uint8_t data[4] = {1, 2, 3, 4};

    reset_ring();
    CHECK(DRV_TxRing_Push(&ring, data, sizeof(data)) == 1U, 140);
    DRV_TxRing_Release(&ring, 99U);
    CHECK(DRV_TxRing_Used(&ring) == 0U, 141);
    CHECK(DRV_TxRing_Free(&ring) == sizeof(storage), 142);
    return 0;
}

static int test_a_ring_without_storage_refuses_everything(void)
{
    /* 换板子时忘了给缓冲区，要安静地什么都不发，而不是往空指针上写。 */
    uint8_t data[4] = {1, 2, 3, 4};
    const uint8_t *chunk = NULL;

    DRV_TxRing_Init(&ring, NULL, 0U);
    CHECK(DRV_TxRing_Push(&ring, data, sizeof(data)) == 0U, 150);
    CHECK(DRV_TxRing_Used(&ring) == 0U, 151);
    CHECK(DRV_TxRing_Free(&ring) == 0U, 152);
    CHECK(DRV_TxRing_PeekContiguous(&ring, &chunk) == 0U, 153);
    DRV_TxRing_Release(&ring, 4U);   /* 不得崩 */
    return 0;
}

static int test_many_random_sized_frames_survive_a_round_trip(void)
{
    /*
     * 长跑：反复以不同长度写入并排空，覆盖每一种 head/tail 相对位置。
     * 回绕类缺陷往往只在某一个对齐关系下出现，固定长度的用例正好会漏掉。
     */
    uint8_t payload[9];
    uint8_t out[9];
    uint32_t seed = 1U;

    reset_ring();
    for (uint32_t round = 0U; round < 500U; ++round) {
        uint32_t length;

        seed = (seed * 1103515245U) + 12345U;
        length = 1U + ((seed >> 16) % sizeof(payload));
        for (uint32_t i = 0U; i < length; ++i) {
            payload[i] = (uint8_t)(round + i);
        }
        CHECK(DRV_TxRing_Push(&ring, payload, length) == 1U, 160);
        CHECK(drain(out, sizeof(out)) == length, 161);
        CHECK(memcmp(out, payload, length) == 0, 162);
        CHECK(DRV_TxRing_Used(&ring) == 0U, 163);
    }
    return 0;
}

int main(void)
{
    int rc;

    rc = test_empty_ring_yields_nothing(); if (rc) { return rc; }
    rc = test_push_is_all_or_nothing(); if (rc) { return rc; }
    rc = test_full_and_empty_are_distinguishable(); if (rc) { return rc; }
    rc = test_a_wrapped_push_comes_back_in_order(); if (rc) { return rc; }
    rc = test_release_never_walks_past_what_was_queued(); if (rc) { return rc; }
    rc = test_a_ring_without_storage_refuses_everything(); if (rc) { return rc; }
    rc = test_many_random_sized_frames_survive_a_round_trip(); if (rc) { return rc; }

    printf("tx ring harness ok\n");
    return 0;
}
"""


def test_tx_ring_on_host_gcc(tmp_path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "harness.c"
    # utf-8，与 test_telem_stream_contract.py 的 harness 一致：C 注释里有中文。
    harness.write_text(RING_HARNESS, encoding="utf-8")
    executable = tmp_path / "harness.exe"
    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{DRIVER_INC}",
            str(DRIVER_SRC / "drv_tx_ring.c"),
            str(harness), "-o", str(executable),
        ],
        check=True, capture_output=True, text=True,
    )
    result = subprocess.run([str(executable)], check=True, capture_output=True)
    assert "tx ring harness ok" in result.stdout.decode("ascii", "replace")


def test_the_ring_stays_free_of_hardware_and_rtos() -> None:
    """它一旦 include 了 HAL 或 RTOS，上面那个 harness 就编不过了。

    这条断言看着多余，其实是防止"顺手加个 __disable_irq 就好了"——那一行会把
    这个模块从可单测变成只能烧录验证，而索引算错正是最该被单测挡住的东西。
    锁由调用方（BSP）提供，头文件里写明了。
    """
    source = read("Driver/Src/drv_tx_ring.c")
    header = read("Driver/Inc/drv_tx_ring.h")

    for text in (source, header):
        assert "stm32" not in text.lower()
        assert "hal_" not in text.lower()
        assert "cmsis_os" not in text
        assert "__disable_irq" not in text
    assert "不是线程安全的" in header    # 锁的归属写在契约里，不靠口头约定
