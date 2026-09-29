"""电流块平均（Driver/Src/drv_current_filter.c）的契约。

宿主 gcc 真编译该模块。它是纯函数 + 调用方持有的状态，不含 HAL/RTOS/IO，
所以不需要任何替身。

最要紧的一条是 **NaN 不得被当成 0 安培**：上游用 NaN 表示"这一拍没有有效读数"，
把它混进均值会让"传感器坏了"表现为"电流变小了"——一个读数从 2 A 平滑降到 1 A
看起来像负载减轻，实际上是一半的采样已经失败了。
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "drv_current_filter.h"

#include <math.h>
#include <stdio.h>

#define CHECK(cond, code) do { if (!(cond)) { \
    fprintf(stderr, "check %d failed at line %d: %s\n", (code), __LINE__, #cond); \
    return (code); } } while (0)

#define NEAR(a, b) (fabsf((a) - (b)) < 1e-4f)

int main(void)
{
    DRV_CurrentFilter f;

    /* 1. 凑满一个块之前 valid 必须是 0：没有完整块时不许给出"均值"。 */
    DRV_CurrentFilter_Init(&f, 4U);
    CHECK(f.valid == 0U, 1);
    DRV_CurrentFilter_Push(&f, 1.0f);
    DRV_CurrentFilter_Push(&f, 2.0f);
    CHECK(f.valid == 0U, 2);
    CHECK(f.blocks == 0U, 3);

    /* 2. 凑满即出块：均值/最小/最大都要对，并且自动开始下一块。 */
    DRV_CurrentFilter_Push(&f, 3.0f);
    DRV_CurrentFilter_Push(&f, 6.0f);
    CHECK(f.valid == 1U, 10);
    CHECK(f.blocks == 1U, 11);
    CHECK(NEAR(f.mean_a, 3.0f), 12);   /* (1+2+3+6)/4 */
    CHECK(NEAR(f.min_a, 1.0f), 13);
    CHECK(NEAR(f.max_a, 6.0f), 14);
    CHECK(f.block_count == 0U, 15);

    /* 3. 第二块独立于第一块——块平均的意义就在于互不重叠。 */
    DRV_CurrentFilter_Push(&f, 10.0f);
    DRV_CurrentFilter_Push(&f, 10.0f);
    DRV_CurrentFilter_Push(&f, 10.0f);
    DRV_CurrentFilter_Push(&f, 10.0f);
    CHECK(f.blocks == 2U, 20);
    CHECK(NEAR(f.mean_a, 10.0f), 21);
    CHECK(NEAR(f.min_a, 10.0f), 22);
    CHECK(NEAR(f.max_a, 10.0f), 23);

    /* 4. **NaN 不得被当成 0**。这是本模块最重要的一条。
     *    四个 2.0 与两个 NaN：均值必须是 2.0（NaN 被拒），
     *    而不是 (2+2+2+2+0+0)/6 = 1.333。 */
    DRV_CurrentFilter_Init(&f, 4U);
    DRV_CurrentFilter_Push(&f, 2.0f);
    DRV_CurrentFilter_Push(&f, NAN);
    DRV_CurrentFilter_Push(&f, 2.0f);
    DRV_CurrentFilter_Push(&f, NAN);
    DRV_CurrentFilter_Push(&f, 2.0f);
    DRV_CurrentFilter_Push(&f, 2.0f);
    CHECK(f.blocks == 1U, 30);
    CHECK(NEAR(f.mean_a, 2.0f), 31);
    CHECK(f.rejected == 2U, 32);
    /* 被拒的样本不能顶替有效样本凑数：块长仍然是 4 个**有效**样本。 */
    CHECK(f.min_a > 1.9f, 33);

    /* 5. Inf 同样被拒（isfinite 而不是 isnan）。 */
    DRV_CurrentFilter_Init(&f, 2U);
    DRV_CurrentFilter_Push(&f, INFINITY);
    DRV_CurrentFilter_Push(&f, -INFINITY);
    CHECK(f.rejected == 2U, 40);
    CHECK(f.valid == 0U, 41);

    /* 6. 全是无效样本时永远不出块——不许因为"等太久了"就给个数出来。 */
    DRV_CurrentFilter_Init(&f, 3U);
    for (int i = 0; i < 100; ++i) { DRV_CurrentFilter_Push(&f, NAN); }
    CHECK(f.valid == 0U, 50);
    CHECK(f.blocks == 0U, 51);
    CHECK(f.rejected == 100U, 52);

    /* 7. 负电流（反向/回充）要能通过，不被当成非法值吞掉。 */
    DRV_CurrentFilter_Init(&f, 2U);
    DRV_CurrentFilter_Push(&f, -1.0f);
    DRV_CurrentFilter_Push(&f, 3.0f);
    CHECK(f.valid == 1U, 60);
    CHECK(NEAR(f.mean_a, 1.0f), 61);
    CHECK(NEAR(f.min_a, -1.0f), 62);
    CHECK(NEAR(f.max_a, 3.0f), 63);

    /* 8. Reset 让结果失效并丢掉半块：跨越配置变更的平均没有物理意义。 */
    DRV_CurrentFilter_Init(&f, 4U);
    DRV_CurrentFilter_Push(&f, 5.0f);
    DRV_CurrentFilter_Push(&f, 5.0f);
    DRV_CurrentFilter_Push(&f, 5.0f);
    DRV_CurrentFilter_Push(&f, 5.0f);
    CHECK(f.valid == 1U, 70);
    DRV_CurrentFilter_Push(&f, 9.0f);
    DRV_CurrentFilter_Reset(&f);
    CHECK(f.valid == 0U, 71);
    CHECK(f.block_count == 0U, 72);
    /* Reset 之后重新凑满，结果不能带上 Reset 之前那个 9.0。 */
    DRV_CurrentFilter_Push(&f, 1.0f);
    DRV_CurrentFilter_Push(&f, 1.0f);
    DRV_CurrentFilter_Push(&f, 1.0f);
    DRV_CurrentFilter_Push(&f, 1.0f);
    CHECK(NEAR(f.mean_a, 1.0f), 73);
    CHECK(NEAR(f.max_a, 1.0f), 74);

    /* 9. window=0 按 1 处理，不除零、不永远不出块。 */
    DRV_CurrentFilter_Init(&f, 0U);
    DRV_CurrentFilter_Push(&f, 7.0f);
    CHECK(f.valid == 1U, 80);
    CHECK(NEAR(f.mean_a, 7.0f), 81);

    /* 10. NULL 安全。 */
    DRV_CurrentFilter_Init(NULL, 4U);
    DRV_CurrentFilter_Push(NULL, 1.0f);
    DRV_CurrentFilter_Reset(NULL);

    return 0;
}
"""


def test_current_filter_contract(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "current_filter_harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "current_filter_harness.exe"

    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-o",
            str(executable),
            str(harness),
            str(ROOT / "Driver" / "Src" / "drv_current_filter.c"),
            f"-I{ROOT / 'Driver' / 'Inc'}",
            "-lm",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_the_app_layer_feeds_the_filter_and_publishes_it() -> None:
    """接线断言：滤波器必须真的被 20 ms 采样路径喂、结果必须真的进快照。

    纯逻辑模块单测全绿但没人调用它，是这类改动最常见的失败方式——
    读数照样是单点瞬时值，而所有测试都绿着。
    """
    source = (ROOT / "App" / "Src" / "app_current.c").read_text(encoding="utf-8")
    assert "DRV_CurrentFilter_Push(&current_filter, reading.current_a)" in source
    assert "DRV_CurrentFilter_Init(&current_filter" in source
    assert "current_snapshot.mean_a = current_filter.mean_a" in source
    # 采样失败那一支也要记一笔，否则 rejected 看不出链路在掉样本。
    assert "DRV_CurrentFilter_Push(&current_filter, NAN)" in source


def test_the_reported_sensitivity_is_not_a_hardcoded_literal() -> None:
    """`nominal_mv_per_a` 必须报实际生效的灵敏度。

    它原本是写死的 `12.75` 字符串。2026-09-21 起这个系数要按实测标定改，
    写死的话诊断行会继续报旧值——和实现脱节的诊断字段比没有更坏。
    """
    source = (ROOT / "App" / "Src" / "app_current.c").read_text(encoding="utf-8")
    assert "nominal_mv_per_a=12.75\\r\\n" not in source
    assert "current_config.sensitivity_v_per_a" in source
