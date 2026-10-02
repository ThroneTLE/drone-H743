"""光流窗口滤波：去毛刺后取平均，不再直接输出中值（2026-10-01）。

FLOWCAP 逐帧抓取：MTF-02P 每 10 ms 只报整像素位移（20 计数步进），慢推时 63% 的帧是 0、中位数
为 0——5 点中值把速度往 0 取整，慢推积分只剩原始 73%（钢尺 330 mm：原始 271、中值 198）。
本文件在宿主上真编译 svc_flow_nav.c，钉住：整像素低速序列输出窗口均值；单帧大毛刺被剔除。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include <stdio.h>
#include <string.h>
#include "svc_flow_nav.h"
static SVC_FLOW_NAV_Sample mk(uint32_t ms, int16_t vx) {
    SVC_FLOW_NAV_Sample s;
    memset(&s, 0, sizeof(s));
    s.frame_valid = 1U; s.distance_valid = 1U; s.flow_valid = 1U; s.distance_mm = 500U;
    s.distance_received_ms = ms; s.flow_received_ms = ms; s.sensor_time_ms = ms;
    s.sample_interval_us = 10000U; s.flow_vel_x = vx; s.flow_vel_y = 0; s.flow_quality = 150U;
    return s;
}
int main(void) {
    /* 0,0,-20,0,-20 循环：均值 -8 计数；中值会是 0。 */
    static const int16_t slow[5] = {0, 0, -20, 0, -20};
    SVC_FLOW_NAV_State st;
    uint32_t ms = 1000U;
    SVC_FLOW_NAV_Sample s;
    SVC_FlowNav_Init();
    for (int k = 0; k < 40; ++k) {
        s = mk(ms, slow[k % 5]); (void)SVC_FlowNav_PushSample(&s, ms); ms += 10U;
    }
    SVC_FlowNav_GetState(&st);
    printf("%d ", (int)st.flow_vel_x_filtered);
    /* 稳定 +40 中夹一帧 +1000 毛刺：输出仍是 40。 */
    for (int k = 0; k < 10; ++k) {
        s = mk(ms, (k == 7) ? 1000 : 40); (void)SVC_FlowNav_PushSample(&s, ms); ms += 10U;
        if (k == 7) { SVC_FlowNav_GetState(&st); printf("%d ", (int)st.flow_vel_x_filtered); }
    }
    SVC_FlowNav_GetState(&st);
    printf("%d\n", (int)st.flow_vel_x_filtered);
    return 0;
}
"""


def test_window_outputs_despiked_mean_not_median(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    harness = tmp_path / "h.c"
    harness.write_text(HARNESS, encoding="utf-8")
    exe = tmp_path / "h.exe"
    subprocess.run([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                    f"-I{ROOT / 'Services' / 'Inc'}", f"-I{ROOT / 'Driver' / 'Inc'}",
                    str(ROOT / "Services" / "Src" / "svc_flow_nav.c"),
                    str(ROOT / "Driver" / "Src" / "drv_nav_ekf.c"), str(harness), "-o", str(exe), "-lm"],
                   check=True, capture_output=True, text=True)
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()
    assert out == ["-8", "40", "40"], out


def test_source_uses_gated_mean_for_both_axes() -> None:
    src = (ROOT / "Services/Src/svc_flow_nav.c").read_text(encoding="utf-8")
    update = src.split("static void flow_nav_update_median_filter")[1].split("\n}\n")[0]
    assert update.count("flow_nav_gated_mean_i16(") == 2
    assert "flow_nav_median_i16(flow_nav_ctx" not in update
    assert "#define SVC_FLOW_NAV_SPIKE_COUNTS             100" in (
        ROOT / "Services/Inc/svc_flow_nav.h").read_text(encoding="utf-8")
