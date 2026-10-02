"""测距跳变门（2026-10-01 自由飞）。

日志 `data/flight_logs/2026-10-01/receive_vjtq4rrv`（速度保持第二次飞行 12.4–12.6 s）：测距先爬 0.64→0.87 m，
停约 90 ms，再报 5.88 m 并被当成有效——旧门以 `height_valid` 为前提，而 `SVC_FlowNav_Age` 超 100 ms 没样本
就清它，尖峰正好落在空档里；定高推力瞬间掉到 9 N、机体沉约 12 cm，滤波高度 1 s 才回落（光流速度按高度放大）。
本文件在宿主上真编译 svc_flow_nav.c，钉住：空档后尖峰被拦；尖峰连报几帧同值也拦；真台阶稳住 ≥150 ms 才改认；
长时间（>1 s）没样本后重新起步。
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
static uint32_t ms = 1000U;
static void push(uint32_t mm, uint32_t step_ms) {
    SVC_FLOW_NAV_Sample s;
    memset(&s, 0, sizeof(s));
    ms += step_ms;
    s.frame_valid = 1U; s.distance_valid = 1U; s.flow_valid = 1U; s.distance_mm = mm;
    s.distance_received_ms = ms; s.flow_received_ms = ms; s.sensor_time_ms = ms;
    s.sample_interval_us = 10000U; s.flow_quality = 150U;
    (void)SVC_FlowNav_PushSample(&s, ms);
    SVC_FlowNav_Age(ms);
}
static void show(const char *tag) {
    SVC_FLOW_NAV_State st;
    SVC_FlowNav_GetState(&st);
    printf("%s %.3f %.3f %u\n", tag, st.height_m, st.height_raw_m, (unsigned)st.height_valid);
}
int main(void) {
    SVC_FlowNav_Init();
    for (int k = 0; k < 50; ++k) push(630U, 10U);
    show("steady");
    /* 实录：爬升后停 ~90 ms，再来一帧 5.88 m（空档期间 Age 已清 valid） */
    push(679U, 10U); push(774U, 10U); push(870U, 10U);
    SVC_FlowNav_Age(ms + 105U);
    push(5879U, 110U);
    show("spike");
    /* 尖峰连报 4 帧同值（共 30 ms）：仍拦 */
    push(5879U, 10U); push(5879U, 10U); push(5879U, 10U);
    show("spike_repeat");
    for (int k = 0; k < 10; ++k) push(640U, 10U);
    show("back");
    /* 真台阶 0.64 → 1.40 m（飞过桌沿反向），稳住：150 ms 后改认 */
    for (int k = 0; k < 10; ++k) push(1400U, 10U);
    show("step_100ms");
    for (int k = 0; k < 8; ++k) push(1400U, 10U);
    show("step_180ms");
    /* 长时间没样本（>1 s）后重新起步：直接接受 */
    push(300U, 1500U);
    show("restart");
    return 0;
}
"""


@pytest.fixture(scope="module")
def out(tmp_path_factory) -> dict[str, tuple[float, float, int]]:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    d = tmp_path_factory.mktemp("hgate")
    (d / "h.c").write_text(HARNESS, encoding="utf-8")
    exe = d / "h.exe"
    subprocess.run([gcc, "-O0", "-Wall", "-Werror", f"-I{ROOT / 'Services/Inc'}", f"-I{ROOT / 'Driver/Inc'}",
                    str(d / "h.c"), str(ROOT / "Services/Src/svc_flow_nav.c"),
                    str(ROOT / "Driver/Src/drv_nav_ekf.c"), "-lm", "-o", str(exe)],
                   check=True)
    lines = subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.split("\n")
    res = {}
    for ln in lines:
        if ln.strip():
            tag, h, raw, valid = ln.split()
            res[tag] = (float(h), float(raw), int(valid))
    return res


def test_spike_after_dropout_is_rejected(out):
    h, raw, _ = out["spike"]
    assert raw < 1.0 and h < 1.0


def test_repeated_spike_frames_are_still_rejected(out):
    h, raw, _ = out["spike_repeat"]
    assert raw < 1.0 and h < 1.0


def test_normal_samples_resume_after_spike(out):
    h, raw, valid = out["back"]
    assert raw == pytest.approx(0.64, abs=1e-3) and h == pytest.approx(0.64, abs=0.02) and valid == 1


def test_real_step_is_accepted_only_after_it_holds(out):
    assert out["step_100ms"][1] < 1.0
    h, raw, _ = out["step_180ms"]
    assert raw == pytest.approx(1.40, abs=1e-3) and h == pytest.approx(1.40, abs=1e-3)   # 重新起步，不经低通拖尾


def test_long_gap_restarts_without_gate(out):
    h, raw, valid = out["restart"]
    assert raw == pytest.approx(0.30, abs=1e-3) and valid == 1
