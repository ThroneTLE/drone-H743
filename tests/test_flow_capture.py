"""光流逐帧抓取诊断 `FLOWCAP`（2026-10-01）。

钢尺实测 5 点中值滤波比原始帧积分少 7–20% 且随运动而变；要看原始帧分布才能定滤波怎么改。
本文件钉住：抓取是独立模块、只记录不改导航量、按收帧时刻去重、抓满即停、缓冲不进 DTCM。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from tools.panel_lib import ai_bridge_policy as policy

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


SVC = read("Services/Src/svc_flow_capture.c")
CMD = read("App/Src/app_cmd_flowcap.c")


def test_wiring_and_layering() -> None:
    assert "if (app_control_handle_flowcap(tokens, count) != 0U) { return; }" in read("App/Src/app_cmd_fallback.c")
    assert "uint8_t app_control_handle_flowcap(char **tokens, uint32_t count);" in read(
        "App/Inc/app_control_internal.h")
    cmake = read("CMakeLists.txt")
    assert "App/Src/app_cmd_flowcap.c" in cmake and "Services/Src/svc_flow_capture.c" in cmake
    assert "FLOWCAP" not in read("App/Src/app_control.c"), "新命令不进 app_control.c"
    # 挂在样本生成之后、推入导航之前，svc_flow_nav.c 不依赖抓帧模块。
    flow = read("App/Src/app_optical_flow.c")
    assert flow.index("SVC_FlowCapture_Push(&sample);") < flow.index("SVC_FlowNav_PushSample(&sample, now);")
    assert "svc_flow_capture" not in read("Services/Src/svc_flow_nav.c")
    assert "stm32" not in SVC.lower() and "hal_" not in SVC.lower()


def test_buffer_lives_in_axi_sram_not_dtcm() -> None:
    assert '__attribute__((section(".ram_d1_noinit"), aligned(32)))' in SVC
    assert "#if defined(__arm__)" in SVC


def test_dump_lines_fit_the_text_buffer() -> None:
    assert "char text[192];" in CMD
    assert "%f" not in CMD, "newlib-nano 没有浮点 printf"


def test_ring_mode_and_freeze_need_no_new_whitelist_words() -> None:
    # 环形与冻结都挂在 START/DUMP 下，作者不用为白名单再重启地面站。
    assert 'strcmp(tokens[2], "RING") == 0' in CMD
    dump = CMD.split('strcmp(tokens[1], "DUMP") == 0')[1]
    assert dump.index("SVC_FlowCapture_Stop();") < dump.index("flowcap_dump_line(next)")


def test_bridge_allows_capture_but_not_motion() -> None:
    for line in ("FLOWCAP START 500", "FLOWCAP START RING", "FLOWCAP DUMP 0", "FLOWCAP?"):
        assert policy.classify(line).kind == policy.ALLOW, line


HARNESS = r"""
#include <stdio.h>
#include "svc_flow_capture.h"
static SVC_FLOW_NAV_Sample mk(uint32_t rx, int16_t vx) {
    SVC_FLOW_NAV_Sample s = {0};
    s.flow_valid = 1; s.frame_valid = 1; s.distance_valid = 1; s.distance_mm = 478;
    s.flow_received_ms = rx; s.sensor_time_ms = rx + 1000; s.flow_vel_x = vx; s.flow_vel_y = -vx; s.flow_quality = 105;
    return s;
}
int main(void) {
    SVC_FlowCaptureFrame f;
    SVC_FLOW_NAV_Sample s = mk(10, 3);
    SVC_FlowCapture_Push(&s);                       /* 未 Arm：不记 */
    printf("%u ", SVC_FlowCapture_Count());
    printf("%u ", SVC_FlowCapture_Arm(3));
    SVC_FlowCapture_Push(&s); SVC_FlowCapture_Push(&s);   /* 同一帧推两次只记一次 */
    s = mk(20, 5); SVC_FlowCapture_Push(&s);
    s = mk(30, 7); SVC_FlowCapture_Push(&s);
    s = mk(40, 9); SVC_FlowCapture_Push(&s);        /* 已满：不记 */
    printf("%u %u ", SVC_FlowCapture_Count(), SVC_FlowCapture_Active());
    printf("%u ", SVC_FlowCapture_Get(2, &f)); printf("%d %d %u %u %u ", f.vel_x, f.vel_y, f.quality, f.flags, f.distance_mm);
    printf("%u ", SVC_FlowCapture_Get(3, &f));
    printf("%u ", SVC_FlowCapture_Arm(0));
    /* 环形：多推 10 帧绕圈，冻结后第 0 帧是最老的（第 11 帧），最后一帧是最新的。 */
    SVC_FlowCapture_ArmRing();
    for (uint32_t k = 1; k <= SVC_FLOW_CAPTURE_MAX_FRAMES + 10U; ++k) {
        s = mk(k * 10U, (int16_t)(k % 30000U)); SVC_FlowCapture_Push(&s);
    }
    SVC_FlowCapture_Stop();
    s = mk(999999U, 1); SVC_FlowCapture_Push(&s);   /* 冻结后不再记 */
    printf("%u %lu ", SVC_FlowCapture_Count(), (unsigned long)SVC_FlowCapture_Total());
    SVC_FlowCapture_Get(0, &f); printf("%d ", f.vel_x);
    SVC_FlowCapture_Get(SVC_FLOW_CAPTURE_MAX_FRAMES - 1U, &f); printf("%d\n", f.vel_x);
    return 0;
}
"""


def test_capture_behaviour_host_compiled(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    harness = tmp_path / "h.c"
    harness.write_text(HARNESS, encoding="utf-8")
    exe = tmp_path / "h.exe"
    subprocess.run([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                    f"-I{ROOT / 'Services' / 'Inc'}", f"-I{ROOT / 'Driver' / 'Inc'}",
                    str(ROOT / "Services" / "Src" / "svc_flow_capture.c"), str(harness), "-o", str(exe)],
                   check=True, capture_output=True, text=True)
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()
    assert out == ["0", "3", "3", "0", "1", "7", "-7", "105", "7", "478", "0", "4096",
                   "4096", "4106", "11", "4106"], out
