"""XY 磁航向在静态倾斜下重新对齐后不得慢摆（真实 Fusion + 真实 svc_mag_heading.c）。

2026-09-30 台架：原来把含大硬铁偏移的 Z 均值（FLU +563 mG）喂给 Fusion，Fusion 按 2.66° 静态倾斜
做倾斜补偿，把 Z 投影进航向；对齐只按水平 XY 算，于是每次重新对齐后 yaw 以约 93 s 的时间常数
慢摆 2~5°（幅度随对齐角按正弦变化，仿真峰值约 4.5°）。现在输出 Z=0，偏移应接近 0。
复算与全角度扫描：data/analysis/mag-fit/2026-09-30/magxy_tilt_alignment_sim.py。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HARNESS = r"""
#include "drv_attitude_fusion.h"
#include "svc_mag_heading.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned long long now_us;

static DRV_AttitudeFusionOutput tick(float gz, const float mag[3], unsigned char valid)
{
    DRV_AttitudeFusionInput in;
    DRV_AttitudeFusionOutput out;
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    now_us += 1000ULL;
    in.time_us = now_us;
    in.dt_s = 0.001f;
    in.gyroscope_dps[2] = gz;
    in.accelerometer_g[0] = 0.037f;   /* bench: about 2.66 degrees static tilt */
    in.accelerometer_g[1] = 0.028f;
    in.accelerometer_g[2] = 0.996f;
    if (mag != NULL) {
        memcpy(in.magnetometer_mgauss, mag, sizeof(in.magnetometer_mgauss));
    }
    in.magnetometer_valid = valid;
    (void)DRV_AttitudeFusion_Update(&in, &out);
    return out;
}

static float offset_after_alignment(float target_yaw)
{
    SVC_MAG_HeadingConfig cfg = {-3.7253387f, 135.43175f, 268.29245f, 1U, 0U, 1U, 1U};
    SVC_MAG_HeadingState st;
    DRV_AttitudeFusionOutput out;
    float field[3] = {0.0f, 0.0f, 0.0f};
    const float raw[3] = {51.0f, 464.0f, 563.0f};   /* bench raw FLU, mG */
    unsigned long long mag_us = 0ULL;
    float aligned;
    float d;
    int i;

    SVC_MAG_HeadingReset(&st);
    DRV_AttitudeFusion_InitForConvention(DRV_ATTITUDE_FUSION_CONVENTION_NWU);
    now_us = 0ULL;
    for (i = 0; i < 5000; ++i) {
        out = tick(0.0f, NULL, 0U);
    }
    while (fabsf(out.yaw_deg - target_yaw) > 0.05f) {
        out = tick((target_yaw > out.yaw_deg) ? 10.0f : -10.0f, NULL, 0U);
    }
    for (i = 0; i < 2000; ++i) {
        out = tick(0.0f, NULL, 0U);
    }
    aligned = out.yaw_deg;
    for (i = 0; i < 400000; ++i) {     /* 400 s: about 4 time constants */
        unsigned char valid = 0U;
        if (i % 50 == 0) {             /* 20 Hz fresh samples into a 1 kHz AHRS */
            SVC_MAG_HeadingInput s;
            memset(&s, 0, sizeof(s));
            mag_us += 50000ULL;
            s.timestamp_us = mag_us;
            memcpy(s.raw_flu_mgauss, raw, sizeof(raw));
            s.roll_deg = out.roll_deg;
            s.pitch_deg = out.pitch_deg;
            s.yaw_deg = out.yaw_deg;
            valid = (SVC_MAG_HeadingUpdate(&cfg, &st, &s, field) ==
                     SVC_MAG_HEADING_READY) ? 1U : 0U;
        }
        out = tick(0.0f, field, valid);
    }
    d = out.yaw_deg - aligned;
    return (d > 180.0f) ? d - 360.0f : ((d < -180.0f) ? d + 360.0f : d);
}

int main(void)
{
    printf("%.4f %.4f %.4f\n", offset_after_alignment(-120.0f),
           offset_after_alignment(30.0f), offset_after_alignment(60.0f));
    return 0;
}
"""


def test_realignment_under_static_tilt_does_not_swing_yaw(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc unavailable")
    harness = tmp_path / "magxy_tilt.c"
    executable = tmp_path / "magxy_tilt.exe"
    harness.write_text(HARNESS, encoding="ascii")
    command = [
        gcc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        f"-I{ROOT / 'Driver' / 'Inc'}",
        f"-I{ROOT / 'Services' / 'Inc'}",
        f"-I{ROOT / 'ThirdParty' / 'Fusion'}",
        str(ROOT / "Driver/Src/drv_attitude_fusion.c"),
        str(ROOT / "Driver/Src/drv_mag_calibration.c"),
        str(ROOT / "Services/Src/svc_mag_heading.c"),
        str(ROOT / "ThirdParty/Fusion/FusionAhrs.c"),
        str(harness), "-lm", "-o", str(executable),
    ]
    built = subprocess.run(command, capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr
    run = subprocess.run([str(executable)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    offsets = [float(value) for value in run.stdout.split()]
    # 旧写法在这三个对齐角上约 −4.5°、+4.2°、+4.4°。
    assert len(offsets) == 3
    assert all(abs(value) < 0.2 for value in offsets), offsets
