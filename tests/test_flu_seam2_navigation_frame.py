"""R-F6-1 seam 2 navigation frame contract.

Seam 2 (navigation) was the only one of the six FLU runtime seams with no
coordinate-system documentation and no test at all.  This module does not
change any numeric behaviour (author boundary for R-F6-1, same shape as
seam 1's option C): it only pins, for each public interface named in the
work order (Services/Inc/svc_flow_nav.h, Driver/Inc/drv_nav_ekf.h,
App/Inc/app_nav_estimator.h):

  * that the frame/unit/sign/timestamp documentation this task added is
    actually present (a doc-only change is easy to silently lose in a later
    edit without a test noticing),
  * that drv_nav_ekf.c and svc_flow_nav.c behave, on the real host-compiled
    code, exactly as documented: the EKF/Service layer passes each axis
    through independently with no cross-axis rotation and no driver-owned
    frame opinion,
  * that the documented units/scale are consistent with real recorded
    optical-flow + rangefinder evidence under data/ (not a self-authored
    input), per AGENTS.md's "algorithm conclusions must rest on recorded
    data" rule.

Do not read this module as evidence that the legacy body-right-positive Y
convention it pins is *correct* -- only that it is what the current code
does.  Fixing it is seam 3/4 (R-F6-2/R-F6-3), not this task.
"""

from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SVC_HEADER = ROOT / "Services" / "Inc" / "svc_flow_nav.h"
SVC_SOURCE = ROOT / "Services" / "Src" / "svc_flow_nav.c"
EKF_HEADER = ROOT / "Driver" / "Inc" / "drv_nav_ekf.h"
EKF_SOURCE = ROOT / "Driver" / "Src" / "drv_nav_ekf.c"
ESTIMATOR_HEADER = ROOT / "App" / "Inc" / "app_nav_estimator.h"
FLOW_RANGE_DIR = ROOT / "data" / "calibration" / "flow_range" / "2026-08-30"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


# --------------------------------------------------------------- documentation


def test_svc_flow_nav_header_names_its_frames_units_and_timebases() -> None:
    header = read(SVC_HEADER)
    assert "MicoLink 光流传感器原始定点角速率" in header
    assert "flow_vel_x / flow_vel_y" in header
    assert "0.01 rad/s" in header
    assert "sensor_time_ms" in header and "distance_received_ms" in header
    assert "不是同一条" in header  # the two timebases are named as distinct

    # FuseInput: nav-frame accel vs. body-frame flow velocity must be
    # distinguished, and the world frame must not be claimed as NED/ENU.
    assert "不得称为 NED 或 ENU" in header
    assert "机体系光流地速" in header
    assert "STABILIZER_VELOCITY_MEAS_Y_SIGN=+1" in header
    assert "不是规范 FLU 的左正" in header

    # State: height sign and vx/vy convention must be named explicitly.
    assert "不是带符号的导航系 Z 坐标" in header
    assert "EKF 本身不做任何额外旋转" in header

    # GetPosition/GetDisplacement axis convention.
    assert "未旋转到导航系" in header


def test_drv_nav_ekf_header_disclaims_frame_ownership() -> None:
    header = read(EKF_HEADER)
    assert "不知道、" in header or "不知道" in header
    assert "不做任何旋转或重映射" in header
    assert "svc_flow_nav.h" in header
    assert "drv_frame_contract.h" in header


def test_app_nav_estimator_header_points_to_the_service_convention() -> None:
    header = read(ESTIMATOR_HEADER)
    assert "不持有、也不重新定义坐标系" in header
    assert "机体系 X 前 / Y 右正" in header
    assert "legacy 约定" in header


# --------------------------------------------------------------- real recordings


def _iter_flow_range_samples():
    for path in sorted(FLOW_RANGE_DIR.glob("*.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        for stage_samples in payload["samples"].values():
            for sample in stage_samples:
                yield sample


def test_recorded_flow_range_evidence_matches_documented_units() -> None:
    """Sanity-check the documented units/scale against real ground evidence.

    This is not a sign derivation (the JSON records the telemetry-published
    FLU display values per app_telem_port's observation-boundary adapter, not
    the internal legacy representation this seam documents) -- only a check
    that "height in metres, velocity in metres/second, bounded by the
    Service's own plausibility gates" holds for real captured data rather
    than a self-authored fixture.
    """
    samples = list(_iter_flow_range_samples())
    assert len(samples) > 0, "expected real flow_range recordings under data/"

    heights = [s["height_m"] for s in samples if s.get("height_m") is not None]
    assert heights
    assert all(0.0 < h < 3.0 for h in heights), (
        "recorded height_m outside a plausible bench range in metres"
    )

    velocities = [
        s[key]
        for s in samples
        for key in ("vx_compensated_m_s", "vy_compensated_m_s")
        if s.get(key) is not None
    ]
    assert velocities
    # SVC_FLOW_NAV_MAX_SPEED_M_S caps the internal plausibility gate at 2.5
    # m/s; the telemetry-published value is a different (FLU-adapted) axis
    # but shares the same physical bench motion, so the same order-of-
    # magnitude bound applies with margin.
    assert all(abs(v) < 3.0 for v in velocities), (
        "recorded velocity magnitude inconsistent with documented m/s units"
    )


# --------------------------------------------------------------- host behaviour


HARNESS = r"""
#include "svc_flow_nav.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

static SVC_FLOW_NAV_Sample make_sample(uint32_t host_ms, uint32_t sensor_ms,
                                       int16_t flow_vx, int16_t flow_vy)
{
    SVC_FLOW_NAV_Sample sample;

    memset(&sample, 0, sizeof(sample));
    sample.frame_valid = 1U;
    sample.distance_valid = 1U;
    sample.flow_valid = 1U;
    sample.distance_mm = 1000U;
    sample.distance_received_ms = host_ms;
    sample.flow_received_ms = host_ms;
    sample.sensor_time_ms = sensor_ms;
    sample.sample_interval_us = 10000U;
    sample.flow_vel_x = flow_vx;
    sample.flow_vel_y = flow_vy;
    sample.flow_quality = 200U;
    return sample;
}

static uint8_t fuse(float vx_m_s, float vy_m_s, uint32_t flow_sample_ms,
                    uint32_t now_ms)
{
    SVC_FLOW_NAV_FuseInput in;

    memset(&in, 0, sizeof(in));
    in.flow_vx_m_s = vx_m_s;
    in.flow_vy_m_s = vy_m_s;
    in.flow_valid = 1U;
    in.flow_quality = 200U;
    in.flow_sample_ms = flow_sample_ms;
    in.dt_sec = 0.001f;
    in.now_ms = now_ms;
    return SVC_FlowNav_Fuse(&in);
}

static void warmup(uint32_t *host_ms, uint32_t *sensor_ms, int16_t flow_vx,
                   int16_t flow_vy)
{
    SVC_FLOW_NAV_Sample sample;
    uint32_t i;
    float sensor_vx = 0.0f;
    float sensor_vy = 0.0f;
    uint32_t sample_ms = 0U;

    for (i = 0U; i < 20U; ++i) {
        *host_ms += 10U;
        *sensor_ms += 10U;
        sample = make_sample(*host_ms, *sensor_ms, flow_vx, flow_vy);
        (void)SVC_FlowNav_PushSample(&sample, *host_ms);
        (void)SVC_FlowNav_GetSensorVelocity(&sensor_vx, &sensor_vy,
                                            &sample_ms, *host_ms);
        (void)fuse(sensor_vx, sensor_vy, *host_ms, *host_ms);
    }
}

/*
 * The frame-contract property this seam pins: the Service/EKF pipeline must
 * carry each axis independently.  A frame bug that quietly rotated or
 * swapped X/Y (exactly the class of bug the frame-migration mode's harness
 * warns about -- it cancels out in same-axis tests but not in physical
 * flight) would make an X-only stimulus leak into the Y output or vice versa.
 */
static int test_axes_are_not_rotated_or_swapped(void)
{
    uint32_t host_ms = 1000U;
    uint32_t sensor_ms = 1000U;
    float vx = 0.0f;
    float vy = 0.0f;

    SVC_FlowNav_Init();
    warmup(&host_ms, &sensor_ms, 40, 0);
    SVC_FlowNav_GetVelocity(&vx, &vy);
    CHECK(vx > 0.05f, 1);
    CHECK(fabsf(vy) < 1.0e-6f, 2);

    SVC_FlowNav_Init();
    warmup(&host_ms, &sensor_ms, 0, 40);
    SVC_FlowNav_GetVelocity(&vx, &vy);
    CHECK(fabsf(vx) < 1.0e-6f, 3);
    CHECK(vy > 0.05f, 4);

    return 0;
}

/*
 * Documented conversion: v = flow_rate * 0.01 * height_m.  Height here is
 * distance_mm * 0.001 (1000mm -> 1.0m), so 40 raw units at 1.0m must settle
 * to 0.40 m/s exactly.  This checks the median-filtered sensor velocity
 * (SVC_FlowNav_GetState/GetSensorVelocity), which is where the conversion
 * this seam documents actually happens; the separate EKF-fused output
 * (SVC_FlowNav_GetVelocity) converges toward the same value but only
 * asymptotically, so it is not the right place to pin an exact scale.
 */
static int test_documented_unit_conversion_holds(void)
{
    uint32_t host_ms = 2000U;
    uint32_t sensor_ms = 2000U;
    SVC_FLOW_NAV_State state;

    SVC_FlowNav_Init();
    warmup(&host_ms, &sensor_ms, 40, 0);
    SVC_FlowNav_GetState(&state);
    CHECK(fabsf(state.vx_m_s - 0.40f) < 1.0e-3f, 10);
    CHECK(fabsf(state.vy_m_s) < 1.0e-6f, 11);

    return 0;
}

int main(void)
{
    int rc;

    rc = test_axes_are_not_rotated_or_swapped();
    if (rc != 0) { return rc; }
    rc = test_documented_unit_conversion_holds();
    if (rc != 0) { return rc; }

    printf("seam2 nav frame harness ok\n");
    return 0;
}
"""


def test_navigation_service_does_not_rotate_or_swap_axes(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    harness_path = tmp_path / "seam2_nav_harness.c"
    harness_path.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "seam2_nav_harness.exe"

    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Services' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(SVC_SOURCE),
            str(EKF_SOURCE),
            str(harness_path),
            "-o",
            str(executable),
            "-lm",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run(
        [str(executable)], check=True, capture_output=True, text=True
    )
    assert "seam2 nav frame harness ok" in result.stdout
