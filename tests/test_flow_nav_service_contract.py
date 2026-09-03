"""R-M5-5：光流导航 Service（Services/Src/svc_flow_nav.c）契约测试。

分两部分：

  1. 行为部分用宿主 gcc 真编译 svc_flow_nav.c + drv_nav_ekf.c，跑一个 C 桩子，
     验证高度 LPF、质量门控、位移积分的边界。Service 不依赖 HAL（now_ms 一律由
     调用方传入），所以能整段搬到宿主上跑真实数值。
  2. 分层部分用源码级断言钉住"数学只在 Service 里"：Driver 不许长出滤波判决，
     App 不许留旧副本，稳定环不许再直接碰 DRV_NAV_EKF_*，控制律公式一字不改。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SERVICE_H = (ROOT / "Services" / "Inc" / "svc_flow_nav.h").read_text(encoding="utf-8")
SERVICE_C = (ROOT / "Services" / "Src" / "svc_flow_nav.c").read_text(encoding="utf-8")
FLOW_DRIVER = (ROOT / "Driver" / "Src" / "drv_optical_flow.c").read_text(encoding="utf-8")
FLOW_APP = (ROOT / "App" / "Src" / "app_optical_flow.c").read_text(encoding="utf-8")
FLOW_APP_H = (ROOT / "App" / "Inc" / "app_optical_flow.h").read_text(encoding="utf-8")
STABILIZER = (ROOT / "App" / "Src" / "app_stabilizer.c").read_text(encoding="utf-8")
COAX = (ROOT / "Driver" / "Src" / "drv_coax_ctrl.c").read_text(encoding="utf-8")
CMAKE = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")


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

#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

/*
 * host_ms 是 HAL_GetTick 那条时间轴（时效判定用），sensor_ms 是传感器自己
 * time_ms 那条（积分步长用）。测试里刻意让两条走不同的速率。
 */
static SVC_FLOW_NAV_Sample make_sample(uint32_t host_ms,
                                       uint32_t sensor_ms,
                                       uint32_t distance_mm,
                                       int16_t flow_vx,
                                       int16_t flow_vy,
                                       uint8_t quality,
                                       uint16_t sample_interval_us)
{
    SVC_FLOW_NAV_Sample sample;

    memset(&sample, 0, sizeof(sample));
    sample.frame_valid = 1U;
    sample.distance_valid = 1U;
    sample.flow_valid = 1U;
    sample.distance_mm = distance_mm;
    sample.distance_received_ms = host_ms;
    sample.flow_received_ms = host_ms;
    sample.sensor_time_ms = sensor_ms;
    sample.sample_interval_us = sample_interval_us;
    sample.flow_vel_x = flow_vx;
    sample.flow_vel_y = flow_vy;
    sample.flow_quality = quality;
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

/*
 * 稳定喂帧直到中值窗填满且速度有效。每帧推完立刻 Fuse 一次——真实系统里控制环
 * 跑得比传感器快得多，积分待处理量不会堆积，热身也必须照这个节奏来。
 */
static void warmup(uint32_t *host_ms, uint32_t *sensor_ms, uint32_t distance_mm)
{
    SVC_FLOW_NAV_Sample sample;

    for (uint32_t i = 0U; i < 5U; ++i) {
        *host_ms += 10U;
        *sensor_ms += 10U;
        sample = make_sample(*host_ms, *sensor_ms, distance_mm, 0, 0, 200U, 10000U);
        (void)SVC_FlowNav_PushSample(&sample, *host_ms);
        (void)fuse(0.0f, 0.0f, *host_ms, *host_ms);
    }
}

static int test_height_lpf_and_quality_gate(void)
{
    SVC_FLOW_NAV_Sample sample;
    SVC_FLOW_NAV_SampleResult result;
    SVC_FLOW_NAV_State state;
    uint32_t host_ms = 1000U;
    uint32_t sensor_ms = 500U;

    SVC_FlowNav_Init();

    /* 首帧：高度直接取原值，不做 LPF；中值窗还没满，出不了速度。 */
    host_ms += 10U; sensor_ms += 10U;
    sample = make_sample(host_ms, sensor_ms, 1000U, 50, 0, 200U, 10000U);
    result = SVC_FlowNav_PushSample(&sample, host_ms);
    CHECK(result == SVC_FLOW_NAV_SAMPLE_WARMUP, 10);
    SVC_FlowNav_GetState(&state);
    CHECK(state.height_valid == 1U, 11);
    CHECK(NEAR(state.height_m, 1.000f, 1e-6f), 12);
    CHECK(NEAR(state.height_raw_m, 1.000f, 1e-6f), 13);
    CHECK(state.velocity_valid == 0U, 14);

    /* 第二帧抬高 100mm：alpha=0.35 的一阶 LPF，1.000 + 0.35*0.100 = 1.035 */
    host_ms += 10U; sensor_ms += 10U;
    sample = make_sample(host_ms, sensor_ms, 1100U, 50, 0, 200U, 10000U);
    result = SVC_FlowNav_PushSample(&sample, host_ms);
    CHECK(result == SVC_FLOW_NAV_SAMPLE_WARMUP, 20);
    SVC_FlowNav_GetState(&state);
    CHECK(NEAR(state.height_m, 1.035f, 1e-5f), 21);
    CHECK(NEAR(state.height_filter_alpha, 0.35f, 1e-6f), 22);

    /* 第三帧：中值窗满 3 个，速度出得来 = 中值 * 0.01 * height */
    host_ms += 10U; sensor_ms += 10U;
    sample = make_sample(host_ms, sensor_ms, 1100U, 50, 0, 200U, 10000U);
    result = SVC_FlowNav_PushSample(&sample, host_ms);
    CHECK(result == SVC_FLOW_NAV_SAMPLE_ACCEPTED, 30);
    SVC_FlowNav_GetState(&state);
    CHECK(state.velocity_valid == 1U, 31);
    CHECK(state.flow_filter_ready == 1U, 32);
    CHECK(state.flow_vel_x_filtered == 50, 33);
    CHECK(NEAR(state.vx_m_s, 50.0f * 0.01f * state.height_m, 1e-5f), 34);

    /* 高度步长门：一帧跳 300mm > 0.18m，整帧丢弃，高度保持不变。 */
    {
        float held_height_m = state.height_m;
        host_ms += 10U; sensor_ms += 10U;
        sample = make_sample(host_ms, sensor_ms, 1400U, 50, 0, 200U, 10000U);
        (void)SVC_FlowNav_PushSample(&sample, host_ms);
        SVC_FlowNav_GetState(&state);
        CHECK(NEAR(state.height_m, held_height_m, 1e-6f), 40);
    }

    /* 质量门：79 拒、80 收（边界就是 SVC_FLOW_NAV_MIN_QUALITY 本身）。 */
    {
        uint32_t rejects_before;

        SVC_FlowNav_GetState(&state);
        rejects_before = state.velocity_reject_count;

        host_ms += 10U; sensor_ms += 10U;
        sample = make_sample(host_ms, sensor_ms, 1100U, 50, 0,
                             SVC_FLOW_NAV_MIN_QUALITY - 1U, 10000U);
        result = SVC_FlowNav_PushSample(&sample, host_ms);
        CHECK(result == SVC_FLOW_NAV_SAMPLE_REJECTED, 50);
        SVC_FlowNav_GetState(&state);
        CHECK(state.velocity_reject_count == rejects_before + 1U, 51);
        CHECK(state.velocity_valid == 0U, 52);

        host_ms += 10U; sensor_ms += 10U;
        sample = make_sample(host_ms, sensor_ms, 1100U, 50, 0,
                             SVC_FLOW_NAV_MIN_QUALITY, 10000U);
        result = SVC_FlowNav_PushSample(&sample, host_ms);
        CHECK(result == SVC_FLOW_NAV_SAMPLE_ACCEPTED, 53);
    }

    /* 超时老化：host 时间跳过 TIMEOUT，高度与速度一起作废。 */
    SVC_FlowNav_Age(host_ms + SVC_FLOW_NAV_TIMEOUT_MS + 1U);
    SVC_FlowNav_GetState(&state);
    CHECK(state.height_valid == 0U, 60);
    CHECK(state.velocity_valid == 0U, 61);

    return 0;
}

static int test_displacement_uses_the_sensor_timebase(void)
{
    SVC_FLOW_NAV_Sample sample;
    uint32_t host_ms = 1000U;
    uint32_t sensor_ms = 20000U;
    float px = 0.0f;
    float py = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;

    uint32_t steps0;

    SVC_FlowNav_Init();
    warmup(&host_ms, &sensor_ms, 1000U);
    SVC_FlowNav_ResetPosition();
    steps0 = SVC_FlowNav_GetIntegratedStepCount();

    /* 两条时间轴分开：host 每帧 +10ms，传感器每帧 +20ms（丢了一帧）。 */
    host_ms += 10U;
    sensor_ms += 20U;
    sample = make_sample(host_ms, sensor_ms, 1000U, 20, 0, 200U, 20000U);
    CHECK(SVC_FlowNav_PushSample(&sample, host_ms) ==
          SVC_FLOW_NAV_SAMPLE_ACCEPTED, 100);
    CHECK(fuse(0.2f, 0.0f, host_ms, host_ms) == 1U, 101);
    /* 步长必须是传感器时间轴的 20ms，不是 host 的 10ms，也不是固定 10ms。 */
    CHECK(SVC_FlowNav_GetLastIntegrationDtUs() == 20000UL, 102);
    CHECK(SVC_FlowNav_GetIntegratedStepCount() == steps0 + 1UL, 103);

    /* 位移必须精确等于"融合后速度 × 该样本跨过的传感器时长"。 */
    SVC_FlowNav_GetVelocity(&vx, &vy);
    SVC_FlowNav_GetPosition(&px, &py);
    CHECK(NEAR(px, vx * 0.020f, 1e-6f), 104);
    CHECK(vx > 0.0f, 105);

    /* 再来一帧，这次传感器只走 10ms：步长必须跟着变，不能沿用上一次。 */
    {
        float prev_px = px;
        float dt_sec;

        host_ms += 10U;
        sensor_ms += 10U;
        sample = make_sample(host_ms, sensor_ms, 1000U, 20, 0, 200U, 10000U);
        CHECK(SVC_FlowNav_PushSample(&sample, host_ms) ==
              SVC_FLOW_NAV_SAMPLE_ACCEPTED, 110);
        CHECK(fuse(0.2f, 0.0f, host_ms, host_ms) == 1U, 111);
        CHECK(SVC_FlowNav_GetLastIntegrationDtUs() == 10000UL, 112);
        dt_sec = 0.010f;
        SVC_FlowNav_GetVelocity(&vx, &vy);
        SVC_FlowNav_GetPosition(&px, &py);
        CHECK(NEAR(px - prev_px, vx * dt_sec, 1e-6f), 113);
    }

    /* 累计位移与位置同源：未限幅那一份必须等于位置（还没到限幅）。 */
    {
        float dx = 0.0f;
        float dy = 0.0f;

        SVC_FlowNav_GetDisplacement(&dx, &dy);
        CHECK(NEAR(dx, px, 1e-6f), 120);
        CHECK(NEAR(dy, py, 1e-6f), 121);
    }

    return 0;
}

static int test_gaps_and_invalid_samples_do_not_extrapolate(void)
{
    SVC_FLOW_NAV_Sample sample;
    uint32_t host_ms = 1000U;
    uint32_t sensor_ms = 20000U;
    uint32_t steps_before;
    float px = 0.0f;
    float py = 0.0f;

    SVC_FlowNav_Init();
    warmup(&host_ms, &sensor_ms, 1000U);
    SVC_FlowNav_ResetPosition();

    host_ms += 10U; sensor_ms += 10U;
    sample = make_sample(host_ms, sensor_ms, 1000U, 20, 0, 200U, 10000U);
    (void)SVC_FlowNav_PushSample(&sample, host_ms);
    (void)fuse(0.2f, 0.0f, host_ms, host_ms);
    steps_before = SVC_FlowNav_GetIntegratedStepCount();
    SVC_FlowNav_GetPosition(&px, &py);

    /* 传感器时间轴上跨了 200ms（远超 60ms 上限）：只重新落锚，不外推。 */
    {
        float gap_px = 0.0f;
        float gap_py = 0.0f;

        host_ms += 10U;
        sensor_ms += 200U;
        sample = make_sample(host_ms, sensor_ms, 1000U, 20, 0, 200U, 10000U);
        CHECK(SVC_FlowNav_PushSample(&sample, host_ms) ==
              SVC_FLOW_NAV_SAMPLE_ACCEPTED, 200);
        (void)fuse(0.2f, 0.0f, host_ms, host_ms);
        CHECK(SVC_FlowNav_GetIntegratedStepCount() == steps_before, 201);
        SVC_FlowNav_GetPosition(&gap_px, &gap_py);
        CHECK(NEAR(gap_px, px, 1e-6f), 202);
        px = gap_px;
    }

    /* 质量掉到门限下：该帧被拒，它跨过的时间一律不计入位移。 */
    {
        float rejected_px = 0.0f;
        float rejected_py = 0.0f;

        steps_before = SVC_FlowNav_GetIntegratedStepCount();
        host_ms += 10U; sensor_ms += 10U;
        sample = make_sample(host_ms, sensor_ms, 1000U, 20, 0, 10U, 10000U);
        CHECK(SVC_FlowNav_PushSample(&sample, host_ms) ==
              SVC_FLOW_NAV_SAMPLE_REJECTED, 210);
        (void)fuse(0.2f, 0.0f, host_ms, host_ms);
        CHECK(SVC_FlowNav_GetIntegratedStepCount() == steps_before, 211);
        SVC_FlowNav_GetPosition(&rejected_px, &rejected_py);
        CHECK(NEAR(rejected_px, px, 1e-6f), 212);
    }

    /* 恢复后的第一帧只重新落锚，不能把无效那段时间补积回来。 */
    {
        host_ms += 10U; sensor_ms += 10U;
        sample = make_sample(host_ms, sensor_ms, 1000U, 20, 0, 200U, 10000U);
        CHECK(SVC_FlowNav_PushSample(&sample, host_ms) ==
              SVC_FLOW_NAV_SAMPLE_ACCEPTED, 220);
        (void)fuse(0.2f, 0.0f, host_ms, host_ms);
        CHECK(SVC_FlowNav_GetIntegratedStepCount() == steps_before, 221);
    }

    return 0;
}

static int test_position_clamp_and_reset(void)
{
    SVC_FLOW_NAV_Sample sample;
    uint32_t host_ms = 1000U;
    uint32_t sensor_ms = 20000U;
    float px = 0.0f;
    float py = 0.0f;
    float dx = 0.0f;
    float dy = 0.0f;

    SVC_FlowNav_Init();
    warmup(&host_ms, &sensor_ms, 1000U);
    SVC_FlowNav_ResetPosition();

    /* 一路正向走，位置必须被限幅住，累计位移不受限幅约束。 */
    for (uint32_t i = 0U; i < 4000U; ++i) {
        host_ms += 10U;
        sensor_ms += 10U;
        sample = make_sample(host_ms, sensor_ms, 1000U, 25, 0, 200U, 10000U);
        (void)SVC_FlowNav_PushSample(&sample, host_ms);
        (void)fuse(0.25f, 0.0f, host_ms, host_ms);
    }
    SVC_FlowNav_GetPosition(&px, &py);
    SVC_FlowNav_GetDisplacement(&dx, &dy);
    CHECK(px <= SVC_FLOW_NAV_POSITION_LIMIT_M + 1e-4f, 300);
    CHECK(NEAR(px, SVC_FLOW_NAV_POSITION_LIMIT_M, 1e-3f), 301);
    CHECK(dx > SVC_FLOW_NAV_POSITION_LIMIT_M * 2.0f, 302);

    /* 归零只清位置与累计位移。 */
    SVC_FlowNav_ResetPosition();
    SVC_FlowNav_GetPosition(&px, &py);
    SVC_FlowNav_GetDisplacement(&dx, &dy);
    CHECK(NEAR(px, 0.0f, 1e-9f) && NEAR(py, 0.0f, 1e-9f), 310);
    CHECK(NEAR(dx, 0.0f, 1e-9f) && NEAR(dy, 0.0f, 1e-9f), 311);

    /* 估计器整体归零后速度也必须是 0。 */
    SVC_FlowNav_ResetEstimator();
    {
        float vx = 1.0f;
        float vy = 1.0f;

        SVC_FlowNav_GetVelocity(&vx, &vy);
        CHECK(NEAR(vx, 0.0f, 1e-9f) && NEAR(vy, 0.0f, 1e-9f), 320);
    }

    return 0;
}

static int test_stale_flow_holds_then_expires(void)
{
    SVC_FLOW_NAV_Sample sample;
    SVC_FLOW_NAV_State state;
    uint32_t host_ms = 1000U;
    uint32_t sensor_ms = 20000U;

    SVC_FlowNav_Init();
    warmup(&host_ms, &sensor_ms, 1000U);

    /* 同一帧再喂一次：不是新样本，仍在有效期内 → HOLD。 */
    sample = make_sample(host_ms, sensor_ms, 1000U, 0, 0, 200U, 10000U);
    CHECK(SVC_FlowNav_PushSample(&sample, host_ms + 5U) ==
          SVC_FLOW_NAV_SAMPLE_HOLD, 400);
    SVC_FlowNav_GetState(&state);
    CHECK(state.velocity_valid == 1U, 401);

    /* 过了有效期还没有新样本 → STALE，速度作废。 */
    CHECK(SVC_FlowNav_PushSample(&sample,
                                 host_ms + SVC_FLOW_NAV_TIMEOUT_MS + 5U) ==
          SVC_FLOW_NAV_SAMPLE_STALE, 410);
    SVC_FlowNav_GetState(&state);
    CHECK(state.velocity_valid == 0U, 411);

    return 0;
}

int main(void)
{
    int rc;

    rc = test_height_lpf_and_quality_gate();
    if (rc != 0) { return rc; }
    rc = test_displacement_uses_the_sensor_timebase();
    if (rc != 0) { return rc; }
    rc = test_gaps_and_invalid_samples_do_not_extrapolate();
    if (rc != 0) { return rc; }
    rc = test_position_clamp_and_reset();
    if (rc != 0) { return rc; }
    rc = test_stale_flow_holds_then_expires();
    if (rc != 0) { return rc; }

    printf("svc_flow_nav harness ok\n");
    return 0;
}
"""


def test_flow_nav_service_behaviour_on_host_gcc(tmp_path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    harness_path = tmp_path / "svc_flow_nav_harness.c"
    harness_path.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "svc_flow_nav_harness.exe"

    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Services' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "Services" / "Src" / "svc_flow_nav.c"),
            str(ROOT / "Driver" / "Src" / "drv_nav_ekf.c"),
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
    assert "svc_flow_nav harness ok" in result.stdout


# ----------------------------------------------------------------- 分层


def test_driver_layer_stays_a_pure_frame_parser() -> None:
    """Driver 只取原始帧：不许长出滤波、质量判决、速度换算。"""
    for forbidden in (
        "LPF", "lpf_alpha", "MIN_QUALITY", "median", "MEDIAN",
        "height_m", "vx_m_s", "EKF",
    ):
        assert forbidden not in FLOW_DRIVER, forbidden
    # 采样间隔统计是"测量"不是"判决"，必须保留——本 REQ 的实机 Hz 证据靠它。
    assert "flow_sample_interval_us" in FLOW_DRIVER


def test_flow_math_has_exactly_one_owner() -> None:
    """LPF / 质量门 / 中值窗只允许在 Service 里有一份定义。"""
    for token in (
        "SVC_FLOW_NAV_HEIGHT_LPF_ALPHA",
        "SVC_FLOW_NAV_VELOCITY_LPF_ALPHA",
        "SVC_FLOW_NAV_MIN_QUALITY",
        "SVC_FLOW_NAV_MEDIAN_WINDOW",
    ):
        assert f"#define {token}" in SERVICE_H, token

    # App 层不许再留任何一份副本。
    for stale in (
        "APP_FLOW_HEIGHT_LPF_ALPHA", "APP_FLOW_VELOCITY_LPF_ALPHA",
        "APP_FLOW_MIN_QUALITY", "APP_FLOW_MEDIAN_WINDOW",
        "app_flow_median_i16", "app_flow_update_height",
        "app_flow_velocity_plausible", "app_flow_frame_usable",
    ):
        assert stale not in FLOW_APP, stale
    assert "APP_OPTICAL_FLOW_MIN_QUALITY" not in FLOW_APP_H
    # App 层退化为透传。
    for call in (
        "SVC_FlowNav_Init();", "SVC_FlowNav_Reset();",
        "SVC_FlowNav_PushSample(", "SVC_FlowNav_GetHeight(",
        "SVC_FlowNav_GetSensorVelocity(", "SVC_FlowNav_GetState(",
    ):
        assert call in FLOW_APP, call


def test_stabilizer_no_longer_owns_the_estimator() -> None:
    """稳定环里的 EKF 与位置积分整体移除，不留半套。"""
    assert "DRV_NAV_EKF" not in STABILIZER
    assert '#include "drv_nav_ekf.h"' not in STABILIZER
    for stale in (
        "stabilizer_velocity_estimator_step", "stabilizer_velocity_estimator_reset",
        "stabilizer_flow_noise_from_quality", "StabilizerVelocityEstimatorState",
        "ctx->velocity_state_x_m_s", "ctx->position_state_x_m",
        "STABILIZER_NAV_EKF_FLOW_NOISE_M_S", "STABILIZER_FLOW_ONLY_MAX_ACCEL_M_S2",
    ):
        assert stale not in STABILIZER, stale
    # 旋转补偿留在稳定环（吃陀螺仪、归 FLU seam 契约管），必须原样还在。
    assert "stabilizer_compensate_flow_rotation" in STABILIZER
    assert "stabilizer_flow_compensation_publish(" in STABILIZER


def test_every_consumer_is_rewired_to_the_service() -> None:
    """原子替换：四个下游全部接到新 Service，不允许读到陈旧值或零值。"""
    # 控制器输入：位置与速度
    assert "SVC_FlowNav_GetVelocity(&nav_vx_m_s, &nav_vy_m_s);" in STABILIZER
    assert "SVC_FlowNav_GetPosition(&position_state_x_m, &position_state_y_m);" in STABILIZER
    assert "frame->attitude.x_m = position_state_x_m;" in STABILIZER
    assert "frame->attitude.y_m = position_state_y_m;" in STABILIZER
    assert "frame->attitude.vx_m_s = velocity_control_x_m_s;" in STABILIZER
    assert "frame->attitude.vy_m_s = velocity_control_y_m_s;" in STABILIZER
    # 验收观测量
    assert "SVC_FlowNav_GetVelocity(&observation.nav_velocity_m_s[0]," in STABILIZER
    # VOFA 通道
    assert "ctx->vofa_debug.pos_est_m[0] = position_state_x_m;" in STABILIZER
    assert "ctx->vofa_debug.vel_est_m_s[0] = nav_vx_m_s;" in STABILIZER


def test_coax_control_law_is_untouched() -> None:
    """只换数据来源，P/D 公式与增益一字不动（不算解冻 S5）。"""
    for expression in (
        "coax_ctrl_params.pos_x_kp * (reference->x_m - attitude->x_m)",
        "coax_ctrl_params.pos_y_kp * (reference->y_m - attitude->y_m)",
        "coax_ctrl_params.vel_x_kd * (reference->vx_m_s - attitude->vx_m_s)",
        "coax_ctrl_params.vel_y_kd * (reference->vy_m_s - attitude->vy_m_s)",
    ):
        assert expression in COAX, expression
    assert "SVC_FlowNav" not in COAX, "控制器不该直接依赖 Service，只吃 frame->attitude"


def test_service_is_registered_in_the_build() -> None:
    assert "Services/Src/svc_flow_nav.c" in CMAKE
    assert "Services/Inc" in CMAKE


def test_displacement_integration_never_uses_a_fixed_step() -> None:
    """⑤ 的硬要求：步长必须来自传感器时间轴。"""
    assert "sample->sensor_time_ms -" in SERVICE_C
    assert "sample->sample_interval_us" in SERVICE_C
    assert "SVC_FLOW_NAV_MAX_INTEGRATION_DT_US" in SERVICE_C
    # 控制环节拍只准喂给 EKF predict，不准进位移积分。
    integrate = SERVICE_C[
        SERVICE_C.index("static void flow_nav_integrate_position(void)"):
        SERVICE_C.index("/* ------------------------------------------------------------------ */\n/* 公共 API")
    ]
    assert "dt_sec" in integrate
    assert "input->dt_sec" not in integrate
    assert "SVC_FLOW_NAV_DEFAULT_DT_SEC" not in integrate
