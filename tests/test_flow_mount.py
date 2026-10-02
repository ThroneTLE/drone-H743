"""R-FLOWMOUNT-1：光流安装方向参数——固件换算、参数写入口、FLOW? 行、推荐算法。

固件部分用宿主 gcc 把**真实的** App/Src/app_optical_flow.c 与
Driver/Src/drv_airframe_params.c 编起来跑：BSP 只提供一帧可控的原始读数，
Service 的 PushSample 把 app_flow_fill_sample() 交出来的样本原样记下。所以这里验的
就是方言边界那一处真正的输出，而不是它的复制品。

期望值不从被测代码来：Python 侧按任务契约的换算表独立重写一遍（含 INT16_MIN
取负钳位），再与上位机推荐算法用的矩阵（ground_calibration.flow_mount_matrix）
互相核对——上位机算出的推荐必须与固件真正做的变换是同一件事。
"""

from __future__ import annotations

import itertools
import json
import math
import re
from pathlib import Path

import pytest

from _micoair_hostfakes import CHECK_MACRO, build_and_run, write_fakes
from tools.ground_calibration import (
    FLOW_MOUNT_YAW_CHOICES,
    GroundCalibrationError,
    describe_flow_mount,
    flow_mount_from_matrix,
    flow_mount_matrix,
    recommend_flow_mount,
)


ROOT = Path(__file__).resolve().parents[1]
FLOW_APP = ROOT / "App" / "Src" / "app_optical_flow.c"
INT16_MIN = -32768
INT16_MAX = 32767
MOUNTS = [(yaw, mirror) for mirror in (0, 1) for yaw in FLOW_MOUNT_YAW_CHOICES]

# 每种安装都喂这些原始读数：普通值、零、正负 1、以及两轴各自的 INT16 极值。
RAW_INPUTS = [
    (123, -45), (-7, 300), (0, 0), (1, -1), (-1, 1), (250, 250),
    (INT16_MIN, 5), (5, INT16_MIN), (INT16_MIN, INT16_MIN),
    (INT16_MAX, -9), (-9, INT16_MAX), (INT16_MAX, INT16_MAX),
    (INT16_MIN, INT16_MAX), (INT16_MAX, INT16_MIN),
]


HARNESS = CHECK_MACRO + r"""
#include "app_optical_flow.h"
#include "bsp_optical_flow.h"
#include "drv_airframe_params.h"
#include "svc_flow_nav.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

/* ---- 替身：BSP 给一帧可控读数，Service 记下交进来的样本 ---- */
static BSP_OPTICAL_FLOW_Status stub_status;
static SVC_FLOW_NAV_Sample last_sample;
static unsigned push_count;
static char text_lines[16][300];
static unsigned text_count;

BSP_OPTICAL_FLOW_StatusCode BSP_OPTICAL_FLOW_Init(void)
{
    stub_status.initialized = 1U;
    return DRV_OPTICAL_FLOW_OK;
}
void BSP_OPTICAL_FLOW_Service(void) {}
void BSP_OPTICAL_FLOW_GetStatus(BSP_OPTICAL_FLOW_Status *status) { *status = stub_status; }
uint32_t SVC_Timestamp_Ms(void) { return 1000U; }
void SVC_FlowNav_Init(void) {}
void SVC_FlowNav_Reset(void) {}
void SVC_FlowNav_Age(uint32_t now_ms) { (void)now_ms; }
SVC_FLOW_NAV_SampleResult SVC_FlowNav_PushSample(const SVC_FLOW_NAV_Sample *sample,
                                                 uint32_t now_ms)
{
    (void)now_ms;
    last_sample = *sample;
    push_count++;
    return SVC_FLOW_NAV_SAMPLE_ACCEPTED;
}
uint8_t SVC_FlowNav_GetHeight(float *h, float *vz, uint32_t *ms, uint32_t now_ms)
{
    (void)h; (void)vz; (void)ms; (void)now_ms;
    return 0U;
}
uint8_t SVC_FlowNav_GetSensorVelocity(float *vx, float *vy, uint32_t *ms, uint32_t now_ms)
{
    (void)vx; (void)vy; (void)ms; (void)now_ms;
    return 0U;
}
void SVC_FlowNav_GetState(SVC_FLOW_NAV_State *state) { memset(state, 0, sizeof(*state)); }
uint32_t SVC_FlowNav_GetLastGoodMs(void) { return 0U; }
void SVC_FlowNav_GetVelocity(float *vx, float *vy) { *vx = 0.0f; *vy = 0.0f; }
void SVC_FlowNav_GetPosition(float *x, float *y) { *x = 0.0f; *y = 0.0f; }
void SVC_FlowNav_GetDisplacement(float *dx, float *dy) { *dx = 0.0f; *dy = 0.0f; }
uint32_t SVC_FlowNav_GetIntegratedStepCount(void) { return 0U; }
uint32_t SVC_FlowNav_GetLastIntegrationDtUs(void) { return 0U; }

/* 与 APP_Control_QueueText 同一个缓冲上限：APP_UART_TX_TEXT_SIZE = 256。 */
void APP_Control_QueueText(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(text_lines[text_count % 16U], 256U, format, args);
    va_end(args);
    text_count++;
}

static void feed(int16_t fx, int16_t fy)
{
    memset(&stub_status.latest, 0, sizeof(stub_status.latest));
    stub_status.latest.valid = DRV_OPTICAL_FLOW_VALID;
    stub_status.latest.flow_valid = 1U;
    stub_status.latest.flow_vel_x = fx;
    stub_status.latest.flow_vel_y = fy;
    stub_status.latest.flow_quality = 200U;
    APP_OpticalFlow_Step();
}

static int set_mount(float yaw, float mirror)
{
    return (DRV_Airframe_SetParam("airframe.flow_mount_yaw_deg", yaw) != 0U) &&
           (DRV_Airframe_SetParam("airframe.flow_mount_mirror", mirror) != 0U);
}

static float read_param(const char *name)
{
    float v = -12345.0f;
    CHECK(DRV_Airframe_GetParam(name, &v) != 0U, 900);
    return v;
}

static const int16_t raw_inputs[][2] = { RAW_INPUTS_C };
static const float yaws[] = { 0.0f, 90.0f, 180.0f, 270.0f };

int main(void)
{
    unsigned before;

    DRV_Airframe_Clear();
    APP_OpticalFlow_Init();

    /* 1. 默认 0/0：与改动前那一行 FRD->FLU 逐位相同，扫完整个 int16 两轴。 */
    for (int32_t v = INT16_MIN; v <= INT16_MAX; ++v) {
        const int16_t s = (int16_t)v;
        feed(s, 77);
        CHECK(last_sample.flow_vel_x == s, 1);
        CHECK(last_sample.flow_vel_y == -77, 2);
        feed(-5, s);
        CHECK(last_sample.flow_vel_x == -5, 3);
        CHECK(last_sample.flow_vel_y ==
              ((s == INT16_MIN) ? INT16_MAX : (int16_t)(-s)), 4);
    }

    /* 2. 八种安装的换算表：打印出来交给 Python 独立核对。 */
    for (unsigned m = 0U; m < 2U; ++m) {
        for (unsigned y = 0U; y < 4U; ++y) {
            CHECK(set_mount(yaws[y], (float)m), 10);
            for (unsigned i = 0U; i < sizeof(raw_inputs) / sizeof(raw_inputs[0]); ++i) {
                before = push_count;
                feed(raw_inputs[i][0], raw_inputs[i][1]);
                CHECK(push_count == before + 1U, 11);
                printf("MAP %u %u %d %d %d %d\n", (unsigned)yaws[y], m,
                       raw_inputs[i][0], raw_inputs[i][1],
                       last_sample.flow_vel_x, last_sample.flow_vel_y);
            }
        }
    }

    /* 3. 写入口只收离散集合；拒绝时原值不动。 */
    CHECK(set_mount(90.0f, 1.0f), 20);
    {
        const float bad_yaw[] = { 45.0f, -90.0f, 360.0f, 89.9f, 1.0f, 1.0e9f, NAN, INFINITY };
        const float bad_mirror[] = { 2.0f, -1.0f, 0.5f, NAN, INFINITY };
        for (unsigned i = 0U; i < sizeof(bad_yaw) / sizeof(bad_yaw[0]); ++i) {
            CHECK(DRV_Airframe_SetParam("airframe.flow_mount_yaw_deg", bad_yaw[i]) == 0U, 21);
        }
        for (unsigned i = 0U; i < sizeof(bad_mirror) / sizeof(bad_mirror[0]); ++i) {
            CHECK(DRV_Airframe_SetParam("airframe.flow_mount_mirror", bad_mirror[i]) == 0U, 22);
        }
    }
    CHECK(read_param("airframe.flow_mount_yaw_deg") == 90.0f, 23);
    CHECK(read_param("airframe.flow_mount_mirror") == 1.0f, 24);
    /* -0 规范成 +0：PARAM? 不会报出 "-0.000000"。 */
    CHECK(DRV_Airframe_SetParam("airframe.flow_mount_yaw_deg", -0.0f) != 0U, 25);
    CHECK(signbit(read_param("airframe.flow_mount_yaw_deg")) == 0, 26);
    CHECK(DRV_Airframe_SetParam("airframe.flow_mount_mirror", -0.0f) != 0U, 27);
    CHECK(signbit(read_param("airframe.flow_mount_mirror")) == 0, 28);
    /* 两个名字都在遍历表里（PARAM? 靠它）。 */
    {
        unsigned found = 0U;
        for (uint32_t i = 0U; i < DRV_Airframe_GetParamCount(); ++i) {
            const char *n = DRV_Airframe_GetParamNameAt(i);
            if ((strcmp(n, "airframe.flow_mount_yaw_deg") == 0) ||
                (strcmp(n, "airframe.flow_mount_mirror") == 0)) {
                found++;
            }
        }
        CHECK(found == 2U, 29);
        CHECK(DRV_Airframe_IsDerivedName("airframe.flow_mount_yaw_deg") == 0U, 30);
    }

    /* 4. 整体写入（Flash 加载）绕过写入口时的兜底：非法值落回 0，读回即生效值。 */
    {
        DRV_Airframe_Params p;
        memset(&p, 0, sizeof(p));
        p.flow_mount_yaw_deg = 45.0f;
        p.flow_mount_mirror = 3.0f;
        DRV_Airframe_SetParams(&p);
        CHECK(DRV_Airframe_Get()->flow_mount_yaw_deg == 0.0f, 40);
        CHECK(DRV_Airframe_Get()->flow_mount_mirror == 0.0f, 41);
        CHECK(DRV_Airframe_FlowMountYawDeg(DRV_Airframe_Get()) == 0U, 42);
        CHECK(DRV_Airframe_FlowMountMirror(DRV_Airframe_Get()) == 0U, 43);
        feed(100, 200);
        CHECK(last_sample.flow_vel_x == 100 && last_sample.flow_vel_y == -200, 44);
        p.flow_mount_yaw_deg = 180.0f;
        p.flow_mount_mirror = 1.0f;
        DRV_Airframe_SetParams(&p);
        CHECK(DRV_Airframe_FlowMountYawDeg(DRV_Airframe_Get()) == 180U, 45);
        CHECK(DRV_Airframe_FlowMountMirror(DRV_Airframe_Get()) == 1U, 46);
        /* 安装两项不进解锁闸门：全零机体报的第一个缺项仍是质量。 */
        CHECK(strcmp(DRV_Airframe_FirstInvalidName(), "airframe.mass_kg") == 0, 47);
        CHECK(DRV_Airframe_FlowMountYawDeg(NULL) == 0U, 48);
        CHECK(DRV_Airframe_FlowMountMirror(NULL) == 0U, 49);
    }

    /* 5. FLOW? 状态行行尾带实际生效值，整行（含回车换行）没被截断。 */
    CHECK(set_mount(270.0f, 1.0f), 50);
    for (unsigned pass = 0U; pass < 2U; ++pass) {
        const char *tail = (pass == 0U) ? " mount_yaw=270 mount_mirror=1\r\n"
                                        : " mount_yaw=0 mount_mirror=0\r\n";
        size_t len;
        if (pass == 1U) {
            DRV_Airframe_Clear();
        }
        text_count = 0U;
        APP_OpticalFlow_Report();
        len = strlen(text_lines[0]);
        CHECK(len > strlen(tail) && len < 256U, 51);
        CHECK(strcmp(text_lines[0] + len - strlen(tail), tail) == 0, 52);
        text_lines[0][len - 2U] = '\0';   /* 去掉回车换行再交给 Python */
        printf("LINE %s\n", text_lines[0]);
    }
    REPORT();
}
"""


def _neg(value: int) -> int:
    """契约里的整数取负：INT16_MIN 钳到 INT16_MAX。"""
    return INT16_MAX if value == INT16_MIN else -value


def _expected(yaw: int, mirror: int, fx: int, fy: int) -> tuple[int, int]:
    """按任务契约逐字重写：v0=(fx,-fy)；mirror 翻 v0.y；再绕 +Z 逆时针转 yaw。"""
    x, y = fx, _neg(fy)
    if mirror:
        y = _neg(y)
    if yaw == 90:
        return _neg(y), x
    if yaw == 180:
        return _neg(x), _neg(y)
    if yaw == 270:
        return y, _neg(x)
    return x, y


@pytest.fixture(scope="module")
def firmware(tmp_path_factory):
    tmp_path = tmp_path_factory.mktemp("flow_mount")
    fakes = write_fakes(tmp_path)
    raw = ", ".join(f"{{ {fx}, {fy} }}" for fx, fy in RAW_INPUTS).replace(
        str(INT16_MIN), "INT16_MIN").replace(str(INT16_MAX), "INT16_MAX")
    result = build_and_run(
        tmp_path, "flow_mount",
        HARNESS.replace("RAW_INPUTS_C", raw),
        sources=[FLOW_APP, ROOT / "Driver" / "Src" / "drv_airframe_params.c"],
        includes=[fakes, ROOT / "App" / "Inc", ROOT / "Driver" / "Inc",
                  ROOT / "Services" / "Inc", ROOT / "BSP" / "Inc"],
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout, result.stdout
    maps: dict[tuple[int, int, int, int], tuple[int, int]] = {}
    lines: list[str] = []
    for line in result.stdout.splitlines():
        if line.startswith("MAP "):
            yaw, mirror, fx, fy, sx, sy = (int(item) for item in line.split()[1:])
            maps[(yaw, mirror, fx, fy)] = (sx, sy)
        elif line.startswith("LINE "):
            lines.append(line[len("LINE "):])
    return maps, lines


def test_default_mount_is_bit_identical_and_params_reject_illegal_values(firmware) -> None:
    """C 侧 CHECK 全过：0/0 扫完整个 int16 两轴与旧换算逐位相同；非法值拒绝、原值不动。"""
    maps, _lines = firmware
    assert len(maps) == len(MOUNTS) * len(RAW_INPUTS)


@pytest.mark.parametrize("yaw,mirror", MOUNTS)
def test_every_mount_matches_the_contract_table_including_int16_min(firmware, yaw, mirror) -> None:
    maps, _lines = firmware
    for fx, fy in RAW_INPUTS:
        assert maps[(yaw, mirror, fx, fy)] == _expected(yaw, mirror, fx, fy), (fx, fy)


@pytest.mark.parametrize("yaw,mirror", MOUNTS)
def test_host_mount_matrix_is_the_same_transform_the_firmware_applies(firmware, yaw, mirror) -> None:
    """推荐算法里的矩阵必须就是固件做的那个变换（不溢出的读数上逐一对拍）。"""
    maps, _lines = firmware
    matrix = flow_mount_matrix(yaw, mirror)
    for fx, fy in RAW_INPUTS:
        if INT16_MIN in (fx, fy):
            continue
        v0 = (fx, -fy)
        expected = (matrix[0][0] * v0[0] + matrix[0][1] * v0[1],
                    matrix[1][0] * v0[0] + matrix[1][1] * v0[1])
        assert maps[(yaw, mirror, fx, fy)] == expected, (fx, fy)


def test_flow_status_line_reports_the_effective_mount(firmware) -> None:
    _maps, lines = firmware
    assert len(lines) == 2
    assert lines[0].startswith("FLOW ok=")
    assert lines[0].endswith(" mount_yaw=270 mount_mirror=1")
    assert lines[1].endswith(" mount_yaw=0 mount_mirror=0")
    # 旧键原样保留在原来的位置：上位机其余各页照旧按 key=value 取。
    assert " height_valid=" in lines[0]
    assert lines[0].index("height_valid=") < lines[0].index("mount_yaw=")


def test_flow_status_line_fits_the_text_buffer_at_its_widest() -> None:
    """按类型上界把真实格式串撑到最长，整行连 \\r\\n 不超过 255。"""
    source = FLOW_APP.read_text(encoding="utf-8")
    match = re.search(r'APP_Control_QueueText\("(FLOW ok=[^"]*)"', source)
    assert match is not None
    fmt = match.group(1).replace("\\r\\n", "\r\n")
    assert fmt.endswith("mount_yaw=%u mount_mirror=%u\r\n")
    widest = {"%u": "255", "%ld": "-2147483648", "%lu": "4294967295", "%s": "none"}
    line = re.sub(r"%(lu|ld|u|s)", lambda m: widest[m.group(0)], fmt)
    assert len(line) <= 255, len(line)


# ── 推荐算法 ─────────────────────────────────────────────────────────────


def _transpose(matrix):
    return ((matrix[0][0], matrix[1][0]), (matrix[0][1], matrix[1][1]))


def _mul(left, right):
    return tuple(
        tuple(sum(left[r][k] * right[k][c] for k in range(2)) for c in range(2))
        for r in range(2)
    )


def _apply(matrix, vector):
    return (matrix[0][0] * vector[0] + matrix[0][1] * vector[1],
            matrix[1][0] * vector[0] + matrix[1][1] * vector[1])


def test_matrix_roundtrip_covers_all_eight_signed_permutations() -> None:
    seen = set()
    for yaw, mirror in MOUNTS:
        matrix = flow_mount_matrix(yaw, mirror)
        assert flow_mount_from_matrix(matrix) == (yaw, mirror)
        seen.add(matrix)
    assert len(seen) == 8


@pytest.mark.parametrize("true_mount,current", list(itertools.product(MOUNTS, MOUNTS)))
def test_recommendation_roundtrips_every_mount_from_every_current_setting(true_mount, current) -> None:
    """8 种真实安装 × 8 种当前参数：按当前参数采到的两步，推荐必须回到真实安装。

    模拟：物理安装 S 是"真实安装参数"对应矩阵的逆（正交阵，逆 = 转置），当前固件
    输出 = M_当前 · S · 真实位移；两步各带一点斜推的串轴分量。
    """
    physical = _transpose(flow_mount_matrix(*true_mount))
    chain = _mul(flow_mount_matrix(*current), physical)
    forward = _apply(chain, (0.48, 0.03))
    left = _apply(chain, (-0.04, 0.52))
    result = recommend_flow_mount(forward, left, *current)
    assert (result["yaw_deg"], result["mirror"]) == true_mount
    assert result["changed"] is (true_mount != current)
    assert result["forward_cosine"] > 0.99 and result["left_cosine"] > 0.99
    assert result["forward_axis_ratio"] == pytest.approx(0.48 / 0.03)
    assert result["left_axis_ratio"] == pytest.approx(0.52 / 0.04)
    # 纠正后两步都落回正轴。
    assert result["forward_corrected_m"][0] == pytest.approx(0.48)
    assert result["left_corrected_m"][1] == pytest.approx(0.52)
    assert result["description"] == describe_flow_mount(*true_mount)
    json.dumps(result)  # 要能原样落进证据 JSON


@pytest.mark.parametrize(
    "forward,left,label",
    [((0.05, 0.0), (0.0, 0.5), "+X"), ((0.5, 0.0), (0.06, -0.05), "+Y"),
     ((0.0, 0.0), (0.0, 0.0), "+X")],
)
def test_too_small_a_step_is_rejected_as_not_pushed(forward, left, label) -> None:
    with pytest.raises(GroundCalibrationError, match=re.escape(label) + ".*没推动.*重采"):
        recommend_flow_mount(forward, left, 0, 0)


@pytest.mark.parametrize(
    "forward,left",
    [
        ((0.5, 0.0), (0.45, 0.02)),     # 两步落在同一根轴上
        ((0.5, 0.0), (-0.5, 0.0)),      # 同轴反向
        ((0.35, 0.35), (-0.35, 0.35)),  # 两步都斜 45°
        ((0.5, 0.0), (0.3, 0.3)),       # +Y 步斜推
    ],
)
def test_contradictory_directions_are_rejected(forward, left) -> None:
    with pytest.raises(GroundCalibrationError, match="方向不一致.*重采"):
        recommend_flow_mount(forward, left, 0, 0)


@pytest.mark.parametrize("yaw,mirror", [(45, 0), (0, 2), (-90, 0)])
def test_current_mount_outside_the_contract_is_rejected(yaw, mirror) -> None:
    with pytest.raises(GroundCalibrationError):
        recommend_flow_mount((0.5, 0.0), (0.0, 0.5), yaw, mirror)


def test_nonfinite_observation_is_rejected() -> None:
    with pytest.raises(GroundCalibrationError, match="非有限"):
        recommend_flow_mount((math.nan, 0.0), (0.0, 0.5), 0, 0)


def test_2026_09_29_recording_refuses_to_guess_from_the_unusable_forward_step() -> None:
    """作者 2026-09-29 的实录：+Y 步把机体往左推，光流量到的全在 −X；+X 步几乎没动。

    只凭 +Y 一步分不出"转 270°"与带镜像的几种，必须等 +X 重采——推荐算法照实拒绝。
    """
    path = (ROOT / "data" / "calibration" / "flow_range" / "2026-09-29"
            / "flow_range_20260929_042725.json")
    if not path.exists():
        pytest.skip(f"缺少地面实录 {path}")
    stages = json.loads(path.read_text(encoding="utf-8"))["stages"]
    forward, left = stages["forward_x"], stages["left_y"]
    forward_obs = (forward["observed_distance_m"], forward["cross_axis_distance_m"])
    left_obs = (left["cross_axis_distance_m"], left["observed_distance_m"])
    assert left_obs[0] < -0.3 and abs(left_obs[1]) < 0.02
    with pytest.raises(GroundCalibrationError, match=r"\+X.*没推动"):
        recommend_flow_mount(forward_obs, left_obs, 0, 0)
    # 若 +X 重采后读到的是 +Y 方向（与 +Y 步的 −X 一起），推荐就是转 270°、不镜像。
    result = recommend_flow_mount((0.0, 0.5), left_obs, 0, 0)
    assert (result["yaw_deg"], result["mirror"]) == (270, 0)
    assert result["description"] == "旋转 270°、不镜像"
