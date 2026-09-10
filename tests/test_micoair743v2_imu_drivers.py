"""MicoAir743v2 移植：IMU 换算表与装配变换的宿主侧契约测试。

为什么这些东西必须单测：
    量程码填错、LSB 表抄错、坐标轴符号翻反 —— 这三类错误**都不会报错**，
    只会让姿态整体缩放或横滚方向相反，要飞起来才发现。它们又恰好是纯函数，
    可以在 PC 上用 host gcc 直接钉死（decoupling-spec D5-1）。

传输层（SPI/I2C 时序）不在这里测，那需要真板子。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER_INC = ROOT / "Driver" / "Inc"
SERVICES_INC = ROOT / "Services" / "Inc"


HARNESS = r"""
#include "drv_bmi088_tables.h"
#include "drv_bmi270_tables.h"
#include "svc_imu.h"

#include <math.h>
#include <stdio.h>

static int failures;

#define CHECK(cond, id)                                                   \
    do {                                                                  \
        if (!(cond)) {                                                    \
            printf("FAIL %d at line %d: %s\n", (id), __LINE__, #cond);    \
            failures++;                                                    \
        }                                                                 \
    } while (0)

#define NEAR(a, b) (fabsf((float)(a) - (float)(b)) < 1.0e-3f)

static void check_bmi088_ranges(void)
{
    /* 通用枚举的 ±16 g 在 BMI088 上映射到物理 ±24 g（该芯片没有 ±16 g 档）。 */
    CHECK(DRV_BMI088_AccelRangeCode(DRV_IMU_ACCEL_RANGE_16G) == 0x03U, 1);
    CHECK(DRV_BMI088_AccelRangeCode(DRV_IMU_ACCEL_RANGE_8G) == 0x02U, 2);
    CHECK(DRV_BMI088_AccelRangeCode(DRV_IMU_ACCEL_RANGE_4G) == 0x01U, 3);
    CHECK(DRV_BMI088_AccelRangeCode(DRV_IMU_ACCEL_RANGE_2G) == 0x00U, 4);

    CHECK(NEAR(DRV_BMI088_AccelLsbPerG(DRV_IMU_ACCEL_RANGE_16G), 32768.0f / 24.0f), 5);
    CHECK(NEAR(DRV_BMI088_AccelLsbPerG(DRV_IMU_ACCEL_RANGE_2G), 32768.0f / 3.0f), 6);

    CHECK(DRV_BMI088_GyroRangeCode(DRV_IMU_GYRO_RANGE_2000DPS) == 0x00U, 7);
    CHECK(DRV_BMI088_GyroRangeCode(DRV_IMU_GYRO_RANGE_1000DPS) == 0x01U, 8);
    CHECK(NEAR(DRV_BMI088_GyroLsbPerDps(DRV_IMU_GYRO_RANGE_1000DPS), 32.768f), 9);

    /* 比 ±125 dps 更细的档 BMI088 没有，必须夹到 ±125 而不是静默按错刻度换算。 */
    CHECK(DRV_BMI088_GyroRangeCode(DRV_IMU_GYRO_RANGE_15D625DPS) == 0x04U, 10);
    CHECK(NEAR(DRV_BMI088_GyroLsbPerDps(DRV_IMU_GYRO_RANGE_15D625DPS),
               32768.0f / 125.0f), 11);
}

static void check_bmi088_rates(void)
{
    uint16_t odr_hz = 0U;
    uint16_t bw_hz = 0U;
    uint8_t code;

    /* 加计没有 1 kHz 档，请求 1 kHz 落到 1600 Hz。 */
    CHECK(DRV_BMI088_AccelOdrCode(DRV_IMU_ODR_1KHZ, &odr_hz) == 0x0CU, 20);
    CHECK(odr_hz == 1600U, 21);

    /* 213 Hz 请求：1600 Hz ODR 下 osr2 是 234 Hz（超了），所以取 osr4 的 145 Hz。 */
    code = DRV_BMI088_AccelBwpCode(1600U, 213U, &bw_hz);
    CHECK(code == DRV_BMI088_ACC_BWP_OSR4, 22);
    CHECK(bw_hz == 145U, 23);

    /* ACC_CONF = bwp<<4 | odr */
    CHECK(DRV_BMI088_BuildAccConf(DRV_IMU_ODR_1KHZ, 213U, &bw_hz) == 0x8CU, 24);

    /* 陀螺的 ODR 与带宽是一个码绑定的，1 kHz 档只有 116 Hz 一种带宽。 */
    code = DRV_BMI088_GyroBandwidthCode(DRV_IMU_ODR_1KHZ, 213U, &odr_hz, &bw_hz);
    CHECK(code == 0x02U, 25);
    CHECK(odr_hz == 1000U, 26);
    CHECK(bw_hz == 116U, 27);
}

static void check_bmi088_temperature(void)
{
    /* 0 计数 = 23 °C 偏置 */
    CHECK(NEAR(DRV_BMI088_ConvertTemperature(0x00U, 0x00U), 23.0f), 30);
    /* MSB=8 → 64 计数 → 64*0.125 + 23 = 31 °C */
    CHECK(NEAR(DRV_BMI088_ConvertTemperature(0x08U, 0x00U), 31.0f), 31);
    /* 11 位有符号：MSB=0x80 → 1024 → 折成 -1024 → -105 °C */
    CHECK(NEAR(DRV_BMI088_ConvertTemperature(0x80U, 0x00U), -105.0f), 32);
    /* LSB 的高 3 位才是有效低位，低 5 位必须被丢掉 */
    CHECK(NEAR(DRV_BMI088_ConvertTemperature(0x00U, 0xE0U),
               23.0f + 7.0f * 0.125f), 33);
    CHECK(NEAR(DRV_BMI088_ConvertTemperature(0x00U, 0x1FU), 23.0f), 34);
}

static void check_bmi270(void)
{
    uint16_t bw_hz = 0U;
    uint16_t odr_hz = 0U;

    /* BMI270 的量程刻度与通用枚举一致，但寄存器编号顺序**相反**。 */
    CHECK(DRV_BMI270_AccelRangeCode(DRV_IMU_ACCEL_RANGE_16G) == 0x03U, 40);
    CHECK(DRV_BMI270_AccelRangeCode(DRV_IMU_ACCEL_RANGE_2G) == 0x00U, 41);
    CHECK(NEAR(DRV_BMI270_AccelLsbPerG(DRV_IMU_ACCEL_RANGE_16G), 2048.0f), 42);
    CHECK(DRV_BMI270_GyroRangeCode(DRV_IMU_GYRO_RANGE_2000DPS) == 0x00U, 43);
    CHECK(DRV_BMI270_GyroRangeCode(DRV_IMU_GYRO_RANGE_125DPS) == 0x04U, 44);

    /* 没有 1 kHz 档，落到相邻的 800 Hz。 */
    CHECK(DRV_BMI270_OdrCode(DRV_IMU_ODR_1KHZ, 0U, &odr_hz) == 0x0BU, 45);
    CHECK(odr_hz == 800U, 46);

    /* 800 Hz 下加计 normal=375 / osr2=187 / osr4=93；请求 213 取 osr2。 */
    CHECK(DRV_BMI270_BwpCode(800U, 213U, 0U, &bw_hz) == DRV_BMI270_BWP_OSR2, 47);
    CHECK(bw_hz == 187U, 48);
    /* bit7 高性能滤波 + bwp<<4 + odr */
    CHECK(DRV_BMI270_BuildAccConf(DRV_IMU_ODR_1KHZ, 213U, &bw_hz) == 0x9BU, 49);

    /* 陀螺 800 Hz 下 normal=188，213 够得着，取 normal。 */
    CHECK(DRV_BMI270_BwpCode(800U, 213U, 1U, &bw_hz) == DRV_BMI270_BWP_NORMAL, 50);
    CHECK(bw_hz == 188U, 51);
    /* bit7 filter_perf + bit6 noise_perf + bwp<<4 + odr */
    CHECK(DRV_BMI270_BuildGyrConf(DRV_IMU_ODR_1KHZ, 213U, &bw_hz) == 0xEBU, 52);

    /* 0x8000 是"温度无效"哨兵，不能当成 -64 °C 一样的有效读数。 */
    uint8_t valid = 9U;
    (void)DRV_BMI270_ConvertTemperature(0x00U, 0x80U, &valid);
    CHECK(valid == 0U, 53);
    CHECK(NEAR(DRV_BMI270_ConvertTemperature(0x00U, 0x00U, &valid), 23.0f), 54);
    CHECK(valid == 1U, 55);
}

/* 行列式必须是 +1：这些是真旋转，不含镜像，所以极向量与轴向量共用同一映射。 */
static float rotation_determinant(SVC_IMU_Rotation rotation)
{
    DRV_FRAME_Vector3f ex = {1.0f, 0.0f, 0.0f};
    DRV_FRAME_Vector3f ey = {0.0f, 1.0f, 0.0f};
    DRV_FRAME_Vector3f ez = {0.0f, 0.0f, 1.0f};

    DRV_FRAME_Vector3f c0 = SVC_IMU_RotateToFlu(rotation, ex);
    DRV_FRAME_Vector3f c1 = SVC_IMU_RotateToFlu(rotation, ey);
    DRV_FRAME_Vector3f c2 = SVC_IMU_RotateToFlu(rotation, ez);

    return c0.x * (c1.y * c2.z - c1.z * c2.y)
         - c1.x * (c0.y * c2.z - c0.z * c2.y)
         + c2.x * (c0.y * c1.z - c0.z * c1.y);
}

static void check_mounting(void)
{
    const SVC_IMU_Rotation all[] = {
        SVC_IMU_ROTATION_NONE, SVC_IMU_ROTATION_YAW_90,
        SVC_IMU_ROTATION_YAW_180, SVC_IMU_ROTATION_YAW_270,
        SVC_IMU_ROTATION_ROLL_180, SVC_IMU_ROTATION_ROLL_180_YAW_90,
        SVC_IMU_ROTATION_PITCH_180, SVC_IMU_ROTATION_ROLL_180_YAW_270,
    };
    unsigned i;

    for (i = 0U; i < sizeof(all) / sizeof(all[0]); i++) {
        CHECK(NEAR(rotation_determinant(all[i]), 1.0f), 60);
    }

    /* 上游 hwdef 记录的贴装朝向。 */
    CHECK(SVC_IMU_DefaultRotation(DRV_IMU_CHIP_BMI088) ==
          SVC_IMU_ROTATION_ROLL_180_YAW_270, 61);
    CHECK(SVC_IMU_DefaultRotation(DRV_IMU_CHIP_BMI270) ==
          SVC_IMU_ROTATION_ROLL_180, 62);

    {
        /*
         * BMI270 是 ROLL_180：芯片轴绕 X 翻 180° 得到 FRD，再 FRD→FLU 又翻一次，
         * 两次抵消 —— 所以它的芯片轴恰好就是 FLU。这个"恰好"必须被钉住，
         * 否则以后有人"顺手修正"一下就把它改错了。
         */
        DRV_FRAME_Vector3f v = {1.0f, 2.0f, 3.0f};
        DRV_FRAME_Vector3f flu = SVC_IMU_RotateToFlu(SVC_IMU_ROTATION_ROLL_180, v);
        CHECK(NEAR(flu.x, 1.0f) && NEAR(flu.y, 2.0f) && NEAR(flu.z, 3.0f), 63);
    }

    {
        /* BMI088 是 ROLL_180_YAW_270：chip(x,y,z) → FLU(-y, x, z)。 */
        DRV_FRAME_Vector3f v = {1.0f, 2.0f, 3.0f};
        DRV_FRAME_Vector3f flu =
            SVC_IMU_RotateToFlu(SVC_IMU_ROTATION_ROLL_180_YAW_270, v);
        CHECK(NEAR(flu.x, -2.0f) && NEAR(flu.y, 1.0f) && NEAR(flu.z, 3.0f), 64);
    }

    {
        /*
         * 老板子的 ICM-42688 映射必须与迁移前 app_sensor.c 里那三行逐字节相同：
         *     out = (-in[2], -in[0], in[1])
         * tests/test_flu_seam0_sensor_frame.py 依赖它，改了会连锁失败。
         */
        const float in[3] = {1.0f, 2.0f, 3.0f};
        float out[3] = {0.0f, 0.0f, 0.0f};
        SVC_IMU_ChipToIntermediate(DRV_IMU_CHIP_ICM42688, in, out);
        CHECK(NEAR(out[0], -3.0f) && NEAR(out[1], -1.0f) && NEAR(out[2], 2.0f), 65);
    }

    {
        /* 中间轴是「X 后 / Y 右 / Z 上」= (-FRD.x, FRD.y, -FRD.z)。 */
        const float in[3] = {1.0f, 2.0f, 3.0f};
        float out[3] = {0.0f, 0.0f, 0.0f};

        SVC_IMU_ChipToIntermediate(DRV_IMU_CHIP_BMI088, in, out);
        CHECK(NEAR(out[0], 2.0f) && NEAR(out[1], -1.0f) && NEAR(out[2], 3.0f), 66);

        SVC_IMU_ChipToIntermediate(DRV_IMU_CHIP_BMI270, in, out);
        CHECK(NEAR(out[0], -1.0f) && NEAR(out[1], -2.0f) && NEAR(out[2], 3.0f), 67);
    }
}

static void check_selection(void)
{
    SVC_IMU_Selection sel;

    SVC_IMU_SelectionReset(&sel);
    CHECK(sel.selected == DRV_IMU_CHIP_NONE, 70);

    /*
     * 记账不等于选中。probe 成功只说明总线上有这颗芯片，它还可能在 init 阶段失败
     * （BMI270 要刷 328 字节配置固件，BMI088 有一串带回读重试的寄存器写）。
     * 所以 Record 之后 selected 必须仍然是 NONE。
     */
    SVC_IMU_SelectionRecord(&sel, DRV_IMU_CHIP_BMI088, 0x00U, DRV_IMU_BAD_ID);
    CHECK(sel.selected == DRV_IMU_CHIP_NONE, 71);

    SVC_IMU_SelectionRecord(&sel, DRV_IMU_CHIP_BMI270, 0x24U, DRV_IMU_OK);
    CHECK(sel.selected == DRV_IMU_CHIP_NONE, 72);

    SVC_IMU_SelectionRecord(&sel, DRV_IMU_CHIP_ICM42688, 0x47U, DRV_IMU_OK);
    CHECK(sel.selected == DRV_IMU_CHIP_NONE, 73);
    CHECK(sel.probe_count == 3U, 74);

    /* 三次探测都如实记下来了，诊断据此回报"哪颗在、ID 是多少"。 */
    CHECK(sel.probed_kind[0] == (uint8_t)DRV_IMU_CHIP_BMI088, 75);
    CHECK(sel.probed_status[0] == DRV_IMU_BAD_ID, 76);
    CHECK(sel.probed_kind[1] == (uint8_t)DRV_IMU_CHIP_BMI270, 77);
    CHECK(sel.probed_chip_id[1] == 0x24U, 78);

    /* init 成功之后才敲定，同时装好该芯片的默认安装旋转。 */
    SVC_IMU_SelectionCommit(&sel, DRV_IMU_CHIP_BMI270, 0x24U);
    CHECK(sel.selected == DRV_IMU_CHIP_BMI270, 79);
    CHECK(sel.selected_chip_id == 0x24U, 80);
    CHECK(sel.rotation == SVC_IMU_ROTATION_ROLL_180, 81);
}

int main(void)
{
    check_bmi088_ranges();
    check_bmi088_rates();
    check_bmi088_temperature();
    check_bmi270();
    check_mounting();
    check_selection();

    if (failures != 0) {
        printf("%d checks failed\n", failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
"""


def test_imu_tables_and_mounting_host_harness(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required for the pure-C IMU harness")

    harness = tmp_path / "micoair_imu_harness.c"
    executable = tmp_path / "micoair_imu_harness.exe"
    harness.write_text(HARNESS, encoding="utf-8")

    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{DRIVER_INC}",
            f"-I{SERVICES_INC}",
            str(ROOT / "Driver" / "Src" / "drv_bmi088_tables.c"),
            str(ROOT / "Driver" / "Src" / "drv_bmi270_tables.c"),
            str(ROOT / "Services" / "Src" / "svc_imu.c"),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    result = subprocess.run(
        [str(executable)], cwd=ROOT, capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _strip_c_comments(text: str) -> str:
    """去掉注释再查禁用符号。

    注释里出现 "main.h"、"HAL_" 是正常的——本仓库的规范就要求把"为什么不依赖 HAL"
    写清楚。只看代码，不看解释。
    """
    out = []
    i = 0
    n = len(text)
    while i < n:
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            i = n if end < 0 else end + 2
        elif text.startswith("//", i):
            end = text.find("\n", i)
            i = n if end < 0 else end
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def test_pure_modules_stay_free_of_hal_and_rtos() -> None:
    """换算表与装配变换必须能脱离单片机编译，否则维度五就断了。"""
    sources = [
        "Driver/Src/drv_bmi088_tables.c",
        "Driver/Src/drv_bmi270_tables.c",
        "Driver/Inc/drv_bmi088_tables.h",
        "Driver/Inc/drv_bmi270_tables.h",
        "Driver/Inc/drv_imu_types.h",
        "Services/Src/svc_imu.c",
        "Services/Inc/svc_imu.h",
    ]
    forbidden = ("stm32h7xx_hal", "HAL_", "FreeRTOS", "cmsis_os", "osDelay", "main.h")

    for relative in sources:
        code = _strip_c_comments((ROOT / relative).read_text(encoding="utf-8"))
        for token in forbidden:
            assert token not in code, f"{relative} must stay HAL/RTOS free ({token})"


def test_bmi270_config_blob_matches_bosch_size() -> None:
    """配置数据长度写错会让上传静默失败，芯片停在 INTERNAL_STATUS != 1。"""
    header = (ROOT / "Driver" / "Inc" / "drv_bmi270_config.h").read_text(encoding="utf-8")
    assert "#define DRV_BMI270_CONFIG_SIZE 328U" in header

    source = (ROOT / "Driver" / "Src" / "drv_bmi270_config.c").read_text(encoding="utf-8")
    assert "BSD-3-Clause" in source, "Bosch 原始许可头不得删除"
    body = source.split("{", 1)[1].rsplit("}", 1)[0]
    assert len([token for token in body.split(",") if token.strip()]) == 328
