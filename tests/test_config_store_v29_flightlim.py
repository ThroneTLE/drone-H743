"""CFG v29：飞行限幅块（coax.alt_max_m、coax.manual_tilt_max_rad、coax.yaw_stick_rate_rad_s）进 Flash。

作者 2026-10-01："飞机限速/加速度等参数让我可以在上位机可以随意配置"。钉住：
* 驱动默认 = 原写死常量（0.40 m、20°、1.04719758 rad/s），非法值被 SetParam 拒收；
* v29 往返：三项原样读回；
* v28 → v29：v28 记录里的竖直通道块照常生效，三项落回默认；升级后第一次保存写成 v29
  （848 字节，提交字仍 864），再读回仍然正确。

假 Flash、宿主桩与源码清单复用 tests/test_config_store_ab_slots.py。
"""

from __future__ import annotations

from pathlib import Path

import test_config_store_ab_slots as ab
from _micoair_hostfakes import CHECK_MACRO, build_and_run, write_fakes

HARNESS = r"""
#include "app_control_config_store.h"
#include "app_magxy.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_frame_contract.h"
#include "svc_mag_heading.h"

#include <stddef.h>
#include <string.h>

extern void fake_flash_reset(void);

static float coax_get(const char *name)
{
    float v = -1.0f;
    CHECK(DRV_COAX_CTRL_GetParam(name, &v) != 0U, 900);
    return v;
}

static void coax_put(const char *name, float value)
{
    CHECK(DRV_COAX_CTRL_SetParam(name, value) != 0U, 901);
}

/* 与 app_control_config_store.c 的 config_checksum 同一算法（旧记录要自己拼）。 */
static uint32_t checksum(const uint8_t *data, uint32_t length)
{
    uint32_t sum = 0xA5A55A5AUL;
    for (uint32_t i = 0U; i < length; ++i) {
        sum = (sum << 5U) | (sum >> 27U);
        sum ^= data[i];
        sum += 0x9E3779B9UL;
    }
    return sum;
}

static uint32_t round_up_word(uint32_t bytes)
{
    return ((bytes + 31U) / 32U) * 32U;
}

static void reseal(uint8_t *image, uint16_t size)
{
    const uint32_t sum = checksum(&image[8], size);
    memcpy(&image[8U + size], &sum, sizeof(sum));
}

static void plant_committed(uint32_t slot, const uint8_t *image, uint32_t total,
                            uint32_t commit_offset, uint32_t sequence)
{
    APP_ControlConfigCommit commit;
    uint32_t body_checksum;

    memcpy(&body_checksum, &image[total - 4U], sizeof(body_checksum));
    memset(&commit, 0xFF, sizeof(commit));
    commit.magic = APP_CONTROL_CFG_COMMIT_MAGIC;
    commit.sequence = sequence;
    commit.body_checksum = body_checksum;
    CHECK(APP_FlashService_EraseSector(slot) == APP_FLASH_SERVICE_OK, 910);
    CHECK(APP_FlashService_WriteData(slot, image, total) == APP_FLASH_SERVICE_OK, 911);
    CHECK(APP_FlashService_WriteData(slot + commit_offset, (const uint8_t *)&commit,
                                     sizeof(commit)) == APP_FLASH_SERVICE_OK, 912);
}

static DRV_Airframe_Params reference_airframe(void)
{
    DRV_Airframe_Params p;
    float *field = (float *)(void *)&p;

    memset(&p, 0, sizeof(p));
    for (uint32_t i = 0U; i < APP_CONTROL_AIRFRAME_V25_FLOAT_COUNT; ++i) {
        field[i] = 0.5f + 0.25f * (float)i;
    }
    p.derived_auto = 0.0f;
    return p;
}

static uint8_t cur[4096];
static uint8_t v28[4096];

int main(void)
{
    APP_ControlConfig cfg, loaded;
    const DRV_Airframe_Params ref = reference_airframe();
    uint8_t header[8];
    uint16_t size29, size28;
    uint32_t total29, total28;

    DRV_COAX_CTRL_Init();
    fake_flash_reset();
    memset(&cfg, 0, sizeof(cfg));
    DRV_Airframe_SetParams(&ref);

    /* 1) 默认 = 原写死常量 */
    CHECK(coax_get("coax.alt_max_m") == 0.40f, 1);
    CHECK(coax_get("coax.manual_tilt_max_rad") == 0.349065850f, 2);
    CHECK(coax_get("coax.yaw_stick_rate_rad_s") == 1.04719758f, 3);

    /* 2) 合法范围 */
    CHECK(DRV_COAX_CTRL_SetParam("coax.alt_max_m", 0.09f) == 0U, 4);
    CHECK(DRV_COAX_CTRL_SetParam("coax.alt_max_m", 20.5f) == 0U, 5);
    CHECK(DRV_COAX_CTRL_SetParam("coax.alt_max_m", 0.1f) == 1U, 6);
    CHECK(DRV_COAX_CTRL_SetParam("coax.alt_max_m", 20.0f) == 1U, 7);
    CHECK(DRV_COAX_CTRL_SetParam("coax.manual_tilt_max_rad", 0.04f) == 0U, 8);
    CHECK(DRV_COAX_CTRL_SetParam("coax.manual_tilt_max_rad", 0.8f) == 0U, 9);
    CHECK(DRV_COAX_CTRL_SetParam("coax.manual_tilt_max_rad", 0.05f) == 1U, 10);
    CHECK(DRV_COAX_CTRL_SetParam("coax.manual_tilt_max_rad", 0.785f) == 1U, 11);
    CHECK(DRV_COAX_CTRL_SetParam("coax.yaw_stick_rate_rad_s", 0.05f) == 0U, 12);
    CHECK(DRV_COAX_CTRL_SetParam("coax.yaw_stick_rate_rad_s", 6.5f) == 0U, 13);
    CHECK(DRV_COAX_CTRL_SetParam("coax.yaw_stick_rate_rad_s", 6.0f) == 1U, 14);

    /* 3) v29 往返 */
    coax_put("coax.alt_max_m", 3.5f);
    coax_put("coax.manual_tilt_max_rad", 0.5f);
    coax_put("coax.yaw_stick_rate_rad_s", 2.5f);
    coax_put("coax.hover_thrust_n", 14.25f);
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 20);
    coax_put("coax.alt_max_m", 0.4f);
    coax_put("coax.manual_tilt_max_rad", 0.3f);
    coax_put("coax.yaw_stick_rate_rad_s", 1.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 21);
    CHECK(coax_get("coax.alt_max_m") == 3.5f, 22);
    CHECK(coax_get("coax.manual_tilt_max_rad") == 0.5f, 23);
    CHECK(coax_get("coax.yaw_stick_rate_rad_s") == 2.5f, 24);

    /* 4) 取出刚存的 v29（槽 B），去掉尾部飞行限幅块拼一条 v28 */
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, header, 8U) == APP_FLASH_SERVICE_OK, 30);
    CHECK(header[4] == 29U && APP_CONTROL_CFG_VERSION == 29U, 31);
    memcpy(&size29, &header[6], sizeof(size29));
    total29 = 8U + size29 + 4U;
    CHECK(total29 == 848U, 32);
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, cur, total29) == APP_FLASH_SERVICE_OK, 33);
    size28 = (uint16_t)(size29 - sizeof(APP_ControlFlightLimitParams));
    total28 = 8U + size28 + 4U;
    memcpy(v28, cur, total28 - 4U);
    v28[4] = 28U;
    v28[5] = 0U;
    memcpy(&v28[6], &size28, sizeof(size28));
    reseal(v28, size28);
    CHECK(total28 == 836U, 34);
    CHECK(round_up_word(total28) == 864U && round_up_word(total29) == 864U, 35);

    fake_flash_reset();
    plant_committed(APP_CONTROL_CFG_SLOT_A, v28, total28, round_up_word(total28), 5U);
    coax_put("coax.alt_max_m", 9.0f);
    coax_put("coax.manual_tilt_max_rad", 0.6f);
    coax_put("coax.yaw_stick_rate_rad_s", 4.0f);
    coax_put("coax.hover_thrust_n", 0.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 40);
    CHECK(coax_get("coax.hover_thrust_n") == 14.25f, 41);              /* v28 的竖直通道块照常生效 */
    CHECK(coax_get("coax.alt_max_m") == 0.40f, 42);                    /* 没有飞行限幅块：默认 */
    CHECK(coax_get("coax.manual_tilt_max_rad") == 0.349065850f, 43);
    CHECK(coax_get("coax.yaw_stick_rate_rad_s") == 1.04719758f, 44);

    /* 5) 升级后第一次保存写成 v29，再读回仍然正确 */
    coax_put("coax.alt_max_m", 2.0f);
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 50);
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, header, 8U) == APP_FLASH_SERVICE_OK, 51);
    CHECK(header[4] == 29U, 52);
    coax_put("coax.alt_max_m", 0.4f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 53);
    CHECK(coax_get("coax.alt_max_m") == 2.0f, 54);
    CHECK(coax_get("coax.hover_thrust_n") == 14.25f, 55);
    REPORT();
}
"""


def test_v29_flightlim_roundtrip_and_v28_upgrade(tmp_path: Path) -> None:
    fakes = write_fakes(tmp_path, {"cmsis_os2.h": ab.FAKE_CMSIS_OS2_H, "main.h": ab.FAKE_MAIN_H})
    result = build_and_run(
        tmp_path, "cfg_v29_flightlim",
        CHECK_MACRO + ab.FAKE_FLASH_C + ab.HOST_STUBS_C + HARNESS,
        sources=ab.SOURCES, includes=ab.includes(fakes),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout, result.stdout
