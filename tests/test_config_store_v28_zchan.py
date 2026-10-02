"""CFG v28（R-ALTID-1）：竖直通道块（coax.hover_thrust_n、coax.z_vel_fusion）进 Flash，用真实配置存储源码在宿主上跑。

作者 2026-09-30："把悬停推力和融合开关存进 Flash"。高度环 3/8/3 已在 Flash，而融合关时位置环 PM 为负：
两项只在 RAM 时，一断电重启高度环就会振荡。这里钉住：
* 驱动默认：悬停推力 0（关）、融合 1（开）；
* v28 往返：两项原样读回；
* v27 → v28：v27 记录里 GPT 的 MAGXY 块照常生效（不能按"非当前版本"清掉），竖直通道两项落回默认；
  升级后第一次保存写成 v28（836 字节，提交字 864），再读回仍然正确。

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
static uint8_t v27[4096];

int main(void)
{
    APP_ControlConfig cfg, loaded;
    APP_MagXY_Persisted xy;
    SVC_MAG_HeadingConfig active;
    const DRV_Airframe_Params ref = reference_airframe();
    uint8_t header[8];
    uint16_t size28, size27;
    uint32_t total28, total27;

    DRV_COAX_CTRL_Init();
    fake_flash_reset();
    memset(&cfg, 0, sizeof(cfg));
    DRV_Airframe_SetParams(&ref);

    /* 1) 驱动默认：悬停推力关、融合开 */
    CHECK(coax_get("coax.hover_thrust_n") == 0.0f, 1);
    CHECK(coax_get("coax.z_vel_fusion") == 1.0f, 2);

    /* 2) v28 往返（顺带开一份 MAGXY，后面 v27 迁移要看它不被清掉） */
    memset(&xy, 0, sizeof(xy));
    xy.bias_x_mgauss = -3.725f;
    xy.bias_y_mgauss = 135.432f;
    xy.radius_xy_mgauss = 268.292f;
    xy.frame_contract_version = DRV_FRAME_CONTRACT_VERSION;
    xy.axis_verified = 1U;
    xy.enabled_default = 1U;
    APP_MagXY_ApplyPersisted(&xy);
    coax_put("coax.hover_thrust_n", 14.25f);
    coax_put("coax.z_vel_fusion", 0.0f);
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 3);
    coax_put("coax.hover_thrust_n", 0.0f);
    coax_put("coax.z_vel_fusion", 1.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 4);
    CHECK(coax_get("coax.hover_thrust_n") == 14.25f, 5);
    CHECK(coax_get("coax.z_vel_fusion") == 0.0f, 6);

    /* 3) 取出刚存的 v28（槽 B），去掉尾部竖直通道块拼一条 v27 */
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, header, 8U) == APP_FLASH_SERVICE_OK, 7);
    CHECK(header[4] == APP_CONTROL_CFG_VERSION && APP_CONTROL_CFG_VERSION == 29U, 8);
    memcpy(&size28, &header[6], sizeof(size28));
    total28 = 8U + size28 + 4U;
    CHECK(total28 == 848U, 9);
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, cur, total28) == APP_FLASH_SERVICE_OK, 10);
    size27 = (uint16_t)(size28 - sizeof(APP_ControlZChannelParams) -
                        sizeof(APP_ControlFlightLimitParams));
    total27 = 8U + size27 + 4U;
    memcpy(v27, cur, total27 - 4U);
    v27[4] = 27U;
    v27[5] = 0U;
    memcpy(&v27[6], &size27, sizeof(size27));
    reseal(v27, size27);
    CHECK(round_up_word(total27) == 832U && round_up_word(total28) == 864U, 11);

    fake_flash_reset();
    plant_committed(APP_CONTROL_CFG_SLOT_A, v27, total27, round_up_word(total27), 5U);
    APP_MagXY_ApplyPersisted(NULL);                 /* RAM 打乱 */
    coax_put("coax.hover_thrust_n", 9.0f);
    coax_put("coax.z_vel_fusion", 0.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 12);
    APP_MagXY_GetConfig(&active);
    CHECK(active.enabled == 1U && active.axis_verified == 1U, 13);   /* v27 的 XY 块照常生效 */
    CHECK(active.bias_x_mgauss == xy.bias_x_mgauss, 14);
    CHECK(coax_get("coax.hover_thrust_n") == 0.0f, 15);               /* 没有竖直通道块：默认 */
    CHECK(coax_get("coax.z_vel_fusion") == 1.0f, 16);

    /* 4) 升级后第一次保存写成 v28，再读回仍然正确 */
    coax_put("coax.hover_thrust_n", 13.5f);
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 17);
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, header, 8U) == APP_FLASH_SERVICE_OK, 18);
    CHECK(header[4] == 29U, 19);
    coax_put("coax.hover_thrust_n", 0.0f);
    APP_MagXY_ApplyPersisted(NULL);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 20);
    CHECK(coax_get("coax.hover_thrust_n") == 13.5f, 21);
    CHECK(coax_get("coax.z_vel_fusion") == 1.0f, 22);
    APP_MagXY_GetConfig(&active);
    CHECK(active.enabled == 1U, 23);
    REPORT();
}
"""


def test_v28_zchan_roundtrip_and_v27_upgrade_keeps_magxy(tmp_path: Path) -> None:
    fakes = write_fakes(tmp_path, {"cmsis_os2.h": ab.FAKE_CMSIS_OS2_H, "main.h": ab.FAKE_MAIN_H})
    result = build_and_run(
        tmp_path, "cfg_v28_zchan",
        CHECK_MACRO + ab.FAKE_FLASH_C + ab.HOST_STUBS_C + HARNESS,
        sources=ab.SOURCES, includes=ab.includes(fakes),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout, result.stdout
