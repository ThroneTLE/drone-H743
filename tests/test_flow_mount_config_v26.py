"""R-FLOWMOUNT-1：CFG v25/v26 → v27 迁移，用真实配置存储源码在宿主上跑。

v26 在机体块尾部追加光流安装两项，记录从 800 字节变成 808 字节，**提交字从 800 挪到
832**。这正是 v25 那次静态断言防的事：读取时若只按当前记录大小找提交字，升级后第一次
上电两个 v25 槽都找不到提交字（序号都退成 1），按槽 A 优先就可能读回较旧的那一份——
作者 Flash 里最近一次保存的参数就这么悄悄丢了。

所以这里完全照作者板子的真实处境摆：两个槽里各有一条**带提交字**的 v25 记录（提交字在
800），一新一旧，两种摆法（新的在 A / 新的在 B）都要读回新的那份；36 项机体参数逐位
保留、安装两项落回 0/0；随后第一次保存写成当前版本（2026-09-30 起 v28，提交字在 864），再读回来仍然正确。

假 Flash、宿主桩与源码清单复用 tests/test_config_store_ab_slots.py（同一套 H7 约束：
整槽擦除、同一 32 字节 word 擦除间只能写一次）。
"""

from __future__ import annotations

from pathlib import Path

import test_config_store_ab_slots as ab
from _micoair_hostfakes import CHECK_MACRO, build_and_run, write_fakes


HARNESS = r"""
#include "app_control_config_store.h"
#include "app_magxy.h"
#include "app_rc_config.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_frame_contract.h"

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

/* 每一项都是互不相同的非零值；手动派生档，派生值不被重算，才能逐位比对。 */
static DRV_Airframe_Params reference_airframe(void)
{
    DRV_Airframe_Params p;
    float *field = (float *)(void *)&p;

    memset(&p, 0, sizeof(p));
    for (uint32_t i = 0U; i < APP_CONTROL_AIRFRAME_V25_FLOAT_COUNT; ++i) {
        field[i] = 0.5f + 0.25f * (float)i;
    }
    p.derived_auto = 0.0f;
    p.flow_mount_yaw_deg = 90.0f;
    p.flow_mount_mirror = 1.0f;
    return p;
}

static uint8_t v27_image[4096];
static uint8_t v26_image[4096];
static uint8_t v25_newer[4096];
static uint8_t v25_older[4096];

/* 在 slot 放一条完整的"已提交"记录：先主体，再在 commit_offset 写提交字。 */
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

static void reseal(uint8_t *image, uint16_t size)
{
    const uint32_t sum = checksum(&image[8], size);
    memcpy(&image[8U + size], &sum, sizeof(sum));
}

static void scramble_ram(void)
{
    DRV_Airframe_Params junk;
    memset(&junk, 0, sizeof(junk));
    junk.board_mass_g = 999.0f;
    junk.flow_mount_yaw_deg = 180.0f;   /* 读旧记录必须显式落回 0/0，不能留着 RAM 里的值 */
    junk.flow_mount_mirror = 1.0f;
    DRV_Airframe_SetParams(&junk);
    coax_put("coax.rate_pitch_kp", 0.3f);
}

static void check_loaded_matches_reference(const DRV_Airframe_Params *ref, int id)
{
    const DRV_Airframe_Params *now = DRV_Airframe_Get();
    CHECK(memcmp(now, ref, sizeof(APP_ControlAirframeParamsV25)) == 0, id);
    CHECK(now->flow_mount_yaw_deg == 0.0f, id + 1);
    CHECK(now->flow_mount_mirror == 0.0f, id + 2);
    CHECK(DRV_Airframe_FlowMountYawDeg(now) == 0U, id + 3);
    CHECK(DRV_Airframe_FlowMountMirror(now) == 0U, id + 4);
    CHECK(coax_get("coax.rate_pitch_kp") == 0.24f, id + 5);
}

int main(void)
{
    APP_ControlConfig cfg, loaded;
    const DRV_Airframe_Params ref = reference_airframe();
    uint16_t size27, size26, size25;
    uint32_t total27, total26, total25, commit27, commit26, commit25, af_off;
    uint8_t header[8];

    DRV_COAX_CTRL_Init();
    fake_flash_reset();
    memset(&cfg, 0, sizeof(cfg));
    cfg.servo[0].id = 9U;

    /* ---- 当前 v27 往返：机体参数仍全部原样读回 ---- */
    DRV_Airframe_SetParams(&ref);
    coax_put("coax.rate_pitch_kp", 0.24f);
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 1);
    scramble_ram();
    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 2);
    CHECK(memcmp(DRV_Airframe_Get(), &ref, sizeof(ref)) == 0, 3);
    CHECK(DRV_Airframe_FlowMountYawDeg(DRV_Airframe_Get()) == 90U, 4);
    CHECK(DRV_Airframe_FlowMountMirror(DRV_Airframe_Get()) == 1U, 5);
    CHECK(loaded.servo[0].id == 9U, 6);

    /* 刚存的那条在槽 B（两个槽都空时活动槽是 A，写对面）。 */
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, header, 8U) == APP_FLASH_SERVICE_OK, 7);
    memcpy(&size27, &header[6], sizeof(size27));
    total27 = 8U + size27 + 4U;
    CHECK(total27 <= sizeof(v27_image), 8);
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, v27_image, total27) ==
          APP_FLASH_SERVICE_OK, 9);
    /* v27_image 装的是当前版本（2026-09-30 起 v28）的记录。 */
    CHECK(v27_image[4] == APP_CONTROL_CFG_VERSION && v27_image[5] == 0U, 10);

    /* 机体块在记录里的位置：头 8 字节 + config + 增益块 + 遥控块。先证明找对了地方。 */
    af_off = 8U + (uint32_t)sizeof(APP_ControlConfig) +
             (uint32_t)sizeof(APP_ControlCoaxTunableParams) + (uint32_t)sizeof(APP_RcConfig);
    CHECK(memcmp(&v27_image[af_off], &ref, sizeof(ref)) == 0, 11);

    /* Freeze a true v26 image by removing the appended XY (v27) and vertical-channel (v28) blocks. */
    size26 = (uint16_t)(size27 - sizeof(APP_MagXY_Persisted) - sizeof(APP_ControlZChannelParams) -
                        sizeof(APP_ControlFlightLimitParams));
    total26 = 8U + size26 + 4U;
    memcpy(v26_image, v27_image, total26 - 4U);
    v26_image[4] = 26U;
    memcpy(&v26_image[6], &size26, sizeof(size26));
    reseal(v26_image, size26);

    /* ---- 拼出 v25：去掉机体块尾部 8 字节，版本 25，重算校验和 ---- */
    size25 = (uint16_t)(size26 - 8U);
    total25 = 8U + size25 + 4U;
    memcpy(v25_newer, v26_image, af_off + sizeof(APP_ControlAirframeParamsV25));
    memcpy(&v25_newer[af_off + sizeof(APP_ControlAirframeParamsV25)],
           &v26_image[af_off + sizeof(DRV_Airframe_Params)],
           total26 - 4U - (af_off + sizeof(DRV_Airframe_Params)));
    v25_newer[4] = 25U;
    v25_newer[5] = 0U;
    memcpy(&v25_newer[6], &size25, sizeof(size25));
    reseal(v25_newer, size25);

    /* 较旧的那一份：同样的记录，只是飞控板质量还是改之前的 11 g。 */
    memcpy(v25_older, v25_newer, total25);
    {
        const float old_mass = 11.0f;
        memcpy(&v25_older[af_off], &old_mass, sizeof(old_mass));
    }
    reseal(v25_older, size25);

    commit27 = round_up_word(total27);
    commit26 = round_up_word(total26);
    commit25 = round_up_word(total25);
    /* 这条测试存在的意义：提交字确实挪了。 */
    CHECK(commit26 != commit25, 12);
    CHECK(commit27 != commit26, 13);   /* v28 的竖直通道块又把提交字从 832 挪到 864 */
    printf("SIZES %u %u %u %u %u %u\n", (unsigned)total27, (unsigned)commit27,
           (unsigned)total26, (unsigned)commit26,
           (unsigned)total25, (unsigned)commit25);

    /* ---- 作者板子的真实处境 1：新的在槽 B ---- */
    fake_flash_reset();
    plant_committed(APP_CONTROL_CFG_SLOT_A, v25_older, total25, commit25, 5U);
    plant_committed(APP_CONTROL_CFG_SLOT_B, v25_newer, total25, commit25, 6U);
    scramble_ram();
    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 20);
    check_loaded_matches_reference(&ref, 21);
    CHECK(DRV_Airframe_Get()->board_mass_g == ref.board_mass_g, 27);
    CHECK(loaded.servo[0].id == 9U, 28);

    /* ---- 处境 2：新的在槽 A（与上面对调），同样要读回新的那一份 ---- */
    fake_flash_reset();
    plant_committed(APP_CONTROL_CFG_SLOT_A, v25_newer, total25, commit25, 8U);
    plant_committed(APP_CONTROL_CFG_SLOT_B, v25_older, total25, commit25, 7U);
    scramble_ram();
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 30);
    check_loaded_matches_reference(&ref, 31);

    /* ---- 升级后第一次保存：写到另一个槽（B），序号接着旧的往上走，版本 27 ---- */
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 40);
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, header, 8U) == APP_FLASH_SERVICE_OK, 41);
    CHECK(header[4] == APP_CONTROL_CFG_VERSION, 42);
    {
        APP_ControlConfigCommit commit;
        CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B + commit27, (uint8_t *)&commit,
                                        sizeof(commit)) == APP_FLASH_SERVICE_OK, 43);
        CHECK(commit.magic == APP_CONTROL_CFG_COMMIT_MAGIC, 44);
        CHECK(commit.sequence == 9U, 45);   /* 旧的最大是 8 */
    }
    scramble_ram();
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 46);
    check_loaded_matches_reference(&ref, 47);   /* 读的是 v27 那份，内容仍是迁移来的 */

    /* ---- 光流安装参数在 v27 上照常存取 ---- */
    CHECK(DRV_Airframe_SetParam("airframe.flow_mount_yaw_deg", 270.0f) != 0U, 60);
    CHECK(DRV_Airframe_SetParam("airframe.flow_mount_mirror", 1.0f) != 0U, 61);
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 62);   /* -> 槽 A，序号 10 */
    scramble_ram();
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 63);
    CHECK(DRV_Airframe_FlowMountYawDeg(DRV_Airframe_Get()) == 270U, 64);
    CHECK(DRV_Airframe_FlowMountMirror(DRV_Airframe_Get()) == 1U, 65);
    CHECK(memcmp(DRV_Airframe_Get(), &ref, sizeof(APP_ControlAirframeParamsV25)) == 0, 66);

    /* A real v26 record has no XY proof. Loading it clears a stale RAM aid. */
    {
        APP_MagXY_Persisted xy = {0};
        SVC_MAG_HeadingConfig active;
        xy.bias_x_mgauss = -3.725f;
        xy.bias_y_mgauss = 135.432f;
        xy.radius_xy_mgauss = 268.292f;
        xy.frame_contract_version = DRV_FRAME_CONTRACT_VERSION;
        xy.axis_verified = 1U;
        xy.enabled_default = 1U;
        APP_MagXY_ApplyPersisted(&xy);
        fake_flash_reset();
        plant_committed(APP_CONTROL_CFG_SLOT_B, v26_image, total26, commit26, 11U);
        CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 67);
        APP_MagXY_GetConfig(&active);
        CHECK(active.enabled == 0U && active.axis_verified == 0U, 68);
        CHECK(DRV_Airframe_FlowMountYawDeg(DRV_Airframe_Get()) == 90U, 69);
    }

    /* ---- 头里 size 比当前版本还大：不可能是任何一版写的，整槽不予考虑 ---- */
    {
        uint8_t bogus[16];
        const uint16_t huge = (uint16_t)(size27 + 32U);
        memset(bogus, 0xFF, sizeof(bogus));
        memcpy(bogus, v27_image, 8U);
        memcpy(&bogus[6], &huge, sizeof(huge));
        fake_flash_reset();
        plant_committed(APP_CONTROL_CFG_SLOT_B, v25_newer, total25, commit25, 1U);
        CHECK(APP_FlashService_EraseSector(APP_CONTROL_CFG_SLOT_A) == APP_FLASH_SERVICE_OK, 70);
        CHECK(APP_FlashService_WriteData(APP_CONTROL_CFG_SLOT_A, bogus, sizeof(bogus)) ==
              APP_FLASH_SERVICE_OK, 71);
        scramble_ram();
        CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 72);
        check_loaded_matches_reference(&ref, 73);
    }
    REPORT();
}
"""


def test_v25_v26_records_upgrade_to_v27_keeping_every_parameter(tmp_path: Path) -> None:
    fakes = write_fakes(tmp_path, {"cmsis_os2.h": ab.FAKE_CMSIS_OS2_H, "main.h": ab.FAKE_MAIN_H})
    result = build_and_run(
        tmp_path, "flow_mount_cfg",
        CHECK_MACRO + ab.FAKE_FLASH_C + ab.HOST_STUBS_C + HARNESS,
        sources=ab.SOURCES, includes=ab.includes(fakes),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout, result.stdout
    sizes = next(line for line in result.stdout.splitlines() if line.startswith("SIZES "))
    total27, commit27, total26, commit26, total25, commit25 = (
        int(item) for item in sizes.split()[1:]
    )
    # All record members have fixed widths; the frozen v25/v26 offsets stay intact.
    # 当前版本（v29）848 字节、提交字 864；冻结的 v26/v25 仍是 808/832、800/800。
    assert (total27, commit27, total26, commit26, total25, commit25) == (
        848, 864, 808, 832, 800, 800,
    )
