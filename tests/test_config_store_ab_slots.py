"""配置记录的 A/B 双槽与两阶段提交，用真实源码在宿主上跑。

为什么必须是**可执行**测试而不是读源码正则：2026-09-11 这条路径上叠着两个缺陷，
而且互相遮掩——

  1. 配置记录的逻辑地址落在参数区里，却没有映射到任何片内 Flash 扇区，
     `SAVE` 恒返回 INVALID_ARG（实机 `OK save st=4`）；
  2. v20 加机体模型块时，`Save` 写的 `size` 含 airframe，而读回来的校验式不含——
     就算能写进去，自己写的记录也过不了自己的检查，机体模型永远读不回来。

缺陷 1 让缺陷 2 一直没机会暴露。读源码的测试抓不到这种"两端各自看着都对"的错，
只有真的存一遍再读一遍才会红。

harness 用一块 RAM 假 Flash 冒充 APP_FlashService_*，但**记录的编解码、槽选择、
提交字、校验和全部是仓库里真正参与构建的那份 app_control_config_store.c**。
假 Flash 还原了片内 Flash 的两条硬性约束：擦除只能整槽、同一个 32 字节 word
在两次擦除之间只能写一次（H7 的 ECC 就是这样）。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from _micoair_hostfakes import CHECK_MACRO, ROOT, build_and_run, write_fakes


FAKE_FLASH_C = r"""
/*
 * RAM 假 Flash：只实现 app_control_config_store.c 用到的那几个入口。
 * 复刻片内 Flash 的两条硬性约束，否则测不出真正的失效模式：
 *   - 擦除粒度 = 一整个逻辑槽；
 *   - 一次擦除后每个 32 字节 word 只能编程一次（二次编程直接失败）。
 */
#include "app_flash_service.h"
#include <string.h>

#define FAKE_SLOTS   APP_FLASH_SERVICE_PARAM_SLOT_COUNT
#define FAKE_SECTOR  APP_FLASH_SERVICE_SECTOR_SIZE
#define FAKE_WORD    32U

static uint8_t fake_flash[FAKE_SLOTS][FAKE_SECTOR];
static uint8_t fake_word_written[FAKE_SLOTS][FAKE_SECTOR / FAKE_WORD];

/* 下一次写入在第几个 word 之后"断电"。0 = 不断电。 */
static uint32_t fake_power_cut_after_words;
/* 只让提交字那一次写失败（提交字是唯一落在槽内非零偏移的写）。 */
static uint8_t fake_fail_commit;
unsigned fake_erase_count;

void fake_flash_reset(void)
{
    memset(fake_flash, 0xFF, sizeof(fake_flash));
    memset(fake_word_written, 0, sizeof(fake_word_written));
    fake_power_cut_after_words = 0U;
    fake_fail_commit = 0U;
    fake_erase_count = 0U;
}

void fake_flash_fail_commit(uint8_t enable)
{
    fake_fail_commit = enable;
}

void fake_flash_cut_power_after(uint32_t words)
{
    fake_power_cut_after_words = words;
}

static int fake_locate(uint32_t address, uint32_t length,
                       uint32_t *slot, uint32_t *offset)
{
    uint32_t rel;
    if (address < APP_FLASH_SERVICE_PARAM_REGION_START) { return 0; }
    rel = address - APP_FLASH_SERVICE_PARAM_REGION_START;
    *slot = rel / FAKE_SECTOR;
    *offset = rel % FAKE_SECTOR;
    if (*slot >= FAKE_SLOTS) { return 0; }
    if (length > (FAKE_SECTOR - *offset)) { return 0; }
    return 1;
}

APP_FlashService_Status APP_FlashService_ReadData(uint32_t address, uint8_t *data,
                                                  uint32_t length)
{
    uint32_t slot, offset;
    if ((data == NULL) || (length == 0U)) { return APP_FLASH_SERVICE_INVALID_ARG; }
    if (!fake_locate(address, length, &slot, &offset)) {
        return APP_FLASH_SERVICE_INVALID_ARG;
    }
    memcpy(data, &fake_flash[slot][offset], length);
    return APP_FLASH_SERVICE_OK;
}

APP_FlashService_Status APP_FlashService_EraseSector(uint32_t address)
{
    uint32_t slot, offset;
    if (!fake_locate(address, 1U, &slot, &offset)) {
        return APP_FLASH_SERVICE_INVALID_ARG;
    }
    if (offset != 0U) { return APP_FLASH_SERVICE_INVALID_ARG; }
    memset(fake_flash[slot], 0xFF, FAKE_SECTOR);
    memset(fake_word_written[slot], 0, FAKE_SECTOR / FAKE_WORD);
    fake_erase_count++;
    return APP_FLASH_SERVICE_OK;
}

APP_FlashService_Status APP_FlashService_WriteData(uint32_t address,
                                                   const uint8_t *data,
                                                   uint32_t length)
{
    uint32_t slot, offset, written = 0U, words = 0U;
    if ((data == NULL) || (length == 0U)) { return APP_FLASH_SERVICE_INVALID_ARG; }
    if (!fake_locate(address, length, &slot, &offset)) {
        return APP_FLASH_SERVICE_INVALID_ARG;
    }
    if ((offset % FAKE_WORD) != 0U) { return APP_FLASH_SERVICE_INVALID_ARG; }
    if ((fake_fail_commit != 0U) && (offset != 0U)) {
        return APP_FLASH_SERVICE_TIMEOUT;   /* 提交字没写上就掉电 */
    }

    while (written < length) {
        const uint32_t index = (offset + written) / FAKE_WORD;
        uint32_t chunk = length - written;
        if (chunk > FAKE_WORD) { chunk = FAKE_WORD; }

        /* H7 的 ECC：同一个 word 二次编程直接报错，不是"再写一次就好"。 */
        if (fake_word_written[slot][index] != 0U) { return APP_FLASH_SERVICE_ERROR; }
        fake_word_written[slot][index] = 1U;
        memcpy(&fake_flash[slot][offset + written], &data[written], chunk);

        written += FAKE_WORD;
        words++;
        if ((fake_power_cut_after_words != 0U) &&
            (words >= fake_power_cut_after_words)) {
            /* 掉电：已写的留在介质上，调用者收不到任何回音。 */
            return APP_FLASH_SERVICE_TIMEOUT;
        }
    }
    return APP_FLASH_SERVICE_OK;
}
"""


HARNESS = (
    CHECK_MACRO
    + FAKE_FLASH_C
    + r"""
#include "app_control_config_store.h"
#include "app_rc_config.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"

/*
 * 遥控映射用桩：app_rc_config.c 里是 ARM 内联汇编的 seqlock（`dmb 0xF`），
 * 在 x86 宿主上编不过。本测试要证的是**槽选择与机体块的存取**，遥控映射
 * 自己有 tests/test_rc_mapping*.py 把关，这里只需要一个能原样存取的桩。
 */
static APP_RcConfig stub_rc_config;

void app_cmd_rcmap_apply_config(const void *config)
{
    if (config != NULL) { stub_rc_config = *(const APP_RcConfig *)config; }
}

const void *app_cmd_rcmap_config(void)
{
    return &stub_rc_config;
}

extern void fake_flash_reset(void);
extern void fake_flash_cut_power_after(uint32_t words);
extern void fake_flash_fail_commit(uint8_t enable);
extern unsigned fake_erase_count;

static void load_airframe(float mass_g)
{
    DRV_Airframe_Params p;
    memset(&p, 0, sizeof(p));
    p.board_mass_g = mass_g;
    p.gravity_m_s2 = 9.81f;
    p.ixx_kgm2 = 0.051f;
    p.iyy_kgm2 = 0.051f;
    p.izz_kgm2 = 0.005f;
    p.pitch_thrust_lever_arm_m = 0.145f;
    p.roll_thrust_lever_arm_m = 0.145f;
    p.lower_rotor_spin_sense = -1.0f;
    p.max_total_thrust_g = 1595.342f;
    p.thrust_point_z_m = -0.2955f;
    p.derived_auto = 1.0f;
    DRV_Airframe_SetParams(&p);
}

static float airframe_board_mass(void)
{
    float value = 0.0f;
    (void)DRV_Airframe_GetParam("airframe.board_mass_g", &value);
    return value;
}

static void check_roundtrip_carries_the_airframe_block(void)
{
    APP_ControlConfig config;
    APP_ControlConfig loaded;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    config.servo[0].id = 7U;

    load_airframe(75.0f);
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 1);

    /* 把 RAM 里的机体模型抹掉，证明下面读回来的那份真的来自"Flash"。 */
    DRV_Airframe_Clear();
    CHECK(DRV_Airframe_IsValid() == 0U, 2);

    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 3);
    CHECK(loaded.servo[0].id == 7U, 4);
    /* v20 的 size 必须把机体块算进去，否则这里读回来的是零。 */
    CHECK(airframe_board_mass() == 75.0f, 5);
    CHECK(DRV_Airframe_IsValid() == 1U, 6);
}

static void check_saves_alternate_between_the_two_slots(void)
{
    APP_ControlConfig config;
    APP_ControlConfig loaded;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));

    for (unsigned i = 0U; i < 4U; ++i) {
        load_airframe(10.0f + (float)i);
        CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 10 + i);
        DRV_Airframe_Clear();
        CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 20 + i);
        CHECK(airframe_board_mass() == (10.0f + (float)i), 30 + i);
    }

    /*
     * 四次保存必须落在两个槽上轮换。都写同一个槽的话双槽形同虚设：
     * 擦到一半掉电就没有任何一份完好的记录了。
     */
    CHECK(fake_erase_count == 4U, 40);
}

static void check_power_loss_keeps_the_previous_record(void)
{
    APP_ControlConfig config;
    APP_ControlConfig loaded;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    config.servo[0].id = 1U;
    load_airframe(75.0f);
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 50);

    /* 第二次保存写到第三个 word 就断电：主体只写了一小截，提交字没写上。 */
    config.servo[0].id = 2U;
    load_airframe(99.0f);
    fake_flash_cut_power_after(3U);
    CHECK(APP_ControlConfigStore_Save(&config) != APP_FLASH_SERVICE_OK, 51);
    fake_flash_cut_power_after(0U);

    /* 上电：必须回到第一份完好的记录，而不是那半条。 */
    DRV_Airframe_Clear();
    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 52);
    CHECK(loaded.servo[0].id == 1U, 53);
    CHECK(airframe_board_mass() == 75.0f, 54);
}

static void check_body_without_commit_word_is_rejected(void)
{
    APP_ControlConfig config;
    APP_ControlConfig loaded;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    config.servo[0].id = 3U;
    load_airframe(75.0f);
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 60);

    /*
     * 主体写完、提交字没写上（正好卡在两阶段之间断电）。这一槽必须整槽作废：
     * 主体看上去完好，但没人证明它写完了——把它当成好的正是两阶段要防的事。
     */
    config.servo[0].id = 4U;
    load_airframe(42.0f);
    fake_flash_fail_commit(1U);
    CHECK(APP_ControlConfigStore_Save(&config) != APP_FLASH_SERVICE_OK, 60 + 1);
    fake_flash_fail_commit(0U);

    DRV_Airframe_Clear();
    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 61);
    CHECK(loaded.servo[0].id == 3U, 62);
    CHECK(airframe_board_mass() == 75.0f, 63);
}

int main(void)
{
    DRV_COAX_CTRL_Init();
    check_roundtrip_carries_the_airframe_block();
    check_saves_alternate_between_the_two_slots();
    check_power_loss_keeps_the_previous_record();
    check_body_without_commit_word_is_rejected();
    REPORT();
}
"""
)


FAKE_CMSIS_OS2_H = r"""
#ifndef FAKE_CMSIS_OS2_H
#define FAKE_CMSIS_OS2_H
/* app_stabilizer.h 只用到这两个句柄类型的名字，不调用任何 RTOS 函数。 */
typedef void *osSemaphoreId_t;
typedef void *osMessageQueueId_t;
typedef void *osMutexId_t;
typedef void *osThreadId_t;
#endif
"""


def test_config_store_ab_slots(tmp_path: Path) -> None:
    fakes = write_fakes(tmp_path, {"cmsis_os2.h": FAKE_CMSIS_OS2_H})
    result = build_and_run(
        tmp_path,
        "config_ab",
        HARNESS,
        sources=[
            ROOT / "App" / "Src" / "app_control_config_store.c",
            ROOT / "App" / "Src" / "app_control_config_compat.c",
            ROOT / "Driver" / "Src" / "drv_airframe_params.c",
            ROOT / "Driver" / "Src" / "drv_coax_ctrl.c",
            ROOT / "Driver" / "Src" / "drv_position_control.c",
            ROOT / "Driver" / "Src" / "drv_attitude_control.c",
            ROOT / "Driver" / "Src" / "drv_rate_control.c",
        ],
        includes=[
            fakes,
            ROOT / "App" / "Inc",
            ROOT / "Driver" / "Inc",
            ROOT / "Services" / "Inc",
            ROOT / "BSP" / "Inc",
        ],
    )
    assert result.returncode == 0, result.stdout + result.stderr
