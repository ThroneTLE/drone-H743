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


# 配置存储在宿主上要补的外部符号（与"记录怎么存"无关）。tests/test_param_trial.py 复用同一份。
HOST_STUBS_C = r"""
#include "app_control_config_store.h"
#include "app_control_internal.h"
#include "app_led_config.h"
#include "app_rc_config.h"
#include "app_stabilizer.h"
#include "app_magxy.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_mag_calibration.h"
#include "drv_frame_contract.h"
#include "drv_prop_map.h"
#include "bsp_dshot.h"
#include "bsp_dshot_rx.h"

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

/*
 * LED 这一块用**真的** app_led_config.c 和**真的** app_cmd_ledmap.c，一个桩都不打。
 *
 * 以前这里自己写了一份 app_cmd_ledmap_apply_config()，于是下面那两条
 * "旧记录没有 LED 块 → 落回默认"和"存的那份过不了校验 → 落回默认"验的是**桩
 * 自己**：把 App/Src/app_cmd_ledmap.c 里那整支 NULL/校验失败的兜底删掉，本文件
 * 照样全绿。而真机上的症状正是这两条声称要挡的——RAM 里留着上一次的颜色，
 * `LEDMAP?` 报的、灯上亮的、Flash 里存的三者互不一致，零报错。
 *
 * 命令族顺带依赖的几个符号在这里打桩：它们和"记录怎么存"无关。
 */
void APP_Control_QueueText(const char *format, ...)
{
    (void)format;
}

uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    (void)text;
    (void)value;
    return 0U;
}

uint8_t APP_Stabilizer_IsArmed(void)
{
    return 0U;
}

void APP_LED_IdentifyPattern(const DRV_RgbPattern *pattern, uint32_t hold_ms)
{
    (void)pattern;
    (void)hold_ms;
}

uint8_t app_control_internal_commit_config_persist(void)
{
    return (uint8_t)APP_FLASH_SERVICE_OK;
}

/*
 * 桨叶接线这一块同样用**真的** app_cmd_propcal.c，理由和 LED 那一条一模一样：
 * "旧记录没有这一块 → 退回未标定、绝不从 airframe 的退役字段迁移"这句话，
 * 只有跑真源码才验得到。自己打一份桩的话，把那支兜底删掉本文件照样全绿，
 * 而真机上的后果是一份反推出来的旋向顶着"已标定"的名义决定偏航方向。
 *
 * 它顺带依赖的几个符号在这里打桩：它们和"记录怎么存"无关。
 */
const char *app_control_token_value(char **tokens, uint32_t count, const char *key)
{
    (void)tokens; (void)count; (void)key;
    return NULL;
}

uint32_t SVC_Timestamp_Ms(void) { return 0U; }
uint8_t APP_Acceptance_IsActive(void) { return 0U; }
uint8_t APP_ServoCal_IsActive(void) { return 0U; }
uint8_t APP_Ident_IsRunning(void) { return 0U; }
uint8_t APP_ThrustBench_IsActive(void) { return 0U; }
const char *BSP_PWM_EscChannelLabel(uint32_t channel) { (void)channel; return "-"; }

/*
 * app_cmd_propcal.c 的心跳回包会捎带电调回传（`esc_telem=` 那几路）。本测试证的是
 * **记录怎么存**，电调有没有在说话与此无关，所以一律回"没有"：available=0。
 */
void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out)
{
    if (out != NULL) { memset(out, 0, sizeof(*out)); }
}

/* 同理：`PROPCAL escdiag` 会量一下电调信号脚的实际电平。宿主上没有 GPIO，
 * 回"两路都是低"即可——本测试证的是记录怎么存，引脚电平与此无关。 */
uint8_t BSP_DShot_ReadEscPinLevels(void) { return 0U; }

/*
 * 磁力计校准块用**真的** app_cmd_magcal.c，理由和 LED/PropMap 一模一样：
 * "旧记录没有这一块 -> 落回出厂未校准/未验证"这句话，只有跑真源码才验得到。
 * 它顺带依赖的几个符号在这里打桩，和"记录怎么存"无关：
 *   - app_control_parse_f32：本测试只经 app_cmd_magcal_apply_config() 直接摆
 *     数值（同 paint_led / declare_props 的写法），不走完整的 SET 命令文本，
 *     所以给一个能编译的空桩即可。
 *   - BSP_Critical_Enter/Exit：宿主上没有中断可关，用一个可重入深度计数器
 *     占位，形态与 tests/test_battery_runtime.py 等既有宿主测试一致。
 */
uint8_t app_control_parse_f32(const char *text, float *value)
{
    (void)text;
    (void)value;
    return 0U;
}

static int magcal_critical_depth;
uint32_t BSP_Critical_Enter(void) { return (uint32_t)magcal_critical_depth++; }
void BSP_Critical_Exit(uint32_t state) { magcal_critical_depth = (int)state; }

/*
 * MAGCAL? 顺带报一行姿态融合有没有真的在用磁力计（app_stabilizer.c 是它的
 * 真正实现，但那个文件跑不进这份不含 RTOS 的宿主测试）。这里只需要一个
 * 能链接的桩，判据不在本测试范围——姿态融合门控有 tests/
 * test_attitude_fusion_magnetometer.py 把关。
 */
void APP_Stabilizer_GetMagFusionStatus(APP_Stabilizer_MagFusionStatus *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

/* Only the runtime status reader is stubbed. MAGXY config/persistence is real. */
void APP_Stabilizer_GetMagXYStatus(APP_Stabilizer_MagXYStatus *out)
{
    if (out != NULL) { memset(out, 0, sizeof(*out)); }
}
"""


HARNESS = (
    CHECK_MACRO
    + FAKE_FLASH_C
    + HOST_STUBS_C
    + r"""
static const APP_LedConfig *led_now(void)
{
    return (const APP_LedConfig *)app_cmd_ledmap_config();
}

static void reset_led(void)
{
    app_cmd_ledmap_apply_config(NULL);
}

/* 给某条绑定涂一个能认出来的颜色。只能整份交回去——工作副本是命令族的私有静态。 */
static void paint_led(uint8_t bind, uint8_t r, uint8_t g, uint8_t b)
{
    APP_LedConfig c = *led_now();

    if (c.magic != APP_LED_CFG_MAGIC) {
        APP_LedConfig_Defaults(&c);
    }
    c.binding[bind].r = r;
    c.binding[bind].g = g;
    c.binding[bind].b = b;
    c.customized = 1U;
    app_cmd_ledmap_apply_config(&c);
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
    /* 2026-09-27：倾转力臂改由舵机转轴高度减重心算出，转轴必填才能通过体检。 */
    p.servo1_axis_z_m = -0.13f;
    p.servo2_axis_z_m = -0.13f;
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

static void check_roundtrip_carries_the_led_block(void)
{
    APP_ControlConfig config;
    APP_ControlConfig loaded;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    reset_led();

    /* 七个解锁被拒原因各涂一个颜色——作者要的就是这个能力。 */
    for (uint8_t reason = 1U; reason <= APP_LED_BIND_BLOCK_COUNT; ++reason) {
        paint_led(APP_LedConfig_BindingForBlockReason(reason),
                  (uint8_t)(reason * 20U), (uint8_t)(200U - reason * 10U), 5U);
    }
    load_airframe(75.0f);
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 70);

    /* 把 RAM 里那份抹成默认，证明下面读回来的真的来自"Flash"。 */
    reset_led();
    CHECK(led_now()->binding[APP_LED_BIND_BLOCK_BASE].r == 255U, 71);

    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 72);

    /*
     * 这一条就是 v20 那个缺陷的机检：CURRENT_SIZE 漏算 LED 块的话，Save 写的
     * size 和 Load 校验用的 size 对不上，整条记录判无效——而上位机只会看到
     * "SAVE OK"，下次上电颜色全回默认，没有任何报错。
     */
    for (uint8_t reason = 1U; reason <= APP_LED_BIND_BLOCK_COUNT; ++reason) {
        const APP_LedBinding *b =
            &led_now()->binding[APP_LedConfig_BindingForBlockReason(reason)];
        CHECK(b->r == (uint8_t)(reason * 20U), 73);
        CHECK(b->g == (uint8_t)(200U - reason * 10U), 74);
    }
    CHECK(led_now()->customized == 1U, 75);
    /* 同一条记录里的别的块不能被挤坏。 */
    CHECK(airframe_board_mass() == 75.0f, 76);
}

static void check_a_record_without_the_led_block_falls_back_to_defaults(void)
{
    /*
     * 旧固件写的记录（v20 及更早）里没有 LED 块。读它时必须**显式**把 LED 表
     * 落回默认，不能什么都不做——什么都不做会让 RAM 里留着上一次的颜色，
     * 于是 LEDMAP? 报的和 Flash 里存的不是一回事，而用户据此认为"存进去了"。
     * airframe 当年漏的就是这半步。
     *
     * 这里直接调用迁移入口来验这半步：真造一条 v20 记录需要冻结整份旧结构体，
     * 而那份结构体的字节布局已经由 config_read_v20 自己的 offsetof 校验钉住。
     */
    reset_led();
    paint_led(APP_LED_BIND_READY, 1U, 2U, 3U);
    CHECK(led_now()->binding[APP_LED_BIND_READY].r == 1U, 80);

    app_cmd_ledmap_apply_config(NULL);
    CHECK(led_now()->binding[APP_LED_BIND_READY].r == 0U, 81);
    CHECK(led_now()->binding[APP_LED_BIND_READY].g == 255U, 82);
    CHECK(led_now()->customized == 0U, 83);
}

static void check_a_corrupt_led_block_does_not_take_the_record_down(void)
{
    /*
     * 存进去的 LED 块如果过不了校验（手改过 Flash、或者将来 schema 变了），
     * 落回默认色即可——不该让整条记录连带增益和机体模型一起作废。
     * 灯配错了是难看，机体模型丢了是飞不起来。
     */
    APP_LedConfig bad;

    APP_LedConfig_Defaults(&bad);
    bad.schema = 0xFFFFU;
    app_cmd_ledmap_apply_config(&bad);
    CHECK(led_now()->schema == APP_LED_CFG_SCHEMA, 90);
    CHECK(led_now()->binding[APP_LED_BIND_ARMED].r == 255U, 91);
}

/* ─────────────────────────────────── 桨叶与电机接线标定块（v22 新增） */

/* 走命令族的 apply 入口，和飞控读 Flash 时是同一条路。 */
static void declare_props(uint8_t upper_channel)
{
    DRV_PropMap map;

    DRV_PropMap_Defaults(&map);
    map.channel[upper_channel - 1U].role = (uint8_t)DRV_PROP_ROLE_UPPER;
    map.channel[upper_channel - 1U].spin_sense = DRV_PROP_SPIN_CCW;
    map.channel[2U - upper_channel].role = (uint8_t)DRV_PROP_ROLE_LOWER;
    map.channel[2U - upper_channel].spin_sense = DRV_PROP_SPIN_CW;
    map.calibrated = 1U;
    app_cmd_propcal_apply_config(&map);
}

static void check_roundtrip_carries_the_prop_block(void)
{
    APP_ControlConfig config;
    APP_ControlConfig loaded;
    DRV_PropMap readback;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    load_airframe(75.0f);

    /* 反着接：通道 2 才是上桨。存进去再读回来必须还是反着的——存丢了的话，
     * 上电后飞控会把上桨的推力发给下桨，而界面上什么都不会变。 */
    declare_props(2U);
    CHECK(DRV_PropMap_IsCalibrated() == 1U, 102);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 2U, 103);
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 104);

    /* 把 RAM 抹成未标定，证明下面读回来的真的来自"Flash"。 */
    app_cmd_propcal_apply_config(NULL);
    CHECK(DRV_PropMap_IsCalibrated() == 0U, 105);

    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 106);
    CHECK(DRV_PropMap_IsCalibrated() == 1U, 107);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 2U, 108);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_LOWER) == 1U, 109);
    CHECK(DRV_PropMap_YawTorquePolarity() == 1.0f, 110);
    CHECK(DRV_PropMap_ReadActive(&readback) == 1U, 111);
    CHECK(readback.calibrated == 1U, 112);
    /* 同一条记录里的别的块不能被挤坏。 */
    CHECK(airframe_board_mass() == 75.0f, 113);
    CHECK(led_now()->binding[APP_LED_BIND_ARMED].r == 255U, 114);
}

static void check_a_record_without_the_prop_block_is_uncalibrated(void)
{
    /*
     * 旧固件写的记录（v21 及更早）里没有这一块。**绝不能**从 airframe 的
     * retired_lower_rotor_spin_sense 迁移过来：那个值是从调参现象反推的，
     * 搬进来就等于让一份从没量过的旋向顶着"已标定"的名义决定偏航方向。
     * 正确行为是退回未标定，让解锁被挡住，逼一次真的通电确认。
     */
    declare_props(1U);
    CHECK(DRV_PropMap_IsCalibrated() == 1U, 120);

    app_cmd_propcal_apply_config(NULL);
    CHECK(DRV_PropMap_IsCalibrated() == 0U, 121);
    CHECK(DRV_PropMap_YawTorquePolarity() == 0.0f, 122);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 0U, 123);
}

static void check_a_corrupt_prop_block_does_not_take_the_record_down(void)
{
    /* 存进去的那份过不了校验时落回未标定即可，不该让整条记录连带增益和
     * 机体模型一起作废——但也绝不能"猜一个"继续飞。 */
    DRV_PropMap bad;

    DRV_PropMap_Defaults(&bad);
    bad.calibrated = 1U;
    bad.channel[0].spin_sense = DRV_PROP_SPIN_CW;
    bad.channel[1].spin_sense = DRV_PROP_SPIN_CW;   /* 共轴不可能同向 */
    app_cmd_propcal_apply_config(&bad);
    CHECK(DRV_PropMap_IsCalibrated() == 0U, 130);
}

static const DRV_MAG_Calibration *mag_now(void)
{
    return (const DRV_MAG_Calibration *)app_cmd_magcal_config();
}

static void declare_mag(float bias_x, uint8_t axis_verified,
                        uint32_t frame_contract_version)
{
    DRV_MAG_Calibration cal;

    memset(&cal, 0, sizeof(cal));
    cal.calibrated = 1U;
    cal.axis_verified = axis_verified;
    cal.frame_contract_version = frame_contract_version;
    cal.hard_iron_bias_mgauss[0] = bias_x;
    cal.hard_iron_bias_mgauss[1] = 2.0f;
    cal.hard_iron_bias_mgauss[2] = 3.0f;
    cal.soft_iron_matrix[0][0] = 1.0f;
    cal.soft_iron_matrix[1][1] = 1.0f;
    cal.soft_iron_matrix[2][2] = 1.0f;
    app_cmd_magcal_apply_config(&cal);
}

static void check_roundtrip_carries_the_mag_block(void)
{
    APP_ControlConfig config;
    APP_ControlConfig loaded;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    load_airframe(75.0f);

    declare_mag(11.0f, DRV_MAG_CAL_AXIS_VERIFIED, 1U);
    CHECK(mag_now()->calibrated == 1U, 140);
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 141);

    /* 把 RAM 抹成未校准，证明下面读回来的真的来自"Flash"。 */
    app_cmd_magcal_apply_config(NULL);
    CHECK(mag_now()->calibrated == 0U, 142);

    memset(&loaded, 0, sizeof(loaded));
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 143);
    CHECK(mag_now()->calibrated == 1U, 144);
    CHECK(mag_now()->axis_verified == DRV_MAG_CAL_AXIS_VERIFIED, 145);
    CHECK(mag_now()->frame_contract_version == 1U, 146);
    CHECK(mag_now()->hard_iron_bias_mgauss[0] == 11.0f, 147);
    /* 同一条记录里的别的块不能被挤坏。 */
    CHECK(airframe_board_mass() == 75.0f, 148);
    CHECK(led_now()->binding[APP_LED_BIND_ARMED].r == 255U, 149);
}

static void check_a_record_without_the_mag_block_is_uncalibrated(void)
{
    /*
     * 旧固件（v22 及更早）写的记录里没有这一块——这本来就是磁力计接入姿态
     * 融合之前的真实状态：从未校准过。必须显式落回出厂未校准/未验证，不能
     * 什么都不做，否则 RAM 里留着上一次的系数，MAGCAL? 报的和 Flash 里存的
     * 不是一回事，姿态融合却已经在用旧系数。
     */
    declare_mag(99.0f, DRV_MAG_CAL_AXIS_VERIFIED, 1U);
    CHECK(mag_now()->calibrated == 1U, 150);

    app_cmd_magcal_apply_config(NULL);
    CHECK(mag_now()->calibrated == 0U, 151);
    CHECK(mag_now()->axis_verified == DRV_MAG_CAL_AXIS_UNVERIFIED, 152);
    CHECK(mag_now()->hard_iron_bias_mgauss[0] == 0.0f, 153);
    CHECK(mag_now()->soft_iron_matrix[0][0] == 1.0f, 154);
}

static void check_a_corrupt_mag_block_does_not_take_the_record_down(void)
{
    /* 存进去的那份过不了校验（行列式越界）时落回未校准即可，不该让整条
     * 记录连带增益和机体模型一起作废——但也绝不能"猜一个"继续融合。 */
    DRV_MAG_Calibration bad;

    memset(&bad, 0, sizeof(bad));
    bad.calibrated = 1U;
    bad.soft_iron_matrix[0][0] = 100.0f;
    bad.soft_iron_matrix[1][1] = 100.0f;
    bad.soft_iron_matrix[2][2] = 100.0f; /* det = 1e6，越过 DET_MAX=10 */
    app_cmd_magcal_apply_config(&bad);
    CHECK(mag_now()->calibrated == 0U, 160);
}

static void check_magxy_v27_roundtrip_and_fail_closed(void)
{
    APP_ControlConfig config, loaded;
    APP_MagXY_Persisted xy = {0};
    SVC_MAG_HeadingConfig active;

    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    load_airframe(75.0f);
    xy.bias_x_mgauss = -3.725f;
    xy.bias_y_mgauss = 135.432f;
    xy.radius_xy_mgauss = 268.292f;
    xy.frame_contract_version = DRV_FRAME_CONTRACT_VERSION;
    xy.axis_verified = 1U;
    xy.enabled_default = 1U;
    APP_MagXY_ApplyPersisted(&xy);
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 170);

    APP_MagXY_ApplyPersisted(NULL);
    APP_MagXY_GetConfig(&active);
    CHECK(active.enabled == 0U, 171);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 172);
    APP_MagXY_GetConfig(&active);
    CHECK(active.enabled == 1U && active.axis_verified == 1U, 173);
    CHECK(active.bias_x_mgauss == xy.bias_x_mgauss, 174);
    CHECK(active.bias_y_mgauss == xy.bias_y_mgauss, 175);
    CHECK(airframe_board_mass() == 75.0f, 176);

    /* A torn v27 update must retain the prior enabled slot. */
    xy.enabled_default = 0U;
    APP_MagXY_ApplyPersisted(&xy);
    fake_flash_cut_power_after(3U);
    CHECK(APP_ControlConfigStore_Save(&config) != APP_FLASH_SERVICE_OK, 179);
    fake_flash_cut_power_after(0U);
    APP_MagXY_ApplyPersisted(NULL);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 180);
    APP_MagXY_GetConfig(&active);
    CHECK(active.enabled == 1U, 181);

    /* Contract changes invalidate the old physical-axis proof. */
    xy.enabled_default = 1U;
    xy.frame_contract_version = DRV_FRAME_CONTRACT_VERSION + 1U;
    APP_MagXY_ApplyPersisted(&xy);
    APP_MagXY_GetConfig(&active);
    CHECK(active.enabled == 0U, 177);
    xy.frame_contract_version = DRV_FRAME_CONTRACT_VERSION;
    xy.radius_xy_mgauss = 0.0f;
    APP_MagXY_ApplyPersisted(&xy);
    APP_MagXY_GetConfig(&active);
    CHECK(active.enabled == 0U && active.axis_verified == 0U, 178);
}

int main(void)
{
    DRV_COAX_CTRL_Init();
    check_roundtrip_carries_the_airframe_block();
    check_roundtrip_carries_the_led_block();
    check_a_record_without_the_led_block_falls_back_to_defaults();
    check_a_corrupt_led_block_does_not_take_the_record_down();
    check_roundtrip_carries_the_prop_block();
    check_a_record_without_the_prop_block_is_uncalibrated();
    check_a_corrupt_prop_block_does_not_take_the_record_down();
    check_roundtrip_carries_the_mag_block();
    check_a_record_without_the_mag_block_is_uncalibrated();
    check_a_corrupt_mag_block_does_not_take_the_record_down();
    check_magxy_v27_roundtrip_and_fail_closed();
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


FAKE_MAIN_H = r"""
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H
/* app_flash_service.h → drv_gd25q32.h 只用到这两个类型的**名字**，从不解引用。 */
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
#endif
"""


# 参与编译的真实源码（tests/test_param_trial.py 复用同一份）。
SOURCES = [
    ROOT / "App" / "Src" / "app_control_config_store.c",
    ROOT / "App" / "Src" / "app_magxy.c",
    ROOT / "Services" / "Src" / "svc_mag_heading.c",
    ROOT / "App" / "Src" / "app_control_config_compat.c",
    ROOT / "App" / "Src" / "app_param_trial.c",
    ROOT / "App" / "Src" / "app_led_config.c",
    ROOT / "App" / "Src" / "app_cmd_ledmap.c",
    ROOT / "Driver" / "Src" / "drv_airframe_params.c",
    ROOT / "Driver" / "Src" / "drv_prop_map.c",
    ROOT / "App" / "Src" / "app_cmd_propcal.c",
    ROOT / "App" / "Src" / "app_prop_spin.c",
    ROOT / "App" / "Src" / "app_cmd_magcal.c",
    ROOT / "Driver" / "Src" / "drv_mag_calibration.c",
    ROOT / "Driver" / "Src" / "drv_coax_ctrl.c",
    ROOT / "Driver" / "Src" / "drv_position_control.c",
    ROOT / "Driver" / "Src" / "drv_attitude_control.c",
    ROOT / "Driver" / "Src" / "drv_rate_control.c",
]


def includes(fakes: Path) -> list[Path]:
    return [
        fakes,
        ROOT / "App" / "Inc",
        ROOT / "Driver" / "Inc",
        ROOT / "Services" / "Inc",
        ROOT / "BSP" / "Inc",
    ]


def test_config_store_ab_slots(tmp_path: Path) -> None:
    fakes = write_fakes(tmp_path, {"cmsis_os2.h": FAKE_CMSIS_OS2_H,
                                   "main.h": FAKE_MAIN_H})
    result = build_and_run(
        tmp_path,
        "config_ab",
        HARNESS,
        sources=SOURCES,
        includes=includes(fakes),
    )
    assert result.returncode == 0, result.stdout + result.stderr
