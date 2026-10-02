#include "app_control_config_store.h"
#include "app_led_config.h"
#include "app_magxy.h"

#include "app_control_config_compat.h"
#include "app_control_internal.h"
#include "app_param_trial.h"
#include "app_rc_config.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_mag_calibration.h"
#include "drv_prop_map.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define APP_CONTROL_CONFIG_PI 3.14159265358979323846f
#define APP_CONTROL_TILT_LIMIT_DEFAULT_RAD 0.4886922f
#define APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD 0.31415927f
#define APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD 0.43633231f
#define APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD 0.001f

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
} APP_ControlFlashHeader;


/*
 * `has_airframe` 是 v20 加的：机体模型块只存在于当前版本的记录里。
 * 它必须排在 checksum **之前**、其余块之后——旧版本记录的字节布局因此原封不动，
 * 迁移读取器仍能按各自的 offsetof 校验通过。`has_mag` 是 v23 加的，同一条规则：
 * 排在 checksum 之前、其余块之后。
 */
#define APP_CONTROL_RECORD_TYPE(name, tunable_type, has_rc, has_airframe, has_led, \
                                has_prop, has_mag, has_shaping) \
    typedef struct { \
        uint32_t magic; \
        uint16_t version; \
        uint16_t size; \
        APP_ControlConfig config; \
        tunable_type coax_tunables; \
        has_rc \
        has_airframe \
        has_led \
        has_prop \
        has_mag \
        has_shaping \
        uint32_t checksum; \
    } name

/* v26 layout remains frozen for migration and A/B slot sequence checks. */
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV26,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        DRV_Airframe_Params airframe;,
                        APP_LedConfig led;,
                        DRV_PropMap prop;,
                        DRV_MAG_Calibration mag;,
                        APP_ControlCoaxShapingParams shaping;);
/* v27 appends XY after shaping, before checksum. Older layouts never move. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxTunableParams coax_tunables;
    APP_RcConfig rc_config;
    DRV_Airframe_Params airframe;
    APP_LedConfig led;
    DRV_PropMap prop;
    DRV_MAG_Calibration mag;
    APP_ControlCoaxShapingParams shaping;
    APP_MagXY_Persisted magxy;
    uint32_t checksum;
} APP_ControlFlashRecordV27;
/* v28 appends the vertical-channel block (hover thrust, z fusion) after XY, before checksum. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxTunableParams coax_tunables;
    APP_RcConfig rc_config;
    DRV_Airframe_Params airframe;
    APP_LedConfig led;
    DRV_PropMap prop;
    DRV_MAG_Calibration mag;
    APP_ControlCoaxShapingParams shaping;
    APP_MagXY_Persisted magxy;
    APP_ControlZChannelParams zchan;
    uint32_t checksum;
} APP_ControlFlashRecordV28;
/* v29 appends the flight-limit block (alt max, manual tilt max, yaw stick rate) before checksum. */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    APP_ControlConfig config;
    APP_ControlCoaxTunableParams coax_tunables;
    APP_RcConfig rc_config;
    DRV_Airframe_Params airframe;
    APP_LedConfig led;
    DRV_PropMap prop;
    DRV_MAG_Calibration mag;
    APP_ControlCoaxShapingParams shaping;
    APP_MagXY_Persisted magxy;
    APP_ControlZChannelParams zchan;
    APP_ControlFlightLimitParams flightlim;
    uint32_t checksum;
} APP_ControlFlashRecord;
/*
 * v25 = v26 形态，但机体块是冻结的 v25 块（没有光流安装两项）。
 * v20～v24 的机体块同样是这份冻结布局：DRV_Airframe_Params 在 v26 才第一次变长，
 * 旧记录必须按它们写下时的字节布局校验。
 */
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV25,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        APP_ControlAirframeParamsV25 airframe;,
                        APP_LedConfig led;,
                        DRV_PropMap prop;,
                        DRV_MAG_Calibration mag;,
                        APP_ControlCoaxShapingParams shaping;);
/* v24 = v25 形态，但整形块是冻结的 v24 块（没有第二级出口陷波）。 */
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV24,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        APP_ControlAirframeParamsV25 airframe;,
                        APP_LedConfig led;,
                        DRV_PropMap prop;,
                        DRV_MAG_Calibration mag;,
                        APP_ControlCoaxShapingParamsV24 shaping;);
/* v23 = v24 形态减掉指令整形/出口陷波块。冻结它是为了让迁移读取器按原字节布局校验。 */
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV23,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        APP_ControlAirframeParamsV25 airframe;,
                        APP_LedConfig led;,
                        DRV_PropMap prop;,
                        DRV_MAG_Calibration mag;, );
/* v22 = 再减掉磁力计校准块。 */
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV22,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        APP_ControlAirframeParamsV25 airframe;,
                        APP_LedConfig led;,
                        DRV_PropMap prop;, , );
/* v21 = 再减掉桨叶标定块。 */
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV21,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        APP_ControlAirframeParamsV25 airframe;,
                        APP_LedConfig led;, , , );
/* v20 = 再减掉 LED 块。 */
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV20,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;,
                        APP_ControlAirframeParamsV25 airframe;, , , , );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV19,
                        APP_ControlCoaxTunableParams,
                        APP_RcConfig rc_config;, , , , , );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV18,
                        APP_ControlCoaxTunableParamsV18,
                        APP_RcConfig rc_config;, , , , , );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV17,
                        APP_ControlCoaxTunableParamsV17,
                        APP_RcConfig rc_config;, , , , , );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV16,
                        APP_ControlCoaxTunableParamsV17, , , , , , );
APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecordV15,
                        APP_ControlCoaxTunableParamsV15, , , , , , );

/* ─────────────────────────────────────────────── A/B 双槽与提交字 */

#define APP_CONTROL_CFG_SLOT_COUNT 2U
#define APP_CONTROL_CFG_FLASH_WORD 32U

static const uint32_t config_slots[APP_CONTROL_CFG_SLOT_COUNT] = {
    APP_CONTROL_CFG_SLOT_A,
    APP_CONTROL_CFG_SLOT_B,
};

/*
 * 提交字必须落在记录主体**之后**的独立 flash word 上。
 * 片内 Flash 一次擦除后每个 word 只能编程一次，主体和提交字压在同一个 word 上
 * 会让第二次编程直接报 ECC 错。这里按 32 字节上取整。
 */
/* 一条 record_bytes 字节的记录之后，第一个 32 字节 flash word 的起点。 */
#define APP_CONTROL_CFG_COMMIT_OFFSET_OF(record_bytes)                          \
    (((((uint32_t)(record_bytes)) + APP_CONTROL_CFG_FLASH_WORD - 1U) /          \
      APP_CONTROL_CFG_FLASH_WORD) * APP_CONTROL_CFG_FLASH_WORD)

/* 本版固件写提交字的位置。读取时用 config_commit_offset()，见下面那段说明。 */
#define APP_CONTROL_CFG_COMMIT_OFFSET \
    APP_CONTROL_CFG_COMMIT_OFFSET_OF(sizeof(APP_ControlFlashRecord))

_Static_assert((APP_CONTROL_CFG_COMMIT_OFFSET % APP_CONTROL_CFG_FLASH_WORD) == 0U,
               "commit word must start on a 32-byte flash word");
_Static_assert((APP_CONTROL_CFG_COMMIT_OFFSET + sizeof(APP_ControlConfigCommit)) <=
                   APP_FLASH_SERVICE_SECTOR_SIZE,
               "record plus commit word must fit inside one logical slot");
_Static_assert(APP_CONTROL_CFG_SLOT_A != APP_CONTROL_CFG_SLOT_B,
               "the two config slots must be distinct sectors");

/*
 * 提交字的位置随记录大小走，**升版本时它会挪**。
 *
 * v24 → v25 只多 8 字节，恰好还落在同一个 32 字节 word 里（两版都是 800）。v25 → v26
 * 机体块尾部又多 8 字节（光流安装两项），记录变成 808 字节，提交字挪到 832。
 *
 * 以前读取时一律按**当前**记录大小去找提交字。那样升级后第一次上电，会到 832 去找
 * v25 记录的提交字——找不到，两个槽都退成"无提交字的旧格式"（序号都是 1），分不出
 * 哪个新，按槽 A 优先可能读回较旧的那一份：用户存过的最近一次修改就这么悄悄丢了，
 * 之后第一次保存还会把它盖掉。
 *
 * 所以读取时按**那条记录自己头里的 size** 算提交字位置（config_commit_offset）。每一版
 * 固件写提交字用的都是"它那一版记录的大小上取整到 32 字节"，而记录大小 = 8 字节头 +
 * size + 4 字节校验和——下面对每一版记录类型钉住"中间和尾部没有填充"，所以按 size
 * 反推出的位置与当年写下的位置逐字节相同。不维护"哪一版在哪"的第二张表。
 * v26 → v27 在记录尾部加 20 字节 XY 块，记录 808 → 828 字节，提交字仍在 832。
 * v27 → v28 再加 8 字节竖直通道块，记录 828 → 836 字节，提交字挪到 864。
 * v28 → v29 再加 12 字节飞行限幅块，记录 836 → 848 字节，提交字仍在 864。
 */
#define APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(type)                                   \
    _Static_assert((offsetof(type, config) == sizeof(APP_ControlFlashHeader)) &&    \
                       (sizeof(type) == (offsetof(type, checksum) + sizeof(uint32_t))) && \
                       (sizeof(type) <= sizeof(APP_ControlFlashRecord)),              \
                   #type ": header + body + checksum, no padding, never larger than current")
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecord);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV28);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV27);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV26);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV25);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV24);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV23);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV22);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV21);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV20);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV19);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV18);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV17);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV16);
APP_CONTROL_CFG_ASSERT_RECORD_LAYOUT(APP_ControlFlashRecordV15);
/* v26 与 v25 记录只差机体块尾部那两个 float。 */
_Static_assert(sizeof(APP_ControlFlashRecordV26) ==
                   sizeof(APP_ControlFlashRecordV25) + (2U * sizeof(float)),
               "v26 record = v25 record + the two flow mount fields");
_Static_assert(sizeof(APP_ControlFlashRecordV27) ==
                   sizeof(APP_ControlFlashRecordV26) + sizeof(APP_MagXY_Persisted),
               "v27 record = v26 record + independent MAGXY block");
_Static_assert(sizeof(APP_ControlFlashRecordV28) ==
                   sizeof(APP_ControlFlashRecordV27) + sizeof(APP_ControlZChannelParams),
               "v28 record = v27 record + vertical-channel block");
_Static_assert(sizeof(APP_ControlFlashRecord) ==
                   sizeof(APP_ControlFlashRecordV28) + sizeof(APP_ControlFlightLimitParams),
               "v29 record = v28 record + flight-limit block");
_Static_assert(sizeof(APP_ControlFlightLimitParams) == (3U * sizeof(float)),
               "flight-limit block is exactly alt_max_m + manual_tilt_max_rad + yaw_stick_rate_rad_s");
_Static_assert(APP_CONTROL_CFG_COMMIT_OFFSET_OF(sizeof(APP_ControlFlashRecord)) ==
                   APP_CONTROL_CFG_COMMIT_OFFSET_OF(sizeof(APP_ControlFlashRecordV28)),
               "v29 kept the v28 commit word offset");
_Static_assert(sizeof(APP_ControlZChannelParams) == (2U * sizeof(float)),
               "vertical-channel block is exactly hover_thrust_n + z_vel_fusion");
_Static_assert(APP_CONTROL_CFG_COMMIT_OFFSET_OF(sizeof(APP_ControlFlashRecordV25)) ==
                   APP_CONTROL_CFG_COMMIT_OFFSET_OF(sizeof(APP_ControlFlashRecordV24)),
               "v25 kept the v24 commit word offset");

/* 当前版本记录主体（config 起到 checksum 前）的字节数，任何一版的 size 都不会比它大。 */
#define APP_CONTROL_CFG_MAX_BODY_SIZE                                          \
    ((uint32_t)(sizeof(APP_ControlFlashRecord) - sizeof(APP_ControlFlashHeader) - \
                sizeof(uint32_t)))

/* 头里写着 body_size 的那条记录，当年写提交字的位置。 */
static uint32_t config_commit_offset(uint16_t body_size)
{
    return APP_CONTROL_CFG_COMMIT_OFFSET_OF((uint32_t)sizeof(APP_ControlFlashHeader) +
                                            (uint32_t)body_size +
                                            (uint32_t)sizeof(uint32_t));
}

static uint32_t config_checksum(const uint8_t *data, uint32_t length);

/*
 * 一个槽的"新旧序号"，同时兼作有效性判据：
 *   返回 0  —— 槽里没有可用记录（连 magic 都不对），不予考虑；
 *   返回 1  —— 有记录但**没有提交字**。这是旧的单槽格式（或 NOR 时代写下的），
 *              按最旧处理，让任何一次新保存都能盖过它；
 *   返回 >1 —— 提交字有效，值为 sequence + 1。
 *
 * 提交字里带主体校验和：主体写了一半掉电、提交字却莫名其妙有效的情况会在这里
 * 被挡住。宁可判这个槽无效、回到另一个槽，也不要把半条记录当成好的。
 */
static uint32_t config_slot_sequence(uint32_t slot)
{
    APP_ControlFlashHeader header;
    APP_ControlConfigCommit commit;
    APP_ControlFlashRecord record;

    if (APP_FlashService_ReadData(slot, (uint8_t *)&header,
                                  sizeof(header)) != APP_FLASH_SERVICE_OK) {
        return 0U;
    }
    if (header.magic != APP_CONTROL_CFG_MAGIC) {
        return 0U;
    }
    /*
     * 记录只增不减，size 比当前版本的主体还大就不可能是任何一版写下的；按它去找
     * 提交字、算校验和都会越过下面这个 record 读出界。
     */
    if ((uint32_t)header.size > APP_CONTROL_CFG_MAX_BODY_SIZE) {
        return 0U;
    }

    /* 提交字按这条记录自己的大小找——升版本后旧记录的提交字不在当前版本的位置上。 */
    if (APP_FlashService_ReadData(slot + config_commit_offset(header.size),
                                  (uint8_t *)&commit,
                                  sizeof(commit)) != APP_FLASH_SERVICE_OK) {
        return 1U;
    }
    if (commit.magic != APP_CONTROL_CFG_COMMIT_MAGIC) {
        return 1U;
    }

    if (APP_FlashService_ReadData(slot, (uint8_t *)&record,
                                  sizeof(record)) != APP_FLASH_SERVICE_OK) {
        return 0U;
    }
    if (config_checksum((const uint8_t *)&record.config, header.size) !=
        commit.body_checksum) {
        return 0U;
    }

    /* +2：0 留给"无记录"，1 留给"无提交字的旧格式"。 */
    return commit.sequence + 2U;
}

/* 当前应当读取的槽。两个都没有时返回槽 A，让上层照旧走"没有有效记录"。 */
static uint32_t config_active_slot(void)
{
    uint32_t best_slot = config_slots[0];
    uint32_t best_rank = 0U;

    for (uint32_t i = 0U; i < APP_CONTROL_CFG_SLOT_COUNT; ++i) {
        const uint32_t rank = config_slot_sequence(config_slots[i]);

        if (rank > best_rank) {
            best_rank = rank;
            best_slot = config_slots[i];
        }
    }
    return best_slot;
}

static uint32_t config_checksum(const uint8_t *data, uint32_t length)
{
    uint32_t sum = 0xA5A55A5AUL;

    for (uint32_t index = 0U; index < length; ++index) {
        sum = (sum << 5U) | (sum >> 27U);
        sum ^= data[index];
        sum += 0x9E3779B9UL;
    }
    return sum;
}

void APP_ControlConfigStore_CaptureTunables(APP_ControlCoaxTunableParams *out)
{
    DRV_COAX_CTRL_Params params;

    if (out == NULL) {
        return;
    }
    DRV_COAX_CTRL_GetParams(&params);
    out->pos_x_kp = params.position.pos_kp[0];
    out->pos_y_kp = params.position.pos_kp[1];
    out->pos_z_kp = params.position.pos_kp[2];
    out->pos_xy_vel_max_m_s = params.position.xy_speed_limit_m_s;
    out->pos_z_vel_up_max_m_s = params.position.z_speed_limit_up_m_s;
    out->pos_z_vel_down_max_m_s = params.position.z_speed_limit_down_m_s;
    out->vel_x_kp = params.position.vel_kp[0];
    out->vel_y_kp = params.position.vel_kp[1];
    out->vel_z_kp = params.position.vel_kp[2];
    out->vel_x_ki = params.position.vel_ki[0];
    out->vel_y_ki = params.position.vel_ki[1];
    out->vel_z_ki = params.position.vel_ki[2];
    out->vel_x_kd = params.position.vel_kd[0];
    out->vel_y_kd = params.position.vel_kd[1];
    out->vel_z_kd = params.position.vel_kd[2];
    out->vel_x_i_limit_m_s2 = params.position.vel_integrator_limit[0];
    out->vel_y_i_limit_m_s2 = params.position.vel_integrator_limit[1];
    out->vel_z_i_limit_m_s2 = params.position.vel_integrator_limit[2];
    out->accel_lpf_cutoff_hz = params.position.accel_lpf_cutoff_hz;
    out->accel_xy_max_m_s2 = params.position.xy_accel_limit_m_s2;
    out->accel_z_up_max_m_s2 = params.position.z_accel_limit_up_m_s2;
    out->accel_z_down_max_m_s2 = params.position.z_accel_limit_down_m_s2;
    out->att_roll_kp = params.attitude.att_kp[0];
    out->att_pitch_kp = params.attitude.att_kp[1];
    out->att_yaw_kp = params.attitude.att_kp[2];
    out->roll_rate_limit_rad_s = params.attitude.rate_limit_rad_s[0];
    out->pitch_rate_limit_rad_s = params.attitude.rate_limit_rad_s[1];
    out->yaw_rate_limit_rad_s = params.attitude.rate_limit_rad_s[2];
    out->rate_roll_kp = params.rate.kp[0];
    out->rate_pitch_kp = params.rate.kp[1];
    out->rate_yaw_kp = params.rate.kp[2];
    out->rate_roll_ki = params.rate.ki[0];
    out->rate_pitch_ki = params.rate.ki[1];
    out->rate_yaw_ki = params.rate.ki[2];
    out->rate_roll_kd = params.rate.kd[0];
    out->rate_pitch_kd = params.rate.kd[1];
    out->rate_yaw_kd = params.rate.kd[2];
    out->rate_roll_i_limit_n_m = params.rate.integrator_limit[0];
    out->rate_pitch_i_limit_n_m = params.rate.integrator_limit[1];
    out->rate_yaw_i_limit_n_m = params.rate.integrator_limit[2];
    out->angular_accel_lpf_cutoff_hz =
        params.rate.alpha_lpf_cutoff_rad_s / (2.0f * APP_CONTROL_CONFIG_PI);
    out->rate_roll_ff = params.rate.ff_gain[0];
    out->rate_pitch_ff = params.rate.ff_gain[1];
    out->rate_yaw_ff = params.rate.ff_gain[2];
    out->tilt_limit_rad = params.tilt_limit_rad;
    out->vel_loop_enable = params.vel_loop_enable;
}

static void config_apply_tunables(const APP_ControlCoaxTunableParams *in)
{
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetDefaultParams(&params);
    params.position.pos_kp[0] = in->pos_x_kp;
    params.position.pos_kp[1] = in->pos_y_kp;
    params.position.pos_kp[2] = in->pos_z_kp;
    params.position.xy_speed_limit_m_s = in->pos_xy_vel_max_m_s;
    params.position.z_speed_limit_up_m_s = in->pos_z_vel_up_max_m_s;
    params.position.z_speed_limit_down_m_s = in->pos_z_vel_down_max_m_s;
    params.position.vel_kp[0] = in->vel_x_kp;
    params.position.vel_kp[1] = in->vel_y_kp;
    params.position.vel_kp[2] = in->vel_z_kp;
    params.position.vel_ki[0] = in->vel_x_ki;
    params.position.vel_ki[1] = in->vel_y_ki;
    params.position.vel_ki[2] = in->vel_z_ki;
    params.position.vel_kd[0] = in->vel_x_kd;
    params.position.vel_kd[1] = in->vel_y_kd;
    params.position.vel_kd[2] = in->vel_z_kd;
    params.position.vel_integrator_limit[0] = in->vel_x_i_limit_m_s2;
    params.position.vel_integrator_limit[1] = in->vel_y_i_limit_m_s2;
    params.position.vel_integrator_limit[2] = in->vel_z_i_limit_m_s2;
    params.position.accel_lpf_cutoff_hz = in->accel_lpf_cutoff_hz;
    params.position.xy_accel_limit_m_s2 = in->accel_xy_max_m_s2;
    params.position.z_accel_limit_up_m_s2 = in->accel_z_up_max_m_s2;
    params.position.z_accel_limit_down_m_s2 = in->accel_z_down_max_m_s2;
    params.attitude.att_kp[0] = in->att_roll_kp;
    params.attitude.att_kp[1] = in->att_pitch_kp;
    params.attitude.att_kp[2] = in->att_yaw_kp;
    params.attitude.rate_limit_rad_s[0] = in->roll_rate_limit_rad_s;
    params.attitude.rate_limit_rad_s[1] = in->pitch_rate_limit_rad_s;
    params.attitude.rate_limit_rad_s[2] = in->yaw_rate_limit_rad_s;
    params.rate.kp[0] = in->rate_roll_kp;
    params.rate.kp[1] = in->rate_pitch_kp;
    params.rate.kp[2] = in->rate_yaw_kp;
    params.rate.ki[0] = in->rate_roll_ki;
    params.rate.ki[1] = in->rate_pitch_ki;
    params.rate.ki[2] = in->rate_yaw_ki;
    params.rate.kd[0] = in->rate_roll_kd;
    params.rate.kd[1] = in->rate_pitch_kd;
    params.rate.kd[2] = in->rate_yaw_kd;
    params.rate.integrator_limit[0] = in->rate_roll_i_limit_n_m;
    params.rate.integrator_limit[1] = in->rate_pitch_i_limit_n_m;
    params.rate.integrator_limit[2] = in->rate_yaw_i_limit_n_m;
    params.rate.alpha_lpf_cutoff_rad_s =
        in->angular_accel_lpf_cutoff_hz * 2.0f * APP_CONTROL_CONFIG_PI;
    params.rate.ff_gain[0] = in->rate_roll_ff;
    params.rate.ff_gain[1] = in->rate_pitch_ff;
    params.rate.ff_gain[2] = in->rate_yaw_ff;
    params.tilt_limit_rad = in->tilt_limit_rad;
    if ((fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_18_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD) ||
        (fabsf(params.tilt_limit_rad - APP_CONTROL_TILT_LIMIT_LEGACY_25_RAD) <=
         APP_CONTROL_TILT_LIMIT_LEGACY_EPS_RAD)) {
        params.tilt_limit_rad = APP_CONTROL_TILT_LIMIT_DEFAULT_RAD;
    }
    params.vel_loop_enable = in->vel_loop_enable;
    DRV_COAX_CTRL_SetParams(&params);
}

/*
 * 指令整形/出口陷波块（v24 起，v25 追加第二级陷波）。NULL = 记录里没有这一块：落回默认（关）。
 * 存的那份过不了参数校验时 SetParams 整份拒收，保持 config_apply_tunables 刚装好的
 * 默认（关）——一块坏数据不连累增益与机体模型，也绝不按坏值去整形。
 */
static void config_capture_shaping(APP_ControlCoaxShapingParams *out)
{
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetParams(&params);
    out->rate_out_notch_hz = params.rate_out_notch_hz;
    out->rate_out_notch_q = params.rate_out_notch_q;
    out->att_ref_wr_rad_s = params.att_ref_wr_rad_s;
    out->att_ref_delay_ms = params.att_ref_delay_ms;
    out->rate_out_notch2_hz = params.rate_out_notch2_hz;
    out->rate_out_notch2_q = params.rate_out_notch2_q;
}

static void config_apply_shaping(const APP_ControlCoaxShapingParams *in)
{
    DRV_COAX_CTRL_Params defaults;
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetDefaultParams(&defaults);
    DRV_COAX_CTRL_GetParams(&params);
    params.rate_out_notch_hz = (in != NULL) ? in->rate_out_notch_hz : defaults.rate_out_notch_hz;
    params.rate_out_notch_q = (in != NULL) ? in->rate_out_notch_q : defaults.rate_out_notch_q;
    params.att_ref_wr_rad_s = (in != NULL) ? in->att_ref_wr_rad_s : defaults.att_ref_wr_rad_s;
    params.att_ref_delay_ms = (in != NULL) ? in->att_ref_delay_ms : defaults.att_ref_delay_ms;
    params.rate_out_notch2_hz = (in != NULL) ? in->rate_out_notch2_hz : defaults.rate_out_notch2_hz;
    params.rate_out_notch2_q = (in != NULL) ? in->rate_out_notch2_q : defaults.rate_out_notch2_q;
    DRV_COAX_CTRL_SetParams(&params);
}

/*
 * 竖直通道块（v28 起）。NULL = 记录里没有这一块：落回驱动默认（悬停推力 0 = 关、融合开）。
 * 存的那份过不了参数校验时 SetParams 整份拒收，保持之前刚装好的值，同整形块。
 */
static void config_capture_zchan(APP_ControlZChannelParams *out)
{
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetParams(&params);
    out->hover_thrust_n = params.hover_thrust_n;
    out->z_vel_fusion = params.z_vel_fusion;
}

static void config_apply_zchan(const APP_ControlZChannelParams *in)
{
    DRV_COAX_CTRL_Params defaults;
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetDefaultParams(&defaults);
    DRV_COAX_CTRL_GetParams(&params);
    params.hover_thrust_n = (in != NULL) ? in->hover_thrust_n : defaults.hover_thrust_n;
    params.z_vel_fusion = (in != NULL) ? in->z_vel_fusion : defaults.z_vel_fusion;
    DRV_COAX_CTRL_SetParams(&params);
}

/*
 * 飞行限幅块（v29 起）。NULL = 记录里没有这一块：落回驱动默认（原写死值），同竖直通道块。
 */
static void config_capture_flightlim(APP_ControlFlightLimitParams *out)
{
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetParams(&params);
    out->alt_max_m = params.alt_max_m;
    out->manual_tilt_max_rad = params.manual_tilt_max_rad;
    out->yaw_stick_rate_rad_s = params.yaw_stick_rate_rad_s;
}

static void config_apply_flightlim(const APP_ControlFlightLimitParams *in)
{
    DRV_COAX_CTRL_Params defaults;
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetDefaultParams(&defaults);
    DRV_COAX_CTRL_GetParams(&params);
    params.alt_max_m = (in != NULL) ? in->alt_max_m : defaults.alt_max_m;
    params.manual_tilt_max_rad =
        (in != NULL) ? in->manual_tilt_max_rad : defaults.manual_tilt_max_rad;
    params.yaw_stick_rate_rad_s =
        (in != NULL) ? in->yaw_stick_rate_rad_s : defaults.yaw_stick_rate_rad_s;
    DRV_COAX_CTRL_SetParams(&params);
}

/* v24 的整形块：前四项照读，第二级陷波落回默认（compat 里定死：关、Q 1.0）。 */
static void config_apply_shaping_v24(const APP_ControlCoaxShapingParamsV24 *in)
{
    APP_ControlCoaxShapingParams current;

    (void)APP_ControlConfigCompat_ShapingV24ToCurrent(in, &current);
    config_apply_shaping(&current);
}

/*
 * v20～v25 的机体块：前 36 项照读，光流安装两项落回 0/0（恒等变换，就是旧固件的实际
 * 行为）。与增益块迁移一样是确定的函数，不读运行时的值。
 *
 * 冻结块就是当前结构体的前缀（下面的静态断言），按字节拷过来而不逐字段抄：逐字段抄
 * 会多出第二份字段清单，而漏抄一项的后果是那一项读回来是 0——对机体模型来说那是
 * "禁止解锁"或更糟的"力臂符号反了"。转成当前结构体后走同一个 DRV_Airframe_SetParams，
 * 派生重算与体检一样不少。
 */
_Static_assert(sizeof(APP_ControlAirframeParamsV25) ==
                   offsetof(DRV_Airframe_Params, flow_mount_yaw_deg),
               "v25 airframe block must be exactly the prefix before the flow mount fields");
_Static_assert(sizeof(DRV_Airframe_Params) ==
                   sizeof(APP_ControlAirframeParamsV25) + (2U * sizeof(float)),
               "v26 airframe block = v25 block + the two flow mount fields, nothing else");
_Static_assert(offsetof(DRV_Airframe_Params, flow_mount_mirror) ==
                   offsetof(DRV_Airframe_Params, flow_mount_yaw_deg) + sizeof(float),
               "the two flow mount fields must be contiguous at the tail");

static void config_apply_airframe_v25(const APP_ControlAirframeParamsV25 *in)
{
    DRV_Airframe_Params current;

    memset(&current, 0, sizeof(current));
    memcpy(&current, in, sizeof(*in));
    current.flow_mount_yaw_deg = 0.0f;
    current.flow_mount_mirror = 0.0f;
    DRV_Airframe_SetParams(&current);
}

/*
 * 当前记录的 size 字段必须把**机体模型块也算进去**。
 *
 * v20 加机体块时这里漏了：Save 写的 size 含 airframe，读回来的校验式不含，
 * 于是每一条自己写的记录都过不了自己的检查，机体模型永远读不回来。
 * 之所以一直没暴露，是因为 SAVE 在更前面就因为地址没有物理落点失败了
 * （见 app_flash_service.h 的 2026-09-11 修复）。两个缺陷叠在一起互相遮掩。
 */
#define APP_CONTROL_CFG_CURRENT_SIZE                                       \
    ((uint16_t)(sizeof(((APP_ControlFlashRecord *)0)->config) +            \
                sizeof(((APP_ControlFlashRecord *)0)->coax_tunables) +     \
                sizeof(((APP_ControlFlashRecord *)0)->rc_config) +         \
                sizeof(((APP_ControlFlashRecord *)0)->airframe) +        \
                sizeof(((APP_ControlFlashRecord *)0)->led) +             \
                sizeof(((APP_ControlFlashRecord *)0)->prop) +            \
                sizeof(((APP_ControlFlashRecord *)0)->mag) +             \
                sizeof(((APP_ControlFlashRecord *)0)->shaping) +          \
                sizeof(((APP_ControlFlashRecord *)0)->magxy) +            \
                sizeof(((APP_ControlFlashRecord *)0)->zchan) +                            sizeof(((APP_ControlFlashRecord *)0)->flightlim)))

_Static_assert(APP_CONTROL_CFG_CURRENT_SIZE ==
                   (uint16_t)(offsetof(APP_ControlFlashRecord, checksum) -
                              offsetof(APP_ControlFlashRecord, config)),
               "record size must cover exactly the checksummed span");

static uint8_t config_read_slot(uint32_t slot, APP_ControlConfig *config)
{
    APP_ControlFlashRecord record;
    if (APP_FlashService_ReadData(slot, (uint8_t *)&record,
                                  sizeof(record)) != APP_FLASH_SERVICE_OK) {
        return 0U;
    }
    if ((record.magic != APP_CONTROL_CFG_MAGIC) ||
        (record.version != APP_CONTROL_CFG_VERSION) ||
        (record.size != APP_CONTROL_CFG_CURRENT_SIZE) ||
        (config_checksum((const uint8_t *)&record.config, record.size) !=
         record.checksum)) {
        return 0U;
    }
    *config = record.config;
    config_apply_tunables(&record.coax_tunables);
    app_cmd_rcmap_apply_config(&record.rc_config);
    DRV_Airframe_SetParams(&record.airframe);
    app_cmd_ledmap_apply_config(&record.led);
    app_cmd_propcal_apply_config(&record.prop);
    app_cmd_magcal_apply_config(&record.mag);
    config_apply_shaping(&record.shaping);
    APP_MagXY_ApplyPersisted(&record.magxy);
    config_apply_zchan(&record.zchan);
    config_apply_flightlim(&record.flightlim);
    return 1U;
}

static uint8_t config_read_current(APP_ControlConfig *config)
{
    return config_read_slot(config_active_slot(), config);
}

/*
 * `apply_blocks_statement` 必须把**每一个块**都显式处理掉，包括这个版本里没有的。
 *
 * 以前宏体末尾硬写着一句 `app_cmd_ledmap_apply_config(NULL)`，看着省事，但它
 * 有个到 v21 才会暴露的毛病：一旦某个旧版本**已经带了** LED 块，那句无条件的
 * 兜底会把刚读出来的值当场抹掉。加块的人很难想到去看宏体尾巴。
 *
 * 所以现在宏体里一句应用语句都不留：谁读哪个版本，就在那一行把它有的块装上、
 * 没有的块显式落回默认。加新块时编译器不会提醒你，但这张表会——每个读取器都
 * 明摆着列出了它认得的块，漏掉的那一行一眼就看得见。
 *
 * "没有的块必须显式落回默认"这条规则本身不能省：什么都不做会让 RAM 里留着
 * 上一次的值，于是 `LEDMAP?` / `PROPCAL?` 报的和 Flash 里存的不是一回事，
 * 而用户据此认为"存进去了"。机体模型当年漏的就是这半步。
 */
#define APP_CONTROL_DEFINE_LEGACY_READER(function_name, type, expected_version, \
                                         convert_fn, apply_blocks_statement) \
    static uint8_t function_name(APP_ControlConfig *config) \
    { \
        type record; \
        APP_ControlCoaxTunableParams migrated; \
        if (APP_FlashService_ReadData(config_active_slot(), \
                                      (uint8_t *)&record, sizeof(record)) != \
            APP_FLASH_SERVICE_OK) return 0U; \
        if ((record.magic != APP_CONTROL_CFG_MAGIC) || \
            (record.version != (expected_version)) || \
            (record.size != (uint16_t)(offsetof(type, checksum) - \
                                       offsetof(type, config))) || \
            (config_checksum((const uint8_t *)&record.config, record.size) != \
             record.checksum) || \
            ((convert_fn)(&record.coax_tunables, &migrated) == 0U)) return 0U; \
        *config = record.config; \
        config_apply_tunables(&migrated); \
        apply_blocks_statement; \
        return 1U; \
    }

APP_CONTROL_DEFINE_LEGACY_READER(config_read_v28, APP_ControlFlashRecordV28,
                                 APP_CONTROL_CFG_VERSION_V28,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     DRV_Airframe_SetParams(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(&record.prop);
                                     app_cmd_magcal_apply_config(&record.mag);
                                     config_apply_shaping(&record.shaping);
                                     APP_MagXY_ApplyPersisted(&record.magxy);
                                     config_apply_zchan(&record.zchan);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v27, APP_ControlFlashRecordV27,
                                 APP_CONTROL_CFG_VERSION_V27,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     DRV_Airframe_SetParams(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(&record.prop);
                                     app_cmd_magcal_apply_config(&record.mag);
                                     config_apply_shaping(&record.shaping);
                                     APP_MagXY_ApplyPersisted(&record.magxy);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v26, APP_ControlFlashRecordV26,
                                 APP_CONTROL_CFG_VERSION_V26,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     DRV_Airframe_SetParams(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(&record.prop);
                                     app_cmd_magcal_apply_config(&record.mag);
                                     config_apply_shaping(&record.shaping);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v25, APP_ControlFlashRecordV25,
                                 APP_CONTROL_CFG_VERSION_V25,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     config_apply_airframe_v25(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(&record.prop);
                                     app_cmd_magcal_apply_config(&record.mag);
                                     config_apply_shaping(&record.shaping);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v24, APP_ControlFlashRecordV24,
                                 APP_CONTROL_CFG_VERSION_V24,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     config_apply_airframe_v25(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(&record.prop);
                                     app_cmd_magcal_apply_config(&record.mag);
                                     config_apply_shaping_v24(&record.shaping);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v23, APP_ControlFlashRecordV23,
                                 APP_CONTROL_CFG_VERSION_V23,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     config_apply_airframe_v25(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(&record.prop);
                                     app_cmd_magcal_apply_config(&record.mag);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v22, APP_ControlFlashRecordV22,
                                 APP_CONTROL_CFG_VERSION_V22,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     config_apply_airframe_v25(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(&record.prop);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v21, APP_ControlFlashRecordV21,
                                 APP_CONTROL_CFG_VERSION_V21,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     config_apply_airframe_v25(&record.airframe);
                                     app_cmd_ledmap_apply_config(&record.led);
                                     app_cmd_propcal_apply_config(NULL);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v20, APP_ControlFlashRecordV20,
                                 APP_CONTROL_CFG_VERSION_V20,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     config_apply_airframe_v25(&record.airframe);
                                     app_cmd_ledmap_apply_config(NULL);
                                     app_cmd_propcal_apply_config(NULL);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v19, APP_ControlFlashRecordV19,
                                 APP_CONTROL_CFG_VERSION_V19,
                                 APP_ControlConfigCompat_CurrentPassthrough,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     app_cmd_ledmap_apply_config(NULL);
                                     app_cmd_propcal_apply_config(NULL);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v18, APP_ControlFlashRecordV18,
                                 APP_CONTROL_CFG_VERSION_V18,
                                 APP_ControlConfigCompat_V18ToCurrent,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     app_cmd_ledmap_apply_config(NULL);
                                     app_cmd_propcal_apply_config(NULL);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v17, APP_ControlFlashRecordV17,
                                 APP_CONTROL_CFG_VERSION_V17,
                                 APP_ControlConfigCompat_V17ToCurrent,
                                 do {
                                     app_cmd_rcmap_apply_config(&record.rc_config);
                                     app_cmd_ledmap_apply_config(NULL);
                                     app_cmd_propcal_apply_config(NULL);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v16, APP_ControlFlashRecordV16,
                                 APP_CONTROL_CFG_VERSION_V16,
                                 APP_ControlConfigCompat_V17ToCurrent,
                                 do {
                                     app_cmd_rcmap_apply_config(NULL);
                                     app_cmd_ledmap_apply_config(NULL);
                                     app_cmd_propcal_apply_config(NULL);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))
APP_CONTROL_DEFINE_LEGACY_READER(config_read_v15, APP_ControlFlashRecordV15,
                                 APP_CONTROL_CFG_VERSION_V15,
                                 APP_ControlConfigCompat_V15ToCurrent,
                                 do {
                                     app_cmd_rcmap_apply_config(NULL);
                                     app_cmd_ledmap_apply_config(NULL);
                                     app_cmd_propcal_apply_config(NULL);
                                     app_cmd_magcal_apply_config(NULL);
                                     config_apply_shaping(NULL);
                                 } while (0))

APP_FlashService_Status APP_ControlConfigStore_Load(APP_ControlConfig *config)
{
    APP_ControlFlashHeader header;
    APP_FlashService_Status status;

    if (config == NULL) {
        return APP_FLASH_SERVICE_ERROR;
    }
    status = APP_FlashService_ReadData(config_active_slot(),
                                       (uint8_t *)&header, sizeof(header));
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }
    if (header.magic != APP_CONTROL_CFG_MAGIC) {
        return APP_FLASH_SERVICE_BAD_ID;
    }
    switch (header.version) {
    case APP_CONTROL_CFG_VERSION:
        if (config_read_current(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V28:
        if (config_read_v28(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V27:
        if (config_read_v27(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V26:
        if (config_read_v26(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V25:
        if (config_read_v25(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V24:
        if (config_read_v24(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V23:
        if (config_read_v23(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V22:
        if (config_read_v22(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V21:
        if (config_read_v21(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V20:
        if (config_read_v20(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V19:
        if (config_read_v19(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V18:
        if (config_read_v18(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V17:
        if (config_read_v17(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V16:
        if (config_read_v16(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    case APP_CONTROL_CFG_VERSION_V15:
        if (config_read_v15(config) == 0U) return APP_FLASH_SERVICE_ERROR;
        break;
    default:
        return APP_FLASH_SERVICE_BAD_ID;
    }
    /* Records older than v27 have no independent XY axis proof. Explicitly clear any
     * previous RAM candidate so reload cannot retain a stale active aid. v27 carries
     * the XY block (config_read_v27 applied it) — clearing on "!= current" would wipe
     * it on the first boot after the v28 upgrade. */
    if (header.version < APP_CONTROL_CFG_VERSION_V27) {
        APP_MagXY_ApplyPersisted(NULL);
    }
    /* v27 及更早没有竖直通道块（v28 起有）：显式落回驱动默认，不留 RAM 里上一次的值。 */
    if (header.version < APP_CONTROL_CFG_VERSION_V28) {
        config_apply_zchan(NULL);
    }
    /* v28 及更早没有飞行限幅块（v29 起有）：同上。 */
    if (header.version != APP_CONTROL_CFG_VERSION) {
        config_apply_flightlim(NULL);
    }
    /* RAM 里的增益已整份换成 Flash 里的值：试用全部结束（app_param_trial.h 契约 5）。 */
    APP_ParamTrial_ClearAll();
    config->loaded_from_flash = 1U;
    config->flash_valid = 1U;
    return APP_FLASH_SERVICE_OK;
}

/*
 * 写入另一个槽，回读校验，最后单独提交。
 *
 * 顺序是有讲究的：**当前那个槽在整个过程中一直没被碰过**。任何一步失败或掉电，
 * 上电后 config_active_slot() 仍然选中它，飞控照旧读到上一份好配置。
 * 以前是单槽原地擦写，擦完到写完之间那一两秒里断电，配置就没了——现在这条
 * 记录里还装着机体模型，没模型就禁止解锁，代价已经不是"增益要重设"那么轻。
 */
APP_FlashService_Status APP_ControlConfigStore_Save(const APP_ControlConfig *config)
{
    APP_ControlFlashRecord record;
    APP_ControlFlashRecord verify;
    APP_ControlConfigCommit commit;
    APP_FlashService_Status status;
    uint32_t current_slot;
    uint32_t target_slot;
    uint32_t current_rank;

    if (config == NULL) {
        return APP_FLASH_SERVICE_ERROR;
    }

    current_slot = config_active_slot();
    current_rank = config_slot_sequence(current_slot);
    target_slot = (current_slot == APP_CONTROL_CFG_SLOT_A) ?
                  APP_CONTROL_CFG_SLOT_B : APP_CONTROL_CFG_SLOT_A;

    memset(&record, 0xFF, sizeof(record));
    record.magic = APP_CONTROL_CFG_MAGIC;
    record.version = APP_CONTROL_CFG_VERSION;
    record.size = APP_CONTROL_CFG_CURRENT_SIZE;
    record.config = *config;
    record.config.loaded_from_flash = 1U;
    record.config.flash_valid = 1U;
    APP_ControlConfigStore_CaptureTunables(&record.coax_tunables);
    record.rc_config = *(const APP_RcConfig *)app_cmd_rcmap_config();
    DRV_Airframe_GetParams(&record.airframe);
    record.led = *(const APP_LedConfig *)app_cmd_ledmap_config();
    record.prop = *(const DRV_PropMap *)app_cmd_propcal_config();
    record.mag = *(const DRV_MAG_Calibration *)app_cmd_magcal_config();
    config_capture_shaping(&record.shaping);
    APP_MagXY_GetPersisted(&record.magxy);
    config_capture_zchan(&record.zchan);
    config_capture_flightlim(&record.flightlim);
    /* 两块捕获的都是 RAM；SYSID PARAM 试用中的名字换回试用前的值再存（app_param_trial.h）。 */
    APP_ParamTrial_RestorePersistent(&record.coax_tunables, &record.shaping);
    record.checksum = config_checksum((const uint8_t *)&record.config,
                                      record.size);

    status = APP_FlashService_EraseSector(target_slot);
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }

    status = APP_FlashService_WriteData(target_slot, (const uint8_t *)&record,
                                        sizeof(record));
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }

    /*
     * 回读校验主体之后才提交。不校验就提交的话，一次写坏会被提交字盖章成
     * "有效"，而且因为 sequence 更大，它还会盖过那份好的——双槽反而帮了倒忙。
     */
    status = APP_FlashService_ReadData(target_slot, (uint8_t *)&verify,
                                       sizeof(verify));
    if (status != APP_FLASH_SERVICE_OK) {
        return status;
    }
    if (memcmp(&record, &verify, sizeof(record)) != 0) {
        return APP_FLASH_SERVICE_ERROR;
    }

    memset(&commit, 0xFF, sizeof(commit));
    commit.magic = APP_CONTROL_CFG_COMMIT_MAGIC;
    /* current_rank 为 0/1 表示对面没有带提交字的记录，从 0 号序列开始。 */
    commit.sequence = (current_rank >= 2U) ? (current_rank - 1U) : 0U;
    commit.body_checksum = record.checksum;
    return APP_FlashService_WriteData(target_slot + APP_CONTROL_CFG_COMMIT_OFFSET,
                                      (const uint8_t *)&commit, sizeof(commit));
}
