/*
 * app_servo_backlash.c —— 舵机回差补偿：策略层。设计意图写在 App/Inc/app_servo_backlash.h。
 *
 * 这里补三处"为什么这么写"：
 *
 * 1. **在角度上补，不在脉宽上补。** 回差 b 是按辨识记录的倾转角拟出来的，那个角度
 *    与脉宽之间就是分配器那条律（2000 µs 对 180°，标定中位为 0，乘 pulse_sign）。
 *    这里用同一条律来回换，b 的单位与辨识口径一致；机体参数里的 servo_us_per_deg 只是
 *    展示用的派生值，脉宽通路从来不读它。回差对称，pulse_sign 在来回两次换算里抵消，
 *    乘上它只是让 d = +1 的含义是"机构正向倾转"。
 *
 * 2. **不补的那一拍直接复位。** 回中、标定、点动、验收这些时候舵盘在哪、负载贴在
 *    空程哪一边都不再是补偿器以为的样子；下次重新接手时从 d = 0 起步（第一拍原样、
 *    走出阈值才定向），比带着一个过时的方向多走 b 安全。
 *
 * 3. **配置只由控制拍取用。** 命令任务把待生效配置和序号在临界区里写好；Apply 看到
 *    序号变了才换配置并复位两路状态。状态发布是一份几十字节的拷贝，在临界区里做，
 *    读者也在临界区里拷，拷的过程中控制拍插不进来。
 */

#include "app_servo_backlash.h"

#include "bsp_critical.h"
#include "drv_coax_ctrl.h"
#include "drv_servo_backlash.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* 与 drv_coax_ctrl.c 的 coax_ctrl_tilt_rad_to_servo_pulse 同一条律。 */
#define SERVO_BACKLASH_US_PER_RAD \
    ((float)(DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US - DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US) / \
     (DRV_COAX_CTRL_SERVO_TRAVEL_DEG * 0.017453292519943295f))
#define SERVO_BACKLASH_RAD_PER_US (1.0f / SERVO_BACKLASH_US_PER_RAD)

/* 只有 StabilizerTask 读写；其它任务只经发布副本与待生效配置交互。 */
typedef struct {
    DRV_ServoBacklash ch[2];
    APP_ServoBacklashConfig cfg;     /* 生效中的配置 */
    uint32_t applied_seq;
    uint8_t active;
    uint8_t source;
    uint8_t has_last;
    uint32_t last_ms;
    uint16_t cal_center_us[2];       /* 上一拍用的标定中位与极性：变了就复位 */
    int8_t cal_sign[2];
    uint8_t has_cal;
    int16_t offset_us[2];
    uint32_t resets, ticks, clamped;
} ServoBacklashContext;

static ServoBacklashContext servo_backlash;
static APP_ServoBacklashStatus servo_backlash_status;          /* 发布副本 */
static uint32_t servo_backlash_status_seq;                     /* 发布副本对应的配置序号 */
static APP_ServoBacklashConfig servo_backlash_pending = {
    APP_SERVO_BACKLASH_DEFAULT_ENABLE,
    { APP_SERVO_BACKLASH_DEFAULT_HALF_MRAD, APP_SERVO_BACKLASH_DEFAULT_HALF_MRAD },
    APP_SERVO_BACKLASH_DEFAULT_THR_MRAD
};
static volatile uint32_t servo_backlash_pending_seq;           /* 命令任务每提交一次加 1 */
/* 补偿器重建（改配置）前的累计计数：状态里报开机以来的总数。 */
static uint32_t servo_backlash_reversal_base[2];
static uint32_t servo_backlash_nonfinite_base;

/* ------------------------------------------------------------------ 配置 */

void APP_ServoBacklash_DefaultConfig(APP_ServoBacklashConfig *cfg)
{
    if (cfg == NULL) {
        return;
    }
    cfg->enable = APP_SERVO_BACKLASH_DEFAULT_ENABLE;
    cfg->half_mrad[APP_SERVO_BACKLASH_ALPHA] = APP_SERVO_BACKLASH_DEFAULT_HALF_MRAD;
    cfg->half_mrad[APP_SERVO_BACKLASH_BETA] = APP_SERVO_BACKLASH_DEFAULT_HALF_MRAD;
    cfg->thr_mrad = APP_SERVO_BACKLASH_DEFAULT_THR_MRAD;
}

uint8_t APP_ServoBacklash_ConfigValid(const APP_ServoBacklashConfig *cfg)
{
    if (cfg == NULL) {
        return 0U;
    }
    return ((cfg->enable <= 1U) &&
            (cfg->half_mrad[APP_SERVO_BACKLASH_ALPHA] <= APP_SERVO_BACKLASH_HALF_MRAD_MAX) &&
            (cfg->half_mrad[APP_SERVO_BACKLASH_BETA] <= APP_SERVO_BACKLASH_HALF_MRAD_MAX) &&
            (cfg->thr_mrad >= APP_SERVO_BACKLASH_THR_MRAD_MIN) &&
            (cfg->thr_mrad <= APP_SERVO_BACKLASH_THR_MRAD_MAX)) ? 1U : 0U;
}

uint8_t APP_ServoBacklash_RequestConfig(const APP_ServoBacklashConfig *cfg)
{
    uint32_t lock;

    if (APP_ServoBacklash_ConfigValid(cfg) == 0U) {
        return 0U;
    }
    lock = BSP_Critical_Enter();
    servo_backlash_pending = *cfg;
    servo_backlash_pending_seq = servo_backlash_pending_seq + 1U;
    BSP_Critical_Exit(lock);
    return 1U;
}

void APP_ServoBacklash_GetConfig(APP_ServoBacklashConfig *out)
{
    uint32_t lock;

    if (out == NULL) {
        return;
    }
    /* 待生效的那份就是最新请求；生效后两者相同。 */
    lock = BSP_Critical_Enter();
    *out = servo_backlash_pending;
    BSP_Critical_Exit(lock);
}

uint8_t APP_ServoBacklash_SourceCompensated(APP_ServoBacklashSource source, uint8_t armed,
                                            uint8_t override)
{
    if (override != 0U) {
        return 0U;
    }
    if (source == APP_SERVO_BACKLASH_SRC_SYSID) {
        return 1U;                     /* SERVO 模式本来就是上锁跑的，FF/RATE/ANGLE 由辨识自己管解锁 */
    }
    if (source == APP_SERVO_BACKLASH_SRC_CONTROLLER) {
        return (armed != 0U) ? 1U : 0U; /* 上锁时控制器照样算、舵机照样动，但那是地面检查，不补 */
    }
    return 0U;
}

const char *APP_ServoBacklash_SourceName(uint8_t source)
{
    switch (source) {
    case APP_SERVO_BACKLASH_SRC_NONE:       return "none";
    case APP_SERVO_BACKLASH_SRC_CONTROLLER: return "controller";
    case APP_SERVO_BACKLASH_SRC_SYSID:      return "sysid";
    default:                                return "unknown";
    }
}

/* ------------------------------------------------------------------ 内部 */

/* 按生效配置重建两路补偿器：状态复位；驱动里的计数先并进累计值再清零。 */
static void servo_backlash_build(void)
{
    for (uint32_t i = 0U; i < 2U; ++i) {
        DRV_ServoBacklashConfig drv;

        servo_backlash_reversal_base[i] += servo_backlash.ch[i].reversal_count;
        servo_backlash_nonfinite_base += servo_backlash.ch[i].nonfinite_count;
        drv.half_gap_rad = (float)servo_backlash.cfg.half_mrad[i] * 0.001f;
        drv.threshold_rad = (float)servo_backlash.cfg.thr_mrad * 0.001f;
        drv.enable = 1U;              /* 开关由本层按来源与配置决定，驱动只管算 */
        (void)DRV_ServoBacklash_Init(&servo_backlash.ch[i], &drv);
    }
}

static void servo_backlash_reset(void)
{
    DRV_ServoBacklash_Reset(&servo_backlash.ch[0]);
    DRV_ServoBacklash_Reset(&servo_backlash.ch[1]);
}

static void servo_backlash_publish(void)
{
    APP_ServoBacklashStatus s;
    uint32_t lock;

    s.cfg = servo_backlash.cfg;
    s.active = servo_backlash.active;
    s.source = servo_backlash.source;
    s.resets = servo_backlash.resets;
    s.ticks = servo_backlash.ticks;
    s.clamped = servo_backlash.clamped;
    s.nonfinite = servo_backlash_nonfinite_base;
    for (uint32_t i = 0U; i < 2U; ++i) {
        s.direction[i] = (servo_backlash.active != 0U) ? servo_backlash.ch[i].direction : 0;
        s.offset_us[i] = servo_backlash.offset_us[i];
        s.reversals[i] = servo_backlash_reversal_base[i] + servo_backlash.ch[i].reversal_count;
        s.nonfinite += servo_backlash.ch[i].nonfinite_count;
    }
    lock = BSP_Critical_Enter();
    servo_backlash_status = s;
    servo_backlash_status_seq = servo_backlash.applied_seq;
    BSP_Critical_Exit(lock);
}

/* 标定中位/极性和上一拍不同（或第一次见）返回 1，并记下这一份。 */
static uint8_t servo_backlash_note_calibration(const DRV_COAX_CTRL_ServoCalibration *cal)
{
    uint8_t changed = (servo_backlash.has_cal == 0U) ? 1U : 0U;

    for (uint32_t i = 0U; i < 2U; ++i) {
        if ((servo_backlash.cal_center_us[i] != cal->center_us[i]) ||
            (servo_backlash.cal_sign[i] != cal->pulse_sign[i])) {
            changed = 1U;
        }
        servo_backlash.cal_center_us[i] = cal->center_us[i];
        servo_backlash.cal_sign[i] = cal->pulse_sign[i];
    }
    servo_backlash.has_cal = 1U;
    return changed;
}

/* 一路：脉宽 → 倾转角 → 补偿 → 脉宽，夹回标定端点。返回补偿后的脉宽。 */
static uint16_t servo_backlash_channel(uint32_t i, uint16_t pulse_us,
                                       const DRV_COAX_CTRL_ServoCalibration *cal)
{
    const float sign = (cal->pulse_sign[i] < 0) ? -1.0f : 1.0f;
    const float cmd_rad = (float)((int32_t)pulse_us - (int32_t)cal->center_us[i]) * sign *
                          SERVO_BACKLASH_RAD_PER_US;
    const float out_rad = DRV_ServoBacklash_Step(&servo_backlash.ch[i], cmd_rad);
    const float pulse_f = (float)cal->center_us[i] + (out_rad * sign * SERVO_BACKLASH_US_PER_RAD);
    int32_t out;

    if (isfinite(pulse_f) == 0) {
        return pulse_us;               /* 走不到：输入是整数脉宽，驱动对有限输入给有限输出 */
    }
    out = (int32_t)floorf(pulse_f + 0.5f);
    if (out < (int32_t)cal->min_us[i]) {
        out = (int32_t)cal->min_us[i];
        servo_backlash.clamped++;
    } else if (out > (int32_t)cal->max_us[i]) {
        out = (int32_t)cal->max_us[i];
        servo_backlash.clamped++;
    }
    return (uint16_t)out;
}

/* ------------------------------------------------------------------ 运行面 */

void APP_ServoBacklash_Init(void)
{
    uint32_t lock;

    memset(&servo_backlash, 0, sizeof(servo_backlash));
    servo_backlash_reversal_base[0] = 0U;
    servo_backlash_reversal_base[1] = 0U;
    servo_backlash_nonfinite_base = 0U;
    lock = BSP_Critical_Enter();
    APP_ServoBacklash_DefaultConfig(&servo_backlash_pending);   /* 阶段一只在 RAM：上电回到编译期默认 */
    servo_backlash.cfg = servo_backlash_pending;
    servo_backlash.applied_seq = servo_backlash_pending_seq;
    BSP_Critical_Exit(lock);
    servo_backlash_build();
    servo_backlash_publish();
}

void APP_ServoBacklash_Apply(uint32_t now_ms, APP_ServoBacklashSource source, uint8_t armed,
                             uint8_t override, uint16_t *alpha_pulse_us, uint16_t *beta_pulse_us)
{
    uint16_t *pulse[2] = { alpha_pulse_us, beta_pulse_us };
    DRV_COAX_CTRL_ServoCalibration cal;
    uint32_t lock, seq;
    uint8_t restart = 0U;
    uint8_t want;

    if ((alpha_pulse_us == NULL) || (beta_pulse_us == NULL)) {
        return;
    }

    /* 1. 命令任务交来的配置：本函数是生效配置的唯一写者。 */
    lock = BSP_Critical_Enter();
    seq = servo_backlash_pending_seq;
    if (seq != servo_backlash.applied_seq) {
        servo_backlash.cfg = servo_backlash_pending;
    }
    BSP_Critical_Exit(lock);
    if (seq != servo_backlash.applied_seq) {
        servo_backlash.applied_seq = seq;
        servo_backlash_build();
        restart = 1U;
    }

    /* 2. 标定换了（中位/极性）或中间断过档：补偿器记的角度零点与空程位置都不作数了。 */
    DRV_COAX_CTRL_GetServoCalibration(&cal);
    if (servo_backlash_note_calibration(&cal) != 0U) {
        restart = 1U;
    }
    if ((servo_backlash.has_last != 0U) &&
        ((uint32_t)(now_ms - servo_backlash.last_ms) > APP_SERVO_BACKLASH_GAP_RESET_MS)) {
        restart = 1U;
    }
    servo_backlash.last_ms = now_ms;
    servo_backlash.has_last = 1U;

    want = ((servo_backlash.cfg.enable != 0U) &&
            (APP_ServoBacklash_SourceCompensated(source, armed, override) != 0U)) ? 1U : 0U;
    if ((servo_backlash.active != 0U) &&
        ((want == 0U) || (restart != 0U) || ((uint8_t)source != servo_backlash.source))) {
        servo_backlash.resets++;       /* 丢掉了一个已有的方向 */
    }
    if ((want == 0U) || (servo_backlash.active == 0U) || (restart != 0U) ||
        ((uint8_t)source != servo_backlash.source)) {
        servo_backlash_reset();        /* 不补的拍、或新接手的第一拍：从 d = 0 起步 */
    }
    servo_backlash.active = want;
    servo_backlash.source = (uint8_t)source;

    if (want == 0U) {
        servo_backlash.offset_us[0] = 0;
        servo_backlash.offset_us[1] = 0;
    } else {
        for (uint32_t i = 0U; i < 2U; ++i) {
            const uint16_t in = *pulse[i];

            *pulse[i] = servo_backlash_channel(i, in, &cal);
            servo_backlash.offset_us[i] = (int16_t)((int32_t)*pulse[i] - (int32_t)in);
        }
        servo_backlash.ticks++;
    }
    servo_backlash_publish();
}

/* ------------------------------------------------------------------ 读出 */

void APP_ServoBacklash_GetStatus(APP_ServoBacklashStatus *out)
{
    APP_ServoBacklashConfig pending;
    uint32_t lock, seq, published;

    if (out == NULL) {
        return;
    }
    lock = BSP_Critical_Enter();
    *out = servo_backlash_status;
    published = servo_backlash_status_seq;
    pending = servo_backlash_pending;
    seq = servo_backlash_pending_seq;
    BSP_Critical_Exit(lock);
    if (seq != published) {
        /* 命令刚提交、控制拍还没取用（最多一拍）：报请求的配置，免得 ON 的回复里还写着 en=0。 */
        out->cfg = pending;
    }
}
