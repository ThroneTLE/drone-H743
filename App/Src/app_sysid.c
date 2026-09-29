/*
 * app_sysid.c —— 内环系统辨识的运行层。
 *
 * 设计意图与不变量写在 App/Inc/app_sysid.h。这里只补三处"为什么这么写"的说明：
 *
 * 1. **执行发生在控制拍上，发送发生在别的拍上。**
 *    采样写环形缓冲，`APP_SysId_StreamTick()` 在遥测任务里排空。时间戳是采样
 *    当时的固件微秒值，所以主机收包的抖动一点都不会进入辨识结果——而这套辨识
 *    要测的恰好是毫秒级的时移。
 *
 * 2. **绕杆轴的角度用融合姿态投影，不用陀螺积分。**
 *    θ ≈ roll·cosψ + pitch·sinψ（小角度下姿态矢量在杆轴上的投影）。
 *    陀螺积分会漂，一趟几秒的激励下来限位判据就不可信了；而限位是安全门。
 *
 * 3. **超限一律 abort，不限幅。**
 *    限幅会让一段被削顶的数据看起来"跑完了"，然后被拿去拟合。宁可这一趟作废。
 */

#include "app_sysid.h"

#include "app_control.h"
#include "app_sysid_alt.h"
#include "app_diag_binary.h"
#include "app_esc_command.h"
#include "app_ident.h"
#include "app_proto.h"
#include "app_servo_jog.h"

#include "drv_airframe_params.h"
#include "drv_att_reference.h"
#include "drv_coax_ctrl.h"
#include "drv_moment_notch.h"
#include "drv_prop_map.h"
#include "bsp_critical.h"
#include "bsp_dshot_rx.h" /* 只读电调回传转速快照做记录；与推力查补表同一份数据 */
#include "bsp_pwm.h"   /* 只取电调脉宽量程常量做封顶；写电调仍只在稳定环 */

#include <math.h>
#include <string.h>

/* 一批最多带走多少条；与线上帧上限同源。 */
#define SYSID_BATCH_MAX DRV_SYSID_RECORD_MAX_COUNT

/*
 * 每个遥测拍最多排空几批。
 *
 * 默认遥测拍是 25 ms（40 Hz），500 Hz 采样只需要约 2.5 批/拍（满批 5 条）。取 16 是为了
 * **不让一个无关的设置悄悄掐住这条流**：`TELEM RATE` 可以被调到 1 Hz，
 * 那时排空节奏跟着掉到 1 次/秒，环会在 0.3 秒内满。16 批 = 80 条样本
 * （500 Hz 下 160 ms），覆盖到大约 6 Hz 的遥测拍；再低就会丢样，但丢样是
 * **报出来的**（`dropped=` 计数 + 批上的 GAP 标志），不是安静地少几段数据。
 */
#define SYSID_BATCHES_PER_TICK 16U

/* 电调回传转速的新鲜度，与 app_thrust_lut.c 的 THRUST_LUT_ERPM_FRESH_MS 同值：
 * 同一份回包，两处对"还算新"的判断不能各说各的。 */
#define SYSID_ERPM_FRESH_MS 100U

typedef struct {
    DRV_SysIdSample sample;
    uint32_t t_us;
    /*
     * 这条之前环满丢过样本。断点只按真实丢样判，不按时间间隔猜：控制拍本身在
     * 2~3 ms 之间抖（调度器"满 2 ms 的下一次唤醒"），250 Hz 采样间隔最坏约 6 ms，
     * 旧的"超过名义 1.5 倍即断点"会把一段一条没丢的数据判成有缺口。
     */
    uint8_t gap_before;
    uint8_t erpm_valid;   /* 这条的上/下桨转速都来自新鲜有效回包（批头 ERPM_VALID 据此置位） */
    uint8_t capped;       /* ALT：这条的电机脉宽被最高油门 % 封顶（批头 THRUST_CAPPED 据此置位） */
} SysIdRingEntry;

typedef struct {
    APP_SysIdState state;
    const char *last_reason;

    DRV_SysIdRig rig;
    DRV_SysIdExcitation spec;
    float inertia_kg_m2;
    float angle_limit_rad;
    float residual_limit_rad_s;
    uint32_t sample_rate_hz;
    APP_SysIdMode mode;
    uint16_t run_flags;          /* 本轮的模式标志（RATE/ANGLE/SERVO），开跑时锁存；排空可能晚于下一次 MODE */
    float angle_amplitude_rad;
    float servo_tilt_rad;        /* SERVO 模式的倾转幅值 */
    DRV_RateControl_State rate_state;
    uint32_t previous_us;
    uint8_t engaged;
    uint8_t notice_pending;
    uint8_t saturation_detail;
    float saturation_force, saturation_requested[2], saturation_achieved[2];
    uint16_t saturation_pulses[2];
    uint8_t stream_gap;
    uint32_t bad_dt_us;          /* control_dt 中止时那一拍的实际间隔，排空时报出 */

    /* 自动油门：target_n=0 即手动（遥控器给油门）。 */
    float throttle_target_n;
    float throttle_max_pct;
    APP_SysIdPhase phase;
    uint32_t phase_start_ms;
    uint16_t idle_pulse_us;      /* 起升前遥控器给的脉宽（杆在最低 = 怠速） */
    uint16_t ramp_from_us;       /* 回落起点 */
    uint16_t motor_pulse_us;
    uint8_t motor_capped;
    float excite_angle0_rad;     /* 激励开始时的自然平衡角；ANGLE 模式以它为零点 */
    uint8_t preroll_started;     /* 稳定段末尾的零激励前导已开始采样 */
    uint8_t phase_notice;        /* 阶段变化待报（控制拍不打字，排空拍打） */
    uint16_t phase_notice_pulse;
    float phase_notice_thrust;
    APP_SysIdPhase phase_notice_phase;
    /* 最近一拍的遥控器状态，供 START 在命令任务里先行拒绝。 */
    volatile uint8_t last_armed;
    volatile uint8_t last_thr_low;

    uint16_t run_id;
    /*
     * 起始时刻单独用一个标志，不拿 start_ms==0 当"还没起跑"的哨兵：
     * now_ms 上电后本来就会经过 0，那一拍会被当成没起跑而把基准挪到 1，
     * 于是 elapsed 立刻变成一个接近 2^32 的数，剖面"瞬间跑完"。
     */
    uint8_t started;
    uint32_t start_ms;
    uint32_t next_sample_us;
    uint32_t sample_period_us;
    uint8_t first_batch_pending;
    uint8_t last_batch_pending;
    uint32_t dropped;

    /*
     * 断点标记挂在**丢样之后的第一条样本**上（SysIdRingEntry.gap_before），不挂在
     * 全局标志上：丢样时环里往往还压着一段丢样之前的连续数据，全局标志会挂到错误
     * 的一批。挂在样本上则断在哪批就报哪批。
     */
    uint8_t gap_pending;         /* 环满丢样后，下一条入环样本带 gap_before */

    uint16_t alpha_us;
    uint16_t beta_us;

    SysIdRingEntry ring[APP_SYSID_RING_SAMPLES];
    volatile uint32_t head;   /* 写指针（控制拍） */
    volatile uint32_t tail;   /* 读指针（遥测拍） */
} SysIdContext;

static SysIdContext sysid;
static uint8_t sysid_initialised;

/*
 * RATE/ANGLE 闭环验证轮用的指令整形与出口陷波状态：与在飞控制器同一实现
 * （drv_att_reference.h / drv_moment_notch.h，参数取 coax.att_ref_* / coax.rate_out_notch_* /
 * coax.rate_out_notch2_*），只是轴换成杆轴。参考历史约 1.3 KB，放 AXI SRAM（NOLOAD，
 * APP_SysId_Init 显式清），不占 DTCM。
 */
typedef struct {
    DRV_AttRef ref;
    DRV_MomentNotch notch;
    DRV_MomentNotch notch2;   /* 第二级出口陷波，串在 notch 之后 */
} SysIdShaping;

#if defined(__GNUC__) && defined(__arm__)
__attribute__((section(".ram_d1_noinit"), aligned(32)))
#endif
static SysIdShaping sysid_shaping;

/* 与速率环积分同时清：开跑、激励起点。 */
static void sysid_shaping_reset(void)
{
    DRV_AttRef_Reset(&sysid_shaping.ref);
    DRV_MomentNotch_Reset(&sysid_shaping.notch);
    DRV_MomentNotch_Reset(&sysid_shaping.notch2);
}

static uint8_t pending_payload[DRV_SYSID_RECORD_MAX_FRAME];
static size_t pending_length;

/* ------------------------------------------------------------------ 小工具 */

static float sysid_finite_or(float value, float fallback)
{
    return (isfinite(value) != 0) ? value : fallback;
}

static void sysid_center_servos(void)
{
    DRV_COAX_CTRL_ServoCalibration calibration;

    DRV_COAX_CTRL_GetServoCalibration(&calibration);
    sysid.alpha_us = calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
    sysid.beta_us = calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX];
}

static void sysid_ring_reset(void)
{
    sysid.head = 0U;
    sysid.tail = 0U;
    sysid.dropped = 0U;
    sysid.gap_pending = 0U;
}

static uint32_t sysid_ring_count(void)
{
    return sysid.head - sysid.tail;
}

static void sysid_ring_push(const DRV_SysIdSample *sample, uint32_t t_us,
                            uint8_t erpm_valid)
{
    uint32_t index;

    if (sysid_ring_count() >= APP_SYSID_RING_SAMPLES) {
        /*
         * 满了就丢**新的**，不覆盖旧的。覆盖会把一段已经排好队、上位机正等着
         * 的连续数据从中间挖掉，而丢新样本只会让这一段在断点处干净地截断——
         * 后者主机能看出来（GAP 标志），前者看不出来。
         */
        sysid.dropped++;
        sysid.gap_pending = 1U;
        return;
    }
    index = sysid.head % APP_SYSID_RING_SAMPLES;
    sysid.ring[index].sample = *sample;
    sysid.ring[index].t_us = t_us;
    sysid.ring[index].gap_before = sysid.gap_pending;
    sysid.ring[index].erpm_valid = erpm_valid;
    sysid.ring[index].capped = (sysid.mode == APP_SYSID_ALT) ? sysid.motor_capped : 0U;
    sysid.gap_pending = 0U;
    BSP_Critical_MemoryBarrier();
    sysid.head = sysid.head + 1U;
}

/* ------------------------------------------------------------------ 配置面 */

void APP_SysId_Init(void)
{
    if (sysid_initialised != 0U) {
        return;
    }
    sysid_initialised = 1U;

    memset(&sysid, 0, sizeof(sysid));
    memset(&sysid_shaping, 0, sizeof(sysid_shaping));
    sysid.state = APP_SYSID_STATE_IDLE;
    sysid.last_reason = "init";
    sysid.rig.azimuth_rad = DRV_SYSID_RIG_AZIMUTH_45_RAD;
    sysid.rig.axis_offset_above_cg_m = 0.0f;
    sysid.rig.imu_above_cg_m = 0.0f;
    sysid.angle_limit_rad = APP_SYSID_ANGLE_LIMIT_DEFAULT_RAD;
    sysid.residual_limit_rad_s = APP_SYSID_RESIDUAL_LIMIT_DEFAULT_RAD_S;
    sysid.sample_rate_hz = APP_SYSID_RATE_DEFAULT_HZ;
    sysid.inertia_kg_m2 = 0.0f;

    sysid.spec.profile = (uint8_t)DRV_SYSID_PROFILE_DOUBLET;
    /*
     * 默认激励 65 mrad/s（2026-09-27 由 150 改）。前馈力矩 = I·α_ff 正比于幅值，
     * 倾转力矩模型改准后同一个力矩数字对应的舵机角放大到约 2.3 倍（力臂
     * 0.0842/0.0825 → 0.0354 m）；幅值按同一比例（约 0.43）缩小，舵机实际
     * 摆动与 2026-09-27 那几趟光杆实录保持一致。
     */
    sysid.spec.amplitude_rad_s = 0.065f;
    sysid.spec.duration_ms = 8000U;
    sysid.spec.hold_ms = 250U;
    sysid.spec.repeat = 16U;  /* 8 s：单摆约 0.7 Hz，要多摆几个周期才定得住重力刚度 */
    sysid.spec.ramp_ms = 150U;
    sysid.angle_amplitude_rad = 0.0523598776f; /* first angle validation: 3 deg */
    sysid.servo_tilt_rad = APP_SYSID_SERVO_TILT_DEFAULT_RAD;
    sysid.spec.chirp_f0_hz = 0.3f;
    sysid.spec.chirp_f1_hz = 6.0f;
    sysid.spec.prbs_bit_ms = 40U;
    sysid.spec.prbs_seed = 1U;
    sysid.throttle_target_n = 0.0f;   /* 默认手动；上位机开跑前显式下发 */
    sysid.throttle_max_pct = 75.0f;
    sysid.phase = APP_SYSID_PHASE_IDLE;

    sysid_center_servos();
}

uint8_t APP_SysId_SetThrottle(float target_n, float max_pct)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();
    float ceiling = (airframe != NULL) ? airframe->max_total_force_n : 0.0f;

    if ((isfinite(target_n) == 0) || (isfinite(max_pct) == 0) ||
        (max_pct < APP_SYSID_THROTTLE_PCT_MIN) || (max_pct > APP_SYSID_THROTTLE_PCT_MAX)) {
        return 0U;
    }
    if (target_n != 0.0f) {
        if ((isfinite(ceiling) == 0) || (ceiling <= APP_SYSID_MIN_THRUST_N)) {
            return 0U;
        }
        if ((target_n < APP_SYSID_MIN_THRUST_N) || (target_n > ceiling)) {
            return 0U;
        }
    }
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.throttle_target_n = target_n;
    sysid.throttle_max_pct = max_pct;
    BSP_Critical_Exit(lock);
    return 1U;
}

void APP_SysId_GetThrottle(float *target_n, float *max_pct)
{
    APP_SysId_Init();
    if (target_n != NULL) { *target_n = sysid.throttle_target_n; }
    if (max_pct != NULL) { *max_pct = sysid.throttle_max_pct; }
}

APP_SysIdPhase APP_SysId_GetPhase(void)
{
    return (sysid.state == APP_SYSID_STATE_RUNNING) ? sysid.phase : APP_SYSID_PHASE_IDLE;
}

uint8_t APP_SysId_GetMotorPulse(uint16_t *pulse_us)
{
    /* SERVO 模式电机一拍都不碰：自动油门设置即使还留着也不生效。 */
    if ((sysid.state != APP_SYSID_STATE_RUNNING) || (sysid.throttle_target_n <= 0.0f) ||
        (sysid.mode == APP_SYSID_SERVO) ||
        (sysid.phase == APP_SYSID_PHASE_IDLE) || (pulse_us == NULL)) {
        return 0U;
    }
    *pulse_us = sysid.motor_pulse_us;
    return 1U;
}

uint8_t APP_SysId_SetRig(const DRV_SysIdRig *rig)
{
    float axis[3];

    if (rig == NULL) {
        return 0U;
    }
    if ((isfinite(rig->azimuth_rad) == 0) ||
        (isfinite(rig->axis_offset_above_cg_m) == 0) ||
        (isfinite(rig->imu_above_cg_m) == 0)) {
        return 0U;
    }
    /* 用几何模块自己的体检，不在这里另写一套判据。 */
    if (DRV_SysIdRig_Axis(rig, axis) != DRV_SYSID_RIG_OK) {
        return 0U;
    }
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.rig = *rig;
    BSP_Critical_Exit(lock);
    return 1U;
}

void APP_SysId_GetRig(DRV_SysIdRig *out) { APP_SysId_Init(); if (out != NULL) { *out = sysid.rig; } }

uint8_t APP_SysId_SetExcitation(const DRV_SysIdExcitation *spec)
{
    if (spec == NULL) {
        return 0U;
    }
    if (DRV_SysIdExcitation_Validate(spec) != DRV_SYSID_EXC_OK) {
        return 0U;
    }
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.spec = *spec;
    BSP_Critical_Exit(lock);
    return 1U;
}

void APP_SysId_GetExcitation(DRV_SysIdExcitation *out) { APP_SysId_Init(); if (out != NULL) { *out = sysid.spec; } }

uint8_t APP_SysId_SetInertia(float kg_m2)
{
    if ((isfinite(kg_m2) == 0) || (kg_m2 < 0.0f) || (kg_m2 > 10.0f)) {
        return 0U;
    }
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.inertia_kg_m2 = kg_m2;
    BSP_Critical_Exit(lock);
    return 1U;
}

float APP_SysId_GetInertia(void) { APP_SysId_Init(); return sysid.inertia_kg_m2; }

uint8_t APP_SysId_SetSampleRate(uint32_t hz)
{
    if ((hz < APP_SYSID_RATE_MIN_HZ) || (hz > APP_SYSID_RATE_MAX_HZ)) {
        return 0U;
    }
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.sample_rate_hz = hz;
    BSP_Critical_Exit(lock);
    return 1U;
}

uint32_t APP_SysId_GetSampleRate(void) { APP_SysId_Init(); return sysid.sample_rate_hz; }

uint8_t APP_SysId_SetAngleLimit(float rad)
{
    if ((isfinite(rad) == 0) || (rad <= 0.0f) ||
        (rad > APP_SYSID_ANGLE_LIMIT_MAX_RAD)) {
        return 0U;
    }
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.angle_limit_rad = rad;
    BSP_Critical_Exit(lock);
    return 1U;
}

float APP_SysId_GetAngleLimit(void) { APP_SysId_Init(); return sysid.angle_limit_rad; }

uint8_t APP_SysId_SetResidualLimit(float rad_s)
{
    if ((isfinite(rad_s) == 0) || (rad_s <= 0.0f) || (rad_s > 20.0f)) {
        return 0U;
    }
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.residual_limit_rad_s = rad_s;
    BSP_Critical_Exit(lock);
    return 1U;
}

float APP_SysId_GetResidualLimit(void) { APP_SysId_Init(); return sysid.residual_limit_rad_s; }

/* ------------------------------------------------------------------ 运行面 */

uint8_t APP_SysId_SetMode(APP_SysIdMode mode, float angle_amplitude_rad)
{
    APP_SysId_Init();
    if (mode > APP_SYSID_ALT || mode < APP_SYSID_FEEDFORWARD ||
        !isfinite(angle_amplitude_rad) || angle_amplitude_rad <= 0.0f ||
        angle_amplitude_rad > 0.2617993878f) { return 0U; }
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.mode = mode;
    sysid.angle_amplitude_rad = angle_amplitude_rad;
    BSP_Critical_Exit(lock);
    return 1U;
}

APP_SysIdMode APP_SysId_GetMode(void) { return sysid.mode; }

uint8_t APP_SysId_SetServoTilt(float rad)
{
    APP_SysId_Init();
    if (!isfinite(rad) || rad < APP_SYSID_SERVO_TILT_MIN_RAD ||
        rad > APP_SYSID_SERVO_TILT_MAX_RAD) { return 0U; }
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid.servo_tilt_rad = rad;
    BSP_Critical_Exit(lock);
    return 1U;
}

float APP_SysId_GetServoTilt(void) { APP_SysId_Init(); return sysid.servo_tilt_rad; }
uint8_t APP_SysId_IsEngaged(void) { return sysid.engaged; }
uint8_t APP_SysId_NeedsService(void)
{
    return APP_SysId_IsRunning() || sysid.notice_pending || sysid_ring_count() || pending_length;
}

uint8_t APP_SysId_Discard(void)
{
    uint32_t lock = BSP_Critical_Enter();
    if (APP_SysId_IsRunning()) { BSP_Critical_Exit(lock); return 0U; }
    sysid_ring_reset(); pending_length = 0U;
    sysid.notice_pending = sysid.last_batch_pending = 0U;
    BSP_Critical_Exit(lock);
    return 1U;
}

void APP_SysId_Hold(void)
{
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (!APP_SysId_IsRunning()) { sysid.engaged = 1U; sysid_center_servos(); }
    BSP_Critical_Exit(lock);
}

uint8_t APP_SysId_IsRunning(void) { return (sysid.state == APP_SYSID_STATE_RUNNING) ? 1U : 0U; }
APP_SysIdState APP_SysId_GetState(void) { return sysid.state; }
const char *APP_SysId_GetLastReason(void) { return (sysid.last_reason != NULL) ? sysid.last_reason : ""; }
uint16_t APP_SysId_GetRunId(void) { return sysid.run_id; }
uint32_t APP_SysId_GetDroppedSamples(void) { return sysid.dropped; }

static void sysid_finish(APP_SysIdState state, const char *reason)
{
    if (sysid.state != APP_SYSID_STATE_RUNNING) {
        return;
    }
    sysid.state = state;
    sysid.last_reason = (reason != NULL) ? reason : "";
    sysid.motor_capped = 0U;   /* 油门已交还，封顶标志不再有意义 */
    sysid.last_batch_pending = 1U;
    sysid_center_servos();
    /* The control task must never block on USB text. StreamTick emits this. */
    sysid.notice_pending = 1U;
}

void APP_SysId_Stop(const char *reason)
{
    APP_SysId_Init();
    uint32_t lock = BSP_Critical_Enter();
    if (sysid.state != APP_SYSID_STATE_RUNNING) {
        sysid_center_servos();
        BSP_Critical_Exit(lock);
        return;
    }
    if (sysid.phase == APP_SYSID_PHASE_RAMP_DOWN) {
        /* 数据已经完整收尾，回落中被停只是提前交还油门，不作废这一轮。 */
        sysid_finish(APP_SYSID_STATE_DONE, "complete");
    } else {
        sysid_finish(APP_SYSID_STATE_ABORTED, (reason != NULL) ? reason : "command");
    }
    BSP_Critical_Exit(lock);
}

uint8_t APP_SysId_Start(void)
{
    const DRV_Airframe_Params *airframe;
    float shaping[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    uint32_t total_ms = 0U;

    APP_SysId_Init();

    if (sysid.state == APP_SYSID_STATE_RUNNING) {
        APP_Control_QueueText("ERR sysid already running\r\n");
        return 0U;
    }
    if (APP_Ident_IsRunning() != 0U) {
        /* 两套辨识都接管舵机。同时跑的话稳定环只会听其中一个，另一个照samples
         * 记下一段"我发了这个"的假数据——那种数据看不出问题。 */
        APP_Control_QueueText("ERR sysid ident running\r\n");
        return 0U;
    }
    if (DRV_Airframe_IsValid() == 0U) {
        APP_Control_QueueText("ERR sysid airframe invalid\r\n");
        return 0U;
    }
    if (sysid.last_batch_pending || sysid_ring_count() || sysid.notice_pending || pending_length) {
        APP_Control_QueueText("ERR sysid previous run draining\r\n");
        return 0U;
    }
    if (DRV_SysIdExcitation_Validate(&sysid.spec) != DRV_SYSID_EXC_OK) {
        APP_Control_QueueText("ERR sysid excitation invalid\r\n");
        return 0U;
    }
    if (DRV_SysIdExcitation_TotalMs(&sysid.spec, &total_ms) != DRV_SYSID_EXC_OK) {
        APP_Control_QueueText("ERR sysid excitation invalid\r\n");
        return 0U;
    }
    if (sysid.mode == APP_SYSID_SERVO) {
        /*
         * 电机转不起来靠"没解锁"保证（跑的过程中一解锁即 rc_arm 中止），自动油门不看。
         * 电调命令窗口被抢占会断在半截序列上；地面点动在稳定环里排在辨识之后，
         * 会盖掉舵机指令、留下"我发了这个"的假数据——都先拒。
         */
        const char *busy = (sysid.last_armed != 0U) ? "needs disarmed" :
                           (APP_EscCommand_IsActive() != 0U) ? "esc command active" :
                           (APP_ServoJog_IsActive() != 0U) ? "servo jog active" : NULL;
        if (busy != NULL) {
            APP_Control_QueueText("ERR sysid servo mode %s\r\n", busy);
            return 0U;
        }
    } else if (sysid.throttle_target_n > 0.0f) {
        /* 先在这里拒，而不是开跑后第一拍再 abort：没解锁就点开始是最常见的误操作，
         * 一条能看懂的 ERR 比一个空的中止轮次有用得多。第一拍的安全门仍会再判一次。 */
        if (sysid.last_armed == 0U) {
            APP_Control_QueueText("ERR sysid not armed: arm with throttle stick low first\r\n");
            return 0U;
        }
        if (sysid.last_thr_low == 0U) {
            APP_Control_QueueText("ERR sysid throttle stick not low\r\n");
            return 0U;
        }
    }
    if (sysid.mode == APP_SYSID_ALT) {
        /* 高度辨识：要自动油门、按注入类型的幅值上限、测高有效（app_sysid_alt.h）。 */
        const char *alt_refusal = APP_SysIdAlt_Precheck(sysid.throttle_target_n,
                                                         sysid.spec.amplitude_rad_s);
        if (alt_refusal != NULL) {
            APP_Control_QueueText("ERR sysid alt %s\r\n", alt_refusal);
            return 0U;
        }
    }

    airframe = DRV_Airframe_Get();
    if (sysid.inertia_kg_m2 <= 0.0f) {
        /*
         * 没显式给假定惯量就从机体模型取 Ixx。45° 杆轴的有效惯量恒等于 Ixx
         * （nᵀJn，见 drv_sysid_rig.h），所以这里不需要再投影一次。
         */
        float guess = (airframe != NULL) ? airframe->ixx_kgm2 : 0.0f;

        if ((isfinite(guess) == 0) || (guess <= 0.0f)) {
            APP_Control_QueueText("ERR sysid inertia unknown, set SYSID INERTIA\r\n");
            return 0U;
        }
        sysid.inertia_kg_m2 = guess;
    }

    if (!APP_SysId_PortBegin(sysid.sample_rate_hz)) {
        APP_Control_QueueText("ERR sysid precheck: need fresh in-range thrust LUT voltage/eRPM; USB above 100Hz; no log export\r\n");
        return 0U;
    }
    uint32_t lock = BSP_Critical_Enter();
    sysid.run_id++;
    sysid.saturation_detail = 0U;
    sysid.engaged = 1U;
    sysid.stream_gap = 0U;
    DRV_RateControl_InitState(&sysid.rate_state);
    sysid_shaping_reset();
    sysid.last_reason = "running";
    sysid.started = 0U;
    sysid.start_ms = 0U;
    sysid.bad_dt_us = 0U;
    sysid.phase = APP_SYSID_PHASE_IDLE;
    sysid.phase_notice = 0U;
    sysid.motor_pulse_us = 0U;
    sysid.motor_capped = 0U;
    sysid.excite_angle0_rad = 0.0f;
    sysid.preroll_started = 0U;
    sysid.sample_period_us = 1000000U / sysid.sample_rate_hz;
    sysid.next_sample_us = 0U;
    sysid.first_batch_pending = 1U;
    sysid.last_batch_pending = 0U;
    sysid.run_flags = (sysid.mode == APP_SYSID_RATE) ? DRV_SYSID_FLAG_RATE :
                      (sysid.mode == APP_SYSID_ANGLE) ? DRV_SYSID_FLAG_ANGLE :
                      (sysid.mode == APP_SYSID_SERVO) ? DRV_SYSID_FLAG_SERVO :
                      (sysid.mode == APP_SYSID_ALT) ? DRV_SYSID_FLAG_ALT : 0U;
    sysid_ring_reset();
    sysid_center_servos();
    if (sysid.mode == APP_SYSID_ALT) {
        APP_SysIdAlt_Begin(&sysid.spec, sysid.throttle_max_pct);
    }
    sysid.state = APP_SYSID_STATE_RUNNING; /* publish after initialization */
    BSP_Critical_Exit(lock);

    /*
     * 行尾六项是本轮闭环用的指令整形与两级出口陷波（coax.att_ref_* / coax.rate_out_notch_* /
     * coax.rate_out_notch2_*，0 = 关）：上位机把整行存进 conditions.json 的 start，验证轮看得出
     * 用的是哪套控制律。最宽写法约 246 字符，仍在 APP_UART_TX_TEXT_SIZE − 1 以内
     * （tests/test_attitude_shaping.py 钉住）。
     */
    (void)DRV_COAX_CTRL_GetParam("coax.att_ref_wr_rad_s", &shaping[0]);
    (void)DRV_COAX_CTRL_GetParam("coax.att_ref_delay_ms", &shaping[1]);
    (void)DRV_COAX_CTRL_GetParam("coax.rate_out_notch_hz", &shaping[2]);
    (void)DRV_COAX_CTRL_GetParam("coax.rate_out_notch_q", &shaping[3]);
    (void)DRV_COAX_CTRL_GetParam("coax.rate_out_notch2_hz", &shaping[4]);
    (void)DRV_COAX_CTRL_GetParam("coax.rate_out_notch2_q", &shaping[5]);
    APP_Control_QueueText(
        "SYSID start run=%u profile=%u amp_mrad_s=%ld dur_ms=%lu rate_hz=%lu "
        "I=%ld ugm2 psi_mrad=%ld auto=%u target_cn=%ld "
        "ref_wr_mrad_s=%ld ref_td_us=%ld onotch_mhz=%ld onotch_q_milli=%ld "
        "onotch2_mhz=%ld onotch2_q_milli=%ld\r\n",
        (unsigned int)sysid.run_id,
        (unsigned int)sysid.spec.profile,
        (long)(sysid.spec.amplitude_rad_s * 1000.0f),
        (unsigned long)total_ms,
        (unsigned long)sysid.sample_rate_hz,
        (long)(sysid.inertia_kg_m2 * 1000000.0f),
        (long)(sysid.rig.azimuth_rad * 1000.0f),
        (unsigned int)((sysid.throttle_target_n > 0.0f) && (sysid.mode != APP_SYSID_SERVO)),
        (long)(sysid.throttle_target_n * 100.0f),
        (long)lroundf(shaping[0] * 1000.0f),
        (long)lroundf(shaping[1] * 1000.0f),
        (long)lroundf(shaping[2] * 1000.0f),
        (long)lroundf(shaping[3] * 1000.0f),
        (long)lroundf(shaping[4] * 1000.0f),
        (long)lroundf(shaping[5] * 1000.0f));
    if (sysid.mode == APP_SYSID_ALT) {
        /* 上一行最宽已约 246 字符，ALT 溯源另起紧跟的一行（SYSID ALTSTART）。 */
        APP_SysIdAlt_ReportStart(sysid.run_id);
    }
    return 1U;
}

void APP_SysId_GetServoTargets(uint16_t *alpha_us, uint16_t *beta_us)
{
    APP_SysId_Init();
    if (sysid.state != APP_SYSID_STATE_RUNNING) {
        /*
         * STOP 在命令任务里执行，可能插进一次 Update 中间，那次 Update 会把刚解出
         * 的倾角写回来。不在跑就只给中位——在唯一的读出口上兜住，不追每个写入点。
         */
        sysid_center_servos();
    }
    if (alpha_us != NULL) {
        *alpha_us = sysid.alpha_us;
    }
    if (beta_us != NULL) {
        *beta_us = sysid.beta_us;
    }
}

/*
 * 安全门。返回非 NULL 即 abort 理由。
 *
 * 顺序是刻意的：先判链路与解锁（操作员手上的开关最优先），再判数据可信度
 * （IMU、轴向残差），最后才判物理量超限。这样报出来的第一个理由总是最根本的
 * 那个——链路断了之后角度当然也会超限，但报"角度超限"会把人引到错的方向。
 */
static const char *sysid_gate(const APP_SysIdObserve *obs,
                              float angle_rad,
                              float residual_rad_s,
                              float thrust_n,
                              uint8_t check_thrust)
{
    if (obs->rc_link_ok == 0U) {
        return "rc_lost";
    }
    if (sysid.mode == APP_SYSID_SERVO) {
        /* 舵机单独辨识的前提是电机转不起来：飞手一解锁就停。 */
        if (obs->rc_armed != 0U) {
            return "rc_arm";
        }
    } else if (obs->rc_armed == 0U) {
        return "rc_disarm";
    }
    /* 自动油门期间飞手一推杆就交还：这是飞手手上最直接的"我来接管"。 */
    if ((sysid.throttle_target_n > 0.0f) && (sysid.mode != APP_SYSID_SERVO) &&
        (obs->rc_throttle_low == 0U)) {
        return "rc_throttle_override";
    }
    if (obs->imu_valid == 0U) {
        return "imu_stale";
    }
    if (obs->actuator_inhibit) { return "actuator_busy"; }
    /* 未解锁时地面点动不让位，会在稳定环里盖掉辨识的舵机指令（见 Start）。 */
    if ((sysid.mode == APP_SYSID_SERVO) && (APP_ServoJog_IsActive() != 0U)) {
        return "actuator_busy";
    }
    if (check_thrust && !obs->thrust_valid) { return "thrust_stale"; }
    if (!isfinite(obs->roll_rad) || !isfinite(obs->pitch_rad) ||
        !isfinite(obs->gyro_rad_s[0]) || !isfinite(obs->gyro_rad_s[1]) ||
        !isfinite(obs->gyro_rad_s[2]) || !isfinite(obs->gyro_ctrl_rad_s[0]) ||
        !isfinite(obs->gyro_ctrl_rad_s[1]) || !isfinite(obs->gyro_ctrl_rad_s[2])) { return "nonfinite"; }
    if (residual_rad_s > sysid.residual_limit_rad_s) {
        return "axis_residual";
    }
    if (fabsf(angle_rad) > sysid.angle_limit_rad) {
        return "angle_limit";
    }
    if (check_thrust && (thrust_n < APP_SYSID_MIN_THRUST_N)) {
        return "thrust_low";
    }
    return NULL;
}

/* 回落阶段数据已经完整，此时的任何停机理由都只是"提前交还油门"。 */
static void sysid_stop_on(const char *reason)
{
    if (sysid.phase == APP_SYSID_PHASE_RAMP_DOWN) {
        sysid_finish(APP_SYSID_STATE_DONE, "complete");
    } else {
        sysid_finish(APP_SYSID_STATE_ABORTED, reason);
    }
}

static void sysid_enter_phase(APP_SysIdPhase phase, uint32_t now_ms, float thrust_n)
{
    sysid.phase = phase;
    sysid.phase_start_ms = now_ms;
    sysid.phase_notice_phase = phase;
    sysid.phase_notice_pulse = sysid.motor_pulse_us;
    sysid.phase_notice_thrust = thrust_n;
    sysid.phase_notice = 1U;
}

static uint16_t sysid_target_pulse(void)
{
    const float span = (float)(BSP_PWM_ESC_MAX_US - BSP_PWM_ESC_MIN_US);
    const uint16_t cap_us = (uint16_t)((float)BSP_PWM_ESC_MIN_US +
                                       sysid.throttle_max_pct * span / 100.0f + 0.5f);
    /* 与读回推力（MotorPulseToTotalThrust）同一张装进分配器的推力表：单桨 = 合推力/2。 */
    uint16_t pulse = DRV_COAX_CTRL_ThrustToMotorPulse(0.5f * sysid.throttle_target_n);

    sysid.motor_capped = (pulse > cap_us) ? 1U : 0U;
    return (pulse > cap_us) ? cap_us : pulse;
}

static uint16_t sysid_lerp_pulse(uint16_t from, uint16_t to, uint32_t t_ms, uint32_t span_ms)
{
    int32_t delta = (int32_t)to - (int32_t)from;

    if ((span_ms == 0U) || (t_ms >= span_ms)) {
        return to;
    }
    return (uint16_t)((int32_t)from + (delta * (int32_t)t_ms) / (int32_t)span_ms);
}

/* 入环，入环前在临界区里复查仍在跑（STOP 可能插进了本拍）。返回 0 表示本轮已结束。 */
static uint8_t sysid_push_if_running(const DRV_SysIdSample *sample, uint32_t t_us,
                                     uint8_t erpm_valid)
{
    uint32_t lock = BSP_Critical_Enter();
    if (sysid.state != APP_SYSID_STATE_RUNNING) {
        BSP_Critical_Exit(lock);
        return 0U;
    }
    sysid_ring_push(sample, t_us, erpm_valid);
    BSP_Critical_Exit(lock);
    return 1U;
}

/*
 * 激励跑完那一拍的收尾，在临界区里复查仍在跑。末条入环与这里之间 STOP 可能已按
 * aborted 收了尾；不复查就会再补一条 phase=done / ramp_down，主机先看到"完成"、
 * 紧接着又是"中止"。以先到的那个为准。
 */
static void sysid_complete(uint32_t now_ms, float thrust_n, uint8_t ramp_down)
{
    uint32_t lock = BSP_Critical_Enter();

    if (sysid.state == APP_SYSID_STATE_RUNNING) {
        if (ramp_down != 0U) {
            /* 采样到此为止：尾批带 LAST；油门再用 1 s 平滑回落后才算结束。 */
            sysid.last_batch_pending = 1U;
            sysid.ramp_from_us = sysid.motor_pulse_us;
            sysid_center_servos();
            sysid_enter_phase(APP_SYSID_PHASE_RAMP_DOWN, now_ms, thrust_n);
        } else {
            if (sysid.mode == APP_SYSID_SERVO) {
                sysid_enter_phase(APP_SYSID_PHASE_DONE, now_ms, 0.0f);
            }
            sysid_finish(APP_SYSID_STATE_DONE, "complete");
        }
    }
    BSP_Critical_Exit(lock);
}

/* 本拍是否落在采样网格上；是则推进网格。迟到太多时重新对齐，不补发——补发会在环里挤成一团。 */
static uint8_t sysid_sample_due(uint32_t now_us, uint8_t force)
{
    if ((force == 0U) && ((int32_t)(now_us - sysid.next_sample_us) < 0)) {
        return 0U;
    }
    sysid.next_sample_us += sysid.sample_period_us;
    if ((int32_t)(now_us - sysid.next_sample_us) >= 0) {
        sysid.next_sample_us = now_us + sysid.sample_period_us;
    }
    return 1U;
}

/*
 * 上/下桨电转速。数据源与判据同推力查补表（app_thrust_lut.c）：双向 DShot 回包
 * 有效、不过期；电调明确回报"未旋转"是有效回包、记 0。桨位按桨叶标定查通道，
 * 不按下标——接反时按下标记就是上下对调的数据。返回 1 = 两路都有效。
 * 年龄按有符号差算：回包在本拍取时间之后才落地（差 -1 ms）也算新鲜，不回绕成"过期"。
 */
static uint8_t sysid_read_rotor_erpm(uint32_t now_ms, uint32_t *upper, uint32_t *lower)
{
    BSP_DShotRxSnapshot rx;
    uint32_t erpm[DRV_PROP_ROLE_COUNT] = { 0U, 0U };
    uint8_t all_valid = 1U;

    BSP_DShotRx_GetSnapshot(&rx);
    for (uint8_t role = 0U; role < (uint8_t)DRV_PROP_ROLE_COUNT; ++role) {
        /* 通道 1..N → 下标 0..N-1；未标定返回 0，减 1 回绕成大数，同样落进"无效"。 */
        const uint32_t index = (uint32_t)DRV_PropMap_EscChannelForRole(role) - 1U;

        if ((rx.available == 0U) || (index >= DRV_PROP_ESC_CHANNEL_COUNT) ||
            (rx.valid[index] == 0U) ||
            ((int32_t)(now_ms - rx.sample_ms[index]) > (int32_t)SYSID_ERPM_FRESH_MS)) {
            all_valid = 0U;
            continue;
        }
        erpm[role] = (rx.not_spinning[index] != 0U) ? 0U : rx.erpm[index];
    }
    *upper = erpm[DRV_PROP_ROLE_UPPER];
    *lower = erpm[DRV_PROP_ROLE_LOWER];
    return all_valid;
}

/*
 * 沿杆轴的倾转方向 d = (sinψ·sgn(L_pitch), cosψ·sgn(L_roll))，机体倾转 X/Y 分量，
 * 返回控制器倾角上限。力臂取分配器此刻在用的那一份（每次取参都由机体模型重算），
 * 所以 d 与 FF 为绕杆轴正力矩反解出的倾转逐轴同号（推导见 app_sysid.h 文件头）。
 */
static float sysid_rod_tilt_direction(float dir[2])
{
    DRV_COAX_CTRL_Params params;

    DRV_COAX_CTRL_GetParams(&params);
    dir[0] = sinf(sysid.rig.azimuth_rad) * ((params.pitch_tilt_lever_arm_m < 0.0f) ? -1.0f : 1.0f);
    dir[1] = cosf(sysid.rig.azimuth_rad) * ((params.roll_tilt_lever_arm_m < 0.0f) ? -1.0f : 1.0f);
    return params.tilt_limit_rad;
}

/* 下发倾转在 d 上的投影：记录字段 servo_tilt，所有模式同一口径。 */
static float sysid_servo_tilt_of(float body_x_rad, float body_y_rad, const float dir[2])
{
    return (body_x_rad * dir[0]) + (body_y_rad * dir[1]);
}

/* 一条样本里与模式无关的那部分：测量量与转速，其余清 0。返回转速是否有效。 */
static uint8_t sysid_sample_measurements(DRV_SysIdSample *sample, const APP_SysIdObserve *obs,
                                         float axis_angle_rad)
{
    memset(sample, 0, sizeof(*sample));
    sample->gyro_rad_s[0] = obs->gyro_rad_s[0];
    sample->gyro_rad_s[1] = obs->gyro_rad_s[1];
    sample->gyro_rad_s[2] = obs->gyro_rad_s[2];
    sample->angle_rad = axis_angle_rad;
    return sysid_read_rotor_erpm(obs->now_ms, &sample->erpm, &sample->erpm_lower);
}

/* 零激励前导：舵机回中、力矩 0，按同一采样网格记录，时间戳与随后的激励连续。 */
static void sysid_record_preroll(const APP_SysIdObserve *obs, float axis_angle_rad, float thrust_n)
{
    DRV_SysIdSample sample;
    uint8_t erpm_valid;

    if (sysid.preroll_started == 0U) {
        sysid.preroll_started = 1U;
        sysid.next_sample_us = obs->now_us;
    }
    if (sysid_sample_due(obs->now_us, 0U) == 0U) {
        return;
    }
    erpm_valid = sysid_sample_measurements(&sample, obs, axis_angle_rad);
    sample.thrust_n = thrust_n;
    /* ANGLE 的目标以激励开始时的平衡角为零点；前导里目标就是当前角，跟踪误差为 0。 */
    sample.angle_sp_rad = (sysid.mode == APP_SYSID_ANGLE) ? axis_angle_rad : 0.0f;
    (void)sysid_push_if_running(&sample, obs->now_us, erpm_valid);
}

static void sysid_begin_excite(const APP_SysIdObserve *obs, float axis_angle_rad, float thrust_n)
{
    sysid.start_ms = obs->now_ms;
    if (sysid.preroll_started == 0U) {
        sysid.next_sample_us = obs->now_us;   /* 手动油门没有前导：从这一拍起采 */
    }
    /* 质心在杆下方时机体自然下垂到一个平衡角，不一定是 0°。ANGLE 以它为零点。 */
    sysid.excite_angle0_rad = axis_angle_rad;
    DRV_RateControl_InitState(&sysid.rate_state);
    sysid_shaping_reset();
    sysid_enter_phase(APP_SYSID_PHASE_EXCITE, obs->now_ms, thrust_n);
}

/* Rodrigues rotation about the confirmed FLU rod. Reuses production SO(3) P
 * and rate PID, then projects moments onto the single free degree of freedom. */
static void sysid_rotation(const float n[3], float theta, float r[3][3])
{
    const float c = cosf(theta), s = sinf(theta);
    const float skew[3][3] = {{0,-n[2],n[1]}, {n[2],0,-n[0]}, {-n[1],n[0],0}};
    for (unsigned i=0; i<3; ++i) for (unsigned j=0; j<3; ++j) {
        r[i][j] = (i==j ? c : 0.0f) + (1.0f-c)*n[i]*n[j] + s*skew[i][j];
    }
}

/*
 * ANGLE 的参考模型：与在飞同一个 DRV_AttRef（参数 coax.att_ref_wr_rad_s / _delay_ms），
 * 标量走杆轴（只用第 0 轴）。关着（ωr = 0）返回 0，调用方照旧直接用 angle_sp。
 * 开着时 *ref_angle = θ_ref(t − Td)、*ref_rate = θ̇_ref(t − Td)、*ref_accel = θ̈_ref(t)；
 * 本拍就提交——辨识每拍只算一遍，没有飞控那种同拍重算。
 */
static uint8_t sysid_shape_angle(const DRV_COAX_CTRL_Params *params, float angle,
                                 float angle_sp, float dt, float *ref_angle,
                                 float *ref_rate, float *ref_accel)
{
    const float command[DRV_ATT_REF_AXES] = { angle_sp, 0.0f };
    const float align[DRV_ATT_REF_AXES] = { angle, 0.0f };
    DRV_AttRefStep step;
    DRV_AttRefOutput shaped;

    if (!(params->att_ref_wr_rad_s > 0.0f)) {
        return 0U;
    }
    DRV_AttRef_Evaluate(&sysid_shaping.ref, params->att_ref_wr_rad_s,
                        params->att_ref_delay_ms * 1.0e-3f, command, align, dt,
                        &step, &shaped);
    DRV_AttRef_Commit(&sysid_shaping.ref, &step);
    *ref_angle = shaped.angle[0];
    *ref_rate = shaped.rate[0];
    *ref_accel = shaped.accel[0];
    return 1U;
}

static uint8_t sysid_closed_loop(const APP_SysIdObserve *obs, float angle,
    float dt, DRV_SysIdExcSample *exc, float *angle_sp, float *moment)
{
    float n[3];
    DRV_COAX_CTRL_Params params;
    DRV_RateControl_Input in = {0};
    DRV_RateControl_Output out;
    DRV_COAX_CTRL_GetParams(&params);
    if (DRV_SysIdRig_Axis(&sysid.rig, n) != DRV_SYSID_RIG_OK) { return 0U; }
    in.dt_s = dt; in.measurement_valid = 1U; in.integrator_enable = 1U;
    for (unsigned i=0; i<3; ++i) {
        in.omega[i] = obs->gyro_ctrl_rad_s[i];   /* 控制用（陷波后）；记录仍是原始陀螺 */
        in.omega_sp[i] = exc->omega_sp_rad_s * n[i];
        /* 模型前馈只有一种：ANGLE 开参考模型时的 θ̈_ref（与在飞同一条 ff 路径），此外为 0。 */
        in.inertia[i] = sysid.inertia_kg_m2;
        in.saturation_positive[i] = 3.0f; in.saturation_negative[i] = -3.0f;
    }
    if ((sysid.mode == APP_SYSID_ANGLE) || (sysid.mode == APP_SYSID_ALT)) {
        /* ALT 走同一条角度环，激励恒为 0 → 目标恒为杆轴角 0。 */
        DRV_AttitudeControl_Input att = {0};
        DRV_AttitudeControl_Output target;
        float ref_angle, ref_rate = 0.0f, ref_accel = 0.0f;
        uint8_t shaped;
        /* 记录的 angle_sp 始终是指令本身；参考模型只改角度环跟的目标。 */
        *angle_sp = sysid.angle_amplitude_rad * exc->omega_sp_rad_s / sysid.spec.amplitude_rad_s;
        ref_angle = *angle_sp;
        shaped = sysid_shape_angle(&params, angle, *angle_sp, dt,
                                   &ref_angle, &ref_rate, &ref_accel);
        sysid_rotation(n, angle, att.actual_rotation);
        sysid_rotation(n, ref_angle, att.desired_rotation);
        if (shaped != 0U) {
            /* 绕 n 转动时 n 本身不动：期望系里的角速度前馈就是 θ̇_ref·n。 */
            for (unsigned i=0; i<3; ++i) {
                att.desired_rate_in_desired_frame[i] = ref_rate * n[i];
                in.alpha_ff[i] = ref_accel * n[i];
            }
        }
        if (!DRV_AttitudeControl_Step(&params.attitude, &att, &target)) { return 0U; }
        memcpy(in.omega_sp, target.omega_sp, sizeof(in.omega_sp));
        exc->omega_sp_rad_s = target.omega_sp[0]*n[0] + target.omega_sp[1]*n[1];
    }
    if (!DRV_RateControl_Step(&params.rate, &sysid.rate_state, &in, &out)) { return 0U; }
    /*
     * 出口陷波：RATE 与 ANGLE 都走，与在飞同一段代码（只滤反馈、两级串联、按同一组限幅重钳）。
     * 第二级关着（notch2_hz = 0 且未生效）时 Configure 把它清成直通，结果与只有第一级逐位相同。
     */
    if ((params.rate_out_notch_hz > 0.0f) || (sysid_shaping.notch.active != 0U) ||
        (params.rate_out_notch2_hz > 0.0f) || (sysid_shaping.notch2.active != 0U)) {
        (void)DRV_MomentNotch_Configure(&sysid_shaping.notch, params.rate_out_notch_hz,
                                        params.rate_out_notch_q, dt);
        (void)DRV_MomentNotch_Configure(&sysid_shaping.notch2, params.rate_out_notch2_hz,
                                        params.rate_out_notch2_q, dt);
        DRV_MomentNotch_ApplyCascadeToRateOutput(&sysid_shaping.notch, &sysid_shaping.notch2,
                                                 &in, &out);
        DRV_MomentNotch_Commit(&sysid_shaping.notch);
        DRV_MomentNotch_Commit(&sysid_shaping.notch2);
    }
    *moment = out.moment_cmd[0]*n[0] + out.moment_cmd[1]*n[1];
    return 1U;
}

/*
 * 舵机脉宽落在标定端点 = 倾转被标定行程（min/max）夹住了，或恰好顶到端点（差不到
 * 半个量化步长）。标定要求中位离两端都至少 50 us，回中与小倾转碰不到这里，
 * 所以不必反算倾角、也不必在这里再抄一遍脉宽换算律。
 */
static uint8_t sysid_servo_at_endpoint(void)
{
    DRV_COAX_CTRL_ServoCalibration cal;

    DRV_COAX_CTRL_GetServoCalibration(&cal);
    return ((sysid.alpha_us <= cal.min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]) ||
            (sysid.alpha_us >= cal.max_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]) ||
            (sysid.beta_us <= cal.min_us[DRV_COAX_CTRL_SERVO_BETA_INDEX]) ||
            (sysid.beta_us >= cal.max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX])) ? 1U : 0U;
}

/*
 * SERVO 模式（电机不转）的一拍：零激励前导 → 激励 → 结束。
 *
 * 推力相关的门（推力不足、推力来源过期）不走，FF 的饱和门（按力矩比对）也不走：
 * 推力为 0 时力矩恒为 0，比不出东西。链路/解锁/IMU/杆轴残差/角度/节拍照判。
 * 倾转不经反解器，所以在这里自己夹到控制器倾角上限；舵机标定行程再夹一次就
 * 改判脉宽是否顶到端点（actuator_saturated）。
 */
static void sysid_servo_step(const APP_SysIdObserve *obs, float axis_angle_rad,
                             float residual_rad_s)
{
    DRV_SysIdExcSample excitation;
    DRV_SysIdSample sample;
    const char *abort_reason;
    float dir[2], tilt_limit_rad, shape, tilt_rad, body_x_rad, body_y_rad;

    abort_reason = sysid_gate(obs, axis_angle_rad, residual_rad_s, 0.0f, 0U);
    if (abort_reason != NULL) {
        sysid_finish(APP_SYSID_STATE_ABORTED, abort_reason);
        return;
    }
    if (sysid.phase == APP_SYSID_PHASE_IDLE) {
        sysid_enter_phase(APP_SYSID_PHASE_PREROLL, obs->now_ms, 0.0f);
    }
    if (sysid.phase == APP_SYSID_PHASE_PREROLL) {
        sysid_center_servos();
        sysid_record_preroll(obs, axis_angle_rad, 0.0f);
        if ((uint32_t)(obs->now_ms - sysid.phase_start_ms) < APP_SYSID_PREROLL_MS) {
            return;
        }
        sysid_begin_excite(obs, axis_angle_rad, 0.0f);   /* 本拍就开始激励（t=0） */
    }

    if (DRV_SysIdExcitation_Eval(&sysid.spec, obs->now_ms - sysid.start_ms, &excitation) !=
        DRV_SYSID_EXC_OK) {
        sysid_finish(APP_SYSID_STATE_ABORTED, "excitation");
        return;
    }
    /* 现有剖面按单位幅值归一化（Validate 保证 amp > 0），再乘舵机倾转幅值。 */
    shape = excitation.omega_sp_rad_s / sysid.spec.amplitude_rad_s;
    shape = fmaxf(-1.0f, fminf(1.0f, sysid_finite_or(shape, 0.0f)));
    tilt_limit_rad = sysid_rod_tilt_direction(dir);
    tilt_rad = fmaxf(-tilt_limit_rad, fminf(tilt_limit_rad, sysid.servo_tilt_rad * shape));
    if (excitation.finished != 0U) {
        tilt_rad = 0.0f;   /* 末条与回中后的输出一致 */
    }
    body_x_rad = tilt_rad * dir[0];
    body_y_rad = tilt_rad * dir[1];
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(body_x_rad, body_y_rad,
                                           &sysid.alpha_us, &sysid.beta_us);
    if (sysid_servo_at_endpoint() != 0U) {
        /* 被标定行程夹短了：记录的 servo_tilt 不再是舵机真走的量。不限幅，这一趟作废。 */
        sysid_finish(APP_SYSID_STATE_ABORTED, "actuator_saturated");
        return;
    }

    if (sysid_sample_due(obs->now_us, excitation.finished) != 0U) {
        uint8_t erpm_valid = sysid_sample_measurements(&sample, obs, axis_angle_rad);

        /* 推力、力矩、ω_sp、α_ff、angle_sp 在本模式恒为 0：输入量只有 servo_tilt。 */
        sample.tilt_cmd_x_rad = body_x_rad;
        sample.tilt_cmd_y_rad = body_y_rad;
        sample.servo_tilt_rad = sysid_servo_tilt_of(body_x_rad, body_y_rad, dir);
        if (sysid_push_if_running(&sample, obs->now_us, erpm_valid) == 0U) {
            return;   /* STOP 插进了本拍 */
        }
    }
    if (excitation.finished != 0U) {
        sysid_complete(obs->now_ms, 0.0f, 0U);
    }
}

/*
 * 绕杆轴的力矩链（FF/RATE/ANGLE 的激励段与 ALT 的姿态保持共用）：
 * 前馈 τ_n = I_est · α_ff，打在杆轴上（RATE/ANGLE/ALT 再过闭环），交给**在飞的那个反解器**，
 * 再用同一张前向表复核实得力矩。I_est 填错只影响幅度，不影响结论——实测/期望角加速度
 * 之比就是 I_true/I_est。返回 NULL = 本拍舵机脉宽已写入；否则是中止理由。
 */
static const char *sysid_moment_chain(const APP_SysIdObserve *obs, float angle_rad, float dt_s,
                                      float thrust_n, DRV_SysIdExcSample *excitation,
                                      float *angle_sp, float *body_x_rad, float *body_y_rad,
                                      float achieved[3])
{
    float moment_body[3] = {0.0f, 0.0f, 0.0f};
    float moment_n = sysid.inertia_kg_m2 * excitation->alpha_ff_rad_s2;

    if (fabsf(excitation->alpha_ff_rad_s2) > 327.0f) {
        return "record_range";
    }
    if (sysid.mode != APP_SYSID_FEEDFORWARD &&
        !sysid_closed_loop(obs, angle_rad, dt_s, excitation, angle_sp, &moment_n)) {
        return "controller";
    }
    if (excitation->finished) { moment_n = 0.0f; } /* final record matches centered output */
    if (DRV_SysIdRig_MomentAboutAxis(&sysid.rig, moment_n, moment_body) != DRV_SYSID_RIG_OK) {
        return "rig";
    }
    if (DRV_COAX_CTRL_SolveBodyTiltFromMoment(moment_body, thrust_n,
                                              body_x_rad, body_y_rad) == 0U) {
        /*
         * 反解失败只有两种可能：推力不够（上面的门已经拦过）或者参数坏了。
         * 这里不静默回中继续跑——那会记下一段"发了力矩其实没发"的数据。
         */
        return "solve";
    }

    DRV_COAX_CTRL_BodyTiltRadToServoPulses(*body_x_rad, *body_y_rad,
                                           &sysid.alpha_us, &sysid.beta_us);
    uint8_t achieved_valid = DRV_COAX_CTRL_MomentFromServoPulses(
        thrust_n, sysid.alpha_us, sysid.beta_us, achieved);
    if (!achieved_valid ||
        fabsf(achieved[0]-moment_body[0]) > 0.004f + 0.05f*fabsf(moment_body[0]) ||
        fabsf(achieved[1]-moment_body[1]) > 0.004f + 0.05f*fabsf(moment_body[1]) ||
        fabsf(moment_n) > 3.27f) {
        /* Includes both allocator clipping and final mechanical pulse limits. */
        sysid.saturation_detail = achieved_valid ? 1U : 2U;
        sysid.saturation_force = thrust_n;
        for (unsigned i = 0; i < 2; ++i) {
            sysid.saturation_requested[i] = moment_body[i];
            sysid.saturation_achieved[i] = achieved_valid ? achieved[i] : 0.0f;
        }
        sysid.saturation_pulses[0] = sysid.alpha_us;
        sysid.saturation_pulses[1] = sysid.beta_us;
        return "actuator_saturated";
    }
    return NULL;
}

/*
 * 高度辨识（ALT）的一拍。序列、高度环与注入在 app_sysid_alt.c；这里只把它接到本模块的
 * 安全门（遥控/IMU/杆轴等排在高度门之前，先报根本原因）、力矩链（同 ANGLE，目标恒为
 * 杆轴角 0，起升/回落舵机回中）、采样网格（全程采样）与油门出口上。
 */
static void sysid_alt_step(const APP_SysIdObserve *obs, float angle_rad, float residual_rad_s,
                           float dt_s)
{
    APP_SysIdAltOutput alt;
    DRV_SysIdExcSample level = { 0.0f, 0.0f, 0U };
    DRV_SysIdSample sample;
    const char *reason;
    float angle_sp = 0.0f, body_x = 0.0f, body_y = 0.0f, dir[2];
    float achieved[3] = { 0.0f, 0.0f, 0.0f };

    APP_SysIdAlt_Step(obs, sysid.phase, obs->now_ms - sysid.phase_start_ms, dt_s, &alt);
    reason = sysid_gate(obs, angle_rad, residual_rad_s, alt.thrust_n, alt.check_thrust);
    if ((reason != NULL) || (alt.abort_reason != NULL)) {
        sysid_stop_on((reason != NULL) ? reason : alt.abort_reason);
        return;
    }
    sysid.motor_pulse_us = alt.motor_pulse_us;
    sysid.motor_capped = alt.capped;
    if (alt.phase != sysid.phase) {
        sysid_enter_phase(alt.phase, obs->now_ms, alt.thrust_n);
    }
    if (alt.hold_attitude == 0U) {
        sysid_center_servos();
    } else {
        reason = sysid_moment_chain(obs, angle_rad, dt_s, alt.thrust_n, &level, &angle_sp,
                                    &body_x, &body_y, achieved);
        if (reason != NULL) {
            sysid_finish(APP_SYSID_STATE_ABORTED, reason);
            return;
        }
    }
    if (sysid.preroll_started == 0U) {
        sysid.preroll_started = 1U;
        sysid.next_sample_us = obs->now_us;   /* 从第一拍起就采（含起升） */
    }
    if (sysid_sample_due(obs->now_us, alt.finished) != 0U) {
        const uint8_t erpm_valid = sysid_sample_measurements(&sample, obs, angle_rad);

        sample.omega_sp_rad_s = level.omega_sp_rad_s;   /* 角度环给出的杆轴角速度指令 */
        sample.tilt_cmd_x_rad = body_x;
        sample.tilt_cmd_y_rad = body_y;
        sample.thrust_n = alt.thrust_n;                 /* 封顶后实际下发脉宽对应的推力 */
        sample.torque_n_m = achieved[0]*cosf(sysid.rig.azimuth_rad) + achieved[1]*sinf(sysid.rig.azimuth_rad);
        (void)sysid_rod_tilt_direction(dir);
        sample.servo_tilt_rad = sysid_servo_tilt_of(body_x, body_y, dir);
        APP_SysIdAlt_FillSample(&sample, obs);
        if (sysid_push_if_running(&sample, obs->now_us, erpm_valid) == 0U) {
            return;   /* STOP 插进了本拍 */
        }
    }
    if (alt.finished != 0U) {
        sysid_complete(obs->now_ms, alt.thrust_n, 0U);
    }
}

void APP_SysId_Update(const APP_SysIdObserve *obs)
{
    DRV_SysIdExcSample excitation;
    DRV_SysIdSample sample;
    const char *abort_reason;
    float axis_angle_rad;
    float residual_rad_s = 0.0f;
    float thrust_n;
    float body_x_rad = 0.0f;
    float body_y_rad = 0.0f;
    float achieved[3], angle_sp = 0.0f, dt_s = 0.002f, tilt_dir[2];
    uint32_t elapsed_ms;
    uint8_t auto_throttle;

    APP_SysId_Init();

    if (obs == NULL) { return; }
    APP_SysIdAlt_Observe(obs);   /* 最近测高：ALT 开跑检查与状态行用 */
    sysid.last_armed = obs->rc_armed;
    sysid.last_thr_low = obs->rc_throttle_low;
    if (!obs->rc_armed && sysid.state != APP_SYSID_STATE_RUNNING) { sysid.engaged = 0U; }
    if (sysid.state != APP_SYSID_STATE_RUNNING) {
        return;
    }
    auto_throttle = (sysid.throttle_target_n > 0.0f) ? 1U : 0U;

    if (sysid.started == 0U) {
        sysid.started = 1U;
        sysid.idle_pulse_us = obs->throttle_us;
        sysid.motor_pulse_us = (sysid.mode == APP_SYSID_SERVO) ? 0U : obs->throttle_us;
    } else {
        uint32_t dt_us = (uint32_t)(obs->now_us - sysid.previous_us);

        dt_s = (float)dt_us * 1.0e-6f;
        if (dt_s < 0.0005f || dt_s > 0.02f) {
            sysid.bad_dt_us = dt_us;
            sysid_stop_on("control_dt"); return;
        }
    }
    sysid.previous_us = obs->now_us;

    /* 绕杆轴的转角：小角度下姿态矢量在 n 上的投影。 */
    axis_angle_rad = (obs->roll_rad * cosf(sysid.rig.azimuth_rad)) +
                     (obs->pitch_rad * sinf(sysid.rig.azimuth_rad));
    (void)DRV_SysIdRig_RateResidual(&sysid.rig, obs->gyro_rad_s, &residual_rad_s);

    if (sysid.mode == APP_SYSID_SERVO) {
        sysid_servo_step(obs, axis_angle_rad, residual_rad_s);
        return;
    }
    if (sysid.mode == APP_SYSID_ALT) {
        sysid_alt_step(obs, axis_angle_rad, residual_rad_s, dt_s);
        return;
    }

    if (sysid.phase == APP_SYSID_PHASE_IDLE) {
        if (auto_throttle != 0U) {
            sysid_enter_phase(APP_SYSID_PHASE_RAMP_UP, obs->now_ms, 0.0f);
        } else {
            sysid_begin_excite(obs, axis_angle_rad, sysid_finite_or(
                DRV_COAX_CTRL_MotorPulseToTotalThrust(obs->throttle_us), 0.0f));
        }
    }

    if (sysid.phase != APP_SYSID_PHASE_EXCITE) {
        /* 起升 / 稳定 / 回落：舵机回中，只按链路、解锁、IMU、杆轴与角度判停。 */
        uint32_t phase_ms = obs->now_ms - sysid.phase_start_ms;
        uint16_t target_us;

        abort_reason = sysid_gate(obs, axis_angle_rad, residual_rad_s, 0.0f, 0U);
        if (abort_reason != NULL) {
            sysid_stop_on(abort_reason);
            return;
        }
        sysid_center_servos();
        target_us = sysid_target_pulse();
        switch (sysid.phase) {
        case APP_SYSID_PHASE_RAMP_UP:
            sysid.motor_pulse_us = sysid_lerp_pulse(sysid.idle_pulse_us, target_us,
                                                    phase_ms, APP_SYSID_RAMP_UP_MS);
            if (phase_ms >= APP_SYSID_RAMP_UP_MS) {
                sysid_enter_phase(APP_SYSID_PHASE_SETTLE, obs->now_ms,
                    sysid_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(target_us), 0.0f));
            }
            return;
        case APP_SYSID_PHASE_SETTLE:
            sysid.motor_pulse_us = target_us;
            if ((phase_ms + APP_SYSID_PREROLL_MS) >= APP_SYSID_SETTLE_MS) {
                sysid_record_preroll(obs, axis_angle_rad,
                    sysid_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(target_us), 0.0f));
            }
            if (phase_ms < APP_SYSID_SETTLE_MS) {
                return;
            }
            thrust_n = sysid_finite_or(DRV_COAX_CTRL_MotorPulseToTotalThrust(target_us), 0.0f);
            abort_reason = sysid_gate(obs, axis_angle_rad, residual_rad_s, thrust_n, 1U);
            if (abort_reason != NULL) {
                sysid_finish(APP_SYSID_STATE_ABORTED, abort_reason);
                return;
            }
            sysid_begin_excite(obs, axis_angle_rad, thrust_n);
            break;  /* 本拍就开始激励（t=0） */
        case APP_SYSID_PHASE_RAMP_DOWN:
            sysid.motor_pulse_us = sysid_lerp_pulse(sysid.ramp_from_us, obs->throttle_us,
                                                    phase_ms, APP_SYSID_RAMP_DOWN_MS);
            if (phase_ms >= APP_SYSID_RAMP_DOWN_MS) {
                sysid_finish(APP_SYSID_STATE_DONE, "complete");
            }
            return;
        default:
            sysid_finish(APP_SYSID_STATE_ABORTED, "phase");
            return;
        }
    }

    elapsed_ms = obs->now_ms - sysid.start_ms;
    if (auto_throttle != 0U) {
        /* 每拍按当前电量电压重算，放电过程中合推力保持在目标上。 */
        sysid.motor_pulse_us = sysid_target_pulse();
    }
    thrust_n = DRV_COAX_CTRL_MotorPulseToTotalThrust(
        (auto_throttle != 0U) ? sysid.motor_pulse_us : obs->throttle_us);
    thrust_n = sysid_finite_or(thrust_n, 0.0f);

    abort_reason = sysid_gate(obs, axis_angle_rad, residual_rad_s, thrust_n, 1U);
    if (abort_reason != NULL) {
        sysid_finish(APP_SYSID_STATE_ABORTED, abort_reason);
        return;
    }

    if (DRV_SysIdExcitation_Eval(&sysid.spec, elapsed_ms, &excitation) !=
        DRV_SYSID_EXC_OK) {
        sysid_finish(APP_SYSID_STATE_ABORTED, "excitation");
        return;
    }

    abort_reason = sysid_moment_chain(obs, axis_angle_rad - sysid.excite_angle0_rad, dt_s,
                                      thrust_n, &excitation, &angle_sp,
                                      &body_x_rad, &body_y_rad, achieved);
    if (abort_reason != NULL) {
        sysid_finish(APP_SYSID_STATE_ABORTED, abort_reason);
        return;
    }

    /* 按采样率抽取。控制拍 500 Hz，线上默认 250 Hz。 */
    if (sysid_sample_due(obs->now_us, excitation.finished) != 0U) {
        uint8_t erpm_valid = sysid_sample_measurements(&sample, obs, axis_angle_rad);

        sample.omega_sp_rad_s = excitation.omega_sp_rad_s;
        sample.alpha_ff_rad_s2 = excitation.alpha_ff_rad_s2;
        sample.tilt_cmd_x_rad = body_x_rad;
        sample.tilt_cmd_y_rad = body_y_rad;
        sample.thrust_n = thrust_n;
        sample.torque_n_m = achieved[0]*cosf(sysid.rig.azimuth_rad) + achieved[1]*sinf(sysid.rig.azimuth_rad);
        /* 与 angle 同口径（绝对角），主机直接相减即跟踪误差。 */
        sample.angle_sp_rad = (sysid.mode == APP_SYSID_ANGLE) ?
            (sysid.excite_angle0_rad + angle_sp) : angle_sp;
        (void)sysid_rod_tilt_direction(tilt_dir);
        sample.servo_tilt_rad = sysid_servo_tilt_of(body_x_rad, body_y_rad, tilt_dir);
        if (sysid_push_if_running(&sample, obs->now_us, erpm_valid) == 0U) {
            return;   /* STOP 插进了本拍：这一轮已经结束，不再往尾批后面追加样本。 */
        }
    }

    if (excitation.finished != 0U) {
        sysid_complete(obs->now_ms, thrust_n, auto_throttle);
    }
}

/* ------------------------------------------------------------------ 发送面 */

static uint32_t sysid_take_batch(DRV_SysIdSample samples[SYSID_BATCH_MAX],
                                  DRV_SysIdBatchHeader *out_header)
{
    DRV_SysIdBatchHeader header;
    uint32_t available;
    uint32_t count = 0U;
    uint32_t tail;
    uint32_t nominal_us;
    uint8_t erpm_valid;
    uint8_t capped = 0U;

    APP_SysId_Init();

    available = sysid_ring_count();
    if (available == 0U) {
        return 0U;
    }

    /*
     * 没跑完就要求攒满一批再发。半批半批地发只会把包头开销翻几倍，而这条流的
     * 实时性要求本来就不高——时间戳在样本里，不在收包时刻上。
     * 跑完之后（last_batch_pending）才允许把尾巴发出去。
     */
    if ((available < SYSID_BATCH_MAX) && (sysid.last_batch_pending == 0U)) {
        return 0U;
    }

    nominal_us = (sysid.sample_period_us != 0U) ? sysid.sample_period_us : 1U;

    header.base_t_us = 0U;
    tail = sysid.tail;
    erpm_valid = 1U;
    while ((count < SYSID_BATCH_MAX) && (count < available)) {
        const SysIdRingEntry *entry = &sysid.ring[(tail + count) % APP_SYSID_RING_SAMPLES];

        if (count == 0U) {
            header.base_t_us = entry->t_us;
        } else if ((entry->gap_before != 0U) ||
                   (uint32_t)(entry->t_us - header.base_t_us) > 65535U) {
            /* 断点：本批到此为止，剩下的留给下一批，由它带 GAP。 */
            break;
        } else {
            /* 连续，继续攒。 */
        }
        samples[count] = entry->sample;
        samples[count].offset_us = (uint32_t)(entry->t_us - header.base_t_us);
        if (entry->erpm_valid == 0U) {
            erpm_valid = 0U;
        }
        capped |= entry->capped;
        count++;
    }

    if (count == 0U) {
        return 0U;
    }

    header.run_id = sysid.run_id;
    header.dt_us = (uint16_t)((nominal_us > 65535U) ? 65535U : nominal_us);
    /* 模式标志取开跑时锁存的那份：跑完、尾批还没排空时 MODE 照样能改，读 sysid.mode 会贴错。 */
    header.flags = sysid.run_flags;
    if (erpm_valid != 0U) { header.flags |= DRV_SYSID_FLAG_ERPM_VALID; }
    if (capped != 0U) { header.flags |= DRV_SYSID_FLAG_THRUST_CAPPED; }
    if (sysid.first_batch_pending != 0U) {
        header.flags |= DRV_SYSID_FLAG_FIRST_BATCH;
    }
    if (sysid.ring[tail % APP_SYSID_RING_SAMPLES].gap_before != 0U) {
        header.flags |= DRV_SYSID_FLAG_GAP;
    }
    if ((sysid.last_batch_pending != 0U) && (count == available)) {
        header.flags |= DRV_SYSID_FLAG_LAST_BATCH;
        if (sysid.state == APP_SYSID_STATE_ABORTED) {
            header.flags |= DRV_SYSID_FLAG_ABORTED;
        }
    }

    sysid.tail = tail + count;
    sysid.first_batch_pending = 0U;
    if ((header.flags & DRV_SYSID_FLAG_LAST_BATCH) != 0U) {
        sysid.last_batch_pending = 0U;
    }
    *out_header = header;
    return count;
}

uint8_t APP_SysId_PopBatch(uint8_t *out, size_t capacity, size_t *out_len)
{
    DRV_SysIdSample samples[SYSID_BATCH_MAX];
    DRV_SysIdBatchHeader header;
    if (!out || !out_len || capacity < DRV_SYSID_RECORD_MAX_FRAME) { return 0U; }
    /* Only copy/dequeue under IRQ masking. Float quantisation and schema hashing
     * must not delay the DShot TX/RX turnaround interrupt. */
    uint32_t lock = BSP_Critical_Enter();
    uint32_t count = sysid_take_batch(samples, &header);
    BSP_Critical_Exit(lock);
    if (!count) { return 0U; }
    if (DRV_SysIdRecord_Pack(&header, samples, count, out, capacity, out_len) != DRV_SYSID_RECORD_OK) {
        lock = BSP_Critical_Enter();
        sysid.dropped += count;
        BSP_Critical_Exit(lock);
        return 0U;
    }
    return 1U;
}

static const char *sysid_phase_name(APP_SysIdPhase phase)
{
    switch (phase) {
    case APP_SYSID_PHASE_RAMP_UP:   return "ramp_up";
    case APP_SYSID_PHASE_SETTLE:    return "settle";
    case APP_SYSID_PHASE_EXCITE:    return "excite";
    case APP_SYSID_PHASE_RAMP_DOWN: return "ramp_down";
    case APP_SYSID_PHASE_PREROLL:   return "preroll";
    case APP_SYSID_PHASE_DONE:      return "done";
    case APP_SYSID_PHASE_CLIMB:     return "climb";
    case APP_SYSID_PHASE_DESCEND:   return "descend";
    default:                        return "idle";
    }
}

void APP_SysId_StreamTick(void)
{
    uint32_t burst;

    APP_SysId_Init();

    if (sysid.phase_notice != 0U) {
        uint32_t lock = BSP_Critical_Enter();
        APP_SysIdPhase phase = sysid.phase_notice_phase;
        uint16_t pulse = sysid.phase_notice_pulse;
        float thrust = sysid.phase_notice_thrust;
        sysid.phase_notice = 0U;
        BSP_Critical_Exit(lock);
        APP_Control_QueueText("SYSID PHASE run=%u phase=%s pulse_us=%u thrust_cn=%ld\r\n",
                              (unsigned)sysid.run_id, sysid_phase_name(phase),
                              (unsigned)pulse, (long)(thrust * 100.0f));
    }

    for (burst = 0U; burst < SYSID_BATCHES_PER_TICK; burst++) {
        if (!pending_length && !APP_SysId_PopBatch(pending_payload, sizeof(pending_payload), &pending_length)) {
            break;
        }
        if (!APP_SysId_PortSend(pending_payload, (uint16_t)pending_length)) {
            break; /* Retry identical bytes, including FIRST/LAST, on the next service tick. */
        }
        pending_length = 0U;
    }
    if (sysid.notice_pending && !pending_length && !sysid_ring_count()) {
        if (sysid.saturation_detail) {
            /* Snapshot taken before centering; print only on the low-priority drain. */
            APP_Control_QueueText("SYSID SAT run=%u valid=%u force_mN=%ld req_x_uNm=%ld req_y_uNm=%ld got_x_uNm=%ld got_y_uNm=%ld alpha_us=%u beta_us=%u\r\n",
                (unsigned)sysid.run_id, (unsigned)(sysid.saturation_detail == 1U),
                (long)(sysid.saturation_force * 1000.0f),
                (long)(sysid.saturation_requested[0] * 1000000.0f),
                (long)(sysid.saturation_requested[1] * 1000000.0f),
                (long)(sysid.saturation_achieved[0] * 1000000.0f),
                (long)(sysid.saturation_achieved[1] * 1000000.0f),
                (unsigned)sysid.saturation_pulses[0], (unsigned)sysid.saturation_pulses[1]);
            sysid.saturation_detail = 0U;
        }
        if (sysid.bad_dt_us != 0U) {
            APP_Control_QueueText("SYSID DT run=%u dt_us=%lu\r\n",
                                  (unsigned)sysid.run_id, (unsigned long)sysid.bad_dt_us);
            sysid.bad_dt_us = 0U;
        }
        APP_Control_QueueText("SYSID end run=%u state=%s reason=%s dropped=%lu\r\n",
            (unsigned)sysid.run_id, sysid.state == APP_SYSID_STATE_DONE ? "done" : "aborted",
            APP_SysId_GetLastReason(), (unsigned long)sysid.dropped);
        sysid.notice_pending = 0U;
        sysid.last_batch_pending = 0U;
    }
}

void APP_SysId_ReportSchema(void)
{
    uint32_t count;
    uint32_t index;

    APP_SysId_Init();
    count = DRV_SysIdRecord_FieldCount();

    APP_Control_QueueText("SYSID SCHEMA ver=%u n=%lu hash=%08lX rec=%u\r\n",
                          (unsigned int)DRV_SYSID_RECORD_VERSION,
                          (unsigned long)count,
                          (unsigned long)DRV_SysIdRecord_SchemaHash(),
                          (unsigned int)DRV_SYSID_RECORD_BYTES);
    for (index = 0U; index < count; index++) {
        const char *name = DRV_SysIdRecord_FieldName(index);
        const char *unit = DRV_SysIdRecord_FieldUnit(index);

        /*
         * 类型报名字而不是编号。编号省几个字节，但一条 `type=1` 的日志谁也看不出
         * 是什么意思，而这几行正是出问题时第一个要读的东西。
         * 缩放是"定点计数 -> 物理量"的乘数，主机逐字段乘回去，布局不在主机侧写死。
         */
        /* newlib-nano is linked without float printf. Emit decimal scale via
         * integer formatting; otherwise the host receives an empty scale. */
        uint64_t scale_nano = (uint64_t)((double)DRV_SysIdRecord_FieldScale(index)
                                       * 1000000000.0 + 0.5);
        APP_Control_QueueText("SYSID FIELD idx=%lu name=%s unit=%s scale=%lu.%09lu type=%s\r\n",
                              (unsigned long)index,
                              (name != NULL) ? name : "?",
                              (unit != NULL) ? unit : "?",
                              (unsigned long)(scale_nano / 1000000000ULL),
                              (unsigned long)(scale_nano % 1000000000ULL),
                              (DRV_SysIdRecord_FieldType(index) ==
                               DRV_SYSID_FIELD_TYPE_U16) ? "u16" : "i16");
    }
}

void APP_SysId_ReportThrottle(void)
{
    float alt_h_m = 0.0f, alt_sp_m = 0.0f;
    uint8_t alt_h_ok = 0U;

    APP_SysId_Init();
    /* 行尾三项给高度辨识看：最近测高（每拍都更新）、是否有效、ALT 在跑时的高度参考（否则 0）。 */
    APP_SysIdAlt_GetLive(&alt_h_m, &alt_h_ok, &alt_sp_m);
    if (!((sysid.mode == APP_SYSID_ALT) && APP_SysId_IsRunning())) { alt_sp_m = 0.0f; }
    APP_Control_QueueText(
        "SYSID THR auto=%u target_cn=%ld max_pct_x10=%ld phase=%s pulse_us=%u "
        "armed=%u thr_low=%u capped=%u alt_h_mm=%ld alt_h_ok=%u alt_sp_mm=%ld\r\n",
        (unsigned int)(sysid.throttle_target_n > 0.0f),
        (long)(sysid.throttle_target_n * 100.0f + 0.5f),
        (long)(sysid.throttle_max_pct * 10.0f + 0.5f),
        sysid_phase_name(APP_SysId_GetPhase()),
        (unsigned int)((APP_SysId_GetPhase() != APP_SYSID_PHASE_IDLE) ? sysid.motor_pulse_us : 0U),
        (unsigned int)sysid.last_armed,
        (unsigned int)sysid.last_thr_low,
        (unsigned int)sysid.motor_capped,
        (long)lroundf(alt_h_m * 1000.0f), (unsigned int)alt_h_ok,
        (long)lroundf(alt_sp_m * 1000.0f));
}

void APP_SysId_ReportStatus(void)
{
    uint32_t total_ms = 0U;
    const char *state_name;

    APP_SysId_Init();
    (void)DRV_SysIdExcitation_TotalMs(&sysid.spec, &total_ms);
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();
    APP_Control_QueueText("SYSID READY ver=3 mode=%u angle_amp_mrad=%ld mass_mg=%ld imu_off_um=%ld thrust=%s engaged=%u\r\n",
        (unsigned)sysid.mode, (long)(sysid.angle_amplitude_rad*1000.0f),
        (long)(airframe->mass_kg*1000000.0f), (long)((airframe->imu_z_m-airframe->cg_z_m)*1000000.0f),
        DRV_COAX_CTRL_GetThrustMap() ? "lut" : "legacy", (unsigned)sysid.engaged);

    switch (sysid.state) {
    case APP_SYSID_STATE_RUNNING: state_name = "running"; break;
    case APP_SYSID_STATE_DONE:    state_name = "done";    break;
    case APP_SYSID_STATE_ABORTED: state_name = "aborted"; break;
    default:                      state_name = "idle";    break;
    }

    APP_Control_QueueText("SYSID state=%s run=%u reason=%s queued=%lu dropped=%lu\r\n",
                          state_name,
                          (unsigned int)sysid.run_id,
                          APP_SysId_GetLastReason(),
                          (unsigned long)sysid_ring_count(),
                          (unsigned long)sysid.dropped);
    APP_Control_QueueText("SYSID RIG psi_mrad=%ld axis_off_um=%ld imu_off_um=%ld\r\n",
                          (long)(sysid.rig.azimuth_rad * 1000.0f),
                          (long)(sysid.rig.axis_offset_above_cg_m * 1000000.0f),
                          (long)(sysid.rig.imu_above_cg_m * 1000000.0f));
    APP_Control_QueueText(
        "SYSID EXC profile=%u amp_mrad_s=%ld dur_ms=%lu hold_ms=%lu repeat=%lu "
        "ramp_ms=%lu f0_mhz=%ld f1_mhz=%ld bit_ms=%lu seed=%lu total_ms=%lu "
        "servo_tilt_mrad=%ld\r\n",
        (unsigned int)sysid.spec.profile,
        (long)(sysid.spec.amplitude_rad_s * 1000.0f),
        (unsigned long)sysid.spec.duration_ms,
        (unsigned long)sysid.spec.hold_ms,
        (unsigned long)sysid.spec.repeat,
        (unsigned long)sysid.spec.ramp_ms,
        (long)(sysid.spec.chirp_f0_hz * 1000.0f),
        (long)(sysid.spec.chirp_f1_hz * 1000.0f),
        (unsigned long)sysid.spec.prbs_bit_ms,
        (unsigned long)sysid.spec.prbs_seed,
        (unsigned long)total_ms,
        (long)(sysid.servo_tilt_rad * 1000.0f + 0.5f));
    /* 排在 LIMITS 之前：上位机把 LIMITS 当作一次状态报告的结束标记。 */
    APP_SysId_ReportThrottle();
    APP_Control_QueueText(
        "SYSID LIMITS rate_hz=%lu I_ugm2=%ld angle_mrad=%ld resid_mrad_s=%ld "
        "min_thrust_cn=%ld\r\n",
        (unsigned long)sysid.sample_rate_hz,
        (long)(sysid.inertia_kg_m2 * 1000000.0f),
        (long)(sysid.angle_limit_rad * 1000.0f),
        (long)(sysid.residual_limit_rad_s * 1000.0f),
        (long)(APP_SYSID_MIN_THRUST_N * 100.0f));
}
