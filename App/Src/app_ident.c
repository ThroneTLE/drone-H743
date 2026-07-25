#include "app_ident.h"

#include "app_control.h"
#include "drv_coax_ctrl.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define APP_IDENT_MAX_OFFSET_US       80
#define APP_IDENT_MAX_DURATION_MS     10000U
#define APP_IDENT_SAMPLE_PERIOD_MS    20U
#define APP_IDENT_DEFAULT_ALPHA_US    DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US
#define APP_IDENT_DEFAULT_BETA_US     DRV_COAX_CTRL_SERVO_BETA_CENTER_US
#define APP_IDENT_ATTITUDE_LIMIT_DEG  25.0f

#define APP_IDENT_ATT_MAX_AMP_MDEG        2000
#define APP_IDENT_ATT_MIN_BIT_MS          80U
#define APP_IDENT_ATT_MAX_BIT_MS          2000U
#define APP_IDENT_ATT_MAX_DURATION_MS     120000U
#define APP_IDENT_ATT_ATTITUDE_LIMIT_DEG  20.0f
#define APP_IDENT_ATT_GYRO_LIMIT_DPS      150.0f
#define APP_IDENT_ATT_PENDING_STABLE_MS   250U
#define APP_IDENT_ATT_PENDING_REPORT_MS   1000U
#define APP_IDENT_ATT_DEG_TO_RAD          0.01745329251994329577f
#define APP_IDENT_ATT_GRAVITY_M_S2        9.81f

typedef enum {
    APP_IDENT_AXIS_ROLL = 0,
    APP_IDENT_AXIS_PITCH
} APP_IdentAxis;

typedef enum {
    APP_IDENT_MODE_NONE = 0,
    APP_IDENT_MODE_STEP,
    APP_IDENT_MODE_DOUBLET,
    APP_IDENT_MODE_PRBS
} APP_IdentMode;

typedef struct {
    APP_IdentState state;
    APP_IdentAxis axis;
    APP_IdentMode mode;
    uint32_t id;
    uint32_t seq;
    uint32_t start_ms;
    uint32_t last_sample_ms;
    uint32_t duration_ms;
    uint32_t hold_ms;
    uint32_t bit_ms;
    uint32_t repeat;
    uint32_t seed;
    int32_t pulse_offset_us;
    int32_t active_offset_us;
    uint16_t alpha_center_us;
    uint16_t beta_center_us;
    uint16_t alpha_target_us;
    uint16_t beta_target_us;
    char last_reason[24];
} APP_IdentContext;

typedef struct {
    APP_IdentState state;
    APP_IdentAttAxis axis;
    APP_IdentAttMode mode;
    uint32_t id;
    uint32_t seq;
    uint32_t start_ms;
    uint32_t duration_ms;
    uint32_t bit_ms;
    uint32_t seed;
    uint32_t ready_start_ms;
    uint32_t last_wait_report_ms;
    int32_t amp_mdeg;
    APP_IdentAttLog log;
    char last_reason[24];
} APP_IdentAttContext;

static APP_IdentContext ident_ctx;
static APP_IdentAttContext ident_att_ctx;

static const char *ident_state_name(APP_IdentState state)
{
    switch (state) {
    case APP_IDENT_STATE_IDLE: return "idle";
    case APP_IDENT_STATE_ARMED: return "armed";
    case APP_IDENT_STATE_RUNNING: return "running";
    case APP_IDENT_STATE_DONE: return "done";
    case APP_IDENT_STATE_ABORTED: return "aborted";
    default: return "unknown";
    }
}

static const char *ident_axis_name(APP_IdentAxis axis)
{
    return (axis == APP_IDENT_AXIS_PITCH) ? "pitch" : "roll";
}

static const char *ident_mode_name(APP_IdentMode mode)
{
    switch (mode) {
    case APP_IDENT_MODE_STEP: return "step";
    case APP_IDENT_MODE_DOUBLET: return "doublet";
    case APP_IDENT_MODE_PRBS: return "prbs";
    default: return "none";
    }
}

static const char *ident_att_axis_name(APP_IdentAttAxis axis)
{
    switch (axis) {
    case APP_IDENT_ATT_AXIS_ROLL: return "roll";
    case APP_IDENT_ATT_AXIS_PITCH: return "pitch";
    default: return "none";
    }
}

static const char *ident_att_mode_name(APP_IdentAttMode mode)
{
    switch (mode) {
    case APP_IDENT_ATT_MODE_PRBS: return "prbs";
    default: return "none";
    }
}

static uint8_t ident_parse_axis(const char *text, APP_IdentAxis *axis)
{
    if ((text == 0) || (axis == 0)) {
        return 0U;
    }

    if (strcmp(text, "roll") == 0) {
        *axis = APP_IDENT_AXIS_ROLL;
        return 1U;
    }
    if (strcmp(text, "pitch") == 0) {
        *axis = APP_IDENT_AXIS_PITCH;
        return 1U;
    }
    return 0U;
}

static uint8_t ident_att_parse_axis(const char *text, APP_IdentAttAxis *axis)
{
    if ((text == 0) || (axis == 0)) {
        return 0U;
    }

    if (strcmp(text, "roll") == 0) {
        *axis = APP_IDENT_ATT_AXIS_ROLL;
        return 1U;
    }
    if (strcmp(text, "pitch") == 0) {
        *axis = APP_IDENT_ATT_AXIS_PITCH;
        return 1U;
    }
    return 0U;
}

static int32_t ident_abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t ident_valid_offset(int32_t pulse_us)
{
    return (ident_abs_i32(pulse_us) <= APP_IDENT_MAX_OFFSET_US) ? 1U : 0U;
}

static uint8_t ident_valid_duration(uint32_t duration_ms)
{
    return ((duration_ms > 0U) && (duration_ms <= APP_IDENT_MAX_DURATION_MS)) ? 1U : 0U;
}

static uint8_t ident_targets_from_offset(int32_t offset,
                                         uint16_t *alpha_us,
                                         uint16_t *beta_us)
{
    int32_t alpha = (int32_t)ident_ctx.alpha_center_us;
    int32_t beta = (int32_t)ident_ctx.beta_center_us;

    /* Current tilt module mounting: servo 1/alpha is roll, servo 2/beta is pitch. */
    if (ident_ctx.axis == APP_IDENT_AXIS_ROLL) {
        alpha += offset;
    } else {
        beta += offset;
    }

    if ((alpha < (int32_t)DRV_COAX_CTRL_SERVO_ALPHA_MIN_US) ||
        (alpha > (int32_t)DRV_COAX_CTRL_SERVO_ALPHA_MAX_US) ||
        (beta < (int32_t)DRV_COAX_CTRL_SERVO_BETA_MIN_US) ||
        (beta > (int32_t)DRV_COAX_CTRL_SERVO_BETA_MAX_US)) {
        return 0U;
    }

    if (alpha_us != 0) {
        *alpha_us = (uint16_t)alpha;
    }
    if (beta_us != 0) {
        *beta_us = (uint16_t)beta;
    }
    return 1U;
}

static void ident_set_reason(const char *reason)
{
    if (reason == 0) {
        reason = "none";
    }
    (void)snprintf(ident_ctx.last_reason, sizeof(ident_ctx.last_reason), "%s", reason);
}

static void ident_att_set_reason(const char *reason)
{
    if (reason == 0) {
        reason = "none";
    }
    (void)snprintf(ident_att_ctx.last_reason,
                   sizeof(ident_att_ctx.last_reason),
                   "%s",
                   reason);
}

static void ident_set_center_targets(void)
{
    ident_ctx.active_offset_us = 0;
    ident_ctx.alpha_target_us = ident_ctx.alpha_center_us;
    ident_ctx.beta_target_us = ident_ctx.beta_center_us;
}

static void ident_att_clear_log(void)
{
    memset(&ident_att_ctx.log, 0, sizeof(ident_att_ctx.log));
    ident_att_ctx.log.axis = (uint8_t)ident_att_ctx.axis;
    ident_att_ctx.log.mode = (uint8_t)ident_att_ctx.mode;
}

static uint8_t ident_att_pending_or_running(void)
{
    return ((ident_att_ctx.state == APP_IDENT_STATE_ARMED) ||
            (ident_att_ctx.state == APP_IDENT_STATE_RUNNING)) ? 1U : 0U;
}

static void ident_att_finish(const char *reason, uint8_t emit)
{
    ident_att_set_reason(reason);
    ident_att_clear_log();
    ident_att_ctx.ready_start_ms = 0U;
    ident_att_ctx.last_wait_report_ms = 0U;
    if ((ident_att_ctx.state == APP_IDENT_STATE_RUNNING) ||
        (ident_att_ctx.state == APP_IDENT_STATE_ARMED)) {
        ident_att_ctx.state =
            (strcmp(ident_att_ctx.last_reason, "complete") == 0) ?
            APP_IDENT_STATE_DONE : APP_IDENT_STATE_ABORTED;
        if (emit != 0U) {
            APP_Control_QueueText("IDENT att stop id=%lu reason=%s seq=%lu\r\n",
                                  (unsigned long)ident_att_ctx.id,
                                  ident_att_ctx.last_reason,
                                  (unsigned long)ident_att_ctx.seq);
        }
    } else if (emit != 0U) {
        APP_Control_QueueText("IDENT att stop reason=%s id=%lu\r\n",
                              ident_att_ctx.last_reason,
                              (unsigned long)ident_att_ctx.id);
    }
}

static const char *ident_att_ready_reason(const APP_IdentAttObserve *obs)
{
    if (obs->rc_link_ok == 0U) {
        return "wait_rc_link";
    }
    if (obs->rc_armed == 0U) {
        return "wait_rc_arm";
    }
    if (obs->imu_valid == 0U) {
        return "wait_imu";
    }
    if (obs->throttle_over_20 == 0U) {
        return "wait_throttle";
    }
    if ((obs->roll_deg > APP_IDENT_ATT_ATTITUDE_LIMIT_DEG) ||
        (obs->roll_deg < -APP_IDENT_ATT_ATTITUDE_LIMIT_DEG) ||
        (obs->pitch_deg > APP_IDENT_ATT_ATTITUDE_LIMIT_DEG) ||
        (obs->pitch_deg < -APP_IDENT_ATT_ATTITUDE_LIMIT_DEG)) {
        return "wait_attitude";
    }
    if ((obs->gyro_x_dps > APP_IDENT_ATT_GYRO_LIMIT_DPS) ||
        (obs->gyro_x_dps < -APP_IDENT_ATT_GYRO_LIMIT_DPS) ||
        (obs->gyro_y_dps > APP_IDENT_ATT_GYRO_LIMIT_DPS) ||
        (obs->gyro_y_dps < -APP_IDENT_ATT_GYRO_LIMIT_DPS)) {
        return "wait_gyro";
    }
    if (obs->control_valid == 0U) {
        return "wait_control";
    }
    return "ready";
}

static void ident_att_report_wait(const APP_IdentAttObserve *obs,
                                  const char *reason)
{
    uint32_t stable_ms = 0U;

    if ((obs == 0) || (ident_att_ctx.state != APP_IDENT_STATE_ARMED)) {
        return;
    }

    if (ident_att_ctx.ready_start_ms != 0U) {
        stable_ms = obs->now_ms - ident_att_ctx.ready_start_ms;
    }
    if ((ident_att_ctx.last_wait_report_ms != 0U) &&
        ((obs->now_ms - ident_att_ctx.last_wait_report_ms) <
         APP_IDENT_ATT_PENDING_REPORT_MS)) {
        return;
    }

    ident_att_ctx.last_wait_report_ms = obs->now_ms;
    APP_Control_QueueText("IDENT att wait id=%lu reason=%s stable_ms=%lu\r\n",
                          (unsigned long)ident_att_ctx.id,
                          reason,
                          (unsigned long)stable_ms);
}

static void ident_att_try_begin(const APP_IdentAttObserve *obs)
{
    const char *reason;

    if ((obs == 0) || (ident_att_ctx.state != APP_IDENT_STATE_ARMED)) {
        return;
    }

    reason = ident_att_ready_reason(obs);
    if (strcmp(reason, "ready") != 0) {
        ident_att_ctx.ready_start_ms = 0U;
        ident_att_set_reason(reason);
        ident_att_report_wait(obs, reason);
        return;
    }

    if (ident_att_ctx.ready_start_ms == 0U) {
        ident_att_ctx.ready_start_ms = obs->now_ms;
        ident_att_set_reason("wait_stable");
        ident_att_report_wait(obs, "wait_stable");
        return;
    }
    if ((obs->now_ms - ident_att_ctx.ready_start_ms) <
        APP_IDENT_ATT_PENDING_STABLE_MS) {
        ident_att_set_reason("wait_stable");
        ident_att_report_wait(obs, "wait_stable");
        return;
    }

    ident_att_ctx.start_ms = obs->now_ms;
    ident_att_ctx.seq = 0U;
    ident_att_ctx.ready_start_ms = 0U;
    ident_att_ctx.last_wait_report_ms = 0U;
    ident_att_ctx.state = APP_IDENT_STATE_RUNNING;
    ident_att_set_reason("running");
    ident_att_clear_log();
    APP_Control_QueueText("IDENT att start id=%lu axis=%s mode=prbs amp_mdeg=%ld bit_ms=%lu duration_ms=%lu seed=%lu\r\n",
                          (unsigned long)ident_att_ctx.id,
                          ident_att_axis_name(ident_att_ctx.axis),
                          (long)ident_att_ctx.amp_mdeg,
                          (unsigned long)ident_att_ctx.bit_ms,
                          (unsigned long)ident_att_ctx.duration_ms,
                          (unsigned long)ident_att_ctx.seed);
}

static uint32_t ident_prbs_next(uint32_t value)
{
    uint32_t bit = ((value >> 0U) ^ (value >> 2U) ^ (value >> 3U) ^ (value >> 5U)) & 1U;
    return (value >> 1U) | (bit << 31U);
}

static int32_t ident_scaled_i32(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    return (int32_t)scaled;
}

static void ident_format_mdeg(char *buffer, size_t size, float value)
{
    int32_t scaled = ident_scaled_i32(value, 1000.0f);
    uint32_t abs_scaled = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;

    (void)snprintf(buffer, size, "%s%lu.%03lu",
                   (scaled < 0) ? "-" : "",
                   (unsigned long)(abs_scaled / 1000U),
                   (unsigned long)(abs_scaled % 1000U));
}

static void ident_format_cdps(char *buffer, size_t size, float value)
{
    int32_t scaled = ident_scaled_i32(value, 100.0f);
    uint32_t abs_scaled = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;

    (void)snprintf(buffer, size, "%s%lu.%02lu",
                   (scaled < 0) ? "-" : "",
                   (unsigned long)(abs_scaled / 100U),
                   (unsigned long)(abs_scaled % 100U));
}

static void ident_finish(const char *reason)
{
    ident_set_center_targets();
    ident_set_reason(reason);
    ident_ctx.state = APP_IDENT_STATE_DONE;
    APP_Control_QueueText("IDENT done id=%lu samples=%lu reason=%s\r\n",
                          (unsigned long)ident_ctx.id,
                          (unsigned long)ident_ctx.seq,
                          ident_ctx.last_reason);
}

static uint8_t ident_start(APP_IdentAxis axis,
                           APP_IdentMode mode,
                           int32_t pulse_us,
                           uint32_t duration_ms,
                           uint32_t hold_ms,
                           uint32_t repeat,
                           uint32_t bit_ms,
                           uint32_t seed)
{
    uint16_t alpha;
    uint16_t beta;

    if (ident_ctx.state != APP_IDENT_STATE_ARMED) {
        APP_Control_QueueText("ERR ident not armed\r\n");
        return 0U;
    }
    if (ident_att_pending_or_running() != 0U) {
        APP_Control_QueueText("ERR ident att running\r\n");
        return 0U;
    }
    if ((ident_valid_offset(pulse_us) == 0U) ||
        (ident_valid_duration(duration_ms) == 0U) ||
        (ident_targets_from_offset(pulse_us, &alpha, &beta) == 0U)) {
        APP_Control_QueueText("ERR ident range pulse_us=%ld duration_ms=%lu\r\n",
                              (long)pulse_us,
                              (unsigned long)duration_ms);
        return 0U;
    }
    if ((mode == APP_IDENT_MODE_DOUBLET) && ((hold_ms == 0U) || (repeat == 0U))) {
        APP_Control_QueueText("ERR ident doublet args\r\n");
        return 0U;
    }
    if ((mode == APP_IDENT_MODE_PRBS) && (bit_ms == 0U)) {
        APP_Control_QueueText("ERR ident prbs args\r\n");
        return 0U;
    }

    ident_ctx.axis = axis;
    ident_ctx.mode = mode;
    ident_ctx.pulse_offset_us = pulse_us;
    ident_ctx.duration_ms = duration_ms;
    ident_ctx.hold_ms = hold_ms;
    ident_ctx.repeat = repeat;
    ident_ctx.bit_ms = bit_ms;
    ident_ctx.seed = (seed == 0U) ? 1U : seed;
    ident_ctx.seq = 0U;
    ++ident_ctx.id;
    ident_ctx.start_ms = 0U;
    ident_ctx.last_sample_ms = 0U;
    ident_ctx.alpha_target_us = alpha;
    ident_ctx.beta_target_us = beta;
    ident_ctx.state = APP_IDENT_STATE_RUNNING;
    ident_set_reason("running");

    APP_Control_QueueText("IDENT start id=%lu axis=%s mode=%s pulse_us=%ld duration_ms=%lu\r\n",
                          (unsigned long)ident_ctx.id,
                          ident_axis_name(ident_ctx.axis),
                          ident_mode_name(ident_ctx.mode),
                          (long)ident_ctx.pulse_offset_us,
                          (unsigned long)ident_ctx.duration_ms);
    return 1U;
}

void APP_Ident_Init(void)
{
    memset(&ident_ctx, 0, sizeof(ident_ctx));
    ident_ctx.state = APP_IDENT_STATE_IDLE;
    ident_ctx.alpha_center_us = APP_IDENT_DEFAULT_ALPHA_US;
    ident_ctx.beta_center_us = APP_IDENT_DEFAULT_BETA_US;
    ident_set_center_targets();
    ident_set_reason("init");

    memset(&ident_att_ctx, 0, sizeof(ident_att_ctx));
    ident_att_ctx.state = APP_IDENT_STATE_IDLE;
    ident_att_ctx.axis = APP_IDENT_ATT_AXIS_NONE;
    ident_att_ctx.mode = APP_IDENT_ATT_MODE_NONE;
    ident_att_set_reason("init");
    ident_att_clear_log();
}

APP_IdentState APP_Ident_GetState(void)
{
    return ident_ctx.state;
}

uint8_t APP_Ident_IsRunning(void)
{
    return (ident_ctx.state == APP_IDENT_STATE_RUNNING) ? 1U : 0U;
}

void APP_Ident_ReportStatus(void)
{
    APP_Control_QueueText("IDENT state=%s id=%lu seq=%lu axis=%s mode=%s alpha_us=%u beta_us=%u center_alpha_us=%u center_beta_us=%u reason=%s\r\n",
                          ident_state_name(ident_ctx.state),
                          (unsigned long)ident_ctx.id,
                          (unsigned long)ident_ctx.seq,
                          ident_axis_name(ident_ctx.axis),
                          ident_mode_name(ident_ctx.mode),
                          (unsigned int)ident_ctx.alpha_target_us,
                          (unsigned int)ident_ctx.beta_target_us,
                          (unsigned int)ident_ctx.alpha_center_us,
                          (unsigned int)ident_ctx.beta_center_us,
                          ident_ctx.last_reason);
    APP_Control_QueueText("IDENT att state=%s id=%lu seq=%lu axis=%s mode=%s amp_mdeg=%ld bit_ms=%lu duration_ms=%lu elapsed_ms=%lu reason=%s\r\n",
                          ident_state_name(ident_att_ctx.state),
                          (unsigned long)ident_att_ctx.id,
                          (unsigned long)ident_att_ctx.seq,
                          ident_att_axis_name(ident_att_ctx.axis),
                          ident_att_mode_name(ident_att_ctx.mode),
                          (long)ident_att_ctx.amp_mdeg,
                          (unsigned long)ident_att_ctx.bit_ms,
                          (unsigned long)ident_att_ctx.duration_ms,
                          (unsigned long)ident_att_ctx.log.elapsed_ms,
                          ident_att_ctx.last_reason);
}

uint8_t APP_Ident_Arm(void)
{
    if (ident_ctx.state == APP_IDENT_STATE_RUNNING) {
        APP_Control_QueueText("ERR ident running\r\n");
        return 0U;
    }
    if (ident_att_pending_or_running() != 0U) {
        APP_Control_QueueText("ERR ident att running\r\n");
        return 0U;
    }
    ident_set_center_targets();
    ident_ctx.state = APP_IDENT_STATE_ARMED;
    ident_set_reason("armed");
    APP_Control_QueueText("OK ident armed\r\n");
    return 1U;
}

void APP_Ident_Disarm(void)
{
    ident_att_finish("disarm", 0U);
    ident_set_center_targets();
    ident_ctx.state = APP_IDENT_STATE_IDLE;
    ident_set_reason("disarm");
    APP_Control_QueueText("OK ident disarmed\r\n");
}

void APP_Ident_Stop(const char *reason)
{
    ident_att_finish((reason == 0) ? "command" : reason, 1U);
    ident_set_center_targets();
    ident_set_reason((reason == 0) ? "command" : reason);
    if (ident_ctx.state == APP_IDENT_STATE_RUNNING) {
        ident_ctx.state = APP_IDENT_STATE_ABORTED;
        APP_Control_QueueText("IDENT abort id=%lu reason=%s\r\n",
                              (unsigned long)ident_ctx.id,
                              ident_ctx.last_reason);
    } else {
        ident_ctx.state = APP_IDENT_STATE_IDLE;
        APP_Control_QueueText("IDENT stop reason=%s id=%lu ms=%lu\r\n",
                              ident_ctx.last_reason,
                              (unsigned long)ident_ctx.id,
                              (unsigned long)0U);
    }
}

uint8_t APP_Ident_SetCenter(uint16_t alpha_us, uint16_t beta_us)
{
    if (ident_ctx.state == APP_IDENT_STATE_RUNNING) {
        APP_Control_QueueText("ERR ident running\r\n");
        return 0U;
    }
    if ((alpha_us < DRV_COAX_CTRL_SERVO_ALPHA_MIN_US) ||
        (alpha_us > DRV_COAX_CTRL_SERVO_ALPHA_MAX_US) ||
        (beta_us < DRV_COAX_CTRL_SERVO_BETA_MIN_US) ||
        (beta_us > DRV_COAX_CTRL_SERVO_BETA_MAX_US)) {
        APP_Control_QueueText("ERR ident center range\r\n");
        return 0U;
    }
    ident_ctx.alpha_center_us = alpha_us;
    ident_ctx.beta_center_us = beta_us;
    ident_set_center_targets();
    APP_Control_QueueText("OK ident center alpha_us=%u beta_us=%u\r\n",
                          (unsigned int)alpha_us,
                          (unsigned int)beta_us);
    return 1U;
}

uint8_t APP_Ident_StartStep(const char *axis, int32_t pulse_us, uint32_t duration_ms)
{
    APP_IdentAxis parsed_axis;

    if (ident_parse_axis(axis, &parsed_axis) == 0U) {
        APP_Control_QueueText("ERR ident axis %s\r\n", (axis == 0) ? "" : axis);
        return 0U;
    }
    return ident_start(parsed_axis, APP_IDENT_MODE_STEP, pulse_us, duration_ms,
                       0U, 0U, 0U, 1U);
}

uint8_t APP_Ident_StartDoublet(const char *axis, int32_t pulse_us, uint32_t hold_ms, uint32_t repeat)
{
    APP_IdentAxis parsed_axis;
    uint32_t duration_ms;

    if (ident_parse_axis(axis, &parsed_axis) == 0U) {
        APP_Control_QueueText("ERR ident axis %s\r\n", (axis == 0) ? "" : axis);
        return 0U;
    }
    if ((hold_ms == 0U) || (repeat == 0U) || (repeat > 10U)) {
        APP_Control_QueueText("ERR ident doublet args\r\n");
        return 0U;
    }
    duration_ms = hold_ms * repeat * 2U;
    if (duration_ms > APP_IDENT_MAX_DURATION_MS) {
        duration_ms = APP_IDENT_MAX_DURATION_MS;
    }
    return ident_start(parsed_axis, APP_IDENT_MODE_DOUBLET, pulse_us, duration_ms,
                       hold_ms, repeat, 0U, 1U);
}

uint8_t APP_Ident_StartPrbs(const char *axis,
                            int32_t pulse_us,
                            uint32_t bit_ms,
                            uint32_t duration_ms,
                            uint32_t seed)
{
    APP_IdentAxis parsed_axis;

    if (ident_parse_axis(axis, &parsed_axis) == 0U) {
        APP_Control_QueueText("ERR ident axis %s\r\n", (axis == 0) ? "" : axis);
        return 0U;
    }
    return ident_start(parsed_axis, APP_IDENT_MODE_PRBS, pulse_us, duration_ms,
                       0U, 0U, bit_ms, seed);
}

uint8_t APP_IdentAtt_StartPrbs(const char *axis,
                               int32_t amp_mdeg,
                               uint32_t bit_ms,
                               uint32_t duration_ms,
                               uint32_t seed)
{
    APP_IdentAttAxis parsed_axis;

    if (ident_ctx.state == APP_IDENT_STATE_RUNNING) {
        APP_Control_QueueText("ERR ident direct running\r\n");
        return 0U;
    }
    if (ident_ctx.state != APP_IDENT_STATE_ARMED) {
        APP_Control_QueueText("ERR ident not armed\r\n");
        return 0U;
    }
    if (ident_att_pending_or_running() != 0U) {
        APP_Control_QueueText("ERR ident att running\r\n");
        return 0U;
    }
    if (ident_att_parse_axis(axis, &parsed_axis) == 0U) {
        APP_Control_QueueText("ERR ident att axis %s\r\n",
                              (axis == 0) ? "" : axis);
        return 0U;
    }
    if ((ident_abs_i32(amp_mdeg) == 0) ||
        (ident_abs_i32(amp_mdeg) > APP_IDENT_ATT_MAX_AMP_MDEG) ||
        (bit_ms < APP_IDENT_ATT_MIN_BIT_MS) ||
        (bit_ms > APP_IDENT_ATT_MAX_BIT_MS) ||
        (duration_ms == 0U) ||
        (duration_ms > APP_IDENT_ATT_MAX_DURATION_MS)) {
        APP_Control_QueueText("ERR ident att range amp_mdeg=%ld bit_ms=%lu duration_ms=%lu\r\n",
                              (long)amp_mdeg,
                              (unsigned long)bit_ms,
                              (unsigned long)duration_ms);
        return 0U;
    }

    ident_att_ctx.axis = parsed_axis;
    ident_att_ctx.mode = APP_IDENT_ATT_MODE_PRBS;
    ident_att_ctx.amp_mdeg = amp_mdeg;
    ident_att_ctx.bit_ms = bit_ms;
    ident_att_ctx.duration_ms = duration_ms;
    ident_att_ctx.seed = (seed == 0U) ? 1U : seed;
    ident_att_ctx.seq = 0U;
    ident_att_ctx.start_ms = 0U;
    ident_att_ctx.ready_start_ms = 0U;
    ident_att_ctx.last_wait_report_ms = 0U;
    ++ident_att_ctx.id;
    ident_att_ctx.state = APP_IDENT_STATE_ARMED;
    ident_att_set_reason("wait_rc_arm");
    ident_att_clear_log();

    APP_Control_QueueText("IDENT att armed id=%lu axis=%s mode=prbs amp_mdeg=%ld bit_ms=%lu duration_ms=%lu seed=%lu wait=safe_start\r\n",
                          (unsigned long)ident_att_ctx.id,
                          ident_att_axis_name(ident_att_ctx.axis),
                          (long)ident_att_ctx.amp_mdeg,
                          (unsigned long)ident_att_ctx.bit_ms,
                          (unsigned long)ident_att_ctx.duration_ms,
                          (unsigned long)ident_att_ctx.seed);
    return 1U;
}

void APP_IdentAtt_Stop(const char *reason)
{
    ident_att_finish((reason == 0) ? "command" : reason, 1U);
}

uint8_t APP_Ident_ApplyPid(const char *axis, const char *kp_text, const char *kd_text)
{
    const char *kd_name;
    char *endptr;

    if (strcmp(axis, "roll") == 0) {
        kd_name = "coax.roll_rate_kd";
    } else if (strcmp(axis, "pitch") == 0) {
        kd_name = "coax.pitch_rate_kd";
    } else {
        APP_Control_QueueText("ERR ident apply axis %s\r\n", (axis == 0) ? "" : axis);
        return 0U;
    }

    if (kp_text != 0) {
        APP_Control_QueueText("ERR ident apply kp unused\r\n");
        return 0U;
    }
    if (kd_text != 0) {
        float value = strtof(kd_text, &endptr);
        if ((endptr == kd_text) || (*endptr != '\0') || (DRV_COAX_CTRL_SetParam(kd_name, value) == 0U)) {
            APP_Control_QueueText("ERR ident apply kd\r\n");
            return 0U;
        }
    }
    APP_Control_QueueText("OK ident apply axis=%s\r\n", axis);
    return 1U;
}

void APP_IdentAtt_Update(uint32_t now_ms)
{
    uint32_t elapsed;
    uint32_t bit_index;
    uint32_t lfsr;
    float signal_rad;
    float accel_m_s2;
    float sign = 1.0f;

    if (ident_att_ctx.state != APP_IDENT_STATE_RUNNING) {
        ident_att_clear_log();
        return;
    }
    if (ident_att_ctx.start_ms == 0U) {
        ident_att_ctx.start_ms = now_ms;
    }

    elapsed = now_ms - ident_att_ctx.start_ms;
    if (elapsed >= ident_att_ctx.duration_ms) {
        ident_att_finish("complete", 1U);
        return;
    }

    bit_index = (ident_att_ctx.bit_ms == 0U) ? 0U :
                (elapsed / ident_att_ctx.bit_ms);
    lfsr = (ident_att_ctx.seed == 0U) ? 1U : ident_att_ctx.seed;
    for (uint32_t index = 0U; index <= bit_index; ++index) {
        lfsr = ident_prbs_next(lfsr);
    }
    if ((lfsr & 1U) == 0U) {
        sign = -1.0f;
    }

    signal_rad = sign * ((float)ident_att_ctx.amp_mdeg * 0.001f) *
                 APP_IDENT_ATT_DEG_TO_RAD;
    accel_m_s2 = APP_IDENT_ATT_GRAVITY_M_S2 * tanf(signal_rad);
    ident_att_ctx.seq = bit_index;
    ident_att_ctx.log.active = 1U;
    ident_att_ctx.log.axis = (uint8_t)ident_att_ctx.axis;
    ident_att_ctx.log.mode = (uint8_t)ident_att_ctx.mode;
    ident_att_ctx.log.seq = bit_index;
    ident_att_ctx.log.signal_rad = signal_rad;
    ident_att_ctx.log.signal_m_s2 =
        (ident_att_ctx.axis == APP_IDENT_ATT_AXIS_ROLL) ?
        -accel_m_s2 : accel_m_s2;
    ident_att_ctx.log.elapsed_ms = elapsed;
}

void APP_IdentAtt_Apply(float *ax_m_s2,
                        float *ay_m_s2,
                        APP_IdentAttLog *log)
{
    if (log != 0) {
        APP_IdentAtt_GetLog(log);
    }
    if ((ident_att_ctx.state != APP_IDENT_STATE_RUNNING) ||
        (ident_att_ctx.log.active == 0U)) {
        return;
    }
    if ((ident_att_ctx.axis == APP_IDENT_ATT_AXIS_PITCH) &&
        (ax_m_s2 != 0)) {
        *ax_m_s2 += ident_att_ctx.log.signal_m_s2;
    } else if ((ident_att_ctx.axis == APP_IDENT_ATT_AXIS_ROLL) &&
               (ay_m_s2 != 0)) {
        *ay_m_s2 += ident_att_ctx.log.signal_m_s2;
    }
    if (log != 0) {
        APP_IdentAtt_GetLog(log);
    }
}

void APP_IdentAtt_GetLog(APP_IdentAttLog *log)
{
    if (log == 0) {
        return;
    }
    *log = ident_att_ctx.log;
}

void APP_IdentAtt_Observe(const APP_IdentAttObserve *obs)
{
    if (obs == 0) {
        return;
    }
    if (ident_att_ctx.state == APP_IDENT_STATE_ARMED) {
        ident_att_try_begin(obs);
        return;
    }
    if (ident_att_ctx.state != APP_IDENT_STATE_RUNNING) {
        return;
    }
    if (obs->rc_link_ok == 0U) {
        APP_IdentAtt_Stop("rc_lost");
        return;
    }
    if (obs->rc_armed == 0U) {
        APP_IdentAtt_Stop("rc_disarm");
        return;
    }
    if (obs->imu_valid == 0U) {
        APP_IdentAtt_Stop("imu_stale");
        return;
    }
    if (obs->throttle_over_20 == 0U) {
        APP_IdentAtt_Stop("throttle_low");
        return;
    }
    if ((obs->roll_deg > APP_IDENT_ATT_ATTITUDE_LIMIT_DEG) ||
        (obs->roll_deg < -APP_IDENT_ATT_ATTITUDE_LIMIT_DEG) ||
        (obs->pitch_deg > APP_IDENT_ATT_ATTITUDE_LIMIT_DEG) ||
        (obs->pitch_deg < -APP_IDENT_ATT_ATTITUDE_LIMIT_DEG)) {
        APP_IdentAtt_Stop("attitude_limit");
        return;
    }
    if ((obs->gyro_x_dps > APP_IDENT_ATT_GYRO_LIMIT_DPS) ||
        (obs->gyro_x_dps < -APP_IDENT_ATT_GYRO_LIMIT_DPS) ||
        (obs->gyro_y_dps > APP_IDENT_ATT_GYRO_LIMIT_DPS) ||
        (obs->gyro_y_dps < -APP_IDENT_ATT_GYRO_LIMIT_DPS)) {
        APP_IdentAtt_Stop("gyro_limit");
        return;
    }
    if (obs->control_valid == 0U) {
        APP_IdentAtt_Stop("control_invalid");
        return;
    }
}

void APP_Ident_Update(uint32_t now_ms)
{
    uint32_t elapsed;
    int32_t offset = 0;
    uint16_t alpha;
    uint16_t beta;

    if (ident_ctx.state != APP_IDENT_STATE_RUNNING) {
        return;
    }

    if (ident_ctx.start_ms == 0U) {
        ident_ctx.start_ms = now_ms;
    }
    elapsed = now_ms - ident_ctx.start_ms;
    if (elapsed >= ident_ctx.duration_ms) {
        ident_finish("complete");
        return;
    }

    switch (ident_ctx.mode) {
    case APP_IDENT_MODE_STEP:
        offset = ident_ctx.pulse_offset_us;
        break;
    case APP_IDENT_MODE_DOUBLET:
        if (ident_ctx.hold_ms != 0U) {
            uint32_t segment = elapsed / ident_ctx.hold_ms;
            offset = ((segment & 1U) == 0U) ? ident_ctx.pulse_offset_us : -ident_ctx.pulse_offset_us;
        }
        break;
    case APP_IDENT_MODE_PRBS:
        if (ident_ctx.bit_ms != 0U) {
            uint32_t bit_index = elapsed / ident_ctx.bit_ms;
            uint32_t lfsr = ident_ctx.seed;
            for (uint32_t index = 0U; index <= bit_index; ++index) {
                lfsr = ident_prbs_next(lfsr);
            }
            offset = ((lfsr & 1U) != 0U) ? ident_ctx.pulse_offset_us : -ident_ctx.pulse_offset_us;
        }
        break;
    default:
        offset = 0;
        break;
    }

    if (ident_targets_from_offset(offset, &alpha, &beta) == 0U) {
        APP_Ident_Stop("servo_limit");
        return;
    }
    ident_ctx.active_offset_us = offset;
    ident_ctx.alpha_target_us = alpha;
    ident_ctx.beta_target_us = beta;
}

void APP_Ident_GetServoTargets(uint16_t *alpha_us, uint16_t *beta_us)
{
    if (alpha_us != 0) {
        *alpha_us = ident_ctx.alpha_target_us;
    }
    if (beta_us != 0) {
        *beta_us = ident_ctx.beta_target_us;
    }
}

void APP_Ident_Observe(const APP_IdentObserve *obs)
{
    uint32_t t_ms;
    char roll_text[16];
    char pitch_text[16];
    char gx_text[16];
    char gy_text[16];

    if ((obs == 0) || (ident_ctx.state != APP_IDENT_STATE_RUNNING)) {
        return;
    }

    if (obs->rc_link_ok == 0U) {
        APP_Ident_Stop("rc_lost");
        return;
    }
    if (obs->rc_armed == 0U) {
        APP_Ident_Stop("rc_disarm");
        return;
    }
    if (obs->imu_valid == 0U) {
        APP_Ident_Stop("imu_stale");
        return;
    }
    if ((obs->roll_deg > APP_IDENT_ATTITUDE_LIMIT_DEG) ||
        (obs->roll_deg < -APP_IDENT_ATTITUDE_LIMIT_DEG) ||
        (obs->pitch_deg > APP_IDENT_ATTITUDE_LIMIT_DEG) ||
        (obs->pitch_deg < -APP_IDENT_ATTITUDE_LIMIT_DEG)) {
        APP_Ident_Stop("attitude_limit");
        return;
    }

    t_ms = obs->now_ms - ident_ctx.start_ms;
    if ((ident_ctx.last_sample_ms != 0U) &&
        ((obs->now_ms - ident_ctx.last_sample_ms) < APP_IDENT_SAMPLE_PERIOD_MS)) {
        return;
    }
    ident_ctx.last_sample_ms = obs->now_ms;

    ident_format_mdeg(roll_text, sizeof(roll_text), obs->roll_deg);
    ident_format_mdeg(pitch_text, sizeof(pitch_text), obs->pitch_deg);
    ident_format_cdps(gx_text, sizeof(gx_text), obs->gyro_x_dps);
    ident_format_cdps(gy_text, sizeof(gy_text), obs->gyro_y_dps);

    APP_Control_QueueText("IDENT sample id=%lu seq=%lu t_ms=%lu axis=%s mode=%s alpha_us=%u beta_us=%u roll=%s pitch=%s gx=%s gy=%s rc_arm=%u throttle_us=%u\r\n",
                          (unsigned long)ident_ctx.id,
                          (unsigned long)ident_ctx.seq,
                          (unsigned long)t_ms,
                          ident_axis_name(ident_ctx.axis),
                          ident_mode_name(ident_ctx.mode),
                          (unsigned int)ident_ctx.alpha_target_us,
                          (unsigned int)ident_ctx.beta_target_us,
                          roll_text,
                          pitch_text,
                          gx_text,
                          gy_text,
                          (unsigned int)obs->rc_armed,
                          (unsigned int)obs->throttle_us);
    ++ident_ctx.seq;
}
