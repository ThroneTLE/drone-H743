#include "app_servo_feedback_bench.h"

#include "app_control.h"
#include "bsp_bus_servo.h"

#include <stddef.h>
#include <string.h>

#define APP_SERVO_FB_MIN_RATE_HZ          1U
#define APP_SERVO_FB_MAX_RATE_HZ        200U
#define APP_SERVO_FB_MIN_DURATION_MS   1000U
#define APP_SERVO_FB_MAX_DURATION_MS  60000U
#define APP_SERVO_FB_MIN_TIMEOUT_MS       2U
#define APP_SERVO_FB_MAX_TIMEOUT_MS     100U
#define APP_SERVO_FB_MOVE_PERIOD_MS      10U
#define APP_SERVO_FB_QUERY_START_DELAY_US 4000ULL

typedef enum {
    APP_SERVO_FB_MODE_IDLE = 0,
    APP_SERVO_FB_MODE_SINGLE,
    APP_SERVO_FB_MODE_SWEEP
} APP_ServoFeedbackBenchMode;

typedef struct {
    APP_ServoFeedbackBenchMode mode;
    uint32_t rate_hz;
    uint32_t duration_ms;
    uint32_t timeout_ms;
    uint32_t segment_index;
    uint32_t segment_start_ms;
    uint64_t next_query_due_us;
    uint32_t query_interval_us;
    uint32_t last_event_sequence;
    uint32_t lost_event_count;
    uint8_t next_slot;
    uint8_t id[2];
    uint32_t scheduled[2];
    uint32_t launched[2];
    uint32_t busy[2];
    uint32_t request_error[2];
    uint32_t valid[2];
    uint32_t timeout[2];
    uint32_t parse_error[2];
    uint32_t uart_error[2];
    uint32_t duplicate[2];
    uint32_t feedback_change[2];
    uint32_t target_change[2];
    uint32_t rtt_sum_ms[2];
    uint32_t rtt_max_ms[2];
    uint32_t move_attempt_count;
    uint32_t move_success_count;
    uint32_t move_busy_count;
    uint32_t move_error_count;
    uint16_t last_position_us[2];
    uint16_t last_target_us[2];
    uint8_t position_valid[2];
    uint8_t target_valid[2];
} APP_ServoFeedbackBenchContext;

static const uint32_t servo_fb_sweep_rates_hz[] = {
    10U, 25U, 50U, 75U, 100U, 125U, 150U, 175U, 200U
};
static APP_ServoFeedbackBenchContext servo_fb_ctx;

static uint8_t servo_fb_args_valid(uint32_t rate_hz,
                                   uint32_t duration_ms,
                                   uint32_t timeout_ms)
{
    return ((rate_hz >= APP_SERVO_FB_MIN_RATE_HZ) &&
            (rate_hz <= APP_SERVO_FB_MAX_RATE_HZ) &&
            (duration_ms >= APP_SERVO_FB_MIN_DURATION_MS) &&
            (duration_ms <= APP_SERVO_FB_MAX_DURATION_MS) &&
            (timeout_ms >= APP_SERVO_FB_MIN_TIMEOUT_MS) &&
            (timeout_ms <= APP_SERVO_FB_MAX_TIMEOUT_MS)) ? 1U : 0U;
}

static const char *servo_fb_mode_name(APP_ServoFeedbackBenchMode mode)
{
    switch (mode) {
    case APP_SERVO_FB_MODE_SINGLE: return "single";
    case APP_SERVO_FB_MODE_SWEEP: return "sweep";
    default: return "idle";
    }
}

static int32_t servo_fb_slot_from_id(uint8_t id)
{
    if (servo_fb_ctx.id[0] == id) { return 0; }
    if (servo_fb_ctx.id[1] == id) { return 1; }
    return -1;
}

static void servo_fb_reset_segment(uint32_t now_ms)
{
    DRV_SERVO_FeedbackDiag diag;

    memset(servo_fb_ctx.scheduled, 0, sizeof(servo_fb_ctx.scheduled));
    memset(servo_fb_ctx.launched, 0, sizeof(servo_fb_ctx.launched));
    memset(servo_fb_ctx.busy, 0, sizeof(servo_fb_ctx.busy));
    memset(servo_fb_ctx.request_error, 0, sizeof(servo_fb_ctx.request_error));
    memset(servo_fb_ctx.valid, 0, sizeof(servo_fb_ctx.valid));
    memset(servo_fb_ctx.timeout, 0, sizeof(servo_fb_ctx.timeout));
    memset(servo_fb_ctx.parse_error, 0, sizeof(servo_fb_ctx.parse_error));
    memset(servo_fb_ctx.uart_error, 0, sizeof(servo_fb_ctx.uart_error));
    memset(servo_fb_ctx.duplicate, 0, sizeof(servo_fb_ctx.duplicate));
    memset(servo_fb_ctx.feedback_change, 0, sizeof(servo_fb_ctx.feedback_change));
    memset(servo_fb_ctx.target_change, 0, sizeof(servo_fb_ctx.target_change));
    memset(servo_fb_ctx.rtt_sum_ms, 0, sizeof(servo_fb_ctx.rtt_sum_ms));
    memset(servo_fb_ctx.rtt_max_ms, 0, sizeof(servo_fb_ctx.rtt_max_ms));
    memset(servo_fb_ctx.last_position_us, 0,
           sizeof(servo_fb_ctx.last_position_us));
    memset(servo_fb_ctx.last_target_us, 0,
           sizeof(servo_fb_ctx.last_target_us));
    memset(servo_fb_ctx.position_valid, 0, sizeof(servo_fb_ctx.position_valid));
    memset(servo_fb_ctx.target_valid, 0, sizeof(servo_fb_ctx.target_valid));
    servo_fb_ctx.lost_event_count = 0U;
    servo_fb_ctx.move_attempt_count = 0U;
    servo_fb_ctx.move_success_count = 0U;
    servo_fb_ctx.move_busy_count = 0U;
    servo_fb_ctx.move_error_count = 0U;
    servo_fb_ctx.next_slot = 0U;
    servo_fb_ctx.segment_start_ms = now_ms;
    servo_fb_ctx.query_interval_us =
        1000000U / (2U * servo_fb_ctx.rate_hz);
    servo_fb_ctx.next_query_due_us =
        (uint64_t)now_ms * 1000ULL + APP_SERVO_FB_QUERY_START_DELAY_US;

    BSP_BusServo_GetFeedbackDiag(&diag);
    servo_fb_ctx.last_event_sequence = diag.last_event.sequence;
}

static uint32_t servo_fb_rate_centi_hz(uint32_t count, uint32_t elapsed_ms)
{
    if (elapsed_ms == 0U) { return 0U; }
    return (count * 100000U) / elapsed_ms;
}

static uint32_t servo_fb_reply_permille(uint32_t valid, uint32_t launched)
{
    if (launched == 0U) { return 0U; }
    return (valid * 1000U) / launched;
}

static void servo_fb_report_segment(const char *reason, uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - servo_fb_ctx.segment_start_ms;

    APP_Control_QueueText(
        "SERVO_FB result mode=%s segment=%lu rate_hz=%lu duration_ms=%lu elapsed_ms=%lu timeout_ms=%lu baud=%lu reason=%s lost_events=%lu move_attempt=%lu move_ok=%lu move_busy=%lu move_err=%lu move_hz_x100=%lu\r\n",
        servo_fb_mode_name(servo_fb_ctx.mode),
        (unsigned long)servo_fb_ctx.segment_index,
        (unsigned long)servo_fb_ctx.rate_hz,
        (unsigned long)servo_fb_ctx.duration_ms,
        (unsigned long)elapsed_ms,
        (unsigned long)servo_fb_ctx.timeout_ms,
        (unsigned long)BSP_BusServo_GetBaudRate(),
        (reason != NULL) ? reason : "none",
        (unsigned long)servo_fb_ctx.lost_event_count,
        (unsigned long)servo_fb_ctx.move_attempt_count,
        (unsigned long)servo_fb_ctx.move_success_count,
        (unsigned long)servo_fb_ctx.move_busy_count,
        (unsigned long)servo_fb_ctx.move_error_count,
        (unsigned long)servo_fb_rate_centi_hz(
            servo_fb_ctx.move_success_count, elapsed_ms));

    for (uint32_t slot = 0U; slot < 2U; ++slot) {
        uint32_t avg_rtt_ms = (servo_fb_ctx.valid[slot] != 0U) ?
            (servo_fb_ctx.rtt_sum_ms[slot] / servo_fb_ctx.valid[slot]) : 0U;
        uint32_t achieved_centi_hz =
            servo_fb_rate_centi_hz(servo_fb_ctx.valid[slot], elapsed_ms);

        APP_Control_QueueText(
            "SERVO_FB servo=%lu id=%u scheduled=%lu launched=%lu valid=%lu reply_permille=%lu achieved_hz_x100=%lu busy=%lu req_err=%lu timeout=%lu parse=%lu uart=%lu avg_rtt_ms=%lu max_rtt_ms=%lu last_us=%u duplicate=%lu fb_changes=%lu target_changes=%lu\r\n",
            (unsigned long)slot,
            (unsigned int)servo_fb_ctx.id[slot],
            (unsigned long)servo_fb_ctx.scheduled[slot],
            (unsigned long)servo_fb_ctx.launched[slot],
            (unsigned long)servo_fb_ctx.valid[slot],
            (unsigned long)servo_fb_reply_permille(servo_fb_ctx.valid[slot],
                                                   servo_fb_ctx.launched[slot]),
            (unsigned long)achieved_centi_hz,
            (unsigned long)servo_fb_ctx.busy[slot],
            (unsigned long)servo_fb_ctx.request_error[slot],
            (unsigned long)servo_fb_ctx.timeout[slot],
            (unsigned long)servo_fb_ctx.parse_error[slot],
            (unsigned long)servo_fb_ctx.uart_error[slot],
            (unsigned long)avg_rtt_ms,
            (unsigned long)servo_fb_ctx.rtt_max_ms[slot],
            (unsigned int)servo_fb_ctx.last_position_us[slot],
            (unsigned long)servo_fb_ctx.duplicate[slot],
            (unsigned long)servo_fb_ctx.feedback_change[slot],
            (unsigned long)servo_fb_ctx.target_change[slot]);
    }
}

static void servo_fb_process_event(void)
{
    DRV_SERVO_FeedbackDiag diag;
    const DRV_SERVO_FeedbackEvent *event;
    int32_t slot;

    BSP_BusServo_GetFeedbackDiag(&diag);
    event = &diag.last_event;
    if (event->sequence == servo_fb_ctx.last_event_sequence) {
        return;
    }
    if (event->sequence > (servo_fb_ctx.last_event_sequence + 1U)) {
        servo_fb_ctx.lost_event_count +=
            event->sequence - servo_fb_ctx.last_event_sequence - 1U;
    }
    servo_fb_ctx.last_event_sequence = event->sequence;
    slot = servo_fb_slot_from_id(event->id);
    if (slot < 0) {
        return;
    }

    switch ((DRV_SERVO_FeedbackEventType)event->type) {
    case DRV_SERVO_FEEDBACK_EVENT_VALID:
        servo_fb_ctx.valid[slot]++;
        servo_fb_ctx.rtt_sum_ms[slot] += event->rtt_ms;
        if (event->rtt_ms > servo_fb_ctx.rtt_max_ms[slot]) {
            servo_fb_ctx.rtt_max_ms[slot] = event->rtt_ms;
        }
        if (servo_fb_ctx.position_valid[slot] != 0U) {
            if (event->position_us == servo_fb_ctx.last_position_us[slot]) {
                servo_fb_ctx.duplicate[slot]++;
            } else {
                servo_fb_ctx.feedback_change[slot]++;
            }
        }
        servo_fb_ctx.last_position_us[slot] = event->position_us;
        servo_fb_ctx.position_valid[slot] = 1U;
        break;
    case DRV_SERVO_FEEDBACK_EVENT_TIMEOUT:
        servo_fb_ctx.timeout[slot]++;
        break;
    case DRV_SERVO_FEEDBACK_EVENT_PARSE_ERROR:
        servo_fb_ctx.parse_error[slot]++;
        break;
    case DRV_SERVO_FEEDBACK_EVENT_UART_ERROR:
        servo_fb_ctx.uart_error[slot]++;
        break;
    default:
        break;
    }
}

static void servo_fb_begin_segment(uint32_t now_ms)
{
    servo_fb_reset_segment(now_ms);
    APP_Control_QueueText(
        "SERVO_FB start mode=%s segment=%lu rate_hz=%lu duration_ms=%lu timeout_ms=%lu move_hz=100 motors_locked=1\r\n",
        servo_fb_mode_name(servo_fb_ctx.mode),
        (unsigned long)servo_fb_ctx.segment_index,
        (unsigned long)servo_fb_ctx.rate_hz,
        (unsigned long)servo_fb_ctx.duration_ms,
        (unsigned long)servo_fb_ctx.timeout_ms);
}

void APP_ServoFeedbackBench_Init(void)
{
    memset(&servo_fb_ctx, 0, sizeof(servo_fb_ctx));
}

uint8_t APP_ServoFeedbackBench_Start(uint32_t rate_hz,
                                     uint32_t duration_ms,
                                     uint32_t timeout_ms,
                                     uint32_t now_ms)
{
    if ((servo_fb_ctx.mode != APP_SERVO_FB_MODE_IDLE) ||
        (servo_fb_args_valid(rate_hz, duration_ms, timeout_ms) == 0U)) {
        return 0U;
    }

    servo_fb_ctx.mode = APP_SERVO_FB_MODE_SINGLE;
    servo_fb_ctx.rate_hz = rate_hz;
    servo_fb_ctx.duration_ms = duration_ms;
    servo_fb_ctx.timeout_ms = timeout_ms;
    servo_fb_ctx.segment_index = 0U;
    servo_fb_begin_segment(now_ms);
    return 1U;
}

uint8_t APP_ServoFeedbackBench_StartSweep(uint32_t duration_ms,
                                          uint32_t timeout_ms,
                                          uint32_t now_ms)
{
    if ((servo_fb_ctx.mode != APP_SERVO_FB_MODE_IDLE) ||
        (servo_fb_args_valid(servo_fb_sweep_rates_hz[0],
                             duration_ms,
                             timeout_ms) == 0U)) {
        return 0U;
    }

    servo_fb_ctx.mode = APP_SERVO_FB_MODE_SWEEP;
    servo_fb_ctx.rate_hz = servo_fb_sweep_rates_hz[0];
    servo_fb_ctx.duration_ms = duration_ms;
    servo_fb_ctx.timeout_ms = timeout_ms;
    servo_fb_ctx.segment_index = 0U;
    servo_fb_begin_segment(now_ms);
    return 1U;
}

void APP_ServoFeedbackBench_Stop(const char *reason, uint32_t now_ms)
{
    if (servo_fb_ctx.mode == APP_SERVO_FB_MODE_IDLE) {
        return;
    }

    servo_fb_process_event();
    servo_fb_report_segment((reason != NULL) ? reason : "stopped", now_ms);
    servo_fb_ctx.mode = APP_SERVO_FB_MODE_IDLE;
    APP_Control_QueueText("SERVO_FB stopped reason=%s\r\n",
                          (reason != NULL) ? reason : "stopped");
}

void APP_ServoFeedbackBench_ReportStatus(uint32_t now_ms)
{
    APP_Control_QueueText(
        "SERVO_FB status mode=%s active=%u segment=%lu rate_hz=%lu elapsed_ms=%lu duration_ms=%lu timeout_ms=%lu pending=%u\r\n",
        servo_fb_mode_name(servo_fb_ctx.mode),
        (unsigned int)(servo_fb_ctx.mode != APP_SERVO_FB_MODE_IDLE),
        (unsigned long)servo_fb_ctx.segment_index,
        (unsigned long)servo_fb_ctx.rate_hz,
        (unsigned long)((servo_fb_ctx.mode != APP_SERVO_FB_MODE_IDLE) ?
                        (now_ms - servo_fb_ctx.segment_start_ms) : 0U),
        (unsigned long)servo_fb_ctx.duration_ms,
        (unsigned long)servo_fb_ctx.timeout_ms,
        (unsigned int)(BSP_BusServo_IsIdle() == 0U));
}

void APP_ServoFeedbackBench_Step(uint32_t now_ms,
                                const DRV_SERVO_MoveCmd moves[2])
{
    uint64_t now_us;
    uint32_t slot;
    DRV_SERVO_Status status;

    if ((servo_fb_ctx.mode == APP_SERVO_FB_MODE_IDLE) || (moves == NULL)) {
        return;
    }

    for (slot = 0U; slot < 2U; ++slot) {
        servo_fb_ctx.id[slot] = moves[slot].id;
        if (servo_fb_ctx.target_valid[slot] != 0U) {
            if (moves[slot].pulse_us != servo_fb_ctx.last_target_us[slot]) {
                servo_fb_ctx.target_change[slot]++;
            }
        }
        servo_fb_ctx.last_target_us[slot] = moves[slot].pulse_us;
        servo_fb_ctx.target_valid[slot] = 1U;
    }

    servo_fb_process_event();
    if ((now_ms - servo_fb_ctx.segment_start_ms) >= servo_fb_ctx.duration_ms) {
        if (BSP_BusServo_IsIdle() == 0U) {
            return;
        }

        servo_fb_report_segment("complete", now_ms);
        if (servo_fb_ctx.mode == APP_SERVO_FB_MODE_SINGLE) {
            servo_fb_ctx.mode = APP_SERVO_FB_MODE_IDLE;
            APP_Control_QueueText("SERVO_FB done\r\n");
            return;
        }

        servo_fb_ctx.segment_index++;
        if (servo_fb_ctx.segment_index >=
            (sizeof(servo_fb_sweep_rates_hz) / sizeof(servo_fb_sweep_rates_hz[0]))) {
            servo_fb_ctx.mode = APP_SERVO_FB_MODE_IDLE;
            APP_Control_QueueText("SERVO_FB sweep_done\r\n");
            return;
        }
        servo_fb_ctx.rate_hz = servo_fb_sweep_rates_hz[servo_fb_ctx.segment_index];
        servo_fb_begin_segment(now_ms);
        return;
    }

    now_us = (uint64_t)now_ms * 1000ULL;
    if (now_us < servo_fb_ctx.next_query_due_us) {
        return;
    }

    slot = servo_fb_ctx.next_slot;
    servo_fb_ctx.next_slot ^= 1U;
    servo_fb_ctx.scheduled[slot]++;
    status = BSP_BusServo_RequestPositionAsync(moves[slot].id,
                                               servo_fb_ctx.timeout_ms);
    if (status == DRV_SERVO_OK) {
        servo_fb_ctx.launched[slot]++;
    } else if (status == DRV_SERVO_BUSY) {
        servo_fb_ctx.busy[slot]++;
    } else {
        servo_fb_ctx.request_error[slot]++;
    }

    servo_fb_ctx.next_query_due_us += servo_fb_ctx.query_interval_us;
    if (servo_fb_ctx.next_query_due_us <= now_us) {
        servo_fb_ctx.next_query_due_us = now_us + servo_fb_ctx.query_interval_us;
    }
}

void APP_ServoFeedbackBench_RecordMoveResult(DRV_SERVO_Status status)
{
    if (servo_fb_ctx.mode == APP_SERVO_FB_MODE_IDLE) {
        return;
    }

    servo_fb_ctx.move_attempt_count++;
    if (status == DRV_SERVO_OK) {
        servo_fb_ctx.move_success_count++;
    } else if (status == DRV_SERVO_BUSY) {
        servo_fb_ctx.move_busy_count++;
    } else {
        servo_fb_ctx.move_error_count++;
    }
}

uint8_t APP_ServoFeedbackBench_IsActive(void)
{
    return (servo_fb_ctx.mode != APP_SERVO_FB_MODE_IDLE) ? 1U : 0U;
}

uint8_t APP_ServoFeedbackBench_MoveRefreshDue(uint32_t now_ms,
                                              uint32_t last_move_ms)
{
    return ((servo_fb_ctx.mode != APP_SERVO_FB_MODE_IDLE) &&
            ((now_ms - last_move_ms) >= APP_SERVO_FB_MOVE_PERIOD_MS)) ? 1U : 0U;
}
