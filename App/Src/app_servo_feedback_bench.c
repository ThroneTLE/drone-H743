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
#define APP_SERVO_FB_STEP_SETTLE_MS     300U
#define APP_SERVO_FB_STEP_MIN_HOLD_MS   300U
#define APP_SERVO_FB_STEP_MAX_HOLD_MS  5000U
#define APP_SERVO_FB_STEP_MIN_DELTA_US   20U
#define APP_SERVO_FB_STEP_MAX_DELTA_US  200U
#define APP_SERVO_FB_STEP_MAX_RATE_HZ   100U
#define APP_SERVO_FB_CENTER_US          1500U
#define APP_SERVO_FB_STEP_MAX_SAMPLES   256U

typedef enum {
    APP_SERVO_FB_MODE_IDLE = 0,
    APP_SERVO_FB_MODE_SINGLE,
    APP_SERVO_FB_MODE_SWEEP,
    APP_SERVO_FB_MODE_STEP,
    APP_SERVO_FB_MODE_STEP_HOLD
} APP_ServoFeedbackBenchMode;

typedef struct {
    uint32_t timestamp_ms;
    uint16_t target_us;
    uint16_t actual_us;
    uint16_t rtt_ms;
    uint8_t id;
    uint8_t reserved;
} APP_ServoFeedbackStepSample;

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
    uint8_t step_slot;
    uint16_t step_delta_us;
    uint16_t step_target_us;
    uint32_t step_hold_ms;
    uint32_t step_sample_sequence;
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
__attribute__((section(".ram_d1_noinit"), aligned(32)))
static APP_ServoFeedbackStepSample
    servo_step_samples[APP_SERVO_FB_STEP_MAX_SAMPLES];

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
    case APP_SERVO_FB_MODE_STEP: return "step";
    case APP_SERVO_FB_MODE_STEP_HOLD: return "step_hold";
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
    servo_fb_ctx.query_interval_us = 1000000U /
        (((servo_fb_ctx.mode == APP_SERVO_FB_MODE_STEP) ? 1U : 2U) *
         servo_fb_ctx.rate_hz);
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
        if ((servo_fb_ctx.mode == APP_SERVO_FB_MODE_STEP) &&
            ((uint32_t)slot == servo_fb_ctx.step_slot)) {
            uint32_t sample_index = servo_fb_ctx.step_sample_sequence;

            if (sample_index < APP_SERVO_FB_STEP_MAX_SAMPLES) {
                servo_step_samples[sample_index].timestamp_ms =
                    event->timestamp_ms - servo_fb_ctx.segment_start_ms;
                servo_step_samples[sample_index].target_us =
                    servo_fb_ctx.step_target_us;
                servo_step_samples[sample_index].actual_us = event->position_us;
                servo_step_samples[sample_index].rtt_ms =
                    (uint16_t)event->rtt_ms;
                servo_step_samples[sample_index].id = event->id;
                servo_step_samples[sample_index].reserved = 0U;
                servo_fb_ctx.step_sample_sequence = sample_index + 1U;
            }
        }
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

uint8_t APP_ServoFeedbackBench_StartStep(uint32_t servo_index,
                                         uint32_t delta_us,
                                         uint32_t rate_hz,
                                         uint32_t hold_ms,
                                         uint32_t timeout_ms,
                                         uint32_t now_ms)
{
    uint32_t duration_ms = (3U * APP_SERVO_FB_STEP_SETTLE_MS) +
                           (2U * hold_ms);
    uint32_t expected_samples =
        ((duration_ms * rate_hz) + 999U) / 1000U;

    if ((servo_fb_ctx.mode != APP_SERVO_FB_MODE_IDLE) ||
        (servo_index >= 2U) ||
        (delta_us < APP_SERVO_FB_STEP_MIN_DELTA_US) ||
        (delta_us > APP_SERVO_FB_STEP_MAX_DELTA_US) ||
        (rate_hz > APP_SERVO_FB_STEP_MAX_RATE_HZ) ||
        (hold_ms < APP_SERVO_FB_STEP_MIN_HOLD_MS) ||
        (hold_ms > APP_SERVO_FB_STEP_MAX_HOLD_MS) ||
        (expected_samples > APP_SERVO_FB_STEP_MAX_SAMPLES) ||
        (servo_fb_args_valid(rate_hz,
                             duration_ms,
                             timeout_ms) == 0U)) {
        return 0U;
    }

    servo_fb_ctx.mode = APP_SERVO_FB_MODE_STEP;
    servo_fb_ctx.rate_hz = rate_hz;
    servo_fb_ctx.duration_ms = duration_ms;
    servo_fb_ctx.timeout_ms = timeout_ms;
    servo_fb_ctx.segment_index = 0U;
    servo_fb_ctx.step_slot = (uint8_t)servo_index;
    servo_fb_ctx.step_delta_us = (uint16_t)delta_us;
    servo_fb_ctx.step_target_us = APP_SERVO_FB_CENTER_US;
    servo_fb_ctx.step_hold_ms = hold_ms;
    servo_fb_ctx.step_sample_sequence = 0U;
    memset(servo_step_samples, 0, sizeof(servo_step_samples));
    servo_fb_begin_segment(now_ms);
    APP_Control_QueueText(
        "SERVO_STEP profile servo=%lu center_us=%u delta_us=%lu settle_ms=%u hold_ms=%lu duration_ms=%lu\r\n",
        (unsigned long)servo_index,
        (unsigned int)APP_SERVO_FB_CENTER_US,
        (unsigned long)delta_us,
        (unsigned int)APP_SERVO_FB_STEP_SETTLE_MS,
        (unsigned long)hold_ms,
        (unsigned long)duration_ms);
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

void APP_ServoFeedbackBench_ApplyTargets(uint32_t now_ms,
                                         DRV_SERVO_MoveCmd moves[2])
{
    uint32_t elapsed_ms;
    uint32_t first_end_ms;
    uint32_t second_start_ms;
    uint32_t second_end_ms;
    uint16_t target_us = APP_SERVO_FB_CENTER_US;

    if ((moves == NULL) ||
        ((servo_fb_ctx.mode != APP_SERVO_FB_MODE_STEP) &&
         (servo_fb_ctx.mode != APP_SERVO_FB_MODE_STEP_HOLD))) {
        return;
    }

    elapsed_ms = now_ms - servo_fb_ctx.segment_start_ms;
    first_end_ms = APP_SERVO_FB_STEP_SETTLE_MS + servo_fb_ctx.step_hold_ms;
    second_start_ms = first_end_ms + APP_SERVO_FB_STEP_SETTLE_MS;
    second_end_ms = second_start_ms + servo_fb_ctx.step_hold_ms;
    if ((servo_fb_ctx.mode == APP_SERVO_FB_MODE_STEP) &&
        (elapsed_ms >= APP_SERVO_FB_STEP_SETTLE_MS) &&
        (elapsed_ms < first_end_ms)) {
        target_us = (uint16_t)(APP_SERVO_FB_CENTER_US +
                               servo_fb_ctx.step_delta_us);
    } else if ((servo_fb_ctx.mode == APP_SERVO_FB_MODE_STEP) &&
               (elapsed_ms >= second_start_ms) &&
               (elapsed_ms < second_end_ms)) {
        target_us = (uint16_t)(APP_SERVO_FB_CENTER_US -
                               servo_fb_ctx.step_delta_us);
    }

    moves[0].pulse_us = APP_SERVO_FB_CENTER_US;
    moves[1].pulse_us = APP_SERVO_FB_CENTER_US;
    moves[servo_fb_ctx.step_slot].pulse_us = target_us;
    servo_fb_ctx.step_target_us = target_us;
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

    if (servo_fb_ctx.mode == APP_SERVO_FB_MODE_STEP_HOLD) {
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
        if (servo_fb_ctx.mode == APP_SERVO_FB_MODE_STEP) {
            servo_fb_ctx.mode = APP_SERVO_FB_MODE_STEP_HOLD;
            APP_Control_QueueText(
                "SERVO_STEP ready samples=%lu motors_locked=1 target_us=%u; use SERVO FB STOP\r\n",
                (unsigned long)servo_fb_ctx.step_sample_sequence,
                (unsigned int)APP_SERVO_FB_CENTER_US);
            return;
        }
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

    if (servo_fb_ctx.mode == APP_SERVO_FB_MODE_STEP) {
        slot = servo_fb_ctx.step_slot;
    } else {
        slot = servo_fb_ctx.next_slot;
        servo_fb_ctx.next_slot ^= 1U;
    }
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
            (servo_fb_ctx.mode != APP_SERVO_FB_MODE_STEP) &&
            (servo_fb_ctx.mode != APP_SERVO_FB_MODE_STEP_HOLD) &&
            ((now_ms - last_move_ms) >= APP_SERVO_FB_MOVE_PERIOD_MS)) ? 1U : 0U;
}
