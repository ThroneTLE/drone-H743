#include "app_servo_feedback.h"

#include "bsp_bus_servo.h"

#include <stddef.h>
#include <string.h>

/* Alternate two servos every 10 ms: 50 Hz per servo, 100 Hz aggregate. */
#define APP_SERVO_FEEDBACK_QUERY_INTERVAL_MS 10U
#define APP_SERVO_FEEDBACK_TIMEOUT_MS         8U
#define APP_SERVO_FEEDBACK_STALE_MS         250U
#define APP_SERVO_FEEDBACK_AGE_MAX_MS     65535U

typedef struct {
    uint32_t next_query_ms;
    uint32_t last_driver_event_sequence;
    uint32_t timestamp_ms[APP_SERVO_FEEDBACK_SLOT_COUNT];
    uint16_t position_us[APP_SERVO_FEEDBACK_SLOT_COUNT];
    uint16_t sample_sequence[APP_SERVO_FEEDBACK_SLOT_COUNT];
    uint8_t id[APP_SERVO_FEEDBACK_SLOT_COUNT];
    uint8_t position_valid_mask;
    uint8_t latest_event_valid_mask;
    uint8_t next_slot;
    uint8_t initialized;
} APP_ServoFeedbackContext;

static APP_ServoFeedbackContext servo_feedback_ctx;

static uint8_t servo_feedback_slot_mask(uint32_t slot)
{
    return (uint8_t)(1U << slot);
}

static int32_t servo_feedback_slot_from_id(uint8_t id)
{
    for (uint32_t slot = 0U; slot < APP_SERVO_FEEDBACK_SLOT_COUNT; ++slot) {
        if (servo_feedback_ctx.id[slot] == id) {
            return (int32_t)slot;
        }
    }
    return -1;
}

static void servo_feedback_update_ids(const DRV_SERVO_MoveCmd moves[2])
{
    for (uint32_t slot = 0U; slot < APP_SERVO_FEEDBACK_SLOT_COUNT; ++slot) {
        uint8_t mask = servo_feedback_slot_mask(slot);

        if (servo_feedback_ctx.id[slot] != moves[slot].id) {
            servo_feedback_ctx.id[slot] = moves[slot].id;
            servo_feedback_ctx.position_valid_mask &= (uint8_t)~mask;
            servo_feedback_ctx.latest_event_valid_mask &= (uint8_t)~mask;
            servo_feedback_ctx.sample_sequence[slot] = 0U;
        }
    }
}

static void servo_feedback_process_event(void)
{
    DRV_SERVO_FeedbackDiag diag;
    const DRV_SERVO_FeedbackEvent *event;
    int32_t slot;
    uint8_t mask;

    BSP_BusServo_GetFeedbackDiag(&diag);
    event = &diag.last_event;
    if (event->sequence == servo_feedback_ctx.last_driver_event_sequence) {
        return;
    }

    servo_feedback_ctx.last_driver_event_sequence = event->sequence;
    slot = servo_feedback_slot_from_id(event->id);
    if (slot < 0) {
        return;
    }
    mask = servo_feedback_slot_mask((uint32_t)slot);

    if ((DRV_SERVO_FeedbackEventType)event->type ==
        DRV_SERVO_FEEDBACK_EVENT_VALID) {
        servo_feedback_ctx.position_us[slot] = event->position_us;
        servo_feedback_ctx.timestamp_ms[slot] = event->timestamp_ms;
        servo_feedback_ctx.sample_sequence[slot]++;
        servo_feedback_ctx.position_valid_mask |= mask;
        servo_feedback_ctx.latest_event_valid_mask |= mask;
    } else {
        servo_feedback_ctx.latest_event_valid_mask &= (uint8_t)~mask;
    }
}

void APP_ServoFeedback_Init(void)
{
    DRV_SERVO_FeedbackDiag diag;

    memset(&servo_feedback_ctx, 0, sizeof(servo_feedback_ctx));
    BSP_BusServo_GetFeedbackDiag(&diag);
    servo_feedback_ctx.last_driver_event_sequence = diag.last_event.sequence;
    servo_feedback_ctx.initialized = 1U;
}

void APP_ServoFeedback_Service(uint32_t now_ms,
                               const DRV_SERVO_MoveCmd moves[2],
                               uint8_t polling_enabled)
{
    DRV_SERVO_Status status;
    uint32_t slot;

    if (moves == NULL) {
        return;
    }
    if (servo_feedback_ctx.initialized == 0U) {
        APP_ServoFeedback_Init();
    }

    servo_feedback_update_ids(moves);
    servo_feedback_process_event();

    if (polling_enabled == 0U) {
        servo_feedback_ctx.next_query_ms =
            now_ms + APP_SERVO_FEEDBACK_QUERY_INTERVAL_MS;
        return;
    }
    if (servo_feedback_ctx.next_query_ms == 0U) {
        servo_feedback_ctx.next_query_ms =
            now_ms + APP_SERVO_FEEDBACK_QUERY_INTERVAL_MS;
        return;
    }
    if ((int32_t)(now_ms - servo_feedback_ctx.next_query_ms) < 0) {
        return;
    }
    if (BSP_BusServo_IsIdle() == 0U) {
        return;
    }

    slot = servo_feedback_ctx.next_slot;
    status = BSP_BusServo_RequestPositionAsync(
        moves[slot].id, APP_SERVO_FEEDBACK_TIMEOUT_MS);
    if (status == DRV_SERVO_OK) {
        servo_feedback_ctx.next_slot ^= 1U;
    } else if (status == DRV_SERVO_BUSY) {
        return;
    } else {
        servo_feedback_ctx.next_slot ^= 1U;
    }
    servo_feedback_ctx.next_query_ms =
        now_ms + APP_SERVO_FEEDBACK_QUERY_INTERVAL_MS;
}

void APP_ServoFeedback_GetLogSample(uint32_t now_ms,
                                    APP_ServoFeedbackLogSample *sample)
{
    if (sample == NULL) {
        return;
    }

    memset(sample, 0, sizeof(*sample));
    for (uint32_t slot = 0U; slot < APP_SERVO_FEEDBACK_SLOT_COUNT; ++slot) {
        uint8_t mask = servo_feedback_slot_mask(slot);
        uint32_t age_ms;

        if ((servo_feedback_ctx.position_valid_mask & mask) == 0U) {
            continue;
        }

        age_ms = now_ms - servo_feedback_ctx.timestamp_ms[slot];
        sample->position_us[slot] = servo_feedback_ctx.position_us[slot];
        sample->age_ms[slot] = (uint16_t)((age_ms > APP_SERVO_FEEDBACK_AGE_MAX_MS) ?
                               APP_SERVO_FEEDBACK_AGE_MAX_MS : age_ms);
        sample->sample_sequence[slot] =
            servo_feedback_ctx.sample_sequence[slot];
        if (((servo_feedback_ctx.latest_event_valid_mask & mask) != 0U) &&
            (age_ms <= APP_SERVO_FEEDBACK_STALE_MS)) {
            sample->valid_mask |= mask;
        }
    }
}
