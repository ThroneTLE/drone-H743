#include "app_acceptance.h"

#include "bsp_pwm.h"

#include <string.h>

#define APP_ACCEPTANCE_SERVO_CENTER_US 1500U
#define APP_ACCEPTANCE_SERVO_DELTA_US    50U
#define APP_ACCEPTANCE_SNAPSHOT_RETRIES   8U

typedef struct {
    uint32_t lease_id;
    uint32_t issued_ms;
    uint32_t expires_ms;
    APP_AcceptanceStage stage;
    uint8_t props_removed;
    uint8_t active;
} APP_AcceptanceState;

static volatile uint32_t acceptance_state_seqlock;
static volatile APP_AcceptanceState acceptance_state;
static volatile uint32_t acceptance_snapshot_seqlock;
static volatile APP_AcceptanceSnapshot acceptance_snapshot;
static uint32_t acceptance_next_lease;
static uint64_t acceptance_link_loss_start_us;

static void acceptance_barrier(void)
{
#if defined(__GNUC__) || defined(__clang__)
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

static APP_AcceptanceState acceptance_read_state(void)
{
    APP_AcceptanceState value;
    uint32_t before;
    uint32_t after;
    do {
        before = acceptance_state_seqlock;
        acceptance_barrier();
        memcpy(&value, (const void *)&acceptance_state, sizeof(value));
        acceptance_barrier();
        after = acceptance_state_seqlock;
    } while ((before != after) || ((after & 1U) != 0U));
    return value;
}

static void acceptance_write_state(const APP_AcceptanceState *value)
{
    acceptance_state_seqlock++;
    acceptance_barrier();
    memcpy((void *)&acceptance_state, value, sizeof(*value));
    acceptance_barrier();
    acceptance_state_seqlock++;
}

void APP_Acceptance_Init(void)
{
    APP_AcceptanceState state = {0};
    APP_AcceptanceSnapshot snapshot = {0};
    state.stage = APP_ACCEPT_STAGE_RC_CENTER;
    acceptance_write_state(&state);
    acceptance_snapshot_seqlock++;
    memcpy((void *)&acceptance_snapshot, &snapshot, sizeof(snapshot));
    acceptance_snapshot_seqlock++;
    acceptance_next_lease = 0xA7430000UL;
    acceptance_link_loss_start_us = 0ULL;
}

uint8_t APP_Acceptance_Start(uint32_t now_ms, uint8_t props_removed,
                             uint32_t *lease_id)
{
    APP_AcceptanceState state = {0};
    if ((props_removed == 0U) || (lease_id == NULL)) {
        return 0U;
    }
    (void)BSP_PWM_DisableEsc(1U);
    (void)BSP_PWM_DisableEsc(2U);
    if ((BSP_PWM_GetEscPulse(1U) != 0U) ||
        (BSP_PWM_GetEscPulse(2U) != 0U)) {
        return 0U;
    }
    ++acceptance_next_lease;
    if (acceptance_next_lease == 0U) {
        ++acceptance_next_lease;
    }
    state.lease_id = acceptance_next_lease;
    state.issued_ms = now_ms;
    state.expires_ms = now_ms + APP_ACCEPTANCE_LEASE_MS;
    state.stage = APP_ACCEPT_STAGE_RC_CENTER;
    state.props_removed = 1U;
    state.active = 1U;
    acceptance_write_state(&state);
    *lease_id = state.lease_id;
    return 1U;
}

uint8_t APP_Acceptance_Keepalive(uint32_t now_ms, uint32_t lease_id)
{
    APP_AcceptanceState state = acceptance_read_state();
    if ((state.active == 0U) || (state.lease_id != lease_id) ||
        ((int32_t)(state.expires_ms - now_ms) < 0)) {
        APP_Acceptance_Stop();
        return 0U;
    }
    state.expires_ms = now_ms + APP_ACCEPTANCE_LEASE_MS;
    acceptance_write_state(&state);
    return 1U;
}

void APP_Acceptance_Stop(void)
{
    APP_AcceptanceState state = acceptance_read_state();
    state.active = 0U;
    acceptance_write_state(&state);
    (void)BSP_PWM_DisableEsc(1U);
    (void)BSP_PWM_DisableEsc(2U);
}

void APP_Acceptance_Service(uint32_t now_ms)
{
    APP_AcceptanceState state = acceptance_read_state();
    if ((state.active != 0U) &&
        ((int32_t)(state.expires_ms - now_ms) < 0)) {
        APP_Acceptance_Stop();
    }
}

uint8_t APP_Acceptance_IsActive(void)
{
    return acceptance_read_state().active;
}

void APP_Acceptance_GetLeaseStatus(APP_AcceptanceLeaseStatus *status)
{
    APP_AcceptanceState state;
    if (status == NULL) return;
    state = acceptance_read_state();
    status->lease_id = state.lease_id;
    status->issued_ms = state.issued_ms;
    status->expires_ms = state.expires_ms;
    status->stage = state.stage;
    status->props_removed = state.props_removed;
    status->active = state.active;
}

uint8_t APP_Acceptance_SetStage(APP_AcceptanceStage stage)
{
    APP_AcceptanceState state = acceptance_read_state();
    if ((state.active == 0U) || (stage >= APP_ACCEPT_STAGE_COUNT)) {
        return 0U;
    }
    state.stage = stage;
    acceptance_write_state(&state);
    return 1U;
}

uint8_t APP_Acceptance_GetServoOverride(uint16_t *alpha_us,
                                        uint16_t *beta_us)
{
    APP_AcceptanceState state = acceptance_read_state();
    if ((state.active == 0U) || (alpha_us == NULL) || (beta_us == NULL)) {
        return 0U;
    }
    *alpha_us = APP_ACCEPTANCE_SERVO_CENTER_US;
    *beta_us = APP_ACCEPTANCE_SERVO_CENTER_US;
    if (state.stage == APP_ACCEPT_STAGE_SERVO_ALPHA_POSITIVE) {
        *alpha_us += APP_ACCEPTANCE_SERVO_DELTA_US;
    } else if (state.stage == APP_ACCEPT_STAGE_SERVO_ALPHA_NEGATIVE) {
        *alpha_us -= APP_ACCEPTANCE_SERVO_DELTA_US;
    } else if (state.stage == APP_ACCEPT_STAGE_SERVO_BETA_POSITIVE) {
        *beta_us += APP_ACCEPTANCE_SERVO_DELTA_US;
    } else if (state.stage == APP_ACCEPT_STAGE_SERVO_BETA_NEGATIVE) {
        *beta_us -= APP_ACCEPTANCE_SERVO_DELTA_US;
    }
    return 1U;
}

void APP_Acceptance_PublishObservation(
    const APP_AcceptanceObservation *observation)
{
    APP_AcceptanceSnapshot next = {0};
    APP_AcceptanceState state;
    if (observation == NULL) {
        return;
    }
    state = acceptance_read_state();
    next.timestamp_us = observation->timestamp_us;
    next.sequence = observation->sequence;
    next.lease_id = state.lease_id;
    next.lease_issued_ms = state.issued_ms;
    next.lease_expires_ms = state.expires_ms;
    next.calibration_generation = observation->calibration_generation;
    memcpy(next.nav_velocity_m_s, observation->nav_velocity_m_s,
           sizeof(next.nav_velocity_m_s));
    memcpy(next.angle_deg, observation->angle_deg, sizeof(next.angle_deg));
    memcpy(next.rate_dps, observation->rate_dps, sizeof(next.rate_dps));
    memcpy(next.moment_n_m, observation->moment_n_m, sizeof(next.moment_n_m));
    memcpy(next.restoring_moment_n_m, observation->restoring_moment_n_m,
           sizeof(next.restoring_moment_n_m));
    memcpy(next.damping_moment_n_m, observation->damping_moment_n_m,
           sizeof(next.damping_moment_n_m));
    memcpy(next.rc_us, observation->rc_us, sizeof(next.rc_us));
    next.servo_center_us[0] = APP_ACCEPTANCE_SERVO_CENTER_US;
    next.servo_center_us[1] = APP_ACCEPTANCE_SERVO_CENTER_US;
    memcpy(next.servo_command_us, observation->servo_command_us,
           sizeof(next.servo_command_us));
    memcpy(next.servo_sent_us, observation->servo_sent_us,
           sizeof(next.servo_sent_us));
    memcpy(next.servo_feedback_us, observation->servo_feedback_us,
           sizeof(next.servo_feedback_us));
    memcpy(next.servo_feedback_age_ms, observation->servo_feedback_age_ms,
           sizeof(next.servo_feedback_age_ms));
    next.esc_ccr[0] = BSP_PWM_GetEscPulse(1U);
    next.esc_ccr[1] = BSP_PWM_GetEscPulse(2U);
    next.stage = (uint8_t)state.stage;
    next.orientation_code = observation->orientation_code;
    next.calibration_valid_mask = observation->calibration_valid_mask;
    next.servo_feedback_valid_mask = observation->servo_feedback_valid_mask;
    next.props_removed = state.props_removed;
    next.active = state.active;
    next.lease_valid = state.active;
    next.link_present = observation->link_present;
    next.failsafe_active = observation->link_present ? 0U : 1U;
    if (observation->link_present != 0U) {
        acceptance_link_loss_start_us = 0ULL;
    } else {
        if (acceptance_link_loss_start_us == 0ULL) {
            acceptance_link_loss_start_us = observation->timestamp_us;
        }
        next.failsafe_elapsed_ms = (uint32_t)(
            (observation->timestamp_us - acceptance_link_loss_start_us) / 1000ULL);
    }
    acceptance_snapshot_seqlock++;
    acceptance_barrier();
    memcpy((void *)&acceptance_snapshot, &next, sizeof(next));
    acceptance_barrier();
    acceptance_snapshot_seqlock++;
}

uint8_t APP_Acceptance_ReadSnapshot(APP_AcceptanceSnapshot *snapshot)
{
    uint32_t before;
    uint32_t after;
    uint32_t retry;
    if (snapshot == NULL) return 0U;
    for (retry = 0U; retry < APP_ACCEPTANCE_SNAPSHOT_RETRIES; ++retry) {
        before = acceptance_snapshot_seqlock;
        if ((before & 1U) != 0U) continue;
        acceptance_barrier();
        memcpy(snapshot, (const void *)&acceptance_snapshot, sizeof(*snapshot));
        acceptance_barrier();
        after = acceptance_snapshot_seqlock;
        if ((before == after) && ((after & 1U) == 0U) &&
            (snapshot->timestamp_us != 0ULL)) return 1U;
    }
    return 0U;
}

static const char *const acceptance_stage_names[APP_ACCEPT_STAGE_COUNT] = {
    "rc_center", "rc_positive_roll", "rc_positive_pitch", "rc_positive_yaw",
    "nav_static", "nav_forward", "nav_left", "restore_positive_roll",
    "restore_positive_pitch", "servo_alpha_positive_50us",
    "servo_alpha_negative_50us", "servo_beta_positive_50us",
    "servo_beta_negative_50us", "failsafe"
};

const char *APP_Acceptance_StageText(APP_AcceptanceStage stage)
{
    return (stage < APP_ACCEPT_STAGE_COUNT) ? acceptance_stage_names[stage] : "invalid";
}

uint8_t APP_Acceptance_ParseStage(const char *text, APP_AcceptanceStage *stage)
{
    uint32_t index;
    if ((text == NULL) || (stage == NULL)) return 0U;
    for (index = 0U; index < APP_ACCEPT_STAGE_COUNT; ++index) {
        if (strcmp(text, acceptance_stage_names[index]) == 0) {
            *stage = (APP_AcceptanceStage)index;
            return 1U;
        }
    }
    return 0U;
}
