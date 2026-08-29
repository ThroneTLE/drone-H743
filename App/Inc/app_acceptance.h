#ifndef APP_ACCEPTANCE_H
#define APP_ACCEPTANCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_ACCEPTANCE_LEASE_MS 500U

typedef enum {
    APP_ACCEPT_STAGE_RC_CENTER = 0U,
    APP_ACCEPT_STAGE_RC_POSITIVE_ROLL,
    APP_ACCEPT_STAGE_RC_POSITIVE_PITCH,
    APP_ACCEPT_STAGE_RC_POSITIVE_YAW,
    APP_ACCEPT_STAGE_NAV_STATIC,
    APP_ACCEPT_STAGE_NAV_FORWARD,
    APP_ACCEPT_STAGE_NAV_LEFT,
    APP_ACCEPT_STAGE_RESTORE_POSITIVE_ROLL,
    APP_ACCEPT_STAGE_RESTORE_POSITIVE_PITCH,
    APP_ACCEPT_STAGE_SERVO_ALPHA_POSITIVE,
    APP_ACCEPT_STAGE_SERVO_ALPHA_NEGATIVE,
    APP_ACCEPT_STAGE_SERVO_BETA_POSITIVE,
    APP_ACCEPT_STAGE_SERVO_BETA_NEGATIVE,
    APP_ACCEPT_STAGE_FAILSAFE,
    APP_ACCEPT_STAGE_COUNT
} APP_AcceptanceStage;

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    uint32_t lease_id;
    uint32_t lease_issued_ms;
    uint32_t lease_expires_ms;
    uint32_t calibration_generation;
    float nav_velocity_m_s[2];
    float angle_deg[2];
    float rate_dps[2];
    float moment_n_m[2];
    float restoring_moment_n_m[2];
    float damping_moment_n_m[2];
    uint16_t rc_us[3];
    uint16_t servo_center_us[2];
    uint16_t servo_command_us[2];
    uint16_t servo_sent_us[2];
    uint16_t servo_feedback_us[2];
    uint16_t servo_feedback_age_ms[2];
    uint16_t esc_ccr[2];
    uint32_t failsafe_elapsed_ms;
    uint8_t stage;
    uint8_t orientation_code;
    uint8_t calibration_valid_mask;
    uint8_t servo_feedback_valid_mask;
    uint8_t props_removed;
    uint8_t active;
    uint8_t lease_valid;
    uint8_t link_present;
    uint8_t failsafe_active;
} APP_AcceptanceSnapshot;

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    uint32_t calibration_generation;
    float nav_velocity_m_s[2];
    float angle_deg[2];
    float rate_dps[2];
    float moment_n_m[2];
    float restoring_moment_n_m[2];
    float damping_moment_n_m[2];
    uint16_t rc_us[3];
    uint16_t servo_command_us[2];
    uint16_t servo_sent_us[2];
    uint16_t servo_feedback_us[2];
    uint16_t servo_feedback_age_ms[2];
    uint8_t orientation_code;
    uint8_t calibration_valid_mask;
    uint8_t servo_feedback_valid_mask;
    uint8_t link_present;
} APP_AcceptanceObservation;

typedef struct {
    uint32_t lease_id;
    uint32_t issued_ms;
    uint32_t expires_ms;
    APP_AcceptanceStage stage;
    uint8_t props_removed;
    uint8_t active;
} APP_AcceptanceLeaseStatus;

void APP_Acceptance_Init(void);
uint8_t APP_Acceptance_Start(uint32_t now_ms, uint8_t props_removed,
                             uint32_t *lease_id);
uint8_t APP_Acceptance_Keepalive(uint32_t now_ms, uint32_t lease_id);
void APP_Acceptance_Stop(void);
void APP_Acceptance_Service(uint32_t now_ms);
uint8_t APP_Acceptance_IsActive(void);
void APP_Acceptance_GetLeaseStatus(APP_AcceptanceLeaseStatus *status);
uint8_t APP_Acceptance_SetStage(APP_AcceptanceStage stage);
uint8_t APP_Acceptance_GetServoOverride(uint16_t *alpha_us,
                                        uint16_t *beta_us);
void APP_Acceptance_PublishObservation(
    const APP_AcceptanceObservation *observation);
uint8_t APP_Acceptance_ReadSnapshot(APP_AcceptanceSnapshot *snapshot);
const char *APP_Acceptance_StageText(APP_AcceptanceStage stage);
uint8_t APP_Acceptance_ParseStage(const char *text,
                                  APP_AcceptanceStage *stage);

#ifdef __cplusplus
}
#endif
#endif
