#ifndef APP_CONTROL_SCHEDULER_H
#define APP_CONTROL_SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Deterministic cascade scheduler for StabilizerTask.
 *
 * `now_us` is the monotonic control timestamp from SVC_Timestamp.  Position
 * and velocity work additionally require a new navigation sample timestamp;
 * reusing the same sample never advances their integrators.  The scheduler
 * contains no RTOS/HAL/I/O calls and does not own controller state.
 */
#define APP_CONTROL_SCHED_RATE_PERIOD_US      2000ULL  /* 500 Hz */
#define APP_CONTROL_SCHED_ATTITUDE_PERIOD_US  4000ULL  /* 250 Hz */
#define APP_CONTROL_SCHED_VELOCITY_PERIOD_US 10000ULL  /* 100 Hz */
#define APP_CONTROL_SCHED_POSITION_PERIOD_US 20000ULL  /*  50 Hz */

typedef struct {
    uint64_t last_now_us;
    uint64_t last_nav_sample_us;
    uint64_t last_rate_us;
    uint64_t last_attitude_us;
    uint64_t last_velocity_us;
    uint64_t last_position_us;
    uint8_t initialized;
} APP_ControlSchedulerState;

typedef struct {
    uint8_t rate_due;
    uint8_t attitude_due;
    uint8_t velocity_due;
    uint8_t position_due;
    uint8_t navigation_sample_new;
    uint8_t timestamp_fault;
    float rate_dt_s;
    float attitude_dt_s;
    float velocity_dt_s;
    float position_dt_s;
} APP_ControlSchedule;

void APP_ControlScheduler_Reset(APP_ControlSchedulerState *state);
void APP_ControlScheduler_Step(APP_ControlSchedulerState *state,
                               uint64_t now_us,
                               uint64_t navigation_sample_us,
                               uint8_t navigation_valid,
                               APP_ControlSchedule *schedule);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_SCHEDULER_H */
