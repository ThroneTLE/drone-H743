#ifndef APP_CMD_SERVOCAL_H
#define APP_CMD_SERVOCAL_H

#include <stdint.h>
#include "app_flight_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * SERVOCAL command domain — split from app_control.c (R-S6-2).
 *
 * Entry points called by app_control.c dispatch/tick:
 *   app_control_handle_servocal  — command parser (SERVOCAL? / APPLY / REVERT / COMMIT)
 *   app_control_service_servocal — periodic retry for pending Flash save
 *
 * Adapters for cross-domain state accessed by app_control.c:
 *   app_cmd_servocal_notify_persisted — called by imuframe_sync_param on confirmed generation change
 *   app_cmd_servocal_is_busy          — returns 1 if SERVOCAL preview or commit is active
 */

/* Original entry points (name unchanged, static removed). */
void app_cmd_servocal_init(void);
void app_control_handle_servocal(char **tokens, uint32_t count);
void app_control_service_servocal(void);

/* Adapter: called by app_control_imuframe_sync_param() when a Flash-confirmed
 * calibration record arrives.  Encapsulates the servocal commit-verify and
 * auto-revert logic that was previously inline in the observer. */
void app_cmd_servocal_notify_persisted(
    const APP_FlightCalibration *calibration);

/* Adapter: returns 1 when a SERVOCAL preview or commit is in flight, used by
 * IMUCAL to reject concurrent calibration sessions. */
uint8_t app_cmd_servocal_is_busy(void);

#ifdef __cplusplus
}
#endif

#endif
