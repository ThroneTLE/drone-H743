#ifndef APP_MAGXY_H
#define APP_MAGXY_H

/* Near-level XY magnetic heading configuration.
 *
 * This is a separate mode from 3-D MAGCAL. A valid, axis-verified CFG v27
 * record requests XY aid automatically after boot. Missing/old/invalid
 * records fall back to no XY aid. The independent axis proof cannot activate
 * an old 3-D MAGCAL record. The stabilizer owns the filter state; this
 * module owns the cross-task configuration snapshot.
 */

#include "svc_mag_heading.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frozen CFG v27 payload. No runtime generation or implicit struct padding. */
typedef struct {
    float bias_x_mgauss;
    float bias_y_mgauss;
    float radius_xy_mgauss;
    uint32_t frame_contract_version;
    uint8_t axis_verified;
    uint8_t enabled_default;
    uint8_t reserved[2];
} APP_MagXY_Persisted;

void APP_MagXY_GetConfig(SVC_MAG_HeadingConfig *out);
void APP_MagXY_GetPersisted(APP_MagXY_Persisted *out);
void APP_MagXY_ApplyPersisted(const APP_MagXY_Persisted *record);
uint8_t APP_MagXY_HandleCommand(char **tokens, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif
