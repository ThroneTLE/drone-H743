#include "app_magxy.h"

#include "app_control.h"
#include "app_control_internal.h"
#include "app_current_format.h"
#include "app_flash_service.h"
#include "app_stabilizer.h"
#include "bsp_critical.h"
#include "drv_frame_contract.h"

#include <stddef.h>
#include <string.h>

/* Until a valid CFG v27 record is loaded, the zero-initialised candidate
 * leaves the original no-mag behaviour in place. */
static SVC_MAG_HeadingConfig magxy_config;

_Static_assert(sizeof(APP_MagXY_Persisted) == 20U,
               "CFG v27 MAGXY payload must remain frozen at 20 bytes");

void APP_MagXY_GetConfig(SVC_MAG_HeadingConfig *out)
{
    uint32_t lock;
    if (out == NULL) {
        return;
    }
    lock = BSP_Critical_Enter();
    *out = magxy_config;
    BSP_Critical_Exit(lock);
}

void APP_MagXY_GetPersisted(APP_MagXY_Persisted *out)
{
    SVC_MAG_HeadingConfig config;

    if (out == NULL) {
        return;
    }
    APP_MagXY_GetConfig(&config);
    memset(out, 0, sizeof(*out));
    if (SVC_MAG_HeadingConfigValid(&config) == 0U) {
        return;
    }
    out->bias_x_mgauss = config.bias_x_mgauss;
    out->bias_y_mgauss = config.bias_y_mgauss;
    out->radius_xy_mgauss = config.radius_xy_mgauss;
    out->frame_contract_version = config.frame_contract_version;
    out->axis_verified = config.axis_verified;
    out->enabled_default = config.enabled;
}

void APP_MagXY_ApplyPersisted(const APP_MagXY_Persisted *record)
{
    SVC_MAG_HeadingConfig candidate = {0};
    uint32_t lock;

    if ((record != NULL) && (record->axis_verified <= 1U) &&
        (record->enabled_default <= 1U)) {
        candidate.bias_x_mgauss = record->bias_x_mgauss;
        candidate.bias_y_mgauss = record->bias_y_mgauss;
        candidate.radius_xy_mgauss = record->radius_xy_mgauss;
        candidate.frame_contract_version = record->frame_contract_version;
        candidate.axis_verified = record->axis_verified;
        if (SVC_MAG_HeadingConfigValid(&candidate) != 0U) {
            candidate.enabled = record->enabled_default;
            if ((candidate.axis_verified == 0U) ||
                (candidate.frame_contract_version !=
                 (uint32_t)DRV_FRAME_CONTRACT_VERSION)) {
                candidate.enabled = 0U;
            }
        } else {
            memset(&candidate, 0, sizeof(candidate));
        }
    }
    lock = BSP_Critical_Enter();
    candidate.generation = magxy_config.generation + 1U;
    magxy_config = candidate;
    BSP_Critical_Exit(lock);
}

static void magxy_report(void)
{
    SVC_MAG_HeadingConfig config;
    APP_Stabilizer_MagXYStatus runtime;
    uint8_t axis_effective;
    char bx[16];
    char by[16];
    char radius[16];
    char z_mean[16];

    APP_MagXY_GetConfig(&config);
    APP_Stabilizer_GetMagXYStatus(&runtime);
    axis_effective = ((config.axis_verified != 0U) &&
        (config.frame_contract_version == (uint32_t)DRV_FRAME_CONTRACT_VERSION)) ?
        1U : 0U;
    (void)APP_Current_FormatFixed(bx, config.bias_x_mgauss, 3U);
    (void)APP_Current_FormatFixed(by, config.bias_y_mgauss, 3U);
    (void)APP_Current_FormatFixed(radius, config.radius_xy_mgauss, 3U);
    (void)APP_Current_FormatFixed(z_mean, runtime.z_mean_mgauss, 2U);
    APP_Control_QueueText(
        "MAGXY configured=%u enabled=%u axis_effective=%u generation=%lu\r\n",
        (unsigned int)SVC_MAG_HeadingConfigValid(&config),
        (unsigned int)config.enabled,
        (unsigned int)axis_effective,
        (unsigned long)config.generation);
    APP_Control_QueueText("MAGXY bias_xy_mgauss=%s,%s radius_xy_mgauss=%s cfg_version=27\r\n",
                          bx, by, radius);
    APP_Control_QueueText("MAGXY runtime ready=%lu tilt=%lu z_mean_mgauss=%s last=%u\r\n",
                          (unsigned long)runtime.ready_count,
                          (unsigned long)runtime.tilt_count,
                          z_mean,
                          (unsigned int)runtime.last_result);
}

uint8_t APP_MagXY_HandleCommand(char **tokens, uint32_t count)
{
    SVC_MAG_HeadingConfig candidate;
    uint32_t lock;

    if ((tokens == NULL) || (count == 0U)) {
        return 0U;
    }
    if ((strcmp(tokens[0], "MAGXY") != 0) &&
        (strcmp(tokens[0], "MAGXY?") != 0)) {
        return 0U;
    }
    if ((strcmp(tokens[0], "MAGXY?") == 0) || (count == 1U)) {
        magxy_report();
        return 1U;
    }
    /* The same disarmed-only rule as MAGCAL/MAGFRAME. Enabling a new heading
     * source during motor operation could change the yaw reference abruptly.
     */
    if (APP_Stabilizer_IsArmed() != 0U) {
        APP_Control_QueueText("MAGXY state=armed_blocked\r\n");
        return 1U;
    }
    if ((count == 5U) && (strcmp(tokens[1], "SET") == 0)) {
        float bx;
        float by;
        float radius;
        if ((app_control_parse_f32(tokens[2], &bx) == 0U) ||
            (app_control_parse_f32(tokens[3], &by) == 0U) ||
            (app_control_parse_f32(tokens[4], &radius) == 0U)) {
            APP_Control_QueueText("ERR usage MAGXY SET <bias_x> <bias_y> <radius_xy>\r\n");
            return 1U;
        }
        APP_MagXY_GetConfig(&candidate);
        candidate.bias_x_mgauss = bx;
        candidate.bias_y_mgauss = by;
        candidate.radius_xy_mgauss = radius;
        candidate.enabled = 0U; /* SET alone must never activate fusion. */
        if (SVC_MAG_HeadingConfigValid(&candidate) == 0U) {
            APP_Control_QueueText("MAGXY state=set_rejected reason=invalid_values\r\n");
            return 1U;
        }
        ++candidate.generation;
        lock = BSP_Critical_Enter();
        magxy_config = candidate;
        BSP_Critical_Exit(lock);
        APP_Control_QueueText("MAGXY state=set_ram enabled=0\r\n");
        return 1U;
    }
    if ((count == 3U) && (strcmp(tokens[1], "VERIFY") == 0) &&
        (strcmp(tokens[2], "CONFIRM") == 0)) {
        APP_MagXY_GetConfig(&candidate);
        if (SVC_MAG_HeadingConfigValid(&candidate) == 0U) {
            APP_Control_QueueText("MAGXY state=verify_rejected reason=unconfigured\r\n");
            return 1U;
        }
        candidate.axis_verified = 1U;
        candidate.frame_contract_version = (uint32_t)DRV_FRAME_CONTRACT_VERSION;
        /* The requested policy is default-on once the physical axis is
         * verified. COMMIT makes that policy survive the next boot. */
        candidate.enabled = 1U;
        ++candidate.generation;
        lock = BSP_Critical_Enter();
        magxy_config = candidate;
        BSP_Critical_Exit(lock);
        APP_Control_QueueText("MAGXY state=axis_verified_ram enabled=1\r\n");
        return 1U;
    }
    if ((count == 3U) && (strcmp(tokens[1], "ENABLE") == 0)) {
        uint32_t enabled;
        if ((app_control_parse_u32(tokens[2], &enabled) == 0U) ||
            (enabled > 1U)) {
            APP_Control_QueueText("ERR usage MAGXY ENABLE 0|1\r\n");
            return 1U;
        }
        APP_MagXY_GetConfig(&candidate);
        if ((enabled != 0U) &&
            (SVC_MAG_HeadingConfigValid(&candidate) == 0U)) {
            APP_Control_QueueText("MAGXY state=enable_rejected reason=unconfigured\r\n");
            return 1U;
        }
        if ((enabled != 0U) &&
            ((candidate.axis_verified == 0U) ||
             (candidate.frame_contract_version !=
              (uint32_t)DRV_FRAME_CONTRACT_VERSION))) {
            APP_Control_QueueText("MAGXY state=enable_rejected reason=axis_unverified\r\n");
            return 1U;
        }
        candidate.enabled = (uint8_t)enabled;
        ++candidate.generation;
        lock = BSP_Critical_Enter();
        magxy_config = candidate;
        BSP_Critical_Exit(lock);
        APP_Control_QueueText("MAGXY state=%s ram=1\r\n",
                              (enabled != 0U) ? "enabled" : "disabled");
        return 1U;
    }
    if ((count == 2U) && (strcmp(tokens[1], "CLEAR") == 0)) {
        APP_MagXY_GetConfig(&candidate);
        lock = BSP_Critical_Enter();
        memset(&magxy_config, 0, sizeof(magxy_config));
        magxy_config.generation = candidate.generation + 1U;
        BSP_Critical_Exit(lock);
        APP_Control_QueueText("MAGXY state=cleared_ram\r\n");
        return 1U;
    }
    if ((count == 2U) && (strcmp(tokens[1], "COMMIT") == 0)) {
        uint8_t save_status;
        APP_MagXY_GetConfig(&candidate);
        if ((SVC_MAG_HeadingConfigValid(&candidate) == 0U) &&
            ((candidate.bias_x_mgauss != 0.0f) ||
             (candidate.bias_y_mgauss != 0.0f) ||
             (candidate.radius_xy_mgauss != 0.0f))) {
            APP_Control_QueueText("MAGXY state=commit_failed reason=invalid_record\r\n");
            return 1U;
        }
        if ((candidate.enabled != 0U) &&
            ((candidate.axis_verified == 0U) ||
             (candidate.frame_contract_version !=
              (uint32_t)DRV_FRAME_CONTRACT_VERSION))) {
            APP_Control_QueueText("MAGXY state=commit_failed reason=axis_unverified\r\n");
            return 1U;
        }
        save_status = app_control_internal_commit_config_persist();
        if (save_status != (uint8_t)APP_FLASH_SERVICE_OK) {
            APP_Control_QueueText("MAGXY state=commit_failed st=%u\r\n",
                                  (unsigned int)save_status);
            return 1U;
        }
        APP_Control_QueueText("MAGXY state=committed enabled_default=%u\r\n",
                              (unsigned int)candidate.enabled);
        return 1U;
    }
    APP_Control_QueueText("ERR usage MAGXY? | SET <bx> <by> <radius> | VERIFY CONFIRM | ENABLE 0|1 | CLEAR | COMMIT\r\n");
    return 1U;
}
