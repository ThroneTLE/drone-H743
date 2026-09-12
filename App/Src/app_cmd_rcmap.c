#include "app_control_internal.h"

#include "app_elrs.h"
#include "app_flash_service.h"
#include "app_proto.h"
#include "app_rc_config.h"
#include "app_stabilizer.h"

#include "svc_timestamp.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* 与 STABILIZER_RC_LOSS_TIMEOUT_MS 一致：上位机看到的 fresh 要和控制环判定同源。 */
#define APP_CONTROL_RC_FRESH_TIMEOUT_MS 500U

/*
 * RAM 中的遥控映射工作副本。RCMAP SET 只改这里并立刻 Publish（所见即所得，
 * 上位机能马上在实时条上看到反向/端点的效果），RCMAP COMMIT 才落 Flash。
 */
static APP_RcConfig control_rc_config;
static uint8_t control_rc_config_dirty;

static void app_control_apply_rc_config(const APP_RcConfig *config)
{
    if ((config == NULL) || (APP_RcConfig_Validate(config) == 0U)) {
        APP_RcConfig_Defaults(&control_rc_config);
    } else {
        control_rc_config = *config;
    }
    control_rc_config_dirty = 0U;
    (void)APP_RcConfig_PublishActive(&control_rc_config);
}

static void app_control_report_rc_map(const char *state)
{
    uint8_t function;

    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_MAP,
        "RCMAP state=%s funcs=%u channels=%u deadband_us=%u calibrated=%u "
        "dirty=%u generation=%lu valid=%u\r\n",
        state,
        (unsigned int)APP_RC_FUNC_COUNT,
        (unsigned int)CRSF_CHANNEL_COUNT,
        (unsigned int)control_rc_config.deadband_us,
        (unsigned int)control_rc_config.calibrated,
        (unsigned int)control_rc_config_dirty,
        (unsigned long)APP_RcConfig_GetActiveGeneration(),
        (unsigned int)APP_RcConfig_Validate(&control_rc_config));
    for (function = 0U; function < APP_RC_FUNC_COUNT; ++function) {
        const APP_RcFunctionMap *map = &control_rc_config.function[function];

        app_control_queue_proto_text(
            APP_PROTO_MSG_RC_MAP,
            "RCMAP func=%s id=%u ch=%d rev=%u min=%u mid=%u max=%u\r\n",
            APP_RcConfig_FunctionName(function),
            (unsigned int)function,
            (map->channel == APP_RC_CHANNEL_UNBOUND) ? -1 : (int)map->channel,
            (unsigned int)map->reversed,
            (unsigned int)map->min_us,
            (unsigned int)map->mid_us,
            (unsigned int)map->max_us);
    }
}

void app_control_report_rc_live(void)
{
    uint16_t channels[CRSF_CHANNEL_COUNT];
    APP_RcInputs inputs;
    const DRV_ELRS_LinkStats *link;
    uint32_t now_ms = SVC_Timestamp_Ms();
    uint8_t fresh;

    APP_ELRS_GetChannels(channels);
    APP_RcConfig_Resolve(&control_rc_config, channels, &inputs);
    link = APP_ELRS_GetLinkStats();
    fresh = APP_ELRS_IsRcFresh(now_ms, APP_CONTROL_RC_FRESH_TIMEOUT_MS);

    /*
     * 两行拆分是为了每行都留在 APP_UART_TX_TEXT_SIZE 以内：16 路各 4 位数字
     * 加分隔符已经接近上限，链路统计只能另起一行。
     */
    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_LIVE,
        "RC us=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (unsigned int)channels[0], (unsigned int)channels[1],
        (unsigned int)channels[2], (unsigned int)channels[3],
        (unsigned int)channels[4], (unsigned int)channels[5],
        (unsigned int)channels[6], (unsigned int)channels[7],
        (unsigned int)channels[8], (unsigned int)channels[9],
        (unsigned int)channels[10], (unsigned int)channels[11],
        (unsigned int)channels[12], (unsigned int)channels[13],
        (unsigned int)channels[14], (unsigned int)channels[15]);
    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_LIVE,
        "RC link fresh=%u frames=%lu crc_err=%lu fps_x10=%lu lq=%u rssi=%u "
        "snr=%d age_ms=%lu armed=%u bound=0x%02X\r\n",
        (unsigned int)fresh,
        (unsigned long)APP_ELRS_GetRcFrames(),
        (unsigned long)APP_ELRS_GetCrcErrors(),
        (unsigned long)DRV_ELRS_GetFpsX10(),
        (unsigned int)((link != NULL) ? link->uplink_lq : 0U),
        (unsigned int)((link != NULL) ? link->uplink_rssi_1 : 0U),
        (int)((link != NULL) ? link->uplink_snr : 0),
        (unsigned long)(now_ms - APP_ELRS_GetLastRcMs()),
        (unsigned int)APP_Stabilizer_IsArmed(),
        (unsigned int)inputs.bound_mask);
    /* 归一化值单独一行：上位机据此画摇杆十字，无需自己复现标定公式。 */
    app_control_queue_proto_text(
        APP_PROTO_MSG_RC_LIVE,
        "RC norm roll=%d pitch=%d throttle=%d yaw=%d arm=%d mode=%d thr01=%d\r\n",
        (int)(inputs.norm[APP_RC_FUNC_ROLL] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_PITCH] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_THROTTLE] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_YAW] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_ARM] * 1000.0f),
        (int)(inputs.norm[APP_RC_FUNC_MODE] * 1000.0f),
        (int)(inputs.throttle_01 * 1000.0f));
}

/*
 * 改映射等于改"哪根杆是油门"。解锁状态下改一次就可能让电机响应错通道，
 * 所以所有写操作都要求飞控 disarmed，和 IMUFRAME 的安全门同源。
 */
static uint8_t app_control_rc_write_allowed(void)
{
    return (APP_Stabilizer_IsArmed() == 0U) ? 1U : 0U;
}

void app_control_handle_rc_map(char *tokens[], uint32_t count)
{
    APP_RcConfig candidate;
    uint8_t function;
    long channel;
    long reversed;
    long min_us;
    long mid_us;
    long max_us;
    long deadband;

    if ((count == 1U) || (strcmp(tokens[0], "RCMAP?") == 0)) {
        app_control_report_rc_map("status");
        return;
    }

    if (strcmp(tokens[1], "SET") == 0) {
        if (count != 8U) {
            app_control_report_rc_map("invalid_usage");
            return;
        }
        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        function = APP_RcConfig_FunctionFromName(tokens[2]);
        if (function >= APP_RC_FUNC_COUNT) {
            app_control_report_rc_map("bad_function");
            return;
        }
        channel  = strtol(tokens[3], NULL, 0);
        reversed = strtol(tokens[4], NULL, 0);
        min_us   = strtol(tokens[5], NULL, 0);
        mid_us   = strtol(tokens[6], NULL, 0);
        max_us   = strtol(tokens[7], NULL, 0);

        candidate = control_rc_config;
        candidate.function[function].channel =
            (channel < 0) ? APP_RC_CHANNEL_UNBOUND : (uint8_t)channel;
        candidate.function[function].reversed = (reversed != 0) ? 1U : 0U;
        candidate.function[function].min_us = (uint16_t)min_us;
        candidate.function[function].mid_us = (uint16_t)mid_us;
        candidate.function[function].max_us = (uint16_t)max_us;
        if (APP_RcConfig_Validate(&candidate) == 0U) {
            app_control_report_rc_map("rejected");
            return;
        }
        control_rc_config = candidate;
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("applied_ram");
        return;
    }

    if (strcmp(tokens[1], "DEADBAND") == 0) {
        if (count != 3U) {
            app_control_report_rc_map("invalid_usage");
            return;
        }
        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        deadband = strtol(tokens[2], NULL, 0);
        candidate = control_rc_config;
        candidate.deadband_us = (uint16_t)((deadband < 0) ? 0 : deadband);
        if (APP_RcConfig_Validate(&candidate) == 0U) {
            app_control_report_rc_map("rejected");
            return;
        }
        control_rc_config = candidate;
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("applied_ram");
        return;
    }

    if (strcmp(tokens[1], "CALIBRATED") == 0) {
        if ((count != 3U) || (app_control_rc_write_allowed() == 0U)) {
            app_control_report_rc_map(
                (count != 3U) ? "invalid_usage" : "armed_blocked");
            return;
        }
        control_rc_config.calibrated =
            (strtol(tokens[2], NULL, 0) != 0) ? 1U : 0U;
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("applied_ram");
        return;
    }

    if (strcmp(tokens[1], "RESET") == 0) {
        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        APP_RcConfig_Defaults(&control_rc_config);
        control_rc_config_dirty = 1U;
        (void)APP_RcConfig_PublishActive(&control_rc_config);
        app_control_report_rc_map("reset_ram");
        return;
    }

    if (strcmp(tokens[1], "COMMIT") == 0) {
        APP_FlashService_Status save_status;

        if (app_control_rc_write_allowed() == 0U) {
            app_control_report_rc_map("armed_blocked");
            return;
        }
        if (APP_RcConfig_Validate(&control_rc_config) == 0U) {
            app_control_report_rc_map("rejected");
            return;
        }
        save_status = app_control_internal_commit_config_persist();
        if (save_status != APP_FLASH_SERVICE_OK) {
            app_control_report_rc_map("commit_failed");
            return;
        }
        control_rc_config_dirty = 0U;
        app_control_report_rc_map("committed");
        return;
    }

    app_control_report_rc_map("invalid_usage");
}

void app_cmd_rcmap_apply_config(const void *config)
{
    app_control_apply_rc_config((const APP_RcConfig *)config);
}

const void *app_cmd_rcmap_config(void)
{
    return &control_rc_config;
}
