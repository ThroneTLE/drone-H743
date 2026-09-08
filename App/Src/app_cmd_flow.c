#include "app_control.h"
#include "app_control_internal.h"

#include "app_optical_flow.h"
#include "app_stabilizer.h"
#include "bsp_optical_flow.h"
#include "svc_flow_nav.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define APP_CONTROL_FLOW_RAW_MAX_BYTES 32U

#define app_control_acceptance_milli \
    app_control_internal_acceptance_milli

static uint8_t app_control_parse_hex_byte(const char *text, uint8_t *value)
{
    char *end = NULL;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0')) {
        return 0U;
    }

    parsed = strtoul(text, &end, 16);
    if ((end == text) || (*end != '\0') || (parsed > 0xFFUL)) {
        return 0U;
    }

    *value = (uint8_t)parsed;
    return 1U;
}

void app_control_handle_flow(char **tokens, uint32_t count)
{
    uint8_t tx_bytes[APP_CONTROL_FLOW_RAW_MAX_BYTES];
    uint8_t rx_bytes[16];
    uint32_t tx_count;
    uint16_t rx_count;
    BSP_OPTICAL_FLOW_StatusCode status;

    if ((count == 1U) || ((count >= 2U) && (strcmp(tokens[1], "?") == 0))) {
        APP_Control_QueueText("ERR usage FLOW ZERO | FLOW TX hex... | FLOW RX [max] | FLOW XCV rx_len hex...\r\n");
        return;
    }

    if (strcmp(tokens[1], "ZERO") == 0) {
        /*
         * 只清里程计（累计位移 + 积分步数）。控制用的位置状态、速度估计、EKF
         * 协方差一概不动——里程计不参与任何控制律，清它对飞行没有影响，
         * 所以这条命令在空中发也是安全的。
         */
        float dx_m = 0.0f;
        float dy_m = 0.0f;

        SVC_FlowNav_ResetDisplacement();
        SVC_FlowNav_GetDisplacement(&dx_m, &dy_m);
        APP_Control_QueueText("FLOW ZERO ok disp_x_mm=%ld disp_y_mm=%ld steps=%lu\r\n",
                              (long)(dx_m * 1000.0f),
                              (long)(dy_m * 1000.0f),
                              (unsigned long)SVC_FlowNav_GetIntegratedStepCount());
        return;
    }

    if (strcmp(tokens[1], "RX") == 0) {
        uint32_t max_rx = 3U;
        if (count >= 3U) {
            max_rx = strtoul(tokens[2], NULL, 0);
        }
        if ((max_rx == 0U) || (max_rx > sizeof(rx_bytes))) {
            APP_Control_QueueText("ERR usage FLOW RX 1..16\r\n");
            return;
        }
        rx_count = BSP_OPTICAL_FLOW_ReceiveRaw(rx_bytes, (uint16_t)max_rx, 100U);
        APP_Control_QueueText("FLOW RX n=%u data=%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                              (unsigned int)rx_count,
                              (unsigned int)((rx_count > 0U) ? rx_bytes[0] : 0U),
                              (unsigned int)((rx_count > 1U) ? rx_bytes[1] : 0U),
                              (unsigned int)((rx_count > 2U) ? rx_bytes[2] : 0U),
                              (unsigned int)((rx_count > 3U) ? rx_bytes[3] : 0U),
                              (unsigned int)((rx_count > 4U) ? rx_bytes[4] : 0U),
                              (unsigned int)((rx_count > 5U) ? rx_bytes[5] : 0U),
                              (unsigned int)((rx_count > 6U) ? rx_bytes[6] : 0U),
                              (unsigned int)((rx_count > 7U) ? rx_bytes[7] : 0U));
        return;
    }

    if (strcmp(tokens[1], "XCV") == 0) {
        uint32_t max_rx;

        if ((count < 4U) || ((count - 3U) > APP_CONTROL_FLOW_RAW_MAX_BYTES)) {
            APP_Control_QueueText("ERR usage FLOW XCV rx_len hex... max=%u\r\n",
                                  (unsigned int)APP_CONTROL_FLOW_RAW_MAX_BYTES);
            return;
        }

        max_rx = strtoul(tokens[2], NULL, 0);
        if ((max_rx == 0U) || (max_rx > sizeof(rx_bytes))) {
            APP_Control_QueueText("ERR usage FLOW XCV rx_len 1..16 hex...\r\n");
            return;
        }

        tx_count = count - 3U;
        for (uint32_t i = 0U; i < tx_count; ++i) {
            if (app_control_parse_hex_byte(tokens[i + 3U], &tx_bytes[i]) == 0U) {
                APP_Control_QueueText("ERR flow hex %s\r\n", tokens[i + 3U]);
                return;
            }
        }

        status = BSP_OPTICAL_FLOW_TransceiveRaw(tx_bytes, (uint16_t)tx_count,
                                                rx_bytes, (uint16_t)max_rx,
                                                &rx_count, 100U);
        APP_Control_QueueText("FLOW XCV st=%ld tx_n=%lu rx_n=%u data=%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
                              (long)status,
                              (unsigned long)tx_count,
                              (unsigned int)rx_count,
                              (unsigned int)((rx_count > 0U) ? rx_bytes[0] : 0U),
                              (unsigned int)((rx_count > 1U) ? rx_bytes[1] : 0U),
                              (unsigned int)((rx_count > 2U) ? rx_bytes[2] : 0U),
                              (unsigned int)((rx_count > 3U) ? rx_bytes[3] : 0U),
                              (unsigned int)((rx_count > 4U) ? rx_bytes[4] : 0U),
                              (unsigned int)((rx_count > 5U) ? rx_bytes[5] : 0U),
                              (unsigned int)((rx_count > 6U) ? rx_bytes[6] : 0U),
                              (unsigned int)((rx_count > 7U) ? rx_bytes[7] : 0U));
        return;
    }

    if (strcmp(tokens[1], "TX") != 0) {
        APP_Control_QueueText("ERR unknown flow subcmd %s\r\n", tokens[1]);
        return;
    }

    if ((count < 3U) || ((count - 2U) > APP_CONTROL_FLOW_RAW_MAX_BYTES)) {
        APP_Control_QueueText("ERR usage FLOW TX hex... max=%u\r\n",
                              (unsigned int)APP_CONTROL_FLOW_RAW_MAX_BYTES);
        return;
    }

    tx_count = count - 2U;
    for (uint32_t i = 0U; i < tx_count; ++i) {
        if (app_control_parse_hex_byte(tokens[i + 2U], &tx_bytes[i]) == 0U) {
            APP_Control_QueueText("ERR flow hex %s\r\n", tokens[i + 2U]);
            return;
        }
    }

    status = BSP_OPTICAL_FLOW_TransmitRaw(tx_bytes, (uint16_t)tx_count, 100U);
    APP_Control_QueueText("FLOW TX st=%ld n=%lu\r\n",
                          (long)status,
                          (unsigned long)tx_count);
}

void app_control_report_flow(void)
{
    StabilizerFlowCompensationSnapshot snapshot;

    APP_OpticalFlow_Report();
    memset(&snapshot, 0, sizeof(snapshot));
    if (APP_Stabilizer_ReadFlowCompensationSnapshot(&snapshot) == 0U) {
        APP_Control_QueueText(
            "FLOW comp valid=0 export=canonical_flu reason=no_snapshot\r\n");
        return;
    }
    APP_Control_QueueText(
        "FLOW comp valid=%u sample_ms=%lu contract=%u orientation=%u "
        "source=calibrated_body_flu export=canonical_flu "
        "sensor_vx_mm_s=%ld sensor_vy_mm_s=%ld "
        "corr_vx_mm_s=%ld corr_vy_mm_s=%ld\r\n",
        (unsigned int)snapshot.valid,
        (unsigned long)snapshot.sample_ms,
        (unsigned int)snapshot.frame_contract,
        (unsigned int)snapshot.orientation_code,
        (long)app_control_acceptance_milli(
            snapshot.sensor_velocity_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.sensor_velocity_flu_m_s[1]),
        (long)app_control_acceptance_milli(
            snapshot.corrected_velocity_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.corrected_velocity_flu_m_s[1]));
    APP_Control_QueueText(
        "FLOW comp_terms sample_ms=%lu export=canonical_flu "
        "opt_rot_vx_mm_s=%ld opt_rot_vy_mm_s=%ld "
        "offset_rot_vx_mm_s=%ld offset_rot_vy_mm_s=%ld\r\n",
        (unsigned long)snapshot.sample_ms,
        (long)app_control_acceptance_milli(
            snapshot.optical_rot_comp_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.optical_rot_comp_flu_m_s[1]),
        (long)app_control_acceptance_milli(
            snapshot.offset_rot_comp_flu_m_s[0]),
        (long)app_control_acceptance_milli(
            snapshot.offset_rot_comp_flu_m_s[1]));
}
