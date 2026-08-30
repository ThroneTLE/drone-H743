#include "app_control.h"
#include "app_control_internal.h"

#include "app_aiwb2.h"
#include "app_baro.h"
#include "app_flash.h"
#include "app_gps.h"
#include "app_mag.h"
#include "app_sensor.h"
#include "bsp_aiwb2_power.h"
#include "bsp_baro.h"
#include "bsp_imu.h"
#include "main.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *app_control_imu_stage_name(uint8_t stage)
{
    switch ((BSP_ICM42688_InitStage)stage) {
    case BSP_ICM42688_INIT_STAGE_NONE:
        return "none";
    case BSP_ICM42688_INIT_STAGE_BANK_SELECT:
        return "bank";
    case BSP_ICM42688_INIT_STAGE_RESET:
        return "reset";
    case BSP_ICM42688_INIT_STAGE_WHO_AM_I:
        return "who";
    case BSP_ICM42688_INIT_STAGE_GYRO_CONFIG:
        return "gyro_cfg";
    case BSP_ICM42688_INIT_STAGE_ACCEL_CONFIG:
        return "accel_cfg";
    case BSP_ICM42688_INIT_STAGE_FILTER_CONFIG:
        return "filter_cfg";
    case BSP_ICM42688_INIT_STAGE_PWR_MGMT:
        return "pwr";
    case BSP_ICM42688_INIT_STAGE_SIGNAL_RESET:
        return "sig_reset";
    case BSP_ICM42688_INIT_STAGE_READY:
        return "ready";
    default:
        return "unknown";
    }
}

static uint8_t app_control_flash_ok(const APP_Flash_Status *status)
{
    return ((status != NULL) &&
            (status->probe_status == 0) &&
            (status->status1_status == 0) &&
            (status->read_status == 0)) ? 1U : 0U;
}

static const char *app_control_age_text(uint32_t age_ms, char *buffer, uint16_t size)
{
    if ((buffer == NULL) || (size == 0U)) {
        return "?";
    }

    if (age_ms == 0xFFFFFFFFUL) {
        (void)snprintf(buffer, size, "none");
    } else {
        (void)snprintf(buffer, size, "%lu", (unsigned long)age_ms);
    }

    return buffer;
}

static const char *app_control_flash_stage(const APP_Flash_Status *status)
{
    if (status == NULL) {
        return "unknown";
    }

    if (status->probe_status != 0) {
        return "probe";
    }

    if (status->status1_status != 0) {
        return "status";
    }

    if (status->read_status != 0) {
        return "read";
    }

    return "ready";
}

static uint8_t app_control_baro_ok(const APP_Baro_Status *status)
{
    uint8_t id_ok;

    if ((status == NULL) || (status->init_status != 0)) {
        return 0U;
    }

    id_ok = ((status->product_id == BSP_SPL06_ID_VALUE) ||
             (status->split_id == BSP_SPL06_ID_VALUE) ||
             (status->txrx_id == BSP_SPL06_ID_VALUE)) ? 1U : 0U;
    return id_ok;
}

static const char *app_control_baro_stage(const APP_Baro_Status *status)
{
    if (status == NULL) {
        return "unknown";
    }

    if (status->split_status != 0) {
        return "split_id";
    }

    if (status->txrx_status != 0) {
        return "txrx_id";
    }

    if (status->init_status != 0) {
        return "init";
    }

    if ((status->product_id != BSP_SPL06_ID_VALUE) &&
        (status->split_id != BSP_SPL06_ID_VALUE) &&
        (status->txrx_id != BSP_SPL06_ID_VALUE)) {
        return "who_id";
    }

    return "ready";
}

static const char *app_control_aiwb2_state_name(APP_AiWB2_State state)
{
    switch (state) {
    case APP_AIWB2_STATE_START_DELAY:
        return "start_delay";
    case APP_AIWB2_STATE_WAIT_PROBE:
        return "wait_probe";
    case APP_AIWB2_STATE_ESCAPE_BEFORE:
        return "escape_before";
    case APP_AIWB2_STATE_ESCAPE_AFTER:
        return "escape_after";
    case APP_AIWB2_STATE_SEND_COMMAND:
        return "send_command";
    case APP_AIWB2_STATE_WAIT_COMMAND:
        return "wait_command";
    case APP_AIWB2_STATE_WAIT_BOOT_CONNECT:
        return "wait_connect";
    case APP_AIWB2_STATE_WAIT_TRANSPARENT_OK:
        return "wait_transparent_ok";
    case APP_AIWB2_STATE_TRANSPARENT:
        return "transparent";
    case APP_AIWB2_STATE_SOCKET_READY:
        return "socket_ready";
    case APP_AIWB2_STATE_RETRY_DELAY:
        return "retry_delay";
    default:
        return "unknown";
    }
}

static uint8_t app_control_parse_u32_auto(const char *text, uint32_t *value)
{
    char *end_ptr;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }

    parsed = strtoul(text, &end_ptr, 0);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }

    *value = (uint32_t)parsed;
    return 1U;
}

static void app_control_protocol_err(uint32_t id,
                                     const char *mod,
                                     const char *op,
                                     const char *code)
{
    APP_Control_QueueText("ERR id=%lu mod=%s op=%s code=%s\r\n",
                           (unsigned long)id,
                           (mod != NULL) ? mod : "?",
                           (op != NULL) ? op : "?",
                           (code != NULL) ? code : "ERR");
}

static void app_control_req_spl06(uint32_t id, const char *op)
{
    APP_Baro_Snapshot snapshot;

    if (op == NULL) {
        app_control_protocol_err(id, "SPL06", "?", "NO_OP");
        return;
    }

    APP_Baro_ReadSnapshot(&snapshot);

    if (strcmp(op, "STATUS") == 0) {
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=STATUS ok=%u stage=%s init=%ld raw=%ld who=0x%02X exp=0x10\r\n",
                               (unsigned long)id,
                               (unsigned int)app_control_baro_ok(&snapshot.status),
                               app_control_baro_stage(&snapshot.status),
                               (long)snapshot.status.init_status,
                               (long)snapshot.raw_status,
                               (unsigned int)snapshot.id);
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=STATUS split=%ld txrx=%ld sid=0x%02X tid=0x%02X cs=%u miso=%u\r\n",
                               (unsigned long)id,
                               (long)snapshot.status.split_status,
                               (long)snapshot.status.txrx_status,
                               (unsigned int)snapshot.status.split_id,
                               (unsigned int)snapshot.status.txrx_id,
                               (unsigned int)snapshot.status.cs_level,
                               (unsigned int)snapshot.status.miso_level);
        return;
    }

    if ((strcmp(op, "SAMPLE") == 0) || (strcmp(op, "READ") == 0)) {
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=%s ok=%u raw_st=%ld coef_st=%ld scaled=%u press_raw=%ld temp_raw=%ld pressure_pa=%ld temp_cdeg=%ld\r\n",
                               (unsigned long)id,
                               op,
                               (snapshot.raw_status == (int32_t)BSP_SPL06_OK) ? 1U : 0U,
                               (long)snapshot.raw_status,
                               (long)snapshot.coef_status,
                               (unsigned int)snapshot.scaled_valid,
                               (long)snapshot.pressure_raw,
                               (long)snapshot.temperature_raw,
                               (long)snapshot.pressure_pa,
                               (long)snapshot.temperature_cdeg);
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=%s cfg prs=0x%02X tmp=0x%02X meas=0x%02X cfg=0x%02X int=0x%02X fifo=0x%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)snapshot.prs_cfg,
                               (unsigned int)snapshot.tmp_cfg,
                               (unsigned int)snapshot.meas_cfg,
                               (unsigned int)snapshot.cfg_reg,
                               (unsigned int)snapshot.int_sts,
                               (unsigned int)snapshot.fifo_sts);
        APP_Control_QueueText("RSP id=%lu mod=SPL06 op=%s regs=%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)snapshot.raw_regs[0],
                               (unsigned int)snapshot.raw_regs[1],
                               (unsigned int)snapshot.raw_regs[2],
                               (unsigned int)snapshot.raw_regs[3],
                               (unsigned int)snapshot.raw_regs[4],
                               (unsigned int)snapshot.raw_regs[5],
                               (unsigned int)snapshot.raw_regs[6],
                               (unsigned int)snapshot.raw_regs[7],
                               (unsigned int)snapshot.raw_regs[8],
                               (unsigned int)snapshot.raw_regs[9],
                               (unsigned int)snapshot.raw_regs[10],
                               (unsigned int)snapshot.raw_regs[11],
                               (unsigned int)snapshot.raw_regs[12],
                               (unsigned int)snapshot.raw_regs[13]);
        return;
    }

    app_control_protocol_err(id, "SPL06", op, "BAD_OP");
}

static void app_control_req_icm42688(uint32_t id, const char *op)
{
    APP_IMU_Status imu_status;

    if (op == NULL) {
        app_control_protocol_err(id, "ICM42688", "?", "NO_OP");
        return;
    }

    APP_IMU_GetStatus(&imu_status);

    if ((strcmp(op, "STATUS") == 0) || (strcmp(op, "DIAG") == 0)) {
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s ok=%u stage=%s stage_id=%u who=0x%02X exp=0x%02X code=%ld\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)imu_status.initialized,
                               app_control_imu_stage_name(imu_status.init_stage),
                               (unsigned int)imu_status.init_stage,
                               (unsigned int)imu_status.who_am_i,
                               (unsigned int)BSP_ICM42688_WHO_AM_I_VALUE,
                               (long)imu_status.last_error);
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s st=%ld n=%lu ax=%d ay=%d az=%d gx=%ld gy=%ld gz=%ld t=%d\r\n",
                               (unsigned long)id,
                               op,
                               (long)imu_status.last_status,
                               (unsigned long)imu_status.sample_count,
                               (int)imu_status.accel_x_mg,
                               (int)imu_status.accel_y_mg,
                               (int)imu_status.accel_z_mg,
                               (long)imu_status.gyro_x_mdps,
                               (long)imu_status.gyro_y_mdps,
                               (long)imu_status.gyro_z_mdps,
                               (int)imu_status.temperature_cdeg);
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s diag valid=%u m0_tok=0x%02X m0_msb=0x%02X m0_b0=0x%02X m3_tok=0x%02X m3_msb=0x%02X m3_b0=0x%02X best_mode=%u best_hdr=%u\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)imu_status.diag_valid,
                               (unsigned int)imu_status.diag_mode0_tokmas,
                               (unsigned int)imu_status.diag_mode0_msb,
                               (unsigned int)imu_status.diag_mode0_bit0,
                               (unsigned int)imu_status.diag_mode3_tokmas,
                               (unsigned int)imu_status.diag_mode3_msb,
                               (unsigned int)imu_status.diag_mode3_bit0,
                               (unsigned int)imu_status.diag_best_mode,
                               (unsigned int)imu_status.diag_best_header);
        APP_Control_QueueText("RSP id=%lu mod=ICM42688 op=%s burst m0_b0=%02X%02X%02X%02X m3_tok=%02X%02X%02X%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)imu_status.diag_burst_m0_b0_1,
                               (unsigned int)imu_status.diag_burst_m0_b0_2,
                               (unsigned int)imu_status.diag_burst_m0_b0_3,
                               (unsigned int)imu_status.diag_burst_m0_b0_4,
                               (unsigned int)imu_status.diag_burst_m3_tok_1,
                               (unsigned int)imu_status.diag_burst_m3_tok_2,
                               (unsigned int)imu_status.diag_burst_m3_tok_3,
                               (unsigned int)imu_status.diag_burst_m3_tok_4);
        return;
    }

    app_control_protocol_err(id, "ICM42688", op, "BAD_OP");
}

static void app_control_req_m9n(uint32_t id, const char *op)
{
    APP_GPS_Status gps_status;
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms = 0U;
    char age_text[16];

    if (op == NULL) {
        app_control_protocol_err(id, "M9N", "?", "NO_OP");
        return;
    }

    APP_GPS_GetStatus(&gps_status);
    if (gps_status.last_rx_ms != 0U) {
        age_ms = now_ms - gps_status.last_rx_ms;
    } else {
        age_ms = 0xFFFFFFFFUL;
    }

    if ((strcmp(op, "STATUS") == 0) || (strcmp(op, "DIAG") == 0)) {
        APP_Control_QueueText("RSP id=%lu mod=M9N op=%s ok=%u init=%ld fix=%u valid=%u sv=%u age_ms=%s packets=%lu nav=%lu nmea=%lu gga=%lu\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)gps_status.initialized,
                               (long)gps_status.init_status,
                               (unsigned int)gps_status.fix_type,
                               (unsigned int)gps_status.valid_fix,
                               (unsigned int)gps_status.num_sv,
                               app_control_age_text(age_ms, age_text, (uint16_t)sizeof(age_text)),
                               (unsigned long)gps_status.packets,
                               (unsigned long)gps_status.nav_pvt_packets,
                               (unsigned long)gps_status.nmea_sentences,
                               (unsigned long)gps_status.nmea_gga_sentences);
        APP_Control_QueueText("RSP id=%lu mod=M9N op=%s baud=%lu bytes=%lu cksum=%lu nmea_ck=%lu ovf=%lu nmea_ovf=%lu rst=%lu uerr=%lu last_err=0x%lX cfg=%lu\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned long)gps_status.baud_rate,
                               (unsigned long)gps_status.bytes,
                               (unsigned long)gps_status.checksum_errors,
                               (unsigned long)gps_status.nmea_checksum_errors,
                               (unsigned long)gps_status.payload_overflows,
                               (unsigned long)gps_status.nmea_overflows,
                               (unsigned long)gps_status.rx_restarts,
                               (unsigned long)gps_status.uart_errors,
                               (unsigned long)gps_status.last_uart_error,
                               (unsigned long)gps_status.config_writes);
        APP_Control_QueueText("RSP id=%lu mod=M9N op=%s lon=%ld lat=%ld hmsl_mm=%ld hacc_mm=%lu vacc_mm=%lu vn=%ld ve=%ld vd=%ld head_e5=%ld utc=%04u-%02u-%02uT%02u:%02u:%02u\r\n",
                               (unsigned long)id,
                               op,
                               (long)gps_status.lon_deg_e7,
                               (long)gps_status.lat_deg_e7,
                               (long)gps_status.hmsl_mm,
                               (unsigned long)gps_status.hacc_mm,
                               (unsigned long)gps_status.vacc_mm,
                               (long)gps_status.vel_n_mm_s,
                               (long)gps_status.vel_e_mm_s,
                               (long)gps_status.vel_d_mm_s,
                               (long)gps_status.heading_motion_deg_e5,
                               (unsigned int)gps_status.year,
                               (unsigned int)gps_status.month,
                               (unsigned int)gps_status.day,
                               (unsigned int)gps_status.hour,
                               (unsigned int)gps_status.minute,
                               (unsigned int)gps_status.second);
        return;
    }

    app_control_protocol_err(id, "M9N", op, "BAD_OP");
}

static void app_control_req_mag(uint32_t id, const char *op)
{
    APP_MAG_Status mag_status;

    if (op == NULL) {
        app_control_protocol_err(id, "MAG", "?", "NO_OP");
        return;
    }

    APP_MAG_GetStatus(&mag_status);

    if ((strcmp(op, "STATUS") == 0) || (strcmp(op, "DIAG") == 0)) {
        APP_Control_QueueText("RSP id=%lu mod=MAG op=%s ok=%u init=%ld st=%ld type=%s addr=0x%02X who=0x%02X n=%lu raw=%d,%d,%d mgauss=%ld,%ld,%ld\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)mag_status.initialized,
                               (long)mag_status.init_status,
                               (long)mag_status.last_status,
                               APP_MAG_GetTypeName(mag_status.type),
                               (unsigned int)mag_status.address,
                               (unsigned int)mag_status.who_am_i,
                               (unsigned long)mag_status.sample_count,
                               (int)mag_status.raw_x,
                               (int)mag_status.raw_y,
                               (int)mag_status.raw_z,
                               (long)mag_status.x_mgauss,
                               (long)mag_status.y_mgauss,
                               (long)mag_status.z_mgauss);
        APP_Control_QueueText("RSP id=%lu mod=MAG op=%s probe ist=%u hmc=%u qmc=%u hmc_id=%02X%02X%02X\r\n",
                               (unsigned long)id,
                               op,
                               (unsigned int)mag_status.detected_ist8310,
                               (unsigned int)mag_status.detected_hmc5883,
                               (unsigned int)mag_status.detected_qmc5883,
                               (unsigned int)mag_status.hmc_id_a,
                               (unsigned int)mag_status.hmc_id_b,
                               (unsigned int)mag_status.hmc_id_c);
        return;
    }

    app_control_protocol_err(id, "MAG", op, "BAD_OP");
}

void app_control_handle_req(char **tokens, uint32_t count)
{
    const char *id_text = app_control_token_value(tokens, count, "id");
    const char *mod = app_control_token_value(tokens, count, "mod");
    const char *op = app_control_token_value(tokens, count, "op");
    uint32_t id = 0U;

    if ((id_text == NULL) || (app_control_parse_u32_auto(id_text, &id) == 0U)) {
        app_control_protocol_err(0U, (mod != NULL) ? mod : "?", (op != NULL) ? op : "?", "BAD_ID");
        return;
    }

    if (mod == NULL) {
        app_control_protocol_err(id, "?", (op != NULL) ? op : "?", "NO_MOD");
        return;
    }

    if (strcmp(mod, "SPL06") == 0) {
        app_control_req_spl06(id, op);
        return;
    }

    if (strcmp(mod, "ICM42688") == 0) {
        app_control_req_icm42688(id, op);
        return;
    }

    if (strcmp(mod, "M9N") == 0) {
        app_control_req_m9n(id, op);
        return;
    }

    if (strcmp(mod, "MAG") == 0) {
        app_control_req_mag(id, op);
        return;
    }

    if (strcmp(mod, "WIFI") == 0) {
        if (op == NULL) {
            app_control_protocol_err(id, "WIFI", "?", "NO_OP");
            return;
        }
        if (strcmp(op, "STATUS") == 0) {
            APP_Control_QueueText("RSP id=%lu mod=WIFI op=STATUS en=%u pin=PC6 last=%u writes=%lu state=%s transparent=%u retry=%lu socket=%ld cycling=%u wait_ms=%lu prov=%u cmd=%lu/%lu\r\n",
                                   (unsigned long)id,
                                   (unsigned int)BSP_AiWB2_IsEnabled(),
                                   (unsigned int)BSP_AiWB2_GetLastWrittenState(),
                                   (unsigned long)BSP_AiWB2_GetWriteCount(),
                                   app_control_aiwb2_state_name(APP_AiWB2_GetState()),
                                   (unsigned int)APP_AiWB2_IsTransparent(),
                                   (unsigned long)APP_AiWB2_GetRetryCount(),
                                   (long)APP_AiWB2_GetLastSocketError(),
                                   (unsigned int)APP_AiWB2_IsPowerRecycleActive(),
                                   (unsigned long)APP_AiWB2_GetDeadlineRemainingMs(),
                                   (unsigned int)APP_AiWB2_IsProvisionActive(),
                                   (unsigned long)APP_AiWB2_GetCommandIndex(),
                                   (unsigned long)APP_AiWB2_GetCommandCount());
            return;
        }
        app_control_protocol_err(id, "WIFI", op, "BAD_OP");
        return;
    }

    app_control_protocol_err(id, mod, (op != NULL) ? op : "?", "BAD_MOD");
}

const char *app_control_internal_imu_stage_name(uint8_t stage)
{
    return app_control_imu_stage_name(stage);
}

uint8_t app_control_internal_flash_ok(const void *status)
{
    return app_control_flash_ok((const APP_Flash_Status *)status);
}

const char *app_control_internal_flash_stage(const void *status)
{
    return app_control_flash_stage((const APP_Flash_Status *)status);
}

uint8_t app_control_internal_baro_ok(const void *status)
{
    return app_control_baro_ok((const APP_Baro_Status *)status);
}

const char *app_control_internal_baro_stage(const void *status)
{
    return app_control_baro_stage((const APP_Baro_Status *)status);
}

const char *app_control_internal_aiwb2_state_name(uint32_t state)
{
    return app_control_aiwb2_state_name((APP_AiWB2_State)state);
}

uint8_t app_control_internal_parse_u32_auto(const char *text, uint32_t *value)
{
    return app_control_parse_u32_auto(text, value);
}
