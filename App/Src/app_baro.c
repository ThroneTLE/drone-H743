#include "app_baro.h"

#include "app_messages.h"
#include "app_proto.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "bsp_baro.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_BARO_BMP280_ID_REG 0xD0U
#define APP_BARO_SPL06_RAW_REG 0x00U
#define APP_BARO_SPL06_RAW_LEN 14U
#define APP_BARO_SPL06_COEF_REG 0x10U
#define APP_BARO_SPL06_COEF_LEN 18U
#define APP_BARO_SPL06_COEF_SRCE_REG 0x28U

static uint8_t baro_report_done;
static APP_Baro_Status baro_status;

static int32_t app_baro_sign_extend(uint32_t value, uint8_t bits)
{
    uint32_t sign_bit = 1UL << (bits - 1U);
    uint32_t mask = (1UL << bits) - 1UL;

    value &= mask;
    if ((value & sign_bit) != 0UL) {
        value |= ~mask;
    }

    return (int32_t)value;
}

static int32_t app_baro_make_signed24(uint8_t msb, uint8_t mid, uint8_t lsb)
{
    int32_t value = ((int32_t)msb << 16) | ((int32_t)mid << 8) | (int32_t)lsb;

    if ((value & 0x00800000L) != 0L) {
        value |= (int32_t)0xFF000000L;
    }

    return value;
}

static int32_t app_baro_scale_factor(uint8_t cfg)
{
    switch (cfg & 0x0FU) {
    case 0x00U:
        return 524288L;
    case 0x01U:
        return 1572864L;
    case 0x02U:
        return 3670016L;
    case 0x03U:
        return 7864320L;
    case 0x04U:
        return 253952L;
    case 0x05U:
        return 516096L;
    case 0x06U:
        return 1040384L;
    case 0x07U:
        return 2088960L;
    default:
        return 524288L;
    }
}

static void app_baro_decode_coefficients(APP_Baro_Snapshot *snapshot)
{
    const uint8_t *c = snapshot->coef_regs;

    snapshot->c0 = (int16_t)app_baro_sign_extend((((uint32_t)c[0] << 4) |
                                                  ((uint32_t)c[1] >> 4)),
                                                 12U);
    snapshot->c1 = (int16_t)app_baro_sign_extend(((((uint32_t)c[1] & 0x0FU) << 8) |
                                                  (uint32_t)c[2]),
                                                 12U);
    snapshot->c00 = app_baro_sign_extend((((uint32_t)c[3] << 12) |
                                          ((uint32_t)c[4] << 4) |
                                          ((uint32_t)c[5] >> 4)),
                                         20U);
    snapshot->c10 = app_baro_sign_extend(((((uint32_t)c[5] & 0x0FU) << 16) |
                                          ((uint32_t)c[6] << 8) |
                                          (uint32_t)c[7]),
                                         20U);
    snapshot->c01 = (int16_t)(((uint16_t)c[8] << 8) | (uint16_t)c[9]);
    snapshot->c11 = (int16_t)(((uint16_t)c[10] << 8) | (uint16_t)c[11]);
    snapshot->c20 = (int16_t)(((uint16_t)c[12] << 8) | (uint16_t)c[13]);
    snapshot->c21 = (int16_t)(((uint16_t)c[14] << 8) | (uint16_t)c[15]);
    snapshot->c30 = (int16_t)(((uint16_t)c[16] << 8) | (uint16_t)c[17]);
}

static void app_baro_compute_scaled(APP_Baro_Snapshot *snapshot)
{
    double k_p;
    double k_t;
    double p_raw_sc;
    double t_raw_sc;
    double pressure;
    double temperature;

    if ((snapshot->raw_status != (int32_t)BSP_SPL06_OK) ||
        (snapshot->coef_status != (int32_t)BSP_SPL06_OK)) {
        return;
    }

    app_baro_decode_coefficients(snapshot);

    k_p = (double)app_baro_scale_factor(snapshot->prs_cfg);
    k_t = (double)app_baro_scale_factor(snapshot->tmp_cfg);
    p_raw_sc = (double)snapshot->pressure_raw / k_p;
    t_raw_sc = (double)snapshot->temperature_raw / k_t;

    temperature = ((double)snapshot->c0 * 0.5) + ((double)snapshot->c1 * t_raw_sc);
    pressure = (double)snapshot->c00 +
               (p_raw_sc * ((double)snapshot->c10 +
                            (p_raw_sc * ((double)snapshot->c20 +
                                         (p_raw_sc * (double)snapshot->c30))))) +
               (t_raw_sc * (double)snapshot->c01) +
               (t_raw_sc * p_raw_sc * ((double)snapshot->c11 +
                                       (p_raw_sc * (double)snapshot->c21)));

    snapshot->temperature_cdeg = (int32_t)((temperature * 100.0) +
                                          ((temperature >= 0.0) ? 0.5 : -0.5));
    snapshot->pressure_pa = (int32_t)(pressure + ((pressure >= 0.0) ? 0.5 : -0.5));
    snapshot->scaled_valid = 1U;
}

static void app_baro_queue_text(const char *format, ...)
{
    APP_UART_TxMessage tx_message;
    APP_UART_TxMessage dropped;
    int written;
    va_list args;

    if ((uartTxQueueHandle == 0) || (format == NULL)) {
        return;
    }

    tx_message.function = APP_PROTO_MSG_TEXT_LINE;
    tx_message.length = 0U;
    tx_message.text[0] = '\0';

    va_start(args, format);
    written = vsnprintf(tx_message.text, sizeof(tx_message.text), format, args);
    va_end(args);

    if (written < 0) {
        return;
    }

    if ((uint32_t)written >= (uint32_t)sizeof(tx_message.text)) {
        tx_message.length = (uint16_t)(sizeof(tx_message.text) - 1U);
        tx_message.text[tx_message.length] = '\0';
    } else {
        tx_message.length = (uint16_t)written;
    }

    if (osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U) != osOK) {
        (void)osMessageQueueGet(uartTxQueueHandle, &dropped, 0U, 0U);
        (void)osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U);
    }
    APP_UART_NotifyTxPending();
}

void APP_Baro_ReportStartup(void)
{
    BSP_SPL06_Status status;
    BSP_SPL06_Status split_status;
    BSP_SPL06_Status txrx_status;
    uint8_t product_id = 0U;
    uint8_t split_id = 0U;
    uint8_t txrx_id = 0U;
    uint8_t bmp280_id = 0U;
    uint8_t cs_level = 0U;
    uint8_t miso_level = 0U;
    const BSP_SPL06_Device *dev;

    if (baro_report_done != 0U) {
        return;
    }

    baro_report_done = 1U;
    baro_status.report_done = 1U;

    BSP_BARO_DebugReadLevels(&cs_level, &miso_level);
    split_status = BSP_BARO_ProbeId(&split_id);
    txrx_status = BSP_BARO_ProbeIdTxRx(&txrx_id);
    (void)BSP_BARO_ReadRawRegister(APP_BARO_BMP280_ID_REG, &bmp280_id);
    baro_status.cs_level = cs_level;
    baro_status.miso_level = miso_level;
    baro_status.split_status = (int32_t)split_status;
    baro_status.txrx_status = (int32_t)txrx_status;
    baro_status.split_id = split_id;
    baro_status.txrx_id = txrx_id;
    baro_status.bmp280_id = bmp280_id;

    app_baro_queue_text("BARO dbg cs=%u miso=%u split(st=%d id=0x%02X) txrx(st=%d id=0x%02X) bmp=0x%02X\r\n",
                        (unsigned int)cs_level,
                        (unsigned int)miso_level,
                        (int)split_status,
                        (unsigned int)split_id,
                        (int)txrx_status,
                        (unsigned int)txrx_id,
                        (unsigned int)bmp280_id);

    status = BSP_BARO_Init();
    dev = BSP_BARO_GetDevice();
    if (dev != NULL) {
        product_id = dev->product_id;
    }
    baro_status.init_status = (int32_t)status;
    baro_status.product_id = product_id;

    if (status != BSP_SPL06_OK) {
        app_baro_queue_text("BARO probe st=%d id=0x%02X\r\n",
                            (int)status,
                            (unsigned int)product_id);
        return;
    }

    app_baro_queue_text("BARO ok id=0x%02X\r\n", (unsigned int)product_id);
}

void APP_Baro_GetStatus(APP_Baro_Status *status)
{
    if (status == 0) {
        return;
    }

    *status = baro_status;
}

void APP_Baro_ReadSnapshot(APP_Baro_Snapshot *snapshot)
{
    if (snapshot == 0) {
        return;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->status = baro_status;
    snapshot->raw_status = (int32_t)BSP_BARO_ReadRawRegisters(APP_BARO_SPL06_RAW_REG,
                                                              snapshot->raw_regs,
                                                              APP_BARO_SPL06_RAW_LEN);
    snapshot->coef_status = (int32_t)BSP_BARO_ReadRawRegisters(APP_BARO_SPL06_COEF_REG,
                                                               snapshot->coef_regs,
                                                               APP_BARO_SPL06_COEF_LEN);
    if (snapshot->raw_status == (int32_t)BSP_SPL06_OK) {
        snapshot->pressure_raw = app_baro_make_signed24(snapshot->raw_regs[0],
                                                        snapshot->raw_regs[1],
                                                        snapshot->raw_regs[2]);
        snapshot->temperature_raw = app_baro_make_signed24(snapshot->raw_regs[3],
                                                           snapshot->raw_regs[4],
                                                           snapshot->raw_regs[5]);
        snapshot->prs_cfg = snapshot->raw_regs[6];
        snapshot->tmp_cfg = snapshot->raw_regs[7];
        snapshot->meas_cfg = snapshot->raw_regs[8];
        snapshot->cfg_reg = snapshot->raw_regs[9];
        snapshot->int_sts = snapshot->raw_regs[10];
        snapshot->fifo_sts = snapshot->raw_regs[11];
        snapshot->id = snapshot->raw_regs[13];
    }
    (void)BSP_BARO_ReadRawRegister(APP_BARO_SPL06_COEF_SRCE_REG,
                                   &snapshot->coef_srce);
    app_baro_compute_scaled(snapshot);
}
