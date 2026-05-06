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

static uint8_t baro_report_done;
static APP_Baro_Status baro_status;

static int32_t app_baro_make_signed24(uint8_t msb, uint8_t mid, uint8_t lsb)
{
    int32_t value = ((int32_t)msb << 16) | ((int32_t)mid << 8) | (int32_t)lsb;

    if ((value & 0x00800000L) != 0L) {
        value |= (int32_t)0xFF000000L;
    }

    return value;
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
}
