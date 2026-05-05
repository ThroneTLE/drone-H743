#include "app_baro.h"

#include "app_messages.h"
#include "app_tasks.h"
#include "bsp_baro.h"

#include <stdarg.h>
#include <stdio.h>

#define APP_BARO_BMP280_ID_REG 0xD0U

static uint8_t baro_report_done;

static void app_baro_queue_text(const char *format, ...)
{
    APP_UART_TxMessage tx_message;
    APP_UART_TxMessage dropped;
    int written;
    va_list args;

    if ((uartTxQueueHandle == 0) || (format == NULL)) {
        return;
    }

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

    BSP_BARO_DebugReadLevels(&cs_level, &miso_level);
    split_status = BSP_BARO_ProbeId(&split_id);
    txrx_status = BSP_BARO_ProbeIdTxRx(&txrx_id);
    (void)BSP_BARO_ReadRawRegister(APP_BARO_BMP280_ID_REG, &bmp280_id);

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

    if (status != BSP_SPL06_OK) {
        app_baro_queue_text("BARO probe st=%d id=0x%02X\r\n",
                            (int)status,
                            (unsigned int)product_id);
        return;
    }

    app_baro_queue_text("BARO ok id=0x%02X\r\n", (unsigned int)product_id);
}
