#include "app_flash.h"

#include "app_messages.h"
#include "app_proto.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "bsp_flash.h"

#include <stdarg.h>
#include <stdio.h>

#define APP_FLASH_REPORT_READ_LENGTH 16U

static uint8_t flash_report_done;
static APP_Flash_Status flash_status;

static void app_flash_queue_text(const char *format, ...)
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

void APP_Flash_ReportStartup(void)
{
    BSP_GD25Q32_Status probe_status;
    BSP_GD25Q32_Status status_status;
    BSP_GD25Q32_Status read_status;
    BSP_GD25Q32_JedecId jedec_id = {0};
    uint8_t status1 = 0U;
    uint8_t data[APP_FLASH_REPORT_READ_LENGTH];

    if (flash_report_done != 0U) {
        return;
    }

    flash_report_done = 1U;
    flash_status.report_done = 1U;

    probe_status = BSP_FLASH_ProbeJedecId(&jedec_id);
    flash_status.probe_status = (int32_t)probe_status;
    flash_status.manufacturer_id = jedec_id.manufacturer_id;
    flash_status.memory_type = jedec_id.memory_type;
    flash_status.capacity_id = jedec_id.capacity_id;
    if (probe_status != BSP_GD25Q32_OK) {
        app_flash_queue_text("FLASH probe st=%d mid=0x%02X type=0x%02X cap=0x%02X\r\n",
                             (int)probe_status,
                             (unsigned int)jedec_id.manufacturer_id,
                             (unsigned int)jedec_id.memory_type,
                             (unsigned int)jedec_id.capacity_id);
        return;
    }

    status_status = BSP_FLASH_ReadStatus1(&status1);
    flash_status.status1_status = (int32_t)status_status;
    flash_status.status1 = status1;
    if (status_status != BSP_GD25Q32_OK) {
        app_flash_queue_text("FLASH status st=%d mid=0x%02X type=0x%02X cap=0x%02X\r\n",
                             (int)status_status,
                             (unsigned int)jedec_id.manufacturer_id,
                             (unsigned int)jedec_id.memory_type,
                             (unsigned int)jedec_id.capacity_id);
        return;
    }

    app_flash_queue_text("FLASH ok mid=0x%02X type=0x%02X cap=0x%02X sr1=0x%02X\r\n",
                         (unsigned int)jedec_id.manufacturer_id,
                         (unsigned int)jedec_id.memory_type,
                         (unsigned int)jedec_id.capacity_id,
                         (unsigned int)status1);

    read_status = BSP_FLASH_ReadData(0U, data, sizeof(data));
    flash_status.read_status = (int32_t)read_status;
    if (read_status != BSP_GD25Q32_OK) {
        app_flash_queue_text("FLASH read st=%d addr=0x000000\r\n", (int)read_status);
        return;
    }

    app_flash_queue_text("FLASH [0x000000] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                         (unsigned int)data[0],
                         (unsigned int)data[1],
                         (unsigned int)data[2],
                         (unsigned int)data[3],
                         (unsigned int)data[4],
                         (unsigned int)data[5],
                         (unsigned int)data[6],
                         (unsigned int)data[7],
                         (unsigned int)data[8],
                         (unsigned int)data[9],
                         (unsigned int)data[10],
                         (unsigned int)data[11],
                         (unsigned int)data[12],
                         (unsigned int)data[13],
                         (unsigned int)data[14],
                         (unsigned int)data[15]);
}

void APP_Flash_GetStatus(APP_Flash_Status *status)
{
    if (status == 0) {
        return;
    }

    *status = flash_status;
}
