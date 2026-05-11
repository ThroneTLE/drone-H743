#include "app_message.h"

#include "app_baro.h"
#include "app_flash.h"
#include "app_messages.h"
#include "app_proto.h"
#include "app_tasks.h"
#include "app_uart.h"

#include <stdio.h>

#define APP_MESSAGE_STARTUP_REPORT_ENABLED 0U
#define APP_MESSAGE_IMU_STREAM_ENABLED     0U

#if (APP_MESSAGE_IMU_STREAM_ENABLED != 0U)
static void APP_Message_Format(const APP_IMU_SampleMessage *sample,
                               APP_UART_TxMessage *tx_message)
{
    int written = 0;

    if ((sample == 0) || (tx_message == 0)) {
        return;
    }

    tx_message->function = APP_PROTO_MSG_TEXT_LINE;
    written = snprintf(tx_message->text,
                       sizeof(tx_message->text),
                       "n=%lu,id=0x%02X,ax=%d,ay=%d,az=%d,gx=%ld,gy=%ld,gz=%ld,t=%d,roll=%d,pitch=%d,yaw=%d\r\n",
                       (unsigned long)sample->sample_count,
                       (unsigned int)sample->who_am_i,
                       (int)sample->accel_x_mg,
                       (int)sample->accel_y_mg,
                       (int)sample->accel_z_mg,
                       (long)sample->gyro_x_mdps,
                       (long)sample->gyro_y_mdps,
                       (long)sample->gyro_z_mdps,
                       (int)sample->temperature_cdeg,
                       (int)sample->roll_cdeg,
                       (int)sample->pitch_cdeg,
                       (int)sample->yaw_cdeg);

    if (written < 0) {
        tx_message->length = 0U;
        tx_message->text[0] = '\0';
        return;
    }

    if ((uint32_t)written >= (uint32_t)sizeof(tx_message->text)) {
        tx_message->length = (uint16_t)(sizeof(tx_message->text) - 1U);
        tx_message->text[tx_message->length] = '\0';
        return;
    }

    tx_message->length = (uint16_t)written;
}
#endif

void APP_Message_Task_Init(void)
{
#if (APP_MESSAGE_STARTUP_REPORT_ENABLED != 0U)
    APP_Flash_ReportStartup();
    APP_Baro_ReportStartup();
#endif
}

void APP_Message_Task_Step(void)
{
    APP_IMU_SampleMessage sample;
#if (APP_MESSAGE_IMU_STREAM_ENABLED != 0U)
    APP_UART_TxMessage    tx_message;
#endif

    if ((imuSampleQueueHandle == 0) || (uartTxQueueHandle == 0)) {
        osDelay(10U);
        return;
    }

    if (osMessageQueueGet(imuSampleQueueHandle, &sample, 0U, osWaitForever) != osOK) {
        return;
    }

#if (APP_MESSAGE_IMU_STREAM_ENABLED == 0U)
    return;
#else
    APP_Message_Format(&sample, &tx_message);
    if (tx_message.length == 0U) {
        return;
    }

    if (osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U) != osOK) {
        APP_UART_TxMessage dropped;

        (void)osMessageQueueGet(uartTxQueueHandle, &dropped, 0U, 0U);
        (void)osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U);
    }
    APP_UART_NotifyTxPending();
#endif
}
