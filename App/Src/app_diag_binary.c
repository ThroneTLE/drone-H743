/* Low-rate command replies; explicit byte lengths and bounded transport waits. */
#include "app_diag_binary.h"
#include "app_proto.h"
#include "app_control_internal.h"
#include "app_maint_uart.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "app_usb_cdc.h"
uint8_t APP_Diag_SendBinary(uint16_t function,const uint8_t *payload,uint16_t length)
{
    APP_UART_TxMessage message;uint16_t size;
    if (!APP_Proto_BuildFrame(APP_PROTO_DIR_FROM_FC,function,payload,length,
                             (uint8_t *)message.text,sizeof(message.text),&size)) { return 0; }
    if (app_control_internal_maint_output_active()) {
        return APP_MaintUART_WritePacket((const uint8_t *)message.text,size);
    }
    uint8_t usb=0,uart=0;
    if (APP_USB_CDC_IsReady()) { usb=APP_USB_CDC_Write((const uint8_t *)message.text,size,10U); }
    message.function=APP_UART_TX_FUNCTION_VOFA_SOCKET;message.length=size;
    if (uartTxQueueHandle && osMessageQueuePut(uartTxQueueHandle,&message,0U,0U)==osOK) {
        APP_UART_NotifyTxPending();uart=1;
    }
    return usb || uart;
}
