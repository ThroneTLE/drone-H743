#ifndef APP_USB_CDC_H
#define APP_USB_CDC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Maximum single-transfer payload. APP_USB_CDC_Write() rejects any length above
 * this, so callers that build fixed-size frames should size-check against it.
 */
#define APP_USB_CDC_TX_SIZE 1536U

void APP_USB_CDC_Init(void);
void APP_USB_CDC_Task_Step(void);
void APP_USB_CDC_SetConfigured(uint8_t configured);
uint8_t APP_USB_CDC_IsReady(void);
void APP_USB_CDC_OnReceive(const uint8_t *data, uint32_t length);
void APP_USB_CDC_OnTransmitComplete(void);
uint8_t APP_USB_CDC_Write(const uint8_t *data,
                          uint16_t length,
                          uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_USB_CDC_H */
