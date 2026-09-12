#include "bsp_usb_cdc.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"

/* CubeMX 在 USB_DEVICE/App/usb_device.c 里定义这个句柄。 */
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t BSP_UsbCdc_Transmit(uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return 0U;
    }

    return (CDC_Transmit_FS(data, length) == USBD_OK) ? 1U : 0U;
}

void BSP_UsbCdc_Teardown(void)
{
    (void)USBD_Stop(&hUsbDeviceFS);
    (void)USBD_DeInit(&hUsbDeviceFS);
    __DSB();
}
