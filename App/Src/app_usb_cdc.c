#include "app_usb_cdc.h"

#include "app_control.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include "cmsis_os2.h"

#include <string.h>

#define APP_USB_CDC_RX_RING_SIZE 1024U
#define APP_USB_CDC_LINE_SIZE    128U
/* APP_USB_CDC_TX_SIZE now lives in app_usb_cdc.h so callers can size-check. */

static volatile uint8_t app_usb_cdc_configured;
static volatile uint8_t app_usb_cdc_tx_in_flight;
static volatile uint8_t app_usb_cdc_tx_locked;
static volatile uint16_t app_usb_cdc_rx_head;
static volatile uint16_t app_usb_cdc_rx_tail;
static volatile uint32_t app_usb_cdc_rx_bytes;
static volatile uint32_t app_usb_cdc_rx_lines;
static volatile uint32_t app_usb_cdc_rx_dropped;
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t app_usb_cdc_rx_ring[APP_USB_CDC_RX_RING_SIZE];
static char app_usb_cdc_line[APP_USB_CDC_LINE_SIZE];
static uint16_t app_usb_cdc_line_used;
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t app_usb_cdc_tx_buffer[APP_USB_CDC_TX_SIZE];

static uint8_t app_usb_cdc_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static void app_usb_cdc_wait_1ms(void)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(1U);
    } else {
        HAL_Delay(1U);
    }
}

static uint8_t app_usb_cdc_take_tx_lock(uint32_t timeout_ms)
{
    uint32_t deadline_ms = HAL_GetTick() + timeout_ms;

    do {
        uint8_t taken = 0U;

        __disable_irq();
        if (app_usb_cdc_tx_locked == 0U) {
            app_usb_cdc_tx_locked = 1U;
            taken = 1U;
        }
        __enable_irq();

        if (taken != 0U) {
            return 1U;
        }

        if (timeout_ms == 0U) {
            return 0U;
        }
        app_usb_cdc_wait_1ms();
    } while (app_usb_cdc_time_reached(HAL_GetTick(), deadline_ms) == 0U);

    return 0U;
}

static void app_usb_cdc_give_tx_lock(void)
{
    __disable_irq();
    app_usb_cdc_tx_locked = 0U;
    __enable_irq();
}

static uint8_t app_usb_cdc_rx_pop(uint8_t *byte)
{
    uint16_t tail;

    if (byte == NULL) {
        return 0U;
    }

    tail = app_usb_cdc_rx_tail;
    if (tail == app_usb_cdc_rx_head) {
        return 0U;
    }

    *byte = app_usb_cdc_rx_ring[tail];
    tail++;
    if (tail >= APP_USB_CDC_RX_RING_SIZE) {
        tail = 0U;
    }
    app_usb_cdc_rx_tail = tail;
    return 1U;
}

static void app_usb_cdc_process_line(void)
{
    if (app_usb_cdc_line_used == 0U) {
        return;
    }

    app_usb_cdc_line[app_usb_cdc_line_used] = '\0';
    APP_Control_Init();
    APP_Control_ProcessLine(app_usb_cdc_line);
    app_usb_cdc_line_used = 0U;
    app_usb_cdc_rx_lines++;
}

void APP_USB_CDC_Init(void)
{
    app_usb_cdc_rx_head = 0U;
    app_usb_cdc_rx_tail = 0U;
    app_usb_cdc_line_used = 0U;
    app_usb_cdc_rx_bytes = 0U;
    app_usb_cdc_rx_lines = 0U;
    app_usb_cdc_rx_dropped = 0U;
    app_usb_cdc_tx_in_flight = 0U;
    app_usb_cdc_tx_locked = 0U;
}

void APP_USB_CDC_Task_Step(void)
{
    uint8_t byte;
    uint32_t processed = 0U;

    while ((processed < 256U) && (app_usb_cdc_rx_pop(&byte) != 0U)) {
        processed++;

        if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n')) {
            app_usb_cdc_process_line();
            continue;
        }

        if ((byte < 0x20U) || (byte > 0x7EU)) {
            continue;
        }

        if (app_usb_cdc_line_used < (APP_USB_CDC_LINE_SIZE - 1U)) {
            app_usb_cdc_line[app_usb_cdc_line_used++] = (char)byte;
        } else {
            app_usb_cdc_line_used = 0U;
            app_usb_cdc_rx_dropped++;
        }
    }
}

void APP_USB_CDC_SetConfigured(uint8_t configured)
{
    app_usb_cdc_configured = (configured != 0U) ? 1U : 0U;
    if (configured == 0U) {
        app_usb_cdc_tx_in_flight = 0U;
    }
}

uint8_t APP_USB_CDC_IsReady(void)
{
    return (app_usb_cdc_configured != 0U) ? 1U : 0U;
}

void APP_USB_CDC_OnReceive(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if ((data == NULL) || (length == 0U)) {
        return;
    }

    for (index = 0U; index < length; ++index) {
        uint16_t head = app_usb_cdc_rx_head;
        uint16_t next = (uint16_t)(head + 1U);

        if (next >= APP_USB_CDC_RX_RING_SIZE) {
            next = 0U;
        }
        if (next == app_usb_cdc_rx_tail) {
            app_usb_cdc_rx_dropped++;
            continue;
        }

        app_usb_cdc_rx_ring[head] = data[index];
        app_usb_cdc_rx_head = next;
        app_usb_cdc_rx_bytes++;
    }
}

void APP_USB_CDC_OnTransmitComplete(void)
{
    app_usb_cdc_tx_in_flight = 0U;
}

uint8_t APP_USB_CDC_Write(const uint8_t *data,
                          uint16_t length,
                          uint32_t timeout_ms)
{
    uint32_t deadline_ms;
    uint8_t result;

    if ((data == NULL) || (length == 0U) || (length > APP_USB_CDC_TX_SIZE) ||
        (APP_USB_CDC_IsReady() == 0U)) {
        return 0U;
    }

    if (app_usb_cdc_take_tx_lock(timeout_ms) == 0U) {
        return 0U;
    }

    deadline_ms = HAL_GetTick() + timeout_ms;
    while ((app_usb_cdc_tx_in_flight != 0U) &&
           (app_usb_cdc_time_reached(HAL_GetTick(), deadline_ms) == 0U)) {
        app_usb_cdc_wait_1ms();
    }

    if (app_usb_cdc_tx_in_flight != 0U) {
        app_usb_cdc_give_tx_lock();
        return 0U;
    }

    memcpy(app_usb_cdc_tx_buffer, data, length);
    app_usb_cdc_tx_in_flight = 1U;
    result = CDC_Transmit_FS(app_usb_cdc_tx_buffer, length);
    if (result != USBD_OK) {
        app_usb_cdc_tx_in_flight = 0U;
        app_usb_cdc_give_tx_lock();
        return 0U;
    }

    deadline_ms = HAL_GetTick() + timeout_ms;
    while ((app_usb_cdc_tx_in_flight != 0U) &&
           (app_usb_cdc_time_reached(HAL_GetTick(), deadline_ms) == 0U)) {
        app_usb_cdc_wait_1ms();
    }

    result = (app_usb_cdc_tx_in_flight == 0U) ? 1U : 0U;
    app_usb_cdc_give_tx_lock();
    return result;
}
