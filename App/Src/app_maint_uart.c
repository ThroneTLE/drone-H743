#include "app_maint_uart.h"

#include "app_aiwb2.h"
#include "app_control.h"
#include "app_gps.h"
#include "app_mag.h"
#include "bsp_uart.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_MAINT_UART_LINE_SIZE 160U
#define APP_MAINT_UART_RING_SIZE 256U
#define APP_MAINT_UART_IDLE_LINE_MS 60U
#define APP_MAINT_UART_BOOT_TEXT_ENABLED 1U
#define APP_MAINT_UART_GPS_STATUS_PERIOD_MS 1000U

static uint8_t maint_rx_byte;
static uint8_t maint_rx_ring[APP_MAINT_UART_RING_SIZE];
static char maint_rx_line[APP_MAINT_UART_LINE_SIZE];
static uint16_t maint_rx_used;
static volatile uint16_t maint_rx_head;
static uint16_t maint_rx_tail;
static uint32_t maint_last_rx_ms;
static volatile uint8_t maint_rx_error;
static volatile uint8_t maint_rx_overflow;
static uint8_t maint_control_ready;
static uint32_t maint_last_gps_status_ms;

static void maint_start_rx(void)
{
    if (HAL_UART_Receive_IT(&huart8, &maint_rx_byte, 1U) != HAL_OK) {
        maint_rx_error = 1U;
    }
}

static void maint_write_literal(const char *text)
{
    if (text == 0) {
        return;
    }

    (void)BSP_UART_Transmit_UART8((const uint8_t *)text,
                                  (uint16_t)strlen(text),
                                  100U);
}

static char *maint_normalize_line(char *line)
{
    char *start = line;
    char *end;

    while ((*start != '\0') && ((uint8_t)*start <= (uint8_t)' ')) {
        ++start;
    }

    end = start + strlen(start);
    while ((end > start) && ((uint8_t)*(end - 1) <= (uint8_t)' ')) {
        --end;
    }
    *end = '\0';

    return start;
}

static void maint_ensure_control_ready(void)
{
    if (maint_control_ready != 0U) {
        return;
    }

    if (APP_AiWB2_IsTransparent() == 0U) {
        APP_AiWB2_AssumeTransparent();
    }
    APP_Control_Init();
    maint_control_ready = 1U;
}

static void maint_handle_line(char *line)
{
    char *normalized = maint_normalize_line(line);

    if (*normalized == '\0') {
        return;
    }

    maint_ensure_control_ready();
    APP_Control_ProcessMaintLine(normalized);
}

static void maint_process_byte(uint8_t byte)
{
    maint_last_rx_ms = HAL_GetTick();

    if ((byte == '\r') || (byte == '\n')) {
        if (maint_rx_used > 0U) {
            maint_rx_line[maint_rx_used] = '\0';
            maint_handle_line(maint_rx_line);
            maint_rx_used = 0U;
        }
        return;
    }

    if (maint_rx_used < (APP_MAINT_UART_LINE_SIZE - 1U)) {
        maint_rx_line[maint_rx_used++] = (char)byte;
    } else {
        maint_rx_used = 0U;
        maint_write_literal("ERR maint line overflow\r\n");
    }
}

static void maint_flush_idle_line(void)
{
    uint32_t now_ms;

    if (maint_rx_used == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - maint_last_rx_ms) < APP_MAINT_UART_IDLE_LINE_MS) {
        return;
    }

    maint_rx_line[maint_rx_used] = '\0';
    maint_handle_line(maint_rx_line);
    maint_rx_used = 0U;
}

static void maint_report_gps_usart2_status(void)
{
    APP_GPS_Status gps_status;
    APP_MAG_Status mag_status;
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms = 0xFFFFFFFFUL;
    char age_text[16];

    if ((now_ms - maint_last_gps_status_ms) < APP_MAINT_UART_GPS_STATUS_PERIOD_MS) {
        return;
    }
    maint_last_gps_status_ms = now_ms;

    APP_GPS_GetStatus(&gps_status);
    APP_MAG_GetStatus(&mag_status);
    if (gps_status.last_rx_ms != 0U) {
        age_ms = now_ms - gps_status.last_rx_ms;
    }
    if (age_ms == 0xFFFFFFFFUL) {
        (void)snprintf(age_text, sizeof(age_text), "none");
    } else {
        (void)snprintf(age_text, sizeof(age_text), "%lu", (unsigned long)age_ms);
    }

    APP_MaintUART_WriteFormat("GPS_USART2 ok=%u init=%ld baud=%lu bytes=%lu fix=%u valid=%u sv=%u age_ms=%s lon=%ld lat=%ld hmsl_mm=%ld\r\n",
                              (unsigned int)gps_status.initialized,
                              (long)gps_status.init_status,
                              (unsigned long)gps_status.baud_rate,
                              (unsigned long)gps_status.bytes,
                              (unsigned int)gps_status.fix_type,
                              (unsigned int)gps_status.valid_fix,
                              (unsigned int)gps_status.num_sv,
                              age_text,
                              (long)gps_status.lon_deg_e7,
                              (long)gps_status.lat_deg_e7,
                              (long)gps_status.hmsl_mm);
    APP_MaintUART_WriteFormat("GPS_USART2 diag pkts=%lu nav=%lu nmea=%lu gga=%lu cksum=%lu nmea_ck=%lu ovf=%lu nmea_ovf=%lu rst=%lu uerr=%lu last_err=0x%lX cfg=%lu\r\n",
                              (unsigned long)gps_status.packets,
                              (unsigned long)gps_status.nav_pvt_packets,
                              (unsigned long)gps_status.nmea_sentences,
                              (unsigned long)gps_status.nmea_gga_sentences,
                              (unsigned long)gps_status.checksum_errors,
                              (unsigned long)gps_status.nmea_checksum_errors,
                              (unsigned long)gps_status.payload_overflows,
                              (unsigned long)gps_status.nmea_overflows,
                              (unsigned long)gps_status.rx_restarts,
                              (unsigned long)gps_status.uart_errors,
                              (unsigned long)gps_status.last_uart_error,
                              (unsigned long)gps_status.config_writes);
    APP_MaintUART_WriteFormat("MAG_I2C1 ok=%u init=%ld st=%ld type=%s addr=0x%02X who=0x%02X n=%lu raw=%d,%d,%d mgauss=%ld,%ld,%ld\r\n",
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
}

void APP_MaintUART_Init(void)
{
    maint_rx_byte = 0U;
    maint_rx_used = 0U;
    maint_rx_head = 0U;
    maint_rx_tail = 0U;
    maint_last_rx_ms = HAL_GetTick();
    maint_last_gps_status_ms = 0U;
    maint_rx_error = 0U;
    maint_rx_overflow = 0U;
    maint_control_ready = 0U;

#if (APP_MAINT_UART_BOOT_TEXT_ENABLED != 0U)
    maint_write_literal("BOOT maint_uart uart8 pe0=rx pe1=tx 115200\r\n");
#endif
    maint_start_rx();
}

void APP_MaintUART_Step(void)
{
    while (maint_rx_tail != maint_rx_head) {
        uint8_t byte = maint_rx_ring[maint_rx_tail];

        ++maint_rx_tail;
        if (maint_rx_tail >= APP_MAINT_UART_RING_SIZE) {
            maint_rx_tail = 0U;
        }
        maint_process_byte(byte);
    }

    if (maint_rx_overflow != 0U) {
        maint_rx_overflow = 0U;
        maint_write_literal("WARN maint uart8 rx overflow\r\n");
    }

    if (maint_rx_error != 0U) {
        maint_rx_error = 0U;
        __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                      UART_CLEAR_PEF | UART_CLEAR_FEF);
        huart8.ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_AbortReceive(&huart8);
        maint_start_rx();
        maint_write_literal("WARN maint uart8 rx restarted\r\n");
    }

    maint_flush_idle_line();

    maint_report_gps_usart2_status();
    if (maint_control_ready != 0U) {
        APP_Control_MaintTick();
    }
}

void APP_MaintUART_Write(const char *text, uint16_t length)
{
    if ((text == 0) || (length == 0U)) {
        return;
    }

    (void)BSP_UART_Transmit_UART8((const uint8_t *)text, length, 100U);
}

void APP_MaintUART_WriteFormat(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int written;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    written = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (written <= 0) {
        return;
    }
    if ((uint32_t)written >= sizeof(buffer)) {
        written = (int)(sizeof(buffer) - 1U);
        buffer[written] = '\0';
    }

    APP_MaintUART_Write(buffer, (uint16_t)written);
}

void APP_MaintUART_OnRxCplt(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if ((huart == 0) || (huart->Instance != UART8)) {
        return;
    }

    next_head = (uint16_t)(maint_rx_head + 1U);
    if (next_head >= APP_MAINT_UART_RING_SIZE) {
        next_head = 0U;
    }

    if (next_head == maint_rx_tail) {
        maint_rx_overflow = 1U;
    } else {
        maint_rx_ring[maint_rx_head] = maint_rx_byte;
        maint_rx_head = next_head;
    }

    maint_start_rx();
}

void APP_MaintUART_OnError(UART_HandleTypeDef *huart)
{
    if ((huart == 0) || (huart->Instance != UART8)) {
        return;
    }

    maint_rx_error = 1U;
}
