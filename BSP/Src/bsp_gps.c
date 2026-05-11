#include "bsp_gps.h"
#include "bsp_board.h"

#include <string.h>

static DRV_GPS_Device gps_dev;

DRV_GPS_Status BSP_GPS_Init(void)
{
    return DRV_GPS_Init(&gps_dev, BSP_Board_GetGpsBus());
}

DRV_GPS_Status BSP_GPS_ConfigureM9NDefault(void)
{
    return DRV_GPS_ConfigureM9NDefault(&gps_dev);
}

void BSP_GPS_Service(void)
{
    DRV_GPS_Service(&gps_dev);
}

void BSP_GPS_OnUartRxCplt(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2)) { return; }
    DRV_GPS_OnUartRxCplt(&gps_dev);
}

void BSP_GPS_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2)) { return; }
    DRV_GPS_OnUartError(&gps_dev, huart->ErrorCode);
}

void BSP_GPS_GetStatus(BSP_GPS_Status *status)
{
    if (status == NULL) { return; }

    memset(status, 0, sizeof(*status));
    status->rx_active         = gps_dev.rx_active;
    status->last_class        = gps_dev.last_class;
    status->last_id           = gps_dev.last_id;
    status->last_length       = gps_dev.last_length;
    status->bytes             = gps_dev.bytes;
    status->baud_rate         = gps_dev.bus.baud_rate;
    status->last_uart_error   = gps_dev.last_uart_error;
    status->packets           = gps_dev.packets;
    status->nav_pvt_packets   = gps_dev.nav_pvt_packets;
    status->nmea_sentences    = gps_dev.nmea_sentences;
    status->nmea_gga_sentences = gps_dev.nmea_gga_sentences;
    status->nmea_checksum_errors = gps_dev.nmea_checksum_errors;
    status->nmea_overflows    = gps_dev.nmea_overflows;
    status->checksum_errors   = gps_dev.checksum_errors;
    status->payload_overflows = gps_dev.payload_overflows;
    status->rx_restarts       = gps_dev.rx_restarts;
    status->uart_errors       = gps_dev.uart_errors;
    status->config_writes     = gps_dev.config_writes;
    status->last_rx_ms        = gps_dev.last_rx_ms;
    status->nav               = gps_dev.nav;
}

void BSP_GPS_Invalidate(void)
{
    DRV_GPS_Invalidate(&gps_dev);
}
