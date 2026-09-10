#include "bsp_gps.h"
#include "bsp_board.h"

#include <string.h>

static DRV_GPS_Device gps_dev;

/*
 * 回调认哪个串口，从**板级绑定**里问，不写死实例名。
 *
 * 这两个回调原来写死 USART2。MicoAir743v2 上 GPS 挪到了 USART3，而 USART2 成了光流口
 * ——照旧写死的话，GPS 回调会去认光流的串口。这类"绑定改了、回调没跟着改"的错位
 * 编译期看不出来，所以干脆让判据只有一个来源：bsp_board.c 里的 gps_bus。
 */
static uint8_t gps_owns_uart(const UART_HandleTypeDef *huart)
{
    const DRV_GPS_Bus *bus = BSP_Board_GetGpsBus();

    if ((huart == NULL) || (bus == NULL) || (bus->huart == NULL)) { return 0U; }
    return (huart->Instance == bus->huart->Instance) ? 1U : 0U;
}

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
    if (gps_owns_uart(huart) == 0U) { return; }
    DRV_GPS_OnUartRxCplt(&gps_dev);
}

void BSP_GPS_OnUartError(UART_HandleTypeDef *huart)
{
    if (gps_owns_uart(huart) == 0U) { return; }
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
