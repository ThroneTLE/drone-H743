#ifndef BSP_GPS_H
#define BSP_GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drv_gps.h"

typedef DRV_GPS_Status  BSP_GPS_StatusCode;
typedef DRV_GPS_NavPvt  BSP_GPS_NavPvt;

#define BSP_GPS_OK           DRV_GPS_OK
#define BSP_GPS_ERROR        DRV_GPS_ERROR
#define BSP_GPS_TIMEOUT      DRV_GPS_TIMEOUT
#define BSP_GPS_INVALID_ARG  DRV_GPS_INVALID_ARG
#define BSP_GPS_UBX_NAV_PVT_CLASS DRV_GPS_UBX_NAV_PVT_CLASS
#define BSP_GPS_UBX_NAV_PVT_ID    DRV_GPS_UBX_NAV_PVT_ID

typedef struct {
    uint8_t        rx_active;
    uint8_t        last_class;
    uint8_t        last_id;
    uint16_t       last_length;
    uint32_t       bytes;
    uint32_t       baud_rate;
    uint32_t       last_uart_error;
    uint32_t       packets;
    uint32_t       nav_pvt_packets;
    uint32_t       nmea_sentences;
    uint32_t       nmea_gga_sentences;
    uint32_t       nmea_checksum_errors;
    uint32_t       nmea_overflows;
    uint32_t       checksum_errors;
    uint32_t       payload_overflows;
    uint32_t       rx_restarts;
    uint32_t       uart_errors;
    uint32_t       config_writes;
    uint32_t       last_rx_ms;
    DRV_GPS_NavPvt nav;
} BSP_GPS_Status;

DRV_GPS_Status BSP_GPS_Init(void);
DRV_GPS_Status BSP_GPS_ConfigureM9NDefault(void);
void BSP_GPS_Service(void);
void BSP_GPS_OnUartRxCplt(UART_HandleTypeDef *huart);
void BSP_GPS_OnUartError(UART_HandleTypeDef *huart);
void BSP_GPS_GetStatus(BSP_GPS_Status *status);
void BSP_GPS_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
