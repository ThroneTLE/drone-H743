#ifndef BSP_GPS_H
#define BSP_GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define BSP_GPS_UBX_NAV_PVT_CLASS 0x01U
#define BSP_GPS_UBX_NAV_PVT_ID    0x07U

typedef enum {
    BSP_GPS_OK = 0,
    BSP_GPS_ERROR,
    BSP_GPS_TIMEOUT,
    BSP_GPS_INVALID_ARG
} BSP_GPS_StatusCode;

typedef struct {
    uint8_t valid;
    uint8_t fix_type;
    uint8_t flags;
    uint8_t num_sv;
    uint8_t valid_flags;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    int32_t lon_deg_e7;
    int32_t lat_deg_e7;
    int32_t height_mm;
    int32_t hmsl_mm;
    uint32_t hacc_mm;
    uint32_t vacc_mm;
    int32_t vel_n_mm_s;
    int32_t vel_e_mm_s;
    int32_t vel_d_mm_s;
    int32_t ground_speed_mm_s;
    int32_t heading_motion_deg_e5;
    uint32_t speed_acc_mm_s;
    uint32_t heading_acc_deg_e5;
    uint16_t pdop_centi;
    uint32_t itow_ms;
    uint32_t received_ms;
} BSP_GPS_NavPvt;

typedef struct {
    uint8_t rx_active;
    uint8_t last_class;
    uint8_t last_id;
    uint16_t last_length;
    uint32_t bytes;
    uint32_t baud_rate;
    uint32_t last_uart_error;
    uint32_t packets;
    uint32_t nav_pvt_packets;
    uint32_t nmea_sentences;
    uint32_t nmea_gga_sentences;
    uint32_t nmea_checksum_errors;
    uint32_t nmea_overflows;
    uint32_t checksum_errors;
    uint32_t payload_overflows;
    uint32_t rx_restarts;
    uint32_t uart_errors;
    uint32_t config_writes;
    uint32_t last_rx_ms;
    BSP_GPS_NavPvt nav;
} BSP_GPS_Status;

BSP_GPS_StatusCode BSP_GPS_Init(void);
BSP_GPS_StatusCode BSP_GPS_ConfigureM9NDefault(void);
void BSP_GPS_Service(void);
void BSP_GPS_OnUartRxCplt(UART_HandleTypeDef *huart);
void BSP_GPS_OnUartError(UART_HandleTypeDef *huart);
void BSP_GPS_GetStatus(BSP_GPS_Status *status);
void BSP_GPS_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
