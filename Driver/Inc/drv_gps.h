#ifndef DRV_GPS_H
#define DRV_GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define DRV_GPS_UBX_NAV_PVT_CLASS 0x01U
#define DRV_GPS_UBX_NAV_PVT_ID    0x07U

typedef enum {
    DRV_GPS_OK = 0,
    DRV_GPS_ERROR,
    DRV_GPS_TIMEOUT,
    DRV_GPS_INVALID_ARG
} DRV_GPS_Status;

typedef struct {
    uint8_t  valid;
    uint8_t  fix_type;
    uint8_t  flags;
    uint8_t  num_sv;
    uint8_t  valid_flags;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    int32_t  lon_deg_e7;
    int32_t  lat_deg_e7;
    int32_t  height_mm;
    int32_t  hmsl_mm;
    uint32_t hacc_mm;
    uint32_t vacc_mm;
    int32_t  vel_n_mm_s;
    int32_t  vel_e_mm_s;
    int32_t  vel_d_mm_s;
    int32_t  ground_speed_mm_s;
    int32_t  heading_motion_deg_e5;
    uint32_t speed_acc_mm_s;
    uint32_t heading_acc_deg_e5;
    uint16_t pdop_centi;
    uint32_t itow_ms;
    uint32_t received_ms;
} DRV_GPS_NavPvt;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t baud_rate;
    void (*delay_ms)(uint32_t ms);
} DRV_GPS_Bus;

typedef struct {
    DRV_GPS_Bus    bus;
    uint8_t        initialized;
    uint8_t        rx_active;
    uint8_t        last_class;
    uint8_t        last_id;
    uint16_t       last_length;
    uint32_t       bytes;
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
} DRV_GPS_Device;

DRV_GPS_Status DRV_GPS_Init(DRV_GPS_Device *dev, const DRV_GPS_Bus *bus);
DRV_GPS_Status DRV_GPS_ConfigureM9NDefault(DRV_GPS_Device *dev);
void DRV_GPS_Service(DRV_GPS_Device *dev);
void DRV_GPS_OnUartRxCplt(DRV_GPS_Device *dev);
void DRV_GPS_OnUartError(DRV_GPS_Device *dev, uint32_t error_code);
void DRV_GPS_Invalidate(DRV_GPS_Device *dev);

#ifdef __cplusplus
}
#endif

#endif
