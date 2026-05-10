#ifndef APP_GPS_H
#define APP_GPS_H

#include <stdint.h>

typedef struct {
    uint8_t initialized;
    int32_t init_status;
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
    uint8_t fix_type;
    uint8_t valid_fix;
    uint8_t num_sv;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    int32_t lon_deg_e7;
    int32_t lat_deg_e7;
    int32_t hmsl_mm;
    uint32_t hacc_mm;
    uint32_t vacc_mm;
    int32_t vel_n_mm_s;
    int32_t vel_e_mm_s;
    int32_t vel_d_mm_s;
    int32_t heading_motion_deg_e5;
} APP_GPS_Status;

void APP_GPS_Init(void);
void APP_GPS_Step(void);
void APP_GPS_GetStatus(APP_GPS_Status *status);

#endif
