#include "app_gps.h"

#include "bsp_gps.h"

#include <string.h>

typedef struct {
    uint8_t initialized;
    BSP_GPS_StatusCode init_status;
    BSP_GPS_Status bsp_status;
} APP_GPS_Context;

static APP_GPS_Context app_gps_ctx;

void APP_GPS_Init(void)
{
    BSP_GPS_StatusCode st;

    memset(&app_gps_ctx, 0, sizeof(app_gps_ctx));
    st = BSP_GPS_Init();
    app_gps_ctx.init_status = st;
    if (st == BSP_GPS_OK) {
        app_gps_ctx.initialized = 1U;
        (void)BSP_GPS_ConfigureM9NDefault();
    } else {
        app_gps_ctx.initialized = 0U;
    }
}

void APP_GPS_Step(void)
{
    BSP_GPS_Service();
    BSP_GPS_GetStatus(&app_gps_ctx.bsp_status);
}

void APP_GPS_GetStatus(APP_GPS_Status *status)
{
    const BSP_GPS_Status *bsp;

    if (status == NULL) {
        return;
    }

    BSP_GPS_GetStatus(&app_gps_ctx.bsp_status);
    bsp = &app_gps_ctx.bsp_status;
    memset(status, 0, sizeof(*status));

    status->initialized = app_gps_ctx.initialized;
    status->init_status = (int32_t)app_gps_ctx.init_status;
    status->bytes = bsp->bytes;
    status->baud_rate = bsp->baud_rate;
    status->last_uart_error = bsp->last_uart_error;
    status->packets = bsp->packets;
    status->nav_pvt_packets = bsp->nav_pvt_packets;
    status->nmea_sentences = bsp->nmea_sentences;
    status->nmea_gga_sentences = bsp->nmea_gga_sentences;
    status->nmea_checksum_errors = bsp->nmea_checksum_errors;
    status->nmea_overflows = bsp->nmea_overflows;
    status->checksum_errors = bsp->checksum_errors;
    status->payload_overflows = bsp->payload_overflows;
    status->rx_restarts = bsp->rx_restarts;
    status->uart_errors = bsp->uart_errors;
    status->config_writes = bsp->config_writes;
    status->last_rx_ms = bsp->last_rx_ms;
    status->fix_type = bsp->nav.fix_type;
    status->valid_fix = bsp->nav.valid;
    status->num_sv = bsp->nav.num_sv;
    status->year = bsp->nav.year;
    status->month = bsp->nav.month;
    status->day = bsp->nav.day;
    status->hour = bsp->nav.hour;
    status->minute = bsp->nav.minute;
    status->second = bsp->nav.second;
    status->lon_deg_e7 = bsp->nav.lon_deg_e7;
    status->lat_deg_e7 = bsp->nav.lat_deg_e7;
    status->hmsl_mm = bsp->nav.hmsl_mm;
    status->hacc_mm = bsp->nav.hacc_mm;
    status->vacc_mm = bsp->nav.vacc_mm;
    status->vel_n_mm_s = bsp->nav.vel_n_mm_s;
    status->vel_e_mm_s = bsp->nav.vel_e_mm_s;
    status->vel_d_mm_s = bsp->nav.vel_d_mm_s;
    status->heading_motion_deg_e5 = bsp->nav.heading_motion_deg_e5;
}
