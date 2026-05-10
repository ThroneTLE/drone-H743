#include "bsp_gps.h"

#include "usart.h"

#include <string.h>

#define BSP_GPS_UBX_SYNC1             0xB5U
#define BSP_GPS_UBX_SYNC2             0x62U
#define BSP_GPS_UBX_MAX_PAYLOAD       100U
#define BSP_GPS_NMEA_MAX_LINE         96U
#define BSP_GPS_UART_TIMEOUT_MS       100U
#define BSP_GPS_UART_BAUD_RATE        38400U

#define BSP_GPS_UBX_CLASS_CFG         0x06U
#define BSP_GPS_UBX_ID_CFG_RATE       0x08U
#define BSP_GPS_UBX_ID_CFG_MSG        0x01U
#define BSP_GPS_UBX_ID_CFG_PRT        0x00U

typedef enum {
    BSP_GPS_PARSE_SYNC1 = 0,
    BSP_GPS_PARSE_SYNC2,
    BSP_GPS_PARSE_CLASS,
    BSP_GPS_PARSE_ID,
    BSP_GPS_PARSE_LEN1,
    BSP_GPS_PARSE_LEN2,
    BSP_GPS_PARSE_PAYLOAD,
    BSP_GPS_PARSE_CK_A,
    BSP_GPS_PARSE_CK_B
} BSP_GPS_ParseState;

typedef struct {
    BSP_GPS_ParseState state;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint16_t offset;
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t rx_byte;
    uint8_t payload[BSP_GPS_UBX_MAX_PAYLOAD];
    char nmea_line[BSP_GPS_NMEA_MAX_LINE];
    uint16_t nmea_length;
    uint8_t nmea_active;
    BSP_GPS_Status status;
} BSP_GPS_Context;

static BSP_GPS_Context gps_ctx;

static uint16_t gps_get_u16(const uint8_t *payload, uint16_t offset)
{
    return (uint16_t)((uint16_t)payload[offset] |
                      ((uint16_t)payload[offset + 1U] << 8U));
}

static uint32_t gps_get_u32(const uint8_t *payload, uint16_t offset)
{
    return (uint32_t)payload[offset] |
           ((uint32_t)payload[offset + 1U] << 8U) |
           ((uint32_t)payload[offset + 2U] << 16U) |
           ((uint32_t)payload[offset + 3U] << 24U);
}

static int32_t gps_get_i32(const uint8_t *payload, uint16_t offset)
{
    return (int32_t)gps_get_u32(payload, offset);
}

static uint8_t gps_hex_nibble(char ch, uint8_t *value)
{
    if (value == NULL) {
        return 0U;
    }

    if ((ch >= '0') && (ch <= '9')) {
        *value = (uint8_t)(ch - '0');
        return 1U;
    }
    if ((ch >= 'A') && (ch <= 'F')) {
        *value = (uint8_t)(ch - 'A' + 10);
        return 1U;
    }
    if ((ch >= 'a') && (ch <= 'f')) {
        *value = (uint8_t)(ch - 'a' + 10);
        return 1U;
    }

    return 0U;
}

static uint8_t gps_parse_u8_field(const char *text)
{
    uint32_t value = 0U;

    if (text == NULL) {
        return 0U;
    }

    while ((*text >= '0') && (*text <= '9')) {
        value = (value * 10U) + (uint32_t)(*text - '0');
        if (value > 255U) {
            return 255U;
        }
        ++text;
    }

    return (uint8_t)value;
}

static uint16_t gps_parse_decimal_centi(const char *text)
{
    uint32_t whole = 0U;
    uint32_t frac = 0U;
    uint8_t frac_digits = 0U;

    if (text == NULL) {
        return 0U;
    }

    while ((*text >= '0') && (*text <= '9')) {
        whole = (whole * 10U) + (uint32_t)(*text - '0');
        ++text;
    }

    if (*text == '.') {
        ++text;
        while ((*text >= '0') && (*text <= '9') && (frac_digits < 2U)) {
            frac = (frac * 10U) + (uint32_t)(*text - '0');
            ++frac_digits;
            ++text;
        }
    }

    while (frac_digits < 2U) {
        frac *= 10U;
        ++frac_digits;
    }

    whole = (whole * 100U) + frac;
    return (whole > 65535U) ? 65535U : (uint16_t)whole;
}

static int32_t gps_parse_decimal_mm(const char *text)
{
    int32_t sign = 1L;
    uint32_t whole = 0U;
    uint32_t frac = 0U;
    uint8_t frac_digits = 0U;
    int64_t millimeters;

    if (text == NULL) {
        return 0L;
    }

    if (*text == '-') {
        sign = -1L;
        ++text;
    } else if (*text == '+') {
        ++text;
    }

    while ((*text >= '0') && (*text <= '9')) {
        whole = (whole * 10U) + (uint32_t)(*text - '0');
        ++text;
    }

    if (*text == '.') {
        ++text;
        while ((*text >= '0') && (*text <= '9') && (frac_digits < 3U)) {
            frac = (frac * 10U) + (uint32_t)(*text - '0');
            ++frac_digits;
            ++text;
        }
    }

    while (frac_digits < 3U) {
        frac *= 10U;
        ++frac_digits;
    }

    millimeters = ((int64_t)whole * 1000LL) + (int64_t)frac;
    millimeters *= (int64_t)sign;

    if (millimeters > 2147483647LL) {
        return 2147483647L;
    }
    if (millimeters < -2147483647LL) {
        return -2147483647L;
    }

    return (int32_t)millimeters;
}

static int32_t gps_parse_nmea_coord_e7(const char *text, char hemisphere)
{
    uint32_t whole = 0U;
    uint32_t frac = 0U;
    uint32_t scale = 1U;
    uint32_t degrees;
    uint32_t minutes_whole;
    uint64_t minutes_e7;
    uint64_t coord_e7;
    uint8_t digit_seen = 0U;
    int32_t signed_coord;

    if ((text == NULL) || (*text == '\0')) {
        return 0L;
    }

    while ((*text >= '0') && (*text <= '9')) {
        whole = (whole * 10U) + (uint32_t)(*text - '0');
        digit_seen = 1U;
        ++text;
    }

    if (digit_seen == 0U) {
        return 0L;
    }

    if (*text == '.') {
        ++text;
        while ((*text >= '0') && (*text <= '9')) {
            if (scale < 10000000U) {
                frac = (frac * 10U) + (uint32_t)(*text - '0');
                scale *= 10U;
            }
            ++text;
        }
    }

    while (scale < 10000000U) {
        frac *= 10U;
        scale *= 10U;
    }

    degrees = whole / 100U;
    minutes_whole = whole % 100U;
    if (minutes_whole >= 60U) {
        return 0L;
    }

    minutes_e7 = ((uint64_t)minutes_whole * 10000000ULL) + (uint64_t)frac;
    coord_e7 = ((uint64_t)degrees * 10000000ULL) + ((minutes_e7 + 30ULL) / 60ULL);
    if (coord_e7 > 2147483647ULL) {
        coord_e7 = 2147483647ULL;
    }

    signed_coord = (int32_t)coord_e7;
    if ((hemisphere == 'S') || (hemisphere == 's') ||
        (hemisphere == 'W') || (hemisphere == 'w')) {
        signed_coord = -signed_coord;
    }

    return signed_coord;
}

static char *gps_next_nmea_field(char **cursor)
{
    char *field;
    char *comma;

    if ((cursor == NULL) || (*cursor == NULL)) {
        return NULL;
    }

    field = *cursor;
    comma = strchr(field, ',');
    if (comma == NULL) {
        *cursor = NULL;
    } else {
        *comma = '\0';
        *cursor = comma + 1;
    }

    return field;
}

static void gps_parse_nmea_utc_time(const char *text, BSP_GPS_NavPvt *nav)
{
    if ((text == NULL) || (nav == NULL)) {
        return;
    }

    if ((text[0] < '0') || (text[0] > '9') ||
        (text[1] < '0') || (text[1] > '9') ||
        (text[2] < '0') || (text[2] > '9') ||
        (text[3] < '0') || (text[3] > '9') ||
        (text[4] < '0') || (text[4] > '9') ||
        (text[5] < '0') || (text[5] > '9')) {
        return;
    }

    nav->hour = (uint8_t)(((uint8_t)(text[0] - '0') * 10U) + (uint8_t)(text[1] - '0'));
    nav->minute = (uint8_t)(((uint8_t)(text[2] - '0') * 10U) + (uint8_t)(text[3] - '0'));
    nav->second = (uint8_t)(((uint8_t)(text[4] - '0') * 10U) + (uint8_t)(text[5] - '0'));
}

static void gps_checksum_add(uint8_t byte)
{
    gps_ctx.ck_a = (uint8_t)(gps_ctx.ck_a + byte);
    gps_ctx.ck_b = (uint8_t)(gps_ctx.ck_b + gps_ctx.ck_a);
}

static void gps_parser_reset(void)
{
    gps_ctx.state = BSP_GPS_PARSE_SYNC1;
    gps_ctx.length = 0U;
    gps_ctx.offset = 0U;
    gps_ctx.ck_a = 0U;
    gps_ctx.ck_b = 0U;
}

static void gps_decode_nav_pvt(const uint8_t *payload, uint16_t length)
{
    BSP_GPS_NavPvt *nav = &gps_ctx.status.nav;

    if ((payload == NULL) || (length < 92U)) {
        return;
    }

    nav->itow_ms = gps_get_u32(payload, 0U);
    nav->year = gps_get_u16(payload, 4U);
    nav->month = payload[6];
    nav->day = payload[7];
    nav->hour = payload[8];
    nav->minute = payload[9];
    nav->second = payload[10];
    nav->valid_flags = payload[11];
    nav->fix_type = payload[20];
    nav->flags = payload[21];
    nav->num_sv = payload[23];
    nav->lon_deg_e7 = gps_get_i32(payload, 24U);
    nav->lat_deg_e7 = gps_get_i32(payload, 28U);
    nav->height_mm = gps_get_i32(payload, 32U);
    nav->hmsl_mm = gps_get_i32(payload, 36U);
    nav->hacc_mm = gps_get_u32(payload, 40U);
    nav->vacc_mm = gps_get_u32(payload, 44U);
    nav->vel_n_mm_s = gps_get_i32(payload, 48U);
    nav->vel_e_mm_s = gps_get_i32(payload, 52U);
    nav->vel_d_mm_s = gps_get_i32(payload, 56U);
    nav->ground_speed_mm_s = gps_get_i32(payload, 60U);
    nav->heading_motion_deg_e5 = gps_get_i32(payload, 64U);
    nav->speed_acc_mm_s = gps_get_u32(payload, 68U);
    nav->heading_acc_deg_e5 = gps_get_u32(payload, 72U);
    nav->pdop_centi = gps_get_u16(payload, 76U);
    nav->received_ms = HAL_GetTick();
    nav->valid = ((nav->fix_type >= 2U) && ((nav->flags & 0x01U) != 0U)) ? 1U : 0U;
    gps_ctx.status.nav_pvt_packets++;
}

static void gps_nmea_reset(void)
{
    gps_ctx.nmea_active = 0U;
    gps_ctx.nmea_length = 0U;
}

static void gps_decode_nmea_gga(char *sentence)
{
    char *cursor;
    char *type;
    char *utc;
    char *lat;
    char *lat_hemi;
    char *lon;
    char *lon_hemi;
    char *quality;
    char *num_sv;
    char *hdop;
    char *alt_m;
    uint8_t fix_quality;
    BSP_GPS_NavPvt *nav = &gps_ctx.status.nav;

    if (sentence == NULL) {
        return;
    }

    cursor = sentence;
    if (*cursor == '$') {
        ++cursor;
    }

    type = gps_next_nmea_field(&cursor);
    if ((type == NULL) || (strlen(type) < 5U) ||
        (strcmp(&type[strlen(type) - 3U], "GGA") != 0)) {
        return;
    }

    utc = gps_next_nmea_field(&cursor);
    lat = gps_next_nmea_field(&cursor);
    lat_hemi = gps_next_nmea_field(&cursor);
    lon = gps_next_nmea_field(&cursor);
    lon_hemi = gps_next_nmea_field(&cursor);
    quality = gps_next_nmea_field(&cursor);
    num_sv = gps_next_nmea_field(&cursor);
    hdop = gps_next_nmea_field(&cursor);
    alt_m = gps_next_nmea_field(&cursor);

    fix_quality = gps_parse_u8_field(quality);
    nav->received_ms = HAL_GetTick();
    nav->fix_type = (fix_quality != 0U) ? 3U : 0U;
    nav->flags = (fix_quality != 0U) ? 0x01U : 0x00U;
    nav->valid_flags = (fix_quality != 0U) ? 0x03U : 0x00U;
    nav->valid = (fix_quality != 0U) ? 1U : 0U;
    nav->num_sv = gps_parse_u8_field(num_sv);
    nav->pdop_centi = gps_parse_decimal_centi(hdop);
    gps_parse_nmea_utc_time(utc, nav);

    if ((lat != NULL) && (lat_hemi != NULL) &&
        (lon != NULL) && (lon_hemi != NULL) &&
        (*lat != '\0') && (*lon != '\0')) {
        nav->lat_deg_e7 = gps_parse_nmea_coord_e7(lat, lat_hemi[0]);
        nav->lon_deg_e7 = gps_parse_nmea_coord_e7(lon, lon_hemi[0]);
    }

    if ((alt_m != NULL) && (*alt_m != '\0')) {
        nav->hmsl_mm = gps_parse_decimal_mm(alt_m);
        nav->height_mm = nav->hmsl_mm;
    }

    gps_ctx.status.nmea_gga_sentences++;
    gps_ctx.status.last_class = 0xF0U;
    gps_ctx.status.last_id = 0x00U;
    gps_ctx.status.last_length = gps_ctx.nmea_length;
}

static void gps_handle_nmea_line(void)
{
    char *asterisk;
    uint8_t checksum = 0U;
    uint8_t high;
    uint8_t low;
    char *cursor;

    gps_ctx.nmea_line[gps_ctx.nmea_length] = '\0';
    if ((gps_ctx.nmea_length < 7U) || (gps_ctx.nmea_line[0] != '$')) {
        gps_nmea_reset();
        return;
    }

    asterisk = strchr(gps_ctx.nmea_line, '*');
    if ((asterisk == NULL) ||
        (gps_hex_nibble(asterisk[1], &high) == 0U) ||
        (gps_hex_nibble(asterisk[2], &low) == 0U)) {
        gps_ctx.status.nmea_checksum_errors++;
        gps_nmea_reset();
        return;
    }

    for (cursor = &gps_ctx.nmea_line[1]; cursor < asterisk; ++cursor) {
        checksum ^= (uint8_t)*cursor;
    }

    if (checksum != (uint8_t)((high << 4U) | low)) {
        gps_ctx.status.nmea_checksum_errors++;
        gps_nmea_reset();
        return;
    }

    *asterisk = '\0';
    gps_ctx.status.nmea_sentences++;
    gps_decode_nmea_gga(gps_ctx.nmea_line);
    gps_nmea_reset();
}

static void gps_consume_nmea_byte(uint8_t byte)
{
    if (byte == '$') {
        gps_ctx.nmea_active = 1U;
        gps_ctx.nmea_length = 0U;
    }

    if (gps_ctx.nmea_active == 0U) {
        return;
    }

    if ((byte == '\r') || (byte == '\n')) {
        if (gps_ctx.nmea_length != 0U) {
            gps_handle_nmea_line();
        } else {
            gps_nmea_reset();
        }
        return;
    }

    if (gps_ctx.nmea_length >= (BSP_GPS_NMEA_MAX_LINE - 1U)) {
        gps_ctx.status.nmea_overflows++;
        gps_nmea_reset();
        return;
    }

    gps_ctx.nmea_line[gps_ctx.nmea_length++] = (char)byte;
}

static void gps_handle_packet(void)
{
    gps_ctx.status.last_class = gps_ctx.msg_class;
    gps_ctx.status.last_id = gps_ctx.msg_id;
    gps_ctx.status.last_length = gps_ctx.length;
    gps_ctx.status.packets++;

    if ((gps_ctx.msg_class == BSP_GPS_UBX_NAV_PVT_CLASS) &&
        (gps_ctx.msg_id == BSP_GPS_UBX_NAV_PVT_ID)) {
        gps_decode_nav_pvt(gps_ctx.payload, gps_ctx.length);
    }
}

static void gps_consume_byte(uint8_t byte)
{
    gps_ctx.status.bytes++;
    gps_ctx.status.last_rx_ms = HAL_GetTick();
    gps_consume_nmea_byte(byte);

    switch (gps_ctx.state) {
    case BSP_GPS_PARSE_SYNC1:
        if (byte == BSP_GPS_UBX_SYNC1) {
            gps_ctx.state = BSP_GPS_PARSE_SYNC2;
        }
        break;

    case BSP_GPS_PARSE_SYNC2:
        if (byte == BSP_GPS_UBX_SYNC2) {
            gps_ctx.state = BSP_GPS_PARSE_CLASS;
            gps_ctx.ck_a = 0U;
            gps_ctx.ck_b = 0U;
        } else {
            gps_parser_reset();
        }
        break;

    case BSP_GPS_PARSE_CLASS:
        gps_ctx.msg_class = byte;
        gps_checksum_add(byte);
        gps_ctx.state = BSP_GPS_PARSE_ID;
        break;

    case BSP_GPS_PARSE_ID:
        gps_ctx.msg_id = byte;
        gps_checksum_add(byte);
        gps_ctx.state = BSP_GPS_PARSE_LEN1;
        break;

    case BSP_GPS_PARSE_LEN1:
        gps_ctx.length = byte;
        gps_checksum_add(byte);
        gps_ctx.state = BSP_GPS_PARSE_LEN2;
        break;

    case BSP_GPS_PARSE_LEN2:
        gps_ctx.length |= (uint16_t)((uint16_t)byte << 8U);
        gps_checksum_add(byte);
        gps_ctx.offset = 0U;
        if (gps_ctx.length > BSP_GPS_UBX_MAX_PAYLOAD) {
            gps_ctx.status.payload_overflows++;
            gps_parser_reset();
        } else if (gps_ctx.length == 0U) {
            gps_ctx.state = BSP_GPS_PARSE_CK_A;
        } else {
            gps_ctx.state = BSP_GPS_PARSE_PAYLOAD;
        }
        break;

    case BSP_GPS_PARSE_PAYLOAD:
        gps_ctx.payload[gps_ctx.offset++] = byte;
        gps_checksum_add(byte);
        if (gps_ctx.offset >= gps_ctx.length) {
            gps_ctx.state = BSP_GPS_PARSE_CK_A;
        }
        break;

    case BSP_GPS_PARSE_CK_A:
        if (byte == gps_ctx.ck_a) {
            gps_ctx.state = BSP_GPS_PARSE_CK_B;
        } else {
            gps_ctx.status.checksum_errors++;
            gps_parser_reset();
        }
        break;

    case BSP_GPS_PARSE_CK_B:
        if (byte == gps_ctx.ck_b) {
            gps_handle_packet();
        } else {
            gps_ctx.status.checksum_errors++;
        }
        gps_parser_reset();
        break;

    default:
        gps_parser_reset();
        break;
    }
}

static BSP_GPS_StatusCode gps_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return BSP_GPS_OK;
    case HAL_TIMEOUT:
        return BSP_GPS_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:
        return BSP_GPS_ERROR;
    }
}

static void gps_ubx_checksum(const uint8_t *data, uint16_t length, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0U;
    uint8_t b = 0U;

    for (uint16_t i = 0U; i < length; ++i) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }

    if (ck_a != NULL) {
        *ck_a = a;
    }
    if (ck_b != NULL) {
        *ck_b = b;
    }
}

static BSP_GPS_StatusCode gps_send_ubx(uint8_t msg_class,
                                       uint8_t msg_id,
                                       const uint8_t *payload,
                                       uint16_t payload_length)
{
    uint8_t frame[32];
    uint16_t index = 0U;
    uint8_t ck_a;
    uint8_t ck_b;
    HAL_StatusTypeDef status;

    if ((payload_length > 24U) || ((payload == NULL) && (payload_length != 0U))) {
        return BSP_GPS_INVALID_ARG;
    }

    frame[index++] = BSP_GPS_UBX_SYNC1;
    frame[index++] = BSP_GPS_UBX_SYNC2;
    frame[index++] = msg_class;
    frame[index++] = msg_id;
    frame[index++] = (uint8_t)(payload_length & 0xFFU);
    frame[index++] = (uint8_t)(payload_length >> 8U);
    for (uint16_t i = 0U; i < payload_length; ++i) {
        frame[index++] = payload[i];
    }
    gps_ubx_checksum(&frame[2], (uint16_t)(4U + payload_length), &ck_a, &ck_b);
    frame[index++] = ck_a;
    frame[index++] = ck_b;

    status = HAL_UART_Transmit(&huart2, frame, index, BSP_GPS_UART_TIMEOUT_MS);
    if (status == HAL_OK) {
        gps_ctx.status.config_writes++;
    }

    return gps_from_hal(status);
}

static void gps_start_rx_it(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive_IT(&huart2, &gps_ctx.rx_byte, 1U);
    if (status == HAL_OK) {
        gps_ctx.status.rx_active = 1U;
        gps_ctx.status.rx_restarts++;
    } else {
        gps_ctx.status.rx_active = 0U;
        gps_ctx.status.uart_errors++;
    }
}

BSP_GPS_StatusCode BSP_GPS_Init(void)
{
    memset(&gps_ctx, 0, sizeof(gps_ctx));
    gps_parser_reset();
    gps_ctx.status.baud_rate = BSP_GPS_UART_BAUD_RATE;
    (void)HAL_UART_AbortReceive(&huart2);
    (void)HAL_UART_DeInit(&huart2);
    huart2.Init.BaudRate = BSP_GPS_UART_BAUD_RATE;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        gps_ctx.status.uart_errors++;
        gps_ctx.status.last_uart_error = huart2.ErrorCode;
        return BSP_GPS_ERROR;
    }
    gps_start_rx_it();

    return (gps_ctx.status.rx_active != 0U) ? BSP_GPS_OK : BSP_GPS_ERROR;
}

BSP_GPS_StatusCode BSP_GPS_ConfigureM9NDefault(void)
{
    BSP_GPS_StatusCode status;
    const uint8_t cfg_rate_5hz[] = {
        0xC8U, 0x00U,
        0x01U, 0x00U,
        0x01U, 0x00U
    };
    const uint8_t cfg_msg_nav_pvt_current_port_1hz[] = {
        BSP_GPS_UBX_NAV_PVT_CLASS,
        BSP_GPS_UBX_NAV_PVT_ID,
        0x01U
    };

    status = gps_send_ubx(BSP_GPS_UBX_CLASS_CFG,
                          BSP_GPS_UBX_ID_CFG_RATE,
                          cfg_rate_5hz,
                          (uint16_t)sizeof(cfg_rate_5hz));
    if (status != BSP_GPS_OK) {
        return status;
    }

    return gps_send_ubx(BSP_GPS_UBX_CLASS_CFG,
                        BSP_GPS_UBX_ID_CFG_MSG,
                        cfg_msg_nav_pvt_current_port_1hz,
                        (uint16_t)sizeof(cfg_msg_nav_pvt_current_port_1hz));
}

void BSP_GPS_Service(void)
{
    if ((gps_ctx.status.rx_active == 0U) ||
        (huart2.RxState != HAL_UART_STATE_BUSY_RX)) {
        gps_start_rx_it();
    }
}

void BSP_GPS_OnUartRxCplt(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2)) {
        return;
    }

    gps_ctx.status.rx_active = 0U;
    gps_consume_byte(gps_ctx.rx_byte);
    gps_start_rx_it();
}

void BSP_GPS_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART2)) {
        return;
    }

    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                   UART_CLEAR_PEF | UART_CLEAR_FEF);
    huart2.ErrorCode = HAL_UART_ERROR_NONE;
    gps_ctx.status.rx_active = 0U;
    gps_ctx.status.uart_errors++;
    gps_ctx.status.last_uart_error = huart2.ErrorCode;
    (void)HAL_UART_AbortReceive_IT(&huart2);
    gps_start_rx_it();
}

void BSP_GPS_GetStatus(BSP_GPS_Status *status)
{
    if (status == NULL) {
        return;
    }

    *status = gps_ctx.status;
}

void BSP_GPS_Invalidate(void)
{
    (void)HAL_UART_AbortReceive_IT(&huart2);
    memset(&gps_ctx, 0, sizeof(gps_ctx));
    gps_parser_reset();
}
