#include "app_proto.h"

#include <string.h>

typedef enum {
    APP_PROTO_PARSE_IDLE = 0,
    APP_PROTO_PARSE_X,
    APP_PROTO_PARSE_DIRECTION,
    APP_PROTO_PARSE_FLAGS,
    APP_PROTO_PARSE_FUNCTION_L,
    APP_PROTO_PARSE_FUNCTION_H,
    APP_PROTO_PARSE_LENGTH_L,
    APP_PROTO_PARSE_LENGTH_H,
    APP_PROTO_PARSE_PAYLOAD,
    APP_PROTO_PARSE_CRC
} APP_ProtoParseState;

typedef struct {
    APP_ProtoParseState state;
    APP_ProtoFrame frame;
    uint16_t payload_index;
    uint8_t flags;
    uint8_t crc;
} APP_ProtoParser;

static APP_ProtoParser app_proto_parser;

static uint8_t app_proto_crc8_dvb_s2_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint32_t bit = 0U; bit < 8U; ++bit) {
        if ((crc & 0x80U) != 0U) {
            crc = (uint8_t)((crc << 1U) ^ 0xD5U);
        } else {
            crc <<= 1U;
        }
    }
    return crc;
}

static void app_proto_reset(void)
{
    app_proto_parser.state = APP_PROTO_PARSE_IDLE;
    app_proto_parser.payload_index = 0U;
    app_proto_parser.flags = 0U;
    app_proto_parser.crc = 0U;
    app_proto_parser.frame.direction = 0U;
    app_proto_parser.frame.function = 0U;
    app_proto_parser.frame.payload_length = 0U;
}

void APP_Proto_Init(void)
{
    memset(&app_proto_parser, 0, sizeof(app_proto_parser));
    app_proto_reset();
}

uint8_t APP_Proto_IsReceiving(void)
{
    return (app_proto_parser.state != APP_PROTO_PARSE_IDLE) ? 1U : 0U;
}

uint8_t APP_Proto_ConsumeByte(uint8_t byte, APP_ProtoFrame *out_frame)
{
    switch (app_proto_parser.state) {
    case APP_PROTO_PARSE_IDLE:
        if (byte == (uint8_t)'$') {
            app_proto_reset();
            app_proto_parser.state = APP_PROTO_PARSE_X;
        }
        return 0U;

    case APP_PROTO_PARSE_X:
        if (byte == (uint8_t)'X') {
            app_proto_parser.state = APP_PROTO_PARSE_DIRECTION;
            return 0U;
        }
        app_proto_reset();
        if (byte == (uint8_t)'$') {
            app_proto_parser.state = APP_PROTO_PARSE_X;
        }
        return 0U;

    case APP_PROTO_PARSE_DIRECTION:
        if ((byte != (uint8_t)APP_PROTO_DIR_TO_FC) &&
            (byte != (uint8_t)APP_PROTO_DIR_FROM_FC)) {
            app_proto_reset();
            if (byte == (uint8_t)'$') {
                app_proto_parser.state = APP_PROTO_PARSE_X;
            }
            return 0U;
        }
        app_proto_parser.frame.direction = byte;
        app_proto_parser.state = APP_PROTO_PARSE_FLAGS;
        return 0U;

    case APP_PROTO_PARSE_FLAGS:
        app_proto_parser.flags = byte;
        app_proto_parser.crc = app_proto_crc8_dvb_s2_update(0U, byte);
        app_proto_parser.state = APP_PROTO_PARSE_FUNCTION_L;
        return 0U;

    case APP_PROTO_PARSE_FUNCTION_L:
        app_proto_parser.frame.function = byte;
        app_proto_parser.crc = app_proto_crc8_dvb_s2_update(app_proto_parser.crc, byte);
        app_proto_parser.state = APP_PROTO_PARSE_FUNCTION_H;
        return 0U;

    case APP_PROTO_PARSE_FUNCTION_H:
        app_proto_parser.frame.function |= (uint16_t)((uint16_t)byte << 8U);
        app_proto_parser.crc = app_proto_crc8_dvb_s2_update(app_proto_parser.crc, byte);
        app_proto_parser.state = APP_PROTO_PARSE_LENGTH_L;
        return 0U;

    case APP_PROTO_PARSE_LENGTH_L:
        app_proto_parser.frame.payload_length = byte;
        app_proto_parser.crc = app_proto_crc8_dvb_s2_update(app_proto_parser.crc, byte);
        app_proto_parser.state = APP_PROTO_PARSE_LENGTH_H;
        return 0U;

    case APP_PROTO_PARSE_LENGTH_H:
        app_proto_parser.frame.payload_length |= (uint16_t)((uint16_t)byte << 8U);
        app_proto_parser.crc = app_proto_crc8_dvb_s2_update(app_proto_parser.crc, byte);
        if (app_proto_parser.frame.payload_length > APP_PROTO_MAX_PAYLOAD) {
            app_proto_reset();
            return 0U;
        }
        app_proto_parser.payload_index = 0U;
        if (app_proto_parser.frame.payload_length == 0U) {
            app_proto_parser.state = APP_PROTO_PARSE_CRC;
        } else {
            app_proto_parser.state = APP_PROTO_PARSE_PAYLOAD;
        }
        return 0U;

    case APP_PROTO_PARSE_PAYLOAD:
        app_proto_parser.frame.payload[app_proto_parser.payload_index++] = byte;
        app_proto_parser.crc = app_proto_crc8_dvb_s2_update(app_proto_parser.crc, byte);
        if (app_proto_parser.payload_index >= app_proto_parser.frame.payload_length) {
            app_proto_parser.state = APP_PROTO_PARSE_CRC;
        }
        return 0U;

    case APP_PROTO_PARSE_CRC:
        if ((out_frame != NULL) && (byte == app_proto_parser.crc)) {
            *out_frame = app_proto_parser.frame;
            app_proto_reset();
            return 1U;
        }
        app_proto_reset();
        return 0U;

    default:
        app_proto_reset();
        return 0U;
    }
}

uint8_t APP_Proto_BuildFrame(uint8_t direction,
                             uint16_t function,
                             const uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *out_buffer,
                             uint16_t out_capacity,
                             uint16_t *out_length)
{
    uint8_t crc = 0U;
    uint16_t offset = 0U;

    if ((out_buffer == NULL) || (out_length == NULL)) {
        return 0U;
    }

    if ((payload_length > 0U) && (payload == NULL)) {
        return 0U;
    }

    if (payload_length > APP_PROTO_MAX_PAYLOAD) {
        return 0U;
    }

    if (out_capacity < (uint16_t)(9U + payload_length)) {
        return 0U;
    }

    out_buffer[offset++] = (uint8_t)'$';
    out_buffer[offset++] = (uint8_t)'X';
    out_buffer[offset++] = direction;
    out_buffer[offset++] = 0U;

    crc = app_proto_crc8_dvb_s2_update(crc, 0U);
    out_buffer[offset++] = (uint8_t)(function & 0xFFU);
    crc = app_proto_crc8_dvb_s2_update(crc, out_buffer[offset - 1U]);
    out_buffer[offset++] = (uint8_t)((function >> 8U) & 0xFFU);
    crc = app_proto_crc8_dvb_s2_update(crc, out_buffer[offset - 1U]);
    out_buffer[offset++] = (uint8_t)(payload_length & 0xFFU);
    crc = app_proto_crc8_dvb_s2_update(crc, out_buffer[offset - 1U]);
    out_buffer[offset++] = (uint8_t)((payload_length >> 8U) & 0xFFU);
    crc = app_proto_crc8_dvb_s2_update(crc, out_buffer[offset - 1U]);

    for (uint16_t index = 0U; index < payload_length; ++index) {
        out_buffer[offset++] = payload[index];
        crc = app_proto_crc8_dvb_s2_update(crc, payload[index]);
    }

    out_buffer[offset++] = crc;
    *out_length = offset;
    return 1U;
}
