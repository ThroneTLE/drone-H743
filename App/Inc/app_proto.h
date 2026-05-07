#ifndef APP_PROTO_H
#define APP_PROTO_H

#include <stdint.h>

#define APP_PROTO_MAX_PAYLOAD 256U

#define APP_PROTO_DIR_TO_FC   '<'
#define APP_PROTO_DIR_FROM_FC '>'

#define APP_PROTO_REQ_PING            0x1000U
#define APP_PROTO_REQ_STATUS          0x1001U
#define APP_PROTO_REQ_CONFIG          0x1002U
#define APP_PROTO_REQ_PARAMS          0x1003U
#define APP_PROTO_REQ_PID            0x1004U
#define APP_PROTO_REQ_BARO           0x1005U
#define APP_PROTO_REQ_BARO_STREAM    0x1006U
#define APP_PROTO_REQ_FLASH          0x1007U
#define APP_PROTO_REQ_IMU            0x1008U
#define APP_PROTO_REQ_MODULES        0x1009U
#define APP_PROTO_REQ_CAPS           0x100AU
#define APP_PROTO_REQ_SAVE           0x100BU
#define APP_PROTO_REQ_LOAD           0x100CU
#define APP_PROTO_REQ_DEFAULTS       0x100DU
#define APP_PROTO_REQ_PARAM_SET      0x100EU
#define APP_PROTO_REQ_PID_SET        0x100FU
#define APP_PROTO_REQ_SERVO_MOVE     0x1010U
#define APP_PROTO_REQ_SERVO_MOVE_ALL 0x1011U
#define APP_PROTO_REQ_SERVO_ID       0x1012U
#define APP_PROTO_REQ_SERVO_SETID    0x1013U
#define APP_PROTO_REQ_SERVO_MODE     0x1014U
#define APP_PROTO_REQ_SERVO_ENABLE   0x1015U
#define APP_PROTO_REQ_SERVO_ACTION   0x1016U
#define APP_PROTO_REQ_SERVO_RAW      0x1017U
#define APP_PROTO_REQ_WIFI           0x1018U

#define APP_PROTO_MSG_CMD_LINE  0x2000U
#define APP_PROTO_MSG_TEXT_LINE 0x2001U
#define APP_PROTO_MSG_CMD_RX    0x2100U
#define APP_PROTO_MSG_CMD_ACK   0x2101U
#define APP_PROTO_MSG_CMD_ERR   0x2102U
#define APP_PROTO_MSG_CMD_OK    0x2103U
#define APP_PROTO_MSG_PONG              0x2200U
#define APP_PROTO_MSG_HW_FLASH          0x2201U
#define APP_PROTO_MSG_HW_BARO           0x2202U
#define APP_PROTO_MSG_HW_IMU            0x2203U
#define APP_PROTO_MSG_STATUS_FLASH      0x2204U
#define APP_PROTO_MSG_STATUS_BARO       0x2205U
#define APP_PROTO_MSG_STATUS_IMU        0x2206U
#define APP_PROTO_MSG_UART_STATS        0x2207U
#define APP_PROTO_MSG_CONFIG_SUMMARY    0x2208U
#define APP_PROTO_MSG_CONFIG_SERVO      0x2209U
#define APP_PROTO_MSG_PARAM_RECORD      0x220AU
#define APP_PROTO_MSG_PID_RECORD        0x220BU
#define APP_PROTO_MSG_FLASH_RECORD      0x220CU
#define APP_PROTO_MSG_BARO_STATE        0x220DU
#define APP_PROTO_MSG_BARO_DIAG         0x220EU
#define APP_PROTO_MSG_BARO_RAW          0x220FU
#define APP_PROTO_MSG_BARO_STREAM       0x2210U
#define APP_PROTO_MSG_IMU_STATE         0x2211U
#define APP_PROTO_MSG_IMU_SCALED        0x2212U
#define APP_PROTO_MSG_MODULES_SUMMARY   0x2213U
#define APP_PROTO_MSG_CAPS_RECORD       0x2214U
#define APP_PROTO_MSG_READY             0x2215U
#define APP_PROTO_MSG_SAVE_RESULT       0x2216U
#define APP_PROTO_MSG_LOAD_RESULT       0x2217U
#define APP_PROTO_MSG_DEFAULTS_RESULT   0x2218U
#define APP_PROTO_MSG_SERVO_RESULT      0x2219U
#define APP_PROTO_MSG_WIFI_RECORD       0x221AU

typedef struct {
    uint8_t direction;
    uint16_t function;
    uint16_t payload_length;
    uint8_t payload[APP_PROTO_MAX_PAYLOAD];
} APP_ProtoFrame;

void APP_Proto_Init(void);
uint8_t APP_Proto_IsReceiving(void);
uint8_t APP_Proto_ConsumeByte(uint8_t byte, APP_ProtoFrame *out_frame);
uint8_t APP_Proto_BuildFrame(uint8_t direction,
                             uint16_t function,
                             const uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *out_buffer,
                             uint16_t out_capacity,
                             uint16_t *out_length);

#endif
