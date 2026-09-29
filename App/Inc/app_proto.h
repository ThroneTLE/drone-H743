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
#define APP_PROTO_REQ_GPS            0x1019U
#define APP_PROTO_REQ_MAG            0x101AU
#define APP_PROTO_REQ_RTOS           0x101BU
#define APP_PROTO_REQ_AIRFRAME       0x101CU
#define APP_PROTO_REQ_IDENT          0x101DU
#define APP_PROTO_REQ_IMU_FRAME      0x101EU
#define APP_PROTO_REQ_BOOT           0x101FU
#define APP_PROTO_REQ_IMU_CAL        0x1020U
#define APP_PROTO_REQ_ACCEPTANCE     0x1021U
#define APP_PROTO_REQ_RC             0x1022U
#define APP_PROTO_REQ_RCMAP          0x1023U
#define APP_PROTO_REQ_SERVO_CAL      0x1024U
#define APP_PROTO_REQ_SERVOTYPE      0x1025U
#define APP_PROTO_REQ_SYSID          0x1026U

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
#define APP_PROTO_MSG_GPS_RECORD        0x221BU
#define APP_PROTO_MSG_MAG_RECORD        0x221CU
#define APP_PROTO_MSG_RTOS_RECORD       0x221DU
#define APP_PROTO_MSG_FLASH_BENCH       0x221EU
#define APP_PROTO_MSG_AIRFRAME_RECORD   0x221FU
#define APP_PROTO_MSG_BOOT_STATUS       0x2220U
#define APP_PROTO_MSG_IMU_CAL           0x2221U
#define APP_PROTO_MSG_ACCEPTANCE        0x2222U
#define APP_PROTO_MSG_RC_LIVE           0x2223U
#define APP_PROTO_MSG_RC_MAP            0x2224U
#define APP_PROTO_MSG_SERVO_CAL         0x2225U
#define APP_PROTO_MSG_SERVO_TYPE        0x2226U
/*
 * 遥测流 v2 的自描述掩码帧（S8 / R-T1-1）。payload 布局见
 * App/Inc/app_telem_frame.h 与 doc/telemetry-protocol.md。
 * 上位机侧同名登记在 tools/panel_lib/proto.py::PROTO_MSG_TELEM_FRAME。
 */
#define APP_PROTO_MSG_TELEM_FRAME       0x2230U
#define APP_PROTO_MSG_COMPONENTS        0x2231U
#define APP_PROTO_MSG_BATTERY           0x2232U
/*
 * 系统辨识的高速采样通道。布局见 Driver/Inc/drv_sysid_record.h，
 * 上位机侧同名登记在 tools/panel_lib/proto.py。
 *
 * 为什么不复用 0x2230 遥测帧：遥测限速 40 Hz、采样发生在遥测任务自己的节拍上
 * （陀螺会混叠），且通道表里没有 gx/gy/gz、舵机脉宽、油门。辨识要在 500 Hz
 * 控制拍上采、用固件微秒时间戳，两者的取数时刻根本不是一回事。
 */
#define APP_PROTO_MSG_SYSID_SCHEMA      0x2233U
#define APP_PROTO_MSG_SYSID_BATCH       0x2234U
#define APP_PROTO_MSG_THRUST_BENCH      0x2235U

/*
 * 成帧器（FC -> PC 方向）在 R-T1-1 重新启用：遥测流 v2 用它把掩码帧包进
 * $X 头 + CRC8-DVB-S2。**解析器（PC -> FC）继续禁用**——上行仍然是文本行，
 * 放开解析器等于同时打开一条没人测过的输入路径。
 */
uint8_t APP_Proto_BuildFrame(uint8_t direction,
                             uint16_t function,
                             const uint8_t *payload,
                             uint16_t payload_length,
                             uint8_t *out_buffer,
                             uint16_t out_capacity,
                             uint16_t *out_length);

/* Parser side remains disabled for VOFA migration
typedef struct {
    uint8_t direction;
    uint16_t function;
    uint16_t payload_length;
    uint8_t payload[APP_PROTO_MAX_PAYLOAD];
} APP_ProtoFrame;

void APP_Proto_Init(void);
uint8_t APP_Proto_IsReceiving(void);
uint8_t APP_Proto_ConsumeByte(uint8_t byte, APP_ProtoFrame *out_frame);
*/

#endif
