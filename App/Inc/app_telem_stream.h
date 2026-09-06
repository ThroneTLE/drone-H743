#ifndef APP_TELEM_STREAM_H
#define APP_TELEM_STREAM_H

#include <stdint.h>

#include "app_telem_frame.h"

/*
 * 遥测流 v2 的策略层：掩码 / 脏位 / 全量刷新计时 / 出口选择 / 超限拒绝。
 *
 * 分层的理由（与规划文档 §2.5 的模块划分有一处刻意偏离，写在这里）：
 * 规划稿把整块逻辑放在 app_telem_stream.c 一个文件里。但验收判据要求脏位、
 * 刷新周期、出口选择、拔出关流、超限 ERR 全部由宿主 gcc 装置覆盖，而这些
 * 判定一旦和 osDelay / USB CDC / 稳定环快照写在同一个 TU 里，宿主上就只能改
 * 用 Python 重写一遍模型来"验证"——那验的是模型不是固件。所以把平台交互抽成
 * 下面这组 Port 函数：目标板由 App/Src/app_telem_port.c 实现，宿主装置自带
 * 一份实现，两边跑的是同一份策略代码。
 */

typedef enum {
    APP_TELEM_SINK_AUTO = 0,  /* 跟随 STREAM on 那条命令是从哪条链路进来的 */
    APP_TELEM_SINK_UART,
    APP_TELEM_SINK_USB
} APP_TelemSink;

typedef enum {
    APP_TELEM_FORMAT_BIN = 0, /* 自描述掩码帧 */
    APP_TELEM_FORMAT_JF       /* 旧 VOFA JustFloat 定长帧，Synex 过渡用 */
} APP_TelemFormat;

typedef enum {
    APP_TELEM_STREAM_OK = 0,
    APP_TELEM_STREAM_ERR_RANGE,      /* 速率 / 刷新周期越界 */
    APP_TELEM_STREAM_ERR_MASK,       /* 掩码为空或含不存在的通道 */
    APP_TELEM_STREAM_ERR_TOO_LARGE,  /* 该配置在该出口上会超帧长上限 */
    APP_TELEM_STREAM_ERR_SINK        /* 出口与格式不兼容（jf 只走 UART） */
} APP_TelemStreamStatus;

#define APP_TELEM_STREAM_RATE_MIN_HZ   1U
#define APP_TELEM_STREAM_RATE_MAX_HZ   40U  /* R-T1；USB 高速档在 R-T2 放宽 */
#define APP_TELEM_STREAM_REFRESH_MAX_S 60U
#define APP_TELEM_STREAM_REFRESH_DEFAULT_S 1U

/*
 * 流开关。**保留历史名**：app_flight_log.c 在 FLOG 导出前后直接读写它做挂起 /
 * 恢复，改名会把那条互斥悄悄拆掉；app_control.c 的 `Sensor_Data:1/0` 别名也
 * 写它。定义在 app_telem_stream.c（原先在 freertos.c 的 USER CODE 段里）。
 * App/Inc/app_tasks.h 里的 extern 是同一份声明。
 */
extern volatile uint8_t vofaStreamActive;

/* ------------------------------------------------------------------ */
/* 命令面（app_cmd_telem.c 使用）                                       */
/* ------------------------------------------------------------------ */

void APP_TelemStream_Init(void);

/*
 * 把流配置恢复成上电默认并关流。流配置只存 RAM（规划文档 §2.4），所以"恢复
 * 默认"就是重新初始化。契约装置用它在用例之间归零。
 */
void APP_TelemStream_Reset(void);

/*
 * 记下最近一条控制命令是从哪条链路进来的。app_usb_cdc.c 与 app_uart.c 在把行
 * 交给 APP_Control_ProcessLine 之前各调一次；`SINK auto` 据此决定往哪儿发。
 */
void APP_TelemStream_NoteCommandSource(APP_TelemSink source);

APP_TelemStreamStatus APP_TelemStream_SetActive(uint8_t active);
APP_TelemStreamStatus APP_TelemStream_SetRate(uint32_t hz);
APP_TelemStreamStatus APP_TelemStream_SetMask(APP_TelemMask mask);
APP_TelemStreamStatus APP_TelemStream_SetRefresh(uint32_t seconds);
APP_TelemStreamStatus APP_TelemStream_SetFormat(APP_TelemFormat format);
APP_TelemStreamStatus APP_TelemStream_SetSink(APP_TelemSink sink);

/* 上电默认掩码：12 路实时通道 + 全部参数通道（后者平时不置位，不占带宽）。 */
APP_TelemMask APP_TelemStream_DefaultMask(void);

/* 当前实际出口（把 AUTO 解开）。 */
APP_TelemSink APP_TelemStream_ActiveSink(void);

/* `TELEM?` 的第二行：流状态。 */
void APP_TelemStream_ReportStatus(void);

/* ------------------------------------------------------------------ */
/* 任务面                                                              */
/* ------------------------------------------------------------------ */

/* 遥测任务的一拍：限速、导出互斥、采样、编码、发送。 */
void APP_TelemStream_Tick(void);

/* ------------------------------------------------------------------ */
/* 平台边界（目标板 = App/Src/app_telem_port.c；宿主 = pytest 装置）      */
/* ------------------------------------------------------------------ */

/* 单调微秒时间戳的低 32 位（约 71 分钟回绕，上位机负责解回绕）。 */
uint32_t APP_TelemStream_PortNowUs(void);

/* 按当前速率让出 CPU。 */
void APP_TelemStream_PortDelayMs(uint32_t ms);

/*
 * IMUCAP / FLOG 导出是否占用了本拍。IMUCAP 导出会在这里搬运一批数据；
 * 返回 1 表示本拍不发遥测帧（两个数据流不能争同一条 CDC 链路）。
 */
uint8_t APP_TelemStream_PortServiceExports(void);

/* USB CDC 是否已枚举并配置好。 */
uint8_t APP_TelemStream_PortUsbReady(void);

/*
 * 取一份新的通道快照，写满 count 个 float。返回 0 表示这一拍没有新样本
 * （不重发上一拍的旧值，宁可不发）。
 */
uint8_t APP_TelemStream_PortSample(float *values, uint32_t count);

/* 出口写。返回 0 表示没发出去，调用方计入 drop。 */
uint8_t APP_TelemStream_PortSendUart(const uint8_t *frame, uint16_t length);
uint8_t APP_TelemStream_PortSendUsb(const uint8_t *frame, uint16_t length);

/* 旧 JustFloat 定长帧（fmt=jf），只走 UART。 */
uint8_t APP_TelemStream_PortSendJustFloat(const float *values, uint32_t count);

/* 该出口能接受的 payload 上限（UART 受 APP_UART_TX_TEXT_SIZE 约束）。 */
uint16_t APP_TelemStream_PortMaxPayload(APP_TelemSink sink);

/* 回一行文本（末尾自带 \r\n）。 */
void APP_TelemStream_PortReply(const char *text);

#endif /* APP_TELEM_STREAM_H */
