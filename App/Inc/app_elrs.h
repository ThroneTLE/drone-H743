#ifndef APP_ELRS_H
#define APP_ELRS_H

#include <stdint.h>

#include "drv_elrs.h"

#ifdef __cplusplus
extern "C" {
#endif

void APP_ELRS_Init(void);

/*
 * 消费 DMA 缓冲区新字节，驱动 CRSF 协议解析。
 * 需要从高频任务（如 StabilizerTask，~1kHz）周期性调用。
 * 执行时间极其短，无阻塞。
 */
void APP_ELRS_Step(void);

/* RC 通道值，us 范围 [800, 2200] */
void APP_ELRS_GetChannels(uint16_t us_out[CRSF_CHANNEL_COUNT]);
uint32_t APP_ELRS_GetLastRcMs(void);
uint8_t APP_ELRS_IsRcFresh(uint32_t now_ms, uint32_t timeout_ms);

/* ---- 遥测发送（非阻塞 DMA）---- */

/* 姿态: pitch/roll/yaw × 10000 弧度 */
void APP_ELRS_SendTelemetryAttitude(int16_t pitch_rad_x10000,
                                    int16_t roll_rad_x10000,
                                    int16_t yaw_rad_x10000);

/* 气压计高度: 分米 */
void APP_ELRS_SendTelemetryBaro(int32_t altitude_dm);

/* 电池: 电压 0.1V, 电流 0.1A, 容量 mAh, 剩余 % */
uint8_t APP_ELRS_SendTelemetryBattery(uint16_t voltage_dv,
                                   uint16_t current_da,
                                   uint32_t capacity_mah,
                                   uint8_t remaining_pct);

/* GPS: lat/lon ×1e7 °, 速度 0.1 km/h, 航向 0.01 °, 高度 m+1000, 卫星数 */
void APP_ELRS_SendTelemetryGps(int32_t lat_e7, int32_t lon_e7,
                               uint16_t speed_kmh_x10,
                               uint16_t heading_deg_x100,
                               uint16_t altitude_m, uint8_t satellites);

/* ---- HAL 回调（ISR 上下文，仅做事件标记）---- */
void APP_ELRS_OnRxEvent(uint16_t size);
void APP_ELRS_OnTxComplete(void);
void APP_ELRS_OnError(void);

/* ---- 诊断 ---- */
uint32_t APP_ELRS_GetRcFrames(void);
uint32_t APP_ELRS_GetCrcErrors(void);
const DRV_ELRS_LinkStats *APP_ELRS_GetLinkStats(void);

/*
 * UART4 收侧的分项计数。`crc_err` 只说"帧坏了"，说不出坏在哪一层，而这两层
 * 的处置完全相反：
 *
 *   ORE（溢出）  = 固件没及时把 DMA 里的字节取走 —— 软件问题，改调用节奏或缓冲。
 *   FE/NE（帧错 / 噪声） = 线上电平本身就不对 —— 波特率、走线、地线，改代码没用。
 *   aborts       = ClearErrors() 因上面任一标志整条 DMA 重启的次数。**一次重启
 *                  就丢掉整个未消费缓冲并让解析器失步**，所以它同时是 CRC 错的
 *                  放大器，必须单独看得见。
 *
 * **这些计数不进任何命令回包，只能用 SWD 直读**（作者裁决 2026-09-06）。
 * 能挂的那几个报告函数（`app_control_report_rc_live`、`app_control_report_uart_stats`
 * 等）都被 D2/D4 的 SHA256 钉住，那些哈希证明的是"当年从 app_control.c 逐字节
 * 搬过来、一个字符没动"；为一组排查用的计数去破坏那个证明不划算。
 *
 * 于是**符号名本身就是接口**：`tools/elrs_link_diag.py` 用 `nm` 从 ELF 找地址，
 * 再以 HOTPLUG 方式读内存——不复位、不打断飞控。改名等于改接口，
 * `tests/test_crsf_parser_resync.py` 有机检把关。
 */
typedef struct {
    uint32_t overrun;    /* ORE */
    uint32_t framing;    /* FE  */
    uint32_t noise;      /* NE  */
    uint32_t parity;     /* PE  */
    uint32_t aborts;     /* 因错误标志整条重启 RX DMA 的次数 */
    uint32_t restarts;   /* StartRxDma() 总次数（含首次） */
    uint32_t events;     /* HAL RxEvent 回调次数 */
    uint32_t start_fail; /* StartRxDma() 起不来的次数：恢复路径本身失灵 */
} APP_ELRS_RxDiag;

void APP_ELRS_GetRxDiag(APP_ELRS_RxDiag *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_ELRS_H */
