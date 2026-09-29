#ifndef APP_ESC_LOG_H
#define APP_ESC_LOG_H
#include <stdint.h>

/* FlightLog V11 extension, little-endian 4B2H6I, captured at Observe time.
 * present=0 means no DShot backend. Counters are since explicit BSP Init.
 * Codes are last accepted values, not a response/acknowledgement from an ESC.
 */
/*
 * `code_ch1`/`code_ch2` 按**硬件通道**编号，不按上/下桨。
 *
 * 2026-09-13 之前它们叫 upper_code / lower_code，那时候通道 1 恒等于上桨。
 * 现在归属由上位机标定（`Driver/Inc/drv_prop_map.h`），通道 1 接哪个电机是装配
 * 决定的，旧名字会在接线交换时变成假的。二进制布局一字未动，改的只是名字——
 * 一个说谎的列名比没有这一列更难查，因为没人会去怀疑它。
 */
typedef struct __attribute__((packed)) {
    uint8_t present, enabled_mask, busy, fault;
    uint16_t code_ch1, code_ch2;
    uint32_t submitted, completed, busy_rejected, errors, cancelled, timer_clock_hz;
} APP_EscLog;

APP_EscLog APP_EscLog_Capture(void);
#endif
