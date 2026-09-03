#ifndef APP_VOFA_H
#define APP_VOFA_H

#include <stdint.h>

/* VOFA+ JustFloat protocol:
 *   [float32 LE data × N] [tail: 0x00 0x00 0x80 0x7f]
 *
 * Max floats per frame (limited by stack buffer, UART frame, and uint8_t count).
 * 64 floats × 4 bytes + 4 byte tail = 260 bytes — fits a small stack buffer.
 */
#define APP_VOFA_MAX_FLOATS 64U

/*
 * R-T1-1 起本模块退化为**数传出口的发送后端**：
 *   - APP_VOFA_SendRaw   —— 把已经成好的字节（遥测流 v2 的 $X 掩码帧）原样发出；
 *   - APP_VOFA_SendFloats —— 旧 JustFloat 定长帧，`TELEM FORMAT jf` 的 Synex 过渡用。
 * 通道装配与格式选择都在 app_telem_stream.c，不在这里。
 */

/* Send N floats as a VOFA JustFloat frame over USART1. Returns 1 when queued. */
uint8_t APP_VOFA_SendFloats(const float *data, uint8_t count);

/* Send already-framed bytes over the same USART1 path. Returns 1 when queued. */
uint8_t APP_VOFA_SendRaw(const uint8_t *data, uint16_t length);

#endif /* APP_VOFA_H */
