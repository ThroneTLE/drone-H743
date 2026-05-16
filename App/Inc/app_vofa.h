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

/* Send N floats as a VOFA JustFloat frame over USART1 (blocking). */
void APP_VOFA_SendFloats(const float *data, uint8_t count);

#endif /* APP_VOFA_H */
