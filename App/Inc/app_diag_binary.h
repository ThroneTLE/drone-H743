#ifndef APP_DIAG_BINARY_H
#define APP_DIAG_BINARY_H
#include <stdint.h>
uint8_t APP_Diag_SendBinary(uint16_t function,const uint8_t *payload,uint16_t length);
#endif
