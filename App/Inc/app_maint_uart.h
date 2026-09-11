#ifndef APP_MAINT_UART_H
#define APP_MAINT_UART_H

#include <stdint.h>
#include "main.h"

void APP_MaintUART_Init(void);
void APP_MaintUART_Step(void);
void APP_MaintUART_Write(const char *text, uint16_t length);

/*
 * 二进制安全的写。遥测帧里含 0x00，任何按 C 字符串处理的路径都会把它截断，
 * 所以出口另开一个入口，而不是把 const char* 那个强转着用。
 */
void APP_MaintUART_WriteRaw(const uint8_t *data, uint16_t length);

/*
 * 蓝牙链路最近是否在用（收到过命令行且未超时）。
 *
 * 用来决定要不要把异步文本也镜像一份到蓝牙：不加这个判断的话，没连蓝牙时
 * 每条结构化文本都要在 UART8 上阻塞发一遍，白白拖慢 UART 任务；而一旦连上，
 * 蓝牙就该和 USB 一样能收到 READY、心跳这些没人问也该来的行。
 */
uint8_t APP_MaintUART_IsLinkActive(void);
void APP_MaintUART_WriteFormat(const char *format, ...);
void APP_MaintUART_OnRxCplt(UART_HandleTypeDef *huart);
void APP_MaintUART_OnError(UART_HandleTypeDef *huart);

#endif
