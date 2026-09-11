#ifndef APP_MAINT_UART_H
#define APP_MAINT_UART_H

#include <stdint.h>

/*
 * 维护 / 调参链路的应用层：收行、分发命令、回文本、把遥测帧推出去。
 *
 * 本文件**不认识任何 UART 实例号**。"哪个口是维护口"由 BSP/Src/bsp_uart.c
 * 一处绑定（本板是板载蓝牙所在的 UART8）。换板子只改那一处，这里不动——
 * 这次移植的经验是：实例名一旦渗进上层逻辑，换板就得逐个文件重写。
 */

void APP_MaintUART_Init(void);
void APP_MaintUART_Step(void);

/*
 * 文本写。**只能从任务上下文调用**（放不下时会让出 CPU 等队列腾出空间，
 * 等不到才丢并计数）。命令回包走这条路：回包丢了比慢危险得多。
 */
void APP_MaintUART_Write(const char *text, uint16_t length);

/*
 * 二进制安全的写，给遥测帧用。遥测帧里含 0x00，任何按 C 字符串处理的路径都会
 * 把它截断，所以出口另开一个入口，而不是把 const char* 那个强转着用。
 *
 * 与文本写的区别不只在类型：这条**从不等待**。队列里已经积着一帧以上时直接
 * 丢弃并返回 0，由调用方计入 drop。理由是波形要的是新鲜，不是完整——排在
 * 一百毫秒队尾才出门的旧帧，画出来的曲线是错的。
 *
 * 返回 0 = 没排进去（调用方应计 drop）。
 */
uint8_t APP_MaintUART_WriteRaw(const uint8_t *data, uint16_t length);

/* `TELEM TX`：发送队列画像（水位、丢帧、DMA 段数、是否退化成阻塞发送）。 */
void APP_MaintUART_ReportTx(void);
void APP_MaintUART_ResetTxStats(void);

/*
 * 蓝牙链路最近是否在用（收到过命令行且未超时）。
 *
 * 用来决定要不要把异步文本也镜像一份到蓝牙：不加这个判断的话，没连蓝牙时
 * 每条结构化文本都会往一条没人收的链路上排队，最后把发送队列填满，
 * 连命令回包都得等位；而一旦连上，蓝牙就该和 USB 一样能收到 READY、心跳
 * 这些没人问也该来的行。
 */
uint8_t APP_MaintUART_IsLinkActive(void);
void APP_MaintUART_WriteFormat(const char *format, ...);

/*
 * 中断上下文的两个入口。实例匹配由 BSP 负责（BSP_UART_IsMaint），
 * 所以这里不需要、也拿不到 HAL 句柄。
 */
void APP_MaintUART_OnRxByte(void);
void APP_MaintUART_OnRxError(void);

#endif
