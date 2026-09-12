#ifndef BSP_UART_EVENTS_H
#define BSP_UART_EVENTS_H

#include <stdint.h>

/*
 * UART 中断事件的板级分发。
 *
 * === 这里解决的是什么 ===
 *
 * HAL 把所有 UART 的收发完成、IDLE、出错都汇到四个**弱回调**里，参数只有一个
 * `UART_HandleTypeDef *`。谁来实现这四个函数、以及"哪个实例归谁"这张表放哪，
 * 决定了换板时要改几个文件。
 *
 * 之前这张表在 `App/Src/app_uart.c`，写成四条 `if (huart->Instance == USART6)`
 * 的链。于是 App 层同时知道了 USART1/2/6、UART7/8 这些实例名，而这次移植里
 * 它们**全都变了**。现在表在 BSP（bsp_uart_events.c 一处），上层只按**角色**
 * 注册；换板改那一张表，App 一行不动。
 *
 * === 分发方向的规矩 ===
 *
 * - **向下**（BSP → BSP/Driver，例如光流、总线舵机）直接调，不注册：
 *   本来就是同层或下层，加一层注册只是噪音。
 * - **向上**（BSP → App）一律经本文件的注册口：BSP 不该 `#include` App 的头，
 *   否则层次就倒过来了。
 *
 * === 契约 ===
 *
 * - 全部回调都在**中断上下文**执行：只许搬数据 / 置标志 / 唤醒任务（D2-1）。
 * - 未注册的角色收到事件时静默丢弃，不是错误——GPS 任务当前就没启用。
 * - 注册要在开中断收发之前做完（各模块自己的 Init 里）。
 */

typedef enum {
    BSP_UART_ROLE_TELEMETRY = 0, /* 数传 / 调参 */
    BSP_UART_ROLE_MAINT,         /* 维护口（本板 = 板载蓝牙） */
    BSP_UART_ROLE_RC,            /* 遥控接收机（ELRS / CRSF） */
    BSP_UART_ROLE_COUNT
} BSP_UartRole;

typedef struct {
    /* 收到 IDLE / 半满事件，size = 本次已收字节数。 */
    void (*rx_event)(uint16_t size);
    /* 逐字节接收完成一个。 */
    void (*rx_byte)(void);
    /* 发送完成。 */
    void (*tx_cplt)(void);
    /* 该口出错（溢出 / 噪声 / 帧错）。 */
    void (*error)(void);
} BSP_UartRoleHandlers;

/* handlers 为 NULL 表示注销。结构体由调用方保有，BSP 只存指针。 */
void BSP_UartEvents_Register(BSP_UartRole role,
                             const BSP_UartRoleHandlers *handlers);

#endif /* BSP_UART_EVENTS_H */
