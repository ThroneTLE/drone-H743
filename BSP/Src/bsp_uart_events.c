#include "bsp_uart_events.h"

#include "bsp_bus_servo.h"
#include "bsp_optical_flow.h"
#include "bsp_uart.h"

#include "drv_servo.h"

#include "usart.h"

#include <stddef.h>

/*
 * ============================ 换板时改的就是这张表 ============================
 *
 * 角色 → UART 实例。除了这张表和 bsp_uart.c 顶部的维护口绑定，整个仓库里再没有
 * 第二处把"某个功能"和"某个 UART 实例"钉在一起的地方。
 *
 * 这两条不在表里，因为它们的事件直接向下分发（见文件头的方向规矩）：
 *   光流  -> UART4    总线舵机 -> UART7
 * 光流 2026-09-29 从 USART2 迁到 UART4（4 针 5 V 口）；这里须与 bsp_board.c 的
 * optical_flow_bus 保持一致（tests/test_optical_flow_contract.py 核对）。
 * USART2（DJI 图传口）与 GPS（USART3）目前都没有消费者：注册表里留空即可，
 * 事件会被静默丢弃——这是正常状态，不是错误。
 */
#define BSP_UART_EVENTS_TELEMETRY_INSTANCE USART1
#define BSP_UART_EVENTS_RC_INSTANCE        USART6
#define BSP_UART_EVENTS_FLOW_INSTANCE      UART4
#define BSP_UART_EVENTS_SERVO_INSTANCE     UART7

static const BSP_UartRoleHandlers *bsp_uart_role_handlers[BSP_UART_ROLE_COUNT];

void BSP_UartEvents_Register(BSP_UartRole role,
                             const BSP_UartRoleHandlers *handlers)
{
    if (role >= BSP_UART_ROLE_COUNT) {
        return;
    }

    bsp_uart_role_handlers[role] = handlers;
}

/*
 * 实例 → 角色。认不出来返回 BSP_UART_ROLE_COUNT。
 *
 * 维护口不写死实例，问 bsp_uart.c——"哪个 UART 是维护口"的唯一事实源在那里，
 * 在这儿再抄一份就等于有了两个事实源，而两处不一致的症状是"命令进得来、
 * 回包出不去"，两边单看都对。
 */
static BSP_UartRole bsp_uart_role_of(const UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return BSP_UART_ROLE_COUNT;
    }
    if (BSP_UART_IsMaint(huart) != 0U) {
        return BSP_UART_ROLE_MAINT;
    }
    if (huart->Instance == BSP_UART_EVENTS_TELEMETRY_INSTANCE) {
        return BSP_UART_ROLE_TELEMETRY;
    }
    if (huart->Instance == BSP_UART_EVENTS_RC_INSTANCE) {
        return BSP_UART_ROLE_RC;
    }
    return BSP_UART_ROLE_COUNT;
}

static const BSP_UartRoleHandlers *bsp_uart_handlers_of(
    const UART_HandleTypeDef *huart)
{
    BSP_UartRole role = bsp_uart_role_of(huart);

    return (role < BSP_UART_ROLE_COUNT) ? bsp_uart_role_handlers[role] : NULL;
}

/* ------------------------------------------------------------ HAL 弱回调 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    const BSP_UartRoleHandlers *handlers;

    if (huart->Instance == BSP_UART_EVENTS_FLOW_INSTANCE) {
        BSP_OPTICAL_FLOW_OnUartRxEvent(huart, Size);
        return;
    }

    handlers = bsp_uart_handlers_of(huart);
    if ((handlers != NULL) && (handlers->rx_event != NULL)) {
        handlers->rx_event(Size);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    const BSP_UartRoleHandlers *handlers;

    if (huart->Instance == BSP_UART_EVENTS_SERVO_INSTANCE) {
        DRV_SERVO_OnUartRxComplete(huart);
        return;
    }

    handlers = bsp_uart_handlers_of(huart);
    if ((handlers != NULL) && (handlers->rx_byte != NULL)) {
        handlers->rx_byte();
        return;
    }

    /* 光流以前靠"谁都没认领就归它"收到这个回调，保持原样。 */
    BSP_OPTICAL_FLOW_OnUartRxCplt(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    const BSP_UartRoleHandlers *handlers;

    if (huart->Instance == BSP_UART_EVENTS_SERVO_INSTANCE) {
        DRV_SERVO_OnUartTxComplete(huart);
        return;
    }
    if (BSP_UART_IsMaint(huart) != 0U) {
        /* 维护口的发送队列在中断里直接续发下一段，见 bsp_uart_tx.c。 */
        BSP_UART_MaintOnTxComplete(huart);
        return;
    }

    handlers = bsp_uart_handlers_of(huart);
    if ((handlers != NULL) && (handlers->tx_cplt != NULL)) {
        handlers->tx_cplt();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    const BSP_UartRoleHandlers *handlers;

    if (huart->Instance == BSP_UART_EVENTS_SERVO_INSTANCE) {
        DRV_SERVO_OnUartError(huart);
        return;
    }
    if (BSP_UART_IsMaint(huart) != 0U) {
        /*
         * 收发两侧都要认领这次出错：只处理接收的话，正在传的那一段 DMA 会把
         * busy 永远挂住，整条发送队列从此一个字节都出不去。
         */
        BSP_UART_MaintOnTxError(huart);
    }

    handlers = bsp_uart_handlers_of(huart);
    if ((handlers != NULL) && (handlers->error != NULL)) {
        handlers->error();
    }

    if (bsp_uart_role_of(huart) == BSP_UART_ROLE_COUNT) {
        /* 没人认领的实例：光流走的就是这条路（与改造前的顺序一致）。 */
        BSP_OPTICAL_FLOW_OnUartError(huart);
    }
}
