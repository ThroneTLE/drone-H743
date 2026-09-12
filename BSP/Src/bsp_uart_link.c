#include "bsp_uart_link.h"

#include "bsp_uart.h"

#include "usart.h"

#include <stddef.h>

/*
 * ============================ 换板时改的就是这张表 ============================
 *
 * 角色 → 句柄，以及每条链路的接收脚偏置（接收机没插时悬空的 RX 会把 UART 的
 * 帧错刷满，偏置到总线的空闲电平才安静）。
 *
 * 维护口的句柄不在这里重复写：它的唯一事实源在 bsp_uart.c。
 */
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *rx_port;
    uint16_t            rx_pin;
    uint32_t            rx_pull;
    uint32_t            rx_alternate;
} BSP_UartLinkBinding;

static const BSP_UartLinkBinding *bsp_uart_link_binding(BSP_UartRole role)
{
    static const BSP_UartLinkBinding telemetry = {
        &huart1, NULL, 0U, 0U, 0U
    };
    /*
     * ELRS/CRSF 在 USART6_RX = PC7，空闲电平是高，所以上拉。
     * 接收机未上电时这条线是悬空的；不加偏置的话噪声会持续触发帧错。
     */
    static const BSP_UartLinkBinding rc = {
        &huart6, GPIOC, GPIO_PIN_7, GPIO_PULLUP, GPIO_AF7_USART6
    };
    static BSP_UartLinkBinding maint;   /* 句柄从 bsp_uart.c 取，不在这儿写死 */

    switch (role) {
    case BSP_UART_ROLE_TELEMETRY:
        return &telemetry;
    case BSP_UART_ROLE_RC:
        return &rc;
    case BSP_UART_ROLE_MAINT:
        maint.huart = BSP_UART_MaintHandle();
        return (maint.huart != NULL) ? &maint : NULL;
    default:
        return NULL;
    }
}

static UART_HandleTypeDef *bsp_uart_link_handle(BSP_UartRole role)
{
    const BSP_UartLinkBinding *binding = bsp_uart_link_binding(role);

    return (binding != NULL) ? binding->huart : NULL;
}

uint8_t BSP_UartLink_HasRxDma(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    return ((huart != NULL) && (huart->hdmarx != NULL)) ? 1U : 0U;
}

uint8_t BSP_UartLink_HasTxDma(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    return ((huart != NULL) && (huart->hdmatx != NULL)) ? 1U : 0U;
}

uint8_t BSP_UartLink_RxIsRunning(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);
    const DMA_Stream_TypeDef *stream;

    if (huart == NULL) {
        return 0U;
    }
    if (huart->RxState != HAL_UART_STATE_BUSY_RX) {
        return 0U;
    }
    if (huart->hdmarx == NULL) {
        return 1U;      /* 纯中断接收：HAL 的状态就是全部真相 */
    }

    /*
     * DMA 还要单独看使能位：HAL 的 RxState 在传输异常中止后不一定跟着回退，
     * 只信它会得到"还在收"的假象，而实际上一个字节都不会再进来。
     */
    stream = (const DMA_Stream_TypeDef *)huart->hdmarx->Instance;
    return ((stream->CR & DMA_SxCR_EN) != 0U) ? 1U : 0U;
}

uint8_t BSP_UartLink_StartRxToIdle(BSP_UartRole role, uint8_t *buffer,
                                   uint16_t size)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if ((huart == NULL) || (buffer == NULL) || (size == 0U) ||
        (huart->hdmarx == NULL)) {
        return 0U;
    }

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size) != HAL_OK) {
        return 0U;
    }

    /* 半满事件只会让上层多醒一次，要的是"一帧收完了"。 */
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return 1U;
}

uint8_t BSP_UartLink_StartRxToIdleIt(BSP_UartRole role, uint8_t *buffer,
                                     uint16_t size)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if ((huart == NULL) || (buffer == NULL) || (size == 0U)) {
        return 0U;
    }

    return (HAL_UARTEx_ReceiveToIdle_IT(huart, buffer, size) == HAL_OK) ? 1U : 0U;
}

uint16_t BSP_UartLink_RxFilled(BSP_UartRole role, uint16_t size)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);
    uint32_t remaining;

    if ((huart == NULL) || (huart->hdmarx == NULL) || (size == 0U)) {
        return 0U;
    }

    remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > (uint32_t)size) {
        return 0U;
    }

    return (uint16_t)((uint32_t)size - remaining);
}

void BSP_UartLink_AbortRx(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if (huart != NULL) {
        (void)HAL_UART_AbortReceive(huart);
    }
}

uint32_t BSP_UartLink_TakeErrors(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);
    uint32_t isr;
    uint32_t hal_error;
    uint32_t flags = 0U;

    if (huart == NULL) {
        return 0U;
    }

    /*
     * 两个来源都要看：HAL 的 ErrorCode 只有在错误中断开着时才会被填，而这条
     * 链路可能刻意关掉了那些中断（见 DisableRxErrorInterrupts），那时真相只在
     * ISR 的硬件标志位上。
     */
    isr = huart->Instance->ISR;
    hal_error = HAL_UART_GetError(huart);

    if (((isr & USART_ISR_ORE) != 0U) ||
        ((hal_error & HAL_UART_ERROR_ORE) != 0U)) {
        flags |= BSP_UART_LINK_ERR_OVERRUN;
    }
    if (((isr & USART_ISR_FE) != 0U) ||
        ((hal_error & HAL_UART_ERROR_FE) != 0U)) {
        flags |= BSP_UART_LINK_ERR_FRAMING;
    }
    if (((isr & USART_ISR_NE) != 0U) ||
        ((hal_error & HAL_UART_ERROR_NE) != 0U)) {
        flags |= BSP_UART_LINK_ERR_NOISE;
    }
    if (((isr & USART_ISR_PE) != 0U) ||
        ((hal_error & HAL_UART_ERROR_PE) != 0U)) {
        flags |= BSP_UART_LINK_ERR_PARITY;
    }
    if ((isr & USART_ISR_RTOF) != 0U) {
        flags |= BSP_UART_LINK_ERR_TIMEOUT;
    }

    if (flags != 0U) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF | UART_CLEAR_FEF |
                                     UART_CLEAR_RTOF | UART_CLEAR_IDLEF);
    }
    return flags;
}

void BSP_UartLink_FlushRx(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if (huart == NULL) {
        return;
    }

    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
}

uint8_t BSP_UartLink_TxIsIdle(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if (huart == NULL) {
        return 0U;
    }

    return ((huart->gState == HAL_UART_STATE_READY) &&
            (huart->ErrorCode == HAL_UART_ERROR_NONE)) ? 1U : 0U;
}

uint8_t BSP_UartLink_TxIsComplete(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);
    uint32_t cr1;
    uint32_t cr3;
    uint32_t isr;

    if (huart == NULL) {
        return 0U;
    }

    /* 驱动自己说完事了，那就是完事了。 */
    if ((huart->gState == HAL_UART_STATE_READY) &&
        (huart->ErrorCode == HAL_UART_ERROR_NONE)) {
        return 1U;
    }

    /*
     * 驱动还没销账，但硬件已经安静：DMA 请求位关了、发送完成中断也关了、
     * 而 TC 标志立着——说明最后一个字节的停止位已经走完，只是那次回调没到。
     * H7 上 TX DMA 是两段式（DMA 传完清 DMAT 并开 TCIE，再由 UART 的 TC 中断
     * 回调上层），中间任一段被错过都会卡在这儿。
     */
    cr1 = huart->Instance->CR1;
    cr3 = huart->Instance->CR3;
    isr = huart->Instance->ISR;

    return (((cr3 & USART_CR3_DMAT) == 0U) &&
            ((cr1 & USART_CR1_TCIE) == 0U) &&
            ((isr & USART_ISR_TC) != 0U) &&
            (huart->ErrorCode == HAL_UART_ERROR_NONE)) ? 1U : 0U;
}

uint8_t BSP_UartLink_TransmitDma(BSP_UartRole role, const uint8_t *data,
                                 uint16_t length)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if ((huart == NULL) || (data == NULL) || (length == 0U) ||
        (huart->hdmatx == NULL)) {
        return 0U;
    }

    return (HAL_UART_Transmit_DMA(huart, data, length) == HAL_OK) ? 1U : 0U;
}

void BSP_UartLink_DisableRxErrorInterrupts(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if (huart == NULL) {
        return;
    }

    CLEAR_BIT(huart->Instance->CR1,
              USART_CR1_IDLEIE | USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE | USART_CR3_RXFTIE);
}

void BSP_UartLink_ApplyRxBias(BSP_UartRole role)
{
    const BSP_UartLinkBinding *binding = bsp_uart_link_binding(role);
    GPIO_InitTypeDef init = {0};

    if ((binding == NULL) || (binding->rx_port == NULL)) {
        return;     /* 这条链路不需要偏置 */
    }

    init.Pin       = binding->rx_pin;
    init.Mode      = GPIO_MODE_AF_PP;
    init.Pull      = binding->rx_pull;
    init.Speed     = GPIO_SPEED_FREQ_LOW;
    init.Alternate = binding->rx_alternate;
    HAL_GPIO_Init(binding->rx_port, &init);
}

void BSP_UartLink_Release(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);

    if (huart != NULL) {
        (void)HAL_UART_DeInit(huart);
    }
}

void BSP_UartLink_ForceTxDmaNormalMode(BSP_UartRole role)
{
    UART_HandleTypeDef *huart = bsp_uart_link_handle(role);
    DMA_HandleTypeDef  *hdma;

    if (huart == NULL) {
        return;
    }

    hdma = huart->hdmatx;
    if ((hdma == NULL) || (hdma->Init.Mode == DMA_NORMAL)) {
        return;
    }

    (void)HAL_DMA_Abort(hdma);
    (void)HAL_DMA_DeInit(hdma);
    hdma->Init.Mode = DMA_NORMAL;
    (void)HAL_DMA_Init(hdma);
}
