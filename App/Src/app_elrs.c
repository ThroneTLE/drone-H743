#include "app_elrs.h"

#include "usart.h"
#include "bsp_cache.h"

#include <string.h>

/* ---- constants ---- */

#define APP_ELRS_DMA_RX_SIZE      256U
#define APP_ELRS_TX_BUF_SIZE      CRSF_MAX_FRAME_SIZE

/* ---- DMA buffers in RAM_D2 ---- */

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t dma_rx_buf[APP_ELRS_DMA_RX_SIZE];

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t tx_buf[APP_ELRS_TX_BUF_SIZE];

/* ---- state ---- */

static uint16_t dma_rx_pos;
static volatile uint8_t dma_started;
static volatile uint8_t tx_busy;
static uint8_t  tx_len;
static uint32_t rx_events;
static uint32_t rx_errors;
static uint32_t rx_restarts;
/* 分项计数：见 app_elrs.h 里 APP_ELRS_RxDiag 的说明。 */
static uint32_t rx_err_overrun;
static uint32_t rx_err_framing;
static uint32_t rx_err_noise;
static uint32_t rx_err_parity;
static uint32_t rx_aborts;
/* StartRxDma() 起不来的次数。恢复路径本身失灵是"链路整条死掉"的唯一解释，
 * 必须和"帧偶尔坏"分开看：前者 total/rc 会一直是 0，后者只是比例变差。 */
static uint32_t rx_start_fail;

/* ---- helpers ---- */

/*
 * 给 USART6_RX 加上拉：接收机没插或没上电时线是浮空的，浮空线会被当成随机电平，
 * 帧错误计数一路涨。上拉把空闲态钉在高电平（UART 空闲就是高）。
 *
 * **引脚必须与 CubeMX 给 USART6_RX 分配的那个脚一致。** 这里一度写着老板子的
 * PD0，移植时改成过 UART4 的 PA1。把第二个脚也配成同一个 AF，等于把两个 GPIO
 * 接到同一路外设输入上——ST 参考手册明确要求一个 AF 输入只能由一个引脚提供，
 * 实际表现取决于硅片内部怎么合并这两路信号，最坏情况是 RC 链路整条收不到。
 * tests/test_micoair743v2_review_fixes.py 会拿 .ioc 的分配来核对这两个宏。
 *
 * 落在 USART6/PC6/PC7 是因为**那才是板子丝印上的 RC 口**：
 * ArduPilot hwdef 记 USART6 = RCIN，PX4 记 CONFIG_BOARD_SERIAL_RC="/dev/ttyS5"
 * （2026-09-10 在实物上用出厂 PX4 的 `rc_input status` 确认过就是 ttyS5）。
 */
#define ELRS_RX_GPIO_PORT GPIOC
#define ELRS_RX_GPIO_PIN  GPIO_PIN_7

static void ConfigureRxPinBias(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = ELRS_RX_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
    HAL_GPIO_Init(ELRS_RX_GPIO_PORT, &GPIO_InitStruct);
}

static void SuppressRxIrqSources(void)
{
    /*
     * CRSF RX is consumed by polling the circular DMA write position in
     * APP_ELRS_Step(). Leaving UART IDLE/error IRQs enabled lets a noisy or
     * floating receiver line trap the CPU in UART4_IRQHandler.
     */
    CLEAR_BIT(huart6.Instance->CR1,
              USART_CR1_IDLEIE | USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
    CLEAR_BIT(huart6.Instance->CR3, USART_CR3_EIE | USART_CR3_RXFTIE);
}

static void ClearErrors(void)
{
    uint32_t err = HAL_UART_GetError(&huart6);
    uint32_t flags = huart6.Instance->ISR;
    uint32_t error_flags = flags & (USART_ISR_PE | USART_ISR_FE |
                                    USART_ISR_NE | USART_ISR_ORE |
                                    USART_ISR_RTOF);

    if ((err == HAL_UART_ERROR_NONE) && (error_flags == 0U)) return;

    /* 先按标志分类再清，否则清完就分不出是哪一层坏的。 */
    if ((error_flags & USART_ISR_ORE) != 0U) rx_err_overrun++;
    if ((error_flags & USART_ISR_FE)  != 0U) rx_err_framing++;
    if ((error_flags & USART_ISR_NE)  != 0U) rx_err_noise++;
    if ((error_flags & USART_ISR_PE)  != 0U) rx_err_parity++;
    rx_errors++;

    __HAL_UART_CLEAR_FLAG(&huart6,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF |
                          UART_CLEAR_RTOF | UART_CLEAR_IDLEF);

    /*
     * 逢错就整条重整流：flush FIFO + Abort + 重启 DMA。
     *
     * 看着很重，但**它不是开销，是前置过滤器**：`UART_RXDATA_FLUSH_REQUEST` 把
     * 出错的那个字节在进入解析器之前就扔掉了，所以坏字节根本不会变成坏帧。
     * 收窄它试过两次，两次都更差，实测（2026-09-06，ELRS 1000 Hz 直连）：
     *
     *   逢错就重整流         真帧 919~948/s   crc_err  8~28/s   ← 当前
     *   只在 ORE 时重整流    真帧 868~883/s   crc_err 74~84/s
     *   完全不重整流（早期） 真帧 280/s       crc_err 700+/s
     *
     * 注意第一档里 `crc_err` **比物理坏字节数（fe+ne≈44/s）还少**——这正是
     * "坏字节没能走到解析器"的直接证据。第二档 crc_err≈1.4×(fe+ne)，是坏字节
     * 打穿一帧、偶尔连累下一帧（撞到长度字节时）的比例。
     *
     * 早期那次完全撤掉之所以灾难性，还叠加了另一个原因：当时
     * `Crsf_IsCommonAddress()` 把 26% 的字节值当成地址，失步要连打七八个假帧
     * 才撞回真帧头。地址判据收窄之后这一项已经不成立了，但即便如此，前置过滤
     * 仍然赢——所以这条路径保留。
     */
    rx_aborts++;
    __HAL_UART_SEND_REQ(&huart6, UART_RXDATA_FLUSH_REQUEST);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
    dma_started = 0U;
    (void)HAL_UART_AbortReceive(&huart6);
}

static void StartRxDma(void)
{
    if (huart6.hdmarx == NULL)
        return;

    dma_rx_pos = 0U;
    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, dma_rx_buf, APP_ELRS_DMA_RX_SIZE);
    if (status != HAL_OK) {
        rx_errors++;
        rx_start_fail++;
        /*
         * 起不来只有一个原因：HAL 的 RxState 不在 READY。下一拍再调还是 BUSY，
         * 光靠重试永远出不来。必须在这里显式收尾——从前是靠 ClearErrors() 里
         * 那个"逢错就 Abort"顺带把状态机复位的，把那条路径收窄之后，这里就是
         * **唯一**的自愈点；缺了它，开机头几拍起不来就等于遥控链路整条死掉
         * （实测 sfail=3：上电确实会失败几次）。
         */
        (void)HAL_UART_AbortReceive(&huart6);
        return;
    }

    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
    SuppressRxIrqSources();
    BSP_Cache_InvalidateDCache(dma_rx_buf, APP_ELRS_DMA_RX_SIZE);
    /* 重启 = 字节流断了一截，解析器手上的半帧已经无意义，留着必然拼出一个坏帧。 */
    DRV_ELRS_ResetParser();
    dma_started = 1U;
    rx_restarts++;
}

static uint8_t DmaNeedsRestart(void)
{
    DMA_HandleTypeDef *hdma = huart6.hdmarx;
    if (hdma == NULL) return 1U;
    if (dma_started == 0U) return 1U;
    if (huart6.RxState != HAL_UART_STATE_BUSY_RX) return 1U;

    DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)hdma->Instance;
    return ((stream->CR & DMA_SxCR_EN) == 0U) ? 1U : 0U;
}

static uint16_t DmaWritePos(void)
{
    DMA_HandleTypeDef *hdma = huart6.hdmarx;
    if (hdma == NULL || dma_started == 0U)
        return dma_rx_pos;

    uint32_t remaining = __HAL_DMA_GET_COUNTER(hdma);
    if (remaining > APP_ELRS_DMA_RX_SIZE)
        return dma_rx_pos;
    return (uint16_t)(APP_ELRS_DMA_RX_SIZE - remaining);
}

/* ---- telemetry TX ---- */

static void StartTxDma(const uint8_t *frame, uint8_t len)
{
    if (huart6.hdmatx == NULL)
        return;

    tx_busy = 1U;
    memcpy(tx_buf, frame, len);
    tx_len = len;
    BSP_Cache_CleanDCache(tx_buf, len);

    if (HAL_UART_Transmit_DMA(&huart6, tx_buf, len) != HAL_OK) {
        tx_busy = 0U;
        rx_errors++;
    }
}

static void SendTelemetryFrame(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t frame[CRSF_MAX_FRAME_SIZE];
    uint8_t len = DRV_ELRS_BuildTelemetry(type, payload, payload_len, frame);
    if (len == 0U) return;

    if (tx_busy != 0U)
        return;  /* drop — previous frame still in flight */

    StartTxDma(frame, len);
}

/* ---- public API ---- */

void APP_ELRS_Init(void)
{
    DRV_ELRS_Init();
    ConfigureRxPinBias();

    dma_rx_pos  = 0U;
    dma_started = 0U;
    tx_busy     = 0U;
    tx_len      = 0U;
    rx_events   = 0U;
    rx_errors   = 0U;
    rx_restarts = 0U;
    rx_err_overrun = 0U;
    rx_err_framing = 0U;
    rx_err_noise   = 0U;
    rx_err_parity  = 0U;
    rx_aborts      = 0U;
    rx_start_fail  = 0U;

    StartRxDma();
}

void APP_ELRS_Step(void)
{
    ClearErrors();

    /* consume new bytes from DMA circular buffer */
    BSP_Cache_InvalidateDCache(dma_rx_buf, APP_ELRS_DMA_RX_SIZE);
    uint16_t write_pos = DmaWritePos();
    uint32_t consumed  = 0U;

    while (dma_rx_pos != write_pos) {
        if (DRV_ELRS_ProcessByte(dma_rx_buf[dma_rx_pos]) != 0U) {
            DRV_ELRS_MarkRcFrameTime(HAL_GetTick());
        }
        dma_rx_pos++;
        consumed++;
        if (dma_rx_pos >= APP_ELRS_DMA_RX_SIZE)
            dma_rx_pos = 0U;
    }

    /*
     * 帧间空闲 = 确定的重同步点。
     *
     * CRSF 是靠"地址字节"起头的字节流，没有转义也没有帧定界符，而
     * `Crsf_IsCommonAddress()` 有 26% 的字节值会被当成地址。所以一旦失步，解析器
     * 就在 payload 里一个字节一个字节地撞运气，一次失步能连打十几二十个假帧，
     * 期间真帧全被吃掉——实测失步状态下 crc_err 高达 700/s 而真帧只剩 280/s。
     *
     * 但链路本身给了一个干净的定界：500 Hz 下 26 字节一帧只占 0.62 ms，帧与帧
     * 之间有约 1.4 ms 空闲。本函数由 IMU 就绪信号量驱动、约 1 kHz 调用，所以
     * "这一拍一个新字节都没来"就等价于"现在落在帧间空隙里"。此刻解析器手上若
     * 还攥着半帧，那半帧永远等不到剩下的字节，留着只会和下一帧的头拼成假帧。
     *
     * 丢掉它，下一个字节就必然是真正的帧头——失步的代价从"十几帧"压回"一帧"。
     */
    if (consumed == 0U) {
        DRV_ELRS_ResetParser();
    }

    if (DmaNeedsRestart() != 0U) {
        dma_started = 0U;
        StartRxDma();
    }

    /* check TX completion */
    if (tx_busy != 0U) {
        if (huart6.gState == HAL_UART_STATE_READY &&
            huart6.ErrorCode == HAL_UART_ERROR_NONE) {
            tx_busy = 0U;
        }
    }
}

void APP_ELRS_GetChannels(uint16_t us_out[CRSF_CHANNEL_COUNT])
{
    DRV_ELRS_GetChannels(NULL, us_out);
}

uint32_t APP_ELRS_GetLastRcMs(void)
{
    return DRV_ELRS_GetLastRcMs();
}

uint8_t APP_ELRS_IsRcFresh(uint32_t now_ms, uint32_t timeout_ms)
{
    return DRV_ELRS_IsRcFresh(now_ms, timeout_ms);
}

/* ---- telemetry ---- */

void APP_ELRS_SendTelemetryAttitude(int16_t pitch_rad_x10000,
                                    int16_t roll_rad_x10000,
                                    int16_t yaw_rad_x10000)
{
    uint8_t payload[6];
    payload[0] = (uint8_t)(pitch_rad_x10000 >> 8);
    payload[1] = (uint8_t)(pitch_rad_x10000);
    payload[2] = (uint8_t)(roll_rad_x10000 >> 8);
    payload[3] = (uint8_t)(roll_rad_x10000);
    payload[4] = (uint8_t)(yaw_rad_x10000 >> 8);
    payload[5] = (uint8_t)(yaw_rad_x10000);
    SendTelemetryFrame(CRSF_FRAME_ATTITUDE, payload, sizeof(payload));
}

void APP_ELRS_SendTelemetryBaro(int32_t altitude_dm)
{
    uint8_t payload[3];
    uint16_t packed;
    if (altitude_dm < -10000)
        packed = 0U;
    else if (altitude_dm > (int32_t)0x7FFE * 10 - 5)
        packed = 0xFFFEU;
    else if (altitude_dm < (int32_t)0x8000 - 10000)
        packed = (uint16_t)(altitude_dm + 10000);
    else
        packed = (uint16_t)(((altitude_dm + 5) / 10) | 0x8000);

    payload[0] = (uint8_t)(packed >> 8);
    payload[1] = (uint8_t)(packed);
    payload[2] = 0;
    SendTelemetryFrame(CRSF_FRAME_BARO_ALTITUDE, payload, sizeof(payload));
}

void APP_ELRS_SendTelemetryBattery(uint16_t voltage_dv, uint16_t current_da,
                                   uint32_t capacity_mah, uint8_t remaining_pct)
{
    uint8_t payload[8];
    payload[0] = (uint8_t)(voltage_dv >> 8);
    payload[1] = (uint8_t)(voltage_dv);
    payload[2] = (uint8_t)(current_da >> 8);
    payload[3] = (uint8_t)(current_da);
    payload[4] = (uint8_t)(capacity_mah >> 16);
    payload[5] = (uint8_t)(capacity_mah >> 8);
    payload[6] = (uint8_t)(capacity_mah);
    payload[7] = remaining_pct;
    SendTelemetryFrame(CRSF_FRAME_BATTERY_SENSOR, payload, sizeof(payload));
}

void APP_ELRS_SendTelemetryGps(int32_t lat_e7, int32_t lon_e7,
                               uint16_t speed_kmh_x10,
                               uint16_t heading_deg_x100,
                               uint16_t altitude_m, uint8_t satellites)
{
    uint8_t payload[15];
    uint16_t alt_plus_1000 = (uint16_t)(1000U + altitude_m);
    payload[0]  = (uint8_t)((uint32_t)lat_e7 >> 24);
    payload[1]  = (uint8_t)((uint32_t)lat_e7 >> 16);
    payload[2]  = (uint8_t)((uint32_t)lat_e7 >> 8);
    payload[3]  = (uint8_t)(lat_e7);
    payload[4]  = (uint8_t)((uint32_t)lon_e7 >> 24);
    payload[5]  = (uint8_t)((uint32_t)lon_e7 >> 16);
    payload[6]  = (uint8_t)((uint32_t)lon_e7 >> 8);
    payload[7]  = (uint8_t)(lon_e7);
    payload[8]  = (uint8_t)(speed_kmh_x10 >> 8);
    payload[9]  = (uint8_t)(speed_kmh_x10);
    payload[10] = (uint8_t)(heading_deg_x100 >> 8);
    payload[11] = (uint8_t)(heading_deg_x100);
    payload[12] = (uint8_t)(alt_plus_1000 >> 8);
    payload[13] = (uint8_t)(alt_plus_1000);
    payload[14] = satellites;
    SendTelemetryFrame(CRSF_FRAME_GPS, payload, sizeof(payload));
}

/* ---- HAL callbacks ---- */

void APP_ELRS_OnRxEvent(uint16_t size)
{
    (void)size;
    rx_events++;
}

void APP_ELRS_OnTxComplete(void)
{
    tx_busy = 0U;
}

void APP_ELRS_OnError(void)
{
    tx_busy = 0U;
    dma_started = 0U;
    rx_errors++;
}

/* ---- diagnostics ---- */

uint32_t APP_ELRS_GetRcFrames(void)  { return DRV_ELRS_GetRcFrames(); }
uint32_t APP_ELRS_GetCrcErrors(void) { return DRV_ELRS_GetCrcErrors(); }
const DRV_ELRS_LinkStats *APP_ELRS_GetLinkStats(void) { return DRV_ELRS_GetLinkStats(); }

void APP_ELRS_GetRxDiag(APP_ELRS_RxDiag *out)
{
    if (out == NULL) return;

    out->overrun  = rx_err_overrun;
    out->framing  = rx_err_framing;
    out->noise    = rx_err_noise;
    out->parity   = rx_err_parity;
    out->aborts   = rx_aborts;
    out->restarts = rx_restarts;
    out->events     = rx_events;
    out->start_fail = rx_start_fail;
}
