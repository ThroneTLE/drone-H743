#include "app_maint_uart.h"

#include "app_aiwb2.h"
#include "app_control.h"
#include "app_telem_stream.h"
#include "bsp_uart.h"
#include "svc_timestamp.h"

#include "cmsis_os2.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_MAINT_UART_LINE_SIZE 160U
#define APP_MAINT_UART_RING_SIZE 256U
#define APP_MAINT_UART_IDLE_LINE_MS 60U
#define APP_MAINT_UART_BOOT_TEXT_ENABLED 1U

/*
 * 文本写最多等多久让发送队列腾出空间。
 *
 * 115200 下队列满（2048 B）要 178 ms 才排空，所以 250 ms 足够等出一次完整排空；
 * 而它又远短于任何一条命令的可容忍无响应时间，卡住时不至于让人以为飞控死了。
 */
#define APP_MAINT_UART_TX_WAIT_MS 250U

/*
 * 遥测帧的排队上限。超过这么多字节还在排队，就丢掉这一帧。
 *
 * 256 B ≈ 两帧多一点（稳态帧 81 B，全量刷新帧 133 B），对应 115200 下约 22 ms
 * 的排队延迟——不到一个 40 Hz 周期。把上限放大只会让曲线延迟变大而不会变密：
 * 出口带宽是固定的，排在队里的帧越多，画出来的波形越滞后。
 */
#define APP_MAINT_UART_TX_BACKLOG_MAX 256U
/*
 * 多久没收到蓝牙命令就算链路闲下来了。
 *
 * 30 秒比"一问一答"的间隔宽得多，所以上位机连着的时候不会来回抖；又比一次
 * 会话短得多，所以断开之后不会一直往一个没人听的串口上阻塞发。
 */
#define APP_MAINT_UART_LINK_IDLE_MS 30000U

static uint8_t maint_rx_byte;
static uint8_t maint_rx_ring[APP_MAINT_UART_RING_SIZE];
static char maint_rx_line[APP_MAINT_UART_LINE_SIZE];
static uint16_t maint_rx_used;
static volatile uint16_t maint_rx_head;
static uint16_t maint_rx_tail;
static uint32_t maint_last_rx_ms;
static volatile uint8_t maint_rx_error;
static volatile uint8_t maint_rx_overflow;
static uint8_t maint_control_ready;
/* 最近一条从蓝牙进来的命令行的时刻；0 = 本次上电还没用过蓝牙。 */
static uint32_t maint_last_command_ms;

static uint32_t maint_tx_waits;
static uint32_t maint_tx_text_drops;
static uint32_t maint_tx_frame_drops;

static void maint_start_rx(void)
{
    if (BSP_UART_MaintRxStart(&maint_rx_byte) == 0U) {
        maint_rx_error = 1U;
    }
}

/*
 * 文本入队。队列满时让出 CPU 等一会儿，等不到才丢。
 *
 * 为什么文本等、遥测不等：命令回包丢了，操作者看到的是"飞控没反应"，
 * 还得靠回包本身去诊断这件事；遥测帧丢一帧只是曲线缺一个点，而等下去反而
 * 让后面的点全部变陈旧。两种数据的新鲜度语义相反，所以策略也相反。
 */
static uint8_t maint_tx_text(const uint8_t *data, uint16_t length)
{
    uint32_t start_ms = SVC_Timestamp_Ms();

    for (;;) {
        if (BSP_UART_MaintWrite(data, length) != 0U) {
            return 1U;
        }
        if ((SVC_Timestamp_Ms() - start_ms) >= APP_MAINT_UART_TX_WAIT_MS) {
            maint_tx_text_drops++;
            return 0U;
        }
        maint_tx_waits++;
        /*
         * 调度器还没起来时（上电阶段的 BOOT 行）osDelay 不可用，退回忙等一小会。
         * 这段只在上电那几毫秒里可能走到。
         */
        if (osKernelGetState() == osKernelRunning) {
            (void)osDelay(1U);
        } else {
            uint32_t spin_ms = SVC_Timestamp_Ms();

            while ((SVC_Timestamp_Ms() - spin_ms) < 1U) {
                /* 等一毫秒 */
            }
        }
    }
}

static void maint_write_literal(const char *text)
{
    if (text == 0) {
        return;
    }

    (void)maint_tx_text((const uint8_t *)text, (uint16_t)strlen(text));
}

static char *maint_normalize_line(char *line)
{
    char *start = line;
    char *end;

    while ((*start != '\0') && ((uint8_t)*start <= (uint8_t)' ')) {
        ++start;
    }

    end = start + strlen(start);
    while ((end > start) && ((uint8_t)*(end - 1) <= (uint8_t)' ')) {
        --end;
    }
    *end = '\0';

    return start;
}

static void maint_ensure_control_ready(void)
{
    if (maint_control_ready != 0U) {
        return;
    }

    APP_Control_Init();
    maint_control_ready = 1U;
}

static void maint_handle_line(char *line)
{
    char *normalized = maint_normalize_line(line);

    if (*normalized == '\0') {
        return;
    }

    maint_ensure_control_ready();
    maint_last_command_ms = SVC_Timestamp_Ms();
    /*
     * 告诉遥测流"最近一条命令是从蓝牙来的"，这样在蓝牙上敲 `TELEM SINK auto`
     * + `TELEM STREAM on` 就会把波形发回蓝牙，与 USB、数传的行为一致。
     */
    APP_TelemStream_NoteCommandSource(APP_TELEM_SINK_BT);
    APP_Control_ProcessMaintLine(normalized);
}

static void maint_process_byte(uint8_t byte)
{
    maint_last_rx_ms = SVC_Timestamp_Ms();

    if ((byte == '\r') || (byte == '\n')) {
        if (maint_rx_used > 0U) {
            maint_rx_line[maint_rx_used] = '\0';
            maint_handle_line(maint_rx_line);
            maint_rx_used = 0U;
        }
        return;
    }

    if (maint_rx_used < (APP_MAINT_UART_LINE_SIZE - 1U)) {
        maint_rx_line[maint_rx_used++] = (char)byte;
    } else {
        maint_rx_used = 0U;
        maint_write_literal("ERR maint line overflow\r\n");
    }
}

static void maint_flush_idle_line(void)
{
    uint32_t now_ms;

    if (maint_rx_used == 0U) {
        return;
    }

    now_ms = SVC_Timestamp_Ms();
    if ((now_ms - maint_last_rx_ms) < APP_MAINT_UART_IDLE_LINE_MS) {
        return;
    }

    maint_rx_line[maint_rx_used] = '\0';
    maint_handle_line(maint_rx_line);
    maint_rx_used = 0U;
}


void APP_MaintUART_Init(void)
{
    maint_rx_byte = 0U;
    maint_rx_used = 0U;
    maint_rx_head = 0U;
    maint_rx_tail = 0U;
    maint_last_rx_ms = SVC_Timestamp_Ms();
    maint_rx_error = 0U;
    maint_rx_overflow = 0U;
    maint_control_ready = 0U;

    maint_tx_waits = 0U;
    maint_tx_text_drops = 0U;
    maint_tx_frame_drops = 0U;

#if (APP_MAINT_UART_BOOT_TEXT_ENABLED != 0U)
    /*
     * 口名从 BSP 问，不写死。这条 BOOT 行是换板后第一个能看到的东西，
     * 它说的口必须就是实际绑的那个口，否则排查会从第一步就走错方向。
     */
    APP_MaintUART_WriteFormat("BOOT maint_uart link=%s 115200 dma_tx=1\r\n",
                              BSP_UART_MaintName());
#endif
    maint_start_rx();
}

void APP_MaintUART_Step(void)
{
    while (maint_rx_tail != maint_rx_head) {
        uint8_t byte = maint_rx_ring[maint_rx_tail];

        ++maint_rx_tail;
        if (maint_rx_tail >= APP_MAINT_UART_RING_SIZE) {
            maint_rx_tail = 0U;
        }
        maint_process_byte(byte);
    }

    if (maint_rx_overflow != 0U) {
        maint_rx_overflow = 0U;
        maint_write_literal("WARN maint rx overflow\r\n");
    }

    if (maint_rx_error != 0U) {
        maint_rx_error = 0U;
        BSP_UART_MaintRxRecover(&maint_rx_byte);
        maint_write_literal("WARN maint rx restarted\r\n");
    }

    maint_flush_idle_line();

    if (maint_control_ready != 0U) {
        APP_Control_MaintTick();
    }
}

void APP_MaintUART_Write(const char *text, uint16_t length)
{
    if ((text == 0) || (length == 0U)) {
        return;
    }

    (void)maint_tx_text((const uint8_t *)text, length);
}

uint8_t APP_MaintUART_WriteRaw(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return 0U;
    }

    /*
     * 队列里还积着上一帧（或者更多）就别再塞：出口带宽是死的，多排进去的帧
     * 只会让上位机看到越来越滞后的曲线，而不是更密的曲线。丢掉并如实计数，
     * `TELEM?` 的 drop= 与 `TELEM TX` 的 frame_drop= 都看得见。
     */
    if (BSP_UART_MaintTxPending() > APP_MAINT_UART_TX_BACKLOG_MAX) {
        maint_tx_frame_drops++;
        return 0U;
    }

    if (BSP_UART_MaintWrite(data, length) == 0U) {
        maint_tx_frame_drops++;
        return 0U;
    }

    return 1U;
}

void APP_MaintUART_ReportTx(void)
{
    BSP_UartTxStats stats;

    BSP_UART_MaintTxGetStats(&stats);
    APP_Control_QueueText(
        "TELEM TX link=%s dma=%u busy=%u pending=%lu/%lu peak=%lu "
        "bytes=%lu writes=%lu q_drop=%lu q_drop_bytes=%lu "
        "frame_drop=%lu text_drop=%lu waits=%lu starts=%lu err=%lu fallback=%lu\r\n",
        BSP_UART_MaintName(),
        (unsigned int)stats.dma,
        (unsigned int)stats.busy,
        (unsigned long)stats.pending,
        (unsigned long)stats.size,
        (unsigned long)stats.peak_used,
        (unsigned long)stats.bytes,
        (unsigned long)stats.writes,
        (unsigned long)stats.drops,
        (unsigned long)stats.dropped_bytes,
        (unsigned long)maint_tx_frame_drops,
        (unsigned long)maint_tx_text_drops,
        (unsigned long)maint_tx_waits,
        (unsigned long)stats.starts,
        (unsigned long)stats.errors,
        (unsigned long)stats.fallback);
}

void APP_MaintUART_ResetTxStats(void)
{
    maint_tx_waits       = 0U;
    maint_tx_text_drops  = 0U;
    maint_tx_frame_drops = 0U;
    BSP_UART_MaintTxResetStats();
}

uint8_t APP_MaintUART_IsLinkActive(void)
{
    uint32_t idle_ms;

    if (maint_last_command_ms == 0U) {
        return 0U;
    }

    idle_ms = SVC_Timestamp_Ms() - maint_last_command_ms;
    return (idle_ms <= APP_MAINT_UART_LINK_IDLE_MS) ? 1U : 0U;
}

void APP_MaintUART_WriteFormat(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int written;

    if (format == NULL) {
        return;
    }

    va_start(args, format);
    written = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (written <= 0) {
        return;
    }
    if ((uint32_t)written >= sizeof(buffer)) {
        written = (int)(sizeof(buffer) - 1U);
        buffer[written] = '\0';
    }

    APP_MaintUART_Write(buffer, (uint16_t)written);
}

void APP_MaintUART_OnRxByte(void)
{
    uint16_t next_head = (uint16_t)(maint_rx_head + 1U);

    if (next_head >= APP_MAINT_UART_RING_SIZE) {
        next_head = 0U;
    }

    if (next_head == maint_rx_tail) {
        maint_rx_overflow = 1U;
    } else {
        maint_rx_ring[maint_rx_head] = maint_rx_byte;
        maint_rx_head = next_head;
    }

    maint_start_rx();
}

void APP_MaintUART_OnRxError(void)
{
    maint_rx_error = 1U;
}
