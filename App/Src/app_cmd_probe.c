/*
 * 通用探针命令族：MEM / SPI / I2C / UART。
 *
 * 动机是省掉重新编译。2026-09-11 定位 BMI088 那个"只有第一笔读得到数据"的缺陷时，
 * 十几轮试验每一轮都要改代码 → 编译 → 跳 DFU → 烧录 → 重启，一轮约 40 秒；而真正
 * 变化的往往只是几个字节的收发内容或一个寄存器位。这一族命令把那类试验搬到串口上。
 *
 * 三条自我约束，都是刻意的：
 *
 *   1. **解锁才让动总线。** SPI/I2C/UART 事务和 POKE 都会打断正在跑的采样与控制，
 *      UART7 更是直接连着总线舵机。飞行中被一条调试命令插一脚，后果不是"数据不准"
 *      而是失控，所以一律先看 APP_Stabilizer_IsArmed()。PEEK 是只读，不受此限。
 *
 *   2. **POKE 只开放外设地址段，且要确认字。** 往 RAM 或 Flash 里写能瞬间毁掉
 *      RTOS 现场或参数区，而那正是最难事后归因的一类故障。
 *
 *   3. **目标由枚举给定，不收裸引脚/裸句柄。** 片选、I2C 总线、UART 总线都只
 *      接受板上真实存在的那几个，拼不出不存在的组合，也不会有人拿它去拨别的脚。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "app_stabilizer.h"
#include "bsp_i2c.h"
#include "bsp_imu.h"
#include "bsp_uart.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define PROBE_HEX_MAX 64U

/* ---------------------------------------------------------------- 小工具 */

static void probe_err(uint32_t id, const char *mod, const char *op,
                      const char *code)
{
    APP_Control_QueueText("ERR id=%lu mod=%s op=%s code=%s\r\n",
                          (unsigned long)id,
                          (mod != NULL) ? mod : "?",
                          (op != NULL) ? op : "?",
                          (code != NULL) ? code : "ERR");
}

/* 解锁才放行会动硬件的操作，理由见文件头第 1 条。 */
static uint8_t probe_blocked_by_arm(uint32_t id, const char *mod, const char *op)
{
    if (APP_Stabilizer_IsArmed() != 0U) {
        probe_err(id, mod, op, "ARMED");
        return 1U;
    }
    return 0U;
}

static uint8_t probe_hex_nibble(char c, uint8_t *value)
{
    if ((c >= '0') && (c <= '9')) { *value = (uint8_t)(c - '0');        return 1U; }
    if ((c >= 'a') && (c <= 'f')) { *value = (uint8_t)(c - 'a' + 10);   return 1U; }
    if ((c >= 'A') && (c <= 'F')) { *value = (uint8_t)(c - 'A' + 10);   return 1U; }
    return 0U;
}

/* "8000ff" → {0x80,0x00,0xff}。长度必须是偶数。 */
static uint8_t probe_parse_hex(const char *text, uint8_t *out, uint16_t max,
                               uint16_t *out_len)
{
    uint16_t n = 0U;
    uint32_t i;
    uint32_t len;

    if ((text == NULL) || (out == NULL) || (out_len == NULL)) { return 0U; }

    len = (uint32_t)strlen(text);
    if (((len % 2U) != 0U) || (len == 0U) || ((len / 2U) > (uint32_t)max)) {
        return 0U;
    }

    for (i = 0U; i < len; i += 2U) {
        uint8_t hi;
        uint8_t lo;
        if ((probe_hex_nibble(text[i], &hi) == 0U) ||
            (probe_hex_nibble(text[i + 1U], &lo) == 0U)) {
            return 0U;
        }
        out[n] = (uint8_t)((hi << 4U) | lo);
        n++;
    }

    *out_len = n;
    return 1U;
}

/* 把字节串回报成连续十六进制，空串报 "-" 以免和"回了 0 字节"混淆。 */
static void probe_report_bytes(uint32_t id, const char *mod, const char *op,
                               const char *label, const uint8_t *data,
                               uint16_t len)
{
    char text[(PROBE_HEX_MAX * 2U) + 1U];
    uint16_t i;

    if (len == 0U) {
        APP_Control_QueueText("RSP id=%lu mod=%s op=%s %s=- n=0\r\n",
                              (unsigned long)id, mod, op, label);
        return;
    }

    for (i = 0U; i < len; i++) {
        static const char digits[] = "0123456789ABCDEF";
        text[i * 2U]        = digits[(data[i] >> 4U) & 0x0FU];
        text[(i * 2U) + 1U] = digits[data[i] & 0x0FU];
    }
    text[len * 2U] = '\0';

    APP_Control_QueueText("RSP id=%lu mod=%s op=%s %s=%s n=%u\r\n",
                          (unsigned long)id, mod, op, label, text,
                          (unsigned int)len);
}

static uint8_t probe_u32(char **tokens, uint32_t count, const char *key,
                         uint32_t *value)
{
    const char *text = app_control_token_value(tokens, count, key);
    return (text != NULL)
        ? app_control_internal_parse_u32_auto(text, value)
        : 0U;
}

/* ---------------------------------------------------------------- MEM */

/*
 * 可读地址段。越界读会触发总线错误 → HardFault，所以宁可先挡住：
 * 一条调试命令把飞控打挂，比它读不到那个地址糟糕得多。
 */
static uint8_t probe_addr_readable(uint32_t addr)
{
    return (((addr >= 0x00000000UL) && (addr < 0x00010000UL)) ||   /* ITCM  64K */
            ((addr >= 0x08000000UL) && (addr < 0x08200000UL)) ||   /* Flash  2M */
            ((addr >= 0x20000000UL) && (addr < 0x20020000UL)) ||   /* DTCM 128K */
            ((addr >= 0x24000000UL) && (addr < 0x24080000UL)) ||   /* AXI  512K */
            ((addr >= 0x30000000UL) && (addr < 0x30048000UL)) ||   /* D2   288K */
            ((addr >= 0x38000000UL) && (addr < 0x38010000UL)) ||   /* D3    64K */
            ((addr >= 0x40000000UL) && (addr < 0x60000000UL)) ||   /* 外设       */
            ((addr >= 0xE0000000UL) && (addr < 0xE0100000UL)))     /* 内核私有   */
        ? 1U : 0U;
}

/* 可写地址段只有外设。理由见文件头第 2 条。 */
static uint8_t probe_addr_writable(uint32_t addr)
{
    return (((addr >= 0x40000000UL) && (addr < 0x60000000UL)) ||
            ((addr >= 0xE0000000UL) && (addr < 0xE0100000UL)))
        ? 1U : 0U;
}

static void probe_mem(uint32_t id, const char *op, char **tokens, uint32_t count)
{
    uint32_t addr = 0U;
    uint32_t value = 0U;
    uint32_t words = 1U;
    uint32_t i;

    if (probe_u32(tokens, count, "addr", &addr) == 0U) {
        probe_err(id, "MEM", op, "BAD_ADDR");
        return;
    }
    if ((addr & 0x3UL) != 0U) {
        probe_err(id, "MEM", op, "UNALIGNED");
        return;
    }

    if (strcmp(op, "PEEK") == 0) {
        (void)probe_u32(tokens, count, "count", &words);
        if ((words == 0U) || (words > 8U)) {
            probe_err(id, "MEM", op, "BAD_COUNT");
            return;
        }
        for (i = 0U; i < words; i++) {
            uint32_t a = addr + (i * 4U);
            if (probe_addr_readable(a) == 0U) {
                probe_err(id, "MEM", op, "ADDR_DENIED");
                return;
            }
            APP_Control_QueueText(
                "RSP id=%lu mod=MEM op=PEEK addr=0x%08lX value=0x%08lX\r\n",
                (unsigned long)id, (unsigned long)a,
                (unsigned long)(*(volatile uint32_t *)a));
        }
        return;
    }

    if (strcmp(op, "POKE") == 0) {
        const char *confirm = app_control_token_value(tokens, count, "confirm");

        if (probe_blocked_by_arm(id, "MEM", op) != 0U) { return; }
        if (probe_u32(tokens, count, "value", &value) == 0U) {
            probe_err(id, "MEM", op, "BAD_VALUE");
            return;
        }
        if (probe_addr_writable(addr) == 0U) {
            probe_err(id, "MEM", op, "ADDR_DENIED");
            return;
        }
        /* 写外设寄存器不可撤销，要求显式确认字，挡住手滑和误粘贴。 */
        if ((confirm == NULL) || (strcmp(confirm, "YES") != 0)) {
            probe_err(id, "MEM", op, "NEED_CONFIRM");
            return;
        }

        *(volatile uint32_t *)addr = value;
        APP_Control_QueueText(
            "RSP id=%lu mod=MEM op=POKE addr=0x%08lX wrote=0x%08lX readback=0x%08lX\r\n",
            (unsigned long)id, (unsigned long)addr, (unsigned long)value,
            (unsigned long)(*(volatile uint32_t *)addr));
        return;
    }

    probe_err(id, "MEM", op, "BAD_OP");
}

/* ---------------------------------------------------------------- SPI */

static uint8_t probe_spi_cs(const char *text, BSP_IMU_SpiCs *cs)
{
    if (text == NULL) { return 0U; }
    if (strcmp(text, "ACC") == 0)  { *cs = BSP_IMU_SPI_CS_BMI088_ACC;  return 1U; }
    if (strcmp(text, "GYRO") == 0) { *cs = BSP_IMU_SPI_CS_BMI088_GYRO; return 1U; }
    if (strcmp(text, "BMI270") == 0) { *cs = BSP_IMU_SPI_CS_BMI270;    return 1U; }
    return 0U;
}

static void probe_spi(uint32_t id, const char *op, char **tokens, uint32_t count)
{
    uint8_t tx[PROBE_HEX_MAX];
    uint8_t rx[PROBE_HEX_MAX];
    uint16_t len = 0U;
    BSP_IMU_SpiCs cs;
    uint32_t bus = 0U;
    DRV_IMU_Status status;

    if (strcmp(op, "XFER") != 0) {
        probe_err(id, "SPI", op, "BAD_OP");
        return;
    }
    if (probe_blocked_by_arm(id, "SPI", op) != 0U) { return; }

    if (probe_spi_cs(app_control_token_value(tokens, count, "cs"), &cs) == 0U) {
        probe_err(id, "SPI", op, "BAD_CS");
        return;
    }
    /* bus= 可省。给了就校验，免得心里想着 SPI3 却发到了 SPI2 上。 */
    if ((probe_u32(tokens, count, "bus", &bus) != 0U) &&
        (bus != (uint32_t)BSP_IMU_DebugSpiBusIndex(cs))) {
        probe_err(id, "SPI", op, "BUS_MISMATCH");
        return;
    }
    if (probe_parse_hex(app_control_token_value(tokens, count, "tx"),
                        tx, (uint16_t)BSP_IMU_SPI_XFER_MAX, &len) == 0U) {
        probe_err(id, "SPI", op, "BAD_TX");
        return;
    }

    memset(rx, 0, sizeof(rx));
    status = BSP_IMU_DebugSpiXfer(cs, tx, rx, len);
    if (status != DRV_IMU_OK) {
        probe_err(id, "SPI", op, (status == DRV_IMU_TIMEOUT) ? "TIMEOUT" : "XFER");
        return;
    }

    APP_Control_QueueText("RSP id=%lu mod=SPI op=XFER bus=%u cs=%s\r\n",
                          (unsigned long)id,
                          (unsigned int)BSP_IMU_DebugSpiBusIndex(cs),
                          app_control_token_value(tokens, count, "cs"));
    probe_report_bytes(id, "SPI", "XFER", "rx", rx, len);
}

/* ---------------------------------------------------------------- I2C */

static const char *probe_i2c_status_name(BSP_I2C_Status status)
{
    switch (status) {
    case BSP_I2C_OK:          return "ok";
    case BSP_I2C_INVALID_ARG: return "BAD_ARG";
    case BSP_I2C_TIMEOUT:     return "TIMEOUT";
    case BSP_I2C_NACK:        return "NACK";
    default:                  return "ERROR";
    }
}

static void probe_i2c(uint32_t id, const char *op, char **tokens, uint32_t count)
{
    uint32_t bus = 0U;
    uint32_t addr = 0U;
    uint32_t rd = 0U;
    uint8_t tx[PROBE_HEX_MAX];
    uint8_t rx[PROBE_HEX_MAX];
    uint16_t tx_len = 0U;
    const char *tx_text;
    BSP_I2C_Status status;

    if (probe_u32(tokens, count, "bus", &bus) == 0U) {
        probe_err(id, "I2C", op, "BAD_BUS");
        return;
    }
    if (BSP_I2C_GetHandle((uint8_t)bus) == NULL) {
        probe_err(id, "I2C", op, "BAD_BUS");
        return;
    }
    if (probe_blocked_by_arm(id, "I2C", op) != 0U) { return; }

    if (strcmp(op, "SCAN") == 0) {
        uint8_t found[16];
        uint8_t n = BSP_I2C_DebugScan((uint8_t)bus, found, (uint8_t)sizeof(found));
        uint8_t i;

        APP_Control_QueueText("RSP id=%lu mod=I2C op=SCAN bus=%lu found=%u\r\n",
                              (unsigned long)id, (unsigned long)bus,
                              (unsigned int)n);
        for (i = 0U; i < n; i++) {
            APP_Control_QueueText(
                "RSP id=%lu mod=I2C op=SCAN bus=%lu addr=0x%02X\r\n",
                (unsigned long)id, (unsigned long)bus, (unsigned int)found[i]);
        }
        return;
    }

    if (strcmp(op, "XFER") != 0) {
        probe_err(id, "I2C", op, "BAD_OP");
        return;
    }

    if ((probe_u32(tokens, count, "addr", &addr) == 0U) || (addr > 0x7FUL)) {
        probe_err(id, "I2C", op, "BAD_ADDR");
        return;
    }
    (void)probe_u32(tokens, count, "rd", &rd);
    if (rd > (uint32_t)BSP_I2C_XFER_MAX) {
        probe_err(id, "I2C", op, "BAD_RD");
        return;
    }

    tx_text = app_control_token_value(tokens, count, "tx");
    if (tx_text != NULL) {
        if (probe_parse_hex(tx_text, tx, (uint16_t)BSP_I2C_XFER_MAX,
                            &tx_len) == 0U) {
            probe_err(id, "I2C", op, "BAD_TX");
            return;
        }
    }
    if ((tx_len == 0U) && (rd == 0U)) {
        probe_err(id, "I2C", op, "NOTHING_TO_DO");
        return;
    }

    memset(rx, 0, sizeof(rx));
    status = BSP_I2C_DebugXfer((uint8_t)bus, (uint8_t)addr, tx, tx_len,
                               rx, (uint16_t)rd);
    if (status != BSP_I2C_OK) {
        probe_err(id, "I2C", op, probe_i2c_status_name(status));
        return;
    }

    APP_Control_QueueText("RSP id=%lu mod=I2C op=XFER bus=%lu addr=0x%02X wrote=%u\r\n",
                          (unsigned long)id, (unsigned long)bus,
                          (unsigned int)addr, (unsigned int)tx_len);
    probe_report_bytes(id, "I2C", "XFER", "rx", rx, (uint16_t)rd);
}

/* ---------------------------------------------------------------- UART */

static void probe_uart(uint32_t id, const char *op, char **tokens, uint32_t count)
{
    uint32_t bus = 0U;
    uint32_t rd = 0U;
    uint32_t timeout = 50U;
    uint8_t tx[PROBE_HEX_MAX];
    uint8_t rx[PROBE_HEX_MAX];
    uint16_t tx_len = 0U;
    uint16_t rx_got = 0U;
    const char *tx_text;

    if (strcmp(op, "XFER") != 0) {
        probe_err(id, "UART", op, "BAD_OP");
        return;
    }
    if (probe_u32(tokens, count, "bus", &bus) == 0U) {
        probe_err(id, "UART", op, "BAD_BUS");
        return;
    }
    if (BSP_UART_GetHandle((uint8_t)bus) == NULL) {
        probe_err(id, "UART", op, "BAD_BUS");
        return;
    }
    /* UART7 直连总线舵机，发字节就会让舵机动，所以这条一定要卡解锁状态。 */
    if (probe_blocked_by_arm(id, "UART", op) != 0U) { return; }

    (void)probe_u32(tokens, count, "rd", &rd);
    (void)probe_u32(tokens, count, "timeout", &timeout);
    if ((rd > (uint32_t)BSP_UART_XFER_MAX) || (timeout > 1000U)) {
        probe_err(id, "UART", op, "BAD_ARG");
        return;
    }

    tx_text = app_control_token_value(tokens, count, "tx");
    if (tx_text != NULL) {
        if (probe_parse_hex(tx_text, tx, (uint16_t)BSP_UART_XFER_MAX,
                            &tx_len) == 0U) {
            probe_err(id, "UART", op, "BAD_TX");
            return;
        }
    }
    if ((tx_len == 0U) && (rd == 0U)) {
        probe_err(id, "UART", op, "NOTHING_TO_DO");
        return;
    }

    memset(rx, 0, sizeof(rx));
    if (BSP_UART_DebugXfer((uint8_t)bus, tx, tx_len, rx, (uint16_t)rd, &rx_got,
                           timeout) != HAL_OK) {
        probe_err(id, "UART", op, "XFER");
        return;
    }

    APP_Control_QueueText(
        "RSP id=%lu mod=UART op=XFER bus=%lu half_duplex=%u wrote=%u\r\n",
        (unsigned long)id, (unsigned long)bus,
        (unsigned int)BSP_UART_IsHalfDuplex((uint8_t)bus),
        (unsigned int)tx_len);
    probe_report_bytes(id, "UART", "XFER", "rx", rx, rx_got);
}

/* ---------------------------------------------------------------- 入口 */

uint8_t app_control_req_probe(uint32_t id, const char *mod, const char *op,
                              char **tokens, uint32_t count)
{
    if ((mod == NULL) || (op == NULL)) { return 0U; }

    if (strcmp(mod, "MEM") == 0)  { probe_mem(id, op, tokens, count);  return 1U; }
    if (strcmp(mod, "SPI") == 0)  { probe_spi(id, op, tokens, count);  return 1U; }
    if (strcmp(mod, "I2C") == 0)  { probe_i2c(id, op, tokens, count);  return 1U; }
    if (strcmp(mod, "UART") == 0) { probe_uart(id, op, tokens, count); return 1U; }

    return 0U;
}
