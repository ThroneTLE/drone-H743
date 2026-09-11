/*
 * IMUSEL 命令族 —— 把 IMU 选型记账原样回报出来。
 *
 * 为什么要单独一条命令：板上有两颗 IMU，`BSP_IMU_Init()` 先把候选全部探测一遍再
 * 按优先级逐个 init，谁先成功谁上岗。结果只留下"选中了谁"，而**没被选中的那几颗
 * 到底是探不到、还是探到了但配置失败**，只存在 `SVC_IMU_Selection` 里，没有任何
 * 出口。2026-09-10 首刷 MicoAir743v2 时就撞上了：BMI088 是设计里的主 IMU，实机上
 * 却是 BMI270 上岗，而现有诊断一个字都说不出原因——只能靠猜。
 *
 * `svc_imu.h` 里那句"供诊断命令原样回报"当时是一张空头支票，这里把它兑现。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "bsp_imu.h"
#include "main.h"
#include "svc_imu.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * 探测状态码 → 短名。刻意区分 bad_id 与 error：
 *   bad_id  —— 总线通了，但读回的 ID 不是这颗芯片（片选接错 / 板上不是这颗）
 *   error   —— HAL 传输本身就失败（时钟、引脚复用、模拟开关没闭合）
 * 这两者的排查方向完全相反，合并成一个 "fail" 就等于什么都没说。
 */
static const char *imusel_status_name(DRV_IMU_Status status)
{
    switch (status) {
    case DRV_IMU_OK:          return "ok";
    case DRV_IMU_ERROR:       return "error";
    case DRV_IMU_TIMEOUT:     return "timeout";
    case DRV_IMU_BAD_ID:      return "bad_id";
    case DRV_IMU_INVALID_ARG: return "bad_arg";
    default:                  return "unknown";
    }
}

/*
 * 引脚现场快照：MODER / AFR / ODR / IDR 四个字段就能把"配置对不对"和"线上有没有电平"
 * 分开。BMI088 探测读回 0x00 时，单看驱动分不清是 MISO 没配成 AF、还是芯片没应答；
 * 这里直接把硅片上的实际状态打出来，不用猜。
 */
static void imusel_report_pin(uint32_t id, const char *name,
                              GPIO_TypeDef *port, uint32_t index)
{
    uint32_t mode = (port->MODER >> (index * 2U)) & 0x3U;
    uint32_t af   = (port->AFR[index >> 3U] >> ((index & 7U) * 4U)) & 0xFU;

    APP_Control_QueueText(
        "RSP id=%lu mod=IMUSEL op=BUS pin=%s mode=%lu af=%lu od=%lu in=%lu\r\n",
        (unsigned long)id, name,
        (unsigned long)mode, (unsigned long)af,
        (unsigned long)((port->ODR >> index) & 1U),
        (unsigned long)((port->IDR >> index) & 1U));
}

static void imusel_report_bus(uint32_t id)
{
    APP_Control_QueueText(
        "RSP id=%lu mod=IMUSEL op=BUS pmcr=0x%08lX spi2_cr1=0x%08lX "
        "spi2_cfg1=0x%08lX spi2_cfg2=0x%08lX spi2_sr=0x%08lX\r\n",
        (unsigned long)id,
        (unsigned long)SYSCFG->PMCR,
        (unsigned long)SPI2->CR1,
        (unsigned long)SPI2->CFG1,
        (unsigned long)SPI2->CFG2,
        (unsigned long)SPI2->SR);

    /* SPI2（BMI088）：期望 mode=2（AF）且 af=5。 */
    imusel_report_pin(id, "PD3_SCK",   GPIOD, 3U);
    imusel_report_pin(id, "PC2_MISO",  GPIOC, 2U);
    imusel_report_pin(id, "PC3_MOSI",  GPIOC, 3U);
    /* 片选：期望 mode=1（推挽输出）且 od=1（空闲拉高）。 */
    imusel_report_pin(id, "PD4_A_CS",  GPIOD, 4U);
    imusel_report_pin(id, "PD5_G_CS",  GPIOD, 5U);
    /*
     * 这两个脚本板上不归 SPI2：PC1 是电池电流采样、PA9 是 USART1_TX。
     * 它们是老板子 SPI2 的 MOSI/SCK，若仍是 mode=2 af=5，说明生成代码的
     * USER CODE 区还留着上一块板的引脚配置。
     */
    imusel_report_pin(id, "PC1_IBAT",  GPIOC, 1U);
    imusel_report_pin(id, "PA9_U1TX",  GPIOA, 9U);
}

void app_control_req_imusel(uint32_t id, const char *op)
{
    SVC_IMU_Selection selection;
    BSP_IMU_Info      info;
    uint32_t          i;

    if (op == NULL) {
        APP_Control_QueueText("ERR id=%lu mod=IMUSEL op=? code=NO_OP\r\n",
                              (unsigned long)id);
        return;
    }

    if (strcmp(op, "BUS") == 0) {
        imusel_report_bus(id);
        return;
    }

    if (strcmp(op, "RAW") == 0) {
        BSP_IMU_RawProbe probe;
        uint32_t n;

        BSP_IMU_DebugRawBmi088(&probe);
        for (n = 0U; n < 4U; n++) {
            APP_Control_QueueText(
                "RSP id=%lu mod=IMUSEL op=RAW n=%lu hal=%u id=0x%02X exp=0x1E "
                "sr=0x%08lX\r\n",
                (unsigned long)id, (unsigned long)n,
                (unsigned int)probe.hal[n], (unsigned int)probe.chip_id[n],
                (unsigned long)probe.sr[n]);
        }
        return;
    }

    if (strcmp(op, "STATUS") != 0) {
        APP_Control_QueueText("ERR id=%lu mod=IMUSEL op=%s code=BAD_OP\r\n",
                              (unsigned long)id, op);
        return;
    }

    BSP_IMU_GetSelection(&selection);
    BSP_IMU_GetInfo(&info);

    APP_Control_QueueText(
        "RSP id=%lu mod=IMUSEL op=STATUS selected=%s chip_id=0x%02X "
        "stage=%u last_err=%ld probes=%u\r\n",
        (unsigned long)id,
        SVC_IMU_ChipName(selection.selected),
        (unsigned int)selection.selected_chip_id,
        (unsigned int)info.init_stage,
        (long)info.last_error,
        (unsigned int)selection.probe_count);

    /*
     * 每颗候选各占一行。探测顺序即优先级，所以行序本身就是"谁先被试"的答案：
     * 排在选中者前面却没上岗的那几行，就是需要排查的对象。
     */
    for (i = 0U; i < (uint32_t)selection.probe_count; i++) {
        APP_Control_QueueText(
            "RSP id=%lu mod=IMUSEL op=STATUS probe[%lu] chip=%s id=0x%02X st=%s\r\n",
            (unsigned long)id,
            (unsigned long)i,
            SVC_IMU_ChipName((DRV_IMU_ChipKind)selection.probed_kind[i]),
            (unsigned int)selection.probed_chip_id[i],
            imusel_status_name(selection.probed_status[i]));
    }
}
