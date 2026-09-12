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

static void imusel_report_bus(uint32_t id)
{
    BSP_IMU_BusSnapshot bus;
    uint32_t i;

    BSP_IMU_GetBusSnapshot(&bus);

    APP_Control_QueueText(
        "RSP id=%lu mod=IMUSEL op=BUS spi=%s pmcr=0x%08lX cr1=0x%08lX "
        "cfg1=0x%08lX cfg2=0x%08lX sr=0x%08lX\r\n",
        (unsigned long)id, bus.spi_name,
        (unsigned long)bus.pmcr,
        (unsigned long)bus.spi_cr1,
        (unsigned long)bus.spi_cfg1,
        (unsigned long)bus.spi_cfg2,
        (unsigned long)bus.spi_sr);

    for (i = 0U; i < bus.pin_count; ++i) {
        const BSP_IMU_PinSnapshot *pin = &bus.pins[i];
        uint8_t foreign = (pin->expect_af == 0xFFU) ? 1U : 0U;
        /*
         * 直接给结论，不让读的人自己对着 mode/af 心算。
         *   ok=1  —— 这根脚配得对
         *   ok=0  —— 配错了，或者"本不该归本总线的脚"却被配成了本总线的复用，
         *            那说明生成代码里还留着上一块板的引脚配置
         */
        uint8_t ok = foreign ? ((pin->af != 5U) ? 1U : 0U)
                             : (((pin->expect_af == 0U) ? (pin->mode == 1U)
                                                        : (pin->mode == 2U)) &&
                                (pin->af == pin->expect_af)) ? 1U : 0U;

        if (foreign != 0U) {
            APP_Control_QueueText(
                "RSP id=%lu mod=IMUSEL op=BUS pin=%s mode=%u af=%u od=%u in=%u exp=foreign ok=%u\r\n",
                (unsigned long)id, pin->name,
                (unsigned int)pin->mode, (unsigned int)pin->af,
                (unsigned int)pin->od, (unsigned int)pin->in,
                (unsigned int)ok);
        } else {
            APP_Control_QueueText(
                "RSP id=%lu mod=IMUSEL op=BUS pin=%s mode=%u af=%u od=%u in=%u exp_mode=%u exp_af=%u ok=%u\r\n",
                (unsigned long)id, pin->name,
                (unsigned int)pin->mode, (unsigned int)pin->af,
                (unsigned int)pin->od, (unsigned int)pin->in,
                (unsigned int)((pin->expect_af == 0U) ? 1U : 2U),
                (unsigned int)pin->expect_af,
                (unsigned int)ok);
        }
    }
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
            "RSP id=%lu mod=IMUSEL op=STATUS probe[%lu] chip=%s id=0x%02X st=%s init=%s\r\n",
            (unsigned long)id,
            (unsigned long)i,
            SVC_IMU_ChipName((DRV_IMU_ChipKind)selection.probed_kind[i]),
            (unsigned int)selection.probed_chip_id[i],
            imusel_status_name(selection.probed_status[i]),
            (selection.init_status[i] == SVC_IMU_INIT_NOT_ATTEMPTED)
                ? "skipped"
                : imusel_status_name(selection.init_status[i]));
    }
}
