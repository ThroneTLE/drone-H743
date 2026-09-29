/*
 * `CURRENT PROBE` —— 电流输入脚（PC1）接线自检。
 *
 * 解决的问题：ADC 读数为 0 **无法区分**"电流真的是 0"和"Curr 信号没接到这个脚"。
 * 2026-09-21 实测就卡在这里：电机 20% 时读数在 0~0.4 A 之间跳、电机停下变成
 * 0~0.03 A，噪声幅度跟电机开关活动相关但跟电流大小不成正比，而且读数一直能回到
 * 0——一个真的有电流流过的传感器输出不会反复掉回 0 V。这些都指向浮空脚拾取 EMI，
 * 但"指向"不是判据，所以需要一个二值的测法。
 *
 * 判据见 bsp_current.h：上拉读一次、下拉读一次，电平跟着拉电阻走就是浮空。
 *
 * 为什么是 App 层做时序：Curr 线上通常有滤波电容，内部拉电阻（几十 kΩ）配 100 nF
 * 的时间常数是毫秒级。等不够会把"浮空"误判成"被驱动"——这个方向的误判最坏，
 * 它会让人放过真正的接线故障转去查固件。BSP 不碰 RTOS，延时归这里。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_stabilizer.h"
#include "bsp_current.h"
#include "cmsis_os2.h"

#include <stdio.h>
#include <string.h>

/*
 * 每种拉电阻状态的稳定等待。50 ms 对 47 kΩ × 100 nF（≈4.7 ms）有十倍裕量；
 * 真遇到更大的滤波电容，表现是两次电平相同（被判成"已驱动"），所以下面的
 * 回包里把等待时间一并报出来，方便人工判断是不是等够了。
 */
#define CURRPROBE_SETTLE_MS 50U

static void currprobe_report_busy(void)
{
    APP_Control_QueueText(
        "CURRPROBE state=refused reason=armed\r\n");
}

/*
 * `CURRENT SEQ` —— 连续转换取证。见 bsp_current.h 里 SeqBurst 那段的判据说明。
 * 与 PROBE 同样只在未解锁时允许：它会让 ADC 序列停在半路若干拍。
 */
static uint8_t currprobe_handle_seq(void)
{
    uint32_t raw[BSP_CURRENT_SEQ_BURST_MAX];
    uint32_t done = 0U;
    BSP_CurrentStatus status;
    char line[160];
    int written;
    uint32_t index;

    if (APP_Stabilizer_IsArmed() != 0U) {
        APP_Control_QueueText("CURRSEQ state=refused reason=armed\r\n");
        return 1U;
    }

    status = BSP_Current_SeqBurst(raw, BSP_CURRENT_SEQ_BURST_MAX, &done);

    written = snprintf(line, sizeof(line), "CURRSEQ st=%d n=%lu raw=",
                       (int)status, (unsigned long)done);
    for (index = 0U; (index < done) && (written > 0) &&
                     ((size_t)written < sizeof(line)); ++index) {
        written += snprintf(line + written, sizeof(line) - (size_t)written,
                            "%s%lu", (index == 0U) ? "" : ",",
                            (unsigned long)raw[index]);
    }
    /*
     * 奇数位是 rank1（电流），偶数位是 rank2（电压）——把这句一起发出去，
     * 免得读日志的人还要回头翻源码才知道该怎么对位。
     */
    APP_Control_QueueText("%s order=rank1_first_current\r\n", line);
    return 1U;
}

uint8_t app_control_handle_currprobe(char **tokens, uint32_t count)
{
    uint8_t level_pullup;
    uint8_t level_pulldown;
    BSP_CurrentStatus restore_status;
    const char *verdict;

    if ((tokens == NULL) || (count != 2U) ||
        (strcmp(tokens[0], "CURRENT") != 0)) {
        return 0U;
    }
    if (strcmp(tokens[1], "SEQ") == 0) {
        return currprobe_handle_seq();
    }
    /* 不是 PROBE 就交还给链上后面的人——`CURRENT` 开头的命令字不止这两个。 */
    if (strcmp(tokens[1], "PROBE") != 0) {
        return 0U;
    }

    /*
     * 解锁状态下拒绝。自检期间 PC1 不是模拟脚，电流采样会停若干拍；更重要的是
     * 在一个正在通电的电调输出上切换引脚方向不是飞行中该做的事。
     */
    if (APP_Stabilizer_IsArmed() != 0U) {
        currprobe_report_busy();
        return 1U;
    }

    if (BSP_Current_SetPinProbeMode(BSP_CURRENT_PIN_PROBE_PULLUP) !=
        BSP_CURRENT_OK) {
        (void)BSP_Current_SetPinProbeMode(BSP_CURRENT_PIN_PROBE_RESTORE);
        APP_Control_QueueText("CURRPROBE state=failed reason=pullup\r\n");
        return 1U;
    }
    (void)osDelay(CURRPROBE_SETTLE_MS);
    level_pullup = BSP_Current_ReadPinLevel();

    if (BSP_Current_SetPinProbeMode(BSP_CURRENT_PIN_PROBE_PULLDOWN) !=
        BSP_CURRENT_OK) {
        (void)BSP_Current_SetPinProbeMode(BSP_CURRENT_PIN_PROBE_RESTORE);
        APP_Control_QueueText("CURRPROBE state=failed reason=pulldown\r\n");
        return 1U;
    }
    (void)osDelay(CURRPROBE_SETTLE_MS);
    level_pulldown = BSP_Current_ReadPinLevel();

    /* 无论上面结果如何都要恢复：让 PC1 停在数字输入上等于永久废掉电流采样。 */
    restore_status = BSP_Current_SetPinProbeMode(BSP_CURRENT_PIN_PROBE_RESTORE);

    if (level_pullup != level_pulldown) {
        /* 电平跟着拉电阻走 = 没有任何东西在驱动这个脚。 */
        verdict = "floating";
    } else if (level_pullup == 0U) {
        /* 被钉在低电平：电调输出确实接着，且此刻电流接近 0；也可能对地短路。 */
        verdict = "driven_low";
    } else {
        verdict = "driven_high";
    }

    APP_Control_QueueText(
        "CURRPROBE pullup=%u pulldown=%u settle_ms=%u verdict=%s restore=%d\r\n",
        (unsigned int)level_pullup, (unsigned int)level_pulldown,
        (unsigned int)CURRPROBE_SETTLE_MS, verdict, (int)restore_status);
    return 1U;
}
