/*
 * `FLOGRATE?` / `FLOGRATE <N>` —— 飞行日志记录频率子分频（N=1..5，频率 125/N Hz）。
 *
 * 外部 Flash 环形区约 4068 条，125 Hz 只能连续保留约 32.5 s，定点悬停试飞开头会被冲掉。
 * 默认 N=2（62.5 Hz，约 65 s）。只存 RAM，上电回默认；记录中/导出中拒绝。
 *
 *   FLOGRATE?   → FLOGRATE subdiv=<N> rate_hz_x10=<1250/N> window_s=<容量*N/125> capacity=<条数>
 *   FLOGRATE 3  → 同一行再加 state=set
 *                 拒绝 → FLOGRATE state=rejected reason=recording|range
 *
 * 全部整数输出（newlib-nano 无浮点 printf）。独立命令词，不塞进 app_control.c 的 FLOG 族。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_flight_log.h"

#include <stdlib.h>
#include <string.h>

static void flograte_report(const char *state_suffix)
{
    uint32_t subdiv = (uint32_t)APP_FlightLog_GetSubdiv();
    uint32_t capacity = APP_FlightLog_GetCapacityRecords();

    APP_Control_QueueText("FLOGRATE subdiv=%lu rate_hz_x10=%lu window_s=%lu capacity=%lu%s\r\n",
                          (unsigned long)subdiv,
                          (unsigned long)((APP_FLIGHT_LOG_RATE_HZ * 10UL) / subdiv),
                          (unsigned long)((capacity * subdiv) / APP_FLIGHT_LOG_RATE_HZ),
                          (unsigned long)capacity,
                          state_suffix);
}

uint8_t app_control_handle_flograte(char **tokens, uint32_t count)
{
    char *end = NULL;
    unsigned long value;

    if ((count < 1U) || (tokens == NULL) || (tokens[0] == NULL)) {
        return 0U;
    }
    if (strcmp(tokens[0], "FLOGRATE?") == 0) {
        if (count != 1U) {
            return 0U;
        }
        flograte_report("");
        return 1U;
    }
    if ((strcmp(tokens[0], "FLOGRATE") != 0) || (count != 2U) || (tokens[1] == NULL)) {
        return 0U;
    }
    value = strtoul(tokens[1], &end, 10);
    if ((end == tokens[1]) || (*end != '\0') ||
        (value < 1UL) || (value > (unsigned long)APP_FLIGHT_LOG_SUBDIV_MAX)) {
        APP_Control_QueueText("FLOGRATE state=rejected reason=range\r\n");
        return 1U;
    }
    if (APP_FlightLog_SetSubdiv((uint8_t)value) == 0U) {
        APP_Control_QueueText("FLOGRATE state=rejected reason=recording\r\n");
        return 1U;
    }
    flograte_report(" state=set");
    return 1U;
}
