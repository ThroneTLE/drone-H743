/*
 * `FLOGDUMP LAST|ALL` —— 飞行日志导出的范围版本。
 *
 *   FLOGDUMP LAST   只导最近一次“开始记录”以来的扇区（新记录的首扇区在扇区头里带起点标记）
 *   FLOGDUMP ALL    与 `FLOG DUMP` 相同：导出环形区里全部有效扇区
 *
 * 之后的流程与 `FLOG DUMP` 完全一致：FLOG BEGIN（带 scope=last|all，total/sectors 为该范围的真实值）
 * -> 数据块 -> FLOG END。启动失败回 `FLOG ERROR start <原因>`；参数错回 `ERR usage FLOGDUMP LAST|ALL`。
 * 旧固件不认本命令，会回 `ERR unknown cmd FLOGDUMP`，上位机据此回退到 `FLOG DUMP`。
 *
 * 独立命令词，不塞进 app_control.c 的 FLOG 族；不放进 AI 接口白名单（导出走地面站按钮）。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_flight_log.h"

#include <string.h>

uint8_t app_control_handle_flogdump(char **tokens, uint32_t count)
{
    APP_FlightLogExportScope scope;
    APP_FlightLogCommandStatus status;

    if ((count < 1U) || (tokens == NULL) || (tokens[0] == NULL) ||
        (strcmp(tokens[0], "FLOGDUMP") != 0)) {
        return 0U;
    }
    if ((count != 2U) || (tokens[1] == NULL)) {
        APP_Control_QueueText("ERR usage FLOGDUMP LAST|ALL\r\n");
        return 1U;
    }
    if (strcmp(tokens[1], "LAST") == 0) {
        scope = APP_FLIGHT_LOG_EXPORT_SCOPE_LAST_RUN;
    } else if (strcmp(tokens[1], "ALL") == 0) {
        scope = APP_FLIGHT_LOG_EXPORT_SCOPE_ALL;
    } else {
        APP_Control_QueueText("ERR usage FLOGDUMP LAST|ALL\r\n");
        return 1U;
    }

    status = APP_FlightLog_StartDumpScope(scope);
    if (status != APP_FLIGHT_LOG_CMD_OK) {
        APP_Control_QueueText("FLOG ERROR start %s\r\n", APP_FlightLog_CommandStatusText(status));
    }
    return 1U;
}
