/*
 * `IMUZERO` / `IMUZERO?` —— 软件重新标定陀螺零偏与姿态零点（只在上锁时接受）。
 *
 * 作者 2026-10-01："每次重新烧录舵机会动一下然后让机身开始晃动，然后这个时候又校准不了，
 * 我扶着也不是很准，你不然就来一个软件重新标定的命令"。
 *
 * 上电时陀螺零偏取前 1000 个静止样本的均值，姿态零点取随后 1.5 s 静止窗口。台架上烧录/重启时
 * 舵机一动机体就绕杆摆，摆动被当成零偏（2026-09-30 实测约 1.5°/s，融合误差 3.3° 超过零点门限
 * 3°，姿态角一直报 0，XY 也拿不到重力水平）。等机体静下来发 `IMUZERO`：两者清零按上电同一规则
 * 重新采样，约 3 s 后 `IMUZERO?` 应报 gyro_bias=1 attitude_zero=1。
 *
 *   IMUZERO     → IMUZERO state=restarted keep_still_ms=3000
 *                 已解锁 → IMUZERO state=armed_blocked（不改任何状态）
 *   IMUZERO?    → IMUZERO gyro_bias=<0|1> attitude_zero=<0|1> ferr_cdeg=<融合加速度误差>
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_sensor.h"
#include "app_stabilizer.h"

#include <math.h>
#include <string.h>

static void imuzero_report(void)
{
    StabilizerValidationImuSnapshot snapshot;
    long ferr_cdeg = -1L;
    unsigned int bias_ready = 0U;

    memset(&snapshot, 0, sizeof(snapshot));
    if (APP_Stabilizer_ReadValidationImuSnapshot(&snapshot) != 0U) {
        bias_ready = (unsigned int)snapshot.gyro_bias_ready;
        if (isfinite(snapshot.fusion_acceleration_error_deg)) {
            ferr_cdeg = lroundf(snapshot.fusion_acceleration_error_deg * 100.0f);
        }
    }
    APP_Control_QueueText("IMUZERO gyro_bias=%u attitude_zero=%u ferr_cdeg=%ld\r\n",
                          bias_ready,
                          (unsigned int)APP_Stabilizer_IsAttitudeZeroReady(),
                          ferr_cdeg);
}

uint8_t app_control_handle_imuzero(char **tokens, uint32_t count)
{
    if ((count != 1U) || (tokens == NULL) || (tokens[0] == NULL)) {
        return 0U;
    }
    if (strcmp(tokens[0], "IMUZERO?") == 0) {
        imuzero_report();
        return 1U;
    }
    if (strcmp(tokens[0], "IMUZERO") != 0) {
        return 0U;
    }
    if (APP_Stabilizer_IsArmed() != 0U) {
        APP_Control_QueueText("IMUZERO state=armed_blocked\r\n");
        return 1U;
    }
    APP_Sensor_RequestGyroRecal();
    APP_Stabilizer_RequestAttitudeRezero();
    APP_Control_QueueText("IMUZERO state=restarted keep_still_ms=3000\r\n");
    return 1U;
}
