/*
 * app_telem_port.c —— 遥测流的目标板平台实现。
 *
 * 这里是 app_telem_stream.c 声明的那组 Port 函数在飞控上的落地：取时间、让出
 * CPU、导出互斥、采样、两个出口的写。策略（掩码 / 脏位 / 刷新 / 出口选择 /
 * 超限拒绝）一律不在这个文件里——那部分要能在宿主 gcc 上跑真代码。
 *
 * 本文件里的采样函数 APP_TelemStream_PortSample 的函数体，是从
 * Core/Src/freertos.c 的 `VOFA_task` USER CODE 段整段搬过来的（R-T1-1）。
 * 局部数组仍叫 vofa_data。相对旧 VOFA 路径的两处有据变更：四个角度增益取消
 * 显示取反以便与 `PARAM?` 同口径；速度/位置在本文件出口从控制器 legacy FRD
 * 显式适配为规范 FLU，避免状态监视把 Y 右正冒充为通用 `vel_est_y`。
 */

#include "app_telem_stream.h"

#include "app_control.h"
#include "app_flight_log.h"
#include "app_imu_capture.h"
#include "app_messages.h"
#include "app_optical_flow.h"
#include "app_stabilizer.h"
#include "app_telemetry.h"
#include "app_usb_cdc.h"
#include "app_vofa.h"

#include "drv_coax_ctrl.h"
#include "drv_frame_contract.h"
#include "svc_timestamp.h"

#include "cmsis_os2.h"
#include "rtos_objects.h"

#include <stddef.h>

/*
 * 原始 IMU 采集导出每个周期搬运的块数。单块 25 ms 会让整段导出拖到 ~50 s；
 * APP_USB_CDC_Write 自身等待 USB 完成，所以这个循环由链路速度自然限流。
 */
#define IMU_CAPTURE_EXPORT_BLOCKS_PER_TICK 16U

/* 遥测帧走 USB 出口时的单次写超时，与文本回复镜像用的那条保持一致。 */
#define APP_TELEM_PORT_USB_TX_TIMEOUT_MS 10U

uint32_t APP_TelemStream_PortNowUs(void)
{
    return (uint32_t)(SVC_Timestamp_Us() & 0xFFFFFFFFULL);
}

void APP_TelemStream_PortDelayMs(uint32_t ms)
{
    osDelay(ms);
}

uint8_t APP_TelemStream_PortServiceExports(void)
{
    /*
     * 原始 IMU 采集导出：放在这个低优先级任务里搬运，采样钩子只写 RAM，
     * 因此 USB 阻塞不会影响 1 kHz 采样或稳定环。导出期间不发遥测帧，
     * 避免两个数据流争用同一条 CDC 链路。
     */
    if (APP_IMU_Capture_IsExportActive() != 0U) {
        uint32_t burst;

        for (burst = 0U; burst < IMU_CAPTURE_EXPORT_BLOCKS_PER_TICK; burst++) {
            if (APP_IMU_Capture_IsExportActive() == 0U) {
                break;
            }
            APP_IMU_Capture_ExportStep();
        }
        return 1U;
    }

    if (APP_FlightLog_IsExportActive() != 0U) {
        return 1U;
    }

    return 0U;
}

uint8_t APP_TelemStream_PortUsbReady(void)
{
    return APP_USB_CDC_IsReady();
}

uint8_t APP_TelemStream_PortSample(float *values, uint32_t count)
{
    APP_Sensor_SampleMessage msg;
    StabilizerVofaDebug      vofa_debug;
    APP_OPTICAL_FLOW_Status  flow_status;
    DRV_COAX_CTRL_Debug      ctrl_debug;
    DRV_FRAME_Vector3f       velocity_flu;
    DRV_FRAME_Vector3f       position_flu;
    float                   *vofa_data = values;

    if ((values == NULL) || (count != (uint32_t)APP_TELEM_CH_COUNT)) {
        return 0U;
    }

    if (vofaLogQueueHandle == 0) {
        return 0U;
    }

    if (osMessageQueueGet(vofaLogQueueHandle, &msg, 0U, 0U) != osOK) {
        return 0U;
    }

    APP_OpticalFlow_GetStatus(&flow_status);

    APP_Stabilizer_ReadVofaDebug(&vofa_debug);
    DRV_COAX_CTRL_GetLastDebug(&ctrl_debug);

    /*
     * 控制器尚保留 X 前/Y 右的 legacy 口径；遥测是对外契约，必须在这里
     * 显式转成规范机体 FLU。只改观察边界，不改变控制器内部反馈符号。
     */
    velocity_flu = DRV_FRAME_FrdToFlu((DRV_FRAME_Vector3f){
        vofa_debug.vel_est_m_s[0], vofa_debug.vel_est_m_s[1], 0.0f});
    position_flu = DRV_FRAME_FrdToFlu((DRV_FRAME_Vector3f){
        vofa_debug.pos_est_m[0], vofa_debug.pos_est_m[1], 0.0f});

    /*
     * 下标一律用 APP_TELEM_CH_* 枚举名，不写裸数字：通道含义的唯一事实源是
     * App/Inc/app_telemetry.h 的枚举与 app_telemetry.c 的元数据表，上位机
     * 通过 TELEM? 拉取同一张表自动建图。改动通道请同时改枚举与表。
     */
    vofa_data[APP_TELEM_CH_ROLL] = msg.roll_deg;
    vofa_data[APP_TELEM_CH_PITCH] = msg.pitch_deg;
    vofa_data[APP_TELEM_CH_YAW] = msg.yaw_deg;

    /* 二合一光流测距高度 [m]；失效或超时立即输出 0，避免保留陈旧值。 */
    vofa_data[APP_TELEM_CH_FLOW_HEIGHT] = (flow_status.height_valid != 0U) ?
                                          flow_status.height_m : 0.0f;

    vofa_data[APP_TELEM_CH_TIME] = (float)(SVC_Timestamp_Us() / 1000ULL) * 0.001f;
    vofa_data[APP_TELEM_CH_VEL_EST_X] = velocity_flu.x;
    vofa_data[APP_TELEM_CH_VEL_EST_Y] = velocity_flu.y;
    (void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[APP_TELEM_CH_ROLL_RATE_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.pitch_rate_kd", &vofa_data[APP_TELEM_CH_PITCH_RATE_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.yaw_angle_kp", &vofa_data[APP_TELEM_CH_YAW_ANGLE_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.yaw_rate_kd", &vofa_data[APP_TELEM_CH_YAW_RATE_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_x_kp", &vofa_data[APP_TELEM_CH_POS_X_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_y_kp", &vofa_data[APP_TELEM_CH_POS_Y_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.vel_x_kd", &vofa_data[APP_TELEM_CH_VEL_X_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.vel_y_kd", &vofa_data[APP_TELEM_CH_VEL_Y_KD]);
    vofa_data[APP_TELEM_CH_POS_EST_X] = position_flu.x;
    vofa_data[APP_TELEM_CH_POS_EST_Y] = position_flu.y;
    (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vofa_data[APP_TELEM_CH_VEL_LOOP_ENABLE]);
    (void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_z_kp", &vofa_data[APP_TELEM_CH_POS_Z_KP]);
    vofa_data[APP_TELEM_CH_POS_Z_KI] = 0.0f;
    (void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[APP_TELEM_CH_VEL_Z_KD]);
    vofa_data[APP_TELEM_CH_FUSION_ACC_ERR] = msg.fusion_acceleration_error_deg;
    vofa_data[APP_TELEM_CH_FUSION_ACC_IGNORED] = (float)msg.fusion_accelerometer_ignored;
    vofa_data[APP_TELEM_CH_FUSION_ACC_RECOVERY] = msg.fusion_acceleration_recovery_trigger;
    vofa_data[APP_TELEM_CH_FUSION_ACC_CORRECTIONS] = (float)msg.fusion_accel_correction_count;
    vofa_data[APP_TELEM_CH_FUSION_ACC_NORM_REJECTED] = (float)msg.fusion_accel_norm_rejected;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        const float linear_sign = (axis == 0U) ? 1.0f : -1.0f;
        vofa_data[APP_TELEM_CH_CTRL_POS_SP_X + axis] =
            linear_sign * ctrl_debug.position_sp_m[axis];
        vofa_data[APP_TELEM_CH_CTRL_POS_ERR_X + axis] =
            linear_sign * ctrl_debug.position_error_m[axis];
        vofa_data[APP_TELEM_CH_CTRL_VEL_SP_X + axis] =
            linear_sign * ctrl_debug.velocity_sp_m_s[axis];
        vofa_data[APP_TELEM_CH_CTRL_VEL_ERR_X + axis] =
            linear_sign * ctrl_debug.velocity_error_m_s[axis];
        vofa_data[APP_TELEM_CH_CTRL_ACCEL_SP_X + axis] =
            linear_sign * ctrl_debug.accel_out_m_s2[axis];
        vofa_data[APP_TELEM_CH_CTRL_ATT_ERR_X + axis] =
            linear_sign * ctrl_debug.attitude_error[axis];
        vofa_data[APP_TELEM_CH_CTRL_RATE_SP_X + axis] =
            linear_sign * ctrl_debug.omega_sp_rad_s[axis];
        vofa_data[APP_TELEM_CH_CTRL_RATE_ERR_X + axis] =
            linear_sign * ctrl_debug.rate_error_rad_s[axis];
        vofa_data[APP_TELEM_CH_CTRL_MOMENT_CMD_X + axis] =
            linear_sign * ctrl_debug.moment_cmd_n_m[axis];
        vofa_data[APP_TELEM_CH_CTRL_MOMENT_ACH_X + axis] =
            linear_sign * ctrl_debug.moment_achieved_n_m[axis];
        vofa_data[APP_TELEM_CH_CTRL_SAT_POS_X + axis] = (float)
            ((axis == 0U) ? ctrl_debug.saturation_positive[axis] :
                            ctrl_debug.saturation_negative[axis]);
        vofa_data[APP_TELEM_CH_CTRL_SAT_NEG_X + axis] = (float)
            ((axis == 0U) ? ctrl_debug.saturation_negative[axis] :
                            ctrl_debug.saturation_positive[axis]);
    }
    /*
     * 增益通道回显的口径 = `PARAM?` 的口径 = app_control_param_to_ui_value()。
     * 那个函数（app_control_ui_sign_for_param）如今对所有参数返回 +1：FLU 迁移后
     * 控制器内部增益本身就是操作者看到的正值。旧 VOFA 填充里对四个角度增益取反
     * 是更早一版符号约定的遗留，R-T1-1 逐字搬家时被一并带了过来；审核实机复核
     * 发现 `PARAM?` 报 roll_angle_kp=+0.0671 而遥测回显 -0.0671，数据驱动的滑块
     * （量程 0..10）拿到负值钳在 0，拖动后回显永远 diverged。两条路径必须同口径，
     * 因此这里不再做任何符号处理（2026-09-03，审核者修复）。
     */

    return 1U;
}

uint8_t APP_TelemStream_PortSendUart(const uint8_t *frame, uint16_t length)
{
    /* 帧已含 $X 头，走 uartTxQueue 的原样字节路径（函数号 VOFA_SOCKET）。 */
    return APP_VOFA_SendRaw(frame, length);
}

uint8_t APP_TelemStream_PortSendUsb(const uint8_t *frame, uint16_t length)
{
    /*
     * APP_USB_CDC_Write 是阻塞等待完成的，只能在这个低优先级遥测任务上下文里
     * 调用；控制环上下文调用它会直接拉长控制周期（spec §5 上下文契约）。
     */
    return APP_USB_CDC_Write(frame, length, APP_TELEM_PORT_USB_TX_TIMEOUT_MS);
}

uint8_t APP_TelemStream_PortSendJustFloat(const float *values, uint32_t count)
{
    if (count > 255U) {
        return 0U;
    }

    return APP_VOFA_SendFloats(values, (uint8_t)count);
}

uint16_t APP_TelemStream_PortMaxPayload(APP_TelemSink sink)
{
    if (sink == APP_TELEM_SINK_USB) {
        /* R-T2 会把 USB 出口放宽到 1024（$X 的 len 本来就是 u16）。 */
        return (uint16_t)APP_TELEM_FRAME_MAX_PAYLOAD;
    }

    /* UART 出口整帧要塞进一条 APP_UART_TxMessage。 */
    return (uint16_t)(APP_UART_TX_TEXT_SIZE - APP_TELEM_FRAME_OVERHEAD);
}

void APP_TelemStream_PortReply(const char *text)
{
    if (text == NULL) {
        return;
    }

    APP_Control_QueueText("%s", text);
}
