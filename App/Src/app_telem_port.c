/*
 * app_telem_port.c —— 遥测流的目标板平台实现。
 *
 * 这里是 app_telem_stream.c 声明的那组 Port 函数在飞控上的落地：取时间、让出
 * CPU、导出互斥、采样、两个出口的写。策略（掩码 / 脏位 / 刷新 / 出口选择 /
 * 超限拒绝）一律不在这个文件里——那部分要能在宿主 gcc 上跑真代码。
 *
 * 本文件里的采样函数 APP_TelemStream_PortSample 的函数体，是从
 * Core/Src/freertos.c 的 `VOFA_task` USER CODE 段整段搬过来的（R-T1-1）。
 * 逐条赋值语句保持原样，包括局部数组仍叫 vofa_data、以及发送前对四个增益取反
 * 让操作者看到正值这条约定——控制器内部保留实测符号，遥测只改显示。
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
    vofa_data[APP_TELEM_CH_VEL_EST_X] = vofa_debug.vel_est_m_s[0];
    vofa_data[APP_TELEM_CH_VEL_EST_Y] = vofa_debug.vel_est_m_s[1];
    (void)DRV_COAX_CTRL_GetParam("coax.roll_rate_kd", &vofa_data[APP_TELEM_CH_ROLL_RATE_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.pitch_rate_kd", &vofa_data[APP_TELEM_CH_PITCH_RATE_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.yaw_angle_kp", &vofa_data[APP_TELEM_CH_YAW_ANGLE_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.yaw_rate_kd", &vofa_data[APP_TELEM_CH_YAW_RATE_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_x_kp", &vofa_data[APP_TELEM_CH_POS_X_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_y_kp", &vofa_data[APP_TELEM_CH_POS_Y_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.vel_x_kd", &vofa_data[APP_TELEM_CH_VEL_X_KD]);
    (void)DRV_COAX_CTRL_GetParam("coax.vel_y_kd", &vofa_data[APP_TELEM_CH_VEL_Y_KD]);
    vofa_data[APP_TELEM_CH_POS_EST_X] = vofa_debug.pos_est_m[0];
    vofa_data[APP_TELEM_CH_POS_EST_Y] = vofa_debug.pos_est_m[1];
    (void)DRV_COAX_CTRL_GetParam("coax.vel_loop_enable", &vofa_data[APP_TELEM_CH_VEL_LOOP_ENABLE]);
    (void)DRV_COAX_CTRL_GetParam("coax.roll_angle_kp", &vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.pitch_angle_kp", &vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_z_kp", &vofa_data[APP_TELEM_CH_POS_Z_KP]);
    (void)DRV_COAX_CTRL_GetParam("coax.pos_z_ki", &vofa_data[APP_TELEM_CH_POS_Z_KI]);
    (void)DRV_COAX_CTRL_GetParam("coax.vel_z_kd", &vofa_data[APP_TELEM_CH_VEL_Z_KD]);
    vofa_data[APP_TELEM_CH_FUSION_ACC_ERR] = msg.fusion_acceleration_error_deg;
    vofa_data[APP_TELEM_CH_FUSION_ACC_IGNORED] = (float)msg.fusion_accelerometer_ignored;
    vofa_data[APP_TELEM_CH_FUSION_ACC_RECOVERY] = msg.fusion_acceleration_recovery_trigger;
    vofa_data[APP_TELEM_CH_FUSION_ACC_CORRECTIONS] = (float)msg.fusion_accel_correction_count;
    vofa_data[APP_TELEM_CH_FUSION_ACC_NORM_REJECTED] = (float)msg.fusion_accel_norm_rejected;
    /* Present operator-facing gains as positive values; controller internals keep the tested signs. */
    vofa_data[APP_TELEM_CH_YAW_ANGLE_KP] = -vofa_data[APP_TELEM_CH_YAW_ANGLE_KP];
    vofa_data[APP_TELEM_CH_YAW_RATE_KD] = -vofa_data[APP_TELEM_CH_YAW_RATE_KD];
    vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP] = -vofa_data[APP_TELEM_CH_ROLL_ANGLE_KP];
    vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP] = -vofa_data[APP_TELEM_CH_PITCH_ANGLE_KP];

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
