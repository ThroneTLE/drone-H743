#include "app_control.h"
#include "app_control_internal.h"

#include "app_aiwb2.h"
#include "app_baro.h"
#include "app_control_config_store.h"
#include "app_flash_service.h"
#include "app_diag.h"
#include "app_flash.h"
#include "app_mag.h"
#include "app_nav_estimator.h"
#include "app_optical_flow.h"
#include "app_proto.h"
#include "app_sensor.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "app_usb_cdc.h"
#include "bsp_aiwb2_power.h"
#include "bsp_imu.h"
#include "bsp_pwm.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <stdint.h>

#define app_control_imu_stage_name(stage) \
    app_control_internal_imu_stage_name(stage)
#define app_control_flash_ok(status) \
    app_control_internal_flash_ok(status)
#define app_control_flash_stage(status) \
    app_control_internal_flash_stage(status)
#define app_control_baro_ok(status) \
    app_control_internal_baro_ok(status)
#define app_control_baro_stage(status) \
    app_control_internal_baro_stage(status)
#define app_control_aiwb2_state_name(state) \
    app_control_internal_aiwb2_state_name((uint32_t)(state))
#define control_config \
    (*(const APP_ControlConfig *)app_control_internal_config_view())

static void app_control_report_usb_cdc_stats(void);

void app_control_report_caps(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST proto=mspv2-lite-v1 resp=frame+typed req=frame+typed\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST legacy=PING,STATUS?,CONFIG?,SAVE,LOAD,SERVO,SERVOTYPE? raw=custom-tab\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST mods=MODULES,SPL06,ICM42688,IMUSEL,MEM,SPI,I2C,UART,FLOW,MAG,PARAM,FLASH,RTOS,WIFI\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=SPL06:STATUS,READ,SAMPLE ICM42688:STATUS,DIAG IMUSEL:STATUS,BUS,RAW FLOW:STATUS MAG:STATUS,DIAG\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=WIFI:STATUS,EN,RESET legacy=WIFI?,WIFI_EN?\r\n");
    app_control_queue_proto_text(APP_PROTO_MSG_CAPS_RECORD,
                                 "RSP id=0 mod=CAPS op=LIST ops=FLASH:VERIFY,BENCH_READ,SCRATCH RTOS:STATUS legacy=RTOS?\r\n");
}

void app_control_report_wifi(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_WIFI_RECORD,
                                 "WIFI en=%u pin=none last=%u writes=%lu state=%s transparent=%u retry=%lu socket=%ld cycling=%u wait_ms=%lu prov=%u cmd=%lu/%lu\r\n",
                                 (unsigned int)BSP_AiWB2_IsEnabled(),
                                 (unsigned int)BSP_AiWB2_GetLastWrittenState(),
                                 (unsigned long)BSP_AiWB2_GetWriteCount(),
                                 app_control_aiwb2_state_name(APP_AiWB2_GetState()),
                                 (unsigned int)APP_AiWB2_IsTransparent(),
                                 (unsigned long)APP_AiWB2_GetRetryCount(),
                                 (long)APP_AiWB2_GetLastSocketError(),
                                 (unsigned int)APP_AiWB2_IsPowerRecycleActive(),
                                 (unsigned long)APP_AiWB2_GetDeadlineRemainingMs(),
                                 (unsigned int)APP_AiWB2_IsProvisionActive(),
                                 (unsigned long)APP_AiWB2_GetCommandIndex(),
                                 (unsigned long)APP_AiWB2_GetCommandCount());
}

static void app_control_report_task_stack(const char *name, osThreadId_t handle)
{
    if ((name == NULL) || (handle == NULL)) {
        return;
    }

    /*
     * 除了栈余量，也报调度状态。只有栈余量的话，分不清一个任务是**在跑**、
     * **阻塞着**还是**根本没被调度**——这三种情况的下一步完全不同。
     * eTaskState：0=Running 1=Ready 2=Blocked 3=Suspended 4=Deleted 5=Invalid。
     */
    app_control_queue_proto_text(APP_PROTO_MSG_RTOS_RECORD,
                                 "RTOS task=%s free_stack_words=%lu state=%u\r\n",
                                 name,
                                 (unsigned long)uxTaskGetStackHighWaterMark((TaskHandle_t)handle),
                                 (unsigned int)eTaskGetState((TaskHandle_t)handle));
}

void app_control_report_rtos(void)
{
    APP_DiagFaultInfo faults;

    APP_Diag_GetFaultInfo(&faults);
    app_control_queue_proto_text(APP_PROTO_MSG_RTOS_RECORD,
                                 "RTOS heap_free=%lu heap_min=%lu q_uart=%lu/%lu q_background_req=%lu/%lu q_background_resp=%lu/%lu fault_stack=%u fault_task=%s fault_malloc=%u malloc_count=%lu\r\n",
                                 (unsigned long)xPortGetFreeHeapSize(),
                                 (unsigned long)xPortGetMinimumEverFreeHeapSize(),
                                 (unsigned long)((uartTxQueueHandle != NULL) ? osMessageQueueGetCount(uartTxQueueHandle) : 0U),
                                 (unsigned long)((uartTxQueueHandle != NULL) ? osMessageQueueGetCapacity(uartTxQueueHandle) : 0U),
                                 (unsigned long)((backgroundReqQueueHandle != NULL) ? osMessageQueueGetCount(backgroundReqQueueHandle) : 0U),
                                 (unsigned long)((backgroundReqQueueHandle != NULL) ? osMessageQueueGetCapacity(backgroundReqQueueHandle) : 0U),
                                 (unsigned long)((backgroundRespQueueHandle != NULL) ? osMessageQueueGetCount(backgroundRespQueueHandle) : 0U),
                                 (unsigned long)((backgroundRespQueueHandle != NULL) ? osMessageQueueGetCapacity(backgroundRespQueueHandle) : 0U),
                                 (unsigned int)faults.stack_overflow_seen,
                                 (faults.stack_overflow_task[0] != '\0') ? faults.stack_overflow_task : "-",
                                 (unsigned int)faults.malloc_failed_seen,
                                 (unsigned long)faults.malloc_failed_count);

    app_control_report_task_stack("STABILIZER", StabilizerHandle);
    app_control_report_task_stack("SENSOR", SensorTaskHandle);
    app_control_report_task_stack("MSG", messageTaskHandle);
    app_control_report_task_stack("UART", UARTTaskHandle);
    app_control_report_task_stack("BACKGROUND", backgroundTaskHandle);
    app_control_report_task_stack("TELEM", VOFA_TaskHandle);
    /* LED 也要在列：它是软件调光的时基，停了灯就只会僵在一个电平上，
     * 而"灯不动"和"固件死了"在眼睛里长得一模一样。 */
    app_control_report_task_stack("LED", LEDTaskHandle);
}

void app_control_report_modules(void)
{
    APP_Flash_Status flash_status;
    APP_Baro_Status baro_status;
    APP_IMU_Status imu_status;
    APP_OPTICAL_FLOW_Status flow_status;
    APP_MAG_Status mag_status;

    APP_Flash_GetStatus(&flash_status);
    APP_Baro_GetStatus(&baro_status);
    APP_IMU_GetStatus(&imu_status);
    APP_OpticalFlow_GetStatus(&flow_status);
    APP_MAG_GetStatus(&mag_status);

    app_control_queue_proto_text(APP_PROTO_MSG_MODULES_SUMMARY,
                                 "RSP id=0 mod=MODULES op=STATUS flash=%u flash_stage=%s baro=%u baro_stage=%s imu=%u flow=%u mag=%u\r\n",
                                 (unsigned int)app_control_flash_ok(&flash_status),
                                 app_control_flash_stage(&flash_status),
                                 (unsigned int)app_control_baro_ok(&baro_status),
                                 app_control_baro_stage(&baro_status),
                                 (unsigned int)imu_status.initialized,
                                 (unsigned int)flow_status.initialized,
                                 (unsigned int)mag_status.initialized);
    app_control_queue_proto_text(APP_PROTO_MSG_MODULES_SUMMARY,
                                 "RSP id=0 mod=MODULES op=STATUS imu_stage=%s mag_type=%s cfg_valid=%u cfg_loaded=%u servo_slots=%u wifi_en=%u\r\n",
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 APP_MAG_GetTypeName(mag_status.type),
                                 (unsigned int)control_config.flash_valid,
                                 (unsigned int)control_config.loaded_from_flash,
                                 (unsigned int)APP_CONTROL_SERVO_COUNT,
                                 (unsigned int)BSP_AiWB2_IsEnabled());
    /*
     * 解锁状态跟着 MODULES 一起发：这条是上位机连上之后必发的一条，
     * 横幅因此在第一次刷新时就有内容，不用等自己那 2 Hz 的轮询转到。
     */
    app_control_report_arm();
}

void app_control_report_status(void)
{
    APP_Flash_Status flash_status;
    APP_Baro_Status baro_status;
    APP_IMU_Status imu_status;
    APP_OPTICAL_FLOW_Status flow_status;
    APP_MAG_Status mag_status;
    DRV_NAV_EKF_Diagnostics ekf_diag;
    int32_t ekf_nis_milli;
    int32_t ekf_gate_milli;
    int32_t ekf_innov_x_mm_s;
    int32_t ekf_innov_y_mm_s;
    int32_t ekf_noise_mm_s;
    int32_t ekf_vx_mm_s;
    int32_t ekf_vy_mm_s;
    int32_t ekf_bias_x_mm_s2;
    int32_t ekf_bias_y_mm_s2;
    int32_t ekf_p0_u;
    int32_t ekf_p1_u;
    int32_t ekf_p2_u;
    int32_t ekf_p3_u;
    uint32_t uart_rx_bytes = 0U;
    uint32_t uart_rx_lines = 0U;
    uint32_t uart_rx_overflows = 0U;
    uint32_t uart_rx_errors = 0U;
    uint32_t uart_rx_events = 0U;
    uint32_t uart_rx_restarts = 0U;
    uint32_t uart_last_rx_event_size = 0U;

    APP_Flash_GetStatus(&flash_status);
    APP_Baro_GetStatus(&baro_status);
    APP_IMU_GetStatus(&imu_status);
    APP_OpticalFlow_GetStatus(&flow_status);
    APP_MAG_GetStatus(&mag_status);
    APP_NavEstimator_GetVelocityEKF(&ekf_diag);
    ekf_nis_milli = (int32_t)(ekf_diag.last_nis * 1000.0f);
    ekf_gate_milli = (int32_t)(ekf_diag.last_gate_nis * 1000.0f);
    ekf_innov_x_mm_s = (int32_t)(ekf_diag.last_innovation_m_s[0] * 1000.0f);
    ekf_innov_y_mm_s = (int32_t)(ekf_diag.last_innovation_m_s[1] * 1000.0f);
    ekf_noise_mm_s = (int32_t)(ekf_diag.last_flow_noise_m_s * 1000.0f);
    ekf_vx_mm_s = (int32_t)(ekf_diag.vel_m_s[0] * 1000.0f);
    ekf_vy_mm_s = (int32_t)(ekf_diag.vel_m_s[1] * 1000.0f);
    ekf_bias_x_mm_s2 = (int32_t)(ekf_diag.accel_bias_m_s2[0] * 1000.0f);
    ekf_bias_y_mm_s2 = (int32_t)(ekf_diag.accel_bias_m_s2[1] * 1000.0f);
    ekf_p0_u = (int32_t)(ekf_diag.covariance_diag[0] * 1000000.0f);
    ekf_p1_u = (int32_t)(ekf_diag.covariance_diag[1] * 1000000.0f);
    ekf_p2_u = (int32_t)(ekf_diag.covariance_diag[2] * 1000000.0f);
    ekf_p3_u = (int32_t)(ekf_diag.covariance_diag[3] * 1000000.0f);
    APP_UART_GetStats(&uart_rx_bytes,
                      &uart_rx_lines,
                      &uart_rx_overflows,
                      &uart_rx_errors);
    APP_UART_GetRxEventStats(&uart_rx_events,
                             &uart_rx_restarts,
                             &uart_last_rx_event_size);

    app_control_queue_proto_text(APP_PROTO_MSG_HW_FLASH,
                                 "HW FLASH ok=%u stage=%s probe=%ld sr=%ld read=%ld id=%02X%02X%02X exp=C84016 sr1=%02X\r\n",
                                 (unsigned int)app_control_flash_ok(&flash_status),
                                 app_control_flash_stage(&flash_status),
                                 (long)flash_status.probe_status,
                                 (long)flash_status.status1_status,
                                 (long)flash_status.read_status,
                                 (unsigned int)flash_status.manufacturer_id,
                                 (unsigned int)flash_status.memory_type,
                                 (unsigned int)flash_status.capacity_id,
                                 (unsigned int)flash_status.status1);
    /*
     * 上面那行说的是**外部 SPI NOR**，MicoAir743V2 板上没有这颗芯片，所以它
     * 恒为 ok=0 —— 那是实话，但单看它会让人以为参数存不了。参数其实存在
     * H743 片内 Flash 上（2026-09-11 起），所以紧跟着把参数存储的真实去向报出来，
     * 免得"外部 Flash 没有"被读成"配置保存坏了"。
     */
    app_control_queue_proto_text(APP_PROTO_MSG_HW_FLASH,
                                 "HW PARAMSTORE backend=%s cfg_slot=0x%06lX cfg_valid=%u cfg_loaded=%u last_save=%u log_backend=%s log_ready=%u nor=absent_on_this_board\r\n",
                                 APP_FlashService_BackendName(
                                     APP_FlashService_BackendFor(APP_CONTROL_CFG_SLOT_A)),
                                 (unsigned long)APP_CONTROL_CFG_SLOT_A,
                                 (unsigned int)control_config.flash_valid,
                                 (unsigned int)control_config.loaded_from_flash,
                                 (unsigned int)control_config.last_flash_status,
                                 APP_FlashService_BackendName(
                                     APP_FlashService_BackendFor(0U)),
                                 (unsigned int)APP_FlashService_IsLogStorageReady());
    app_control_queue_proto_text(APP_PROTO_MSG_HW_BARO,
                                 "HW SPL06 ok=%u stage=%s init=%ld split=%ld txrx=%ld id=%02X split_id=%02X txrx_id=%02X exp=10 cs=%u miso=%u\r\n",
                                 (unsigned int)app_control_baro_ok(&baro_status),
                                 app_control_baro_stage(&baro_status),
                                 (long)baro_status.init_status,
                                 (long)baro_status.split_status,
                                 (long)baro_status.txrx_status,
                                 (unsigned int)baro_status.product_id,
                                 (unsigned int)baro_status.split_id,
                                 (unsigned int)baro_status.txrx_id,
                                 (unsigned int)baro_status.cs_level,
                                 (unsigned int)baro_status.miso_level);
    app_control_queue_proto_text(APP_PROTO_MSG_HW_IMU,
                                 "HW ICM42688 ok=%u stage=%s st=%ld err=%ld who=%02X exp=%02X n=%lu\r\n",
                                 (unsigned int)imu_status.initialized,
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 (long)imu_status.last_status,
                                 (long)imu_status.last_error,
                                 (unsigned int)imu_status.who_am_i,
                                 (unsigned int)BSP_ICM42688_WHO_AM_I_VALUE,
                                 (unsigned long)imu_status.sample_count);
    app_control_queue_proto_text(APP_PROTO_MSG_HW_IMU,
                                 "HW ICM42688 diag valid=%u m0_tok=%02X m0_msb=%02X m0_b0=%02X m3_tok=%02X m3_msb=%02X m3_b0=%02X best_mode=%u best_hdr=%u\r\n",
                                 (unsigned int)imu_status.diag_valid,
                                 (unsigned int)imu_status.diag_mode0_tokmas,
                                 (unsigned int)imu_status.diag_mode0_msb,
                                 (unsigned int)imu_status.diag_mode0_bit0,
                                 (unsigned int)imu_status.diag_mode3_tokmas,
                                 (unsigned int)imu_status.diag_mode3_msb,
                                 (unsigned int)imu_status.diag_mode3_bit0,
                                 (unsigned int)imu_status.diag_best_mode,
                                 (unsigned int)imu_status.diag_best_header);
    app_control_queue_proto_text(APP_PROTO_MSG_HW_IMU,
                                 "HW ICM42688 burst m0_b0=%02X%02X%02X%02X m3_tok=%02X%02X%02X%02X\r\n",
                                 (unsigned int)imu_status.diag_burst_m0_b0_1,
                                 (unsigned int)imu_status.diag_burst_m0_b0_2,
                                 (unsigned int)imu_status.diag_burst_m0_b0_3,
                                 (unsigned int)imu_status.diag_burst_m0_b0_4,
                                 (unsigned int)imu_status.diag_burst_m3_tok_1,
                                 (unsigned int)imu_status.diag_burst_m3_tok_2,
                                 (unsigned int)imu_status.diag_burst_m3_tok_3,
                                 (unsigned int)imu_status.diag_burst_m3_tok_4);
    app_control_queue_proto_text(APP_PROTO_MSG_GPS_RECORD,
                                 "HW FLOW ok=%u init=%ld baud=%lu bytes=%lu frames=%lu valid=0x%02X age_ms=%lu source=%s vel_valid=%u\r\n",
                                 (unsigned int)flow_status.initialized,
                                 (long)flow_status.init_status,
                                 (unsigned long)flow_status.baud_rate,
                                 (unsigned long)flow_status.bytes,
                                 (unsigned long)flow_status.frames,
                                 (unsigned int)flow_status.valid,
                                 (unsigned long)flow_status.age_ms,
                                 APP_OpticalFlow_VelSourceName(flow_status.velocity_source),
                                 (unsigned int)flow_status.velocity_valid);
    app_control_queue_proto_text(APP_PROTO_MSG_MAG_RECORD,
                                 "HW MAG ok=%u init=%ld st=%ld type=%s addr=0x%02X who=0x%02X n=%lu x=%ld y=%ld z=%ld\r\n",
                                 (unsigned int)mag_status.initialized,
                                 (long)mag_status.init_status,
                                 (long)mag_status.last_status,
                                 APP_MAG_GetTypeName(mag_status.type),
                                 (unsigned int)mag_status.address,
                                 (unsigned int)mag_status.who_am_i,
                                 (unsigned long)mag_status.sample_count,
                                 (long)mag_status.x_mgauss,
                                 (long)mag_status.y_mgauss,
                                 (long)mag_status.z_mgauss);

    app_control_queue_proto_text(APP_PROTO_MSG_STATUS_FLASH,
                                 "STATUS flash probe=%ld sr_st=%ld read=%ld id=%02X%02X%02X sr1=%02X\r\n",
                                 (long)flash_status.probe_status,
                                 (long)flash_status.status1_status,
                                 (long)flash_status.read_status,
                                 (unsigned int)flash_status.manufacturer_id,
                                 (unsigned int)flash_status.memory_type,
                                 (unsigned int)flash_status.capacity_id,
                                 (unsigned int)flash_status.status1);
    app_control_queue_proto_text(APP_PROTO_MSG_STATUS_BARO,
                                 "STATUS baro init=%ld split=%ld txrx=%ld id=0x%02X split_id=0x%02X txrx_id=0x%02X bmp=0x%02X cs=%u miso=%u\r\n",
                                 (long)baro_status.init_status,
                                 (long)baro_status.split_status,
                                 (long)baro_status.txrx_status,
                                 (unsigned int)baro_status.product_id,
                                 (unsigned int)baro_status.split_id,
                                 (unsigned int)baro_status.txrx_id,
                                 (unsigned int)baro_status.bmp280_id,
                                 (unsigned int)baro_status.cs_level,
                                 (unsigned int)baro_status.miso_level);
    app_control_queue_proto_text(APP_PROTO_MSG_STATUS_IMU,
                                 "STATUS imu init=%u stage=%s st=%ld err=%ld who=0x%02X n=%lu ax=%d ay=%d az=%d gx=%ld gy=%ld gz=%ld t=%d\r\n",
                                 (unsigned int)imu_status.initialized,
                                 app_control_imu_stage_name(imu_status.init_stage),
                                 (long)imu_status.last_status,
                                 (long)imu_status.last_error,
                                 (unsigned int)imu_status.who_am_i,
                                 (unsigned long)imu_status.sample_count,
                                 (int)imu_status.accel_x_mg,
                                 (int)imu_status.accel_y_mg,
                                 (int)imu_status.accel_z_mg,
                                 (long)imu_status.gyro_x_mdps,
                                 (long)imu_status.gyro_y_mdps,
                                 (long)imu_status.gyro_z_mdps,
                                 (int)imu_status.temperature_cdeg);
    app_control_queue_proto_text(APP_PROTO_MSG_GPS_RECORD,
                                 "STATUS flow init=%u st=%ld valid=0x%02X frames=%lu cksum=%lu age=%lu h=%.3f vx=%.3f vy=%.3f source=%s\r\n",
                                 (unsigned int)flow_status.initialized,
                                 (long)flow_status.init_status,
                                 (unsigned int)flow_status.valid,
                                 (unsigned long)flow_status.frames,
                                 (unsigned long)flow_status.checksum_errors,
                                 (unsigned long)flow_status.age_ms,
                                 (double)flow_status.height_m,
                                 (double)flow_status.vx_m_s,
                                 (double)flow_status.vy_m_s,
                                 APP_OpticalFlow_VelSourceName(flow_status.velocity_source));
    APP_Control_QueueText("STATUS ekf init=%u pred=%lu upd=%lu rej=%lu skip=%lu nis_milli=%ld gate_milli=%ld innov_mm_s=%ld,%ld noise_mm_s=%ld vx_mm_s=%ld vy_mm_s=%ld bias_mm_s2=%ld,%ld p_u=%ld,%ld,%ld,%ld\r\n",
                          (unsigned int)ekf_diag.initialized,
                          (unsigned long)ekf_diag.predict_count,
                          (unsigned long)ekf_diag.flow_update_count,
                          (unsigned long)ekf_diag.flow_reject_count,
                          (unsigned long)ekf_diag.flow_skip_count,
                          (long)ekf_nis_milli,
                          (long)ekf_gate_milli,
                          (long)ekf_innov_x_mm_s,
                          (long)ekf_innov_y_mm_s,
                          (long)ekf_noise_mm_s,
                          (long)ekf_vx_mm_s,
                          (long)ekf_vy_mm_s,
                          (long)ekf_bias_x_mm_s2,
                          (long)ekf_bias_y_mm_s2,
                          (long)ekf_p0_u,
                          (long)ekf_p1_u,
                          (long)ekf_p2_u,
                          (long)ekf_p3_u);
    app_control_queue_proto_text(APP_PROTO_MSG_MAG_RECORD,
                                 "STATUS mag init=%u st=%ld type=%s addr=0x%02X who=0x%02X n=%lu raw=%d,%d,%d mgauss=%ld,%ld,%ld\r\n",
                                 (unsigned int)mag_status.initialized,
                                 (long)mag_status.last_status,
                                 APP_MAG_GetTypeName(mag_status.type),
                                 (unsigned int)mag_status.address,
                                 (unsigned int)mag_status.who_am_i,
                                 (unsigned long)mag_status.sample_count,
                                 (int)mag_status.raw_x,
                                 (int)mag_status.raw_y,
                                 (int)mag_status.raw_z,
                                 (long)mag_status.x_mgauss,
                                 (long)mag_status.y_mgauss,
                                 (long)mag_status.z_mgauss);
    app_control_queue_proto_text(APP_PROTO_MSG_UART_STATS,
                                 "UART1 rx_bytes=%lu rx_lines=%lu rx_overflows=%lu rx_errors=%lu rx_evt=%lu rx_rst=%lu rx_evt_size=%lu\r\n",
                                 (unsigned long)uart_rx_bytes,
                                 (unsigned long)uart_rx_lines,
                                 (unsigned long)uart_rx_overflows,
                                 (unsigned long)uart_rx_errors,
                                 (unsigned long)uart_rx_events,
                                 (unsigned long)uart_rx_restarts,
                                 (unsigned long)uart_last_rx_event_size);
    app_control_report_usb_cdc_stats();
    app_control_report_wifi();
}

static void app_control_report_usb_cdc_stats(void)
{
    app_control_queue_proto_text(APP_PROTO_MSG_UART_STATS,
                                 "USBCDC tx_sent=%lu tx_dropped=%lu\r\n",
                                 (unsigned long)APP_USB_CDC_GetTxSent(),
                                 (unsigned long)APP_USB_CDC_GetTxDropped());
}

static void app_control_report_uart_stats(uint32_t rx_bytes,
                                          uint32_t rx_lines,
                                          uint32_t rx_overflows,
                                          uint32_t rx_errors)
{
    uint32_t rx_events = 0U;
    uint32_t rx_restarts = 0U;
    uint32_t last_rx_event_size = 0U;

    APP_UART_GetRxEventStats(&rx_events,
                             &rx_restarts,
                             &last_rx_event_size);
    app_control_queue_proto_text(APP_PROTO_MSG_UART_STATS,
                                 "UART1 rx_bytes=%lu rx_lines=%lu rx_overflows=%lu rx_errors=%lu rx_evt=%lu rx_rst=%lu rx_evt_size=%lu\r\n",
                                 (unsigned long)rx_bytes,
                                 (unsigned long)rx_lines,
                                 (unsigned long)rx_overflows,
                                 (unsigned long)rx_errors,
                                 (unsigned long)rx_events,
                                 (unsigned long)rx_restarts,
                                 (unsigned long)last_rx_event_size);
    app_control_report_usb_cdc_stats();
}

void APP_Control_ReportUartStats(uint32_t rx_bytes,
                                  uint32_t rx_lines,
                                  uint32_t rx_overflows,
                                  uint32_t rx_errors)
{
    app_control_report_uart_stats(rx_bytes, rx_lines, rx_overflows, rx_errors);
}

/* ------------------------------------------------------------------ PWM 诊断 */

static void app_cmd_pwm_report_timer(const char *role,
                                     const BSP_PWM_TimerDebug *t)
{
    APP_Control_QueueText("PWM %s=%s cr1=0x%08lX ccer=0x%08lX ccmr1=0x%08lX ccmr2=0x%08lX psc=%lu arr=%lu cnt=%lu ccr=%lu,%lu,%lu,%lu\r\n",
                          role, t->name,
                          (unsigned long)t->cr1, (unsigned long)t->ccer,
                          (unsigned long)t->ccmr1, (unsigned long)t->ccmr2,
                          (unsigned long)t->psc, (unsigned long)t->arr,
                          (unsigned long)t->cnt,
                          (unsigned long)t->ccr[0], (unsigned long)t->ccr[1],
                          (unsigned long)t->ccr[2], (unsigned long)t->ccr[3]);
}

/*
 * `PWM?`。寄存器快照由 BSP 给，本函数一个寄存器都不读。
 *
 * 这条诊断因为直接读寄存器而错过一次：移植前四路 PWM 挂 TIM2，这里就写死了
 * `TIM2->CR1`；搬到 TIM1/TIM4 之后没人改，于是它一直在报一颗**本板没初始化**的
 * 定时器，实测 `cr1=0x00000000 psc=0 arr=0`——查"PWM 没输出"的人看到这行会认定
 * 定时器没配好，而真正的 TIM1/TIM4 好好的。定时器归属现在只有 bsp_pwm.c 知道。
 */
void app_control_report_pwm(void)
{
    BSP_PWM_TimerDebug timer;

    BSP_PWM_GetEscTimerDebug(&timer);
    app_cmd_pwm_report_timer("esc", &timer);
    BSP_PWM_GetServoTimerDebug(&timer);
    app_cmd_pwm_report_timer("servo", &timer);

    APP_Control_QueueText("PWM esc_us=%u,%u servo_us=%u,%u start=%u,%u,%u,%u\r\n",
                          (unsigned int)BSP_PWM_GetEscPulse(1U),
                          (unsigned int)BSP_PWM_GetEscPulse(2U),
                          (unsigned int)BSP_PWM_GetServoPulse(1U),
                          (unsigned int)BSP_PWM_GetServoPulse(2U),
                          (unsigned int)BSP_PWM_GetStartStatus(1U),
                          (unsigned int)BSP_PWM_GetStartStatus(2U),
                          (unsigned int)BSP_PWM_GetStartStatus(3U),
                          (unsigned int)BSP_PWM_GetStartStatus(4U));
}
