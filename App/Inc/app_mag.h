#ifndef APP_MAG_H
#define APP_MAG_H

#include <stdint.h>

/*
 * 缓存快照容许的最大年龄：磁力计 20Hz 步进（周期 50ms），给调度抖动/任务
 * 饿死留足余量再判过期（约 4 倍名义周期）。契约第 3 条（样本新鲜）的门限，
 * 消费者（App/Src/app_stabilizer.c）用它判断要不要把这一拍的磁场值喂进
 * 姿态融合。
 */
#define APP_MAG_SNAPSHOT_MAX_AGE_US 200000ULL

typedef struct {
    uint8_t initialized;
    int32_t init_status;
    int32_t last_status;
    uint8_t type;
    uint8_t address;
    uint8_t who_am_i;
    uint32_t sample_count;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    int32_t x_mgauss;
    int32_t y_mgauss;
    int32_t z_mgauss;
    uint8_t detected_ist8310;
    uint8_t detected_hmc5883;
    uint8_t detected_qmc5883;
    uint8_t hmc_id_a;
    uint8_t hmc_id_b;
    uint8_t hmc_id_c;
} APP_MAG_Status;

/*
 * 缓存快照：贴装变换（Services/Inc/svc_mag.h）+ 硬磁/软磁校正
 * （Driver/Inc/drv_mag_calibration.h）之后的机体 FLU 磁场，形态参照
 * APP_Current_GetSnapshot / APP_Battery_GetSnapshot——临界区拷贝，不做 I/O，
 * 不阻塞，可以在 500Hz 控制路径上安全调用。
 *
 * calibrated / axis_verified 是**有效**视图（已经过 App_Magcal 的
 * frame_contract_version 失效判定，见 app_magcal.h），姿态融合门控第 1/2
 * 条直接读这两个字段即可，不必再自己比对契约版本。
 */
typedef struct {
    /* SVC_Timestamp_Us() 在本次采样时刻的值；0 = 从未成功采样过。 */
    uint64_t timestamp_us;
    /* 机体 FLU，毫高斯；calibrated==0 时是恒等变换后的原始贴装值。 */
    float field_flu_mgauss[3];
    /* 本次（或最近一次）BSP 读取是否健康。 */
    uint8_t sensor_healthy;
    /* DRV_MAG_Calibration.calibrated 的有效值。 */
    uint8_t calibrated;
    /* 轴向验证的有效值（已按 frame_contract_version 失效判定）。 */
    uint8_t axis_verified;
    uint8_t reserved;
} APP_MAG_Snapshot;

void APP_MAG_Init(void);
void APP_MAG_Step(void);
void APP_MAG_GetStatus(APP_MAG_Status *status);
void APP_MAG_GetSnapshot(APP_MAG_Snapshot *out);
void APP_MAG_Report(void);
const char *APP_MAG_GetTypeName(uint8_t type);

#endif
