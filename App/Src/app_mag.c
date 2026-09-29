#include "app_mag.h"

#include "app_control.h"
#include "app_magcal.h"
#include "bsp_critical.h"
#include "bsp_mag.h"
#include "drv_frame_contract.h"
#include "drv_mag_calibration.h"
#include "svc_mag.h"
#include "svc_timestamp.h"

#include <string.h>

typedef struct {
    uint8_t initialized;
    BSP_MAG_StatusCode init_status;
    BSP_MAG_Status bsp_status;
    APP_MAG_Snapshot snapshot;
} APP_MAG_Context;

static APP_MAG_Context app_mag_ctx;

void APP_MAG_Init(void)
{
    BSP_MAG_StatusCode st;

    memset(&app_mag_ctx, 0, sizeof(app_mag_ctx));
    st = BSP_MAG_Init();
    app_mag_ctx.init_status = st;
    if (st == BSP_MAG_OK) {
        app_mag_ctx.initialized = 1U;
    }
}

/*
 * 贴装变换（svc_mag）+ 硬磁/软磁校正（drv_mag_calibration）之后写进缓存快照。
 * 20Hz 调用，纯浮点运算 + 一次临界区拷贝，不做额外 I/O，不阻塞。
 */
static void app_mag_update_snapshot(const BSP_MAG_ScaledData *scaled)
{
    DRV_FRAME_Vector3f chip_axis;
    DRV_FRAME_Vector3f flu_raw;
    DRV_MAG_Calibration effective_cal;
    float raw_flu_mgauss[3];
    float corrected_mgauss[3];
    DRV_MAG_CalibrationStatus apply_status;
    uint32_t lock;

    chip_axis.x = (float)scaled->x_mgauss;
    chip_axis.y = (float)scaled->y_mgauss;
    chip_axis.z = (float)scaled->z_mgauss;
    flu_raw = SVC_MAG_RotateToFlu(SVC_MAG_DefaultRotation(), chip_axis);
    raw_flu_mgauss[0] = flu_raw.x;
    raw_flu_mgauss[1] = flu_raw.y;
    raw_flu_mgauss[2] = flu_raw.z;

    APP_MagCal_GetEffective(&effective_cal);
    apply_status = DRV_MAG_Calibration_Apply(&effective_cal, raw_flu_mgauss,
                                             corrected_mgauss);

    lock = BSP_Critical_Enter();
    if (apply_status == DRV_MAG_CAL_VALID) {
        app_mag_ctx.snapshot.field_flu_mgauss[0] = corrected_mgauss[0];
        app_mag_ctx.snapshot.field_flu_mgauss[1] = corrected_mgauss[1];
        app_mag_ctx.snapshot.field_flu_mgauss[2] = corrected_mgauss[2];
        app_mag_ctx.snapshot.calibrated = effective_cal.calibrated;
        app_mag_ctx.snapshot.axis_verified = effective_cal.axis_verified;
    } else {
        /*
         * 一份连自己的数值自检都过不了的校准记录（Flash 损坏、行列式越界）
         * 绝不能被当成可用——失闭到"没有磁力计"，而不是继续用一份可能是
         * 半新半旧的场值。calibrated=0 已经足够：融合门控第 1 条要求
         * calibrated!=0，不管 axis_verified 是什么都会被排除在融合之外。
         */
        app_mag_ctx.snapshot.calibrated = 0U;
        app_mag_ctx.snapshot.axis_verified = 0U;
    }
    app_mag_ctx.snapshot.timestamp_us = SVC_Timestamp_Us();
    app_mag_ctx.snapshot.sensor_healthy = 1U;
    BSP_Critical_Exit(lock);
}

void APP_MAG_Step(void)
{
    BSP_MAG_RawData raw;
    BSP_MAG_ScaledData scaled;
    uint8_t read_ok;

    if (app_mag_ctx.initialized == 0U) {
        return;
    }

    read_ok = (BSP_MAG_Read(&raw, &scaled) == BSP_MAG_OK) ? 1U : 0U;
    if (read_ok == 0U) {
        app_mag_ctx.initialized = 0U;
    }
    BSP_MAG_GetStatus(&app_mag_ctx.bsp_status);

    if (read_ok != 0U) {
        app_mag_update_snapshot(&scaled);
    } else {
        uint32_t lock = BSP_Critical_Enter();
        app_mag_ctx.snapshot.sensor_healthy = 0U;
        BSP_Critical_Exit(lock);
    }
}

void APP_MAG_GetSnapshot(APP_MAG_Snapshot *out)
{
    uint32_t lock;

    if (out == NULL) {
        return;
    }
    lock = BSP_Critical_Enter();
    *out = app_mag_ctx.snapshot;
    BSP_Critical_Exit(lock);
}

void APP_MAG_GetStatus(APP_MAG_Status *status)
{
    const BSP_MAG_Status *bsp;

    if (status == NULL) {
        return;
    }

    BSP_MAG_GetStatus(&app_mag_ctx.bsp_status);
    bsp = &app_mag_ctx.bsp_status;
    memset(status, 0, sizeof(*status));

    status->initialized = app_mag_ctx.initialized;
    status->init_status = (int32_t)app_mag_ctx.init_status;
    status->last_status = (int32_t)bsp->last_status;
    status->type = (uint8_t)bsp->type;
    status->address = bsp->address;
    status->who_am_i = bsp->who_am_i;
    status->sample_count = bsp->sample_count;
    status->raw_x = bsp->raw.x;
    status->raw_y = bsp->raw.y;
    status->raw_z = bsp->raw.z;
    status->x_mgauss = bsp->scaled.x_mgauss;
    status->y_mgauss = bsp->scaled.y_mgauss;
    status->z_mgauss = bsp->scaled.z_mgauss;
    status->detected_ist8310 = bsp->detected_ist8310;
    status->detected_hmc5883 = bsp->detected_hmc5883;
    status->detected_qmc5883 = bsp->detected_qmc5883;
    status->hmc_id_a = bsp->hmc_id[0];
    status->hmc_id_b = bsp->hmc_id[1];
    status->hmc_id_c = bsp->hmc_id[2];
}

const char *APP_MAG_GetTypeName(uint8_t type)
{
    return BSP_MAG_TypeName((BSP_MAG_Type)type);
}

void APP_MAG_Report(void)
{
    APP_MAG_Status mag_status;

    APP_MAG_GetStatus(&mag_status);

    APP_Control_QueueText("MAG ok=%u init=%ld st=%ld type=%s addr=0x%02X who=0x%02X n=%lu raw=%d,%d,%d mgauss=%ld,%ld,%ld\r\n",
                           (unsigned int)mag_status.initialized,
                           (long)mag_status.init_status,
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
    APP_Control_QueueText("MAG probe ist=%u hmc=%u qmc=%u hmc_id=%02X%02X%02X\r\n",
                           (unsigned int)mag_status.detected_ist8310,
                           (unsigned int)mag_status.detected_hmc5883,
                           (unsigned int)mag_status.detected_qmc5883,
                           (unsigned int)mag_status.hmc_id_a,
                           (unsigned int)mag_status.hmc_id_b,
                           (unsigned int)mag_status.hmc_id_c);
}
