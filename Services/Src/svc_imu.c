/*
 * IMU 服务层实现。纯函数 + 纯数据，不含 HAL / RTOS / I/O。
 * 设计与符号陷阱的说明在 svc_imu.h，此处不重复。
 */

#include "svc_imu.h"

#include <stddef.h>

/*
 * 第一步：芯片轴 → **FRD** 机体轴，语义与 ArduPilot 的 ROTATION_* 完全一致。
 *
 * 各式子的来源（把基本旋转依次作用于向量，而不是背表）：
 *   YAW_90   : (x, y) -> (-y,  x)
 *   YAW_180  : (x, y) -> (-x, -y)
 *   YAW_270  : (x, y) -> ( y, -x)
 *   ROLL_180 : (y, z) -> (-y, -z)
 * 组合 ROLL_180_YAW_n 是"先 ROLL_180 再 YAW_n"，例如
 *   ROLL_180        -> ( x, -y, -z)
 *   再 YAW_270      -> (-y, -x, -z)   即 ROLL_180_YAW_270
 */
static DRV_FRAME_Vector3f rotate_chip_to_frd(SVC_IMU_Rotation rotation,
                                             DRV_FRAME_Vector3f v)
{
    DRV_FRAME_Vector3f out = v;

    switch (rotation) {
    case SVC_IMU_ROTATION_NONE:
        break;
    case SVC_IMU_ROTATION_YAW_90:
        out.x = -v.y; out.y =  v.x; out.z =  v.z;
        break;
    case SVC_IMU_ROTATION_YAW_180:
        out.x = -v.x; out.y = -v.y; out.z =  v.z;
        break;
    case SVC_IMU_ROTATION_YAW_270:
        out.x =  v.y; out.y = -v.x; out.z =  v.z;
        break;
    case SVC_IMU_ROTATION_ROLL_180:
        out.x =  v.x; out.y = -v.y; out.z = -v.z;
        break;
    case SVC_IMU_ROTATION_ROLL_180_YAW_90:
        out.x =  v.y; out.y =  v.x; out.z = -v.z;
        break;
    case SVC_IMU_ROTATION_PITCH_180:
        out.x = -v.x; out.y =  v.y; out.z = -v.z;
        break;
    case SVC_IMU_ROTATION_ROLL_180_YAW_270:
        out.x = -v.y; out.y = -v.x; out.z = -v.z;
        break;
    default:
        /*
         * 未知朝向按"不旋转"处理，而不是返回零或随便挑一个。
         * 姿态会明显不对、一眼能看出来；返回零反而像"传感器坏了"，更难定位。
         */
        break;
    }

    return out;
}

DRV_FRAME_Vector3f SVC_IMU_RotateToFlu(SVC_IMU_Rotation rotation,
                                       DRV_FRAME_Vector3f chip_axis_value)
{
    /* 两步走，第二步复用坐标契约里既有的适配器，不另起一套符号约定。 */
    DRV_FRAME_Vector3f frd = rotate_chip_to_frd(rotation, chip_axis_value);
    return DRV_FRAME_FrdToFlu(frd);
}

void SVC_IMU_ApplyMounting(SVC_IMU_Rotation rotation,
                           const DRV_IMU_ScaledData *chip_axis,
                           DRV_IMU_ScaledData *body_flu)
{
    DRV_FRAME_Vector3f accel;
    DRV_FRAME_Vector3f gyro;
    float temperature_c;

    if ((chip_axis == NULL) || (body_flu == NULL)) { return; }

    accel.x = chip_axis->accel_x_g;
    accel.y = chip_axis->accel_y_g;
    accel.z = chip_axis->accel_z_g;

    gyro.x = chip_axis->gyro_x_dps;
    gyro.y = chip_axis->gyro_y_dps;
    gyro.z = chip_axis->gyro_z_dps;

    /* 先取出温度，允许 in 与 out 是同一个指针。 */
    temperature_c = chip_axis->temperature_c;

    accel = SVC_IMU_RotateToFlu(rotation, accel);
    gyro = SVC_IMU_RotateToFlu(rotation, gyro);

    body_flu->accel_x_g = accel.x;
    body_flu->accel_y_g = accel.y;
    body_flu->accel_z_g = accel.z;

    body_flu->gyro_x_dps = gyro.x;
    body_flu->gyro_y_dps = gyro.y;
    body_flu->gyro_z_dps = gyro.z;

    body_flu->temperature_c = temperature_c;
}

SVC_IMU_Rotation SVC_IMU_DefaultRotation(DRV_IMU_ChipKind kind)
{
    switch (kind) {
    case DRV_IMU_CHIP_BMI088: return SVC_IMU_ROTATION_ROLL_180_YAW_270;
    case DRV_IMU_CHIP_BMI270: return SVC_IMU_ROTATION_ROLL_180;
    case DRV_IMU_CHIP_ICM42688:
    case DRV_IMU_CHIP_NONE:
    default:                  return SVC_IMU_ROTATION_NONE;
    }
}

void SVC_IMU_ChipToIntermediate(DRV_IMU_ChipKind kind,
                                const float in[3],
                                float out[3])
{
    DRV_FRAME_Vector3f chip;
    DRV_FRAME_Vector3f frd;

    if ((in == NULL) || (out == NULL)) { return; }

    if (kind == DRV_IMU_CHIP_ICM42688) {
        /*
         * 老板子的实测安装：IMU +X 朝机左、+Y 朝机上、+Z 朝机前。
         * 这三行与迁移前 app_sensor.c 里的固定映射逐字节相同，
         * tests/test_flu_seam0_sensor_frame.py 钉的就是它，不能改。
         */
        out[0] = -in[2];
        out[1] = -in[0];
        out[2] =  in[1];
        return;
    }

    chip.x = in[0];
    chip.y = in[1];
    chip.z = in[2];

    frd = rotate_chip_to_frd(SVC_IMU_DefaultRotation(kind), chip);

    /* 中间轴是「X 后 / Y 右 / Z 上」，相对 FRD 只翻 X 和 Z。 */
    out[0] = -frd.x;
    out[1] =  frd.y;
    out[2] = -frd.z;
}

uint8_t SVC_IMU_DefaultOrientationCode(DRV_IMU_ChipKind kind)
{
    switch (kind) {
    case DRV_IMU_CHIP_BMI088:
    case DRV_IMU_CHIP_BMI270:
        return SVC_IMU_ORIENTATION_CODE_MICOAIR;
    case DRV_IMU_CHIP_ICM42688:
    case DRV_IMU_CHIP_NONE:
    default:
        return SVC_IMU_ORIENTATION_CODE_LEGACY;
    }
}

void SVC_IMU_SelectionReset(SVC_IMU_Selection *selection)
{
    uint32_t i;

    if (selection == NULL) { return; }

    selection->selected = DRV_IMU_CHIP_NONE;
    selection->rotation = SVC_IMU_ROTATION_NONE;
    selection->selected_chip_id = 0U;
    selection->probe_count = 0U;

    for (i = 0U; i < 4U; i++) {
        selection->probed_kind[i] = (uint8_t)DRV_IMU_CHIP_NONE;
        selection->probed_chip_id[i] = 0U;
        selection->probed_status[i] = DRV_IMU_ERROR;
        selection->init_status[i] = SVC_IMU_INIT_NOT_ATTEMPTED;
    }
}

void SVC_IMU_SelectionRecordInit(SVC_IMU_Selection *selection,
                                 DRV_IMU_ChipKind kind,
                                 DRV_IMU_Status status)
{
    uint32_t i;

    if (selection == NULL) { return; }

    /* 按 kind 回填到对应的探测槽位；没有对应槽位就丢弃（记录已满时会发生）。 */
    for (i = 0U; i < (uint32_t)selection->probe_count; i++) {
        if (selection->probed_kind[i] == (uint8_t)kind) {
            selection->init_status[i] = status;
            return;
        }
    }
}

void SVC_IMU_SelectionRecord(SVC_IMU_Selection *selection,
                             DRV_IMU_ChipKind kind,
                             uint8_t chip_id,
                             DRV_IMU_Status status)
{
    if (selection == NULL) { return; }

    /*
     * 探测记录本身有上限，写满就不再记（但选型逻辑照常），
     * 这样一颗反复重连的传感器不会把记录冲掉或越界。
     */
    if (selection->probe_count < 4U) {
        selection->probed_kind[selection->probe_count] = (uint8_t)kind;
        selection->probed_chip_id[selection->probe_count] = chip_id;
        selection->probed_status[selection->probe_count] = status;
        selection->init_status[selection->probe_count] = SVC_IMU_INIT_NOT_ATTEMPTED;
        selection->probe_count++;
    }

    /*
     * 到此为止：只记账。选中与否由 SVC_IMU_SelectionCommit 在 init 成功后决定，
     * 原因见 svc_imu.h 里这个函数的注释。
     */
}

void SVC_IMU_SelectionCommit(SVC_IMU_Selection *selection,
                             DRV_IMU_ChipKind kind,
                             uint8_t chip_id)
{
    if (selection == NULL) { return; }

    selection->selected = kind;
    selection->selected_chip_id = chip_id;
    selection->rotation = SVC_IMU_DefaultRotation(kind);
}

const char *SVC_IMU_ChipName(DRV_IMU_ChipKind kind)
{
    switch (kind) {
    case DRV_IMU_CHIP_ICM42688: return "ICM42688";
    case DRV_IMU_CHIP_BMI088:   return "BMI088";
    case DRV_IMU_CHIP_BMI270:   return "BMI270";
    case DRV_IMU_CHIP_NONE:
    default:                    return "NONE";
    }
}
