/*
 * 辨识台架几何。全部是纯算术；能在 PC 上判对错，因此不允许出现任何硬件依赖。
 */
#include "drv_sysid_rig.h"

#include <math.h>
#include <stddef.h>

static uint8_t rig_finite(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

static uint8_t rig_valid(const DRV_SysIdRig *rig)
{
    if (rig == NULL) {
        return 0U;
    }
    return (rig_finite(rig->azimuth_rad) != 0U) &&
           (rig_finite(rig->axis_offset_above_cg_m) != 0U) &&
           (rig_finite(rig->imu_above_cg_m) != 0U);
}

DRV_SysIdRigStatus DRV_SysIdRig_Axis(const DRV_SysIdRig *rig, float axis_out[3])
{
    if ((rig_valid(rig) == 0U) || (axis_out == NULL)) {
        return DRV_SYSID_RIG_INVALID;
    }
    axis_out[0] = cosf(rig->azimuth_rad);
    axis_out[1] = sinf(rig->azimuth_rad);
    axis_out[2] = 0.0f;
    return DRV_SYSID_RIG_OK;
}

DRV_SysIdRigStatus DRV_SysIdRig_EffectiveInertia(const DRV_SysIdRig *rig,
                                                 const float inertia[3],
                                                 float *out)
{
    float axis[3];

    if ((inertia == NULL) || (out == NULL) ||
        (DRV_SysIdRig_Axis(rig, axis) != DRV_SYSID_RIG_OK)) {
        return DRV_SYSID_RIG_INVALID;
    }
    for (uint32_t i = 0U; i < 3U; ++i) {
        if ((rig_finite(inertia[i]) == 0U) || (inertia[i] <= 0.0f)) {
            return DRV_SYSID_RIG_INVALID;
        }
    }
    /* nᵀ J n，J 对角。n_z = 0 时 Jzz 不参与——这正是绕杆轴只看 Jxx/Jyy 的原因。 */
    *out = (inertia[0] * axis[0] * axis[0]) +
           (inertia[1] * axis[1] * axis[1]) +
           (inertia[2] * axis[2] * axis[2]);
    return DRV_SYSID_RIG_OK;
}

DRV_SysIdRigStatus DRV_SysIdRig_ProjectRate(const DRV_SysIdRig *rig,
                                            const float omega[3],
                                            float *out)
{
    float axis[3];

    if ((omega == NULL) || (out == NULL) ||
        (DRV_SysIdRig_Axis(rig, axis) != DRV_SYSID_RIG_OK)) {
        return DRV_SYSID_RIG_INVALID;
    }
    for (uint32_t i = 0U; i < 3U; ++i) {
        if (rig_finite(omega[i]) == 0U) {
            return DRV_SYSID_RIG_INVALID;
        }
    }
    *out = (omega[0] * axis[0]) + (omega[1] * axis[1]) + (omega[2] * axis[2]);
    return DRV_SYSID_RIG_OK;
}

DRV_SysIdRigStatus DRV_SysIdRig_RateResidual(const DRV_SysIdRig *rig,
                                             const float omega[3],
                                             float *out)
{
    float axis[3];
    float along = 0.0f;
    float sum = 0.0f;

    if ((out == NULL) ||
        (DRV_SysIdRig_Axis(rig, axis) != DRV_SYSID_RIG_OK) ||
        (DRV_SysIdRig_ProjectRate(rig, omega, &along) != DRV_SYSID_RIG_OK)) {
        return DRV_SYSID_RIG_INVALID;
    }
    for (uint32_t i = 0U; i < 3U; ++i) {
        const float perpendicular = omega[i] - (along * axis[i]);
        sum += perpendicular * perpendicular;
    }
    *out = sqrtf(sum);
    return DRV_SYSID_RIG_OK;
}

DRV_SysIdRigStatus DRV_SysIdRig_MomentAboutAxis(const DRV_SysIdRig *rig,
                                                float moment_n_m,
                                                float moment_out[3])
{
    float axis[3];

    if ((moment_out == NULL) || (rig_finite(moment_n_m) == 0U) ||
        (DRV_SysIdRig_Axis(rig, axis) != DRV_SYSID_RIG_OK)) {
        return DRV_SYSID_RIG_INVALID;
    }
    for (uint32_t i = 0U; i < 3U; ++i) {
        moment_out[i] = moment_n_m * axis[i];
    }
    return DRV_SYSID_RIG_OK;
}

DRV_SysIdRigStatus DRV_SysIdRig_GravityMoment(const DRV_SysIdRig *rig,
                                              float mass_kg, float gravity_m_s2,
                                              float angle_rad, float *out)
{
    if ((rig_valid(rig) == 0U) || (out == NULL) ||
        (rig_finite(mass_kg) == 0U) || (mass_kg <= 0.0f) ||
        (rig_finite(gravity_m_s2) == 0U) || (gravity_m_s2 <= 0.0f) ||
        (rig_finite(angle_rad) == 0U)) {
        return DRV_SYSID_RIG_INVALID;
    }
    /*
     * 杆在质心上方（d > 0）时是**稳定**单摆：偏离后重力把机体拉回，恢复力矩与
     * 转角反号，所以这里带负号。d = 0 时整项消失，退化成纯双积分。
     */
    *out = -mass_kg * gravity_m_s2 * rig->axis_offset_above_cg_m * sinf(angle_rad);
    return DRV_SYSID_RIG_OK;
}
