#ifndef DRV_SYSID_RIG_H
#define DRV_SYSID_RIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 辨识台架的几何契约：**纯函数**，没有寄存器、没有 RTOS、没有全局可变量。
 *
 * 当前台架是一根光杆，从机体 XY 之间 45° 方向穿过、且穿过质心，摩擦可忽略。
 * 机体只能绕这根杆转，是一个**单自由度**系统。这带来四个直接可用的性质，
 * 整套内环辨识都建在上面：
 *
 * 1. **绕杆轴的有效惯量就是 Ixx。**
 *    J = diag(Ixx, Iyy, Izz)，机体左右前后近似对称所以 Ixx ≈ Iyy = I，
 *    杆轴 n = (cos ψ, sin ψ, 0) 的 z 分量为 0，于是
 *        I_n = nᵀ J n = I·cos²ψ + I·sin²ψ = I
 *    也就是说**绕 45° 轴辨出来的惯量直接就是 Ixx/Iyy**，不必分两个轴做。
 *    这正是"XY 耦合、惯量当一致"这个前提在数学上的出口。
 *
 * 2. **激励必须把力矩打在 n 上。**
 *    垂直于 n 的力矩分量全部被光杆的约束力吃掉：不产生任何运动，只产生轴承载荷。
 *    所以期望力矩是 τ_body = τ_n · n，两个舵机因此是**联动**的，不是单轴激励。
 *
 * 3. **数据自检有硬判据。**
 *    理想情况下实测 ω 必须平行于 n，即垂直分量为零。这个残差是"执行结果符不符合"
 *    的第一道机检：超限说明杆不在 ψ 方向、机体没夹紧、或轴向填错了，
 *    该趟数据必须作废而不是拿去拟合。
 *
 * 4. **杆过质心 ⇒ 没有重力恢复力矩。**
 *    但"准不准过质心"是未知的，所以辨识模型里保留残余偏心项由数据说话，
 *    不在这里假定它为零。
 *
 * ──────────────── 坐标与符号 ────────────────
 *
 * 一律用 `Driver/Inc/drv_frame_contract.h` 的规范 FLU 机体系：+X 前、+Y 左、+Z 上。
 * ψ 是杆轴在 XY 平面内相对 +X 的方位角，右手绕 +Z 为正，单位 rad。
 * 当前台架 ψ = π/4（45°）。角速度 rad/s，力矩 N·m，惯量 kg·m²。
 */

typedef enum {
    DRV_SYSID_RIG_OK = 0,
    DRV_SYSID_RIG_INVALID
} DRV_SysIdRigStatus;

/* 当前光杆台架的方位角：XY 之间 45°。 */
#define DRV_SYSID_RIG_AZIMUTH_45_RAD  0.78539816339744830961f

typedef struct {
    /* 杆轴方位角 [rad]，XY 平面内相对 +X，绕 +Z 为正。 */
    float azimuth_rad;
    /*
     * 杆轴相对整机质心的**垂直偏移** [m]，+ 为杆在质心上方。
     * 名义为 0（杆过质心）。非零时会引入 -m·g·d·sinθ 的单摆恢复力矩，
     * 这一项由辨识拟合出来，不在这里假定。
     */
    float axis_offset_above_cg_m;
    /*
     * 飞控/IMU 相对整机质心的垂直偏移 [m]，+ 为 IMU 在质心上方。
     * 取自 `airframe.imu_z_m - airframe.cg_z_m`。
     *
     * 角速度是刚体不变量，不受它影响——所以**内环辨识以陀螺为准**。
     * 但加速度计会多出 ω̇×r 与 ω×(ω×r)：杆过质心时质心线加速度为零，
     * 这两项就是姿态角估计误差的主要来源，姿态只能做交叉校验与限位。
     */
    float imu_above_cg_m;
} DRV_SysIdRig;

/* 方位角 -> 机体系单位向量 n = (cos ψ, sin ψ, 0)。 */
DRV_SysIdRigStatus DRV_SysIdRig_Axis(const DRV_SysIdRig *rig, float axis_out[3]);

/*
 * 绕杆轴的有效惯量 nᵀ J n。`inertia` 是对角 J=[Jxx,Jyy,Jzz]。
 * 对称机体 + ψ 任意 + n_z=0 时结果恒等于 Jxx（见文件头第 1 条）。
 */
DRV_SysIdRigStatus DRV_SysIdRig_EffectiveInertia(const DRV_SysIdRig *rig,
                                                 const float inertia[3],
                                                 float *out);

/* 实测角速度在杆轴上的投影 ω·n，即这套系统唯一的自由度。 */
DRV_SysIdRigStatus DRV_SysIdRig_ProjectRate(const DRV_SysIdRig *rig,
                                            const float omega[3],
                                            float *out);

/*
 * 角速度垂直于杆轴的**残差范数** ‖ω − (ω·n)n‖。
 * 理想台架恒为 0；超过门限即判本趟数据无效。这是"结果符不符合"的机检入口。
 */
DRV_SysIdRigStatus DRV_SysIdRig_RateResidual(const DRV_SysIdRig *rig,
                                             const float omega[3],
                                             float *out);

/*
 * 绕杆轴的期望力矩 τ_n -> 机体系力矩矢量 τ_n · n。
 * 调用方把它交给 `DRV_COAX_CTRL_SolveBodyTiltFromMoment` 反解倾角。
 */
DRV_SysIdRigStatus DRV_SysIdRig_MomentAboutAxis(const DRV_SysIdRig *rig,
                                                float moment_n_m,
                                                float moment_out[3]);

/*
 * 单摆恢复力矩 -m·g·d·sin(θ)，d 为杆轴相对质心的偏移。
 * d = 0（杆过质心）时恒返回 0。θ 是绕杆轴的转角 [rad]。
 */
DRV_SysIdRigStatus DRV_SysIdRig_GravityMoment(const DRV_SysIdRig *rig,
                                              float mass_kg, float gravity_m_s2,
                                              float angle_rad, float *out);

#ifdef __cplusplus
}
#endif
#endif
