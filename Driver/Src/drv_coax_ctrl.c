#include "drv_coax_ctrl.h"

#include "bsp_pwm.h"
#include "drv_airframe_params.h"
#include "drv_att_reference.h"
#include "drv_hover_adapt.h"
#include "drv_moment_notch.h"
#include "drv_prop_map.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRV_COAX_CTRL_TILT_LIMIT_RAD 0.4886922f
#define DRV_COAX_CTRL_PI 3.141592654f
/*
 * 力矩反解允许的最小总推力 [N]。低于它就认为"这架飞机现在产生不了倾转力矩"。
 * 0.5 N 约合 50 g，远低于任何能起转的工况，只用来挡住零推力那个退化情形。
 */
#define COAX_CTRL_MIN_SOLVE_FORCE_N 0.5f
#define DRV_COAX_CTRL_SERVO_TRAVEL_RAD \
    (DRV_COAX_CTRL_SERVO_TRAVEL_DEG * DRV_COAX_CTRL_PI / 180.0f)
#define DRV_COAX_CTRL_SERVO_LIMIT_RAD \
    (DRV_COAX_CTRL_SERVO_LIMIT_DEG * DRV_COAX_CTRL_PI / 180.0f)
/* ════════════════════════════════════════════════════════════════════════ */
/*  极性约定（唯一声明处）                                                   */
/*                                                                        */
/*  从传感器到舵机这条链路上曾经散落着 8 个互相独立的符号开关，2^8 = 256    */
/*  种组合里只有少数是自洽的，而且它们的效果会互相掩盖：负增益在数学上等价  */
/*  于翻转符号，所以极性错误可以被"把增益调成负的"吸收掉——飞机看起来能自稳， */
/*  但摇杆方向是反的。这正是极性问题反复出现的原因：自稳只验证了"负反馈"    */
/*  一个条件，不验证绝对方向。                                              */
/*                                                                        */
/*  因此本文件把所有符号集中在这里，每一项都写明物理含义，并由              */
/*  tests/test_coax_sign_convention.py 逐条锁定。增益一律为正值，负反馈由   */
/*  控制律的 -K_R*e_R - K_w*e_w 结构保证——极性错了飞机会立刻发散，而不是    */
/*  悄悄反向工作。                                                          */
/*                                                                        */
/*  姿态约定：本层拿到的角度**口径由姿态融合的 convention 决定**，而它按    */
/*  APP_Sensor_IsFluOrientationActive() 在 NED / NWU 之间切换：              */
/*    roll_rad  > 0  →  机身右侧下沉   （两种口径相同，迁移不改它）         */
/*    pitch_rad > 0  →  legacy(NED)=机头上仰；FLU(NWU)=机头**下俯**         */
/*    gyro_x    > 0  →  正 roll 方向的角速率                               */
/*    gyro_y    > 0  →  正 pitch 方向的角速率                              */
/*  可执行证据见 tests/test_flu_seam1_estimator_frame.py。                  */
/*                                                                        */
/*  入口口径：Reference / AttitudeInput 的 x_m/y_m/z_m、vx/vy/vz_m_s 与     */
/*  姿态、角速率全部是规范 FLU。上游把安装差异消化在传感器出口的唯一        */
/*  Adapter 里（光流见 app_stabilizer.c 的 stabilizer_compensate_flow_       */
/*  rotation），本层不再有任何"再取一次反"的边界适配，也不得新增。          */
/*  因此力坐标系不需要符号补偿，曾经的                                      */
/*  DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN / _PITCH_SIGN /                     */
/*  DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN / _PITCH_SIGN 四个常量已删除：       */
/*  coax_ctrl_rpy_matrix 直接吃 FLU 角度，标准 ZYX 欧拉矩阵公式对任何右手系 */
/*  都成立，FLU 的 roll/pitch/yaw 本来就是绕自己 X/Y/Z 的右手旋转           */
/*  （见 drv_frame_contract.h），无需额外补偿。                             */
/*                                                                        */
/*  出口口径：力矩 → 倾转 → 舵机这一段的符号也不再是常量开关。倾转到力矩的 */
/*  力臂（含符号）由实测几何推出：重心 z − 舵机转轴 z（见下方"倾转 → 机体  */
/*  力矩"），倾转到舵机的 90° 机构映射是固定运动学                          */
/*  （coax_ctrl_body_tilt_to_servo_tilts），唯一可变量是上位机机械标定写进来 */
/*  的 ServoCalibration。                                                    */
/* ════════════════════════════════════════════════════════════════════════ */

#define DRV_COAX_CTRL_FORCE_EPS_N          1.0e-4f
/*
 * 总推力给偏航留的差速余量 [N]：合推力不超过 2·T单max − 此值，任何时候上下桨都还有 ±一半可差。
 * 2026-10-01 自由飞（receive_lok1zuby 72.8 s）：大倾角掉高、两桨顶满 1940 µs，偏航余量归零，下桨反扭矩
 * 偏大的那部分没人抵（悬停时积分常驻约 0.0014 N·m ≈ 0.28 N 差速），机身转到 −170 °/s，光流因转速判无效，
 * 高度环随之冻结在最大推力上冲到 1.7 m。0.8 N 约为该偏差的 1.4 倍，代价是满油门少约 5%。
 */
#define DRV_COAX_CTRL_YAW_RESERVE_N        0.8f
#define DRV_COAX_CTRL_RATE_SCALE_EPS       1.0e-6f
#define DRV_COAX_CTRL_SERVO_ANGLE_TOL_RAD  8.0e-4f
/*
 * 桨的反扭矩系数：每 1 N 推力产生多少 N·m 反扭矩，单位是米。
 * 净偏航力矩 Mz = k·(T_下 − T_上)；分配器反解 ΔT = Mz/k。
 *
 * ⚠ 2026-09-07 由 0.0001 改为 0.005。**这是个工程估计值，不是实测**——作者知情
 * 并明确要求"先给一个合理的虚拟 K"。原值 0.0001 在 doc/ 中**查无出处**，既无
 * 辨识记录也无引用来源。
 *
 * 估计依据：9047 桨 D = 0.2286 m，小型螺旋桨的 C_Q/C_T 约 0.02~0.03，
 * 故 Q/T ≈ 0.02 × 0.2286 ≈ 5e-3 m。原值比这小约 50 倍。
 * 旁证：按原值算，偏航最大角加速度只有 7e-4/0.005 = 0.14 rad/s²（8°/s²），
 * 到 60°/s 要 7.5 秒——与作者能明显感觉到偏航力矩变化的实测不符。
 *
 * **改它不会让飞机变有劲**，k 只是"N·m 数字"与"推力差"之间的齿轮比：
 *   物理动作 ΔT = rate_kp·e / k，力矩上限换算成推力差后是 k 无关的
 *   （ΔT_max = 2·min(F/2, T_max − F/2) ≈ 7 N，只取决于实测的 F 与 T_max）。
 * 它带来的是**单位变得可解释**：rate_kp/I_zz 才真的等于内环带宽，力矩上限
 * 才真的对应一个可信的角加速度。
 *
 * 同步补偿：默认 rate.kp[2] 按同一比例放大，使物理行为逐位不变（见下方）。
 * 但**持久化在 Flash 里的用户增益不会自动跟着换算**——按新 k 调过的值与按旧 k
 * 调的值相差 50 倍，重新标定前请以本次为准重调。
 *
 * 待办：一次系留纯偏航台阶（固定推力差、记 gyro_z、取初始斜率）即可定住
 * k/I_zz 这个比值——控制律真正需要的也只是这个比值。
 */
#define DRV_COAX_CTRL_PROP9047_YAW_M_PER_N 0.005f
#define DRV_COAX_CTRL_SINGLE_MAX_THRUST_N 10.2f   /* 封顶；实际可用值见 coax_ctrl_single_max_thrust_n() */
#define DRV_COAX_CTRL_SINGLE_MAX_THRUST_MIN_N 2.0f /* 映射给出低于此值视为电压未知，退回封顶 */
#define DRV_COAX_CTRL_THRUST_TABLE_POINTS  21U
#define DRV_COAX_CTRL_GRAMS_PER_NEWTON     101.971621f
/*
 * 倾转 → 机体力矩。大小和方向都直接从实测机体几何读出来：没有经验系数，
 * 也没有可调符号，想改它只能重新测量飞机。
 *
 * 规范 FLU，推力大小 T，倾转角的正方向由 coax_ctrl_body_tilt_to_servo_tilts()
 * 钉死（那里写明了机构运动学的实测依据）：
 *     body_y_tilt > 0  →  推力轴倒向 +Y（左）  →  F_y = +T*sin(tilt)
 *     body_x_tilt > 0  →  推力轴倒向 -X（后）  →  F_x = -T*sin(tilt)
 *
 * 力臂取哪一点？电机轴线穿过舵机转轴，所以推力的**作用线**过转轴。τ = r × F
 * 只取决于作用线在哪，与力沿线作用在哪一点无关——桨装高一点还是低一点，只是
 * 沿作用线滑动，力矩一分不变。于是 r 取"重心 → 倾转转轴"，r = (0, 0, r_z)，
 * r_z = 转轴 z − 重心 z（DRV_Airframe_Roll/PitchTiltAxisToCgZ）：
 *     τ_roll  = -r_z1 * F_y = -r_z1 * T * sin(body_y_tilt)     （1 号舵机转轴）
 *     τ_pitch =  r_z2 * F_x = -r_z2 * T * sin(body_x_tilt)     （2 号舵机转轴）
 * 俯仰保留原有的 cos(body_y_tilt) 耦合项。控制律把 -r_z 存成带符号的力臂
 * coax_ctrl_params.roll/pitch_tilt_lever_arm_m（= 重心 z − 转轴 z，转轴在
 * 重心下方为正），见 coax_ctrl_apply_fixed_model_params()。
 *
 * 2026-09-27 板上存的几何：两个转轴实测都在 z = −0.13 m（相交），重心取部件
 * 表算出的 −0.094558 m，r_z = −0.035442 m、力臂 +0.035442 m。这组数**不是**
 * 已确认的真实力臂：作者随后实测重心约 −0.01 m（力臂约 0.12 m），部件表待
 * 台架刚度测试后再改。两组都是转轴在重心下方，力臂为正，正倾转产生正的 FLU
 * 力矩——符号结论不受影响，大小要以改好的机体模型为准。
 *
 * 拿飞机而不是拿代码复核一遍：转轴在重心下方，把推力倒向左边等于把机体
 * 下半部往左推，上半部就往右倒——右翼下沉，按 drv_frame_contract.h 正是
 * +roll。这一步反直觉，但叉乘和实物是一致的。
 *
 * 两轴各用自己的 r_z，数学上可以一正一负。能解锁的飞机上不会：闸门要求两个
 * 转轴都与推力点在重心同侧（DRV_Airframe_FirstInvalidName），转轴为 0（没填）
 * 或离重心不到 1 cm 也拒绝解锁——转轴留 0 会让 r_z 变成 +0.0946 m，两轴极性
 * 同时反掉，那是起飞即翻。
 *
 * ── 2026-09-27 之前是怎么算的，为什么改 ──
 * 力矩写成 极性 × EFFECTIVENESS（横滚 0.581 / 俯仰 0.569）× 0.145 m × T × sin，
 * 极性取自推力点（thrust_point_to_cg_z_m）。0.145 m 与两个 EFFECTIVENESS 出自
 * 2026-07-25 的辨识，"力臂是重心到对应舵机转轴的距离"，量的是**旧机体
 * （1.367 kg）**；机体换成 0.7546 kg 后没人更新，有效力臂一直停在
 * 0.0842/0.0825 m，而按当晚板上几何只有 0.0354 m（高估约 2.3 倍；按作者随后
 * 实测的重心则是 0.12 m，反成低估——哪一种都说明那个数早已和机体脱节）。
 * 光杆辨识（data/identification/attitude/2026-09-27）得到的"推力 × 舵机角"
 * 标度 k 按当晚板上几何约 0.8~1.0 倍几何力臂（按实测重心重算为 0.45，疑有
 * 台架刚度或其他链路误差，待查），再乘一个经验系数已没有依据，所以两个
 * EFFECTIVENESS 宏删掉，而不是改值。推力点也不再决定符号：作用线过转轴，
 * 真正起作用的是转轴的位置；推力点只留在闸门里做方向交叉核对。
 */

/*
 * 偏航力矩极性。和倾转极性一样，它不是可调符号，而是由桨的旋向推出来的。
 *
 * 桨对机体的反作用力矩与自身旋向相反，两桨共轴反转，令 s = 下桨旋向：
 *     Mz = -s_upper*ku*T_upper - s_lower*kl*T_lower
 *        = s*(ku*T_upper - kl*T_lower)            (s_upper = -s)
 *        = -s*(kl*T_lower - ku*T_upper)
 * 因此本极性 = -s。s = -1（下桨俯视顺时针）时极性为 +1，即
 *     Mz = +(kl*T_lower - ku*T_upper)
 * ——正偏航力矩靠**加大下桨**推力获得。
 *
 * 2026-09-13：s 的来源从 `airframe.lower_rotor_spin_sense` 换成上位机标定的
 * `drv_prop_map`。那个旧字段是**从调参现象反推的**（偏航 Kp 加大会抖振而不是
 * 发散 → 闭环是负反馈 → 反推出下桨顺时针）。推理自洽，但它证明的是"整条链的
 * 符号彼此不矛盾"，不是"桨真的往那边转"：换一套增益、或者把某处符号和它一起
 * 翻过来，现象一模一样，而飞机的偏航方向已经反了。现在 s 来自人在上位机上
 * 通电看一眼填进去的事实，与任何增益的正负无关。
 *
 * 未标定时返回 0：偏航通道因此没有权限，而不是朝一个猜出来的方向使劲。
 * 解锁在更前面就被挡住了（`App/Src/app_stabilizer.c` 的解锁链）。
 * 要改只改标定（上位机「桨叶与电机方向」页），本文件与遥控映射都不该动。
 */
static float coax_ctrl_yaw_torque_polarity(void)
{
    return DRV_PropMap_YawTorquePolarity();
}
#define DRV_COAX_CTRL_HORIZONTAL_ACCEL_LIMIT_M_S2 3.70f
#define DRV_COAX_CTRL_VEL_D_ACCEL_LIMIT_M_S2 3.70f
#define DRV_COAX_CTRL_POS_Z_I_ACCEL_LIMIT_M_S2 1.50f
#define DRV_COAX_CTRL_DT_MAX_S 0.05f
#define DRV_COAX_CTRL_ATTITUDE_PROTECT_START_RAD 0.436332f
#define DRV_COAX_CTRL_ATTITUDE_PROTECT_END_RAD   0.785398f
#define DRV_COAX_CTRL_MOMENT_PROTECT_START       0.95f
#define DRV_COAX_CTRL_MOMENT_PROTECT_END         1.15f
#define DRV_COAX_CTRL_THRUST_PROTECT_START       1.05f
#define DRV_COAX_CTRL_THRUST_PROTECT_END         1.25f

typedef struct {
    const char *name;
    uint16_t offset;
} DRV_COAX_CTRL_ParamEntry;

typedef struct {
    DRV_POSITION_CONTROL_State position;
    DRV_POSITION_CONTROL_PositionOutput position_output;
    DRV_POSITION_CONTROL_VelocityOutput velocity_output;
    DRV_AttitudeControl_Output attitude_output;
    DRV_RateControl_State rate;
    DRV_RateControl_Output rate_output;
    DRV_POSITION_CONTROL_SaturationFeedback translation_saturation;
    uint8_t moment_saturation_positive[3];
    uint8_t moment_saturation_negative[3];
} DRV_COAX_CTRL_State;

typedef struct {
    float desired_force_local_n[3];
    float desired_force_body_n[3];
    float desired_body_r[3][3];
    float attitude_error[3];
    float attitude_error_angle_rad;
    float attitude_tilt_error_rad;
    float rate_error_rad_s[3];
    float moment_cmd_n_m[3];
    /*
     * 真正下发给差动推力分配器的偏航力矩：已按 coax_ctrl_yaw_limit_moment()
     * 钳过。moment_cmd_n_m[2] 保持**未钳**的原始需求，遥测因此还能看出
     * "要了多少 / 给了多少"的差距；分配器只能用这一个。
     */
    float yaw_moment_applied_n_m;
    float alpha_rad;
    float beta_rad;
    float total_force_n;
    float raw_total_force_n;
    float moment_utilization;
    float thrust_utilization;
} DRV_COAX_CTRL_BalanceSolution;

/*
 * 姿态指令整形与力矩出口陷波的状态（横滚/俯仰）。**不在** coax_ctrl_state 里：
 * 直接姿态模式下 integrator_reset 每拍都成立，RunScheduled 每拍清一次回路状态；
 * 参考模型若跟着清，每拍都会对齐到实测角，角度环误差恒为 0——等于没有角度反馈。
 * 所以它只在公开的 DRV_COAX_CTRL_ResetState()（解锁前低油门、IMU 失效、辨识占用、
 * 改参数……）与目标来源切换时对齐。参考历史约 1.3 KB，放 AXI SRAM（NOLOAD，由
 * ResetState 显式初始化，DRV_COAX_CTRL_Init 保证先于第一次 Run），不占 DTCM。
 */
typedef struct {
    DRV_MomentNotch notch;
    DRV_MomentNotch notch2;   /* 第二级出口陷波，串在 notch 之后 */
    DRV_AttRefStep ref_pending;
    uint8_t ref_pending_valid;
    uint8_t ref_source;   /* 0 = 未知，1 = 力矢量（位置/速度环），2 = 直接姿态目标 */
} DRV_COAX_CTRL_Shaping;

#if defined(__GNUC__) && defined(__arm__)
#define COAX_CTRL_AXI_NOINIT __attribute__((section(".ram_d1_noinit"), aligned(32)))
#else
#define COAX_CTRL_AXI_NOINIT
#endif

static uint8_t coax_ctrl_initialized;
static DRV_COAX_CTRL_Params coax_ctrl_params;
static const DRV_COAX_CTRL_ThrustMap *volatile coax_ctrl_thrust_map;
static DRV_COAX_CTRL_ServoCalibration coax_ctrl_servo_calibration;
static DRV_COAX_CTRL_Debug coax_ctrl_last_debug;
static DRV_COAX_CTRL_State coax_ctrl_state;
/* 悬停推力运行时覆盖（自适应学习结果，只在 RAM），0 = 用 coax.hover_thrust_n / m·g。见 drv_hover_adapt.h。 */
static float coax_ctrl_hover_adapt_n = 0.0f;
static DRV_COAX_CTRL_Shaping coax_ctrl_shaping;
COAX_CTRL_AXI_NOINIT static DRV_AttRef coax_ctrl_att_ref;

#define DRV_COAX_CTRL_PARAM_ENTRY(field) \
    { "coax." #field, (uint16_t)offsetof(DRV_COAX_CTRL_Params, field) }

#define DRV_COAX_CTRL_NAMED_PARAM_ENTRY(name, field) \
    { "coax." name, (uint16_t)offsetof(DRV_COAX_CTRL_Params, field) }

static const DRV_COAX_CTRL_ParamEntry coax_ctrl_param_table[] = {
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_x_kp", position.pos_kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_y_kp", position.pos_kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_z_kp", position.pos_kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_xy_vel_max_m_s", position.xy_speed_limit_m_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_z_vel_up_max_m_s", position.z_speed_limit_up_m_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pos_z_vel_down_max_m_s", position.z_speed_limit_down_m_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_kp", position.vel_kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_kp", position.vel_kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_kp", position.vel_kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_ki", position.vel_ki[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_ki", position.vel_ki[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_ki", position.vel_ki[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_kd", position.vel_kd[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_kd", position.vel_kd[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_kd", position.vel_kd[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_x_i_limit_m_s2", position.vel_integrator_limit[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_y_i_limit_m_s2", position.vel_integrator_limit[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("vel_z_i_limit_m_s2", position.vel_integrator_limit[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_lpf_cutoff_hz", position.accel_lpf_cutoff_hz),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_xy_max_m_s2", position.xy_accel_limit_m_s2),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_z_up_max_m_s2", position.z_accel_limit_up_m_s2),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("accel_z_down_max_m_s2", position.z_accel_limit_down_m_s2),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("att_roll_kp", attitude.att_kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("att_pitch_kp", attitude.att_kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("att_yaw_kp", attitude.att_kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("roll_rate_limit_rad_s", attitude.rate_limit_rad_s[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("pitch_rate_limit_rad_s", attitude.rate_limit_rad_s[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("yaw_rate_limit_rad_s", attitude.rate_limit_rad_s[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_kp", rate.kp[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_kp", rate.kp[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_kp", rate.kp[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_ki", rate.ki[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_ki", rate.ki[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_ki", rate.ki[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_kd", rate.kd[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_kd", rate.kd[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_kd", rate.kd[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_i_limit_n_m", rate.integrator_limit[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_i_limit_n_m", rate.integrator_limit[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_i_limit_n_m", rate.integrator_limit[2]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("angular_accel_lpf_cutoff_rad_s", rate.alpha_lpf_cutoff_rad_s),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_roll_ff", rate.ff_gain[0]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_pitch_ff", rate.ff_gain[1]),
    DRV_COAX_CTRL_NAMED_PARAM_ENTRY("rate_yaw_ff", rate.ff_gain[2]),
    DRV_COAX_CTRL_PARAM_ENTRY(vel_loop_enable),
    DRV_COAX_CTRL_PARAM_ENTRY(tilt_limit_rad),
    DRV_COAX_CTRL_PARAM_ENTRY(rate_out_notch_hz),
    DRV_COAX_CTRL_PARAM_ENTRY(rate_out_notch_q),
    DRV_COAX_CTRL_PARAM_ENTRY(rate_out_notch2_hz),
    DRV_COAX_CTRL_PARAM_ENTRY(rate_out_notch2_q),
    DRV_COAX_CTRL_PARAM_ENTRY(att_ref_wr_rad_s),
    DRV_COAX_CTRL_PARAM_ENTRY(att_ref_delay_ms),
    DRV_COAX_CTRL_PARAM_ENTRY(hover_thrust_n),
    DRV_COAX_CTRL_PARAM_ENTRY(z_vel_fusion),
    DRV_COAX_CTRL_PARAM_ENTRY(alt_max_m),
    DRV_COAX_CTRL_PARAM_ENTRY(manual_tilt_max_rad),
    DRV_COAX_CTRL_PARAM_ENTRY(yaw_stick_rate_rad_s),
};

static const uint32_t coax_ctrl_param_count =
    sizeof(coax_ctrl_param_table) / sizeof(coax_ctrl_param_table[0]);

/*
 * 旧曲线（LEGACY，勿更新）：旧 ESP 台架由 tools/thrust_bench/emit.py 生成，无电压补偿。
 * 飞控不再默认使用它：启动时 app_thrust_lut.c 注入推力台查补表（drv_thrust_lut，当前版本见
 * data/identification/thrust/models/lut/current.json）。只在 THRUSTLUT MODE LEGACY 或未注入时生效。
 */
static const uint16_t coax_ctrl_dual_pwm_us[DRV_COAX_CTRL_THRUST_TABLE_POINTS] = {
    1100U, 1142U, 1184U, 1226U, 1268U, 1310U, 1352U, 1394U,
    1436U, 1478U, 1520U, 1562U, 1604U, 1646U, 1688U, 1730U,
    1772U, 1814U, 1856U, 1898U, 1940U,
};

static const float coax_ctrl_dual_thrust_g[DRV_COAX_CTRL_THRUST_TABLE_POINTS] = {
    0.000f, 5.069f, 25.589f, 60.655f, 106.361f, 165.084f,
    216.696f, 287.758f, 386.724f, 501.680f, 624.697f, 725.173f,
    828.680f, 923.574f, 981.674f, 1114.845f, 1256.137f, 1366.352f,
    1466.668f, 1541.404f, 1595.342f,
};

static float coax_ctrl_clamp_f32(float value, float lo, float hi)
{
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static uint16_t coax_ctrl_clamp_u16(int32_t value, uint16_t lo, uint16_t hi)
{
    if (value < (int32_t)lo) { return lo; }
    if (value > (int32_t)hi) { return hi; }
    return (uint16_t)value;
}

/*
 * R-F6-2 (2026-09-06): controller inputs use the canonical FLU local frame
 * (X forward, Y left, Z up).  This is R^T(roll, pitch, yaw), the standard
 * ZYX Euler rotation matrix transposed -- it needs no per-axis sign
 * adaptation because FLU's own roll/pitch/yaw are already defined as
 * right-hand rotations about FLU's own X/Y/Z (drv_frame_contract.h).
 */
static void coax_ctrl_local_down_to_body(const DRV_COAX_CTRL_AttitudeInput *attitude,
                                         const float local_down[3],
                                         float body[3])
{
    const float phi = attitude->roll_rad;
    const float theta = attitude->pitch_rad;
    const float psi = attitude->yaw_rad;
    const float cphi = cosf(phi);
    const float sphi = sinf(phi);
    const float ctheta = cosf(theta);
    const float stheta = sinf(theta);
    const float cpsi = cosf(psi);
    const float spsi = sinf(psi);

    body[0] = (ctheta * cpsi * local_down[0]) +
              (ctheta * spsi * local_down[1]) -
              (stheta * local_down[2]);
    body[1] = ((sphi * stheta * cpsi - cphi * spsi) * local_down[0]) +
              ((sphi * stheta * spsi + cphi * cpsi) * local_down[1]) +
              (sphi * ctheta * local_down[2]);
    body[2] = ((cphi * stheta * cpsi + sphi * spsi) * local_down[0]) +
              ((cphi * stheta * spsi - sphi * cpsi) * local_down[1]) +
              (cphi * ctheta * local_down[2]);
}

static float coax_ctrl_norm3(const float value[3])
{
    return sqrtf((value[0] * value[0]) +
                 (value[1] * value[1]) +
                 (value[2] * value[2]));
}

/*
 * 倾转力臂此刻能不能拿来换算。
 *
 * 机体模型无效时力臂不是"小"，是"没有意义"：转轴没填（存 0）时力臂成了
 * 重心 − 0 = −0.0946，符号是反的。照样反解的话，未解锁时舵机会朝反方向动，
 * 手扳机体核对舵机方向就会看起来"反了"，很容易误导人去翻 pulse_sign；力臂
 * 恰好为 0 时二分还会一路顶到负限位并报成功。所以这两种情形反解一律回中
 * （零倾角）、公开出口报失败，正向力矩记 0：宁可不动，不可动错。
 * 模型有效时解锁闸门已保证 |力臂| ≥ 下限，这道判断不改变任何数值。
 */
static uint8_t coax_ctrl_tilt_lever_usable(float lever_arm_m)
{
    return ((DRV_Airframe_IsValid() != 0U) && isfinite(lever_arm_m) &&
            (fabsf(lever_arm_m) >= DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M))
        ? 1U : 0U;
}

/*
 * 倾转 → 力矩的唯一正向模型（推导见文件头"倾转 → 机体力矩"）。反解器、
 * 力矩上限、实际达成力矩、调试分解与辨识出口全部经由这两个函数，
 * 不许在别处另写一份力臂乘法。力臂不可用时记 0（见上）。
 */
static float coax_ctrl_roll_moment_from_tilt(float total_force_n,
                                             float beta_rad)
{
    if (coax_ctrl_tilt_lever_usable(coax_ctrl_params.roll_tilt_lever_arm_m) == 0U) {
        return 0.0f;
    }
    /* 带符号几何力臂 = 重心 z − 1 号舵机转轴 z，方向与大小都来自实测。 */
    return coax_ctrl_params.roll_tilt_lever_arm_m *
           total_force_n *
           sinf(beta_rad);
}

static float coax_ctrl_pitch_moment_from_tilt(float total_force_n,
                                              float alpha_rad,
                                              float beta_rad)
{
    if (coax_ctrl_tilt_lever_usable(coax_ctrl_params.pitch_tilt_lever_arm_m) == 0U) {
        return 0.0f;
    }
    /* 带符号几何力臂 = 重心 z − 2 号舵机转轴 z。 */
    return coax_ctrl_params.pitch_tilt_lever_arm_m *
           total_force_n *
           sinf(alpha_rad) *
           cosf(beta_rad);
}

static float coax_ctrl_solve_roll_tilt_from_moment(float moment_n_m,
                                                   float total_force_n,
                                                   float tilt_limit_rad)
{
    float lo = -tilt_limit_rad;
    float hi = tilt_limit_rad;
    float moment_lo = coax_ctrl_roll_moment_from_tilt(total_force_n, lo);
    float moment_hi = coax_ctrl_roll_moment_from_tilt(total_force_n, hi);
    const float min_moment = fminf(moment_lo, moment_hi);
    const float max_moment = fmaxf(moment_lo, moment_hi);
    float target = moment_n_m;

    /* 力臂不可用：回中，不让二分收敛到限位（见 coax_ctrl_tilt_lever_usable）。 */
    if (coax_ctrl_tilt_lever_usable(coax_ctrl_params.roll_tilt_lever_arm_m) == 0U) {
        return 0.0f;
    }
    if (target < min_moment) {
        target = min_moment;
    } else if (target > max_moment) {
        target = max_moment;
    }

    for (uint32_t i = 0U; i < 18U; ++i) {
        const float mid = 0.5f * (lo + hi);
        const float moment_mid =
            coax_ctrl_roll_moment_from_tilt(total_force_n, mid);
        const uint8_t increasing = (moment_hi > moment_lo) ? 1U : 0U;

        if (((increasing != 0U) && (moment_mid < target)) ||
            ((increasing == 0U) && (moment_mid > target))) {
            lo = mid;
            moment_lo = moment_mid;
        } else {
            hi = mid;
            moment_hi = moment_mid;
        }
    }

    return 0.5f * (lo + hi);
}

static float coax_ctrl_solve_pitch_tilt_from_moment(float moment_n_m,
                                                    float total_force_n,
                                                    float beta_rad,
                                                    float tilt_limit_rad)
{
    float lo = -tilt_limit_rad;
    float hi = tilt_limit_rad;
    float moment_lo =
        coax_ctrl_pitch_moment_from_tilt(total_force_n, lo, beta_rad);
    float moment_hi =
        coax_ctrl_pitch_moment_from_tilt(total_force_n, hi, beta_rad);
    const float min_moment = fminf(moment_lo, moment_hi);
    const float max_moment = fmaxf(moment_lo, moment_hi);
    float target = moment_n_m;

    if (coax_ctrl_tilt_lever_usable(coax_ctrl_params.pitch_tilt_lever_arm_m) == 0U) {
        return 0.0f;
    }
    if (target < min_moment) {
        target = min_moment;
    } else if (target > max_moment) {
        target = max_moment;
    }

    for (uint32_t i = 0U; i < 18U; ++i) {
        const float mid = 0.5f * (lo + hi);
        const float moment_mid =
            coax_ctrl_pitch_moment_from_tilt(total_force_n, mid, beta_rad);
        const uint8_t increasing = (moment_hi > moment_lo) ? 1U : 0U;

        if (((increasing != 0U) && (moment_mid < target)) ||
            ((increasing == 0U) && (moment_mid > target))) {
            lo = mid;
            moment_lo = moment_mid;
        } else {
            hi = mid;
            moment_hi = moment_mid;
        }
    }

    return 0.5f * (lo + hi);
}

static void coax_ctrl_matrix_transpose(const float input[3][3],
                                       float output[3][3])
{
    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t col = 0U; col < 3U; ++col) {
            output[row][col] = input[col][row];
        }
    }
}

static void coax_ctrl_matrix_multiply(const float left[3][3],
                                      const float right[3][3],
                                      float output[3][3])
{
    float product[3][3];

    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t col = 0U; col < 3U; ++col) {
            product[row][col] = 0.0f;
            for (uint32_t index = 0U; index < 3U; ++index) {
                product[row][col] += left[row][index] * right[index][col];
            }
        }
    }
    memcpy(output, product, sizeof(product));
}

static void coax_ctrl_rpy_matrix(float roll_rad,
                                 float pitch_rad,
                                 float yaw_rad,
                                 float rotation[3][3])
{
    const float cr = cosf(roll_rad);
    const float sr = sinf(roll_rad);
    const float cp = cosf(pitch_rad);
    const float sp = sinf(pitch_rad);
    const float cy = cosf(yaw_rad);
    const float sy = sinf(yaw_rad);

    rotation[0][0] = cp * cy;
    rotation[0][1] = (sr * sp * cy) - (cr * sy);
    rotation[0][2] = (cr * sp * cy) + (sr * sy);
    rotation[1][0] = cp * sy;
    rotation[1][1] = (sr * sp * sy) + (cr * cy);
    rotation[1][2] = (cr * sp * sy) - (sr * cy);
    rotation[2][0] = -sp;
    rotation[2][1] = sr * cp;
    rotation[2][2] = cr * cp;
}

static void coax_ctrl_attitude_matrix(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    float rotation[3][3])
{
    coax_ctrl_rpy_matrix(attitude->roll_rad, attitude->pitch_rad,
                        attitude->yaw_rad, rotation);
}

static void coax_ctrl_attitude_error(const float desired[3][3],
                                     const float actual[3][3],
                                     float error[3],
                                     float *error_angle_rad,
                                     float *tilt_error_rad)
{
    float desired_t[3][3];
    float actual_t[3][3];
    float desired_t_actual[3][3];
    float actual_t_desired[3][3];

    coax_ctrl_matrix_transpose(desired, desired_t);
    coax_ctrl_matrix_transpose(actual, actual_t);
    coax_ctrl_matrix_multiply(desired_t, actual, desired_t_actual);
    coax_ctrl_matrix_multiply(actual_t, desired, actual_t_desired);

    error[0] = 0.5f * (desired_t_actual[2][1] - actual_t_desired[2][1]);
    error[1] = 0.5f * (desired_t_actual[0][2] - actual_t_desired[0][2]);
    error[2] = 0.5f * (desired_t_actual[1][0] - actual_t_desired[1][0]);
    if (error_angle_rad != NULL) {
        const float cos_angle = 0.5f *
            (desired_t_actual[0][0] + desired_t_actual[1][1] +
             desired_t_actual[2][2] - 1.0f);
        *error_angle_rad = acosf(coax_ctrl_clamp_f32(cos_angle, -1.0f, 1.0f));
    }
    if (tilt_error_rad != NULL) {
        const float z_axis_dot =
            (desired[0][2] * actual[0][2]) +
            (desired[1][2] * actual[1][2]) +
            (desired[2][2] * actual[2][2]);
        *tilt_error_rad = acosf(coax_ctrl_clamp_f32(z_axis_dot, -1.0f, 1.0f));
    }
}

static void coax_ctrl_rotation_to_rpy(const float rotation[3][3],
                                      float rpy_rad[3])
{
    const float sin_pitch = coax_ctrl_clamp_f32(-rotation[2][0], -1.0f, 1.0f);

    rpy_rad[0] = atan2f(rotation[2][1], rotation[2][2]);
    rpy_rad[1] = asinf(sin_pitch);
    rpy_rad[2] = atan2f(rotation[1][0], rotation[0][0]);
}

/*
 * 单桨"此刻真能出"的最大推力：推力映射在当前电量电压下把 ESC 满行程换成两桨合推力，取一半；
 * 写死的 motor_single_max_thrust_n（10.2 N）只作封顶。2026-10-01 推力表满油门两桨合计约
 * 16 N@12 V（单桨约 8 N），悬停已占 14.25 N——按 10.2 N 算，分配器以为还有 6 N 差速余量，
 * 偏航上限算成 0.031 N·m（实际约 0.01），推力早已顶满还在按"未饱和"分配。映射缺失或电压
 * 未知（结果不合理）时退回封顶值，即旧行为。
 */
static float coax_ctrl_single_max_thrust_n(void)
{
    const float cap = coax_ctrl_params.motor_single_max_thrust_n;
    const DRV_COAX_CTRL_ThrustMap *map = coax_ctrl_thrust_map;

    if ((map != NULL) && (map->total_thrust_for_pulse != NULL)) {
        const float half = 0.5f * map->total_thrust_for_pulse(BSP_PWM_ESC_MAX_US);
        if (isfinite(half) && (half >= DRV_COAX_CTRL_SINGLE_MAX_THRUST_MIN_N)) {
            return fminf(half, cap);
        }
    }
    return cap;
}

float DRV_COAX_CTRL_SingleMaxThrustN(void)
{
    DRV_COAX_CTRL_Init();
    return coax_ctrl_single_max_thrust_n();
}

/*
 * 差动推力能提供的偏航力矩上限，与 coax_ctrl_allocate_motor_thrust 的
 * 钳位边界严格对偶：任一路推力越界都会让指令被静默削掉。
 */
/* 合推力上限：机体参数的总推力上限与"给偏航留余量"的 2·T单max − YAW_RESERVE 取小；机体无效（≤0）原样返回。 */
static float coax_ctrl_force_cap_n(float max_total_force_n)
{
    const float yaw_cap_n = (2.0f * coax_ctrl_single_max_thrust_n()) - DRV_COAX_CTRL_YAW_RESERVE_N;

    if ((max_total_force_n > 0.0f) && (yaw_cap_n > DRV_COAX_CTRL_FORCE_EPS_N) &&
        (yaw_cap_n < max_total_force_n)) {
        return yaw_cap_n;
    }
    return max_total_force_n;
}

static float coax_ctrl_yaw_limit_moment(float total_force_n)
{
    const float ku = coax_ctrl_params.yaw_torque_upper_m_per_n;
    const float kl = coax_ctrl_params.yaw_torque_lower_m_per_n;
    const float span = (ku + kl) * coax_ctrl_single_max_thrust_n();
    float limit = fminf(kl * total_force_n, ku * total_force_n);

    limit = fminf(limit, span - (ku * total_force_n));
    limit = fminf(limit, span - (kl * total_force_n));
    return (limit > 0.0f) ? limit : 0.0f;
}

static float coax_ctrl_protection_scale(float value, float start, float end)
{
    if (value <= start) {
        return 1.0f;
    }
    if (value >= end) {
        return 0.0f;
    }
    return (end - value) / (end - start);
}

static float *coax_ctrl_param_ptr(DRV_COAX_CTRL_Params *params,
                                  const DRV_COAX_CTRL_ParamEntry *entry)
{
    return (float *)((uint8_t *)params + entry->offset);
}

static const DRV_COAX_CTRL_ParamEntry *coax_ctrl_find_param(const char *name)
{
    if (name == NULL) {
        return NULL;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (strcmp(name, coax_ctrl_param_table[i].name) == 0) {
            return &coax_ctrl_param_table[i];
        }
    }

    return NULL;
}

static uint8_t coax_ctrl_param_value_valid(const DRV_COAX_CTRL_ParamEntry *entry,
                                           float value)
{
    if ((entry == NULL) || !isfinite(value)) {
        return 0U;
    }

    if (fabsf(value) > 2000.0f) {
        return 0U;
    }

    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, tilt_limit_rad)) {
        return ((value > 0.0f) &&
                (value <= DRV_COAX_CTRL_TILT_LIMIT_RAD)) ? 1U : 0U;
    }

    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, vel_loop_enable)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, z_vel_fusion))) {
        return ((value >= 0.0f) && (value <= 1.0f)) ? 1U : 0U;
    }
    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, hover_thrust_n)) {
        return ((value == 0.0f) ||
                ((value >= DRV_COAX_CTRL_HOVER_THRUST_MIN_N) &&
                 (value <= DRV_COAX_CTRL_HOVER_THRUST_MAX_N))) ? 1U : 0U;
    }
    /* 飞行限幅（v29）：范围见 drv_coax_ctrl.h。 */
    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, alt_max_m)) {
        return ((value >= 0.1f) && (value <= 20.0f)) ? 1U : 0U;
    }
    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, manual_tilt_max_rad)) {
        return ((value >= 0.05f) && (value <= 0.785f)) ? 1U : 0U;
    }
    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, yaw_stick_rate_rad_s)) {
        return ((value >= 0.1f) && (value <= 6.0f)) ? 1U : 0U;
    }
    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params,
                                   attitude.rate_limit_rad_s[0])) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params,
                                   attitude.rate_limit_rad_s[1])) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params,
                                   attitude.rate_limit_rad_s[2]))) {
        return (value > 0.0f) ? 1U : 0U;
    }
    /* 整形参数：0 = 关；非 0 时的范围由各自模块定义（drv_moment_notch.h / drv_att_reference.h）。 */
    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, rate_out_notch_hz)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, rate_out_notch2_hz))) {
        return DRV_MomentNotch_FrequencyValid(value);
    }
    if ((entry->offset == offsetof(DRV_COAX_CTRL_Params, rate_out_notch_q)) ||
        (entry->offset == offsetof(DRV_COAX_CTRL_Params, rate_out_notch2_q))) {
        return DRV_MomentNotch_QValid(value);
    }
    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, att_ref_wr_rad_s)) {
        return DRV_AttRef_BandwidthValid(value);
    }
    if (entry->offset == offsetof(DRV_COAX_CTRL_Params, att_ref_delay_ms)) {
        return DRV_AttRef_DelayValid(value);
    }
    return (value >= 0.0f) ? 1U : 0U;
}

static uint8_t coax_ctrl_params_valid(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return 0U;
    }

    for (uint32_t i = 0U; i < coax_ctrl_param_count; ++i) {
        if (coax_ctrl_param_value_valid(&coax_ctrl_param_table[i],
                                        *coax_ctrl_param_ptr((DRV_COAX_CTRL_Params *)params,
                                                             &coax_ctrl_param_table[i])) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

/*
 * 把机体模型里的物理量复制进 params。它们**不是可调参数**：改它们要拿秤和尺
 * 重新量，然后从上位机机体模型页写进 Flash，而不是在调参页拖滑块。
 *
 * 每次 SetParams/GetDefaultParams/GetParams/Run/反解出口都重刷一遍，代价是
 * 几个 float 赋值加两次减法。这样机体模型一改（`PARAM SET airframe.*` 当场
 * 重算重心）下一拍就生效，不存在"改了模型但控制律还在用旧质量/旧力臂"
 * 这种只能靠重启才发现的中间态。
 */
static void coax_ctrl_apply_fixed_model_params(DRV_COAX_CTRL_Params *params)
{
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();

    if (params == NULL) {
        return;
    }

    params->mass_kg = airframe->mass_kg;
    params->gravity_m_s2 = airframe->gravity_m_s2;
    /*
     * 倾转力臂（2026-09-27 起）：**带符号的几何力臂** = 重心 z − 舵机转轴 z
     * = −r_z，转轴在重心下方时为正。力矩 = 力臂 × T × sin(倾角)，极性已含在
     * 符号里（推导见文件头"倾转 → 机体力矩"）。字段名沿用是因为飞行日志 v6~v9
     * 的参数快照按这两个名字存档；那些旧日志里存的是退役的 0.145 m 输入值，
     * 不是这个几何量，解读旧日志时别混用。
     */
    params->roll_tilt_lever_arm_m = -DRV_Airframe_RollTiltAxisToCgZ(airframe);
    params->pitch_tilt_lever_arm_m = -DRV_Airframe_PitchTiltAxisToCgZ(airframe);
    params->yaw_inertia = airframe->izz_kgm2;
    params->motor_single_max_thrust_n = DRV_COAX_CTRL_SINGLE_MAX_THRUST_N;
    params->yaw_torque_upper_m_per_n = DRV_COAX_CTRL_PROP9047_YAW_M_PER_N;
    params->yaw_torque_lower_m_per_n = DRV_COAX_CTRL_PROP9047_YAW_M_PER_N;
}

static void coax_ctrl_compute_accel_cmd(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_Schedule *schedule,
    float horizontal_scale,
    DRV_COAX_CTRL_Debug *debug)
{
    DRV_POSITION_CONTROL_PositionInput position_input;
    DRV_POSITION_CONTROL_VelocityInput velocity_input;
    const float position_sp[3] = { reference->x_m, reference->y_m, reference->z_m };
    const float position_meas[3] = { attitude->x_m, attitude->y_m, attitude->z_m };
    const float velocity_ff[3] = { reference->vx_m_s, reference->vy_m_s,
                                   reference->vz_m_s };
    const float velocity_meas[3] = { attitude->vx_m_s, attitude->vy_m_s,
                                     attitude->vz_m_s };
    const float accel_ff[3] = { reference->ax_m_s2, reference->ay_m_s2,
                                reference->az_m_s2 };
    const uint8_t translation_bypass =
        ((reference->manual_total_force_valid != 0U) ||
         (reference->direct_attitude_target_valid != 0U) ||
         (coax_ctrl_params.vel_loop_enable < 0.5f)) ? 1U : 0U;

    memset(&position_input, 0, sizeof(position_input));
    memset(&velocity_input, 0, sizeof(velocity_input));

    if ((translation_bypass == 0U) && (schedule->position_update != 0U)) {
        position_input.position_sp_m[0] = reference->x_m;
        position_input.position_sp_m[1] = reference->y_m;
        position_input.position_sp_m[2] = reference->z_m;
        position_input.position_meas_m[0] = attitude->x_m;
        position_input.position_meas_m[1] = attitude->y_m;
        position_input.position_meas_m[2] = attitude->z_m;
        if (reference->split_axis_validity != 0U) {
            /* 分轴：无效轴把测量换成目标（位置误差 0，只剩速度前馈），见 drv_coax_ctrl.h。 */
            if (reference->navigation_position_valid == 0U) {
                position_input.position_meas_m[0] = reference->x_m;
                position_input.position_meas_m[1] = reference->y_m;
            }
            if (reference->vertical_measurement_valid == 0U) {
                position_input.position_meas_m[2] = reference->z_m;
            }
        }
        position_input.direct_velocity_m_s[0] = reference->vx_m_s;
        position_input.direct_velocity_m_s[1] = reference->vy_m_s;
        position_input.direct_velocity_m_s[2] = reference->vz_m_s;
        memcpy(position_input.velocity_ff_m_s, velocity_ff,
               sizeof(position_input.velocity_ff_m_s));
        position_input.dt_sec = schedule->position_dt_s;
        position_input.measurement_valid =
            (reference->split_axis_validity != 0U) ?
                (uint8_t)((reference->navigation_position_valid != 0U) ||
                          (reference->vertical_measurement_valid != 0U)) :
                reference->navigation_position_valid;
        position_input.position_bypass = reference->position_control_bypass;
        DRV_POSITION_CONTROL_PositionStep(&coax_ctrl_params.position,
                                          &position_input,
                                          &coax_ctrl_state.position_output);
    }

    if ((translation_bypass == 0U) && (schedule->velocity_update != 0U)) {
        memcpy(velocity_input.velocity_sp_m_s,
               coax_ctrl_state.position_output.velocity_sp_m_s,
               sizeof(velocity_input.velocity_sp_m_s));
        velocity_input.velocity_meas_m_s[0] = attitude->vx_m_s;
        velocity_input.velocity_meas_m_s[1] = attitude->vy_m_s;
        velocity_input.velocity_meas_m_s[2] = attitude->vz_m_s;
        velocity_input.accel_ff_m_s2[0] = reference->ax_m_s2;
        velocity_input.accel_ff_m_s2[1] = reference->ay_m_s2;
        velocity_input.accel_ff_m_s2[2] = reference->az_m_s2;
        memcpy(velocity_input.measured_accel_m_s2,
               attitude->accel_m_s2,
               sizeof(velocity_input.measured_accel_m_s2));
        velocity_input.dt_sec = schedule->velocity_dt_s;
        velocity_input.measurement_valid =
            (reference->navigation_velocity_valid != 0U) &&
            (attitude->acceleration_valid != 0U);
        if (reference->split_axis_validity != 0U) {
            /* 分轴：任一通道有效就闭环；无效轴速度测量换成目标（误差 0，积分保持配平）。
             * 加速度测量只喂 D 项低通，无效时给 0。 */
            velocity_input.measurement_valid =
                (uint8_t)((reference->navigation_velocity_valid != 0U) ||
                          (reference->vertical_measurement_valid != 0U));
            if (reference->navigation_velocity_valid == 0U) {
                velocity_input.velocity_meas_m_s[0] = velocity_input.velocity_sp_m_s[0];
                velocity_input.velocity_meas_m_s[1] = velocity_input.velocity_sp_m_s[1];
            }
            if (reference->vertical_measurement_valid == 0U) {
                velocity_input.velocity_meas_m_s[2] = velocity_input.velocity_sp_m_s[2];
            }
            if (attitude->acceleration_valid == 0U) {
                memset(velocity_input.measured_accel_m_s2, 0,
                       sizeof(velocity_input.measured_accel_m_s2));
            }
        }
        velocity_input.integrator_enable = schedule->integrator_enable;
        velocity_input.integrator_freeze = schedule->integrator_freeze;
        velocity_input.integrator_reset = schedule->integrator_reset;
        velocity_input.downstream_saturation =
            coax_ctrl_state.translation_saturation;
        DRV_POSITION_CONTROL_VelocityStep(&coax_ctrl_params.position,
                                          &coax_ctrl_state.position,
                                          &velocity_input,
                                          &coax_ctrl_state.velocity_output);
    }

    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        debug->position_sp_m[axis] = position_sp[axis];
        debug->position_m[axis] = position_meas[axis];
        debug->position_error_m[axis] =
            debug->position_sp_m[axis] - debug->position_m[axis];
        debug->velocity_ff_m_s[axis] = velocity_ff[axis];
        debug->velocity_sp_m_s[axis] =
            coax_ctrl_state.position_output.velocity_sp_m_s[axis];
        debug->velocity_m_s[axis] = velocity_meas[axis];
        debug->velocity_error_m_s[axis] =
            coax_ctrl_state.velocity_output.error_m_s[axis];
        debug->velocity_p_m_s2[axis] =
            coax_ctrl_state.velocity_output.p_term_m_s2[axis];
        debug->velocity_i_m_s2[axis] =
            coax_ctrl_state.velocity_output.i_term_m_s2[axis];
        debug->velocity_d_m_s2[axis] =
            coax_ctrl_state.velocity_output.d_term_m_s2[axis];
        debug->velocity_ff_m_s2[axis] =
            coax_ctrl_state.velocity_output.ff_term_m_s2[axis];
        debug->accel_unsat_m_s2[axis] =
            coax_ctrl_state.velocity_output.accel_unsat_m_s2[axis];
        debug->accel_out_m_s2[axis] =
            coax_ctrl_state.velocity_output.accel_sat_m_s2[axis];
        debug->pos_p_m_s2[axis] =
            coax_ctrl_params.position.vel_kp[axis] *
            coax_ctrl_params.position.pos_kp[axis] *
            debug->position_error_m[axis];
        debug->vel_d_m_s2[axis] = debug->velocity_p_m_s2[axis];
    }

    debug->pos_z_i_m_s2 = debug->velocity_i_m_s2[2];
    if (translation_bypass != 0U) {
        memset(&coax_ctrl_state.position_output, 0,
               sizeof(coax_ctrl_state.position_output));
        memset(&coax_ctrl_state.velocity_output, 0,
               sizeof(coax_ctrl_state.velocity_output));
        for (uint32_t axis = 0U; axis < 3U; ++axis) {
            debug->pos_p_m_s2[axis] = 0.0f;
            debug->vel_d_m_s2[axis] = 0.0f;
            debug->velocity_error_m_s[axis] = 0.0f;
            debug->velocity_p_m_s2[axis] = 0.0f;
            debug->velocity_i_m_s2[axis] = 0.0f;
            debug->velocity_d_m_s2[axis] = 0.0f;
            debug->velocity_ff_m_s2[axis] = accel_ff[axis];
            debug->accel_out_m_s2[axis] = accel_ff[axis];
            debug->accel_unsat_m_s2[axis] = debug->accel_out_m_s2[axis];
        }
        if ((reference->manual_total_force_valid != 0U) ||
            (reference->direct_attitude_target_valid != 0U)) {
            debug->accel_out_m_s2[0] = 0.0f;
            debug->accel_out_m_s2[1] = 0.0f;
            debug->accel_unsat_m_s2[0] = 0.0f;
            debug->accel_unsat_m_s2[1] = 0.0f;
        }
    }

    if (horizontal_scale < 0.999f) {
        debug->accel_out_m_s2[0] *= horizontal_scale;
        debug->accel_out_m_s2[1] *= horizontal_scale;
    }
}

static void coax_ctrl_reset_shaping(void)
{
    memset(&coax_ctrl_shaping, 0, sizeof(coax_ctrl_shaping));
    DRV_AttRef_Reset(&coax_ctrl_att_ref);
}

/*
 * 姿态指令整形（参考模型，横滚/俯仰）。关闭（att_ref_wr_rad_s = 0）时目标角原样
 * 返回、两个前馈为 +0，与加入它之前逐位相同。开启时（drv_att_reference.h）：
 *   roll/pitch   ← θ_ref(t − Td)：角度环反馈的目标
 *   rate_ff      ← θ̇_ref(t − Td)：角度环前馈（desired_rate_in_desired_frame）
 *   accel_ff     ← θ̈_ref(t)：速率环 α_ff，力矩前馈 = ff_gain × I × α_ff
 * 欧拉角速率/加速度直接当机体量用（小角度），与偏航前馈 yaw_rate_rad_s 的现有口径一致。
 * 直接姿态目标进来前已按 tilt_limit_rad 夹过，临界阻尼参考不超调，延后参考越不过它；
 * 目标突变（遥控/位置环）时参考模型本身就是限速器。
 * 只算不提交：保护缩放同拍重算时再算一遍，拍末由 coax_ctrl_commit_shaping 提交。
 */
static void coax_ctrl_shape_attitude_target(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_Schedule *schedule,
    float *roll_rad,
    float *pitch_rad,
    float rate_ff[DRV_ATT_REF_AXES],
    float accel_ff[DRV_ATT_REF_AXES])
{
    const uint8_t source = (reference->direct_attitude_target_valid != 0U) ? 2U : 1U;
    const float command[DRV_ATT_REF_AXES] = { *roll_rad, *pitch_rad };
    float align[DRV_ATT_REF_AXES] = { attitude->roll_rad, attitude->pitch_rad };
    DRV_AttRefOutput shaped;

    for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
        rate_ff[axis] = 0.0f;
        accel_ff[axis] = 0.0f;
        if (!isfinite(align[axis])) {
            align[axis] = command[axis];
        }
    }
    if (!(coax_ctrl_params.att_ref_wr_rad_s > 0.0f)) {
        return;
    }
    if (source != 2U) {
        /*
         * 位置/速度环给的姿态目标不过参考模型（同 PX4：位置控制器的姿态设定值不再整形）。
         * 参考模型为遥控阶跃设计，放进速度环里等于多一节约 0.22 s 的滞后：2026-10-01 自由飞
         * 定点 2.4 s 持续振荡（日志 receive_fgdfdpto，加速度指令→期望姿态互相关峰 210–220 ms），
         * 拟合对象仿真去掉它后现增益即充分阻尼（data/analysis/flight-position-loop/2026-10-01/）。
         * 记下来源，切回直接姿态时照常从实测角重新起步。
         */
        coax_ctrl_shaping.ref_source = source;
        return;
    }
    if (coax_ctrl_shaping.ref_source != source) {
        /* 目标来源切换（直接姿态 ↔ 位置/速度环）：从当前姿态重新起步，不带旧速度。 */
        DRV_AttRef_Reset(&coax_ctrl_att_ref);
        coax_ctrl_shaping.ref_source = source;
    }
    DRV_AttRef_Evaluate(&coax_ctrl_att_ref,
                        coax_ctrl_params.att_ref_wr_rad_s,
                        coax_ctrl_params.att_ref_delay_ms * 1.0e-3f,
                        command,
                        align,
                        (schedule->attitude_update != 0U) ? schedule->attitude_dt_s : 0.0f,
                        &coax_ctrl_shaping.ref_pending,
                        &shaped);
    coax_ctrl_shaping.ref_pending_valid = 1U;
    *roll_rad = shaped.angle[0];
    *pitch_rad = shaped.angle[1];
    for (uint32_t axis = 0U; axis < DRV_ATT_REF_AXES; ++axis) {
        rate_ff[axis] = shaped.rate[axis];
        accel_ff[axis] = shaped.accel[axis];
    }
}

/*
 * 速率环力矩出口陷波（横滚/俯仰，drv_moment_notch.h）：只滤反馈 P + I − D，两级串联
 * （第二级 notch2 在第一级之后），前馈原样叠回，再按同一组力矩限重钳位。系数只在 Step 拍
 * 按本拍真实 dt 刷新；两级都关着时不碰输出，第二级关着时与只有第一级逐位相同。
 */
static void coax_ctrl_apply_moment_notch(const DRV_RateControl_Input *rate_input,
                                         const DRV_COAX_CTRL_Schedule *schedule)
{
    if (!(coax_ctrl_params.rate_out_notch_hz > 0.0f) &&
        (coax_ctrl_shaping.notch.active == 0U) &&
        !(coax_ctrl_params.rate_out_notch2_hz > 0.0f) &&
        (coax_ctrl_shaping.notch2.active == 0U)) {
        return;
    }
    if (schedule->rate_update != 0U) {
        (void)DRV_MomentNotch_Configure(&coax_ctrl_shaping.notch,
                                        coax_ctrl_params.rate_out_notch_hz,
                                        coax_ctrl_params.rate_out_notch_q,
                                        schedule->rate_dt_s);
        (void)DRV_MomentNotch_Configure(&coax_ctrl_shaping.notch2,
                                        coax_ctrl_params.rate_out_notch2_hz,
                                        coax_ctrl_params.rate_out_notch2_q,
                                        schedule->rate_dt_s);
    }
    DRV_MomentNotch_ApplyCascadeToRateOutput(&coax_ctrl_shaping.notch,
                                             &coax_ctrl_shaping.notch2, rate_input,
                                             &coax_ctrl_state.rate_output);
}

/* 拍末提交：参考模型只在姿态拍推进，陷波只在速率 Step 拍推进。 */
static void coax_ctrl_commit_shaping(const DRV_COAX_CTRL_Schedule *schedule)
{
    if ((coax_ctrl_shaping.ref_pending_valid != 0U) &&
        (schedule->attitude_update != 0U)) {
        DRV_AttRef_Commit(&coax_ctrl_att_ref, &coax_ctrl_shaping.ref_pending);
    }
    coax_ctrl_shaping.ref_pending_valid = 0U;
    if (schedule->rate_update != 0U) {
        DRV_MomentNotch_Commit(&coax_ctrl_shaping.notch);
        DRV_MomentNotch_Commit(&coax_ctrl_shaping.notch2);
    } else {
        DRV_MomentNotch_Discard(&coax_ctrl_shaping.notch);
        DRV_MomentNotch_Discard(&coax_ctrl_shaping.notch2);
    }
}

static void coax_ctrl_compute_balance_solution(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_Schedule *schedule,
    DRV_COAX_CTRL_Debug *debug,
    DRV_COAX_CTRL_BalanceSolution *solution)
{
    float actual_r[3][3];
    DRV_AttitudeControl_Input attitude_input;
    DRV_RateControl_Input rate_input;
    float error_for_angle[3];
    const float actual_omega[3] = {
        attitude->gyro_x_rad_s,
        attitude->gyro_y_rad_s,
        attitude->gyro_z_rad_s,
    };
    float force_scale = 1.0f;
    /*
     * 合推力上限来自机体模型（max_total_thrust_g × g）。模型没写过时它是 0：
     * 既不能当分母，也不能当"无上限"。下面每处都显式判了 > 0——不是防御性
     * 代码洁癖，是因为 0/0 会算出 NaN，而 NaN 会沿着推力分配一路扩散到舵机
     * 指令，最后表现成某个完全无关的地方"突然发散"，根本追不回这一行。
     * 真正的闸门在解锁那一层（DRV_Airframe_IsValid），这里只保证未解锁时的
     * 计算是干净的 0。
     */
    const float max_total_force_n = DRV_Airframe_Get()->max_total_force_n;
    float target_pitch_rad;
    float target_roll_rad;
    float command_rp_rad[2];
    float ref_rate_ff[DRV_ATT_REF_AXES];
    float ref_accel_ff[DRV_ATT_REF_AXES];
    float roll_limit_moment_n_m;
    float pitch_limit_moment_n_m;
    float yaw_limit_moment_n_m;
    float roll_utilization;
    float pitch_utilization;
    float yaw_utilization;
    const float mass_eff_kg = DRV_COAX_CTRL_EffectiveMassKg(coax_ctrl_params.mass_kg);

    memset(solution, 0, sizeof(*solution));
    solution->desired_force_local_n[0] =
        mass_eff_kg * debug->accel_out_m_s2[0];
    solution->desired_force_local_n[1] =
        mass_eff_kg * debug->accel_out_m_s2[1];
    if (reference->manual_total_force_valid != 0U) {
        solution->desired_force_local_n[2] =
            coax_ctrl_clamp_f32(reference->manual_total_force_n,
                                DRV_COAX_CTRL_FORCE_EPS_N,
                                max_total_force_n);
    } else {
        /*
         * R-F6-2: Z is now up-positive, so Newton's second law along Z gives
         * F_thrust = m*(g + a_up) -- more commanded upward acceleration
         * means more thrust, not less.  (Legacy down-positive Z used
         * F_thrust = m*(g - a_down); this is the same physics, opposite
         * sign convention.)
         */
        solution->desired_force_local_n[2] =
            mass_eff_kg *
            (coax_ctrl_params.gravity_m_s2 + debug->accel_out_m_s2[2]);
        if (solution->desired_force_local_n[2] < DRV_COAX_CTRL_FORCE_EPS_N) {
            solution->desired_force_local_n[2] = DRV_COAX_CTRL_FORCE_EPS_N;
        }
    }

    solution->raw_total_force_n =
        coax_ctrl_norm3(solution->desired_force_local_n);
    solution->thrust_utilization = (max_total_force_n > 0.0f)
        ? (solution->raw_total_force_n / max_total_force_n)
        : 0.0f;
    if ((max_total_force_n > 0.0f) &&
        (solution->raw_total_force_n > coax_ctrl_force_cap_n(max_total_force_n))) {
        force_scale = coax_ctrl_force_cap_n(max_total_force_n) /
                      solution->raw_total_force_n;
        for (uint32_t axis = 0U; axis < 3U; ++axis) {
            solution->desired_force_local_n[axis] *= force_scale;
        }
    }
    solution->total_force_n = coax_ctrl_norm3(solution->desired_force_local_n);

    /*
     * Horizontal outer loop owns acceleration only. The requested force vector
     * is converted once into a target body attitude; servos are not commanded
     * directly from velocity or position terms.
     */
    if (reference->direct_attitude_target_valid != 0U) {
        target_roll_rad =
            coax_ctrl_clamp_f32(reference->target_roll_rad,
                                -coax_ctrl_params.tilt_limit_rad,
                                 coax_ctrl_params.tilt_limit_rad);
        target_pitch_rad =
            coax_ctrl_clamp_f32(reference->target_pitch_rad,
                                -coax_ctrl_params.tilt_limit_rad,
                                 coax_ctrl_params.tilt_limit_rad);
    } else {
        DRV_COAX_CTRL_TiltFromForce(solution->desired_force_local_n,
                                    &target_roll_rad, &target_pitch_rad);
    }
    /* 指令留作调试；参考模型开启时角度环跟的是整形后的延后参考。 */
    command_rp_rad[0] = target_roll_rad;
    command_rp_rad[1] = target_pitch_rad;
    coax_ctrl_shape_attitude_target(attitude, reference, schedule,
                                    &target_roll_rad, &target_pitch_rad,
                                    ref_rate_ff, ref_accel_ff);
    coax_ctrl_attitude_matrix(attitude, actual_r);
    coax_ctrl_rpy_matrix(target_roll_rad,
                         target_pitch_rad,
                         reference->yaw_rad,
                         solution->desired_body_r);
    coax_ctrl_attitude_error(solution->desired_body_r,
                             actual_r,
                             error_for_angle,
                             &solution->attitude_error_angle_rad,
                             &solution->attitude_tilt_error_rad);
    memset(&attitude_input, 0, sizeof(attitude_input));
    memcpy(attitude_input.actual_rotation, actual_r, sizeof(actual_r));
    memcpy(attitude_input.desired_rotation, solution->desired_body_r,
           sizeof(solution->desired_body_r));
    attitude_input.desired_rate_in_desired_frame[0] = ref_rate_ff[0];
    attitude_input.desired_rate_in_desired_frame[1] = ref_rate_ff[1];
    attitude_input.desired_rate_in_desired_frame[2] =
        reference->yaw_rate_rad_s;
    if (schedule->attitude_update != 0U) {
        (void)DRV_AttitudeControl_Step(&coax_ctrl_params.attitude,
                                       &attitude_input,
                                       &coax_ctrl_state.attitude_output);
    }

    roll_limit_moment_n_m = fabsf(coax_ctrl_roll_moment_from_tilt(
        solution->total_force_n, coax_ctrl_params.tilt_limit_rad));
    pitch_limit_moment_n_m = fabsf(coax_ctrl_pitch_moment_from_tilt(
        solution->total_force_n, coax_ctrl_params.tilt_limit_rad, 0.0f));
    yaw_limit_moment_n_m = coax_ctrl_yaw_limit_moment(solution->total_force_n);
    roll_limit_moment_n_m = fmaxf(roll_limit_moment_n_m,
                                  DRV_COAX_CTRL_RATE_SCALE_EPS);
    pitch_limit_moment_n_m = fmaxf(pitch_limit_moment_n_m,
                                   DRV_COAX_CTRL_RATE_SCALE_EPS);
    yaw_limit_moment_n_m = fmaxf(yaw_limit_moment_n_m,
                                 DRV_COAX_CTRL_RATE_SCALE_EPS);

    memset(&rate_input, 0, sizeof(rate_input));
    memcpy(rate_input.omega, actual_omega, sizeof(actual_omega));
    memcpy(rate_input.omega_sp, coax_ctrl_state.attitude_output.omega_sp,
           sizeof(rate_input.omega_sp));
    rate_input.alpha_ff[0] = ref_accel_ff[0];
    rate_input.alpha_ff[1] = ref_accel_ff[1];
    rate_input.alpha_ff[2] = reference->yaw_accel_rad_s2;
    rate_input.inertia[0] = DRV_Airframe_Get()->ixx_kgm2;
    rate_input.inertia[1] = DRV_Airframe_Get()->iyy_kgm2;
    rate_input.inertia[2] = DRV_Airframe_Get()->izz_kgm2;
    rate_input.dt_s = schedule->rate_dt_s;
    rate_input.saturation_positive[0] = roll_limit_moment_n_m;
    rate_input.saturation_positive[1] = pitch_limit_moment_n_m;
    rate_input.saturation_positive[2] = yaw_limit_moment_n_m;
    rate_input.saturation_negative[0] = -roll_limit_moment_n_m;
    rate_input.saturation_negative[1] = -pitch_limit_moment_n_m;
    rate_input.saturation_negative[2] = -yaw_limit_moment_n_m;
    rate_input.measurement_valid = 1U;
    rate_input.integrator_enable = schedule->integrator_enable;
    rate_input.integrator_freeze = schedule->integrator_freeze;
    rate_input.integrator_reset = schedule->integrator_reset;
    memcpy(rate_input.saturation_positive_active,
           coax_ctrl_state.moment_saturation_positive,
           sizeof(rate_input.saturation_positive_active));
    memcpy(rate_input.saturation_negative_active,
           coax_ctrl_state.moment_saturation_negative,
           sizeof(rate_input.saturation_negative_active));
    if (schedule->rate_update != 0U) {
        (void)DRV_RateControl_Step(&coax_ctrl_params.rate,
                                   &coax_ctrl_state.rate,
                                   &rate_input,
                                   &coax_ctrl_state.rate_output);
    } else {
        (void)DRV_RateControl_Evaluate(&coax_ctrl_params.rate,
                                       &coax_ctrl_state.rate,
                                       &rate_input,
                                       &coax_ctrl_state.rate_output);
    }
    coax_ctrl_apply_moment_notch(&rate_input, schedule);

    memcpy(solution->attitude_error,
           coax_ctrl_state.attitude_output.attitude_error,
           sizeof(solution->attitude_error));
    memcpy(solution->rate_error_rad_s,
           coax_ctrl_state.rate_output.error,
           sizeof(solution->rate_error_rad_s));
    memcpy(solution->moment_cmd_n_m,
           coax_ctrl_state.rate_output.moment_unsat,
           sizeof(solution->moment_cmd_n_m));
    /*
     * 偏航必须用**钳过**的那份去分配。
     *
     * roll/pitch 走 coax_ctrl_solve_*_tilt_from_moment，那两个求解器内部会把
     * 目标力矩夹进可达区间，所以喂未钳值无害；偏航是直接进差动推力分配器的，
     * 中间没有任何一道钳位。而分配器的两路推力是**各自独立**钳到 [0, T_max]
     * 的，一路撞上限时另一路不会补——总推力就这么悄悄少了。
     *
     * 现象是"一加偏航增益，桨憋死、升力维持不住"：上下桨共用同一个推力预算，
     * 偏航力矩靠拉开两者的差获得，差拉过头时弱的那个趋近 0、强的那个撞满，
     * 和 = F 的约束首先被牺牲掉。角速度环已经算出了这个上限并按它做抗饱和
     * （rate_input.saturation_*[2]），只是结果一直没被分配器采用。
     */
    solution->yaw_moment_applied_n_m =
        coax_ctrl_state.rate_output.moment_cmd[2];
    debug->yaw_angle_p_rad_s =
        -coax_ctrl_params.attitude.att_kp[2] * solution->attitude_error[2];
    debug->yaw_rate_d_rad_s = solution->rate_error_rad_s[2];

    solution->beta_rad = coax_ctrl_solve_roll_tilt_from_moment(
        solution->moment_cmd_n_m[0],
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad);

    solution->alpha_rad = coax_ctrl_solve_pitch_tilt_from_moment(
        solution->moment_cmd_n_m[1],
        solution->total_force_n,
        solution->beta_rad,
        coax_ctrl_params.tilt_limit_rad);

    roll_limit_moment_n_m = fabsf(coax_ctrl_roll_moment_from_tilt(
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad));
    pitch_limit_moment_n_m = fabsf(coax_ctrl_pitch_moment_from_tilt(
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad,
        solution->beta_rad));
    if (roll_limit_moment_n_m < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        roll_limit_moment_n_m = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }
    if (pitch_limit_moment_n_m < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        pitch_limit_moment_n_m = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }
    /*
     * 偏航的可行力矩上限由差动推力分配决定，不是倾转限位。分配式
     *   upper = (kl*F - Mz)/(ku+kl)，lower = (ku*F + Mz)/(ku+kl)
     * 要求两路都落在 [0, T_max]，解出四个边界，取最紧的一个：
     *   |Mz| <= min(kl*F, ku*F, (ku+kl)*T_max - ku*F, (ku+kl)*T_max - kl*F)
     * 悬停 F=13.4N、T_max=10.2N、ku=kl=1e-4 时约束来自上桨推力上限，
     * 上限只有 7e-4 N*m —— 偏航权限很紧，所以它必须参与保护缩放，
     * 否则分配环节的 clamp 会静默削掉指令而保护层毫无感知。
     */
    yaw_limit_moment_n_m = coax_ctrl_yaw_limit_moment(solution->total_force_n);
    if (yaw_limit_moment_n_m < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        yaw_limit_moment_n_m = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }
    roll_utilization =
        fabsf(solution->moment_cmd_n_m[0]) / roll_limit_moment_n_m;
    pitch_utilization =
        fabsf(solution->moment_cmd_n_m[1]) / pitch_limit_moment_n_m;
    yaw_utilization =
        fabsf(solution->moment_cmd_n_m[2]) / yaw_limit_moment_n_m;
    solution->moment_utilization = fmaxf(fmaxf(roll_utilization,
                                               pitch_utilization),
                                         yaw_utilization);

    coax_ctrl_local_down_to_body(attitude,
                                 solution->desired_force_local_n,
                                 solution->desired_force_body_n);
    debug->force_cmd_n[0] = solution->desired_force_body_n[0];
    debug->force_cmd_n[1] = solution->desired_force_body_n[1];
    debug->force_cmd_n[2] = solution->desired_force_body_n[2];

    debug->target_attitude_rp_rad[0] = command_rp_rad[0];
    debug->target_attitude_rp_rad[1] = command_rp_rad[1];

    debug->tilt_angle_p_rad[0] = coax_ctrl_solve_pitch_tilt_from_moment(
        -coax_ctrl_params.rate.kp[1] *
         coax_ctrl_params.attitude.att_kp[1] * solution->attitude_error[1],
        solution->total_force_n,
        solution->beta_rad,
        coax_ctrl_params.tilt_limit_rad);
    debug->tilt_angle_p_rad[1] = coax_ctrl_solve_roll_tilt_from_moment(
        -coax_ctrl_params.rate.kp[0] *
         coax_ctrl_params.attitude.att_kp[0] * solution->attitude_error[0],
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad);
    debug->tilt_rate_d_rad[0] = coax_ctrl_solve_pitch_tilt_from_moment(
        coax_ctrl_state.rate_output.p_term[1] +
        (coax_ctrl_params.rate.kp[1] *
         coax_ctrl_params.attitude.att_kp[1] * solution->attitude_error[1]),
        solution->total_force_n,
        solution->beta_rad,
        coax_ctrl_params.tilt_limit_rad);
    debug->tilt_rate_d_rad[1] = coax_ctrl_solve_roll_tilt_from_moment(
        coax_ctrl_state.rate_output.p_term[0] +
        (coax_ctrl_params.rate.kp[0] *
         coax_ctrl_params.attitude.att_kp[0] * solution->attitude_error[0]),
        solution->total_force_n,
        coax_ctrl_params.tilt_limit_rad);

    debug->tilt_out_rad[0] = solution->alpha_rad;
    debug->tilt_out_rad[1] = solution->beta_rad;
    coax_ctrl_rotation_to_rpy(solution->desired_body_r,
                              debug->desired_attitude_rpy_rad);
    memcpy(debug->attitude_error,
           solution->attitude_error,
           sizeof(debug->attitude_error));
    memcpy(debug->rate_error_rad_s,
           solution->rate_error_rad_s,
           sizeof(debug->rate_error_rad_s));
    memcpy(debug->moment_cmd_n_m,
           solution->moment_cmd_n_m,
           sizeof(debug->moment_cmd_n_m));
    memcpy(debug->omega_ff_rad_s,
           coax_ctrl_state.attitude_output.omega_ff,
           sizeof(debug->omega_ff_rad_s));
    memcpy(debug->omega_sp_rad_s,
           coax_ctrl_state.attitude_output.omega_sp,
           sizeof(debug->omega_sp_rad_s));
    memcpy(debug->omega_rad_s, actual_omega, sizeof(debug->omega_rad_s));
    memcpy(debug->rate_limit_rad_s,
           coax_ctrl_params.attitude.rate_limit_rad_s,
           sizeof(debug->rate_limit_rad_s));
    memcpy(debug->rate_p_n_m, coax_ctrl_state.rate_output.p_term,
           sizeof(debug->rate_p_n_m));
    memcpy(debug->rate_i_n_m, coax_ctrl_state.rate_output.i_term,
           sizeof(debug->rate_i_n_m));
    memcpy(debug->rate_d_n_m, coax_ctrl_state.rate_output.d_term,
           sizeof(debug->rate_d_n_m));
    memcpy(debug->rate_ff_n_m, coax_ctrl_state.rate_output.ff_term,
           sizeof(debug->rate_ff_n_m));
    debug->total_force_n = solution->total_force_n;
    debug->moment_utilization = solution->moment_utilization;
    debug->thrust_utilization = solution->thrust_utilization;
}

static float coax_ctrl_balance_protection_scale(
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_BalanceSolution *solution,
    uint32_t *flags)
{
    float scale = 1.0f;
    float candidate;

    *flags = 0U;
    if ((reference->direct_attitude_target_valid == 0U) &&
        (coax_ctrl_params.vel_loop_enable >= 0.5f) &&
        (reference->horizontal_velocity_valid == 0U)) {
        *flags |= DRV_COAX_CTRL_PROTECT_VELOCITY_INVALID;
        scale = 0.0f;
    }

    candidate = coax_ctrl_protection_scale(
        solution->attitude_tilt_error_rad,
        DRV_COAX_CTRL_ATTITUDE_PROTECT_START_RAD,
        DRV_COAX_CTRL_ATTITUDE_PROTECT_END_RAD);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_ATTITUDE;
        scale = fminf(scale, candidate);
    }

    candidate = coax_ctrl_protection_scale(
        solution->moment_utilization,
        DRV_COAX_CTRL_MOMENT_PROTECT_START,
        DRV_COAX_CTRL_MOMENT_PROTECT_END);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_MOMENT;
        scale = fminf(scale, candidate);
    }

    candidate = coax_ctrl_protection_scale(
        solution->thrust_utilization,
        DRV_COAX_CTRL_THRUST_PROTECT_START,
        DRV_COAX_CTRL_THRUST_PROTECT_END);
    if (candidate < 1.0f) {
        *flags |= DRV_COAX_CTRL_PROTECT_THRUST;
        scale = fminf(scale, candidate);
    }

    return scale;
}

static void coax_ctrl_compute_balance_command(
    const DRV_COAX_CTRL_AttitudeInput *attitude,
    const DRV_COAX_CTRL_Reference *reference,
    const DRV_COAX_CTRL_Schedule *schedule,
    DRV_COAX_CTRL_Debug *debug,
    DRV_COAX_CTRL_BalanceSolution *solution)
{
    float horizontal_scale;
    DRV_COAX_CTRL_Schedule recalc_schedule;

    coax_ctrl_compute_accel_cmd(attitude,
                                reference,
                                schedule,
                                1.0f,
                                debug);
    coax_ctrl_compute_balance_solution(attitude, reference, schedule,
                                       debug, solution);
    horizontal_scale = coax_ctrl_balance_protection_scale(reference,
                                                           solution,
                                                           &debug->protection_flags);

    /* 保护缩放介入时按缩放后的加速度指令重算一遍，让力矩与实际下发一致。 */
    if (horizontal_scale < 0.999f) {
        recalc_schedule = *schedule;
        recalc_schedule.position_update = 0U;
        recalc_schedule.velocity_update = 0U;
        recalc_schedule.rate_update = 0U;
        coax_ctrl_compute_accel_cmd(attitude,
                                    reference,
                                    &recalc_schedule,
                                    horizontal_scale,
                                    debug);
        coax_ctrl_compute_balance_solution(attitude, reference,
                                           &recalc_schedule, debug, solution);
    }

    coax_ctrl_commit_shaping(schedule);
    debug->horizontal_command_scale = horizontal_scale;
    coax_ctrl_state.translation_saturation.horizontal_scale = horizontal_scale;
    coax_ctrl_state.translation_saturation.tilt_saturated =
        (horizontal_scale < 0.999f) ? 1U : 0U;
}

static void coax_ctrl_allocate_motor_thrust(float total_force_n,
                                            float yaw_torque_cmd,
                                            float *upper_n,
                                            float *lower_n,
                                            uint8_t *upper_saturated,
                                            uint8_t *lower_saturated)
{
    const float ku = coax_ctrl_params.yaw_torque_upper_m_per_n;
    const float kl = coax_ctrl_params.yaw_torque_lower_m_per_n;
    const float single_max_n = coax_ctrl_single_max_thrust_n();
    float denom = ku + kl;

    if (denom < DRV_COAX_CTRL_RATE_SCALE_EPS) {
        denom = DRV_COAX_CTRL_RATE_SCALE_EPS;
    }

    /*
     * 由 Mz = P*(kl*T_lower - ku*T_upper) 与 F = T_upper + T_lower 反解，
     * P = coax_ctrl_yaw_torque_polarity() = ±1，故 1/P = P。
     * P=+1 时与历史实现逐位相同——这次只是把隐含假设变成可推导的。
     */
    const float yaw_torque = coax_ctrl_yaw_torque_polarity() * yaw_torque_cmd;
    const float upper_raw = (kl * total_force_n - yaw_torque) / denom;
    const float lower_raw = (ku * total_force_n + yaw_torque) / denom;

    *upper_n = upper_raw;
    *lower_n = lower_raw;

    *upper_n = coax_ctrl_clamp_f32(*upper_n, 0.0f, single_max_n);
    *lower_n = coax_ctrl_clamp_f32(*lower_n, 0.0f, single_max_n);
    if (upper_saturated != NULL) {
        *upper_saturated = (fabsf(*upper_n - upper_raw) >
                            DRV_COAX_CTRL_RATE_SCALE_EPS) ? 1U : 0U;
    }
    if (lower_saturated != NULL) {
        *lower_saturated = (fabsf(*lower_n - lower_raw) >
                            DRV_COAX_CTRL_RATE_SCALE_EPS) ? 1U : 0U;
    }
}

/*
 * 吊绳偏航辨识（SYSID MODE YAW，doc/sysid-yaw-contract.md）用的公开入口：
 * 与生产偏航通道同一套分配（含偏航极性 P 与单桨推力上限钳位），不改任何行为，只包一层。
 */
void DRV_COAX_CTRL_AllocateYawPair(float total_force_n, float yaw_moment_n_m,
                                   float *upper_n, float *lower_n, uint8_t *saturated)
{
    float upper = 0.0f;
    float lower = 0.0f;
    uint8_t upper_saturated = 0U;
    uint8_t lower_saturated = 0U;

    DRV_COAX_CTRL_Init();
    coax_ctrl_allocate_motor_thrust(total_force_n, yaw_moment_n_m, &upper, &lower,
                                    &upper_saturated, &lower_saturated);
    if (upper_n != NULL) { *upper_n = upper; }
    if (lower_n != NULL) { *lower_n = lower; }
    if (saturated != NULL) { *saturated = ((upper_saturated | lower_saturated) != 0U) ? 1U : 0U; }
}

void DRV_COAX_CTRL_GetYawTorqueCoefficients(float *upper_m_per_n, float *lower_m_per_n)
{
    DRV_COAX_CTRL_Init();
    if (upper_m_per_n != NULL) { *upper_m_per_n = coax_ctrl_params.yaw_torque_upper_m_per_n; }
    if (lower_m_per_n != NULL) { *lower_m_per_n = coax_ctrl_params.yaw_torque_lower_m_per_n; }
}

float DRV_COAX_CTRL_YawLimitMomentNm(float total_force_n)
{
    DRV_COAX_CTRL_Init();
    return coax_ctrl_yaw_limit_moment(total_force_n);
}

void DRV_COAX_CTRL_Init(void)
{
    if (coax_ctrl_initialized == 0U) {
        DRV_COAX_CTRL_GetDefaultParams(&coax_ctrl_params);
        DRV_COAX_CTRL_GetDefaultServoCalibration(
            &coax_ctrl_servo_calibration);
        DRV_COAX_CTRL_ResetState();
        coax_ctrl_initialized = 1U;
    }
}

/* 回路状态（积分、微分滤波、饱和反馈、平移环）。RunScheduled 的 integrator_reset 只清这些。 */
static void coax_ctrl_reset_loop_state(void)
{
    memset(&coax_ctrl_state, 0, sizeof(coax_ctrl_state));
    memset(&coax_ctrl_last_debug, 0, sizeof(coax_ctrl_last_debug));
}

void DRV_COAX_CTRL_ResetState(void)
{
    coax_ctrl_reset_loop_state();
    coax_ctrl_reset_shaping();
}

void DRV_COAX_CTRL_GetDefaultParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    memset(params, 0, sizeof(*params));
    params->position.pos_kp[0] = 0.375f;
    params->position.pos_kp[1] = 0.375f;
    params->position.pos_kp[2] = 3.8f;
    params->position.vel_kp[0] = 0.80f;
    params->position.vel_kp[1] = 0.80f;
    params->position.vel_kp[2] = 1.0f;
    params->position.xy_speed_limit_m_s = 0.40f;
    params->position.z_speed_limit_up_m_s = 0.30f;
    params->position.z_speed_limit_down_m_s = 0.30f;
    params->position.vel_integrator_limit[0] = 1.50f;
    params->position.vel_integrator_limit[1] = 1.50f;
    params->position.vel_integrator_limit[2] = 1.50f;
    params->position.xy_accel_limit_m_s2 = 3.70f;
    params->position.z_accel_limit_up_m_s2 = 3.70f;
    params->position.z_accel_limit_down_m_s2 = 3.70f;
    params->position.accel_lpf_cutoff_hz = 20.0f;
    params->vel_loop_enable = 1.0f;
    /*
     * 姿态环 kp 的单位是 1/s（输出期望角速度，不是力矩），与力臂无关。
     * 横滚/俯仰：2026-09-28 晚的 A1 基线（与速率环同一组设计，出处见下方速率环注释）。
     */
    params->attitude.att_kp[0] = 1.8105f;
    params->attitude.att_kp[1] = 1.7131f;
    params->attitude.att_kp[2] = 1.0f / 0.15f;
    params->attitude.rate_limit_rad_s[0] = 3.49065850f;
    params->attitude.rate_limit_rad_s[1] = 3.49065850f;
    /*
     * 偏航期望角速度上限：作者要求由 60°/s 放开到 200°/s（2026-09-07），
     * 与 roll/pitch 取齐，用于验证"偏航几乎没反应是不是被这条钳位限住的"。
     *
     * 这是一次**放宽限幅**，按 AGENTS.md 规则 4 属需作者批准的变更，作者已明确
     * 指示。放宽的直接后果：角度环输出不再被钳在 60°/s，同样的角度误差会要求
     * 更大的角速度，进而更容易顶到分配器的偏航力矩上限（悬停约 7e-4 N·m）。
     *
     * 需要留意的是它**大概率不是**主因：角度环输出 omega_sp 要靠内环去跟，而
     * 内环带宽只有约 0.0105 1/s，本来就到不了 60°/s。真正让"调角度 P 没反应"的
     * 是 rate.kp[2]=0 时固件按除零保护拒收 coax.yaw_angle_kp（界面此前不显示这个
     * 拒绝，本次一并修掉）。测完请按结论决定收回还是保留。
     */
    params->attitude.rate_limit_rad_s[2] = 3.49065850f;
    /*
     * 横滚/俯仰角速度环里凡是**力矩单位**的量——kp [N·m/(rad/s)]、ki [N·m/rad]、
     * kd [N·m·s²/rad]、积分限幅 [N·m]、前馈倍率 ff（乘在 J·α 这个力矩项上）——都按
     * **实测机体**的倾转力臂 L = 重心 z − 舵机转轴 z = −0.0218 − (−0.13) = 0.1082 m 取值
     * （2026-09-27 称重 1145.3 g、自由摆 + 挂砝码测得重心板下 0.0218 m）。这组数与
     * 2026-09-28 存进这架飞机 Flash 的配置逐位相同：机体数据只存 Flash（见
     * drv_airframe_params.c），这里的默认值只在控制配置丢失时顶上，必须与那架机体配套。
     *
     * 俯仰：光杆辨识对象与**鲁棒**整定方法（2026-09-28 舵机 50 Hz 时的第一版）。回差补偿开、
     * 带桨 FF 双脉冲两轮联合（rod_004508 + rod_003912，带内拟合 83%，κ = 0.997，I_cg 0.0359 kg·m²），
     * 飞行对象 延迟·舵机·(κ + ρs²)/(I_cg·s) 在模型不确定度角点（台架与悬停）上配合 6 Hz 出口陷波仍满足
     * GM ≥ 6 dB、PM ≥ 45° 的最大 kp，ki = kp·ωc/5。早先的名义整定 kp 0.291/ki 0.375
     * （台架 100% 验证 rod_005148/rod_005207）名义裕度够、但在角点上最坏只剩 0.3 dB。台架验证 rod_051131/
     * rod_051206/rod_051758（陷波 + 参考前馈）稳定，模型复现 0.98–0.99。
     * 数据与推导：data/analysis/sysid-rig-params/2026-09-28/summary.md 第 5、6 节。
     *
     * 横滚：台架只能 ±45°。−45° 斜杆 FF 联合（rod_055342 + rod_055508）减去俯仰推出横滚对象
     * （假设 Ixy = 0；作者决定不测 +45°），舵机比俯仰快（26 rad/s）。summary.md 第 7 节。
     *
     * 舵机 333 Hz（2026-09-28 起开机默认，BSP_PWM_SERVO_FRAME_HZ）：−45° 带载纯延迟 30.2 → 22.4 ms，
     * 两轴按减去的 7.8 ms 重新做同一套角点鲁棒整定（design_333hz.py；横滚对象取 50/333 Hz 两组 −45°
     * 联合的平均：Ixx 0.0343、κ 0.99、ρ 0.0048）。上一版默认（L0）就是这组：俯仰 .265/.344/1.619、
     * 横滚 .249/.322/1.619（kp/ki/角度，−45° 台架 rod_065433/rod_065500 验证），需要退回时用它。
     * 舵机改回 50 Hz 时须换回 50 Hz 那组（俯仰 0.240/0.288/1.524、横滚 0.196/0.215/1.375）。
     *
     * 现行默认 A1（作者 2026-09-28 晚存入 Flash 的新基线）：内环压榨版 A 档增益（悬停角点裕度放宽到
     * 5 dB/40°、ki = kp·ωc/4，inner_extreme.py）配宽陷波 Q1.2 与参考 ωr 12 / Td 45 ms——A 档原配的
     * 6 Hz Q3 窄陷波在约 15 Hz 少给约 11° 相位，台架上把舵机抖起来；换回 Q1.2 后 10–20 Hz 回到 L0 水平。
     * 俯仰 kp 0.2748/ki 0.4607/角度 1.7131、横滚 kp 0.2748/ki 0.4862/角度 1.8105；悬停角点最坏
     * 俯仰 9.9 dB/41°、横滚 8.9 dB/42°。−45° 台架 RATE rod_224114、ANGLE rod_224131 验证（模型复现
     * 1.00/0.99，ANGLE 跟踪误差较 L0 −13%）。summary.md 第 10 节。
     *
     * 力矩只是"控制律的数字"与"舵机角度"之间的齿轮比：倾角 = asin(力矩 / (L·T))，
     * 每单位误差打出的舵机偏角 ∝ 1/L。所以重心或舵机转轴一改，这几项 N·m 增益就要按
     * 新力臂重新推导（上位机「临时应用/恢复原参数」按 L当前/L录制 自动换算）。
     *
     * 积分限幅 0.05 N·m（作者 2026-09-27 同意，原 0.013）：悬停推力 11.2 N 下约可配平
     * 重心水平偏 4.5 mm。α_ff 只在姿态参考模型开启时非零（R-ATTFF-1）：俯仰 ff 取 1（κ ≈ 1，力矩单位即真实 N·m，
     * 前馈 = I·θ̈_ref）；横滚同取 1（Ixx 0.0343 为 −45° 推算，Ixy = 0 假设的误差只让前馈偏同样比例）。
     *
     * ⚠ 持久化在 Flash 里的用户增益不会随这里改变——已存的配置优先。
     */
    params->rate.kp[0] = 0.2748f;     /* 横滚 A1（舵机 333 Hz） */
    params->rate.ki[0] = 0.4862f;
    params->rate.kp[1] = 0.2748f;     /* 俯仰 A1（舵机 333 Hz） */
    params->rate.ki[1] = 0.4607f;
    /*
     * 系数 0.0105 = 偏航内环带宽 [1/s]，`rate.kp = I_zz * 带宽`。
     *
     * 这个数字看着别扭是有原因的：I_zz 在 2026-09-07 由 0.00035 改成 0.005
     * （机体模型现在在 drv_airframe_params.h），若沿用原来的 0.15，默认增益会从 5.25e-5 跳到
     * 7.5e-4 —— 而作者实测偏航内环 kp 到 1e-4 左右就抖振，7.5e-4 是那个阈值的
     * 7 倍多，任何一次"恢复默认"都会让飞机在偏航上立刻发散。所以这里保持默认
     * 增益的**数值**不变，只把系数改成它真实对应的带宽。
     *
     * 2026-09-07 二次修订：偏航反扭矩系数 k 由 1e-4 改成 5e-3（×50）之后，物理
     * 动作 ΔT = rate_kp·e/k 会缩小 50 倍。为了让这次换算**逐位不改变飞机行为**，
     * 这里把系数同比例放大 50 倍（0.0105 → 0.525），使 rate_kp/k 保持 0.525 不变。
     *
     * 于是内环带宽从"0.0105 1/s"变成"0.525 1/s"。**不是内环变快了**——是原来那个
     * 0.0105 本身就是被错误的 k 扭曲出来的假数字；同一份物理行为，用可解释的 k
     * 读出来就是 0.525 1/s（时间常数约 1.9 s）。
     *
     * 2026-09-11：I_zz 改为从机体模型取。模型没写过时它是 0，默认偏航增益也就
     * 是 0——"没有惯量就没有默认增益"是正确的，编一个非零默认值反而会让人以为
     * 那是按这架飞机算出来的。
     */
    params->rate.kp[2] = DRV_Airframe_Get()->izz_kgm2 * 0.525f;
    params->rate.integrator_limit[0] = 0.05f;
    params->rate.integrator_limit[1] = 0.05f;
    params->rate.integrator_limit[2] = 0.00020f;
    /*
     * 角加速度低通截止：2026-09-07 由 188.495559（30 Hz）改为 18.8495559（3 Hz）。
     *
     * 起因是三个轴的 rate.kd 一加就发散。共因不在控制律里，在出口：TIM2 预分频
     * 120、周期 19999 → 1 MHz 计数、20 ms 周期 = **50 Hz**，而两个 ESC 和两个舵机
     * 全挂在 htim2 上（见 Core/Src/tim.c 与 BSP/Src/bsp_pwm.c）。角速率环 500 Hz
     * 算 10 次只送得出去 1 次，纯死区时间约
     *     零阶保持半帧 10 ms + 本环离散化 1 ms + 脉冲本身 1.5 ms ≈ 13 ms。
     *
     * D 是唯一逃不掉这段死区的项：D 力矩 kd·α 与惯性力矩 J·α 之比是 kd/J，**与
     * 频率无关**；P 的回路增益按 1/ω 衰减、I 更快，只有 D 在高频不衰减，于是死区
     * 造成的相位翻转只有 D 会撞上。绕刚体积分一圈的回路传函是
     *     L(jω) = (kd/J)·H(jω)·e^(-jωT),  ∠L = -atan(ω/ω_c) - ω·T
     * 令 ∠L = -180° 解出穿越频率，再要求 |L| < 1，就得到 kd/J 的起振门槛：
     *     ω_c = 188.5 → ω_180 ≈ 182 rad/s，|H| = 0.72 → kd/J < 1.39
     *     ω_c =  18.8 → ω_180 ≈ 132 rad/s，|H| = 0.14 → kd/J < 7.08
     * 也就是说旧截止把 D 最有害的那一段（29 Hz 处相位已完全倒置）原样放了过去，
     * 而执行器的 Nyquist 只有 25 Hz。横滚/俯仰 J = 0.051 时门槛 kd ≈ 0.07，偏航
     * J = 0.005 时仅 ≈ 0.007 —— 按 kp 的量级（rate.kp[0] = 0.11）去试 kd，三个轴
     * 会同时越线，这正是实测到的现象。
     *
     * 代价近乎为零：真正有用的阻尼只需要 kd/J ≈ 0.2~0.6，回路穿越在 2 rad/s 量级，
     * 3 Hz 截止在那里只带来 6° 相位滞后、0.6% 幅值衰减；被滤掉的全是执行器本来就
     * 跟不上的频段。附带效果是奈奎斯特自激门槛（kd/J < 2/a - 1，a = 1-exp(-ω_c·dt)）
     * 从 5.37 抬到 54，彻底不再是约束。
     *
     * ⚠ Flash 里已持久化的配置存的是 Hz（app_control_config_store.c），不会跟着
     * 这个默认值走；要让新截止生效需重置参数或显式下发
     * angular_accel_lpf_cutoff_rad_s。
     *
     * 真正的天花板仍是 50 Hz 出口本身：ESC 被舵机拖在同一个定时器上，单把 ESC 挪
     * 到独立定时器跑 400 Hz，死区就从 13 ms 降到 4 ms 以内，比任何调参都管用。那
     * 属于 CubeMX 生成代码，须在 CubeMX 里重配，不能手改。
     */
    params->rate.alpha_lpf_cutoff_rad_s = 18.8495559f;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        params->rate.large_error_threshold[axis] = 1.5f;
        params->rate.large_error_scale[axis] = 0.0f;
    }
    /* 前馈倍率乘在 J·α 力矩项上（力矩单位已是真实 N·m）；偏航不动。 */
    params->rate.ff_gain[0] = 1.0f;
    params->rate.ff_gain[1] = 1.0f;
    params->rate.ff_gain[2] = 1.0f;
    params->tilt_limit_rad = DRV_COAX_CTRL_TILT_LIMIT_RAD;
    /*
     * 指令整形与出口陷波**代码默认**全关（三个开关 notch_hz、notch2_hz、wr 为 0），控制律与加入
     * 它们之前逐位相同。这架飞机的 Flash 里是开的（A1：notch 6 Hz Q1.2、wr 12、Td 45 ms，2026-09-28 晚
     * 作者存入，快照 data/analysis/sysid-rig-params/2026-09-28/fc_config/fc_params_2026-09-28_best.json）；
     * 配置丢失时机体参数一起丢、无法解锁，按快照整份恢复。Q 1.2 与 Td 45 ms 按 A1 预置。
     * 第二级（约 16 Hz、Q≈1 压舵机高阶动态/结构那段约 15 Hz 的回路，hf_filter_study.py）待台架
     * 试用后再定，Q 预置 1.0。
     */
    params->rate_out_notch_hz = 0.0f;
    params->rate_out_notch_q = 1.2f;
    params->rate_out_notch2_hz = 0.0f;
    params->rate_out_notch2_q = 1.0f;
    params->att_ref_wr_rad_s = 0.0f;
    params->att_ref_delay_ms = 45.0f;
    params->hover_thrust_n = 0.0f;
    /* 默认开（2026-09-30 台架验证）：高度环 3/8/3 在融合关（vz 滞后约 92 ms）时位置环 PM 为负。 */
    params->z_vel_fusion = 1.0f;
    /* 飞行限幅默认 = 原写死常量：0.40 m、20°、60°/s（约 1.0472 rad/s）。 */
    params->alt_max_m = 0.40f;
    params->manual_tilt_max_rad = 0.349065850f;
    params->yaw_stick_rate_rad_s = 1.04719758f;
    coax_ctrl_apply_fixed_model_params(params);
}

void DRV_COAX_CTRL_ResetParams(void)
{
    DRV_COAX_CTRL_GetDefaultParams(&coax_ctrl_params);
    DRV_COAX_CTRL_ResetState();
}

void DRV_COAX_CTRL_GetParams(DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return;
    }

    DRV_COAX_CTRL_Init();
    /* 回读也走同一条刷新路径，上位机读到的质量/惯量与控制律正在用的是同一组。 */
    coax_ctrl_apply_fixed_model_params(&coax_ctrl_params);
    *params = coax_ctrl_params;
}

static uint8_t coax_ctrl_try_set_params(const DRV_COAX_CTRL_Params *params)
{
    if (params == NULL) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    DRV_COAX_CTRL_Params candidate = *params;
    coax_ctrl_apply_fixed_model_params(&candidate);
    if (coax_ctrl_params_valid(&candidate) != 0U) {
        coax_ctrl_params = candidate;
        DRV_COAX_CTRL_ResetState();
        return 1U;
    }
    return 0U;
}

void DRV_COAX_CTRL_SetParams(const DRV_COAX_CTRL_Params *params)
{
    (void)coax_ctrl_try_set_params(params);
}

uint32_t DRV_COAX_CTRL_ParamCount(void)
{
    return coax_ctrl_param_count;
}

const char *DRV_COAX_CTRL_ParamName(uint32_t index)
{
    return (index < coax_ctrl_param_count) ? coax_ctrl_param_table[index].name : NULL;
}

uint8_t DRV_COAX_CTRL_GetParam(const char *name, float *value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);

    if ((name == NULL) || (value == NULL)) {
        return 0U;
    }

    DRV_COAX_CTRL_Init();
    if (entry == NULL) {
        return 0U;
    }
    *value = *coax_ctrl_param_ptr(&coax_ctrl_params, entry);
    return 1U;
}

uint8_t DRV_COAX_CTRL_SetParam(const char *name, float value)
{
    const DRV_COAX_CTRL_ParamEntry *entry = coax_ctrl_find_param(name);
    DRV_COAX_CTRL_Params candidate;

    if ((name == NULL) || !isfinite(value) || (value < 0.0f)) {
        return 0U;
    }
    DRV_COAX_CTRL_Init();
    candidate = coax_ctrl_params;
    if (coax_ctrl_param_value_valid(entry, value) == 0U) {
        return 0U;
    }

    *coax_ctrl_param_ptr(&candidate, entry) = value;
    if (coax_ctrl_params_valid(&candidate) == 0U) {
        return 0U;
    }

    coax_ctrl_params = candidate;
    DRV_COAX_CTRL_ResetState();
    return 1U;
}

void DRV_COAX_CTRL_GetDefaultServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration)
{
    if (calibration == NULL) {
        return;
    }
    calibration->center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] =
        DRV_COAX_CTRL_SERVO_ALPHA_CENTER_US;
    calibration->center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] =
        DRV_COAX_CTRL_SERVO_BETA_CENTER_US;
    calibration->min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] =
        DRV_COAX_CTRL_SERVO_ALPHA_MIN_US;
    calibration->min_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] =
        DRV_COAX_CTRL_SERVO_BETA_MIN_US;
    calibration->max_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] =
        DRV_COAX_CTRL_SERVO_ALPHA_MAX_US;
    calibration->max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX] =
        DRV_COAX_CTRL_SERVO_BETA_MAX_US;
    calibration->pulse_sign[DRV_COAX_CTRL_SERVO_ALPHA_INDEX] = 1;
    calibration->pulse_sign[DRV_COAX_CTRL_SERVO_BETA_INDEX] = 1;
}

uint8_t DRV_COAX_CTRL_ValidateServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration)
{
    uint32_t index;

    if (calibration == NULL) {
        return 0U;
    }
    for (index = 0U; index < DRV_COAX_CTRL_SERVO_COUNT; ++index) {
        const uint16_t center = calibration->center_us[index];
        const uint16_t minimum = calibration->min_us[index];
        const uint16_t maximum = calibration->max_us[index];
        if ((minimum < DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US) ||
            (maximum > DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US) ||
            (minimum >= center) || (center >= maximum) ||
            ((uint16_t)(center - minimum) <
             DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US) ||
            ((uint16_t)(maximum - center) <
             DRV_COAX_CTRL_SERVO_MIN_CAL_SPAN_US) ||
            ((calibration->pulse_sign[index] != 1) &&
             (calibration->pulse_sign[index] != -1))) {
            return 0U;
        }
    }
    return 1U;
}

void DRV_COAX_CTRL_ResetServoCalibration(void)
{
    DRV_COAX_CTRL_GetDefaultServoCalibration(&coax_ctrl_servo_calibration);
    DRV_COAX_CTRL_ResetState();
}

void DRV_COAX_CTRL_GetServoCalibration(
    DRV_COAX_CTRL_ServoCalibration *calibration)
{
    if (calibration == NULL) {
        return;
    }
    DRV_COAX_CTRL_Init();
    *calibration = coax_ctrl_servo_calibration;
}

uint8_t DRV_COAX_CTRL_SetServoCalibration(
    const DRV_COAX_CTRL_ServoCalibration *calibration)
{
    if (DRV_COAX_CTRL_ValidateServoCalibration(calibration) == 0U) {
        return 0U;
    }
    DRV_COAX_CTRL_Init();
    coax_ctrl_servo_calibration = *calibration;
    DRV_COAX_CTRL_ResetState();
    return 1U;
}

static uint16_t coax_ctrl_tilt_rad_to_servo_pulse(float tilt_rad,
                                                  uint16_t center_us,
                                                  uint16_t min_us,
                                                  uint16_t max_us)
{
    const float servo_span_us = (float)(DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US -
                                        DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US);
    const float servo_us_per_rad = servo_span_us / DRV_COAX_CTRL_SERVO_TRAVEL_RAD;
    DRV_COAX_CTRL_Init();
    const float tilt = coax_ctrl_clamp_f32(tilt_rad,
                                          -coax_ctrl_params.tilt_limit_rad,
                                           coax_ctrl_params.tilt_limit_rad);
    const float pulse_f = (float)center_us + tilt * servo_us_per_rad;
    const int32_t pulse_i =
        (int32_t)(pulse_f + ((pulse_f >= 0.0f) ? 0.5f : -0.5f));

    return coax_ctrl_clamp_u16(pulse_i, min_us, max_us);
}

uint16_t DRV_COAX_CTRL_AlphaTiltRadToServoPulse(float tilt_rad)
{
    DRV_COAX_CTRL_Init();
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
        coax_ctrl_servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        coax_ctrl_servo_calibration.min_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX],
        coax_ctrl_servo_calibration.max_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]);
}

uint16_t DRV_COAX_CTRL_BetaTiltRadToServoPulse(float tilt_rad)
{
    DRV_COAX_CTRL_Init();
    return coax_ctrl_tilt_rad_to_servo_pulse(tilt_rad,
        coax_ctrl_servo_calibration.center_us[DRV_COAX_CTRL_SERVO_BETA_INDEX],
        coax_ctrl_servo_calibration.min_us[DRV_COAX_CTRL_SERVO_BETA_INDEX],
        coax_ctrl_servo_calibration.max_us[DRV_COAX_CTRL_SERVO_BETA_INDEX]);
}

static void coax_ctrl_body_tilt_to_servo_tilts(float body_x_tilt_rad,
                                               float body_y_tilt_rad,
                                               float *servo_alpha_tilt_rad,
                                               float *servo_beta_tilt_rad)
{
    /*
     * 倾转机构运动学 —— 固定，不是可调符号。
     *
     * 倾转舵机组相对机体转了 90°：控制左右倾（body_y_tilt）的是 1 号舵机
     * （alpha 通道），控制前后倾（body_x_tilt）的是 2 号舵机（beta 通道），
     * 且两条连杆同手性，所以这里是两个同号的负号，而不是一正一负。
     *
     * 这两个负号 + 标定写入的 pulse_sign 共同决定了 body_*_tilt 的正方向。
     * 以在册标定（data/calibration/servo_mechanical/2026-08-30，
     * alpha_sign=beta_sign=-1）代入本函数与 DRV_COAX_CTRL_BodyTiltRadToServoPulses：
     *     servo_alpha_us = center + body_y_tilt * k
     *     servo_beta_us  = center + body_x_tilt * k
     * 作者已实测确认该标定下"舵机脉宽加大 = 推力轴向左(+Y) / 向后(-X)"，
     * 于是两个倾转量的正方向被钉死为：
     *     body_y_tilt > 0  →  推力轴倒向 +Y（左）
     *     body_x_tilt > 0  →  推力轴倒向 -X（后）
     * 一个顺 +Y、一个逆 +X 看着别扭，但这正是让 τ = r × F 的 roll/pitch 两轴
     * 都写成同一形式 −r_z·T·sin(tilt) 的定义（推导见文件头"倾转 → 机体力矩"）。
     *
     * 分工：本函数与力矩律固定不变；换飞机、换舵机、连杆反装，全部只允许
     * 改上位机标定写进来的 pulse_sign / center_us / min_us / max_us。
     */
    if (servo_alpha_tilt_rad != NULL) {
        *servo_alpha_tilt_rad = -body_y_tilt_rad;
    }
    if (servo_beta_tilt_rad != NULL) {
        *servo_beta_tilt_rad = -body_x_tilt_rad;
    }
}

void DRV_COAX_CTRL_BodyTiltRadToServoPulses(float body_x_tilt_rad,
                                            float body_y_tilt_rad,
                                            uint16_t *servo_alpha_us,
                                            uint16_t *servo_beta_us)
{
    float servo_alpha_tilt_rad = 0.0f;
    float servo_beta_tilt_rad = 0.0f;

    coax_ctrl_body_tilt_to_servo_tilts(body_x_tilt_rad,
                                       body_y_tilt_rad,
                                       &servo_alpha_tilt_rad,
                                       &servo_beta_tilt_rad);
    if (servo_alpha_us != NULL) {
        *servo_alpha_us = DRV_COAX_CTRL_AlphaTiltRadToServoPulse(
            servo_alpha_tilt_rad *
            (float)coax_ctrl_servo_calibration.pulse_sign[
                DRV_COAX_CTRL_SERVO_ALPHA_INDEX]);
    }
    if (servo_beta_us != NULL) {
        *servo_beta_us = DRV_COAX_CTRL_BetaTiltRadToServoPulse(
            servo_beta_tilt_rad *
            (float)coax_ctrl_servo_calibration.pulse_sign[
                DRV_COAX_CTRL_SERVO_BETA_INDEX]);
    }
}

static void coax_ctrl_servo_pulses_to_body_tilts(uint16_t servo_alpha_us,
                                                  uint16_t servo_beta_us,
                                                  float *body_x_tilt_rad,
                                                  float *body_y_tilt_rad)
{
    const float servo_span_us =
        (float)(DRV_COAX_CTRL_SERVO_PHYSICAL_MAX_US -
                DRV_COAX_CTRL_SERVO_PHYSICAL_MIN_US);
    const float rad_per_us = DRV_COAX_CTRL_SERVO_TRAVEL_RAD / servo_span_us;
    const float servo_alpha_tilt =
        ((float)servo_alpha_us -
         (float)coax_ctrl_servo_calibration.center_us[
             DRV_COAX_CTRL_SERVO_ALPHA_INDEX]) * rad_per_us *
        (float)coax_ctrl_servo_calibration.pulse_sign[
            DRV_COAX_CTRL_SERVO_ALPHA_INDEX];
    const float servo_beta_tilt =
        ((float)servo_beta_us -
         (float)coax_ctrl_servo_calibration.center_us[
             DRV_COAX_CTRL_SERVO_BETA_INDEX]) * rad_per_us *
        (float)coax_ctrl_servo_calibration.pulse_sign[
            DRV_COAX_CTRL_SERVO_BETA_INDEX];

    if (body_x_tilt_rad != NULL) {
        *body_x_tilt_rad = -servo_beta_tilt;
    }
    if (body_y_tilt_rad != NULL) {
        *body_y_tilt_rad = -servo_alpha_tilt;
    }
}

void DRV_COAX_CTRL_SetThrustMap(const DRV_COAX_CTRL_ThrustMap *map)
{
    coax_ctrl_thrust_map = map;
}

const DRV_COAX_CTRL_ThrustMap *DRV_COAX_CTRL_GetThrustMap(void)
{
    return coax_ctrl_thrust_map;
}

uint16_t DRV_COAX_CTRL_ThrustToMotorPulse(float thrust_n)
{
    float thrust_g;
    const DRV_COAX_CTRL_ThrustMap *map = coax_ctrl_thrust_map;

    DRV_COAX_CTRL_Init();

    thrust_n = coax_ctrl_clamp_f32(thrust_n, 0.0f, coax_ctrl_single_max_thrust_n());
    if ((map != NULL) && (map->pulse_for_motor_thrust != NULL)) {
        return coax_ctrl_clamp_u16((int32_t)map->pulse_for_motor_thrust(thrust_n),
                                   BSP_PWM_ESC_MIN_US,
                                   BSP_PWM_ESC_MAX_US);
    }
    thrust_g = thrust_n * DRV_COAX_CTRL_GRAMS_PER_NEWTON * 2.0f;

    if (thrust_g <= coax_ctrl_dual_thrust_g[0]) {
        return coax_ctrl_dual_pwm_us[0];
    }

    for (uint32_t i = 1U; i < DRV_COAX_CTRL_THRUST_TABLE_POINTS; ++i) {
        if (thrust_g <= coax_ctrl_dual_thrust_g[i]) {
            const float left_g = coax_ctrl_dual_thrust_g[i - 1U];
            const float right_g = coax_ctrl_dual_thrust_g[i];
            const float left_pwm = (float)coax_ctrl_dual_pwm_us[i - 1U];
            const float right_pwm = (float)coax_ctrl_dual_pwm_us[i];
            const float ratio =
                (right_g > left_g) ? ((thrust_g - left_g) / (right_g - left_g)) : 0.0f;
            const float pulse_f = left_pwm + ratio * (right_pwm - left_pwm);
            const int32_t pulse_i = (int32_t)(pulse_f + 0.5f);
            return coax_ctrl_clamp_u16(pulse_i,
                                       BSP_PWM_ESC_MIN_US,
                                       BSP_PWM_ESC_MAX_US);
        }
    }

    return BSP_PWM_ESC_MAX_US;
}

void DRV_COAX_CTRL_TiltFromForce(const float force_n[3], float *roll_rad, float *pitch_rad)
{
    /* 两行公式原样从 coax_ctrl_solve_force 搬来（运算顺序不变，生产输出逐位不变）。 */
    const float pitch = atan2f(force_n[0], force_n[2]);
    const float roll = -atan2f(force_n[1] * cosf(pitch), force_n[2]);

    if (roll_rad != NULL) {
        *roll_rad = roll;
    }
    if (pitch_rad != NULL) {
        *pitch_rad = pitch;
    }
}

float DRV_COAX_CTRL_EffectiveMassKg(float nominal_mass_kg)
{
    const float g = coax_ctrl_params.gravity_m_s2;

    if ((coax_ctrl_hover_adapt_n > 0.0f) && (g > 0.0f)) {
        return coax_ctrl_hover_adapt_n / g;
    }
    if ((coax_ctrl_params.hover_thrust_n > 0.0f) && (g > 0.0f)) {
        return coax_ctrl_params.hover_thrust_n / g;
    }
    return nominal_mass_kg;
}

float DRV_COAX_CTRL_ConfiguredHoverThrustN(void)
{
    if (coax_ctrl_params.hover_thrust_n > 0.0f) {
        return coax_ctrl_params.hover_thrust_n;
    }
    return coax_ctrl_params.mass_kg * coax_ctrl_params.gravity_m_s2;
}

float DRV_COAX_CTRL_GetHoverThrustAdapt(void)
{
    return coax_ctrl_hover_adapt_n;
}

void DRV_COAX_CTRL_SetHoverThrustAdapt(float hover_n)
{
    const float g = coax_ctrl_params.gravity_m_s2;
    const float old_n = DRV_COAX_CTRL_EffectiveMassKg(coax_ctrl_params.mass_kg) * g;
    float new_n;
    float *integ = &coax_ctrl_state.position.velocity_integrator_m_s2[2];
    const float limit = coax_ctrl_params.position.vel_integrator_limit[2];

    if (!(hover_n >= DRV_COAX_CTRL_HOVER_THRUST_MIN_N) ||
        !(hover_n <= DRV_COAX_CTRL_HOVER_THRUST_MAX_N)) {
        hover_n = 0.0f;
    }
    coax_ctrl_hover_adapt_n = hover_n;
    new_n = DRV_COAX_CTRL_EffectiveMassKg(coax_ctrl_params.mass_kg) * g;
    if ((old_n > 0.0f) && (new_n > 0.0f) && (old_n != new_n)) {
        /* 无扰切换：竖直速度积分反向挪同样的推力，当拍合推力不变（drv_hover_adapt.h）。 */
        *integ += DRV_HoverAdapt_IntegratorShift(
            old_n, new_n, g, coax_ctrl_state.velocity_output.accel_sat_m_s2[2]);
        if (limit > 0.0f) {
            *integ = coax_ctrl_clamp_f32(*integ, -limit, limit);
        }
    }
}

float DRV_COAX_CTRL_MotorPulseToTotalThrust(uint16_t pulse_us)
{
    float thrust_g;
    const DRV_COAX_CTRL_ThrustMap *map = coax_ctrl_thrust_map;

    DRV_COAX_CTRL_Init();

    if ((map != NULL) && (map->total_thrust_for_pulse != NULL)) {
        return coax_ctrl_clamp_f32(map->total_thrust_for_pulse(pulse_us),
                                   0.0f,
                                   2.0f * coax_ctrl_single_max_thrust_n());
    }

    if (pulse_us <= coax_ctrl_dual_pwm_us[0]) {
        return 0.0f;
    }

    for (uint32_t i = 1U; i < DRV_COAX_CTRL_THRUST_TABLE_POINTS; ++i) {
        if (pulse_us <= coax_ctrl_dual_pwm_us[i]) {
            const float left_pwm = (float)coax_ctrl_dual_pwm_us[i - 1U];
            const float right_pwm = (float)coax_ctrl_dual_pwm_us[i];
            const float ratio = ((float)pulse_us - left_pwm) /
                                (right_pwm - left_pwm);

            thrust_g = coax_ctrl_dual_thrust_g[i - 1U] +
                       ratio * (coax_ctrl_dual_thrust_g[i] -
                                coax_ctrl_dual_thrust_g[i - 1U]);
            return coax_ctrl_clamp_f32(
                thrust_g / DRV_COAX_CTRL_GRAMS_PER_NEWTON,
                0.0f,
                2.0f * coax_ctrl_single_max_thrust_n());
        }
    }

    return coax_ctrl_dual_thrust_g[DRV_COAX_CTRL_THRUST_TABLE_POINTS - 1U] /
           DRV_COAX_CTRL_GRAMS_PER_NEWTON;
}

void DRV_COAX_CTRL_RunScheduled(const DRV_COAX_CTRL_AttitudeInput *attitude,
                                const DRV_COAX_CTRL_Reference *reference,
                                const DRV_COAX_CTRL_Schedule *schedule,
                                DRV_COAX_CTRL_Output *output)
{
    DRV_COAX_CTRL_Debug debug;
    DRV_COAX_CTRL_BalanceSolution solution;
    float yaw_torque_cmd;
    float thrust_upper_n;
    float thrust_lower_n;
    float requested_alpha_rad;
    float requested_beta_rad;
    float achieved_total_force_n;
    uint8_t upper_motor_saturated = 0U;
    uint8_t lower_motor_saturated = 0U;

    if ((attitude == NULL) || (reference == NULL) || (schedule == NULL) ||
        (output == NULL)) {
        return;
    }

    DRV_COAX_CTRL_Init();
    /*
     * 机体模型可能在两次 Run 之间被上位机改写（量完新电池就地写入）。这里重刷
     * 一遍，避免出现"模型已改、控制律还在用旧质量"这种只有重启才会暴露的中间态。
     */
    coax_ctrl_apply_fixed_model_params(&coax_ctrl_params);
    if (schedule->integrator_reset != 0U) {
        /*
         * 只清回路状态，整形状态（参考模型、出口陷波）保留：直接姿态模式下这里每拍
         * 都成立，参考若跟着对齐到实测角，角度环就永远没有误差（见 DRV_COAX_CTRL_Shaping）。
         */
        coax_ctrl_reset_loop_state();
    }
    memset(&debug, 0, sizeof(debug));
    memset(output, 0, sizeof(*output));

    coax_ctrl_compute_balance_command(attitude, reference, schedule,
                                      &debug, &solution);
    /* 三轴力矩同出一套 SO(3) 控制律；分配器才是分叉点（倾转 vs 差动推力）。 */
    yaw_torque_cmd = solution.yaw_moment_applied_n_m;
    coax_ctrl_allocate_motor_thrust(debug.total_force_n,
                                    yaw_torque_cmd,
                                    &thrust_upper_n,
                                    &thrust_lower_n,
                                    &upper_motor_saturated,
                                    &lower_motor_saturated);

    output->thrust_upper_n = thrust_upper_n;
    output->thrust_lower_n = thrust_lower_n;
    requested_alpha_rad = solution.alpha_rad;
    requested_beta_rad = solution.beta_rad;
    {
        const DRV_COAX_CTRL_ThrustMap *map = coax_ctrl_thrust_map;
        if ((map != NULL) && (map->pulses_for_pair != NULL)) {
            map->pulses_for_pair(thrust_upper_n, thrust_lower_n,
                                 &output->motor_upper_us, &output->motor_lower_us);
            output->motor_upper_us = coax_ctrl_clamp_u16((int32_t)output->motor_upper_us,
                                                         BSP_PWM_ESC_MIN_US, BSP_PWM_ESC_MAX_US);
            output->motor_lower_us = coax_ctrl_clamp_u16((int32_t)output->motor_lower_us,
                                                         BSP_PWM_ESC_MIN_US, BSP_PWM_ESC_MAX_US);
        } else {
            output->motor_upper_us = DRV_COAX_CTRL_ThrustToMotorPulse(thrust_upper_n);
            output->motor_lower_us = DRV_COAX_CTRL_ThrustToMotorPulse(thrust_lower_n);
        }
    }
    DRV_COAX_CTRL_BodyTiltRadToServoPulses(requested_alpha_rad,
                                           requested_beta_rad,
                                           &output->servo_alpha_us,
                                           &output->servo_beta_us);
    coax_ctrl_servo_pulses_to_body_tilts(output->servo_alpha_us,
                                         output->servo_beta_us,
                                         &output->alpha_rad,
                                         &output->beta_rad);
    debug.tilt_out_rad[0] = output->alpha_rad;
    debug.tilt_out_rad[1] = output->beta_rad;

    debug.motor_thrust_cmd_n[0] = output->thrust_upper_n;
    debug.motor_thrust_cmd_n[1] = output->thrust_lower_n;
    debug.motor_cmd_us[0] = (float)output->motor_upper_us;
    debug.motor_cmd_us[1] = (float)output->motor_lower_us;
    /* 实际达成的偏航力矩：与分配式同一套极性，不能只在一边用。 */
    debug.yaw_torque_cmd =
        coax_ctrl_yaw_torque_polarity() *
        ((coax_ctrl_params.yaw_torque_lower_m_per_n * output->thrust_lower_n) -
         (coax_ctrl_params.yaw_torque_upper_m_per_n * output->thrust_upper_n));

    achieved_total_force_n = output->thrust_upper_n + output->thrust_lower_n;
    output->moment_achieved_n_m[0] = coax_ctrl_roll_moment_from_tilt(
        achieved_total_force_n, output->beta_rad);
    output->moment_achieved_n_m[1] = coax_ctrl_pitch_moment_from_tilt(
        achieved_total_force_n, output->alpha_rad, output->beta_rad);
    output->moment_achieved_n_m[2] = debug.yaw_torque_cmd;
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        uint8_t constrained =
            coax_ctrl_state.rate_output.saturated_pos[axis] ||
            coax_ctrl_state.rate_output.saturated_neg[axis];
        if ((axis == 0U) &&
            (fabsf(requested_beta_rad - output->beta_rad) >
             DRV_COAX_CTRL_SERVO_ANGLE_TOL_RAD)) constrained = 1U;
        if ((axis == 1U) &&
            (fabsf(requested_alpha_rad - output->alpha_rad) >
             DRV_COAX_CTRL_SERVO_ANGLE_TOL_RAD)) constrained = 1U;
        if ((axis == 2U) &&
            ((upper_motor_saturated != 0U) ||
             (lower_motor_saturated != 0U))) constrained = 1U;
        if ((constrained != 0U) && (solution.moment_cmd_n_m[axis] > 0.0f)) {
            output->saturation_positive[axis] = 1U;
        } else if ((constrained != 0U) &&
                   (solution.moment_cmd_n_m[axis] < 0.0f)) {
            output->saturation_negative[axis] = 1U;
        }
        coax_ctrl_state.moment_saturation_positive[axis] =
            output->saturation_positive[axis];
        coax_ctrl_state.moment_saturation_negative[axis] =
            output->saturation_negative[axis];
    }
    output->tilt_saturated =
        output->saturation_positive[0] || output->saturation_negative[0] ||
        output->saturation_positive[1] || output->saturation_negative[1] ||
        (debug.horizontal_command_scale < 0.999f);
    output->yaw_differential_saturated =
        output->saturation_positive[2] || output->saturation_negative[2];
    output->thrust_saturated =
        (solution.raw_total_force_n > coax_ctrl_force_cap_n(DRV_Airframe_Get()->max_total_force_n)) ||
        (fabsf((output->thrust_upper_n + output->thrust_lower_n) -
               debug.total_force_n) > DRV_COAX_CTRL_RATE_SCALE_EPS);
    memcpy(debug.moment_achieved_n_m, output->moment_achieved_n_m,
           sizeof(debug.moment_achieved_n_m));
    memcpy(debug.saturation_positive, output->saturation_positive,
           sizeof(debug.saturation_positive));
    memcpy(debug.saturation_negative, output->saturation_negative,
           sizeof(debug.saturation_negative));
    debug.thrust_saturated = output->thrust_saturated;
    debug.tilt_saturated = output->tilt_saturated;
    debug.yaw_differential_saturated = output->yaw_differential_saturated;
    coax_ctrl_state.translation_saturation.thrust_saturated =
        output->thrust_saturated;
    coax_ctrl_state.translation_saturation.tilt_saturated =
        output->tilt_saturated;
    memset(coax_ctrl_state.translation_saturation.pos_limit, 0,
           sizeof(coax_ctrl_state.translation_saturation.pos_limit));
    memset(coax_ctrl_state.translation_saturation.neg_limit, 0,
           sizeof(coax_ctrl_state.translation_saturation.neg_limit));
    for (uint32_t axis = 0U; axis < 3U; ++axis) {
        const uint8_t limited = (axis < 2U) ? output->tilt_saturated :
                                             output->thrust_saturated;
        if ((limited != 0U) && (debug.accel_out_m_s2[axis] > 0.0f)) {
            coax_ctrl_state.translation_saturation.pos_limit[axis] = 1U;
        } else if ((limited != 0U) && (debug.accel_out_m_s2[axis] < 0.0f)) {
            coax_ctrl_state.translation_saturation.neg_limit[axis] = 1U;
        }
    }

    coax_ctrl_last_debug = debug;
}

void DRV_COAX_CTRL_Run(const DRV_COAX_CTRL_AttitudeInput *attitude,
                       const DRV_COAX_CTRL_Reference *reference,
                       DRV_COAX_CTRL_Output *output)
{
    DRV_COAX_CTRL_Schedule schedule;

    if (reference == NULL) {
        return;
    }
    memset(&schedule, 0, sizeof(schedule));
    schedule.position_update = 1U;
    schedule.velocity_update = 1U;
    schedule.attitude_update = 1U;
    schedule.rate_update = 1U;
    schedule.integrator_enable = 1U;
    schedule.position_dt_s = reference->dt_sec;
    schedule.velocity_dt_s = reference->dt_sec;
    schedule.attitude_dt_s = reference->dt_sec;
    schedule.rate_dt_s = reference->dt_sec;
    DRV_COAX_CTRL_RunScheduled(attitude, reference, &schedule, output);
}

void DRV_COAX_CTRL_GetLastDebug(DRV_COAX_CTRL_Debug *debug)
{
    if (debug == NULL) {
        return;
    }

    *debug = coax_ctrl_last_debug;
}

uint8_t DRV_COAX_CTRL_SolveBodyTiltFromMoment(const float moment_n_m[3],
                                              float total_force_n,
                                              float *body_x_tilt_rad,
                                              float *body_y_tilt_rad)
{
    float beta_rad;
    float alpha_rad;

    DRV_COAX_CTRL_Init();

    if (body_x_tilt_rad != NULL) {
        *body_x_tilt_rad = 0.0f;
    }
    if (body_y_tilt_rad != NULL) {
        *body_y_tilt_rad = 0.0f;
    }
    if ((moment_n_m == NULL) || !isfinite(total_force_n) ||
        (total_force_n <= COAX_CTRL_MIN_SOLVE_FORCE_N) ||
        !isfinite(moment_n_m[0]) || !isfinite(moment_n_m[1])) {
        /*
         * 推力不足时直接回零倾角。反解器在 force≈0 时可达力矩区间退化成一个点，
         * 二分会收敛到 -tilt_limit —— 也就是"没有推力反而把舵机打到底"。
         * 这条判断就是挡住它的。
         */
        return 0U;
    }

    /* 与 coax_ctrl_solve_actuator_setpoints 同序：先 roll 解出 beta，再用 beta 解 pitch。 */
    coax_ctrl_apply_fixed_model_params(&coax_ctrl_params);
    if ((coax_ctrl_tilt_lever_usable(coax_ctrl_params.roll_tilt_lever_arm_m) == 0U) ||
        (coax_ctrl_tilt_lever_usable(coax_ctrl_params.pitch_tilt_lever_arm_m) == 0U)) {
        /* 机体模型无效或力臂过小：零倾角（舵机回中）并报失败，不给一个方向可能反了的解。 */
        return 0U;
    }
    beta_rad = coax_ctrl_solve_roll_tilt_from_moment(
        moment_n_m[0], total_force_n, coax_ctrl_params.tilt_limit_rad);
    alpha_rad = coax_ctrl_solve_pitch_tilt_from_moment(
        moment_n_m[1], total_force_n, beta_rad, coax_ctrl_params.tilt_limit_rad);

    /* 机体倾转与舵机倾转的换向定义见 coax_ctrl_body_tilt_to_servo_tilts。 */
    if (body_x_tilt_rad != NULL) {
        *body_x_tilt_rad = alpha_rad;
    }
    if (body_y_tilt_rad != NULL) {
        *body_y_tilt_rad = beta_rad;
    }
    return 1U;
}

uint8_t DRV_COAX_CTRL_MomentFromServoPulses(float total_force_n,
    uint16_t alpha_us, uint16_t beta_us, float moment[3])
{
    float x, y;
    if (moment == NULL || !isfinite(total_force_n) || total_force_n <= 0.0f) {
        return 0U;
    }
    DRV_COAX_CTRL_Init();
    coax_ctrl_servo_pulses_to_body_tilts(alpha_us, beta_us, &x, &y);
    coax_ctrl_apply_fixed_model_params(&coax_ctrl_params);
    if ((coax_ctrl_tilt_lever_usable(coax_ctrl_params.roll_tilt_lever_arm_m) == 0U) ||
        (coax_ctrl_tilt_lever_usable(coax_ctrl_params.pitch_tilt_lever_arm_m) == 0U)) {
        moment[0] = 0.0f;
        moment[1] = 0.0f;
        moment[2] = 0.0f;
        return 0U;
    }
    moment[0] = coax_ctrl_roll_moment_from_tilt(total_force_n, y);
    moment[1] = coax_ctrl_pitch_moment_from_tilt(total_force_n, x, y);
    moment[2] = 0.0f;
    return isfinite(moment[0]) && isfinite(moment[1]);
}
