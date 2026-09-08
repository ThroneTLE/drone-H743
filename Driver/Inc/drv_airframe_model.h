#ifndef DRV_AIRFRAME_MODEL_H
#define DRV_AIRFRAME_MODEL_H

#define DRV_AIRFRAME_BOARD_MASS_G                 75.0f
#define DRV_AIRFRAME_BATTERY_MASS_G              232.0f
#define DRV_AIRFRAME_BASE_MASS_G                  99.0f
#define DRV_AIRFRAME_SERVO_MOTOR_MASS_G          348.6f

#define DRV_AIRFRAME_BOARD_CG_Z_M                  0.0f
#define DRV_AIRFRAME_BATTERY_CG_Z_M                0.109f
#define DRV_AIRFRAME_BASE_CG_Z_M                  -0.117f
#define DRV_AIRFRAME_SERVO_MOTOR_CG_Z_M           -0.244f

#define DRV_AIRFRAME_MASS_KG                       1.3670f
#define DRV_AIRFRAME_CG_Z_M                       -0.0946f
#define DRV_AIRFRAME_IMU_Z_M                       0.0f

#define DRV_AIRFRAME_TETHER_ATTACH_Z_M             0.1563f
#define DRV_AIRFRAME_TETHER_ATTACH_TO_CG_M         0.2509f
#define DRV_AIRFRAME_TETHER_ROPE_M                 0.6400f
#define DRV_AIRFRAME_TETHER_ROD_TO_CG_M            0.8909f

#define DRV_AIRFRAME_SERVO_DEG_PER_US              0.090f
#define DRV_AIRFRAME_SERVO_US_PER_DEG             11.111111f

/* Installed bus-servo dynamics at full supply, motors stopped, 100 Hz PRAD.
 * Alpha is software servo index 0 / ID1 and drives gimbal beta (roll moment).
 * Beta is software servo index 1 / ID2 and drives gimbal alpha (pitch moment).
 * These +/-200 us FOPDT fits are large-signal engineering models; neutral
 * feedback is encoder-referenced and is not a mechanical trim correction. */
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_IDENT_STEP_US      200.0f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_GAIN                 0.781794f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_DELAY_S              0.016231f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_TAU_INCREASE_S       0.071236f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_TAU_DECREASE_S       0.306416f
#define DRV_AIRFRAME_SERVO_ALPHA_NEUTRAL_FEEDBACK_US        1457.177648f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_R2               0.963346f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_RMSE_US          17.696884f
#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_SAMPLE_COUNT    835U

#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_IDENT_STEP_US       200.0f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_GAIN                  0.769332f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_DELAY_S               0.041320f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_INCREASE_S        0.076881f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_DECREASE_S        0.057051f
#define DRV_AIRFRAME_SERVO_BETA_NEUTRAL_FEEDBACK_US         1509.527201f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_R2                0.994138f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_RMSE_US            8.459764f
#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_SAMPLE_COUNT    1253U

#define DRV_AIRFRAME_GRAVITY_M_S2                  9.81f
#define DRV_AIRFRAME_WEIGHT_N                     13.410270f
#define DRV_AIRFRAME_THRUST_TABLE_SCOPE            "dual_motor_total"
#define DRV_AIRFRAME_MAX_TOTAL_THRUST_G         1595.342f
#define DRV_AIRFRAME_MAX_TOTAL_FORCE_N            15.644959f
#define DRV_AIRFRAME_HOVER_THRUST_PERCENT         85.716236f

/* Measured neutral-axis distances for the serial tilt mechanism. */
#define DRV_AIRFRAME_PROP_PLANE_D_M                0.2500f
#define DRV_AIRFRAME_ROLL_AXIS_TO_PROP_PLANE_M     0.1450f
#define DRV_AIRFRAME_PITCH_AXIS_TO_PROP_PLANE_M    0.1050f

/* Controller effective thrust moment arms. */
#define DRV_AIRFRAME_PITCH_THRUST_LEVER_ARM_M      0.1450f
#define DRV_AIRFRAME_ROLL_THRUST_LEVER_ARM_M       0.1450f

/* Servo axis z-positions (origin = board center, z+ up) */
#define DRV_AIRFRAME_SERVO1_AXIS_Z_M              -0.161f
#define DRV_AIRFRAME_SERVO2_AXIS_Z_M              -0.215f

/* Effective thrust application point (coax twin-prop midpoint) */
#define DRV_AIRFRAME_THRUST_POINT_Z_M             -0.2955f

/*
 * Thrust application point expressed relative to the centre of gravity, in the
 * canonical FLU body frame (z+ up).  This is the `r` of `tau = r x F` and is
 * therefore the only thing that decides the *sign* of a tilt-generated moment.
 *
 * It is negative here: the tilt mechanism and both props hang below the board,
 * so the thrust acts below the CG.  Derive polarity from this constant instead
 * of writing a standalone sign macro -- a sign macro can be flipped to chase a
 * symptom, whereas this value can only change when the airframe is re-measured.
 */
#define DRV_AIRFRAME_THRUST_POINT_TO_CG_Z_M \
    (DRV_AIRFRAME_THRUST_POINT_Z_M - DRV_AIRFRAME_CG_Z_M)

/*
 * 下桨旋向（俯视）。+1 = 逆时针（与规范 FLU 的 +yaw 同向），-1 = 顺时针。
 * 上下桨共轴反转，所以上桨旋向恒为它的相反数，不单独设常量。
 *
 * ⚠ 本值是**反推**的，不是量出来的，作者已知悉并同意暂用（2026-09-07）。
 *
 * 推理链（三步，任一步被推翻则本值作废）：
 *   1. 作者实测：偏航角速度环 Kp 加大到一定程度会**抖振**而不是一路发散。
 *      抖振是负反馈失稳的表现，发散才是正反馈，所以整条
 *      gyro_z → 力矩 → 差动推力 → 真实机体力矩 的符号是自洽的。
 *   2. 于是 Mz > 0 必须真的产生 +yaw（机头左转 = 俯视逆时针）。
 *   3. 桨对机体的反作用力矩与自身旋向相反；分配式让 Mz > 0 时**下桨**推力
 *      增大，要它给出逆时针反作用，下桨自身就必须是顺时针 → -1。
 *
 * 待办：拆桨直接看一眼桨面/桨型号（或核对电机线序）即可证实或推翻，届时把
 * 本注释改成实测出处。若结论相反，改这一个常量即可——偏航极性由它推导
 * （DRV_COAX_CTRL_YAW_TORQUE_POLARITY），不要去翻分配式或遥控映射。
 */
#define DRV_AIRFRAME_LOWER_ROTOR_SPIN_SENSE       (-1.0f)

/* Estimated moments of inertia about CG (kg*m^2) */
#define DRV_AIRFRAME_IXX_KGM2                      0.051f
#define DRV_AIRFRAME_IYY_KGM2                      0.051f
/*
 * I_zz：作者裁定改为 0.005（2026-09-07）。⚠ 仍是**估计值，未实测**——它替换的
 * 是另一个估计值，不是把估计换成了测量。
 *
 * 为什么原值 0.00035 站不住：它出自
 * doc/history/identification-and-tether-geometry-2026-07-25.md 的组件形状粗估
 * （文档自己标注"量级估计，需校核"），而**那份估计用的整机质量是 0.7546 kg，
 * 代码现在是 1.3670 kg**。飞机重了将近一倍，I_xx/I_yy 被更新过（文档 0.019 →
 * 代码 0.051），I_zz 却一直没动。物理上也说不通：回转半径
 * sqrt(0.00035/1.367) = 16 mm，而光流传感器就装在离重心 200 mm 处
 * （STABILIZER_FLOW_SENSOR_OFFSET_X_M），那里哪怕 50 g 单项就有 2e-3 kg·m²。
 * 0.005 对应回转半径 60 mm，与实际外形量级相符。
 *
 * 影响范围很窄，别误会它能让飞机变有劲：偏航前馈项恒为 0
 * （app_stabilizer.c 把 yaw_accel_rad_s2 写死 0），偏航的陀螺耦合项用的是
 * (I_yy - I_xx) 而两者相等，所以 **I_zz 根本不进偏航控制律**。它只影响
 *   (1) 默认增益，(2) coax.yaw_angle_kp / coax.yaw_rate_kd 两个派生 UI 量的
 * 单位换算。改它是把刻度改准（派生量因此才真的等于 ω_n² 和内环带宽），
 * 不是改物理。
 *
 * 待办：用双线摆实测，或用一次系留纯偏航台阶把 k/I_zz 的比值定死。
 */
#define DRV_AIRFRAME_IZZ_KGM2                      0.005f

#endif /* DRV_AIRFRAME_MODEL_H */
