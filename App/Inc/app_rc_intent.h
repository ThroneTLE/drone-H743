#ifndef APP_RC_INTENT_H
#define APP_RC_INTENT_H

/*
 * 摇杆意图 → 规范 FLU 的唯一 Adapter。
 *
 * 这个模块存在的理由：整条链路上只允许有一个地方知道"飞手往哪边推"。
 * 它的每一条符号都不是经验值，而是由下面两个外部事实推出来的——
 *
 *   1) 摇杆侧的正方向来自**上位机标定向导**，不是代码约定。
 *      tools/panel_lib/pages/rc_wizard.py 的 RC_WIZARD_STEPS 把
 *      「前推 / 右打 / 右转」三个物理动作分别记为 pitch/roll/yaw 的 +1。
 *      APP_RcConfig 用这份标定吸收发射机差异，所以到了本模块入口，
 *      norm > 0 恒等于「前 / 右 / 右转」，与是哪台发射机无关。
 *   2) 机体侧的正方向来自 Driver/Inc/drv_frame_contract.h：
 *      +X 前、+Y 左、+Z 上，+roll 右翼下沉，+pitch 机头下俯，+yaw 机头左转。
 *
 * 两条一交叉，五个函数的符号就唯一确定了，没有可调空间（逐条推导写在
 * app_rc_intent.c 的函数体上）。换飞机、换发射机、摇杆接反，唯一该动的是
 * 上位机标定；本文件与下游控制器、EKF、分配器都不该再出现摇杆符号。
 */
float APP_RcIntent_ForwardVelocity(float pitch_norm, float limit_m_s);
float APP_RcIntent_LeftVelocity(float roll_norm, float limit_m_s);
float APP_RcIntent_TargetPitch(float pitch_norm, float limit_rad);
float APP_RcIntent_TargetRoll(float roll_norm, float limit_rad);
float APP_RcIntent_YawRateLeft(float yaw_norm, float limit_rad_s);

#endif
