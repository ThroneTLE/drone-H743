#include "app_rc_intent.h"

/*
 * 每个函数只有一行，但那一行的符号必须能被推出来。推导格式统一为：
 *   [标定向导的物理动作] + [drv_frame_contract.h 的机体正方向] → 符号
 * 想改这里的任何一个符号，先得推翻上面两条事实之一。
 */

float APP_RcIntent_ForwardVelocity(float pitch_norm, float limit_m_s)
{
    /* 前推(+) 要飞机向前；FLU +X 就是前。同号。 */
    return pitch_norm * limit_m_s;
}

float APP_RcIntent_LeftVelocity(float roll_norm, float limit_m_s)
{
    /* 右打(+) 要飞机向右；FLU +Y 是左，向右即 -Y。取反。 */
    return -roll_norm * limit_m_s;
}

float APP_RcIntent_TargetPitch(float pitch_norm, float limit_rad)
{
    /*
     * 前推(+) 要机头下俯才能前飞；FLU +pitch 的定义就是机头下俯。同号。
     *
     * 这一条与 2026-08-30 之前留在 app_stabilizer.c 的「实机确认 PITCH_SIGN
     * 取 -1」相反，那条台架结论不是错的，只是已经过期：它是在姿态源还按
     * legacy NED 输出（pitch>0 = 机头上仰）时测的。seam 0/1 把姿态源迁到
     * FLU/NWU 之后，pitch 的物理含义整个翻了一次，所以这里必须同号。
     * 该差异属于「换了口径」，不是「换了飞机」，因此不走标定。
     */
    return pitch_norm * limit_rad;
}

float APP_RcIntent_TargetRoll(float roll_norm, float limit_rad)
{
    /* 右打(+) 要右翼下沉才能右飞；FLU +roll 的定义就是右翼下沉。同号。 */
    return roll_norm * limit_rad;
}

float APP_RcIntent_YawRateLeft(float yaw_norm, float limit_rad_s)
{
    /*
     * 右转(+) 要机头向右；FLU +yaw 的定义是机头向左，向右即负。取反。
     *
     * 注意：本函数只保证「摇杆 → 期望偏航角速率」这一段符合契约。偏航的
     * 实际转向还取决于上下桨的旋向与差动推力分配，那是机械事实，不由本
     * 模块也不由标定覆盖，须在拆桨方向验收里单独观察后才能下结论。
     */
    return -yaw_norm * limit_rad_s;
}
