"""宿主测试共用的参考机体。

为什么需要它：2026-09-11 起机体模型的**唯一来源是 Flash**，固件里一个默认值
都没有（Driver/Inc/drv_airframe_params.h）。好处是不会再有两份互相打架的机体
数据；代价是宿主 harness 里的控制律不再自带质量/惯量/力臂——不先装一组，
DRV_COAX_CTRL_Run() 算出来的每个力矩都是 0。所以每个跑真控制律的测试都必须
先说清"我在测哪架飞机"。

这组数值**逐位复刻改造前 drv_airframe_model.h 的内容，包括那处矛盾**：
四个部件质量加起来是 754.6 g，而整机质量写的是 1.3670 kg；重心 −0.0946 按前者
算，重量 13.41 N 按后者算。所以这里用手动派生档（derived_auto = 0），把派生值
原样钉住。

这不是在维护那个矛盾，而是为了让"机体模型改成运行时取值"这次改动**在测试里
逐位不改变行为**——否则所有数值断言都会同时动，改动对不对就没法判断了。
真机的正确数值要拿秤重新量，从上位机写进 Flash；那件事该由实测证据推动，
不该由一次重构顺手决定。

想测"自动派生算得对不对"的，见 tests/test_airframe_params.py 里那份
fill_repo_measurements——它故意开着自动档，职责不同，别把两者合并。

2026-09-27 倾转力矩模型改准：力臂改为几何量"重心 z − 舵机转轴 z"，经验系数
EFFECTIVENESS 删除，默认横滚/俯仰增益按 2026-09-27 当晚板上几何同比缩小。本夹具的
两个舵机转轴高度因此**不是**旧头文件的 −0.161/−0.215，而是反推出来的值：让
重心 −0.0946 减去它恰好等于退役的有效力臂 0.145×0.581 / 0.145×0.569（俯仰逐位
相等，横滚差 1 个 float ulp，见 C 注释）。这样倾转→力矩这一段在所有 harness 里
与改动前一致；没有任何一条数值断言因此移动（跑真控制律的 harness 用的是默认
增益时，它们只断言方向与阈值，缩小后的默认增益仍然满足）。
"""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

AIRFRAME_SOURCE = ROOT / "Driver" / "Src" / "drv_airframe_params.c"
# 2026-09-13：旋向从机体模型搬到了桨叶接线标定。任何编译 drv_coax_ctrl.c 的
# harness 都必须把这一份也编进去——否则偏航极性恒为 0（未标定），偏航相关的
# 断言会以"数值全是 0"的形态失败，而原因看起来像控制律坏了。
PROP_MAP_SOURCE = ROOT / "Driver" / "Src" / "drv_prop_map.c"


# ASCII only: several harnesses write their C file with encoding="ascii".
AIRFRAME_FIXTURE_C = r"""
#include "drv_airframe_params.h"
#include "drv_prop_map.h"
#include <string.h>

/*
 * Test fixture, NOT this aircraft's correct model -- see the module docstring
 * in tests/_airframe_fixture.py.  Values reproduce the pre-2026-09-11
 * compile-time constants bit for bit, contradiction included, so that moving
 * the airframe model to runtime changes no numeric assertion anywhere.
 */
static void airframe_load_reference(void)
{
    DRV_Airframe_Params airframe;

    memset(&airframe, 0, sizeof(airframe));

    airframe.board_mass_g               = 75.0f;
    airframe.battery_mass_g             = 232.0f;
    airframe.base_mass_g                = 99.0f;
    airframe.servo_motor_mass_g         = 348.6f;
    airframe.board_cg_z_m               = 0.0f;
    airframe.battery_cg_z_m             = 0.109f;
    airframe.base_cg_z_m                = -0.117f;
    airframe.servo_motor_cg_z_m         = -0.244f;

    airframe.imu_z_m                    = 0.0f;
    airframe.prop_plane_d_m             = 0.2500f;
    airframe.roll_axis_to_prop_plane_m  = 0.1450f;
    airframe.pitch_axis_to_prop_plane_m = 0.1050f;
    /*
     * 2026-09-27: the tilt lever is now the signed geometry cg_z - servoN_axis_z
     * (the retired pitch/roll_thrust_lever_arm_m inputs are no longer read).
     * These two axis heights are NOT the old header's -0.161/-0.215: they are
     * chosen so that, with this fixture's cg_z_m = -0.0946, the geometric lever
     * reproduces the retired effective lever 0.145 * EFFECTIVENESS:
     *   pitch: -0.0946 - (-0.177105) = 0.145 * 0.569  (bit-exact in float)
     *   roll : -0.0946 - (-0.178845) = 0.145 * 0.581  (+1 float ulp; no float
     *          value makes this subtraction land exactly on it)
     * so every harness sees the same tilt->moment map as before the fix.
     */
    airframe.servo1_axis_z_m            = -0.178845f;
    airframe.servo2_axis_z_m            = -0.177105f;
    airframe.thrust_point_z_m           = -0.2955f;
    airframe.tether_attach_z_m          = 0.1563f;
    airframe.tether_rope_m              = 0.6400f;

    airframe.ixx_kgm2                   = 0.051f;
    airframe.iyy_kgm2                   = 0.051f;
    airframe.izz_kgm2                   = 0.005f;
    airframe.gravity_m_s2               = 9.81f;
    airframe.max_total_thrust_g         = 1595.342f;
    airframe.servo_deg_per_us           = 0.090f;

    /* Manual derived mode: literals from the old header, not recomputed. */
    airframe.derived_auto               = 0.0f;
    airframe.mass_kg                    = 1.3670f;
    airframe.cg_z_m                     = -0.0946f;
    airframe.weight_n                   = 13.410270f;
    airframe.thrust_point_to_cg_z_m     = -0.2955f - (-0.0946f);
    airframe.tether_attach_to_cg_m      = 0.2509f;
    airframe.tether_rod_to_cg_m         = 0.8909f;
    airframe.max_total_force_n          = 15.644959f;
    airframe.hover_thrust_percent       = 85.716236f;
    airframe.servo_us_per_deg           = 11.111111f;

    DRV_Airframe_SetParams(&airframe);

    /*
     * Rotor wiring calibration.  Until 2026-09-13 this lived in the airframe
     * model as lower_rotor_spin_sense = -1 and the ESC channel order was
     * hard-wired (channel 1 = upper).  Reproducing exactly that here keeps the
     * yaw polarity at +1 and every numeric assertion bit-identical: the move
     * changed where the fact comes from, not what the fact is.
     */
    {
        DRV_PropMap prop;

        DRV_PropMap_Defaults(&prop);
        prop.channel[0].role = (uint8_t)DRV_PROP_ROLE_UPPER;
        prop.channel[0].spin_sense = DRV_PROP_SPIN_CCW;
        prop.channel[1].role = (uint8_t)DRV_PROP_ROLE_LOWER;
        prop.channel[1].spin_sense = DRV_PROP_SPIN_CW;
        prop.calibrated = 1U;
        DRV_PropMap_PublishActive(&prop);
    }
}
"""


__all__ = [
    "AIRFRAME_FIXTURE_C",
    "AIRFRAME_SOURCE",
    "PROP_MAP_SOURCE",
    "ROOT",
]
