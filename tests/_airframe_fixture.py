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
"""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

AIRFRAME_SOURCE = ROOT / "Driver" / "Src" / "drv_airframe_params.c"


# ASCII only: several harnesses write their C file with encoding="ascii".
AIRFRAME_FIXTURE_C = r"""
#include "drv_airframe_params.h"
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
    airframe.pitch_thrust_lever_arm_m   = 0.1450f;
    airframe.roll_thrust_lever_arm_m    = 0.1450f;
    airframe.servo1_axis_z_m            = -0.161f;
    airframe.servo2_axis_z_m            = -0.215f;
    airframe.thrust_point_z_m           = -0.2955f;
    airframe.tether_attach_z_m          = 0.1563f;
    airframe.tether_rope_m              = 0.6400f;

    airframe.ixx_kgm2                   = 0.051f;
    airframe.iyy_kgm2                   = 0.051f;
    airframe.izz_kgm2                   = 0.005f;
    airframe.lower_rotor_spin_sense     = -1.0f;
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
}
"""


__all__ = ["AIRFRAME_FIXTURE_C", "AIRFRAME_SOURCE", "ROOT"]
