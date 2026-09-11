"""机体模型的上位机侧描述：字段分层、单位，以及派生值的预览计算。

飞控是权威。这里只做两件本地的事：

1. **告诉界面每个字段该怎么摆** —— 哪些是拿秤和尺量得到的（基础层，直接可改），
   哪些是估算/辨识出来的（高级层，改之前要二次确认）。这个划分不是美观问题：
   惯量和下桨旋向都**不是量出来的**，把它们和"电池多重"摆在一起会让人以为
   同样可信，随手就改了。

2. **算"若切到自动会是多少"** —— 只在手动派生档下用得上。手动档存在的理由是
   有些量（整机惯量）可能来自双线摆实测而不是部件表推算；代价是它可能和部件表
   悄悄对不上。仓库上一版正是这么坏的：四个部件质量加起来 754.6 g，而整机质量
   写的是 1.3670 kg，两套数各喂各的公式，谁也没报错。所以手动档必须把
   "推算值是多少"并排摆出来，差多少一眼可见。

公式与 Driver/Src/drv_airframe_params.c 的 DRV_Airframe_ComputeDerived() 一一对应，
**tests/test_airframe_page.py 会编译那份 C 源码逐项比对**——这是本模块唯一被允许
存在的理由：预览必须和飞控算的是同一件事，否则它只是又一个会骗人的数字。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AirframeField:
    """一个机体模型字段在界面上的全部元信息。"""

    key: str            # 不带前缀的字段名，与固件结构体成员同名
    label: str          # 中文标签
    unit: str
    group: str          # 分组标题
    tier: str           # "basic" = 可手测量，"advanced" = 估算/辨识，需二次确认
    derived: bool = False
    note: str = ""      # 一句话说明它从哪来；空字符串表示"顾名思义"

    @property
    def name(self) -> str:
        return f"airframe.{self.key}"


# ── 基础层：拿秤和尺量得到 ────────────────────────────────────────────────
_MASS_FIELDS = (
    AirframeField("board_mass_g", "飞控板 + 线束", "g", "部件质量", "basic"),
    AirframeField("battery_mass_g", "电池", "g", "部件质量", "basic"),
    AirframeField("base_mass_g", "底座 / 机架", "g", "部件质量", "basic"),
    AirframeField("servo_motor_mass_g", "舵机 + 电机 + 桨", "g", "部件质量", "basic"),
)

_CG_FIELDS = (
    AirframeField("board_cg_z_m", "飞控板重心 z", "m", "部件重心（原点 = 板中心，z+ 向上）", "basic"),
    AirframeField("battery_cg_z_m", "电池重心 z", "m", "部件重心（原点 = 板中心，z+ 向上）", "basic"),
    AirframeField("base_cg_z_m", "底座重心 z", "m", "部件重心（原点 = 板中心，z+ 向上）", "basic"),
    AirframeField("servo_motor_cg_z_m", "舵机电机组重心 z", "m", "部件重心（原点 = 板中心，z+ 向上）", "basic"),
)

_GEOMETRY_FIELDS = (
    AirframeField("thrust_point_z_m", "推力作用点 z", "m", "几何", "basic",
                  note="双桨中点。它减去整机重心就是倾转力矩的力臂 r_z，"
                       "**r_z 的正负决定俯仰/横滚力矩的方向**，量错会让姿态环变成正反馈"),
    AirframeField("imu_z_m", "IMU 安装 z", "m", "几何", "basic"),
    AirframeField("prop_plane_d_m", "桨盘间距", "m", "几何", "basic"),
    AirframeField("roll_axis_to_prop_plane_m", "横滚轴到桨盘", "m", "几何", "basic"),
    AirframeField("pitch_axis_to_prop_plane_m", "俯仰轴到桨盘", "m", "几何", "basic"),
    AirframeField("servo1_axis_z_m", "舵机 1 转轴 z", "m", "几何", "basic"),
    AirframeField("servo2_axis_z_m", "舵机 2 转轴 z", "m", "几何", "basic"),
    AirframeField("tether_attach_z_m", "系留挂点 z", "m", "几何", "basic"),
    AirframeField("tether_rope_m", "系留绳长", "m", "几何", "basic"),
)

_THRUST_FIELDS = (
    AirframeField("max_total_thrust_g", "双桨最大总推力", "g", "推力", "basic",
                  note="拉力计满油门实测，**双桨合计**（不是单桨）"),
)

# ── 高级层：估算 / 辨识 / 反推得来，改之前必须知道自己在改什么 ─────────────
_ADVANCED_FIELDS = (
    AirframeField("pitch_thrust_lever_arm_m", "俯仰有效力臂", "m", "控制有效力臂", "advanced",
                  note="来自 2026-07-25 系统辨识，不是卷尺量的几何距离"),
    AirframeField("roll_thrust_lever_arm_m", "横滚有效力臂", "m", "控制有效力臂", "advanced",
                  note="来自 2026-07-25 系统辨识，不是卷尺量的几何距离"),
    AirframeField("ixx_kgm2", "I_xx", "kg·m²", "转动惯量（估计值）", "advanced",
                  note="**估计值，未实测**。可用双线摆实测替换"),
    AirframeField("iyy_kgm2", "I_yy", "kg·m²", "转动惯量（估计值）", "advanced",
                  note="**估计值，未实测**。可用双线摆实测替换"),
    AirframeField("izz_kgm2", "I_zz", "kg·m²", "转动惯量（估计值）", "advanced",
                  note="**估计值，未实测**。默认偏航增益按它缩放"),
    AirframeField("lower_rotor_spin_sense", "下桨旋向（俯视）", "+1 逆 / -1 顺", "旋向", "advanced",
                  note="⚠ **反推值，不是实测**：由「偏航角速度环高增益抖振而非发散」"
                       "推出。拆桨看一眼桨面即可证实或推翻。它单独决定偏航力矩极性"),
    AirframeField("gravity_m_s2", "重力加速度", "m/s²", "环境", "advanced"),
    AirframeField("servo_deg_per_us", "舵机角度/脉宽", "deg/us", "执行器标度", "advanced"),
)

# ── 派生层：由输入算出 ────────────────────────────────────────────────────
_DERIVED_FIELDS = (
    AirframeField("mass_kg", "整机质量", "kg", "派生", "basic", derived=True),
    AirframeField("cg_z_m", "整机重心 z", "m", "派生", "basic", derived=True),
    AirframeField("weight_n", "重量", "N", "派生", "basic", derived=True),
    AirframeField("thrust_point_to_cg_z_m", "推力点到重心 r_z", "m", "派生", "basic",
                  derived=True, note="决定倾转力矩的符号；为零则极性无定义，禁止解锁"),
    AirframeField("max_total_force_n", "最大总推力", "N", "派生", "basic", derived=True),
    AirframeField("hover_thrust_percent", "悬停油门", "%", "派生", "basic", derived=True),
    AirframeField("tether_attach_to_cg_m", "挂点到重心", "m", "派生", "basic", derived=True),
    AirframeField("tether_rod_to_cg_m", "绳+挂点到重心", "m", "派生", "basic", derived=True),
    AirframeField("servo_us_per_deg", "舵机脉宽/角度", "us/deg", "派生", "advanced", derived=True),
)

AIRFRAME_FIELDS: tuple[AirframeField, ...] = (
    _MASS_FIELDS + _CG_FIELDS + _GEOMETRY_FIELDS + _THRUST_FIELDS
    + _ADVANCED_FIELDS + _DERIVED_FIELDS
)

DERIVED_AUTO_FIELD = AirframeField(
    "derived_auto", "派生值自动重算", "1 自动 / 0 手动", "派生", "advanced",
    note="自动档下写派生值会被飞控当场拒绝——那是刻意的：写进去再被下次重算"
         "覆盖，上位机会以为写成功了，下次读回来又变了，找不到原因",
)

AIRFRAME_FIELD_BY_NAME = {field.name: field for field in AIRFRAME_FIELDS}
AIRFRAME_FIELD_BY_NAME[DERIVED_AUTO_FIELD.name] = DERIVED_AUTO_FIELD

INPUT_FIELDS = tuple(f for f in AIRFRAME_FIELDS if not f.derived)
DERIVED_FIELDS = tuple(f for f in AIRFRAME_FIELDS if f.derived)


def compute_derived(values: dict[str, float]) -> dict[str, float]:
    """与 DRV_Airframe_ComputeDerived() 逐项对应的派生值计算。

    `values` 以不带前缀的字段名为键；缺的键按 0 处理，和固件那边"全零 = 还没写过"
    是同一套语义。返回的字典只含派生字段。
    """

    def v(key: str) -> float:
        return float(values.get(key, 0.0) or 0.0)

    total_g = (v("board_mass_g") + v("battery_mass_g")
               + v("base_mass_g") + v("servo_motor_mass_g"))
    mass_kg = total_g / 1000.0

    # 部件表为空时不去算 Σ(m·z)/Σm：0/0 是 NaN，而 NaN 会一路扩散，
    # 最后在某个毫不相干的地方显示成"—"或"nan"，根本追不回这里。
    if total_g > 0.0:
        cg_z_m = ((v("board_mass_g") * v("board_cg_z_m"))
                  + (v("battery_mass_g") * v("battery_cg_z_m"))
                  + (v("base_mass_g") * v("base_cg_z_m"))
                  + (v("servo_motor_mass_g") * v("servo_motor_cg_z_m"))) / total_g
    else:
        cg_z_m = 0.0

    weight_n = mass_kg * v("gravity_m_s2")
    thrust_point_to_cg_z_m = v("thrust_point_z_m") - cg_z_m
    tether_attach_to_cg_m = v("tether_attach_z_m") - cg_z_m
    max_total_force_n = (v("max_total_thrust_g") / 1000.0) * v("gravity_m_s2")
    servo_deg_per_us = v("servo_deg_per_us")

    return {
        "mass_kg": mass_kg,
        "cg_z_m": cg_z_m,
        "weight_n": weight_n,
        "thrust_point_to_cg_z_m": thrust_point_to_cg_z_m,
        "tether_attach_to_cg_m": tether_attach_to_cg_m,
        "tether_rod_to_cg_m": v("tether_rope_m") + tether_attach_to_cg_m,
        "max_total_force_n": max_total_force_n,
        "hover_thrust_percent": (
            (weight_n / max_total_force_n) * 100.0 if max_total_force_n > 0.0 else 0.0
        ),
        "servo_us_per_deg": 1.0 / servo_deg_per_us if servo_deg_per_us != 0.0 else 0.0,
    }


# 固件 airframe_check_nonzero[] 的镜像：这几项为零（或 NaN）就禁止解锁。
# 飞控仍是权威（ARM 报文里的 airframe_missing 才是真话），这里只是让界面
# 在还没连上、或还没轮询到的时候也能把该标红的标红。
REQUIRED_NONZERO = (
    "mass_kg", "ixx_kgm2", "iyy_kgm2", "izz_kgm2", "gravity_m_s2",
    "max_total_force_n", "pitch_thrust_lever_arm_m", "roll_thrust_lever_arm_m",
    "lower_rotor_spin_sense", "thrust_point_to_cg_z_m",
)


def first_missing(values: dict[str, float]) -> str | None:
    """返回第一个不合格字段（不带前缀），全合格返回 None。"""
    for key in REQUIRED_NONZERO:
        value = values.get(key)
        if value is None:
            return key
        # 与固件同一条判据：`!(v > 0 || v < 0)` 对 NaN 与 ±0 都为真。
        if not (value > 0.0 or value < 0.0):
            return key
    return None


__all__ = [
    "AIRFRAME_FIELDS",
    "AIRFRAME_FIELD_BY_NAME",
    "AirframeField",
    "DERIVED_AUTO_FIELD",
    "DERIVED_FIELDS",
    "INPUT_FIELDS",
    "REQUIRED_NONZERO",
    "compute_derived",
    "first_missing",
]
