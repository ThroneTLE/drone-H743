"""机体模型的上位机侧描述：字段分层、单位，以及派生值的预览计算。

飞控是权威。这里只做两件本地的事：

1. **告诉界面每个字段该怎么摆** —— 哪些是拿秤和尺量得到的（基础层，直接可改），
   哪些是估算/辨识出来的（高级层，改之前要二次确认）。这个划分不是美观问题：
   惯量**不是量出来的**，把它和"电池多重"摆在一起会让人以为同样可信，随手就改了。

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

import math
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
    AirframeField("servo1_axis_z_m", "舵机 1 转轴 z（横滚）", "m", "几何", "basic",
                  note="横滚倾转舵机（1 号 / alpha / PWM ch1）转轴的高度，原点 = 板中心，"
                       "z+ 向上，在板下方填负数。它减去整机重心就是横滚倾转力矩的力臂 r_z，"
                       "**决定力矩的大小和方向**；为 0（没填）、离重心不到 1 cm、"
                       "或与推力点不在重心同侧时禁止解锁"),
    AirframeField("servo2_axis_z_m", "舵机 2 转轴 z（俯仰）", "m", "几何", "basic",
                  note="俯仰倾转舵机（2 号 / beta / PWM ch2）转轴的高度，原点 = 板中心，"
                       "z+ 向上，在板下方填负数。它减去整机重心就是俯仰倾转力矩的力臂 r_z，"
                       "**决定力矩的大小和方向**；为 0（没填）、离重心不到 1 cm、"
                       "或与推力点不在重心同侧时禁止解锁"),
    AirframeField("thrust_point_z_m", "推力作用点 z", "m", "几何", "basic",
                  note="双桨中点。推力作用线穿过舵机转轴，所以它**不决定**倾转力矩；"
                       "飞控用它核对转轴高度的正负——两者须在重心同侧，否则多半是某个符号填反了"),
    AirframeField("imu_z_m", "IMU 安装 z", "m", "几何", "basic"),
    AirframeField("prop_plane_d_m", "桨盘间距", "m", "几何", "basic"),
    AirframeField("roll_axis_to_prop_plane_m", "横滚轴到桨盘", "m", "几何", "basic"),
    AirframeField("pitch_axis_to_prop_plane_m", "俯仰轴到桨盘", "m", "几何", "basic"),
    AirframeField("tether_attach_z_m", "系留挂点 z", "m", "几何", "basic"),
    AirframeField("tether_rope_m", "系留绳长", "m", "几何", "basic"),
)

_THRUST_FIELDS = (
    AirframeField("max_total_thrust_g", "双桨最大总推力", "g", "推力", "basic",
                  note="拉力计满油门实测，**双桨合计**（不是单桨）"),
)

# ── 高级层：估算 / 辨识 / 反推得来，改之前必须知道自己在改什么 ─────────────
# 2026-09-27：俯仰/横滚"有效力臂"两项退役。它们是旧机体（1.367 kg）上辨识出的
# 0.145 m，再乘 0.569/0.581 的经验系数；机体换了没人更新，倾转权限被高估约 2.3 倍。
# 力臂现在由舵机转轴高度减重心算出（见上面两个转轴字段与 tilt_axis_to_cg_z），
# 飞控参数表里已经没有这两个名字，这里也不再给输入框。
_ADVANCED_FIELDS = (
    AirframeField("ixx_kgm2", "I_xx", "kg·m²", "转动惯量（估计值）", "advanced",
                  note="**估计值，未实测**。可用双线摆实测替换"),
    AirframeField("iyy_kgm2", "I_yy", "kg·m²", "转动惯量（估计值）", "advanced",
                  note="**估计值，未实测**。可用双线摆实测替换"),
    AirframeField("izz_kgm2", "I_zz", "kg·m²", "转动惯量（估计值）", "advanced",
                  note="**估计值，未实测**。默认偏航增益按它缩放"),
    AirframeField("gravity_m_s2", "重力加速度", "m/s²", "环境", "advanced"),
    AirframeField("servo_deg_per_us", "舵机角度/脉宽", "deg/us", "执行器标度", "advanced"),
)

# ── 派生层：由输入算出 ────────────────────────────────────────────────────
_DERIVED_FIELDS = (
    AirframeField("mass_kg", "整机质量", "kg", "派生", "basic", derived=True),
    AirframeField("cg_z_m", "整机重心 z", "m", "派生", "basic", derived=True),
    AirframeField("weight_n", "重量", "N", "派生", "basic", derived=True),
    AirframeField("thrust_point_to_cg_z_m", "推力点到重心", "m", "派生", "basic",
                  derived=True, note="转轴方向的交叉核对：须与两个转轴的 r_z 同号；为零禁止解锁"),
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
    "max_total_force_n", "servo1_axis_z_m", "servo2_axis_z_m",
    "thrust_point_to_cg_z_m",
)

# 固件 DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M 的镜像：转轴离重心不到它就禁止解锁。
TILT_AXIS_MIN_LEVER_M = 0.01

# 倾转转轴 → 它管的那个轴。顺序与固件复合判据一致（先横滚后俯仰）。
TILT_AXES = (
    ("roll", "servo1_axis_z_m"),
    ("pitch", "servo2_axis_z_m"),
)

# 复合判据的原因词（固件 FirstInvalidName 返回 "<字段>:<原因>"）→ 界面说法。
INVALID_REASON_TEXT = {
    "near": "转轴离重心不到 1 cm，倾转几乎产生不了力矩",
    "sign": "转轴与推力点不在重心同侧，多半是某一个的符号填反了",
}


def tilt_axis_to_cg_z(values: dict[str, float]) -> dict[str, float | None]:
    """**上位机**按"舵机转轴 z − 重心 z"算出的每轴倾转力臂 r_z（负 = 转轴在重心下方）。

    这不是飞控字段：飞控在控制律里现算同一个减法
    （DRV_Airframe_Roll/PitchTiltAxisToCgZ），但不把结果存成参数——机体模型是
    Flash ABI，加不了字段。这里只做预览；重心取 values 里的 cg_z_m，调用方应当
    传**飞控派生出来的那个重心**，界面上也要这么标注。缺数时对应轴给 None，
    不编一个 0 出来冒充"转轴正好在重心上"。
    """
    cg = values.get("cg_z_m")
    out: dict[str, float | None] = {}
    for axis, key in TILT_AXES:
        axis_z = values.get(key)
        out[axis] = (None if (axis_z is None or cg is None)
                     else float(axis_z) - float(cg))
    return out
# 旋向 2026-09-13 从这里搬去了「桨叶与电机方向」页（PROPCAL）：它是量出来的
# 接线事实，不是机体尺寸。那道解锁闸门没有消失，只是换了地方——固件侧是
# DRV_PropMap_IsCalibrated()，解锁横幅里的 propcal 一项就是它。


def first_missing(values: dict[str, float]) -> str | None:
    """返回第一个不合格项（不带前缀），全合格返回 None。

    与固件 DRV_Airframe_FirstInvalidName() 同序同名：先逐项非零，再按轴查倾转
    转轴的两条复合判据，复合项返回 "servo1_axis_z_m:near" 这种"字段:原因"形式。
    重心用 values 里的 cg_z_m（飞控派生值）；没有时按部件表推算。
    """
    for key in REQUIRED_NONZERO:
        value = values.get(key)
        if value is None:
            return key
        # 与固件同一条判据：`!(v > 0 || v < 0)` 对 NaN 与 ±0 都为真；±Inf 另由 isfinite 拦下。
        if not (value > 0.0 or value < 0.0) or not math.isfinite(value):
            return key

    if values.get("cg_z_m") is None:
        values = {**values, "cg_z_m": compute_derived(values)["cg_z_m"]}
    thrust_above = values["thrust_point_to_cg_z_m"] > 0.0
    levers = tilt_axis_to_cg_z(values)
    for axis, key in TILT_AXES:
        r_z = levers[axis]
        # 同样写成"不满足 |r| >= 下限"，NaN 一并判不合格。
        if r_z is None or not math.isfinite(r_z) or not (r_z >= TILT_AXIS_MIN_LEVER_M
                                                     or r_z <= -TILT_AXIS_MIN_LEVER_M):
            return f"{key}:near"
        if (r_z > 0.0) != thrust_above:
            return f"{key}:sign"
    return None


def describe_invalid(name: str) -> str:
    """把 ARM 报文 / first_missing 给出的不合格项翻成一句人话。

    接受带或不带 `airframe.` 前缀的名字。单字段项说"缺 <名字>"；复合项说清
    是哪个字段、为什么。
    """
    field, _, reason = name.partition(":")
    if not reason:
        return f"缺 {name}"
    return f"{field}：{INVALID_REASON_TEXT.get(reason, reason)}"


__all__ = [
    "AIRFRAME_FIELDS",
    "AIRFRAME_FIELD_BY_NAME",
    "AirframeField",
    "DERIVED_AUTO_FIELD",
    "DERIVED_FIELDS",
    "INPUT_FIELDS",
    "INVALID_REASON_TEXT",
    "REQUIRED_NONZERO",
    "TILT_AXES",
    "TILT_AXIS_MIN_LEVER_M",
    "compute_derived",
    "describe_invalid",
    "first_missing",
    "tilt_axis_to_cg_z",
]
