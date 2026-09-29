"""辨识拟合：延迟、二阶+纯延迟单摆模型、以及惯量 / 阻尼 / 力矩模型比例的反演。

老面板那套（`drone_tcp_panel.fit_ident_step`）是阈值穿越法：找到响应越过某个比例
的时刻，然后 `kp = 0.35/|K|` 拍一个增益。它没有回归、没有延迟估计、没有置信度，
而且拟出来的东西根本没接进四环控制器。本文件是它的替代品。

台架几何（全在机体 z 中心线上，FLU）：

* 光杆水平，方位角 ψ；杆在质心**上方** d（d > 0 ⇒ 质心吊在杆下，稳定单摆）。
  d 是**量出来的输入**，不拟合（原因见下）。
* 横滚/俯仰倾转轴在质心上方 h_roll / h_pitch（尺量到飞控板 + 飞控到质心，界面算好传入）。
* 桨盘中点在质心上方 h = `airframe.thrust_point_to_cg_z_m`（现值 −0.2009 m），只用于
  气动阻尼换算。
* 固件逐样本记录的 `torque` 是分配器**按模型算的**绕质心力矩 τ_rec（效率 × 力臂 ×
  查表推力 × sin δ），真实绕质心力矩是 κ·τ_rec，κ 未知。

旋翼绕舵机倾转轴转 δ 时，对任意点 O 的力矩 = T·sin δ·(z_倾转轴 − z_O)，桨盘位置在
推导里消掉——所以力臂的参考点是**倾转轴**，不是桨盘中点。杆轴 n = (cos ψ, sin ψ, 0)，
激励沿 n 分配，于是绕杆力矩是 G·τ_rec，G = κ·s(d)::

    s(d) = cos²ψ·(h_roll − d)/h_roll + sin²ψ·(h_pitch − d)/h_pitch = 1 − d·Σ wᵢ/hᵢ

权重 < 0.02 的轴忽略（不需要它的高度）。沿机体 z 的推力线过杆轴，机体摆动本身不产生
推力力矩。模型（绕光杆轴的一自由度，小角度线性化）::

    I_杆·θ̈ + c·θ̇ + m·g·d·θ = G·τ_rec(t − T)

回归只给得出三个量：G/I_杆、c/I_杆、m·g·d/I_杆。d 和 κ 在里面是混在一起的，
所以 d 必须量出来。d 已知之后，**重力是唯一一个已知的力矩**：m·g·d/I_杆 直接标定
I_杆，与力矩模型无关；再由 G/I_杆 得 G，κ = G/s(d)。线缆、夹持等还有额外回中刚度时，
m·g·d 低估了真实刚度，I_杆、G、κ、k 同比例偏低——挂砝码实测了台架总刚度 K
（`rig_stiffness_n_m_rad`，见 `rig_stiffness.py`）就用 K 代替 m·g·d。

**整定用的结构是 TWD（"尾巴摇狗"）**（三位评审独立复跑两轮实录后一致推荐）：各激励谱线
（2/6/10 Hz）实测增益约平 2 rad/s/N·m、6 Hz 附近相位跳变约 +180°，刚体积分 + 延迟被证伪。
机理是舵机甩动 349 g 倾转组件的反作用力矩——飞行中同样存在，不能滤掉——模型为
刚体单摆 + 二阶舵机 + 反作用力偶 ρ·u'' + 纯延迟（见 `_fit_twd`），拟合频段 12 Hz/4 阶、
起步 0.5 s 之后；碳杆弹性只是约 11–14 Hz 的次要模态。按 4 Hz 刚体结果合成的 kp
（约 0.42–0.49）在 TWD 对象上 10 Hz 增益裕度 ≤ 0 dB，所以刚体读数只留作诊断基线。

刚体积分读数（第四轮，`structure="rigid"`，见 `FIT_BAND_HZ`）：4 Hz（4 阶零相位）以下、
起步 0.5 s 之后的刚体积分 + 延迟，c = 0。6–18 Hz 的成分是台架与执行器的**确定性动态**，
不是噪声；当时把它当台架模态滤掉，评审证实其中 10 Hz 主要是舵机反作用（飞行中也有）。
4 Hz 带内只有一条受激谱线（2 Hz），两轮 I_cg/κ 只差 0.1% 只说明它在 2 Hz 上自洽，
不说明结构对。单轮里 I_杆 与 G 此消彼长（两轮各差约 25%），不确定度门槛因此作用在
整定惯量 I_cg/κ 这个组合量上（两种结构都是）。

* `I_cg = I_杆 − m·d²`（平行轴）是物理惯量；控制器按固件力矩单位出力，所以
  **整定用 I_cg/κ**。
* `T` —— 总延迟：舵机 + 传输 + 控制拍。`drv_servo_actuator_model.h` 里那组
  16/41 ms 是**停电机、±200 µs 大信号、总线舵机**条件下测的，带桨负载下必须重测，
  不得直接引用。
* `c` —— 绕杆阻尼（桨叶气动 + 轴承）。实测杆高时固定 0，旧工况仍拟合。
* 杆近似过质心（d 未给或 |d| < 2 cm）时重力矩太弱，标定不了力矩模型：退回旧工况，
  κ 按 1，d 当残余偏心在 ±5 cm 内拟合。

已知风险写在明处：短激励下阻尼与回中项强相关（两者都表现为"回中"）。
本模块把相关系数一起报出来，超过门限时标成"不可分离"而不是硬给一个数。
"""
from __future__ import annotations

import functools
import math
from dataclasses import dataclass

import numpy as np

try:
    from scipy import signal
    from scipy.linalg import expm
    from scipy.optimize import least_squares
    _HAVE_SCIPY = True
except ImportError:  # pragma: no cover - 环境里没有 scipy 时退化为只做延迟估计
    _HAVE_SCIPY = False

#: 阻尼与回中项的相关系数超过它就判为不可分离。
SEPARABILITY_LIMIT = 0.95

#: "读不出来 / 不适用"统一用这一个 NaN 对象：同样的输入给出的 ModelFit 逐字段相等
#: （NaN 只有是同一个对象时才在元组比较里算相等）。
_NAN = float("nan")

#: 零相位低通的名义截止频率 [Hz]。带桨时陀螺噪声 0.03–0.05 rad/s，响应只有
#: 0.1–0.2 rad/s；台架摆动在 0.5–3 Hz，15 Hz 以上基本只剩噪声。
LOWPASS_HZ = 15.0

#: 实测杆高至少这么大才用重力标定力矩模型；更小时重力矩太弱，退回杆过质心的旧工况。
PIVOT_MEASURED_MIN_M = 0.02

#: 旧工况（杆近似过质心）残余偏心的拟合范围 [m]。
LEGACY_ECCENTRICITY_M = 0.05

#: 杆与推力点之间至少留的距离 [m]：s(d) 在 d = h 处过零，那里杆上力矩消失。
THRUST_POINT_CLEARANCE_M = 0.02

#: 倾转轴在杆轴方向上的权重（cos²ψ / sin²ψ）低于它就忽略，不需要它的高度。
TILT_AXIS_WEIGHT_MIN = 0.02

#: 绕质心回转半径 √(I_cg/m) 的合理范围 [m]，出了这个范围就不像这类机体。
GYRATION_RADIUS_RANGE_M = (0.03, 0.5)

#: I_cg 占 I_杆 的比例低于它时，I_cg 主要由 m·d² 决定，d 的小误差会被成倍放大。
INERTIA_CG_SHARE_MIN = 0.1

#: 推力×舵机模型比例 k = κ/κ_几何 的可信范围。几何（倾转轴高度、固件力臂）已经扣掉，
#: k 里只剩推力查补表与舵机角度标定的误差，两者各自约 ±15%，叠加约 ±30–40%；
#: 越界说明查补表、舵机标定或倾转轴/杆高量错了。
THRUST_SERVO_MODEL_RANGE = (0.7, 1.4)

#: 拿不到固件力臂时只能对 κ 做宽范围检查。
TORQUE_MODEL_SCALE_FALLBACK_RANGE = (0.2, 3.0)

#: κ_几何 落在这个范围之外时，提示固件力矩模型本身偏了多少倍（提示，不拦）。
GEOMETRIC_SCALE_NOTE_RANGE = (0.7, 1.4)

#: 绕质心惯量相对不确定度的上限 [%]。按残差算的协方差只含输出噪声，漏掉了控制拍
#: 抖动带来的力矩边沿时刻误差；多种子合成实测的真实散布约为报出值的 2 倍，
#: 所以门槛取 8%（≈ 真实 15%）。旧工况作 blocker；实测杆高时只作提示（门槛改作用在
#: 整定惯量 I_cg/κ 上，见 `TUNING_INERTIA_UNCERTAINTY_MAX_PCT`）。
INERTIA_UNCERTAINTY_MAX_PCT = 8.0

#: 延迟低于它就不可信：比一个控制拍还短，整定带宽会被推到没有意义的高度。
DELAY_MIN_S = 1e-3

#: 旧工况（15 Hz 全段）拟合优度下限 [%]。
FIT_PERCENT_MIN = 70.0

#: 实测杆高时代价函数的频段：零相位 Butterworth 低通截止 [Hz] 与阶数。
FIT_BAND_HZ = 4.0
FIT_BAND_ORDER = 4

#: 起步瞬态：代价与初值/零偏的最小二乘只看这之后 [s]；仿真仍从 t = 0 起。
BURN_IN_S = 0.5

#: 延迟精修的固定起点 [s]，与同频段方程误差扫出的延迟一起多起点，取代价最低。
DELAY_STARTS_S = (0.04, 0.07, 0.10)

#: 带内拟合优度下限 [%]。
FIT_BAND_PERCENT_MIN = 85.0

#: 整定惯量 I_cg/κ 的相对不确定度上限 [%]。
TUNING_INERTIA_UNCERTAINTY_MAX_PCT = 8.0

#: 实测杆高时可选的模型结构："twd"（默认，整定用）与 "rigid"（4 Hz 刚体积分，只作对照）。
STRUCTURES = ("twd", "rigid")

#: TWD 结构的代价频段：零相位 Butterworth 截止 [Hz] 与阶数。两轮实录跨轮验证里
#: 12 Hz/4 阶与 10 Hz/2 阶参数几乎一致、交叉拟合打平；选 12/4 是因为它保留了决定
#: 增益裕度的 10 Hz 反作用谱线（幅值 81%，10/2 只剩 50%），对 14 Hz 台架模态的衰减相同，
#: 两轮的舵机参数与纯延迟更一致、整定惯量不确定度更低。
TWD_BAND_HZ = 12.0
TWD_BAND_ORDER = 4

#: TWD 带内拟合优度下限 [%]。两轮实录在 12 Hz/4 阶带内：TWD 87.8% / 76.2%，
#: 同一把尺子上刚体积分 70.2% / 64.4%；72% 把两者分开。
TWD_FIT_PERCENT_MIN = 72.0

#: TWD 参数边界：纯延迟 [s]、舵机自然频率 [rad/s]、舵机阻尼比、反作用力偶 ρ [s²]。
TWD_DEAD_TIME_BOUNDS_S = (0.0, 0.15)
SERVO_WN_BOUNDS = (10.0, 150.0)
SERVO_ZETA_BOUNDS = (0.1, 1.5)
REACTION_COUPLE_BOUNDS = (-0.02, 0.05)

#: TWD 精修的起点：(相对刚体延迟提前的量 [s], 舵机自然频率 [rad/s])，取代价最低。
TWD_STARTS = ((0.028, 31.0), (0.040, 45.0), (0.018, 22.0))

#: 与刚体基线无关的补充起点：κ 的名义取值。
TWD_GENERIC_KAPPA_STARTS = (0.5, 1.0)

#: 刚体基线的带内拟合优度低于它时，不拿它当 TWD 初值 [%]。
RIGID_SEED_FIT_MIN = 50.0

#: 等效延迟的参考频率 [Hz]：`delay_s` 报这个频率上"纯延迟 + 舵机"的等效延迟。
EQUIVALENT_DELAY_HZ = 1.0

#: TWD 绕杆阻尼 c 的拟合范围 [N·m·s]。双脉冲激励不到摆的共振，c 落在 0 附近、不影响
#: 其余参数；扫频从摆频附近起步时 c 才辨得出来（2026-09-27 扫频轮：摆的阻尼比约 0.14），
#: 固定为 0 会让无阻尼的共振吃掉整个代价。整定仍按 0。
DAMPING_BOUNDS_N_M_S = (0.0, 0.5)

#: 参数离边界不到区间宽度的这个比例就算"贴边界"。
BOUND_TOLERANCE = 0.01

#: 模型无关频响（整段 ETFE，谱平滑）的平滑宽度 [Hz]、最短记录 [s]、可用的相干下限、
#: 与模型不一致到多少算严重。整段 ETFE 的分辨率 1/T 比 Welch 分段细得多：合成扫频上
#: 摆频误差 < 2%，Welch 4 s 分段在摆频附近偏低 12–16%。
MODEL_FREE_SMOOTH_HZ = 0.25
MODEL_FREE_MIN_DURATION_S = 8.0
MODEL_FREE_COHERENCE_MIN = 0.6
MODEL_FREE_DISAGREEMENT_MAX = 0.30
#: 搜 TWD 零点与摆频的频段 [Hz]。
MODEL_FREE_ZERO_BAND_HZ = (1.2, 8.0)
MODEL_FREE_PENDULUM_BAND_HZ = (0.2, 1.5)

#: 联合拟合里各轮力矩 rms 相差超过这个倍数就提示：舵机回差让小幅值轮的等效增益偏低、
#: 滞后偏大，一组线性参数描述不了两种工况。
AMPLITUDE_MISMATCH_RATIO = 1.5


@dataclass(frozen=True)
class DelayEstimate:
    seconds: float
    #: 峰值互相关系数。太低说明输入输出根本没对上，延迟数字没有意义。
    correlation: float
    #: 分辨率下限：采样周期。报出来免得有人把 2 ms 网格上的数字当成亚毫秒精度。
    resolution_s: float

    @property
    def trustworthy(self) -> bool:
        return abs(self.correlation) >= 0.5


@dataclass(frozen=True)
class ModelFit:
    #: 绕质心的物理惯量 I_cg = I_杆 − m·d² [kg·m²]。
    inertia_kg_m2: float
    #: 绕杆惯量 I_杆（拟合直接得到的量）。
    inertia_rod_kg_m2: float
    #: 杆高 d：杆在质心上方为正。实测输入原样返回；旧工况为拟合出的残余偏心。
    pivot_above_cg_m: float
    #: 旧字段名，数值与 pivot_above_cg_m 相同，留给旧档案与报告。
    eccentricity_m: float
    #: 几何力矩换算 s(d) = Σ wᵢ·(hᵢ − d)/hᵢ（按倾转轴高度）；旧工况无几何信息时为 1。
    torque_scale: float
    #: 绕杆阻尼 c [N·m·s]。
    damping_n_m_s: float
    delay_s: float
    #: 拟合优度 [%]。实测杆高时是带内值（4 Hz 以下、0.5 s 之后；多轮取最差一轮）；
    #: 旧工况是 15 Hz 全段值。
    fit_percent: float
    #: 单摆固有频率 √(m·g·d/I_杆)/2π；d ≤ 0 时为 NaN。
    natural_hz: float
    #: 参数相关系数：实测杆高时是 I_杆 与 G，旧工况是 c 与残余偏心。
    damping_eccentricity_correlation: float
    separable: bool
    samples: int
    #: 力矩模型比例 κ = 真实力矩 / 固件记录力矩；旧工况固定 1。
    torque_model_scale: float
    #: 整定惯量 I_cg/κ（固件力矩单位）。**整定只用这个**。
    tuning_inertia_kg_m2: float
    #: 整定阻尼（固件力矩单位）。实测杆高时为 0（此频段与延迟不可分）；旧工况为 c。
    tuning_damping_n_m_s: float
    #: σ(I_cg)/I_cg [%]，由雅可比协方差 × 残差方差、按低通后有效样本数修正。
    inertia_uncertainty_pct: float
    #: σ(I_cg/κ)/(I_cg/κ) [%]，按组合量带协方差传播。实测杆高时的 σ 门槛作用在它上。
    tuning_inertia_uncertainty_pct: float
    #: 15 Hz 2 阶零相位低通下的拟合优度 [%]，只作诊断（台架 9 Hz 模态使其上限约 70%）。
    fit_percent_15hz: float
    #: 不能拿去整定的理由（中文短句）；空 = 可整定。
    tuning_blockers: tuple[str, ...] = ()
    #: 拟合时才判得出的提示与说明（需要质量、边界、推力点是否给出）。
    notes: tuple[str, ...] = ()
    #: 逐轮拟合优度 [%]（多轮联合拟合时每轮一个）。
    fit_percent_runs: tuple[float, ...] = ()
    #: 模型结构："twd" / "rigid" / "legacy"（杆过质心的旧工况）。
    structure: str = "rigid"
    #: TWD：纯延迟 T [s]（`delay_s` 是 1 Hz 等效延迟，含舵机滞后）。
    dead_time_s: float = float("nan")
    #: TWD：舵机二阶的自然频率 [rad/s] 与阻尼比。
    servo_wn_rad_s: float = float("nan")
    servo_zeta: float = float("nan")
    #: TWD：舵机甩动组件的反作用力偶 ρ [s²]（杆上力矩 = G·u + ρ·u''）。
    reaction_couple_s2: float = float("nan")
    #: TWD 零点 √(G/ρ)/2π（杆上）与 √(κ/ρ)/2π（飞行，纯力偶最坏情况）[Hz]；ρ ≤ 0 时 NaN。
    rig_zero_hz: float = float("nan")
    flight_zero_hz: float = float("nan")
    #: 1 Hz 处"纯延迟 + 舵机"的等效延迟 [s]。
    equivalent_delay_1hz_s: float = float("nan")
    #: 按飞行 TWD 对象回路整形后的增益裕度 [dB]、相位裕度 [°]、主穿越频率 [Hz]。
    gain_margin_db: float = float("nan")
    phase_margin_deg: float = float("nan")
    crossover_hz: float = float("nan")
    #: 4 Hz 刚体积分诊断基线：整定惯量 [kg·m²]、延迟 [s]、带内拟合优度 [%]。不用于整定。
    rigid_tuning_inertia_kg_m2: float = float("nan")
    rigid_delay_s: float = float("nan")
    rigid_fit_percent: float = float("nan")
    #: 模型无关频响（Welch）量到的 TWD 零点与摆频 [Hz]；激励不够宽或记录太短时 NaN。
    measured_zero_hz: float = float("nan")
    measured_pendulum_hz: float = float("nan")
    #: κ_几何：固件力臂 × 实测倾转轴高度预测的 κ；k = κ/κ_几何：推力查补表 × 舵机标定的准确度。
    torque_model_geometric_scale: float = float("nan")
    thrust_servo_model_scale: float = float("nan")
    #: 挂砝码实测的台架刚度 K [N·m/rad]（给了才有，代替 m·g·d）；等效杆高 d_eff = K/(m·g)
    #: 与多出来的刚度 K − m·g·d（线缆等）。没给时都是 NaN。
    rig_stiffness_n_m_rad: float = float("nan")
    effective_pivot_m: float = float("nan")
    extra_stiffness_n_m_rad: float = float("nan")

    @property
    def warnings(self) -> list[str]:
        out = []
        if not (math.isfinite(self.inertia_kg_m2) and self.inertia_kg_m2 > 0.0):
            out.append(
                f"绕质心惯量 I_cg = I_杆 − m·d² = {self.inertia_kg_m2:.4g} kg·m² 不为正："
                "杆高 d 或推力点位置不对，这组结果不能用于整定。")
        out.extend(self.notes)
        return out


# ---------------------------------------------------------------- 预处理


def uniform_signal(t: np.ndarray, x: np.ndarray, fs: float = 250.0
                   ) -> tuple[np.ndarray, np.ndarray]:
    """重采样到等距时间轴。沿用 `tools/attitude_ident_pid.py` 的做法。"""
    t0 = float(np.nanmin(t))
    t1 = float(np.nanmax(t))
    grid = np.arange(math.ceil(t0 * fs) / fs, math.floor(t1 * fs) / fs, 1.0 / fs)
    if grid.size < 2:
        raise ValueError("数据太短，重采样之后不足两点")
    return grid - grid[0], np.interp(grid, t, x)


def differentiate(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    """中心差分。端点用单边差分，不做外推。"""
    return np.gradient(x, t, edge_order=2, axis=0)


def lowpass_zero_phase(x: np.ndarray, fs: float,
                       cutoff_hz: float = LOWPASS_HZ, order: int = 2) -> np.ndarray:
    """Butterworth（默认二阶）正反各滤一遍。

    零相位是硬要求：单向滤波会把群延迟原样加进辨出的 T，而 T 正是这套辨识要
    交付的量。截止频率压在 0.4·fs 以下，低采样率时也不会越过奈奎斯特。
    """
    cutoff = min(float(cutoff_hz), 0.4 * fs)
    sos = _butter_sos(int(order), cutoff / (0.5 * fs))
    return signal.sosfiltfilt(sos, np.asarray(x, dtype=float), axis=0)


@functools.lru_cache(maxsize=64)
def _butter_sos(order: int, normalized_cutoff: float) -> np.ndarray:
    """拟合里同一个滤波器要设计上千次：设计结果缓存起来（只读使用）。"""
    return signal.butter(order, normalized_cutoff, output="sos")


def _lowpass_noise_gain(fs: float, cutoff_hz: float, order: int = 2) -> float:
    """白噪声过 `lowpass_zero_phase` 后的方差比 Σg²，也就是"有效样本数 / 样本数"。

    低通后的残差相邻样本高度相关，直接按 N 个独立样本算协方差会把不确定度
    低估 1/√(Σg²) 倍（15 Hz / 250 Hz 时约 3 倍）。
    """
    impulse = np.zeros(4001)
    impulse[2000] = 1.0
    response = lowpass_zero_phase(impulse, fs, cutoff_hz, order)
    return float(np.dot(response, response))


# ---------------------------------------------------------------- 台架几何


def torque_scale(pivot_above_cg_m: float,
                 thrust_point_to_cg_z_m: float | None) -> float:
    """几何换算 s(d) = (h − d)/h：同一横向推力绕杆与绕质心的力矩之比。未给 h 时取 1。"""
    if thrust_point_to_cg_z_m is None:
        return 1.0
    h = thrust_point_to_cg_z_m
    return (h - pivot_above_cg_m) / h


def valid_height(value) -> float | None:
    """倾转轴/推力点高度：None、0（|h| < 1 mm）或非有限值都当"缺"。"""
    if value is None:
        return None
    value = float(value)
    if not math.isfinite(value) or abs(value) < 1e-3:
        return None
    return value


def tilt_lever_inverse(azimuth_rad: float, roll_pivot_to_cg_z_m: float | None,
                       pitch_pivot_to_cg_z_m: float | None,
                       fallback_height_m: float | None = None
                       ) -> tuple[float, tuple[str, ...]]:
    """Σ wᵢ/hᵢ 与缺了高度的轴名，使 s(d) = 1 − d·Σ wᵢ/hᵢ。

    wᵢ = cos²ψ / sin²ψ；权重 < `TILT_AXIS_WEIGHT_MIN` 的轴忽略，其余权重归一。
    缺高度的轴用 `fallback_height_m`（桨盘中点）兜底，也没有时按 s = 1。
    """
    weights = (("横滚", math.cos(azimuth_rad) ** 2, valid_height(roll_pivot_to_cg_z_m)),
               ("俯仰", math.sin(azimuth_rad) ** 2, valid_height(pitch_pivot_to_cg_z_m)))
    used = [item for item in weights if item[1] >= TILT_AXIS_WEIGHT_MIN]
    total = sum(weight for _name, weight, _height in used)
    fallback = valid_height(fallback_height_m)
    inverse, missing = 0.0, []
    for name, weight, height in used:
        if height is None:
            missing.append(name)
            height = fallback
        if height is not None:
            inverse += weight / total / height
    return inverse, tuple(missing)


def pivot_bounds(thrust_point_to_cg_z_m: float | None) -> tuple[float, float]:
    """旧工况残余偏心的拟合区间 ±5 cm，并让开推力点，保证 s(d) > 0。"""
    low, high = -LEGACY_ECCENTRICITY_M, LEGACY_ECCENTRICITY_M
    h = thrust_point_to_cg_z_m
    if h is not None:
        if h < 0.0:
            low = max(low, h + THRUST_POINT_CLEARANCE_M)
        else:
            high = min(high, h - THRUST_POINT_CLEARANCE_M)
    return low, high


def pivot_from_ratio(ratio: float, thrust_point_to_cg_z_m: float | None,
                     bounds: tuple[float, float] | None = None) -> float:
    """旧工况（κ = 1）下由回归比值 R = a3/(a1·m·g) = d/s 反解 d。

    s = (h − d)/h 代入 R = d/s 得 d = R·h/(h + R)。越过渐近线 R = −h 的比值没有
    有限 d 与之对应，取区间端点，交给输出误差精修。
    """
    low, high = bounds if bounds is not None else pivot_bounds(thrust_point_to_cg_z_m)
    h = thrust_point_to_cg_z_m
    if not math.isfinite(ratio):
        return _clip(0.0, low, high)
    if h is None:
        return _clip(ratio, low, high)
    denominator = h + ratio
    if abs(denominator) < 1e-12:
        return high if ratio > 0.0 else low
    pivot = ratio * h / denominator
    if torque_scale(pivot, h) <= 0.0:
        return high if ratio > 0.0 else low
    return _clip(pivot, low, high)


# ---------------------------------------------------------------- 延迟


def estimate_delay(t: np.ndarray, command: np.ndarray, response: np.ndarray,
                   max_delay_s: float = 0.30) -> DelayEstimate:
    """互相关求时移。

    对已知量（发出去的期望角加速度）和实测量（实际角加速度）做归一化互相关，
    取峰值位置。只在 [0, max_delay_s] 里找：负延迟是因果性不允许的，
    允许它只会在噪声大的时候给出一个"响应早于命令"的漂亮数字。
    """
    if t.size != command.size or t.size != response.size:
        raise ValueError("三条序列长度必须一致")
    dt = float(np.median(np.diff(t)))
    if dt <= 0.0:
        raise ValueError("时间轴必须递增")

    u = command - np.mean(command)
    y = response - np.mean(response)
    norm = float(np.std(u) * np.std(y))
    if norm < 1e-12:
        return DelayEstimate(seconds=float("nan"), correlation=0.0, resolution_s=dt)

    max_lag = min(int(round(max_delay_s / dt)), t.size - 2)
    best_lag, best_corr = 0, -2.0
    for lag in range(0, max_lag + 1):
        if lag == 0:
            a, b = u, y
        else:
            a, b = u[:-lag], y[lag:]
        value = float(np.mean(a * b)) / norm
        if value > best_corr:
            best_corr, best_lag = value, lag
    return DelayEstimate(seconds=best_lag * dt, correlation=best_corr,
                         resolution_s=dt)


def inertia_ratio(command_alpha: np.ndarray, measured_alpha: np.ndarray,
                  delay_samples: int = 0) -> float:
    """I_true / I_est —— 模型反演收敛的核心量。

    前馈按假定惯量 I_est 算出力矩，若真实惯量更大，同样的力矩只能产生更小的角加
    速度，所以 **I_true/I_est = α_期望 / α_实测**。反复几轮之后估计与初值无关地
    收敛，这就是"一套程序适配多种惯量"的机制。

    用最小二乘的斜率而不是逐点求商：逐点求商在 α 过零附近会被噪声放大成天文数字。
    """
    if delay_samples > 0:
        command_alpha = command_alpha[:-delay_samples]
        measured_alpha = measured_alpha[delay_samples:]
    denominator = float(np.dot(measured_alpha, measured_alpha))
    if denominator < 1e-12:
        return float("nan")
    return float(np.dot(command_alpha, measured_alpha)) / denominator


# ---------------------------------------------------------------- 二阶模型


def _sample_period(t: np.ndarray) -> float:
    steps = np.diff(t)
    dt = float(np.median(steps))
    if not dt > 0.0 or float(np.max(np.abs(steps - dt))) > 1e-3 * dt:
        raise ValueError("时间轴必须等距递增；先用 uniform_signal 重采样")
    return dt


def _discrete_rate_model(inertia: float, damping: float, stiffness: float,
                         gain: float, dt: float
                         ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """一阶保持（FOH）离散化，返回 (num, den, Φ, b_next)：输入力矩 → 输出角速度。

    用 FOH 而不是 ZOH：ZOH 把输入当阶梯，等于凭空多出半个采样周期的延迟，会原样
    记进 T。直接写状态空间而不是传递函数：`s/(I s² + c s + k)` 在 k→0 时零极点
    对消，`tf2ss` 在那个点上病态，而旧台架的杆过质心（k=0）正是名义工况。
    """
    a_matrix = np.array([[0.0, 1.0],
                         [-stiffness / inertia, -damping / inertia]])
    augmented = np.zeros((4, 4))
    augmented[:2, :2] = a_matrix * dt
    augmented[1, 2] = gain / inertia * dt
    augmented[2, 3] = 1.0
    transition = expm(augmented)
    phi = transition[:2, :2]
    b_next = transition[:2, 3]            # 乘 u[k+1]
    b_now = transition[:2, 2] - b_next    # 乘 u[k]
    # x' = x − b_next·u 化成标准离散状态空间 (Φ, Φ·b_next + b_now, C, C·b_next)，C = [0, 1]。
    g = phi @ b_next + b_now
    den = np.array([1.0, -float(np.trace(phi)), float(np.linalg.det(phi))])
    num = np.array([0.0, g[1], phi[1, 0] * g[0] - phi[0, 0] * g[1]]) + b_next[1] * den
    return num, den, phi, b_next


def _responses(inertia: float, damping: float, stiffness: float, gain: float,
               delay_s: float, t: np.ndarray, torque: np.ndarray, left: float
               ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(零初值强迫响应, θ₀=1 的自由响应, ω₀=1 的自由响应)，输出都是角速度。

    `left` 是延迟段之前（记录开始前）的输入值。
    """
    dt = _sample_period(t)
    delayed = np.interp(t - delay_s, t, torque, left=left, right=torque[-1])
    num, den, phi, b_next = _discrete_rate_model(inertia, damping, stiffness, gain, dt)
    # 自由响应 y[k] = C·Φᵏ·x₀ 满足同一特征多项式的递推，只需给出前两拍。
    impulse = np.zeros(t.size)
    impulse[0] = 1.0
    free = []
    for column in (0, 1):
        first = 1.0 if column == 1 else 0.0
        second = float(phi[1, column])
        free.append(signal.lfilter([first, second + den[1] * first], den, impulse))
    # lfilter 零初值对应 x' = 0，即 x₀ = b_next·u₀；首个输入不为 0 时（doublet 从斜坡
    # 起步）要把这份假初值的自由响应扣掉，才是真正的零初值响应。
    forced = (signal.lfilter(num, den, delayed)
              - delayed[0] * (b_next[0] * free[0] + b_next[1] * free[1]))
    return forced, free[0], free[1]


def simulate(params: np.ndarray, t: np.ndarray, torque: np.ndarray,
             mass_kg: float, gravity: float, *,
             thrust_point_to_cg_z_m: float | None = None,
             torque_model_scale: float = 1.0) -> np.ndarray:
    """在给定 (I_杆, c, d, T) 下、零初值仿真角速度响应。

    杆上力矩 = κ·s(d)·τ。小角度下 sin θ ≈ θ，模型退化成线性二阶。这个近似在
    ±15° 内误差 < 1.2%，而台架本来就只跑 ±15°。输出取角速度：辨识以陀螺为准
    （角速度是刚体不变量，不受 IMU 偏离转轴的影响）。
    """
    inertia, damping, pivot, delay_s = (float(v) for v in params)
    gain = torque_model_scale * torque_scale(pivot, thrust_point_to_cg_z_m)
    forced, _free_angle, _free_rate = _responses(
        inertia, damping, mass_kg * gravity * pivot, gain, delay_s,
        np.asarray(t, dtype=float), np.asarray(torque, dtype=float), left=0.0)
    return forced


def _equation_error_scan(t: np.ndarray, torque: np.ndarray, rate: np.ndarray,
                         angle: np.ndarray, max_delay_s: float,
                         lowpass_hz: float = LOWPASS_HZ, left: float = 0.0,
                         order: int = 2) -> tuple[float, float, float, float]:
    """粗扫延迟 + 线性回归，给非线性精修一个靠谱的初值。

    在**给定**延迟 T 下，运动方程对参数是线性的::

        α[k] = a1·τ[k−T] − a2·ω[k] − a3·θ[k] + 常数 + 斜率·t
        a1 = G/I_杆，a2 = c/I_杆，a3 = m·g·d/I_杆

    所以每个 T 只要解一次最小二乘，取残差最小的 T。这一步之所以必要，是因为
    输出误差拟合对延迟初值很敏感：从 10 ms 起步去找 30 ms，优化器会先把误差
    塞进阻尼里，最后停在一个"延迟偏大、阻尼偏小"的局部解上。

    陀螺差分会把噪声放大成角加速度的好几倍，所以三路信号先过**同一个**零相位
    低通：线性方程两边同乘一个 LTI 滤波器仍然成立，也不引入时移。
    延迟段之前的力矩按 `left` 补齐（记录开始前激励尚未开始）。

    返回 (delay_s, a1, a2, a3)；回归失败时后三项为 NaN。
    """
    dt = float(np.median(np.diff(t)))
    fs = 1.0 / dt
    torque_f = lowpass_zero_phase(torque, fs, lowpass_hz, order)
    rate_f = lowpass_zero_phase(rate, fs, lowpass_hz, order)
    angle_f = lowpass_zero_phase(angle, fs, lowpass_hz, order)
    alpha = differentiate(t, rate_f)
    # 两端各让出一小段：零相位滤波和差分在端点都不可信。
    edge = min(int(round(0.1 * fs)), t.size // 10)
    keep = slice(edge, t.size - edge)
    ones = np.ones(t.size)
    # 积分得到的转角差一个初值和一条斜坡（陀螺零偏、以及去均值时扣掉的真实净转角）；
    # 常数列和时间列把 a3·(θ₀ + b·t) 吸收掉，否则斜坡会被当成刚度、把延迟扫偏。
    trend = t - float(np.mean(t))
    max_lag = max(1, int(round(max_delay_s / dt)))
    best = None
    for lag in range(0, min(max_lag, t.size - 2) + 1):
        if lag == 0:
            shifted = torque_f
        else:
            shifted = np.concatenate([np.full(lag, left), torque_f[:-lag]])
        design = np.column_stack([shifted, rate_f, angle_f, ones, trend])[keep]
        solution, *_ = np.linalg.lstsq(design, alpha[keep], rcond=None)
        residual = float(np.linalg.norm(design @ solution - alpha[keep]))
        if solution[0] <= 0.0:
            continue  # G/I 必须为正，否则力矩和角加速度反号，物理上不成立
        if best is None or residual < best[0]:
            best = (residual, lag, solution)
    if best is None:
        nan = float("nan")
        return max_delay_s * 0.3, nan, nan, nan
    _residual, lag, solution = best
    return lag * dt, float(solution[0]), -float(solution[1]), -float(solution[2])


@dataclass
class _Record:
    """一轮等距采样，力矩与角速度已去均值。"""
    t: np.ndarray
    torque: np.ndarray
    rate: np.ndarray
    angle: np.ndarray
    #: 记录开始前的力矩（激励尚未开始，固件力矩为 0；去均值坐标下就是 −均值）。
    left: float
    fs: float


def _prepare_record(t, torque_n_m, rate_rad_s, angle_rad=None) -> _Record:
    t = np.asarray(t, dtype=float)
    if t.size < 64:
        raise ValueError("样本太少，拟合没有意义")
    dt = _sample_period(t)
    torque = np.asarray(torque_n_m, dtype=float)
    torque_mean = float(np.mean(torque))
    torque = torque - torque_mean
    rate = np.asarray(rate_rad_s, dtype=float)
    rate = rate - float(np.mean(rate))
    if float(np.std(torque)) < 1e-9:
        raise ValueError("力矩激励幅度太小，这段数据辨不出东西")
    if angle_rad is None:
        angle = np.concatenate([[0.0], np.cumsum((rate[1:] + rate[:-1]) * 0.5 * dt)])
    else:
        angle = np.asarray(angle_rad, dtype=float)
    angle = angle - float(np.mean(angle))
    return _Record(t=t, torque=torque, rate=rate, angle=angle, left=-torque_mean, fs=1.0 / dt)


@dataclass(frozen=True)
class _Geometry:
    #: 桨盘中点高度 h（只用于兜底与阻尼换算）。
    h: float | None
    pivot_input: float | None
    measured: bool
    #: Σ wᵢ/hᵢ，s(d) = 1 − d·lever_inverse。
    lever_inverse: float
    missing_axes: tuple[str, ...]
    #: 固件力臂 × 实测倾转轴高度预测的杆上增益 G_几何；缺数据时 None。
    geometric_gain: float | None = None
    #: 这一趟辨到的轴（"俯仰"、"横滚" 或 "横滚/俯仰"）。
    axis_label: str = "横滚/俯仰"


def _geometry(*, mass_kg: float, inertia_guess: float,
              thrust_point_to_cg_z_m: float | None, pivot_above_cg_m: float | None,
              pivot_guess_m: float | None, azimuth_rad: float | None,
              roll_pivot_to_cg_z_m: float | None,
              pitch_pivot_to_cg_z_m: float | None,
              firmware_tilt_levers_m: tuple[float, float] | None = None) -> _Geometry:
    if not _HAVE_SCIPY:
        raise RuntimeError("二阶+纯延迟拟合需要 scipy")
    if not (math.isfinite(mass_kg) and mass_kg > 0.0):
        raise ValueError("质量必须为正")
    if not (math.isfinite(inertia_guess) and inertia_guess > 0.0):
        raise ValueError("假定惯量必须为正")
    h = thrust_point_to_cg_z_m
    if h is not None:
        h = float(h)
        if not math.isfinite(h) or abs(h) < 1e-3:
            raise ValueError("推力点到质心的垂直距离必须是非零有限值")
    pivot_input = pivot_above_cg_m if pivot_above_cg_m is not None else pivot_guess_m
    if pivot_input is not None:
        pivot_input = float(pivot_input)
        if not math.isfinite(pivot_input):
            raise ValueError("杆高必须是有限值")
    measured = pivot_input is not None and abs(pivot_input) >= PIVOT_MEASURED_MIN_M
    roll_height = valid_height(roll_pivot_to_cg_z_m)
    pitch_height = valid_height(pitch_pivot_to_cg_z_m)
    if azimuth_rad is None:
        if roll_height is not None or pitch_height is not None:
            raise ValueError("给了倾转轴高度就必须给杆轴方位角")
        lever_inverse = 0.0 if h is None else 1.0 / h
        missing_axes: tuple[str, ...] = ("横滚", "俯仰")
    else:
        azimuth_rad = float(azimuth_rad)
        if not math.isfinite(azimuth_rad):
            raise ValueError("杆轴方位角必须是有限值")
        lever_inverse, missing_axes = tilt_lever_inverse(
            azimuth_rad, roll_height, pitch_height, h)
    if measured:
        if azimuth_rad is not None:
            for name, axis_weight, height in (
                    ("横滚", math.cos(azimuth_rad) ** 2, roll_height),
                    ("俯仰", math.sin(azimuth_rad) ** 2, pitch_height)):
                if (axis_weight >= TILT_AXIS_WEIGHT_MIN and height is not None
                        and 1.0 - pivot_input / height <= 0.0):
                    raise ValueError(f"杆高与{name}倾转轴高度矛盾：填的倾转轴在杆的高度上或杆的另一侧，"
                                     "推力绕杆的力矩会为零或变号。倾转轴不是台架的杆，是舵机带电机组"
                                     "摆动的转轴（一般在飞控板下方 0.15～0.25 m）；改好后点「重新分析」")
        if not 1.0 - pivot_input * lever_inverse > 0.05:
            raise ValueError("杆高与倾转轴位置矛盾：倾转轴几乎在杆的高度上或杆的另一侧，"
                             "杆上看不到倾转力矩。倾转轴不是台架的杆，是舵机带电机组摆动的转轴；"
                             "核对后点「重新分析」")
    geometric_gain = None
    axis_label = "横滚/俯仰"
    if azimuth_rad is not None:
        used = [name for name, weight in (("横滚", math.cos(azimuth_rad) ** 2),
                                          ("俯仰", math.sin(azimuth_rad) ** 2))
                if weight >= TILT_AXIS_WEIGHT_MIN]
        axis_label = "/".join(used)
        if measured:
            geometric_gain = firmware_geometric_gain(
                pivot_input, azimuth_rad, roll_height, pitch_height, firmware_tilt_levers_m)
    return _Geometry(h=h, pivot_input=pivot_input, measured=measured,
                     lever_inverse=lever_inverse, missing_axes=missing_axes,
                     geometric_gain=geometric_gain, axis_label=axis_label)


def firmware_geometric_gain(pivot_above_cg_m: float, azimuth_rad: float,
                            roll_pivot_to_cg_z_m: float | None,
                            pitch_pivot_to_cg_z_m: float | None,
                            firmware_tilt_levers_m: tuple[float, float] | None) -> float | None:
    """几何预测的杆上增益 G_几何 = 绕杆力矩 / 固件记录力矩，不经过质心。

    `firmware_tilt_levers_m` 是固件把倾角换成记录力矩用的带符号有效力臂 (横滚, 俯仰)：
    τ_fw = Lᵢ·T·sin δ（见 `_core.firmware_tilt_levers`）。实际绕杆力矩 = (d − hᵢ)·T·sin δ
    （倾转轴在质心下方时 −hᵢ > 0；d − hᵢ 是杆到倾转轴的距离），于是
    G_几何 = Σ wᵢ·(d − hᵢ)/Lᵢ，权重与 `tilt_lever_inverse` 相同；κ_几何 = G_几何/s(d)，
    单轴时就是 −hᵢ/Lᵢ（真实绕质心力臂 / 固件力臂）。所需轴的高度或力臂缺一个就返回 None。
    """
    if firmware_tilt_levers_m is None:
        return None
    roll_lever, pitch_lever = firmware_tilt_levers_m
    axes = ((math.cos(azimuth_rad) ** 2, valid_height(roll_pivot_to_cg_z_m), roll_lever),
            (math.sin(azimuth_rad) ** 2, valid_height(pitch_pivot_to_cg_z_m), pitch_lever))
    used = [axis for axis in axes if axis[0] >= TILT_AXIS_WEIGHT_MIN]
    total = sum(axis[0] for axis in used)
    gain = 0.0
    for weight, height, lever in used:
        if height is None or lever is None:
            return None
        lever = float(lever)
        if not math.isfinite(lever) or abs(lever) < 1e-4:
            return None
        gain += weight / total * (pivot_above_cg_m - height) / lever
    return gain


def _torque_model_checks(kappa: float, gain: float, scale: float, geometry: _Geometry,
                         notes: list[str], gain_sigma_rel: float = 0.0
                         ) -> tuple[list[str], float, float]:
    """κ 的检查：有固件力臂与倾转轴高度时按几何一致性（k = κ/κ_几何），否则宽范围。

    k 与拟合出的 G 同比例，单轮里 G 带着 I_杆–G 此消彼长的不确定度（两轮双脉冲的 k
    相差约 40%，整定惯量却只差 0.1%）；所以只有 k 超出范围超过 2σ(G) 时才拦。
    返回 (blockers, κ_几何, k)；长说明追加进 notes。κ_几何 偏离 1 很多只是提示：
    那是固件力矩模型本身高估/低估了操纵力矩，辨识已经按 κ 折算。
    """
    blockers: list[str] = []
    if geometry.geometric_gain is None:
        low, high = TORQUE_MODEL_SCALE_FALLBACK_RANGE
        notes.append("缺固件倾转力臂或倾转轴高度，κ 只按宽范围 "
                     f"{low:g}–{high:g} 检查（传入 firmware_tilt_levers_m 可做几何一致性核对）。")
        if not low <= kappa <= high:
            message = (f"力矩模型比例 κ = {kappa:.2f} 不在 {low:g}–{high:g}："
                       "力矩模型或杆距测量可能有误，先核对")
            notes.append(message + "。")
            blockers.append(message)
        return blockers, _NAN, _NAN
    geometric_scale = geometry.geometric_gain / scale
    model_scale = gain / geometry.geometric_gain
    if not kappa > 0.0 or not geometric_scale > 0.0:
        message = (f"力矩极性不一致（κ = {kappa:.2f}，几何预测 {geometric_scale:.2f}）："
                   "固件认为的倾转力矩方向与实测相反，先核对倾转轴高度与推力点位置")
        notes.append(message + "。")
        blockers.append(message)
        return blockers, float(geometric_scale), float(model_scale)
    low, high = THRUST_SERVO_MODEL_RANGE
    spread = 2.0 * (gain_sigma_rel if math.isfinite(gain_sigma_rel) else 0.0)
    consistent = model_scale * (1.0 - spread) <= high and model_scale * (1.0 + spread) >= low
    if not consistent:
        message = (f"推力×舵机模型比例 k = κ/κ_几何 = {model_scale:.2f}"
                   f"（±{100.0 * spread / 2.0:.0f}%）不在 {low:g}–{high:g}："
                   "推力查补表、舵机角度标定或倾转轴/杆高测量有误，或小幅值激励落在舵机回差里，先核对")
        notes.append(message + "。")
        blockers.append(message)
    elif not low <= model_scale <= high:
        notes.append(f"推力×舵机模型比例 k = {model_scale:.2f} 略出 {low:g}–{high:g}，"
                     f"但在单轮 G 的不确定度（±{100.0 * spread / 2.0:.0f}%）之内，不拦。")
    note_low, note_high = GEOMETRIC_SCALE_NOTE_RANGE
    if not note_low <= geometric_scale <= note_high:
        word, factor = (("高估", 1.0 / geometric_scale) if geometric_scale < 1.0
                        else ("低估", geometric_scale))
        agreement = "二者一致" if consistent else "二者不一致"
        notes.append(f"固件力矩模型{word}{geometry.axis_label}操纵力矩约 {factor:.1f} 倍"
                     f"（几何预测 κ = {geometric_scale:.3f}，实测 {kappa:.3f}，{agreement}）；"
                     "整定已按 κ 折算，飞行中其它用到力矩模型的地方（前馈、限幅）同样偏差。")
    return blockers, float(geometric_scale), float(model_scale)


# ---------------------------------------------------------------- 模型无关频响


def model_free_response(t: np.ndarray, torque: np.ndarray, rate: np.ndarray):
    """整段 ETFE：H = ⟨Y·U*⟩/⟨|U|²⟩（频域平滑），相干 = |⟨Y·U*⟩|²/(⟨|U|²⟩⟨|Y|²⟩)。

    返回 (频率, H, 相干)；记录短于 `MODEL_FREE_MIN_DURATION_S` 时 None——短记录的
    频率分辨率不够，平滑核里频点太少，相干也就没有意义（4 s 的双脉冲轮都在此列）。
    """
    t = np.asarray(t, dtype=float)
    dt = _sample_period(t)
    if t.size * dt < MODEL_FREE_MIN_DURATION_S:
        return None
    torque = np.asarray(torque, dtype=float) - float(np.mean(torque))
    rate = np.asarray(rate, dtype=float) - float(np.mean(rate))
    freq = np.fft.rfftfreq(t.size, dt)
    spectrum_in = np.fft.rfft(torque)
    spectrum_out = np.fft.rfft(rate)
    width = max(5, int(round(MODEL_FREE_SMOOTH_HZ / (freq[1] - freq[0]))) | 1)
    kernel = np.hanning(width + 2)[1:-1]
    kernel = kernel / kernel.sum()

    def smooth(values: np.ndarray) -> np.ndarray:
        return np.convolve(values, kernel, mode="same")

    power_in = smooth(np.abs(spectrum_in) ** 2)
    power_out = smooth(np.abs(spectrum_out) ** 2)
    cross = smooth(spectrum_out * np.conj(spectrum_in))
    response = cross / np.maximum(power_in, 1e-300)
    coherence = np.abs(cross) ** 2 / np.maximum(power_in * power_out, 1e-300)
    return freq, response, coherence


def _parabolic_peak(freq: np.ndarray, values: np.ndarray, index: int) -> float:
    y0, y1, y2 = values[index - 1:index + 2]
    curvature = y0 - 2.0 * y1 + y2
    offset = 0.5 * (y0 - y2) / curvature if curvature != 0.0 else 0.0
    return float(freq[index] + max(-0.5, min(0.5, offset)) * (freq[1] - freq[0]))


def model_free_features(t: np.ndarray, torque: np.ndarray, rate: np.ndarray
                        ) -> tuple[float, float]:
    """(摆频, TWD 零点) [Hz]：从模型无关频响读，读不出来的给 NaN。

    零点：搜索带内大半频点相干 ≥ 0.6（宽带激励）时取 |H| 最小处；凹口两侧最近的
    相干频点（各 0.5 Hz 以内）之间相位翻转要 > 90°——凹口正中本来就没输出、相干必低。
    摆频：0.2–1.5 Hz 内相干够的局部极大里最高的一个。都用对数幅值抛物线插值。
    """
    estimate = model_free_response(t, torque, rate)
    if estimate is None:
        return _NAN, _NAN
    freq, response, coherence = estimate
    log_mag = np.log(np.maximum(np.abs(response), 1e-300))
    step = freq[1] - freq[0]
    reach = max(1, int(round(0.5 / step)))

    zero = _NAN
    band = np.where((freq >= MODEL_FREE_ZERO_BAND_HZ[0]) & (freq <= MODEL_FREE_ZERO_BAND_HZ[1]))[0]
    if band.size >= 5 and np.mean(coherence[band] >= MODEL_FREE_COHERENCE_MIN) >= 0.6:
        j = int(np.argmin(log_mag[band]))
        if 0 < j < band.size - 1:
            i = band[j]
            below = [k for k in range(i - 1, max(i - reach, 0) - 1, -1)
                     if coherence[k] >= MODEL_FREE_COHERENCE_MIN]
            above = [k for k in range(i + 1, min(i + reach, freq.size - 1) + 1)
                     if coherence[k] >= MODEL_FREE_COHERENCE_MIN]
            if below and above:
                flip = abs(math.degrees(float(np.angle(response[above[0]] / response[below[0]]))))
                if flip > 90.0:
                    zero = _parabolic_peak(freq, log_mag, i)

    # 摆频取带内相干够的局部极大里最高的那个：扫频起点以下常有一段一路抬头的低频响应，
    # 直接取最大值会落在带边。
    pendulum = _NAN
    band = np.where((freq >= MODEL_FREE_PENDULUM_BAND_HZ[0])
                    & (freq <= MODEL_FREE_PENDULUM_BAND_HZ[1]))[0]
    peaks = [i for i in band[1:-1]
             if log_mag[i] > log_mag[i - 1] and log_mag[i] >= log_mag[i + 1]
             and coherence[i] >= MODEL_FREE_COHERENCE_MIN]
    if peaks:
        pendulum = _parabolic_peak(freq, log_mag, max(peaks, key=lambda i: log_mag[i]))
    return pendulum, zero


def _pendulum_stiffness(mass_kg: float, gravity: float, pivot: float,
                        rig_stiffness_n_m_rad: float | None) -> float:
    """单摆回中刚度：给了挂砝码实测的台架刚度就用它，否则 m·g·d（只有重力）。"""
    if rig_stiffness_n_m_rad is None:
        return mass_kg * gravity * pivot
    value = float(rig_stiffness_n_m_rad)
    if not (math.isfinite(value) and value > 0.0):
        raise ValueError("台架刚度必须是正的有限值（N·m/rad）")
    return value


def _stiffness_report(mass_kg: float, gravity: float, pivot: float,
                      rig_stiffness_n_m_rad: float | None, notes: list[str]
                      ) -> tuple[float, float, float]:
    """(K, d_eff, K − m·g·d)；没给实测刚度时三个都是 NaN。说明追加进 notes。

    重力是 I_杆 的唯一标定：代价只认 K/I_杆，所以 K 换了，I_杆、G、κ 与 k 同比例跟着换，
    整定惯量 I_cg/κ = (I_杆 − m·d²)/κ 也随之改变（m·d² 仍按几何杆高）。
    """
    if rig_stiffness_n_m_rad is None:
        return _NAN, _NAN, _NAN
    stiffness = float(rig_stiffness_n_m_rad)
    gravity_part = mass_kg * gravity * pivot
    effective = stiffness / (mass_kg * gravity)
    extra = stiffness - gravity_part
    notes.append(f"台架刚度按挂砝码实测 K = {stiffness:.4f} N·m/rad 代替 m·g·d = {gravity_part:.4f}："
                 f"等效杆高 d_eff = K/(m·g) = {effective:.4f} m（几何 d = {pivot:.4f} m），"
                 f"多出来的刚度 K − m·g·d = {extra:+.4f} N·m/rad（线缆、杆的约束等）；"
                 "I_杆 与 κ 按 K 标定。")
    if extra < 0.0:
        notes.append("实测刚度比只算重力的 m·g·d 还小：核对砝码试验（角度读数、水平距离）或杆高。")
    return stiffness, effective, extra


def _swing_note(records: list[_Record]) -> list[str]:
    swing = 0.0
    for record in records:
        detrended = record.angle - np.polyval(np.polyfit(record.t, record.angle, 1), record.t)
        swing = max(swing, 0.5 * float(np.max(detrended) - np.min(detrended)))
    if swing > math.radians(15.0):
        return [f"摆幅约 ±{math.degrees(swing):.0f}°，超过 ±15°：sin θ ≈ θ 的线性化误差"
                "会被记进延迟和惯量。减小激励幅值重测。"]
    return []


def _uncertainty_pct(covariance: np.ndarray | None, gradient: np.ndarray,
                     value: float) -> float:
    if covariance is None or not (math.isfinite(value) and value > 0.0):
        return float("inf")
    sigma = math.sqrt(max(float(gradient @ covariance @ gradient), 0.0))
    return 100.0 * sigma / value if math.isfinite(sigma) else float("inf")


def _covariance(jacobian: np.ndarray, error: np.ndarray, effective: float,
                active_mask: np.ndarray | None = None) -> np.ndarray | None:
    """协方差 = 残差方差 × (JᵀJ)⁻¹；低通后的残差高度相关，残差方差按有效样本数计。

    贴在边界上的参数（`active_mask` ≠ 0）按固定处理：只对自由参数求逆，它们的行列
    置 0。否则一个压在边界上的参数（例如双脉冲轮里落在 0 的阻尼 c）会让 JᵀJ 近奇异。
    """
    size = jacobian.shape[1]
    free = (np.ones(size, dtype=bool) if active_mask is None
            else np.asarray(active_mask) == 0)
    variance = float(np.dot(error, error)) / max(effective - int(free.sum()), 1.0)
    reduced = jacobian[:, free]
    try:
        inverse = np.linalg.inv(reduced.T @ reduced)
    except np.linalg.LinAlgError:
        return None
    covariance = np.zeros((size, size))
    covariance[np.ix_(free, free)] = variance * inverse
    return covariance


def _covariance_correlation(covariance: np.ndarray | None, index_a: int, index_b: int) -> float:
    """协方差里两个参数的相关系数；算不出来（奇异或有一方贴边界）时按 1.0（不可分离）。"""
    if covariance is None:
        return 1.0
    var_a, var_b = covariance[index_a, index_a], covariance[index_b, index_b]
    if var_a <= 0.0 or var_b <= 0.0:
        return 1.0
    return float(covariance[index_a, index_b] / math.sqrt(var_a * var_b))


def fit_model(t: np.ndarray, torque_n_m: np.ndarray, rate_rad_s: np.ndarray,
              *, mass_kg: float, gravity_m_s2: float = 9.81,
              angle_rad: np.ndarray | None = None,
              inertia_guess: float = 0.02,
              delay_guess_s: float | None = None,
              max_delay_s: float = 0.25,
              thrust_point_to_cg_z_m: float | None = None,
              pivot_above_cg_m: float | None = None,
              pivot_guess_m: float | None = None,
              azimuth_rad: float | None = None,
              roll_pivot_to_cg_z_m: float | None = None,
              pitch_pivot_to_cg_z_m: float | None = None,
              lowpass_hz: float = LOWPASS_HZ,
              structure: str = "twd",
              pendulum_hz: float | None = None,
              firmware_tilt_levers_m: tuple[float, float] | None = None,
              rig_stiffness_n_m_rad: float | None = None) -> ModelFit:
    """拟合绕杆单摆，交出物理惯量 I_cg 与整定用的 I_cg/κ。延迟是**拟合参数**。

    两步：先用方程误差粗扫延迟并线性回归拿初值，再做输出误差非线性精修。
    只做第二步会掉进局部解（延迟被阻尼吃掉）；只做第一步则有方程误差固有的
    噪声偏置。两步合起来既稳又无偏。

    * `t` 必须等距（先 `uniform_signal`）；`inertia_guess` 是假定的**绕质心**惯量，
      只用于定界。
    * `pivot_above_cg_m`（旧名 `pivot_guess_m`）是**实测**杆高 d。|d| ≥ 2 cm 时 d 固定，
      默认 `structure="twd"`：刚体单摆 + 二阶舵机 + 反作用力偶 + 纯延迟，在
      `TWD_BAND_HZ` 带内拟合，κ = G/s(d)，并按飞行对象回路整形给出裕度；
      `structure="rigid"` 是 `FIT_BAND_HZ` 带内的刚体积分读数，只作对照。
      `pendulum_hz` 给了就按 I_杆 = m·g·d/(2π f)² 钉死。
      否则按杆过质心的旧工况，κ = 1，15 Hz 全段拟合 (I_杆, c, d, T)，d 限在 ±5 cm，
      不需要倾转轴高度。
    * `roll_pivot_to_cg_z_m` / `pitch_pivot_to_cg_z_m` 是倾转轴在质心上方的高度，
      与杆轴方位角 `azimuth_rad` 一起给出 s(d)。实测杆高而所需高度缺失时照算，
      κ 用桨盘中点兜底，但记入 `tuning_blockers`。
    * `thrust_point_to_cg_z_m` 是桨盘中点 h，只用于兜底、力矩极性与旧工况的阻尼换算。
    * `firmware_tilt_levers_m` 是固件把倾角换成记录力矩用的带符号有效力臂 (横滚, 俯仰)
      （`_core.firmware_tilt_levers` 从参数回显算）：给了就把 κ 与几何预测对拍
      （k = κ/κ_几何），否则 κ 只按宽范围检查。
    * `rig_stiffness_n_m_rad` 是挂砝码实测的台架总刚度 K（见 `rig_stiffness.py`）：给了就
      代替 m·g·d 作单摆回中刚度（两种实测杆高结构都是），I_杆 与 κ、k 都按它标定；
      结果里报等效杆高 K/(m·g) 与多出来的刚度 K − m·g·d。旧工况（杆过质心）不用它。
    * `angle_rad` 默认不用：摆动中偏离转轴的 IMU 吃到切向/向心加速度，融合姿态
      会被带偏；回归里的转角改由去均值陀螺积分得到。
    """
    geometry = _geometry(
        mass_kg=mass_kg, inertia_guess=inertia_guess,
        thrust_point_to_cg_z_m=thrust_point_to_cg_z_m, pivot_above_cg_m=pivot_above_cg_m,
        pivot_guess_m=pivot_guess_m, azimuth_rad=azimuth_rad,
        roll_pivot_to_cg_z_m=roll_pivot_to_cg_z_m, pitch_pivot_to_cg_z_m=pitch_pivot_to_cg_z_m,
        firmware_tilt_levers_m=firmware_tilt_levers_m)
    _check_structure(structure)
    record = _prepare_record(t, torque_n_m, rate_rad_s, angle_rad)
    if geometry.measured and structure == "twd":
        return _fit_twd([record], geometry, mass_kg=mass_kg, gravity=gravity_m_s2,
                        inertia_guess=inertia_guess, max_delay_s=max_delay_s,
                        pendulum_hz=pendulum_hz, rig_stiffness_n_m_rad=rig_stiffness_n_m_rad)
    if geometry.measured:
        return _fit_measured([record], geometry, mass_kg=mass_kg, gravity=gravity_m_s2,
                             inertia_guess=inertia_guess, delay_guess_s=delay_guess_s,
                             max_delay_s=max_delay_s, rig_stiffness_n_m_rad=rig_stiffness_n_m_rad)
    result = _fit_legacy(record, geometry, mass_kg=mass_kg, gravity=gravity_m_s2,
                         inertia_guess=inertia_guess, delay_guess_s=delay_guess_s,
                         max_delay_s=max_delay_s, lowpass_hz=lowpass_hz)
    if rig_stiffness_n_m_rad is not None:
        from dataclasses import replace
        result = replace(result, notes=result.notes + (
            "给了台架刚度，但杆近似过质心（旧工况）时残余偏心是拟合量，刚度不参与；"
            "量了杆高（|d| ≥ 2 cm）的单摆台架才用它。",))
    return result


def fit_model_multi(records, *, mass_kg: float, gravity_m_s2: float = 9.81,
                    inertia_guess: float = 0.02,
                    delay_guess_s: float | None = None,
                    max_delay_s: float = 0.25,
                    thrust_point_to_cg_z_m: float | None = None,
                    pivot_above_cg_m: float | None = None,
                    pivot_guess_m: float | None = None,
                    azimuth_rad: float | None = None,
                    roll_pivot_to_cg_z_m: float | None = None,
                    pitch_pivot_to_cg_z_m: float | None = None,
                    structure: str = "twd",
                    pendulum_hz: float | None = None,
                    firmware_tilt_levers_m: tuple[float, float] | None = None,
                    rig_stiffness_n_m_rad: float | None = None) -> ModelFit:
    """多轮联合拟合：`records` 为若干 (t, torque, rate)，共享一组模型参数
    （TWD：I_杆, G, T, ωs, ζs, ρ；刚体：I_杆, G, T），每轮各自解初值与零偏。
    只用于实测杆高（同一台架、同一几何、同一套固件力矩模型：各轮共用一个
    `firmware_tilt_levers_m`，不同力矩模型的轮次不能混在一起）。

    单轮里 I_杆 与 G 此消彼长（两轮实录各自差约 25%，而 I_cg/κ 只差 0.1%）；
    几轮一起拟合把这条"山谷"压窄。`fit_percent` 取各轮带内拟合优度的最小值，
    逐轮值在 `fit_percent_runs`。
    """
    geometry = _geometry(
        mass_kg=mass_kg, inertia_guess=inertia_guess,
        thrust_point_to_cg_z_m=thrust_point_to_cg_z_m, pivot_above_cg_m=pivot_above_cg_m,
        pivot_guess_m=pivot_guess_m, azimuth_rad=azimuth_rad,
        roll_pivot_to_cg_z_m=roll_pivot_to_cg_z_m, pitch_pivot_to_cg_z_m=pitch_pivot_to_cg_z_m,
        firmware_tilt_levers_m=firmware_tilt_levers_m)
    if not geometry.measured:
        raise ValueError("多轮联合拟合只用于量了杆高（|d| ≥ 2 cm）的单摆台架")
    _check_structure(structure)
    prepared = [_prepare_record(t, torque, rate) for t, torque, rate in records]
    if not prepared:
        raise ValueError("没有可拟合的轮次")
    if structure == "twd":
        return _fit_twd(prepared, geometry, mass_kg=mass_kg, gravity=gravity_m_s2,
                        inertia_guess=inertia_guess, max_delay_s=max_delay_s,
                        pendulum_hz=pendulum_hz, rig_stiffness_n_m_rad=rig_stiffness_n_m_rad)
    return _fit_measured(prepared, geometry, mass_kg=mass_kg, gravity=gravity_m_s2,
                         inertia_guess=inertia_guess, delay_guess_s=delay_guess_s,
                         max_delay_s=max_delay_s, rig_stiffness_n_m_rad=rig_stiffness_n_m_rad)


def _check_structure(structure: str) -> None:
    if structure not in STRUCTURES:
        raise ValueError(f"未知模型结构 {structure!r}，只能是 {' / '.join(STRUCTURES)}")


def _fit_measured(records: list[_Record], geometry: _Geometry, *, mass_kg: float,
                  gravity: float, inertia_guess: float, delay_guess_s: float | None,
                  max_delay_s: float, rig_stiffness_n_m_rad: float | None = None) -> ModelFit:
    """实测杆高：带内输出误差拟合 (I_杆, G, T)，c 固定 0，可多轮共享参数。

    代价只看 `FIT_BAND_HZ` 以下、`BURN_IN_S` 之后：6–18 Hz 里是碳杆约 9 Hz 的弯曲模态
    和执行器的确定性动态，刚体单摆模型描述不了，硬拟会把它们塞进 I_杆 / κ / T；
    开头半秒是起步瞬态。仿真仍从 t = 0 起，激励前力矩为 0。
    """
    pivot = float(geometry.pivot_input)
    scale = 1.0 - pivot * geometry.lever_inverse
    stiffness = _pendulum_stiffness(mass_kg, gravity, pivot, rig_stiffness_n_m_rad)
    rod_centre = inertia_guess + mass_kg * pivot ** 2
    lower = np.array([rod_centre * 0.05, scale * 0.05, 0.0])
    upper = np.array([rod_centre * 20.0, scale * 20.0, max_delay_s])

    # 同频段方程误差：扫延迟，并用重力这个唯一已知的力矩标定 I_杆 的初值。
    inertia_starts, gain_starts, delay_starts = [], [], set(DELAY_STARTS_S)
    for record in records:
        scan_delay, a1, _a2, a3 = _equation_error_scan(
            record.t, record.torque, record.rate, record.angle, max_delay_s,
            FIT_BAND_HZ, record.left, FIT_BAND_ORDER)
        delay_starts.add(float(scan_delay))
        if all(math.isfinite(v) for v in (a1, a3)) and a1 > 0.0 and a3 * stiffness > 0.0:
            inertia_starts.append(stiffness / a3)
            gain_starts.append(a1 * stiffness / a3)
    if delay_guess_s is not None:
        delay_starts.add(float(delay_guess_s))
    inertia0 = float(np.median(inertia_starts)) if inertia_starts else rod_centre
    gain0 = float(np.median(gain_starts)) if gain_starts else scale
    starts = [np.clip(np.array([inertia0, gain0, delay]), lower, upper)
              for delay in sorted(delay_starts)]

    prepared = []
    for record in records:
        keep = int(round(BURN_IN_S * record.fs))
        if record.t.size - keep < 64:
            raise ValueError("扣掉起步段后样本太少，拟合没有意义")
        band = lowpass_zero_phase(record.rate, record.fs, FIT_BAND_HZ, FIT_BAND_ORDER)
        centred = band[keep:] - float(np.mean(band[keep:]))
        prepared.append((record, keep, band, max(float(np.linalg.norm(centred)), 1e-12)))

    def run_error(x: np.ndarray, record: _Record, keep: int, measured: np.ndarray,
                  cutoff_hz: float, order: int) -> np.ndarray:
        forced, free_angle, free_rate = _responses(
            float(x[0]), 0.0, stiffness, float(x[1]), float(x[2]),
            record.t, record.torque, record.left)
        columns = lowpass_zero_phase(np.column_stack([forced, free_angle, free_rate]),
                                     record.fs, cutoff_hz, order)
        # 初始摆角/角速度与陀螺零偏对输出是线性的，每次在保留段上直接最小二乘解掉。
        basis = np.column_stack([columns[keep:, 1], columns[keep:, 2],
                                 np.ones(record.t.size - keep)])
        target = measured[keep:] - columns[keep:, 0]
        coefficients, *_ = np.linalg.lstsq(basis, target, rcond=None)
        return basis @ coefficients - target

    def residual(x: np.ndarray) -> np.ndarray:
        error = np.concatenate([
            run_error(x, record, keep, band, FIT_BAND_HZ, FIT_BAND_ORDER) / norm
            for record, keep, band, norm in prepared])
        if not np.all(np.isfinite(error)):
            return np.full(error.size, 1e3)
        return error

    result = None
    for x0 in starts:
        candidate = least_squares(residual, x0, bounds=(lower, upper),
                                  x_scale="jac", max_nfev=600)
        if result is None or candidate.cost < result.cost:
            result = candidate
    x = result.x

    fits, fits_15hz, effective = [], [], 0.0
    for record, keep, band, norm in prepared:
        fits.append(100.0 * (1.0 - float(np.linalg.norm(
            run_error(x, record, keep, band, FIT_BAND_HZ, FIT_BAND_ORDER))) / norm))
        wide = lowpass_zero_phase(record.rate, record.fs, LOWPASS_HZ)
        wide_norm = max(float(np.linalg.norm(wide[keep:] - float(np.mean(wide[keep:])))), 1e-12)
        fits_15hz.append(100.0 * (1.0 - float(np.linalg.norm(
            run_error(x, record, keep, wide, LOWPASS_HZ, 2))) / wide_norm))
        effective += (record.t.size - keep) * _lowpass_noise_gain(
            record.fs, FIT_BAND_HZ, FIT_BAND_ORDER)
    fit_pct, fit_15hz = min(fits), min(fits_15hz)

    inertia_rod, gain, delay_s = (float(v) for v in x)
    kappa = gain / scale
    inertia_cg = inertia_rod - mass_kg * pivot ** 2
    tuning_inertia = inertia_cg / kappa
    natural = (math.sqrt(stiffness / inertia_rod) / (2.0 * math.pi)
               if pivot > 0.0 else float("nan"))
    covariance = _covariance(result.jac, residual(x), effective)
    uncertainty_pct = _uncertainty_pct(covariance, np.array([1.0, 0.0, 0.0]), inertia_cg)
    # I_cg/κ = (I_杆 − m·d²)·s/G：I_杆 与 G 此消彼长，组合量的不确定度要带协方差一起传播。
    tuning_uncertainty_pct = _uncertainty_pct(
        covariance, np.array([scale / gain, -inertia_cg * scale / gain ** 2, 0.0]),
        tuning_inertia)
    correlation = _parameter_correlation(result.jac, 0, 1)
    separable = abs(correlation) < SEPARABILITY_LIMIT

    notes = _swing_note(records)
    rig_k, effective_pivot, extra_k = _stiffness_report(
        mass_kg, gravity, pivot, rig_stiffness_n_m_rad, notes)
    blockers: list[str] = []
    if len(records) > 1:
        notes.append(f"{len(records)} 轮联合拟合：共享 I_杆、G、T，每轮各自解初值与零偏；"
                     "逐轮带内拟合优度 " + " / ".join(f"{v:.1f}%" for v in fits) + "。")
    if geometry.missing_axes:
        axes = "、".join(geometry.missing_axes)
        notes.append(f"缺{axes}倾转轴高度：κ 暂按桨盘中点换算（未给桨盘中点时按 s = 1），"
                     "κ 与整定惯量都不可信。")
        blockers.append(f"缺倾转轴高度（{axes}），κ 与整定惯量无法换算")
    notes.append("刚体积分读数：被 6/10 Hz 谱线证伪，不用于整定。")
    blockers.append("刚体积分结构只作对照，不用于整定")
    notes.append("此频段阻尼与延迟不可分，按 0（保守）：整定阻尼也取 0。")
    notes.append(f"15 Hz 带宽下的拟合优度 {fit_15hz:.1f}% 只作诊断，上限约 70%：刚体积分描述不了"
                 "6–10 Hz 的舵机反作用谱线与约 11–14 Hz 的台架模态。")
    if fit_pct < FIT_BAND_PERCENT_MIN:
        notes.append(f"带内（{FIT_BAND_HZ:g} Hz 以下、{BURN_IN_S:g} s 之后）拟合优度只有 "
                     f"{fit_pct:.1f}%，模型没有描述住这段数据。")
        blockers.append(f"带内拟合优度 {fit_pct:.0f}% 低于 {FIT_BAND_PERCENT_MIN:.0f}%")
    if delay_s < DELAY_MIN_S:
        notes.append(f"辨出的延迟只有 {delay_s * 1000:.2f} ms，比一个控制拍还短，不可信。")
        blockers.append("延迟不足 1 ms，不可信")
    blockers.extend(_inertia_checks(inertia_cg, inertia_rod, mass_kg, notes))
    if not separable:
        notes.append(f"I_杆 与 G 强相关（相关系数 {correlation:.3f}）：两者此消彼长，I_cg 与 κ "
                     "单独都不可信；整定用的 I_cg/κ 不受影响。")
    if not uncertainty_pct <= INERTIA_UNCERTAINTY_MAX_PCT:
        notes.append(f"绕质心惯量单独的相对不确定度约 {uncertainty_pct:.0f}%：与 κ 此消彼长，"
                     "只作提示，不影响整定。")
    if not tuning_uncertainty_pct <= TUNING_INERTIA_UNCERTAINTY_MAX_PCT:
        notes.append(f"整定惯量 I_cg/κ 的相对不确定度约 {tuning_uncertainty_pct:.0f}%，超过 "
                     f"{TUNING_INERTIA_UNCERTAINTY_MAX_PCT:.0f}%：加长激励或加大幅值重测。")
        blockers.append(f"整定惯量不确定度 {tuning_uncertainty_pct:.0f}% 超过 "
                        f"{TUNING_INERTIA_UNCERTAINTY_MAX_PCT:.0f}%")
    gain_sigma = _uncertainty_pct(covariance, np.array([0.0, 1.0, 0.0]), gain) / 100.0
    model_blockers, geometric_scale, model_scale = _torque_model_checks(
        kappa, gain, scale, geometry, notes, gain_sigma)
    blockers.extend(model_blockers)

    return ModelFit(
        inertia_kg_m2=float(inertia_cg),
        inertia_rod_kg_m2=inertia_rod,
        pivot_above_cg_m=pivot,
        eccentricity_m=pivot,
        torque_scale=float(scale),
        damping_n_m_s=0.0,
        delay_s=delay_s,
        fit_percent=float(fit_pct),
        natural_hz=float(natural),
        damping_eccentricity_correlation=float(correlation),
        separable=bool(separable),
        samples=int(sum(record.t.size for record in records)),
        torque_model_scale=float(kappa),
        tuning_inertia_kg_m2=float(tuning_inertia),
        tuning_damping_n_m_s=0.0,
        inertia_uncertainty_pct=float(uncertainty_pct),
        tuning_inertia_uncertainty_pct=float(tuning_uncertainty_pct),
        fit_percent_15hz=float(fit_15hz),
        tuning_blockers=tuple(blockers),
        notes=tuple(notes),
        fit_percent_runs=tuple(float(v) for v in fits),
        structure="rigid",
        rigid_tuning_inertia_kg_m2=float(tuning_inertia),
        rigid_delay_s=float(delay_s),
        rigid_fit_percent=float(fit_pct),
        torque_model_geometric_scale=geometric_scale,
        thrust_servo_model_scale=model_scale,
        rig_stiffness_n_m_rad=rig_k,
        effective_pivot_m=effective_pivot,
        extra_stiffness_n_m_rad=extra_k,
    )


def _twd_matrices(inertia_rod: float, gain: float, stiffness: float, servo_wn: float,
                  servo_zeta: float, rho: float, damping: float = 0.0
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """TWD 杆上模型的状态空间，x = [u, u', θ, ω]，输入 v = τ_rec(t − T)，输出 ω。

    舵机 u'' = ωs²(v − u) − 2ζs·ωs·u'（u 以固件力矩为单位，单位直流增益）；
    机体 I_杆·θ'' + c·θ' + m·g·d·θ = G·u + ρ·u''。
    """
    wn2 = servo_wn * servo_wn
    a_matrix = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [-wn2, -2.0 * servo_zeta * servo_wn, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [(gain - rho * wn2) / inertia_rod, -2.0 * rho * servo_zeta * servo_wn / inertia_rod,
         -stiffness / inertia_rod, -damping / inertia_rod]])
    b_vector = np.array([0.0, wn2, 0.0, rho * wn2 / inertia_rod])
    c_vector = np.array([0.0, 0.0, 0.0, 1.0])
    return a_matrix, b_vector, c_vector


def _foh(a_matrix: np.ndarray, b_vector: np.ndarray, dt: float
         ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """n 状态一阶保持离散化：x[k+1] = Φ·x[k] + b_now·v[k] + b_next·v[k+1]。"""
    n = a_matrix.shape[0]
    augmented = np.zeros((n + 2, n + 2))
    augmented[:n, :n] = a_matrix * dt
    augmented[:n, n] = b_vector * dt
    augmented[n, n + 1] = 1.0
    transition = expm(augmented)
    phi = transition[:n, :n]
    b_next = transition[:n, n + 1]
    return phi, transition[:n, n] - b_next, b_next


def _state_space_columns(a_matrix: np.ndarray, b_vector: np.ndarray, c_vector: np.ndarray,
                         dt: float, v: np.ndarray, x_init: np.ndarray,
                         free_states: tuple[int, ...]) -> tuple[np.ndarray, list[np.ndarray]]:
    """FOH 精确离散后的强迫响应（从 x_init 起）与各状态单位初值的自由响应。

    不逐点循环：离散系统的传递函数分子由前 n 个 Markov 参数与特征多项式卷积得到，
    整段交给 lfilter；自由响应同理（前 n 个输出决定整条递推）。
    """
    phi, b_now, b_next = _foh(a_matrix, b_vector, dt)
    n = phi.shape[0]
    den = np.real(np.poly(phi))
    rows = [c_vector]
    for _ in range(n):
        rows.append(rows[-1] @ phi)
    rows = np.array(rows[:n])                      # 第 k 行 = C·Φᵏ
    impulse = np.zeros(v.size)
    impulse[0] = 1.0

    def free(x0: np.ndarray) -> np.ndarray:
        return signal.lfilter(np.convolve(den, rows @ x0)[:n], den, impulse)

    # x' = x − b_next·v 化成标准离散状态空间 (Φ, Φ·b_next + b_now, C, C·b_next)。
    g = phi @ b_next + b_now
    markov = np.concatenate([[c_vector @ b_next], rows @ g])
    forced = (signal.lfilter(np.convolve(den, markov)[:n + 1], den, v)
              + free(np.asarray(x_init, dtype=float) - b_next * v[0]))
    columns = []
    for index in free_states:
        unit = np.zeros(n)
        unit[index] = 1.0
        columns.append(free(unit))
    return forced, columns


def simulate_twd(t: np.ndarray, torque: np.ndarray, *, inertia_rod: float, gain: float,
                 stiffness: float, dead_time_s: float, servo_wn: float, servo_zeta: float,
                 reaction_couple: float, left: float = 0.0, damping: float = 0.0) -> np.ndarray:
    """TWD 杆上角速度响应：舵机停在 `left`、机体静止起步，记录开始前力矩为 `left`。"""
    t = np.asarray(t, dtype=float)
    torque = np.asarray(torque, dtype=float)
    a_matrix, b_vector, c_vector = _twd_matrices(inertia_rod, gain, stiffness, servo_wn,
                                                 servo_zeta, reaction_couple, damping)
    delayed = np.interp(t - dead_time_s, t, torque, left=left, right=torque[-1])
    forced, _columns = _state_space_columns(a_matrix, b_vector, c_vector, _sample_period(t),
                                            delayed, np.array([left, 0.0, 0.0, 0.0]), ())
    return forced


def equivalent_delay_s(dead_time_s: float, servo_wn: float, servo_zeta: float,
                       freq_hz: float = EQUIVALENT_DELAY_HZ) -> float:
    """纯延迟 + 二阶舵机在 f 处的等效延迟 T − arg S(jω)/ω。"""
    omega = 2.0 * math.pi * freq_hz
    lag = math.atan2(2.0 * servo_zeta * servo_wn * omega, servo_wn ** 2 - omega ** 2)
    return dead_time_s + lag / omega


def _fit_twd(records: list[_Record], geometry: _Geometry, *, mass_kg: float, gravity: float,
             inertia_guess: float, max_delay_s: float,
             pendulum_hz: float | None,
             rig_stiffness_n_m_rad: float | None = None) -> ModelFit:
    """实测杆高的默认结构：刚体单摆 + 二阶舵机 + 反作用力偶 ρ·u''（TWD）+ 纯延迟。

    先跑 4 Hz 刚体积分拟合作诊断基线并给初值，再在 `TWD_BAND_HZ` 带内、起步
    `BURN_IN_S` 之后做输出误差拟合 (I_杆, G, c, T, ωs, ζs, ρ)；给了 `pendulum_hz` 时
    I_杆 = m·g·d/(2π f)² 固定。VarPro 只解 θ0、ω0 与零偏，舵机从静止起步。
    另从模型无关频响读 TWD 零点与摆频，与模型对照。
    """
    rigid = _fit_measured(records, geometry, mass_kg=mass_kg, gravity=gravity,
                          inertia_guess=inertia_guess, delay_guess_s=None,
                          max_delay_s=max_delay_s, rig_stiffness_n_m_rad=rig_stiffness_n_m_rad)
    pivot = float(geometry.pivot_input)
    scale = 1.0 - pivot * geometry.lever_inverse
    stiffness = _pendulum_stiffness(mass_kg, gravity, pivot, rig_stiffness_n_m_rad)
    rod_centre = inertia_guess + mass_kg * pivot ** 2
    pinned = None
    if pendulum_hz is not None:
        pendulum_hz = float(pendulum_hz)
        if not (math.isfinite(pendulum_hz) and pendulum_hz > 0.0) or stiffness <= 0.0:
            raise ValueError("摆频必须是正的有限值，且只用于杆在质心上方的单摆")
        pinned = stiffness / (2.0 * math.pi * pendulum_hz) ** 2

    names = ([] if pinned is not None else ["I_rod"]) + ["G", "c", "T", "ws", "zs", "rho"]
    bounds = {"I_rod": (rod_centre * 0.05, rod_centre * 20.0),
              "G": (scale * 0.05, scale * 20.0), "c": DAMPING_BOUNDS_N_M_S,
              "T": TWD_DEAD_TIME_BOUNDS_S, "ws": SERVO_WN_BOUNDS, "zs": SERVO_ZETA_BOUNDS,
              "rho": REACTION_COUPLE_BOUNDS}
    lower = np.array([bounds[name][0] for name in names])
    upper = np.array([bounds[name][1] for name in names])

    # 初值取自刚体基线：刚体读数把舵机滞后与 TWD 零点都折进了惯量与延迟里。
    rigid_gain = rigid.torque_model_scale * scale
    rigid_rod = rigid.inertia_rod_kg_m2
    gain0 = 2.0 * rigid_gain * (pinned / rigid_rod if pinned is not None else 1.0)
    rho0 = gain0 / (2.0 * math.pi * 2.8) ** 2
    starts = []
    # 刚体读数自己都没描述住数据时（扫频轮的刚体 4 Hz 拟合只有十几个百分点），
    # 拿它当初值只会浪费时间，只用下面与它无关的起点。
    rigid_starts = TWD_STARTS if rigid.fit_percent >= RIGID_SEED_FIT_MIN else ()
    for delay_offset, wn0 in rigid_starts:
        start = {"I_rod": rigid_rod, "G": gain0, "c": 0.02,
                 "T": max(rigid.delay_s - delay_offset, 0.005), "ws": wn0, "zs": 0.45,
                 "rho": rho0}
        starts.append(np.clip(np.array([start[name] for name in names]), lower, upper))
    # 刚体读数本身可能落在错的一侧（κ 很小时 TWD 零点跑到 2 Hz 谱线之下，刚体积分把相位
    # 读反），再补几个与它无关的起点：κ 取名义值，舵机与延迟取评审给的量级。
    for kappa0 in TWD_GENERIC_KAPPA_STARTS:
        gain_generic = kappa0 * scale
        start = {"I_rod": pinned if pinned is not None else rod_centre, "G": gain_generic,
                 "c": 0.02, "T": 0.04, "ws": 33.0, "zs": 0.45,
                 "rho": gain_generic / (2.0 * math.pi * 2.8) ** 2}
        starts.append(np.clip(np.array([start[name] for name in names]), lower, upper))

    def unpack(x: np.ndarray) -> tuple[float, float, float, float, float, float, float]:
        values = dict(zip(names, (float(v) for v in x)))
        return (values.get("I_rod", pinned), values["G"], values["c"], values["T"],
                values["ws"], values["zs"], values["rho"])

    prepared = []
    for record in records:
        keep = int(round(BURN_IN_S * record.fs))
        if record.t.size - keep < 64:
            raise ValueError("扣掉起步段后样本太少，拟合没有意义")
        band = lowpass_zero_phase(record.rate, record.fs, TWD_BAND_HZ, TWD_BAND_ORDER)
        centred = band[keep:] - float(np.mean(band[keep:]))
        prepared.append((record, keep, band, max(float(np.linalg.norm(centred)), 1e-12)))

    def run_error(x: np.ndarray, record: _Record, keep: int, measured: np.ndarray,
                  cutoff_hz: float, order: int) -> np.ndarray:
        inertia_x, gain_x, damping_x, delay_x, wn_x, zeta_x, rho_x = unpack(x)
        a_matrix, b_vector, c_vector = _twd_matrices(inertia_x, gain_x, stiffness, wn_x,
                                                     zeta_x, rho_x, damping_x)
        delayed = np.interp(record.t - delay_x, record.t, record.torque,
                            left=record.left, right=record.torque[-1])
        forced, (free_angle, free_rate) = _state_space_columns(
            a_matrix, b_vector, c_vector, 1.0 / record.fs, delayed,
            np.array([record.left, 0.0, 0.0, 0.0]), (2, 3))
        columns = lowpass_zero_phase(np.column_stack([forced, free_angle, free_rate]),
                                     record.fs, cutoff_hz, order)
        basis = np.column_stack([columns[keep:, 1], columns[keep:, 2],
                                 np.ones(record.t.size - keep)])
        target = measured[keep:] - columns[keep:, 0]
        coefficients, *_ = np.linalg.lstsq(basis, target, rcond=None)
        return basis @ coefficients - target

    def residual(x: np.ndarray) -> np.ndarray:
        error = np.concatenate([
            run_error(x, record, keep, band, TWD_BAND_HZ, TWD_BAND_ORDER) / norm
            for record, keep, band, norm in prepared])
        if not np.all(np.isfinite(error)):
            return np.full(error.size, 1e3)
        return error

    result = None
    for x0 in starts:
        candidate = least_squares(residual, x0, bounds=(lower, upper), x_scale="jac",
                                  diff_step=1e-4, max_nfev=400)
        if result is None or candidate.cost < result.cost:
            result = candidate
    x = result.x

    fits, fits_15hz, effective = [], [], 0.0
    for record, keep, band, norm in prepared:
        fits.append(100.0 * (1.0 - float(np.linalg.norm(
            run_error(x, record, keep, band, TWD_BAND_HZ, TWD_BAND_ORDER))) / norm))
        wide = lowpass_zero_phase(record.rate, record.fs, LOWPASS_HZ)
        wide_norm = max(float(np.linalg.norm(wide[keep:] - float(np.mean(wide[keep:])))), 1e-12)
        fits_15hz.append(100.0 * (1.0 - float(np.linalg.norm(
            run_error(x, record, keep, wide, LOWPASS_HZ, 2))) / wide_norm))
        effective += (record.t.size - keep) * _lowpass_noise_gain(
            record.fs, TWD_BAND_HZ, TWD_BAND_ORDER)
    fit_pct, fit_15hz = min(fits), min(fits_15hz)

    inertia_rod, gain, damping, dead_time, servo_wn, servo_zeta, rho = unpack(x)
    kappa = gain / scale
    inertia_cg = inertia_rod - mass_kg * pivot ** 2
    tuning_inertia = inertia_cg / kappa
    natural = (math.sqrt(stiffness / inertia_rod) / (2.0 * math.pi)
               if stiffness > 0.0 else float("nan"))
    rig_zero = math.sqrt(gain / rho) / (2.0 * math.pi) if rho > 0.0 else float("nan")
    flight_zero = math.sqrt(kappa / rho) / (2.0 * math.pi) if rho > 0.0 else float("nan")
    delay_eq = equivalent_delay_s(dead_time, servo_wn, servo_zeta)

    covariance = _covariance(result.jac, residual(x), effective, result.active_mask)
    gain_index = names.index("G")
    tuning_gradient = np.zeros(len(names))
    tuning_gradient[gain_index] = -inertia_cg * scale / gain ** 2
    if pinned is None:
        tuning_gradient[0] = scale / gain
        uncertainty_pct = _uncertainty_pct(covariance, np.eye(len(names))[0], inertia_cg)
        correlation = _covariance_correlation(covariance, 0, gain_index)
    else:
        uncertainty_pct = 0.0          # I_杆 由摆频钉死，拟合本身不给它不确定度
        correlation = _covariance_correlation(covariance, gain_index, names.index("rho"))
    tuning_uncertainty_pct = _uncertainty_pct(covariance, tuning_gradient, tuning_inertia)
    separable = abs(correlation) < SEPARABILITY_LIMIT

    notes = _swing_note(records)
    rig_k, effective_pivot, extra_k = _stiffness_report(
        mass_kg, gravity, pivot, rig_stiffness_n_m_rad, notes)
    blockers: list[str] = []
    if len(records) > 1:
        notes.append(f"{len(records)} 轮联合拟合：共享 I_杆、G、T、ωs、ζs、ρ，每轮各自解初值与"
                     "零偏；逐轮带内拟合优度 " + " / ".join(f"{v:.1f}%" for v in fits) + "。")
    if pinned is not None:
        notes.append(f"I_杆 按摆频 {pendulum_hz:.3f} Hz 钉死为 {pinned:.5f} kg·m²。")
    if geometry.missing_axes:
        axes = "、".join(geometry.missing_axes)
        notes.append(f"缺{axes}倾转轴高度：κ 暂按桨盘中点换算（未给桨盘中点时按 s = 1），"
                     "κ 与整定惯量都不可信。")
        blockers.append(f"缺倾转轴高度（{axes}），κ 与整定惯量无法换算")
    notes.append(f"刚体积分读数：被 6/10 Hz 谱线证伪，不用于整定（整定惯量 "
                 f"{rigid.tuning_inertia_kg_m2:.4f} kg·m²、延迟 {rigid.delay_s * 1000:.1f} ms、"
                 f"4 Hz 带内拟合 {rigid.fit_percent:.1f}%）。")
    pendulum_zeta = (damping / (2.0 * math.sqrt(stiffness * inertia_rod))
                     if stiffness > 0.0 else float("nan"))
    notes.append(f"绕杆阻尼 c = {damping:.4f} N·m·s（摆的阻尼比约 {pendulum_zeta:.2f}）只进拟合："
                 "它只在摆频附近看得出来，与舵机滞后、延迟在更高频段不可分，整定阻尼按 0（保守）。")
    at_bound = [label for name, label in (("ws", "ωs"), ("zs", "ζs"))
                if min(x[names.index(name)] - lower[names.index(name)],
                       upper[names.index(name)] - x[names.index(name)])
                <= BOUND_TOLERANCE * (upper[names.index(name)] - lower[names.index(name)])]
    if at_bound:
        labels = "、".join(at_bound)
        notes.append(f"舵机二阶的 {labels} 贴到拟合边界：这段激励分不开舵机滞后与纯延迟"
                     "（激励上限没越过舵机转折频率），只有 1 Hz 等效延迟可信，10 Hz 附近的回路增益"
                     "是外推。扫频终点提高到 12 Hz 以上，或与双脉冲轮联合拟合。")
        blockers.append(f"舵机二阶（{labels}）贴边界，10 Hz 裕度无法核实")
    if len(records) > 1:
        amplitudes = [float(np.std(record.torque)) for record in records]
        if max(amplitudes) > AMPLITUDE_MISMATCH_RATIO * min(amplitudes):
            notes.append("各轮力矩 rms 相差 "
                         f"{max(amplitudes) / min(amplitudes):.1f} 倍（"
                         + " / ".join(f"{v:.3f}" for v in amplitudes) + " N·m）：舵机回差让小幅值"
                         "轮的等效增益偏低、滞后偏大，一组线性参数描述不了两种工况；幅值相当的轮次"
                         "再联合。")
    measured_pendulum, measured_zero = _model_free_summary(records)
    for label, measured, model in (("TWD 零点", measured_zero, rig_zero),
                                   ("摆频", measured_pendulum, natural)):
        if not (math.isfinite(measured) and math.isfinite(model)):
            continue
        gap = model / measured - 1.0
        notes.append(f"模型无关频响量到的{label} {measured:.2f} Hz，模型 {model:.2f} Hz"
                     f"（相差 {100.0 * gap:+.0f}%）。")
        # 只作提示、不拦：整段 ETFE 只有宽带激励（扫频）才可靠，双脉冲的谐波会把它带偏——
        # 2026-09-27 rod_225017 双脉冲读出摆频 1.26 Hz，而自由摆直接测得 0.675 Hz、与模型一致。
        if abs(gap) > MODEL_FREE_DISAGREEMENT_MAX:
            notes.append(f"{label}与模型无关频响相差 {100.0 * abs(gap):.0f}%：扫频轮出现这种差距要怀疑"
                         "模型结构；双脉冲轮的模型无关估计本身不可靠，可忽略。")
    notes.append(f"15 Hz 带宽下的拟合优度 {fit_15hz:.1f}% 只作诊断：约 11–14 Hz 的台架/杆弹性"
                 "次要模态不在模型里。")
    if fit_pct < TWD_FIT_PERCENT_MIN:
        notes.append(f"带内（{TWD_BAND_HZ:g} Hz 以下、{BURN_IN_S:g} s 之后）拟合优度只有 "
                     f"{fit_pct:.1f}%，模型没有描述住这段数据。")
        blockers.append(f"带内拟合优度 {fit_pct:.0f}% 低于 {TWD_FIT_PERCENT_MIN:.0f}%")
    if delay_eq < DELAY_MIN_S:
        notes.append(f"1 Hz 等效延迟只有 {delay_eq * 1000:.2f} ms，比一个控制拍还短，不可信。")
        blockers.append("延迟不足 1 ms，不可信")
    blockers.extend(_inertia_checks(inertia_cg, inertia_rod, mass_kg, notes))
    if not separable:
        pair = "I_杆 与 G" if pinned is None else "G 与 ρ"
        notes.append(f"{pair} 强相关（相关系数 {correlation:.3f}）：单独看都不可信；"
                     "整定用的 I_cg/κ 与裕度检查不受影响。量摆频（pendulum_hz）或多轮联合可以收窄。")
    if pinned is None and not uncertainty_pct <= INERTIA_UNCERTAINTY_MAX_PCT:
        notes.append(f"绕质心惯量单独的相对不确定度约 {uncertainty_pct:.0f}%：与 κ 此消彼长，"
                     "只作提示，不影响整定。")
    if not tuning_uncertainty_pct <= TUNING_INERTIA_UNCERTAINTY_MAX_PCT:
        notes.append(f"整定惯量 I_cg/κ 的相对不确定度约 {tuning_uncertainty_pct:.0f}%，超过 "
                     f"{TUNING_INERTIA_UNCERTAINTY_MAX_PCT:.0f}%：加长激励或加大幅值重测。")
        blockers.append(f"整定惯量不确定度 {tuning_uncertainty_pct:.0f}% 超过 "
                        f"{TUNING_INERTIA_UNCERTAINTY_MAX_PCT:.0f}%")
    gain_sigma = _uncertainty_pct(covariance, np.eye(len(names))[gain_index], gain) / 100.0
    model_blockers, geometric_scale, model_scale = _torque_model_checks(
        kappa, gain, scale, geometry, notes, gain_sigma)
    blockers.extend(model_blockers)
    if rho <= 0.0:
        notes.append(f"反作用力偶 ρ = {rho:.5f} s² 不为正：没有 TWD 零点，按实数零点处理。")

    margins = (float("nan"), float("nan"), float("nan"))
    if inertia_cg > 0.0 and kappa > 0.0:
        from .tune import DEPLOYED_RPM_NOTCH, FlightPlant, synthesise_rate_shaped
        shaped = synthesise_rate_shaped(FlightPlant(
            inertia_cg_kg_m2=inertia_cg, torque_model_scale=kappa, dead_time_s=dead_time,
            servo_wn_rad_s=servo_wn, servo_zeta=servo_zeta, reaction_couple_s2=rho,
            feedback_notch=DEPLOYED_RPM_NOTCH))
        margins = (shaped.gain_margin_db, shaped.phase_margin_deg, shaped.crossover_hz)
        if not shaped.feasible:
            blockers.append("此对象下无法同时满足 6 dB/45° 裕度")

    return ModelFit(
        inertia_kg_m2=float(inertia_cg),
        inertia_rod_kg_m2=float(inertia_rod),
        pivot_above_cg_m=pivot,
        eccentricity_m=pivot,
        torque_scale=float(scale),
        damping_n_m_s=float(damping),
        delay_s=float(delay_eq),
        fit_percent=float(fit_pct),
        natural_hz=float(natural),
        damping_eccentricity_correlation=float(correlation),
        separable=bool(separable),
        samples=int(sum(record.t.size for record in records)),
        torque_model_scale=float(kappa),
        tuning_inertia_kg_m2=float(tuning_inertia),
        tuning_damping_n_m_s=0.0,
        inertia_uncertainty_pct=float(uncertainty_pct),
        tuning_inertia_uncertainty_pct=float(tuning_uncertainty_pct),
        fit_percent_15hz=float(fit_15hz),
        tuning_blockers=tuple(blockers),
        notes=tuple(notes),
        fit_percent_runs=tuple(float(v) for v in fits),
        structure="twd",
        dead_time_s=float(dead_time),
        servo_wn_rad_s=float(servo_wn),
        servo_zeta=float(servo_zeta),
        reaction_couple_s2=float(rho),
        rig_zero_hz=float(rig_zero),
        flight_zero_hz=float(flight_zero),
        equivalent_delay_1hz_s=float(delay_eq),
        gain_margin_db=float(margins[0]),
        phase_margin_deg=float(margins[1]),
        crossover_hz=float(margins[2]),
        rigid_tuning_inertia_kg_m2=float(rigid.tuning_inertia_kg_m2),
        rigid_delay_s=float(rigid.delay_s),
        rigid_fit_percent=float(rigid.fit_percent),
        measured_zero_hz=float(measured_zero),
        measured_pendulum_hz=float(measured_pendulum),
        torque_model_geometric_scale=geometric_scale,
        thrust_servo_model_scale=model_scale,
        rig_stiffness_n_m_rad=rig_k,
        effective_pivot_m=effective_pivot,
        extra_stiffness_n_m_rad=extra_k,
    )


def _model_free_summary(records: list[_Record]) -> tuple[float, float]:
    """各轮模型无关读数（摆频, 零点）的中位数；读不出来的轮不算。"""
    pendulums, zeros = [], []
    for record in records:
        pendulum, zero = model_free_features(record.t, record.torque, record.rate)
        if math.isfinite(pendulum):
            pendulums.append(pendulum)
        if math.isfinite(zero):
            zeros.append(zero)
    return (float(np.median(pendulums)) if pendulums else _NAN,
            float(np.median(zeros)) if zeros else _NAN)


def _inertia_checks(inertia_cg: float, inertia_rod: float, mass_kg: float,
                    notes: list[str]) -> list[str]:
    """I_cg 本身的合理性（两种工况共用）；返回 blockers，长说明追加进 notes。"""
    if not (math.isfinite(inertia_cg) and inertia_cg > 0.0):
        return ["绕质心惯量不为正"]
    blockers = []
    if inertia_cg < INERTIA_CG_SHARE_MIN * inertia_rod:
        notes.append(f"I_cg 只占绕杆惯量的 {100.0 * inertia_cg / inertia_rod:.0f}%，"
                     "主要由 m·d² 决定：d 的小误差会被成倍放大。")
        blockers.append("I_cg 不足绕杆惯量的 10%")
    radius = math.sqrt(inertia_cg / mass_kg)
    low_radius, high_radius = GYRATION_RADIUS_RANGE_M
    if not low_radius <= radius <= high_radius:
        notes.append(f"绕质心惯量 {inertia_cg:.4g} kg·m² 折合回转半径 {radius:.3f} m，"
                     f"不在 {low_radius:g}–{high_radius:g} m 的合理范围内。")
        blockers.append(f"回转半径 {radius:.3f} m 越界")
    return blockers


def _fit_legacy(record: _Record, geometry: _Geometry, *, mass_kg: float, gravity: float,
                inertia_guess: float, delay_guess_s: float | None, max_delay_s: float,
                lowpass_hz: float) -> ModelFit:
    """旧工况（杆近似过质心）：15 Hz 全段拟合 (I_杆, c, d, T)，κ = 1。"""
    t, torque, rate, angle, left, fs = (record.t, record.torque, record.rate,
                                        record.angle, record.left, record.fs)
    pivot_input = geometry.pivot_input
    weight = mass_kg * gravity
    scan_delay, a1, a2, a3 = _equation_error_scan(
        t, torque, rate, angle, max_delay_s, lowpass_hz, left)
    if delay_guess_s is not None:
        scan_delay = float(delay_guess_s)
    regression_ok = all(math.isfinite(v) for v in (a1, a2, a3)) and a1 > 0.0

    # d 很小，几何换算只是个小修正：有倾转轴高度用它，缺的轴用桨盘中点兜底。
    lever = None if geometry.lever_inverse == 0.0 else 1.0 / geometry.lever_inverse
    low, high = pivot_bounds(lever)
    if regression_ok:
        pivot_regression = pivot_from_ratio(a3 / (a1 * weight), lever, (low, high))
    else:
        pivot_regression = _clip(pivot_input if pivot_input is not None else 0.0, low, high)
    rod_centre = inertia_guess + mass_kg * pivot_regression ** 2
    lower = np.array([rod_centre * 0.05, 0.0, low, 0.0])
    upper = np.array([rod_centre * 20.0, 1.0, high, max_delay_s])

    def start(pivot_start: float) -> np.ndarray:
        if regression_ok:
            inertia_start = torque_scale(pivot_start, lever) / a1
            damping_start = a2 * inertia_start
        else:
            inertia_start = inertia_guess + mass_kg * pivot_start ** 2
            damping_start = 1e-3
        if not math.isfinite(damping_start) or damping_start < 0.0:
            damping_start = 1e-3
        return np.clip(np.array([inertia_start, damping_start, pivot_start, scan_delay]),
                       lower, upper)

    starts = [start(pivot_regression)]
    if pivot_input is not None:
        pivot_hint = _clip(pivot_input, low, high)
        if abs(pivot_hint - pivot_regression) > 0.005:
            starts.append(start(pivot_hint))

    measured_rate = lowpass_zero_phase(rate, fs, lowpass_hz)
    ones = np.ones(t.size)

    def predict(x: np.ndarray) -> np.ndarray:
        inertia_x, damping_x, pivot_x, delay_x = (float(v) for v in x)
        forced, free_angle, free_rate = _responses(
            inertia_x, damping_x, weight * pivot_x, torque_scale(pivot_x, lever), delay_x,
            t, torque, left)
        columns = lowpass_zero_phase(np.column_stack([forced, free_angle, free_rate]),
                                     fs, lowpass_hz)
        # 初始摆角/角速度与陀螺零偏对输出是线性的，每次直接最小二乘解掉（变量投影）。
        basis = np.column_stack([columns[:, 1], columns[:, 2], ones])
        coefficients, *_ = np.linalg.lstsq(basis, measured_rate - columns[:, 0], rcond=None)
        return columns[:, 0] + basis @ coefficients

    def residual(x: np.ndarray) -> np.ndarray:
        error = predict(x) - measured_rate
        if not np.all(np.isfinite(error)):
            return np.full(t.size, 1e3)
        return error

    result = None
    for x0 in starts:
        candidate = least_squares(residual, x0, bounds=(lower, upper),
                                  x_scale="jac", max_nfev=600)
        if result is None or candidate.cost < result.cost:
            result = candidate

    error = predict(result.x) - measured_rate
    centred = measured_rate - float(np.mean(measured_rate))
    fit_pct = 100.0 * (1.0 - float(np.linalg.norm(error))
                       / max(float(np.linalg.norm(centred)), 1e-9))
    correlation = _parameter_correlation(result.jac, 1, 2)   # c 与残余偏心
    separable = abs(correlation) < SEPARABILITY_LIMIT

    inertia_rod, damping, pivot, delay_s = (float(v) for v in result.x)
    scale = torque_scale(pivot, lever)
    inertia_cg = inertia_rod - mass_kg * pivot ** 2
    natural = (math.sqrt(weight * pivot / inertia_rod) / (2.0 * math.pi)
               if pivot > 0.0 else float("nan"))
    covariance = _covariance(result.jac, error, t.size * _lowpass_noise_gain(fs, lowpass_hz))
    uncertainty_pct = _uncertainty_pct(
        covariance, np.array([1.0, 0.0, -2.0 * mass_kg * pivot, 0.0]), inertia_cg)

    notes = _swing_note([record])
    blockers: list[str] = []
    if geometry.lever_inverse == 0.0:
        notes.append("未提供推力点位置，按杆过质心近似：几何换算 s 取 1。")
    notes.append("杆过质心时无法用重力标定力矩模型，κ 按 1：惯量与阻尼都以固件力矩为准。")
    if pivot - lower[2] < 1e-3 or upper[2] - pivot < 1e-3:
        notes.append(f"残余偏心 {pivot:.3f} m 贴到拟合边界 [{lower[2]:.3f}, "
                     f"{upper[2]:.3f}] m：杆明显不过质心，请量出杆高 d 后重拟合。")
        blockers.append("旧工况残余偏心贴边界：请量杆高 d")
    if fit_pct < FIT_PERCENT_MIN:
        notes.append(f"拟合优度只有 {fit_pct:.1f}%，模型没有描述住这段数据。")
        blockers.append(f"拟合优度 {fit_pct:.0f}% 低于 {FIT_PERCENT_MIN:.0f}%")
    if delay_s < DELAY_MIN_S:
        notes.append(f"辨出的延迟只有 {delay_s * 1000:.2f} ms，比一个控制拍还短，不可信。")
        blockers.append("延迟不足 1 ms，不可信")
    blockers.extend(_inertia_checks(inertia_cg, inertia_rod, mass_kg, notes))
    if not separable:
        notes.append("阻尼与偏心不可分离（相关系数 "
                     f"{correlation:.3f}）：两者在这段激励里都只表现为「回中」。"
                     "用不同频率的 chirp 重测可以分开。")
        blockers.append("阻尼与回中项不可分离")
    if not uncertainty_pct <= INERTIA_UNCERTAINTY_MAX_PCT:
        notes.append(f"绕质心惯量的相对不确定度约 {uncertainty_pct:.0f}%，"
                     f"超过 {INERTIA_UNCERTAINTY_MAX_PCT:.0f}%：加长激励或加大幅值重测。")
        blockers.append(f"惯量不确定度 {uncertainty_pct:.0f}% 超过 "
                        f"{INERTIA_UNCERTAINTY_MAX_PCT:.0f}%")

    return ModelFit(
        inertia_kg_m2=float(inertia_cg),
        inertia_rod_kg_m2=inertia_rod,
        pivot_above_cg_m=pivot,
        eccentricity_m=pivot,
        torque_scale=float(scale),
        damping_n_m_s=damping,
        delay_s=delay_s,
        fit_percent=float(fit_pct),
        natural_hz=float(natural),
        damping_eccentricity_correlation=float(correlation),
        separable=bool(separable),
        samples=int(t.size),
        torque_model_scale=1.0,
        tuning_inertia_kg_m2=float(inertia_cg),
        tuning_damping_n_m_s=float(damping),
        inertia_uncertainty_pct=float(uncertainty_pct),
        tuning_inertia_uncertainty_pct=float(uncertainty_pct),
        fit_percent_15hz=float(fit_pct),
        tuning_blockers=tuple(blockers),
        notes=tuple(notes),
        fit_percent_runs=(float(fit_pct),),
        structure="legacy",
    )


def _clip(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _parameter_correlation(jacobian: np.ndarray, index_a: int, index_b: int) -> float:
    """从雅可比估计两个参数的相关系数。

    协方差 ∝ (JᵀJ)⁻¹。奇异时返回 1.0（判为不可分离）——奇异本身就是
    "这两个参数在这段数据里没法分开"的直接证据。
    """
    try:
        gram = jacobian.T @ jacobian
        covariance = np.linalg.inv(gram)
    except np.linalg.LinAlgError:
        return 1.0
    var_a = covariance[index_a, index_a]
    var_b = covariance[index_b, index_b]
    if var_a <= 0.0 or var_b <= 0.0:
        return 1.0
    return float(covariance[index_a, index_b] / math.sqrt(var_a * var_b))


def dominant_hz(t: np.ndarray, y: np.ndarray,
                low: float = 0.1, high: float = 20.0) -> float:
    """主导频率。用来判断这段激励到底激出了什么频段。"""
    if t.size < 64:
        return float("nan")
    fs = 1.0 / float(np.median(np.diff(t)))
    centred = y - float(np.mean(y))
    freq = np.fft.rfftfreq(centred.size, d=1.0 / fs)
    amplitude = np.abs(np.fft.rfft(centred))
    keep = (freq >= low) & (freq <= high)
    if not np.any(keep):
        return float("nan")
    return float(freq[keep][int(np.argmax(amplitude[keep]))])
