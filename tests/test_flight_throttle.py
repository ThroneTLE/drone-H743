"""飞行油门模式（方案 B，doc/flight-throttle-contract.md）。

* 宿主 gcc 编真 C（Driver/Src/drv_flight_throttle.c，无 HAL/RTOS，不需要桩），逐条覆盖契约"验证顺序 1"：
  分区边界、起飞需连续 0.30 s、SPOOLUP 中止回 LANDED、斜坡 1.0 s 到 0.85*hover、两种落地判定的时间阈值、
  测距无效时只认收杆判定、角度模式 t=0.5 -> hover 且单调连续、LEGACY/旁路输出旧语义标志；
* 源码契约：LANDED 时直通分支电机为最小、旁路条件齐全、THRMODE 拒绝解锁切换、体积闸门。
"""
from __future__ import annotations

import ctypes
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "Driver/Src/drv_flight_throttle.c"
STAB = ROOT / "App/Src/app_stabilizer.c"
CMD = ROOT / "App/Src/app_cmd_thrmode.c"

DISARMED, LANDED, SPOOLUP, FLYING = range(4)
LOW, DESCEND, HOLD, CLIMB = range(4)
HOVER = 14.25
MAXF = 20.4
DT = 0.002


class In(ctypes.Structure):
    _fields_ = [
        ("dt_s", ctypes.c_float), ("armed", ctypes.c_uint8), ("link_ok", ctypes.c_uint8),
        ("throttle_01", ctypes.c_float), ("mode_switch_01", ctypes.c_float),
        ("height_valid", ctypes.c_uint8), ("height_m", ctypes.c_float),
        ("vz_m_s", ctypes.c_float), ("imu_valid", ctypes.c_uint8),
        ("bypass", ctypes.c_uint8), ("hover_thrust_n", ctypes.c_float),
        ("spool_start_force_n", ctypes.c_float), ("max_thrust_n", ctypes.c_float),
        ("v_up_m_s", ctypes.c_float), ("v_dn_m_s", ctypes.c_float),
    ]


class Out(ctypes.Structure):
    _fields_ = [
        ("state", ctypes.c_int), ("zone", ctypes.c_int), ("legacy_semantics", ctypes.c_uint8),
        ("active", ctypes.c_uint8), ("climb_rate_m_s", ctypes.c_float),
        ("manual_force_valid", ctypes.c_uint8), ("manual_force_n", ctypes.c_float),
        ("ground_spin_01", ctypes.c_float), ("xy_mode", ctypes.c_int),
        ("manual_integrate", ctypes.c_uint8),
    ]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("宿主没有 gcc")
    out = tmp_path_factory.mktemp("ft") / "ft.dll"
    subprocess.run([gcc, "-shared", "-O0", "-Wall", "-Werror", f"-I{ROOT / 'Driver/Inc'}",
                    str(SRC), "-o", str(out)], check=True)
    dll = ctypes.CDLL(str(out))
    dll.DRV_FlightThrottle_Update.argtypes = [ctypes.POINTER(In), ctypes.POINTER(Out)]
    dll.DRV_FlightThrottle_ZoneOf.argtypes = [ctypes.c_float]
    dll.DRV_FlightThrottle_ZoneOf.restype = ctypes.c_int
    dll.DRV_FlightThrottle_ClimbRate.argtypes = [ctypes.c_float]
    dll.DRV_FlightThrottle_ClimbRate.restype = ctypes.c_float
    dll.DRV_FlightThrottle_ClimbRateWith.argtypes = [ctypes.c_float] * 3
    dll.DRV_FlightThrottle_ClimbRateWith.restype = ctypes.c_float
    dll.DRV_FlightThrottle_AngleModeForce.argtypes = [ctypes.c_float] * 3
    dll.DRV_FlightThrottle_AngleModeForce.restype = ctypes.c_float
    dll.DRV_FlightThrottle_SetMode.argtypes = [ctypes.c_int]
    dll.DRV_FlightThrottle_GetMode.restype = ctypes.c_int
    dll.DRV_FlightThrottle_GetState.restype = ctypes.c_int
    dll.DRV_FlightThrottle_XyModeOf.argtypes = [ctypes.c_float]
    dll.DRV_FlightThrottle_XyModeOf.restype = ctypes.c_int
    dll.DRV_FlightThrottle_GetXyMode.restype = ctypes.c_int
    dll.DRV_FlightThrottle_ManualForce.argtypes = [ctypes.c_float] * 3
    dll.DRV_FlightThrottle_ManualForce.restype = ctypes.c_float
    return dll


class Sim:
    def __init__(self, lib):
        self.lib = lib
        lib.DRV_FlightThrottle_Init()
        self.i = In(dt_s=DT, armed=1, link_ok=1, throttle_01=0.0, mode_switch_01=0.0, height_valid=1,
                    height_m=0.0, vz_m_s=0.0, imu_valid=1, bypass=0, hover_thrust_n=HOVER,
                    max_thrust_n=MAXF)
        self.o = Out()

    def run(self, seconds, **kw):
        for k, v in kw.items():
            setattr(self.i, k, v)
        for _ in range(int(round(seconds / DT))):
            self.lib.DRV_FlightThrottle_Update(ctypes.byref(self.i), ctypes.byref(self.o))
        return self.o

    def takeoff(self):
        self.run(0.1, throttle_01=0.0)
        self.run(0.4, throttle_01=0.8)          # 0.30 s 迟滞 + 余量
        assert self.o.state == SPOOLUP
        self.run(1.1, throttle_01=0.8)
        assert self.o.state == FLYING


# --------------------------------------------------------------------------- 分区与爬升率
@pytest.mark.parametrize("t,zone", [
    (0.0, LOW), (0.099, LOW), (0.10, DESCEND), (0.399, DESCEND), (0.40, HOLD),
    (0.60, HOLD), (0.6001, CLIMB), (1.0, CLIMB),
])
def test_zone_boundaries(lib, t, zone):
    assert lib.DRV_FlightThrottle_ZoneOf(t) == zone


def test_climb_rate_map(lib):
    r = lib.DRV_FlightThrottle_ClimbRate
    assert r(0.0) == pytest.approx(-0.20)
    assert r(0.05) == pytest.approx(-0.20)
    assert r(0.10) == pytest.approx(-0.20)            # DESCEND 起点与 LOW 连续
    assert r(0.25) == pytest.approx(-0.10)
    assert r(0.40) == pytest.approx(0.0, abs=1e-6)
    assert r(0.50) == 0.0
    assert r(0.60) == 0.0
    assert r(0.80) == pytest.approx(0.15)
    assert r(1.0) == pytest.approx(0.30)


def test_climb_rate_with_configurable_limits(lib):
    """作者 2026-10-01：定高档满杆升/降速度取 coax.pos_z_vel_up/down_max_m_s；分区与线性形状不变。"""
    w = lib.DRV_FlightThrottle_ClimbRateWith
    nan = float("nan")
    assert w(1.0, 1.0, 0.5) == pytest.approx(1.0)
    assert w(0.80, 1.0, 0.5) == pytest.approx(0.50)
    assert w(0.0, 1.0, 0.5) == pytest.approx(-0.5)
    assert w(0.25, 1.0, 0.5) == pytest.approx(-0.25)
    assert w(0.5, 1.0, 0.5) == 0.0
    # 无效值（<=0、非有限）退回 0.30/0.20 常量
    assert w(1.0, 0.0, -1.0) == pytest.approx(0.30)
    assert w(0.0, nan, nan) == pytest.approx(-0.20)
    # 状态机输出走同一份限速
    s = Sim(lib)
    s.i.v_up_m_s = 0.6
    s.i.v_dn_m_s = 0.4
    s.run(0.1, armed=1, throttle_01=1.0)
    assert s.o.climb_rate_m_s == pytest.approx(0.6)
    s.run(0.1, armed=1, throttle_01=0.0)
    assert s.o.climb_rate_m_s == pytest.approx(-0.4)


# --------------------------------------------------------------------------- 状态机
def test_disarmed_until_armed_then_landed(lib):
    s = Sim(lib)
    s.run(0.1, armed=0)
    assert s.o.state == DISARMED and s.o.active == 0
    s.run(0.1, armed=1)
    assert s.o.state == LANDED and s.o.active == 0 and s.o.legacy_semantics == 0
    s.run(0.1, link_ok=0)
    assert s.o.state == DISARMED


def test_takeoff_needs_continuous_030s(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0)
    s.run(0.28, throttle_01=0.8)
    assert s.o.state == LANDED
    s.run(0.02, throttle_01=0.5)                     # 掉出 CLIMB 区 -> 计时清零
    s.run(0.28, throttle_01=0.8)
    assert s.o.state == LANDED
    s.run(0.04, throttle_01=0.8)
    assert s.o.state == SPOOLUP


def test_takeoff_needs_imu(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0)
    s.run(1.0, throttle_01=0.8, imu_valid=0)
    assert s.o.state == LANDED
    s.run(0.4, imu_valid=1)
    assert s.o.state == SPOOLUP


def test_takeoff_allowed_without_range(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0, height_valid=0)
    s.run(0.4, throttle_01=0.8)
    assert s.o.state == SPOOLUP


def test_spoolup_ramp_to_085_hover_in_1s(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0)
    s.i.throttle_01 = 0.8
    n = 0
    while s.o.state != SPOOLUP:                      # 逐拍等进入 SPOOLUP
        s.run(DT)
        n += 1
        assert n < 1000
    assert s.o.manual_force_valid == 1 and s.o.active == 1
    assert s.o.manual_force_n == pytest.approx(0.0, abs=0.01)
    s.run(0.5)
    assert s.o.manual_force_n == pytest.approx(0.85 * HOVER * 0.5, rel=0.02)
    s.run(0.49)
    assert s.o.state == SPOOLUP
    assert s.o.manual_force_n == pytest.approx(0.85 * HOVER, rel=0.02)
    s.run(0.02)
    assert s.o.state == FLYING and s.o.manual_force_valid == 0 and s.o.active == 1


def test_spoolup_ramp_starts_at_ground_spin_force(lib):
    s = Sim(lib)
    s.i.spool_start_force_n = 0.4
    s.run(0.1, throttle_01=0.0)
    s.i.throttle_01 = 0.8
    n = 0
    while s.o.state != SPOOLUP:
        s.run(DT)
        n += 1
        assert n < 1000
    assert s.o.manual_force_n == pytest.approx(0.4, abs=0.02)          # 起点＝地面慢转上限推力，不从 0 掉一下
    s.run(0.5)
    assert s.o.manual_force_n == pytest.approx(0.4 + (0.85 * HOVER - 0.4) * 0.5, rel=0.02)
    s.run(0.49)
    assert s.o.manual_force_n == pytest.approx(0.85 * HOVER, rel=0.02)


def test_spoolup_abort_when_stick_leaves_climb(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0)
    s.run(0.31, throttle_01=0.8)
    s.run(0.4, throttle_01=0.8)
    assert s.o.state == SPOOLUP
    s.run(0.01, throttle_01=0.6)                     # 0.60 属 HOLD，不是 CLIMB
    assert s.o.state == LANDED and s.o.active == 0 and s.o.manual_force_valid == 0
    # 中止后不会自己再起飞：杆位仍在 HOLD
    s.run(1.0, throttle_01=0.5)
    assert s.o.state == LANDED


def test_flying_stick_modes_and_climb_rate(lib):
    s = Sim(lib)
    s.takeoff()
    s.run(0.1, throttle_01=0.8, height_m=0.3)
    assert s.o.climb_rate_m_s == pytest.approx(0.15)
    s.run(0.1, throttle_01=0.5)
    assert s.o.climb_rate_m_s == 0.0


def test_landing_rule1_hover_stick_low_height(lib):
    s = Sim(lib)
    s.takeoff()
    s.run(0.2, throttle_01=0.5, height_m=0.30)       # 在空中
    s.run(0.98, throttle_01=0.30, height_m=0.05, vz_m_s=0.0)   # DESCEND，爬升率<0
    assert s.o.state == FLYING
    s.run(0.04, throttle_01=0.30, height_m=0.05, vz_m_s=0.0)
    assert s.o.state == LANDED


def test_landing_rule1_needs_slow_vz_and_not_climbing(lib):
    s = Sim(lib)
    s.takeoff()
    s.run(2.0, throttle_01=0.30, height_m=0.05, vz_m_s=-0.15)   # 还在掉 -> 不判
    assert s.o.state == FLYING
    s.run(2.0, throttle_01=0.70, height_m=0.05, vz_m_s=0.0)     # 在爬升 -> 不判
    assert s.o.state == FLYING
    s.run(2.0, throttle_01=0.30, height_m=0.07, vz_m_s=0.0)     # 高度够高 -> 不判
    assert s.o.state == FLYING


def test_landing_rule1_not_in_angle_mode(lib):
    s = Sim(lib)
    s.takeoff()
    s.run(3.0, throttle_01=0.30, height_m=0.05, vz_m_s=0.0, mode_switch_01=1.0)
    assert s.o.state == FLYING


def test_landing_rule1_also_in_velocity_hold(lib):
    """CH6 中位（速度保持）也定高，判据 1 照样生效。"""
    s = Sim(lib)
    s.takeoff()
    s.run(0.98, throttle_01=0.30, height_m=0.05, vz_m_s=0.0, mode_switch_01=0.5)
    assert s.o.state == FLYING and s.o.xy_mode == XY_VEL
    s.run(0.04, throttle_01=0.30, height_m=0.05, vz_m_s=0.0, mode_switch_01=0.5)
    assert s.o.state == LANDED


def test_landing_rule2_stick_cut_any_mode(lib):
    for sw in (0.0, 0.5, 1.0):
        s = Sim(lib)
        s.takeoff()
        s.run(0.48, throttle_01=0.05, height_m=0.14, vz_m_s=-0.5, mode_switch_01=sw)
        assert s.o.state == FLYING
        s.run(0.04, throttle_01=0.05, height_m=0.14, vz_m_s=-0.5, mode_switch_01=sw)
        assert s.o.state == LANDED


def test_landing_rule2_height_threshold_and_stick(lib):
    s = Sim(lib)
    s.takeoff()
    s.run(2.0, throttle_01=0.05, height_m=0.16, vz_m_s=-0.5)    # 高于 0.15 m 不判
    assert s.o.state == FLYING
    s.run(2.0, throttle_01=0.12, height_m=0.14, vz_m_s=-0.5)    # 杆没收到 <0.10 不判
    assert s.o.state == FLYING


def test_no_range_only_stick_cut_rule(lib):
    s = Sim(lib)
    s.takeoff()
    # 测距无效：规则 1 不成立，规则 2 在相对高度未知时也不成立 -> 不会空中误判
    s.run(3.0, throttle_01=0.0, height_valid=0, height_m=0.0)
    assert s.o.state == FLYING
    s.run(3.0, throttle_01=0.3, height_valid=0, height_m=0.0)
    assert s.o.state == FLYING


def test_imu_loss_drops_to_landed(lib):
    s = Sim(lib)
    s.takeoff()
    s.run(0.01, imu_valid=0)
    assert s.o.state == LANDED and s.o.active == 0


def test_relaunch_after_landing_by_stick(lib):
    s = Sim(lib)
    s.takeoff()
    s.run(0.6, throttle_01=0.05, height_m=0.05)
    assert s.o.state == LANDED
    s.run(0.5, throttle_01=0.8)                      # 落地过程中杆已在 LOW，再推杆可重新起飞
    assert s.o.state == SPOOLUP


# --------------------------------------------------------------------------- 角度模式推力
def test_angle_mode_force_midpoint_is_hover_and_monotonic(lib):
    f = lambda t: lib.DRV_FlightThrottle_AngleModeForce(t, HOVER, MAXF)
    assert f(0.5) == pytest.approx(HOVER)
    assert f(0.0) == pytest.approx(0.0)
    assert f(1.0) == pytest.approx(MAXF)
    prev = -1.0
    for i in range(0, 1001):
        v = f(i / 1000.0)
        assert v >= prev - 1e-6
        assert abs(v - prev) < 0.05 or prev < 0     # 连续：相邻 0.001 步长跳变很小
        prev = v
    # 上限不足悬停时不会比 hover 小
    assert lib.DRV_FlightThrottle_AngleModeForce(1.0, HOVER, 1.0) == pytest.approx(HOVER)


# --------------------------------------------------------------------------- LEGACY / 旁路
def test_legacy_mode_reports_legacy_semantics(lib):
    s = Sim(lib)
    lib.DRV_FlightThrottle_SetMode(1)
    try:
        assert lib.DRV_FlightThrottle_GetMode() == 1
        o = s.run(5.0, throttle_01=0.9)
        assert o.legacy_semantics == 1 and o.active == 0 and o.manual_force_valid == 0
        assert o.state == LANDED                       # 状态机不跟着杆位起飞
    finally:
        lib.DRV_FlightThrottle_SetMode(0)


def test_bypass_reports_legacy_semantics_and_resets_to_landed(lib):
    s = Sim(lib)
    s.takeoff()
    o = s.run(0.01, bypass=1)
    assert o.legacy_semantics == 1 and o.active == 0 and o.state == LANDED
    # 旁路结束：强制 LANDED，杆还在 CLIMB 区时不会立刻起飞（起飞闭锁）
    o = s.run(1.0, bypass=0, throttle_01=0.9)
    assert o.legacy_semantics == 0 and o.state == LANDED
    s.run(0.1, throttle_01=0.3)
    o = s.run(0.4, throttle_01=0.9)
    assert o.state == SPOOLUP


# --------------------------------------------------------------------------- 源码契约
def _src(p: Path) -> str:
    return p.read_text(encoding="utf-8")


def test_driver_has_no_hal_or_rtos():
    for p in (SRC, ROOT / "Driver/Inc/drv_flight_throttle.h"):
        t = _src(p)
        assert "stm32" not in t.lower() and "cmsis" not in t.lower() and "FreeRTOS" not in t


def test_landed_direct_branch_uses_capped_ground_spin_not_raw_stick():
    # 作者 2026-10-01 选"只跟杆慢转"：解锁不转、推杆转速慢慢上来，上限 30%（飞不起来）；杆位不直接当油门。
    t = _src(STAB)
    i = t.index("uint16_t direct_us = frame->rc_throttle_motor_us;")
    seg = t[i:i + 1400]
    spin = "direct_us = stabilizer_rc_throttle_to_motor_pulse(frame->ft_ground_spin_01);"
    assert "if (frame->ft_landed_idle != 0U) {" in seg
    assert spin in seg
    assert seg.index(spin) < seg.index("BSP_PWM_SetEscPulse(1, direct_us);")
    h = _src(ROOT / "Driver/Inc/drv_flight_throttle.h")
    assert "#define DRV_FLIGHT_THROTTLE_GROUND_MIN_01  0.05f" in h
    assert "#define DRV_FLIGHT_THROTTLE_GROUND_MAX_01  0.30f" in h
    assert "IDLE_01" not in h and "IDLE_01" not in t
    assert "frame->ft_ground_spin_01 = ft_out.ground_spin_01;" in t
    assert "ft_in.spool_start_force_n = DRV_COAX_CTRL_MotorPulseToTotalThrust(" in t
    assert "APP_FLIGHT_LOG_MOTOR_REASON_LANDED_IDLE" in seg
    assert "APP_FLIGHT_LOG_MOTOR_REASON_SPOOLUP" in t


def test_bypass_conditions_complete():
    t = _src(STAB)
    i = t.index("ft_in.bypass =")
    seg = t[i:i + 400]
    for cond in ("frame->ident_running", "frame->servo_cal_active",
                 "APP_ThrustBench_IsActive()", "APP_PropSpin_IsActive()"):
        assert cond in seg
    # sysid 包含在 ident_running 里（见 ident_running 的定义）
    assert "(frame->sysid_running != 0U)) ? 1U : 0U;" in t
    # 旧语义时一个标志都不覆盖
    assert "if (ft_out.legacy_semantics == 0U) {" in t
    assert "stabilizer_rc_use_stabilized_motor_mix(frame->rc.throttle_01);" in t


def test_thrmode_rejects_switch_when_armed_and_stays_out_of_big_files():
    c = _src(CMD)
    assert "THRMODE state=rejected reason=armed" in c
    assert c.index("reason=armed") < c.index("DRV_FlightThrottle_SetMode(mode)")
    assert "APP_Stabilizer_IsArmed()" in c
    assert "%f" not in c
    assert "app_control_handle_thrmode" in _src(ROOT / "App/Src/app_cmd_fallback.c")
    assert "app_cmd_thrmode.c" in _src(ROOT / "CMakeLists.txt")
    assert "drv_flight_throttle.c" in _src(ROOT / "CMakeLists.txt")
    assert "THRMODE" not in _src(ROOT / "App/Src/app_control.c")
    assert "THRMODE" not in _src(ROOT / "tools/drone_tcp_panel.py")


def test_ai_bridge_allows_query_only():
    import sys
    sys.path.insert(0, str(ROOT / "tools"))
    from panel_lib import ai_bridge_policy as pol
    assert pol.classify("THRMODE?").kind == pol.ALLOW
    assert pol.classify("THRMODE FLIGHT").kind == pol.DENY
    assert pol.classify("THRMODE LEGACY").kind == pol.DENY


def test_ground_spin_mapping_follows_stick_with_cap(lib):
    lib.DRV_FlightThrottle_GroundSpin01.restype = ctypes.c_float
    lib.DRV_FlightThrottle_GroundSpin01.argtypes = [ctypes.c_float]
    g = lib.DRV_FlightThrottle_GroundSpin01
    assert g(0.0) == 0.0 and g(0.099) == 0.0                     # 解锁（杆最低）不转
    assert g(0.10) == pytest.approx(0.05, abs=1e-4)
    assert g(0.35) == pytest.approx(0.175, abs=1e-3)
    assert g(0.60) == pytest.approx(0.30, abs=1e-4)
    assert g(1.0) == pytest.approx(0.30, abs=1e-4)                # 上限 30%
    vals = [g(x / 100) for x in range(10, 101)]
    assert all(b >= a for a, b in zip(vals, vals[1:]))


def test_ground_spin_only_in_landed(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0)
    assert s.o.state == LANDED and s.o.ground_spin_01 == 0.0
    s.run(0.1, throttle_01=0.35)
    assert s.o.state == LANDED and s.o.ground_spin_01 == pytest.approx(0.175, abs=1e-3)
    s.i.throttle_01 = 0.8
    n = 0
    while s.o.state != SPOOLUP:
        s.run(DT)
        n += 1
        assert n < 1000
    assert s.o.ground_spin_01 == 0.0                              # 起旋后由 manual_force 接管


# --------------------------------------------------------------------------- CH6 三段拨码
XY_POS, XY_VEL, XY_ANGLE = 0, 1, 2


@pytest.mark.parametrize("sw,mode", [
    (0.0, XY_POS), (0.30, XY_POS), (0.3332, XY_POS), (0.34, XY_VEL), (0.5, XY_VEL),
    (0.503, XY_VEL), (0.66, XY_VEL), (0.67, XY_ANGLE), (1.0, XY_ANGLE),
    (float("nan"), XY_POS), (-0.2, XY_POS), (1.3, XY_ANGLE),
])
def test_ch6_three_position_decode(lib, sw, mode):
    """作者"CH6是三段拨码"，中位实测 1503 us -> 0.503 落在速度保持。"""
    assert lib.DRV_FlightThrottle_XyModeOf(sw) == mode


def test_xy_mode_reported_in_every_state_and_switchable_in_flight(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0, mode_switch_01=0.5)
    assert s.o.state == LANDED and s.o.xy_mode == XY_VEL
    assert lib.DRV_FlightThrottle_GetXyMode() == XY_VEL
    s.takeoff()
    for sw, mode in ((0.0, XY_POS), (0.5, XY_VEL), (1.0, XY_ANGLE), (0.5, XY_VEL)):
        s.run(0.01, mode_switch_01=sw, throttle_01=0.5, height_m=0.3)
        assert s.o.state == FLYING and s.o.xy_mode == mode


def test_velocity_hold_glue_follows_position_without_p_term():
    c = _src(ROOT / "App/Src/app_stabilizer.c")
    # 开关量：已绑定才取 norm 映射到 [0,1]，未绑定给 0（定点）。
    assert "0.5f * (frame->rc.norm[APP_RC_FUNC_MODE] + 1.0f) : 0.0f;" in c
    # FLIGHT 下角度模式只认三档的高位；LEGACY 仍是旧 50% 判定（逐位旧行为）。
    assert "(ft_out.xy_mode == DRV_FLIGHT_THROTTLE_XY_ANGLE)) ? 1U : 0U;" in c
    assert c.count("STABILIZER_RC_SWITCH_HIGH_PERCENT) != 0U)) ? 1U : 0U;") == 1
    # 速度保持：位置参考每拍跟到估计（位置误差 0），ready 清零以便切回定点时重新锁位置。
    i = c.index("if ((velocity_loop_enabled != 0U) && (frame->ft_xy_velocity_hold != 0U)) {")
    seg = c[i:i + 700]
    assert "ctx->position_ref_x_m = position_state_x_m;" in seg
    assert "ctx->position_ref_y_m = position_state_y_m;" in seg
    assert "ctx->position_ref_xy_ready = 0U;" in seg
    t = _src(ROOT / "App/Src/app_cmd_thrmode.c")
    assert "xy=%s" in t and '"pos", "vel", "angle"' in t


# --------------------------------------------------------------------------- 角度档手控油门（PX4 自稳式）
def _angle_force(lib, t):
    return lib.DRV_FlightThrottle_AngleModeForce(max(t, 0.10), HOVER, MAXF)


def test_angle_mode_takes_off_as_soon_as_stick_passes_low_without_spoolup(lib):
    """作者"我想让它更像 px4那个一样手控油门"：无 60%/0.3 s 门槛、无 1 s 斜坡，推力直接按杆。"""
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0, mode_switch_01=1.0)
    assert s.o.state == LANDED and s.o.manual_force_valid == 0
    s.run(DT, throttle_01=0.15)
    assert s.o.state == FLYING                        # 一拍内直接起转，不经 SPOOLUP
    assert s.o.manual_force_valid == 1
    assert s.o.manual_force_n == pytest.approx(_angle_force(lib, 0.15), rel=1e-5)
    s.run(0.2, throttle_01=0.5)
    assert s.o.manual_force_n == pytest.approx(HOVER, rel=1e-5)
    s.run(0.2, throttle_01=1.0)
    assert s.o.manual_force_n == pytest.approx(MAXF, rel=1e-5)


def test_angle_mode_needs_stick_through_low_after_landing_or_switching(lib):
    """LANDED 时拨到角度档、杆在中位：不能一拨就按中位推力起转，先把杆拉到底。"""
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0)                      # 定点档解锁
    s.run(0.2, throttle_01=0.5, mode_switch_01=1.0)  # 杆在中位才拨到角度档
    assert s.o.state == LANDED and s.o.manual_force_valid == 0
    assert s.o.ground_spin_01 > 0.0                   # 仍是地面慢转
    s.run(0.05, throttle_01=0.0)
    s.run(DT, throttle_01=0.5)
    assert s.o.state == FLYING and s.o.manual_force_n == pytest.approx(HOVER, rel=1e-5)


def test_angle_mode_stick_cut_in_air_keeps_floor_thrust_until_landed(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0, mode_switch_01=1.0)
    s.run(1.0, throttle_01=0.5, height_m=1.0)
    s.run(2.0, throttle_01=0.0, height_m=1.0)        # 空中杆到底：不停桨
    assert s.o.state == FLYING
    assert s.o.manual_force_n == pytest.approx(_angle_force(lib, 0.10), rel=1e-5)
    assert s.o.manual_force_n > 0.15 * HOVER
    s.run(0.48, throttle_01=0.0, height_m=0.10)      # 贴地 + 杆在底：0.5 s 后才落地
    assert s.o.state == FLYING
    s.run(0.04, throttle_01=0.0, height_m=0.10)
    assert s.o.state == LANDED and s.o.manual_force_valid == 0 and s.o.ground_spin_01 == 0.0


def test_angle_mode_integrator_gate_follows_thrust(lib):
    s = Sim(lib)
    s.run(0.1, throttle_01=0.0, mode_switch_01=1.0)
    s.run(0.1, throttle_01=0.30)                     # 0.6·hover：地面卷积分风险，不开
    assert s.o.state == FLYING and s.o.manual_integrate == 0
    s.run(0.1, throttle_01=0.40)                     # 0.8·hover：开
    assert s.o.manual_integrate == 1
    s.run(0.1, throttle_01=0.6)
    assert s.o.manual_integrate == 1
    s.run(0.1, mode_switch_01=0.0)                   # 切回定点：手动推力与该标志都撤
    assert s.o.manual_force_valid == 0 and s.o.manual_integrate == 0


def test_position_and_velocity_modes_still_use_spoolup(lib):
    for sw in (0.0, 0.5):
        s = Sim(lib)
        s.run(0.1, throttle_01=0.0, mode_switch_01=sw)
        s.run(0.2, throttle_01=0.8)
        assert s.o.state == LANDED                    # 0.30 s 迟滞未到
        s.run(0.2, throttle_01=0.8)
        assert s.o.state == SPOOLUP


def test_manual_force_floor(lib):
    f = lib.DRV_FlightThrottle_ManualForce
    assert f(0.0, HOVER, MAXF) == pytest.approx(f(0.10, HOVER, MAXF))
    assert f(0.5, HOVER, MAXF) == pytest.approx(HOVER)
    assert f(float("nan"), HOVER, MAXF) == pytest.approx(f(0.10, HOVER, MAXF))


def test_angle_mode_glue_uses_state_machine_force_and_integrator_gate():
    c = _src(ROOT / "App/Src/app_stabilizer.c")
    assert "frame->ft_manual_force_n :    /* 角度档手控油门" in c
    assert "DRV_FlightThrottle_AngleModeForce(" not in c   # 推力只由状态机算一处
    assert "ft_in.max_thrust_n = DRV_COAX_CTRL_MotorPulseToTotalThrust(BSP_PWM_ESC_MAX_US);" in c
    assert "frame->ft_manual_integrate = ft_out.manual_integrate;" in c
    gate = c.split("schedule.integrator_enable =")[1].split(";")[0]
    assert "(frame->rc_attitude_debug_mode != 0U) &&" in gate
    assert "(frame->ft_manual_integrate != 0U)" in gate


def test_position_mode_brakes_before_locking_after_stick_release():
    """定点档松杆"先刹停再锁位"（同 PX4）：作者"要定点就能定点，要移动就最快响应"；自由飞数据仿真旧做法松杆过冲约 0.35 m。"""
    c = _src(ROOT / "App/Src/app_stabilizer.c")
    assert "#define STABILIZER_XY_LOCK_SPEED_M_S   0.15f" in c
    assert "#define STABILIZER_XY_BRAKE_MAX_S      2.0f" in c
    i = c.index("(ctx->xy_brake_pending != 0U))) {")
    seg = c[i:i + 2200]
    # 打杆或刹车中：位置参考跟到估计（无位置 P），锁位分支在其后
    assert seg.index("ctx->position_ref_x_m = position_state_x_m;") < seg.index("} else if (velocity_loop_enabled != 0U) {")
    assert "(STABILIZER_XY_LOCK_SPEED_M_S * STABILIZER_XY_LOCK_SPEED_M_S)) ||" in seg
    assert "(ctx->xy_brake_elapsed_s >= STABILIZER_XY_BRAKE_MAX_S)) {" in seg
    # 速度保持档切回定点也先刹车
    vh = c.index("if ((velocity_loop_enabled != 0U) && (frame->ft_xy_velocity_hold != 0U)) {")
    assert "ctx->xy_brake_pending = 1U;" in c[vh:i]
