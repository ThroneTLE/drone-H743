"""转速陷波的策略层与命令面（App/Src/app_rpm_notch.c + app_cmd_rpmnotch.c）—— 宿主上跑真代码。

平台边界（电调快照、名义 ODR、时钟、解锁/辨识占用、文本回复）在
tests/fixtures/rpm_notch/app_stubs.c，全部由测试控制。判据：

* **关着就是原样**：关、或没有双向回传时，控制用陀螺逐位等于原始陀螺；关着也照样发布转速。
* **新鲜度判得对**：20 ms 内算新鲜；回包比本拍时间晚 1 ms（sample_ms = now + 1）也算新鲜——
  无符号相减会回绕成约 4e9 被判过期，这是规格里点名要防的坑。
* **采样率按时间戳估**：名义值作种子，偏 6% 判 fs_bad 淡出直通；丢一个样本补中点，大缺口复位。
* **陷波放对地方**：实测 fs、极对数真的决定中心（量线被压了多少，不只看状态里报的数）；
  复位真的把滤波组拉回当前输入的稳态（不只看计数）。
* **状态读得完整**：读者拷副本途中控制拍发布过就重拷；权重与跟踪统计看掩码里最低的谐波。
* **命令面**：解锁/辨识中拒绝，范围与用法拒绝，每行 < 255 字符、只用整数格式。
"""
from __future__ import annotations

import ctypes
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    "App/Src/app_rpm_notch.c",
    "App/Src/app_cmd_rpmnotch.c",
    "Driver/Src/drv_rpm_notch.c",
    "Driver/Src/drv_dshot_telemetry.c",
    "tests/fixtures/rpm_notch/app_stubs.c",
]
INCLUDES = ["App/Inc", "Driver/Inc", "BSP/Inc", "Services/Inc"]
ROD = ROOT / "data" / "identification" / "attitude" / "2026-09-27" / "rod_035506_982cc822"

STATE_OFF, STATE_UNAVAILABLE, STATE_FS_UNKNOWN, STATE_FS_BAD, STATE_IDLE, STATE_TRACKING = range(6)
F = ctypes.c_float
U8 = ctypes.c_uint8
U16 = ctypes.c_uint16
U32 = ctypes.c_uint32
U64 = ctypes.c_uint64
FP = ctypes.POINTER(F)


class Config(ctypes.Structure):
    _fields_ = [("enable", U8), ("pole_pairs", U8), ("harmonic_mask", U8),
                ("q", F), ("min_hz", F), ("fade_hz", F)]


class Motors(ctypes.Structure):
    _fields_ = [("erpm", U32 * 2), ("hz", F * 2), ("age_ms", U32 * 2), ("fresh", U8 * 2),
                ("spinning", U8 * 2), ("available", U8)]


class Status(ctypes.Structure):
    _fields_ = [("cfg", Config), ("motors", Motors), ("state", ctypes.c_int),
                ("fs_nominal_hz", U16), ("fs_hz", F), ("weight_base", F * 2), ("active_slots", U8)] + [
        (name, U32) for name in ("stale", "gap1", "reset", "nonfinite", "slewclamp", "reject", "wdog",
                                 "samples", "spin", "tracked", "apply_us_sum", "apply_count",
                                 "apply_us_max", "tick_us_sum", "tick_count", "tick_us_max")]


def build_policy_lib(tmp_path_factory, sources=SOURCES, name="rpm-notch-policy"):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the rpm notch policy contract")
    build = tmp_path_factory.mktemp(name)
    stub = build / "stub"
    stub.mkdir()
    (stub / "cmsis_os2.h").write_text("typedef void *osSemaphoreId_t; typedef void *osMessageQueueId_t;\n")
    out = build / ("policy.dll" if os.name == "nt" else "policy.so")
    includes = ["-I", str(stub)]
    for path in INCLUDES:
        includes += ["-I", str(ROOT / path)]
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror",
         "-DBSP_ESC_PROTOCOL=2", *includes, *[str(ROOT / path) if not Path(path).is_absolute()
                                              else str(path) for path in sources],
         "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    lib = ctypes.CDLL(str(out))
    lib.APP_RpmNotch_ApplySample.argtypes = [FP, FP, U64]
    lib.APP_RpmNotch_GetStatus.argtypes = [ctypes.POINTER(Status)]
    lib.APP_RpmNotch_GetMotors.argtypes = [ctypes.POINTER(Motors)]
    lib.APP_RpmNotch_GetConfig.argtypes = [ctypes.POINTER(Config)]
    lib.APP_RpmNotch_DefaultConfig.argtypes = [ctypes.POINTER(Config)]
    lib.APP_RpmNotch_RequestConfig.argtypes = [ctypes.POINTER(Config)]
    lib.APP_RpmNotch_RequestConfig.restype = U8
    lib.APP_RpmNotch_ConfigValid.argtypes = [ctypes.POINTER(Config)]
    lib.APP_RpmNotch_ConfigValid.restype = U8
    lib.APP_RpmNotch_ReportProvenance.argtypes = [U16]
    lib.stub_set_rotor.argtypes = [U32, U32, U32, U8, U8]
    lib.stub_set_available.argtypes = [U8]
    lib.stub_set_odr.argtypes = [U16]
    lib.stub_set_ms.argtypes = [U32]
    lib.stub_get_ms.restype = U32
    lib.stub_set_us.argtypes = [U64]
    lib.stub_set_us_step.argtypes = [U32]
    lib.stub_set_armed.argtypes = [U8]
    lib.stub_set_sysid.argtypes = [U8]
    lib.stub_tick_at_barrier.argtypes = [U32]
    lib.stub_text_lines.restype = U32
    lib.stub_text_line.argtypes = [U32]
    lib.stub_text_line.restype = ctypes.c_char_p
    lib.stub_text_max.restype = U32
    lib.stub_command.argtypes = [ctypes.c_char_p]
    lib.stub_command.restype = U8
    lib.stub_run.argtypes = [U32, ctypes.POINTER(U64), FP, FP, ctypes.POINTER(U8),
                             ctypes.POINTER(U32), ctypes.POINTER(U32)]
    return lib


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build_policy_lib(tmp_path_factory)


# ---------------------------------------------------------------- 驱动小工具


def reset(lib, *, enable=1, available=1, odr=1000, pp=7, harm=1, q=3.0):
    lib.stub_reset()
    lib.stub_set_available(available)
    lib.stub_set_odr(odr)
    lib.APP_RpmNotch_Init()
    cfg = Config()
    lib.APP_RpmNotch_DefaultConfig(ctypes.byref(cfg))
    cfg.enable, cfg.pole_pairs, cfg.harmonic_mask, cfg.q = enable, pp, harm, q
    assert lib.APP_RpmNotch_RequestConfig(ctypes.byref(cfg)) == 1
    lib.APP_RpmNotch_Tick()
    lib.stub_clear_text()


def status(lib) -> Status:
    s = Status()
    lib.APP_RpmNotch_GetStatus(ctypes.byref(s))
    return s


def texts(lib) -> list[str]:
    return [lib.stub_text_line(i).decode() for i in range(lib.stub_text_lines())]


def command(lib, line: str) -> list[str]:
    lib.stub_clear_text()
    assert lib.stub_command(line.encode()) == 1
    return texts(lib)


def run(lib, signal, *, period_us=1000.0, erpm=(45150, 46000), start_us=10_000_000,
        timestamps=None, tick_every=None, seed=1, ms_offset=0):
    """按 IMU 样本喂（默认 1 kHz），控制拍每 2～3 个样本一次；每拍把两路 eRPM 写成新鲜回包。"""
    signal = np.ascontiguousarray(signal, np.float32)
    n = signal.shape[0]
    if timestamps is None:
        timestamps = start_us + np.round(np.arange(n) * period_us).astype(np.uint64)
    timestamps = np.ascontiguousarray(timestamps, np.uint64)
    rng = np.random.default_rng(seed)
    ticks = np.zeros(n, np.uint8)
    i = 0
    while i < n:
        ticks[i] = 1
        i += tick_every if tick_every else int(rng.integers(2, 4))
    ticks[-1] = 1                               # 最后补一拍：状态是在控制拍里发布的
    ms = np.ascontiguousarray(timestamps // 1000 + ms_offset, np.uint32)
    rotor = None
    if erpm is not None:
        rotor = np.ascontiguousarray(np.tile(np.asarray(erpm, np.uint32), (n, 1)))
    out = np.zeros_like(signal)
    lib.stub_run(n, timestamps.ctypes.data_as(ctypes.POINTER(U64)), signal.ctypes.data_as(FP),
                 out.ctypes.data_as(FP), ticks.ctypes.data_as(ctypes.POINTER(U8)),
                 ms.ctypes.data_as(ctypes.POINTER(U32)),
                 rotor.ctypes.data_as(ctypes.POINTER(U32)) if rotor is not None else None)
    return out


def spin_both(lib, erpm=(45150, 46000), ms=None):
    now = lib.stub_get_ms() if ms is None else ms
    for index, value in enumerate(erpm):
        lib.stub_set_rotor(index, value, now, 1, 0)


def line_signal(n, freqs=(107.5,), amplitude=0.1, dc=(0.2, -0.1, 0.05), fs=1000.0):
    t = np.arange(n) / fs
    line = sum(amplitude * np.sin(2 * np.pi * f * t + k) for k, f in enumerate(freqs))
    return (np.asarray(dc)[None, :] + line[:, None]).astype(np.float32)


def attenuation_db(signal, out, tail, dc=0.2):
    """轴 0 上的线（扣掉 line_signal 的直流 0.2）被压了多少 dB。"""
    line = np.asarray(signal[tail, 0], np.float64) - dc
    left = np.asarray(out[tail, 0], np.float64) - dc
    return 20.0 * math.log10(float(np.sqrt(np.mean(line ** 2))) / float(np.sqrt(np.mean(left ** 2))))


def spin_one(lib, erpm):
    """只有电调 1 有回包（电调 2 从未收到）：单个陷波，位置误差不会被第二个陷波掩盖。"""
    lib.stub_set_rotor(0, erpm, lib.stub_get_ms(), 1, 0)


# ---------------------------------------------------------------- 关着就是原样


def _rod_gyro():
    if not (ROD / "samples.csv").is_file():
        pytest.skip("实录不在本机")
    raw = np.genfromtxt(ROD / "samples.csv", delimiter=",", names=True)
    return np.stack([raw["gx"], raw["gy"], raw["gz"]], axis=1).astype(np.float32)


@pytest.mark.parametrize("case", ["off", "unavailable"])
def test_off_or_without_speed_is_bitwise_over_the_real_rod_log(lib, case):
    gyro = _rod_gyro()
    reset(lib, enable=0 if case == "off" else 1, available=1 if case == "off" else 0)
    lib.stub_set_rotor(0, 45150, lib.stub_get_ms(), 1, 0)
    lib.stub_set_rotor(1, 46000, lib.stub_get_ms(), 1, 0)
    out = run(lib, gyro)
    assert out.tobytes() == gyro.tobytes()
    s = status(lib)
    assert s.state == (STATE_OFF if case == "off" else STATE_UNAVAILABLE)
    if case == "off":
        # 关着也发布转速：状态行与辨识溯源照样看得到。
        assert list(s.motors.erpm) == [45150, 46000] and s.motors.available == 1
        assert s.motors.hz[0] == pytest.approx(107.5, abs=0.02)
    assert s.samples == len(gyro)


def test_non_bidir_build_reports_unavailable_and_bypasses(lib):
    reset(lib, available=0)
    signal = line_signal(500)
    assert run(lib, signal).tobytes() == signal.tobytes()
    s = status(lib)
    assert s.state == STATE_UNAVAILABLE and s.motors.available == 0
    assert list(s.motors.age_ms) == [9999, 9999]


# ---------------------------------------------------------------- 新鲜度


@pytest.mark.parametrize("offset,fresh", [(-20, True), (-21, False), (1, True), (0, True)])
def test_freshness_window_is_signed_and_twenty_ms(lib, offset, fresh):
    reset(lib)
    now = lib.stub_get_ms()
    for index in (0, 1):
        lib.stub_set_rotor(index, 45150, (now + offset) & 0xFFFFFFFF, 1, 0)
    lib.APP_RpmNotch_Tick()
    s = status(lib)
    assert s.motors.fresh[0] == int(fresh) and s.motors.spinning[0] == int(fresh)
    assert s.motors.erpm[0] == (45150 if fresh else 0)
    assert s.state == (STATE_TRACKING if fresh else STATE_IDLE)
    assert s.motors.age_ms[0] == max(0, -offset)


def test_speed_to_hz_uses_the_pole_pairs(lib):
    reset(lib, pp=7)
    spin_both(lib)
    lib.APP_RpmNotch_Tick()
    s = status(lib)
    assert s.motors.hz[0] == pytest.approx(107.5, abs=0.02)
    assert s.motors.hz[1] == pytest.approx(46000 / 7 / 60, abs=0.02)


def test_not_spinning_zeroes_the_target_immediately(lib):
    reset(lib)
    spin_both(lib)
    run(lib, line_signal(100))
    assert status(lib).weight_base[0] == 1.0
    now = lib.stub_get_ms()
    lib.stub_set_rotor(0, 0, now, 1, 1)
    lib.stub_set_rotor(1, 0, now, 1, 1)
    lib.APP_RpmNotch_Tick()
    s = status(lib)
    assert s.motors.fresh[0] == 1 and s.motors.spinning[0] == 0 and s.motors.erpm[0] == 0
    assert s.stale == 0                         # 明确"未旋转"不是过期
    out = run(lib, line_signal(40), erpm=None)
    assert status(lib).active_slots == 0 and status(lib).state == STATE_IDLE
    assert np.all(np.isfinite(out))


def test_an_implausible_erpm_is_rejected_and_the_last_value_held_briefly(lib):
    reset(lib)
    now = lib.stub_get_ms()
    lib.stub_set_rotor(0, 45150, now, 1, 0)
    lib.stub_set_rotor(1, 46000, now, 1, 0)
    lib.APP_RpmNotch_Tick()
    lib.stub_set_rotor(0, 150001, now + 5, 1, 0)
    lib.stub_set_ms(now + 5)
    lib.APP_RpmNotch_Tick()
    lib.APP_RpmNotch_Tick()                     # 同一坏帧跨拍只数一次
    s = status(lib)
    assert s.reject == 1
    assert s.motors.erpm[0] == 45150 and s.motors.hz[0] == pytest.approx(107.5, abs=0.02)
    lib.stub_set_ms(now + 21)                   # 最后接受的值已过 20 ms：不再顶用
    lib.stub_set_rotor(0, 150001, now + 21, 1, 0)
    lib.APP_RpmNotch_Tick()
    s = status(lib)
    assert s.reject == 2 and s.motors.erpm[0] == 0


def test_a_fresh_to_stale_transition_is_counted(lib):
    reset(lib)
    spin_both(lib)
    lib.APP_RpmNotch_Tick()
    lib.stub_set_ms(lib.stub_get_ms() + 25)
    lib.APP_RpmNotch_Tick()
    lib.APP_RpmNotch_Tick()
    assert status(lib).stale == 2               # 两路各一次，之后不重复数


# ---------------------------------------------------------------- 采样率


def _fs_after(lib, period_us, n=5000, odr=1000):
    reset(lib, odr=odr)
    spin_both(lib)
    run(lib, line_signal(n, fs=1e6 / period_us), period_us=period_us, erpm=(45150, 46000))
    return status(lib)


@pytest.mark.parametrize("period_us", [1000.3, 1020.0, 985.0])
def test_fs_estimate_converges_from_the_nominal_seed(lib, period_us):
    s = _fs_after(lib, period_us)
    assert s.fs_nominal_hz == 1000
    assert s.fs_hz == pytest.approx(1e6 / period_us, rel=5e-4)
    assert s.state == STATE_TRACKING


@pytest.mark.parametrize("period_us", [1020.0, 985.0])
def test_the_measured_sample_rate_is_what_places_the_notch(lib, period_us):
    """芯片振荡器偏 2%：陷波按**实测** fs 设计才落在 107.5 Hz 上，线压 30 dB 以上。

    若把名义 1000 Hz 送进滤波组，物理中心会挪 2 Hz 多，衰减掉到 20 dB 上下——而状态里报的
    fs 照样是对的，所以这里量的是线本身。只开一个陷波：第二个陷波会顺带盖住位置误差。"""
    fs = 1e6 / period_us
    reset(lib)
    spin_one(lib, 45150)
    n = 8000
    signal = line_signal(n, fs=fs)
    out = run(lib, signal, period_us=period_us, erpm=(45150, 0))
    assert status(lib).fs_hz == pytest.approx(fs, rel=5e-4)
    assert attenuation_db(signal, out, slice(n - 2000, n)) >= 30.0


def test_the_pole_pairs_place_the_notch(lib):
    """极对数决定陷波落在哪：pp=14、eRPM 90300 就是 107.5 Hz，线压 30 dB 以上。
    极对数若被忽略（当成 7），陷波会停在 215 Hz，这条线原样过去。"""
    reset(lib, pp=14)
    spin_one(lib, 90300)
    n = 3000
    signal = line_signal(n)
    out = run(lib, signal, erpm=(90300, 0))
    assert status(lib).motors.hz[0] == pytest.approx(107.5, abs=0.02)
    assert attenuation_db(signal, out, slice(n - 1000, n)) >= 30.0


def test_bmi270_nominal_800_hz_works(lib):
    s = _fs_after(lib, 1250.0, odr=800)
    assert s.fs_nominal_hz == 800 and s.fs_hz == pytest.approx(800.0, rel=5e-4)
    assert s.state == STATE_TRACKING and s.weight_base[0] == 1.0


def test_unknown_odr_bypasses_bitwise(lib):
    reset(lib, odr=0)
    spin_both(lib)
    signal = line_signal(500)
    assert run(lib, signal).tobytes() == signal.tobytes()
    assert status(lib).state == STATE_FS_UNKNOWN


def test_an_fs_six_percent_off_nominal_fades_out_to_bitwise(lib):
    reset(lib)
    spin_both(lib)
    period = 1e6 / 1060.0
    run(lib, line_signal(600, fs=1060.0), period_us=period)
    assert status(lib).weight_base[0] == 1.0      # 估计还没走远时照常跟踪
    n = 9000
    signal = line_signal(n, fs=1060.0)
    start = 10_000_000 + int(600 * period) + 1000
    out = run(lib, signal, timestamps=start + np.round(np.arange(n) * period).astype(np.uint64))
    s = status(lib)
    assert s.state == STATE_FS_BAD and s.fs_hz == pytest.approx(1060.0, rel=2e-3)
    assert s.active_slots == 0
    assert out[-500:].tobytes() == signal[-500:].tobytes()


# ---------------------------------------------------------------- 丢样与复位


def _residual(out, dc):
    return float(np.sqrt(np.mean((out[:, 1] - dc[1]) ** 2)))


def test_one_missing_sample_is_bridged_by_a_midpoint(lib):
    n = 3000
    dc = (0.2, -0.1, 0.05)
    signal = line_signal(n, freqs=(107.5, 109.5), dc=dc)
    stamps = 10_000_000 + np.arange(n, dtype=np.uint64) * 1000
    keep = np.ones(n, bool)
    keep[np.arange(700, n, 97)] = False         # 每 97 个样本丢一个（2P 的缺口）
    reset(lib)
    spin_both(lib)
    full = run(lib, signal, timestamps=stamps)
    reset(lib)
    spin_both(lib)
    gapped = run(lib, signal[keep], timestamps=stamps[keep])
    s = status(lib)
    assert s.gap1 == int((~keep).sum()) and s.reset == 1
    tail = keep.copy()
    tail[:700] = False
    line_rms = 0.1
    base = _residual(full[tail], dc)
    bridged = _residual(gapped[tail[keep]], dc)
    # 补中点实测只多出线 rms 的约 1%；拿上一个样本顶（保持）是约 3%。门定在 2%，分得开两者。
    assert bridged - base <= 0.02 * line_rms
    assert bridged < 0.25 * line_rms


@pytest.mark.parametrize("jump", ["long_gap", "backwards", "frame_reset"])
def test_a_reset_restarts_the_filter_from_the_current_gyro(lib, jump):
    """大缺口、时间戳倒退、坐标系/校准复位（ResetState）：下一个样本按第一个样本处理——
    计数 +1，**并且**滤波组真的回到以当前输入为直流的稳态。

    场景：跟踪中 0.5 rad/s 直流，这次复位前后真实角速度变成 3 rad/s。复位后控制用陀螺偏离 3
    不超过线幅；若只记数不复位，直流跳变的振铃（约 0.3 rad/s）会全权重进角速度环。"""
    reset(lib)
    spin_both(lib)
    run(lib, line_signal(400, dc=(0.5, 0.5, 0.5)))
    assert status(lib).state == STATE_TRACKING and status(lib).weight_base[0] == 1.0
    before = status(lib).reset
    last = 10_000_000 + 399 * 1000
    stamp = {"long_gap": last + 20_000, "backwards": last - 5, "frame_reset": last + 1000}[jump]
    if jump == "frame_reset":
        lib.APP_RpmNotch_ResetState()
    out = run(lib, line_signal(120, dc=(3.0, 3.0, 3.0)),
              timestamps=stamp + np.arange(120, dtype=np.uint64) * 1000)
    assert status(lib).reset == before + 1
    assert float(np.max(np.abs(out[:30, 0] - 3.0))) <= 0.12
    assert float(np.max(np.abs(out[50:, 0] - 3.0))) <= 0.005       # 两个陷波的振铃 50 ms 后衰完


# ---------------------------------------------------------------- 发布副本


def test_a_publish_landing_during_the_copy_forces_a_re_read(lib):
    """读者（命令任务）拷状态副本的途中控制拍发布过，就得重拷。

    双缓冲下读者这份会被"再下一次"发布改写，而那次写到一半时序号只前进了 1——所以序号
    动过这份就不可信。装置在读者拷完、复核序号之前插一个完整的控制拍：收下的必须是新发布的
    那份（样本数对得上），不能是拷之前那份。"""
    reset(lib)
    spin_both(lib)
    run(lib, line_signal(100))
    published = status(lib).samples
    signal = line_signal(7)
    out = np.zeros_like(signal)
    for i in range(len(signal)):
        lib.APP_RpmNotch_ApplySample(signal[i].ctypes.data_as(FP), out[i].ctypes.data_as(FP),
                                     10_100_000 + i * 1000)
    assert status(lib).samples == published       # 控制拍还没把这 7 个发布出去
    lib.stub_tick_at_barrier(2)                    # 读者第 2 次过屏障（拷完、复核序号前）插一拍
    s = status(lib)
    assert s.samples == published + 7
    motors = Motors()
    lib.stub_tick_at_barrier(2)
    lib.APP_RpmNotch_GetMotors(ctypes.byref(motors))
    assert list(motors.erpm) == [45150, 46000]


# ---------------------------------------------------------------- 配置交接


def test_an_invalid_request_changes_nothing(lib):
    reset(lib)
    before = Config()
    lib.APP_RpmNotch_GetConfig(ctypes.byref(before))
    bad = Config.from_buffer_copy(before)
    bad.q = 11.0
    assert lib.APP_RpmNotch_RequestConfig(ctypes.byref(bad)) == 0
    bad = Config.from_buffer_copy(before)
    bad.min_hz, bad.fade_hz = 200.0, 100.1
    assert lib.APP_RpmNotch_ConfigValid(ctypes.byref(bad)) == 0
    after = Config()
    lib.APP_RpmNotch_GetConfig(ctypes.byref(after))
    assert bytes(after) == bytes(before)


def test_a_valid_request_takes_effect_at_the_next_tick(lib):
    reset(lib, enable=0)
    spin_both(lib)
    lib.APP_RpmNotch_Tick()
    cfg = Config()
    lib.APP_RpmNotch_GetConfig(ctypes.byref(cfg))
    cfg.enable = 1
    assert lib.APP_RpmNotch_RequestConfig(ctypes.byref(cfg)) == 1
    signal = line_signal(1)
    out = np.zeros_like(signal)
    lib.APP_RpmNotch_ApplySample(signal.ctypes.data_as(FP), out.ctypes.data_as(FP), 20_000_000)
    assert out.tobytes() == signal.tobytes()     # 控制拍取用之前仍是旧配置
    lib.APP_RpmNotch_Tick()
    assert status(lib).state == STATE_TRACKING


# ---------------------------------------------------------------- 命令面


def test_changes_are_rejected_while_armed_or_while_sysid_owns_the_rig(lib):
    reset(lib, enable=0)
    lib.stub_set_armed(1)
    for line in ("RPMNOTCH ON", "RPMNOTCH OFF", "RPMNOTCH SET Q 5", "RPMNOTCH DEFAULTS"):
        assert command(lib, line) == ["RPMNOTCH event=rejected reason=armed\r\n"], line
    lib.stub_set_armed(0)
    lib.stub_set_sysid(1)
    assert command(lib, "RPMNOTCH ON") == ["RPMNOTCH event=rejected reason=sysid\r\n"]
    lib.stub_set_sysid(0)
    lib.APP_RpmNotch_Tick()
    assert status(lib).cfg.enable == 0
    reply = command(lib, "RPMNOTCH ?")          # 读状态任何时候都可以
    assert len(reply) == 4


def test_on_replies_with_the_state_it_is_going_into(lib):
    reset(lib, enable=0)
    reply = command(lib, "RPMNOTCH ON")
    assert len(reply) == 4 and reply[0].startswith("RPMNOTCH cfg en=1 state=idle src=bidir pp=7 ")
    lib.APP_RpmNotch_Tick()
    assert command(lib, "RPMNOTCH OFF")[0].startswith("RPMNOTCH cfg en=0 state=off ")


@pytest.mark.parametrize("line,key", [
    ("RPMNOTCH SET POLES 0", "POLES"), ("RPMNOTCH SET POLES 31", "POLES"),
    ("RPMNOTCH SET HARM 8", "HARM"), ("RPMNOTCH SET HARM 0", "HARM"),
    ("RPMNOTCH SET Q 1.4", "Q"), ("RPMNOTCH SET Q 10.5", "Q"),
    ("RPMNOTCH SET MINHZ 19", "MINHZ"), ("RPMNOTCH SET MINHZ 201", "MINHZ"),
    ("RPMNOTCH SET FADEHZ 4", "FADEHZ"), ("RPMNOTCH SET FADEHZ 101", "FADEHZ")])
def test_out_of_range_values_are_rejected(lib, line, key):
    reset(lib)
    assert command(lib, line) == [f"RPMNOTCH event=rejected reason=range key={key}\r\n"]


def test_the_band_may_reach_three_hundred_hz_at_the_limits(lib):
    """MINHZ ≤ 200、FADEHZ ≤ 100 两个单项上限已经保证 MINHZ+FADEHZ ≤ 300；边界上照收。"""
    reset(lib)
    assert command(lib, "RPMNOTCH SET MINHZ 200")[0].startswith("RPMNOTCH cfg ")
    lib.APP_RpmNotch_Tick()
    assert "min_hz=200 fade_hz=100 " in command(lib, "RPMNOTCH SET FADEHZ 100")[0]


@pytest.mark.parametrize("line", ["RPMNOTCH SET", "RPMNOTCH SET Q", "RPMNOTCH SET FOO 3",
                                  "RPMNOTCH SET Q abc", "RPMNOTCH on", "RPMNOTCH ON 1",
                                  "RPMNOTCH BANANA"])
def test_usage_errors_say_how_to_use_it(lib, line):
    reset(lib)
    reply = command(lib, line)
    assert len(reply) == 1 and reply[0].startswith("RPMNOTCH event=rejected reason=usage usage=")


def test_unrelated_commands_are_not_claimed(lib):
    lib.stub_clear_text()
    assert lib.stub_command(b"THRUSTLUT ?") == 0
    assert texts(lib) == []


def test_set_and_defaults_round_trip(lib):
    reset(lib)
    command(lib, "RPMNOTCH SET POLES 14")
    command(lib, "RPMNOTCH SET HARM 3")
    command(lib, "RPMNOTCH SET Q 5")
    reply = command(lib, "RPMNOTCH SET MINHZ 60.4")
    assert "pp=14 harm=3 q_x100=500 min_hz=60 fade_hz=20 " in reply[0]
    lib.APP_RpmNotch_Tick()
    s = status(lib)
    assert (s.cfg.pole_pairs, s.cfg.harmonic_mask, s.cfg.q) == (14, 3, 5.0)
    reply = command(lib, "RPMNOTCH DEFAULTS")
    assert "en=1 " in reply[0] and "pp=7 harm=1 q_x100=300 min_hz=50 fade_hz=20 " in reply[0]


def test_clear_zeroes_counters_at_the_next_tick(lib):
    reset(lib)
    spin_both(lib)
    lib.stub_set_us_step(3)
    run(lib, line_signal(300))
    s = status(lib)
    assert s.samples == 300 and s.apply_count == 300 and s.apply_us_max > 0 and s.tick_count > 0
    assert command(lib, "RPMNOTCH CLEAR") == ["RPMNOTCH event=cleared\r\n"]
    lib.APP_RpmNotch_Tick()
    s = status(lib)
    assert s.samples == 0 and s.apply_us_max == 0 and s.reset == 0 and s.spin == 0
    assert s.tick_count == 0


@pytest.mark.parametrize("harm,base,erpm", [
    (2, 2, (45150, 46000)), (4, 3, (45150, 46000)), (6, 2, (45150, 46000)), (7, 1, (45150, 46000)),
    (2, 2, (21000, 21000))])                    # 最后一组：1x 50 Hz 在淡入下限，2x 100 Hz 在全权重频段
def test_weight_and_tracking_follow_the_lowest_enabled_harmonic(lib, harm, base, erpm):
    """HARM 2/4/6 没有 1x：权重（ch*_w_x100）与 spin/tracked 看掩码里最低的那个谐波（频段也按它的频率判）。
    若死盯 1x 槽，陷波明明在滤，状态也会报权重 0、跟踪 0%。"""
    reset(lib, harm=harm)
    spin_both(lib, erpm)
    lib.APP_RpmNotch_Tick()
    n = 1000
    signal = line_signal(n, freqs=(base * erpm[0] / 7 / 60,))
    out = run(lib, signal, erpm=erpm)
    s = status(lib)
    assert s.state == STATE_TRACKING and list(s.weight_base) == [1.0, 1.0]
    assert s.spin == n and s.samples - s.tracked <= 25
    assert attenuation_db(signal, out, slice(n - 400, n)) >= 30.0     # 它确实在滤
    esc = command(lib, "RPMNOTCH ?")[1]
    assert " ch1_w_x100=100 " in esc and esc.endswith(" ch2_w_x100=100\r\n")


def test_spin_is_not_counted_above_the_full_weight_band(lib):
    """最低谐波高过 0.40·fs（这里 pp=1，1x 约 752 Hz，陷波按频段本来就不滤）不算 spin，
    免得 tracked/spin 把"频段规定不滤"算成"陷波没跟上"。"""
    reset(lib, pp=1)
    spin_both(lib)
    lib.APP_RpmNotch_Tick()
    run(lib, line_signal(300))
    s = status(lib)
    assert s.spin == 0 and s.tracked == 0 and s.state == STATE_IDLE


def test_tracking_statistics_follow_the_one_x_weight(lib):
    reset(lib)
    spin_both(lib)
    lib.APP_RpmNotch_Tick()                     # 这一拍起两路都在全权重频段
    run(lib, line_signal(1000))
    s = status(lib)
    assert s.spin == 1000
    assert s.samples - s.tracked <= 25          # 只有淡入那 20 个样本不算"满权重"
    reset(lib, enable=0)
    spin_both(lib)
    lib.APP_RpmNotch_Tick()
    run(lib, line_signal(200))
    s = status(lib)
    assert s.spin == 200 and s.tracked == 0


# ---------------------------------------------------------------- 报文


STATUS_PREFIXES = ("RPMNOTCH cfg ", "RPMNOTCH esc ", "RPMNOTCH count ", "RPMNOTCH time ")


def test_status_block_and_provenance_parse_with_the_sysid_kv_parser(lib):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from sysid.decode import _parse_kv
    finally:
        sys.path.pop(0)
    reset(lib)
    spin_both(lib)
    run(lib, line_signal(200))
    block = command(lib, "RPMNOTCH ?")
    assert [line.split(" ", 2)[0] + " " + line.split(" ", 2)[1] + " " for line in block] == \
        list(STATUS_PREFIXES)
    cfg = _parse_kv(block[0])
    assert (cfg["en"], cfg["state"], cfg["src"], cfg["pp"], cfg["harm"], cfg["q_x100"]) == \
        ("1", "tracking", "bidir", "7", "1", "300")
    assert (cfg["fs_nom"], cfg["fs_x10"], cfg["active"]) == ("1000", "10000", "2")
    esc = _parse_kv(block[1])
    assert (esc["ch1_erpm"], esc["ch1_hz_x10"], esc["ch1_w_x100"]) == ("45150", "1075", "100")
    for key in ("stale", "gap1", "reset", "nonfinite", "slewclamp", "reject", "wdog"):
        assert key in _parse_kv(block[2])
    for key in ("samples", "spin", "tracked", "apply_us_avg_x100", "apply_us_max",
                "tick_us_avg_x100", "tick_us_max"):
        assert key in _parse_kv(block[3])
    lib.stub_clear_text()
    lib.APP_RpmNotch_ReportProvenance(42)
    line, = texts(lib)
    values = _parse_kv(line)
    assert line.startswith("SYSID NOTCH run=42 ")
    assert {key: values[key] for key in ("en", "state", "src", "pp", "harm", "q_x100", "min_hz",
                                         "fade_hz", "fs_x10")} == {
        "en": "1", "state": "tracking", "src": "bidir", "pp": "7", "harm": "1", "q_x100": "300",
        "min_hz": "50", "fade_hz": "20", "fs_x10": "10000"}


def _format_strings(source: str) -> list[str]:
    """QueueText 的格式串（相邻字面量拼起来，宏展开 RPMNOTCH_USAGE）。"""
    usage = re.search(r'#define RPMNOTCH_USAGE\s*\\\s*\n\s*"([^"]*)"', source).group(1)
    calls = re.findall(r"APP_Control_QueueText\(\s*((?:\"[^\"]*\"\s*|RPMNOTCH_USAGE\s*)+)", source)
    formats = []
    for call in calls:
        pieces = re.findall(r'"([^"]*)"|(RPMNOTCH_USAGE)', call)
        formats.append("".join(text if text else usage for text, _macro in pieces))
    return formats


def test_every_line_fits_the_255_byte_text_buffer_with_worst_case_values():
    """把每个 %lu/%u 换成 10 位最大值、%s 换成最长的取值，整行（含 \\r\\n）仍 < 255。"""
    source = (ROOT / "App/Src/app_cmd_rpmnotch.c").read_text(encoding="utf-8")
    formats = _format_strings(source)
    assert len(formats) >= 7
    longest_word = max(["unavailable", "fs_unknown", "tracking", "MINHZ+FADEHZ", "armed", "sysid",
                        "range", "usage"], key=len)
    for fmt in formats:
        assert "%f" not in fmt and "%g" not in fmt and "%e" not in fmt, fmt
        worst = re.sub(r"%l?u", "4294967295", fmt)
        worst = worst.replace("%s", longest_word).replace("\\r\\n", "\r\n")
        assert len(worst.encode()) < 255, (len(worst), fmt)
    assert any(fmt.startswith("SYSID NOTCH run=") for fmt in formats)
    assert sum(fmt.startswith(STATUS_PREFIXES) for fmt in formats) == 4


def test_real_lines_are_short_and_integer_only(lib):
    reset(lib)
    spin_both(lib)
    run(lib, line_signal(200))
    lines = command(lib, "RPMNOTCH ?") + command(lib, "RPMNOTCH SET Q 99") + command(lib, "RPMNOTCH X")
    lib.stub_clear_text()
    lib.APP_RpmNotch_ReportProvenance(65535)
    lines += texts(lib)
    for line in lines:
        assert len(line.encode()) < 255 and line.endswith("\r\n")
        assert "." not in line.split("usage=")[0], line     # 只有整数
