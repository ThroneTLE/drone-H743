"""舵机回差补偿的策略层、命令面与接线（App/Src/app_servo_backlash.c + app_cmd_backlash.c）。

行为部分在宿主上跑真代码，平台边界（舵机标定、解锁/辨识占用、文本回复）在
tests/fixtures/servo_backlash/app_stubs.c，全部由测试控制。判据：

* **只补该补的**：在飞控制器只在解锁时补；光杆辨识（含上锁跑的 SERVO 模式）补；回中/上电/老 IDENT
  （来源 none）与验收覆盖、反馈台架、地面点动接管的拍一律逐字节原样——回中必须是标定中位本身。
* **什么时候从头定向**：退出补偿、来源切换、改配置、标定中位/极性变了、两拍间隔超过 50 ms，
  下一拍都按"复位后第一个样本"原样输出。
* **脉宽上的量**：20 mrad = 12.7 µs → 13 µs，按舵机走的方向加（pulse_sign 在来回换算里抵消），
  换向跳 26 µs；补完夹回标定端点并计数。
* **命令面**：解锁/辨识中拒绝改（原因写明），范围与用法拒绝；状态两行、溯源一行，每行 < 255 字符、
  只有整数。

接线部分（静态）钉的是"接在哪"：接错位置在宿主测试里全绿，到实机才表现成回中不准或辨识记录被补偿污染。
"""
from __future__ import annotations

import ctypes
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    "App/Src/app_servo_backlash.c",
    "App/Src/app_cmd_backlash.c",
    "Driver/Src/drv_servo_backlash.c",
    "tests/fixtures/servo_backlash/app_stubs.c",
]
INCLUDES = ["App/Inc", "Driver/Inc", "BSP/Inc", "Services/Inc"]

U8 = ctypes.c_uint8
U16 = ctypes.c_uint16
U32 = ctypes.c_uint32
SRC_NONE, SRC_CONTROLLER, SRC_SYSID = 0, 1, 2
B_US = 13            # 20 mrad × 2000/π µs/rad = 12.73 µs
CENTER = 1500


class Config(ctypes.Structure):
    _fields_ = [("enable", U8), ("half_mrad", U16 * 2), ("thr_mrad", U16)]


class Status(ctypes.Structure):
    _fields_ = [("cfg", Config), ("active", U8), ("source", U8), ("direction", ctypes.c_int8 * 2),
                ("offset_us", ctypes.c_int16 * 2), ("reversals", U32 * 2), ("resets", U32),
                ("nonfinite", U32), ("ticks", U32), ("clamped", U32)]


def build_policy_lib(tmp_path_factory, name="servo-backlash-policy"):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the servo backlash policy contract")
    build = tmp_path_factory.mktemp(name)
    stub = build / "stub"
    stub.mkdir()
    (stub / "cmsis_os2.h").write_text("typedef void *osSemaphoreId_t; typedef void *osMessageQueueId_t;\n")
    out = build / ("policy.dll" if os.name == "nt" else "policy.so")
    includes = ["-I", str(stub)]
    for path in INCLUDES:
        includes += ["-I", str(ROOT / path)]
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", *includes,
         *[str(ROOT / path) for path in SOURCES], "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    lib = ctypes.CDLL(str(out))
    lib.APP_ServoBacklash_GetStatus.argtypes = [ctypes.POINTER(Status)]
    lib.APP_ServoBacklash_GetConfig.argtypes = [ctypes.POINTER(Config)]
    lib.APP_ServoBacklash_DefaultConfig.argtypes = [ctypes.POINTER(Config)]
    lib.APP_ServoBacklash_RequestConfig.argtypes = [ctypes.POINTER(Config)]
    lib.APP_ServoBacklash_RequestConfig.restype = U8
    lib.APP_ServoBacklash_ConfigValid.argtypes = [ctypes.POINTER(Config)]
    lib.APP_ServoBacklash_ConfigValid.restype = U8
    lib.APP_ServoBacklash_SourceCompensated.argtypes = [ctypes.c_int, U8, U8]
    lib.APP_ServoBacklash_SourceCompensated.restype = U8
    lib.APP_ServoBacklash_ReportProvenance.argtypes = [U16]
    lib.APP_ServoBacklash_Apply.argtypes = [U32, ctypes.c_int, U8, U8, ctypes.POINTER(U16),
                                            ctypes.POINTER(U16)]
    lib.stub_set_calibration.argtypes = [U16, U16, U16, U16, U16, U16, ctypes.c_int8, ctypes.c_int8]
    lib.stub_set_armed.argtypes = [U8]
    lib.stub_set_sysid.argtypes = [U8]
    lib.stub_text_lines.restype = U32
    lib.stub_text_line.argtypes = [U32]
    lib.stub_text_line.restype = ctypes.c_char_p
    lib.stub_command.argtypes = [ctypes.c_char_p]
    lib.stub_command.restype = U8
    lib.stub_critical_balance.restype = U32
    lib.stub_critical_nesting.restype = U32
    return lib


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    return build_policy_lib(tmp_path_factory)


# ---------------------------------------------------------------- 小工具


class Clock:
    def __init__(self, ms=10_000):
        self.ms = ms


def reset(lib):
    lib.stub_reset()
    lib.APP_ServoBacklash_Init()
    lib.stub_clear_text()


def texts(lib) -> list[str]:
    return [lib.stub_text_line(i).decode() for i in range(lib.stub_text_lines())]


def command(lib, line: str) -> list[str]:
    lib.stub_clear_text()
    assert lib.stub_command(line.encode()) == 1
    return texts(lib)


def status(lib) -> Status:
    s = Status()
    lib.APP_ServoBacklash_GetStatus(ctypes.byref(s))
    return s


def enable(lib, *extra):
    for line in ("BACKLASH ON", *extra):
        reply = command(lib, line)
        assert reply[0].startswith("BACKLASH cfg "), reply


def disable(lib):
    """开机默认已是开（2026-09-28）：要测"关"的行为，先用命令显式关掉。"""
    reply = command(lib, "BACKLASH OFF")
    assert reply[0].startswith("BACKLASH cfg en=0 "), reply
    lib.stub_clear_text()


def apply(lib, clock, alpha, beta, *, source=SRC_CONTROLLER, armed=1, override=0, dt_ms=2):
    clock.ms += dt_ms
    a, b = U16(alpha), U16(beta)
    lib.APP_ServoBacklash_Apply(clock.ms, source, armed, override, ctypes.byref(a), ctypes.byref(b))
    return a.value, b.value


def feed(lib, clock, pairs, **kwargs):
    return [apply(lib, clock, a, b, **kwargs) for a, b in pairs]


def ramp(start, stop, step=1):
    return list(range(start, stop, step))


def up_and_down(n=20):
    """alpha 从中位往上走、beta 往下走（每拍 1 µs = 1.57 mrad）。"""
    return [(CENTER + k, CENTER - k) for k in range(n)]


# ---------------------------------------------------------------- 开机默认（开）与关


def test_boot_defaults_are_on_with_twenty_twenty_five(lib):
    """2026-09-28 作者定为开机即开（依据见 test_boot_default_is_on_after_the_rig_ab）；幅值不变。"""
    reset(lib)
    cfg = Config()
    lib.APP_ServoBacklash_GetConfig(ctypes.byref(cfg))
    assert (cfg.enable, cfg.half_mrad[0], cfg.half_mrad[1], cfg.thr_mrad) == (1, 20, 20, 5)
    # 开着但还没跑过一拍：active=0、来源 none、方向与偏置都是 0。
    expected = [
        "BACKLASH cfg en=1 alpha_mrad=20 beta_mrad=20 thr_mrad=5 active=0 src=none dir_alpha=0 "
        "dir_beta=0 off_alpha_us=0 off_beta_us=0\r\n",
        "BACKLASH count rev_alpha=0 rev_beta=0 reset=0 nonfinite=0 ticks=0 clamped=0\r\n",
    ]
    assert command(lib, "BACKLASH ?") == expected
    assert command(lib, "BACKLASH") == expected


@pytest.mark.parametrize("source,armed,override", [
    (SRC_CONTROLLER, 1, 0), (SRC_SYSID, 0, 0), (SRC_NONE, 1, 0)])
def test_off_is_an_exact_pass_through_for_every_source(lib, source, armed, override):
    reset(lib)
    disable(lib)
    clock = Clock()
    pairs = up_and_down(15) + [(1510, 1490)] * 3 + [(1480, 1520)] * 3
    assert feed(lib, clock, pairs, source=source, armed=armed, override=override) == pairs
    s = status(lib)
    assert (s.active, s.ticks, s.offset_us[0], s.offset_us[1]) == (0, 0, 0, 0)


# ---------------------------------------------------------------- 补谁


def test_the_controller_is_compensated_only_while_armed(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    pairs = up_and_down(20)
    out = feed(lib, clock, pairs)
    for (a_in, b_in), (a_out, b_out) in zip(pairs, out):
        moved = a_in - CENTER
        if moved * 1.5707963e-3 > 0.005:       # 走出 5 mrad 迟滞带（第 4 µs 起）才定向
            assert (a_out, b_out) == (a_in + B_US, b_in - B_US)
        else:
            assert (a_out, b_out) == (a_in, b_in), "复位后第一拍与起始迟滞带里原样"
    s = status(lib)
    # pulse_sign = −1：脉宽往上 = 机构负向（d = −1），往下 = 正向（d = +1）。
    assert (s.active, s.source, list(s.direction), list(s.offset_us)) == (1, SRC_CONTROLLER, [-1, 1],
                                                                         [B_US, -B_US])
    # 同样的指令、上锁：原样（地面检查时控制器照样算，不补）。
    reset(lib)
    enable(lib)
    assert feed(lib, Clock(), pairs, armed=0) == pairs


def test_the_offset_follows_the_pulse_direction_whatever_the_pulse_sign(lib):
    """pulse_sign = +1 与 −1 两种装法：脉宽往上走都加 +13 µs（空程在舵盘上，与机构正方向无关）。"""
    for sign in (1, -1):
        reset(lib)
        lib.stub_set_calibration(1500, 1480, 1000, 1000, 2000, 2000, sign, -sign)
        enable(lib)
        out = feed(lib, Clock(), [(1500 + k, 1480 + k) for k in range(10)])
        assert out[-1] == (1509 + B_US, 1489 + B_US)
        assert list(status(lib).direction) == [sign, -sign]


def test_sysid_is_compensated_even_while_disarmed(lib):
    """舵机单独（SERVO）模式本来就是上锁跑的；FF/RATE/ANGLE 的解锁由辨识自己管。"""
    reset(lib)
    enable(lib)
    out = feed(lib, Clock(), up_and_down(10), source=SRC_SYSID, armed=0)
    assert out[-1] == (1509 + B_US, 1491 - B_US)
    assert status(lib).source == SRC_SYSID


@pytest.mark.parametrize("source,armed,override", [
    (SRC_NONE, 1, 0),            # 回中 / 上电无姿态 / 老 IDENT
    (SRC_NONE, 0, 0),
    (SRC_CONTROLLER, 1, 1),      # 验收覆盖、反馈台架、地面点动接管了这一拍
    (SRC_SYSID, 0, 1),
])
def test_other_owners_are_never_compensated(lib, source, armed, override):
    reset(lib)
    enable(lib)
    pairs = up_and_down(12)
    assert feed(lib, Clock(), pairs, source=source, armed=armed, override=override) == pairs
    assert status(lib).active == 0


def test_the_pure_policy_table(lib):
    table = {(src, armed, override): lib.APP_ServoBacklash_SourceCompensated(src, armed, override)
             for src in (SRC_NONE, SRC_CONTROLLER, SRC_SYSID) for armed in (0, 1) for override in (0, 1)}
    assert {key for key, value in table.items() if value} == {
        (SRC_CONTROLLER, 1, 0), (SRC_SYSID, 0, 0), (SRC_SYSID, 1, 0)}
    assert lib.APP_ServoBacklash_SourceCompensated(7, 1, 0) == 0


# ---------------------------------------------------------------- 从头定向


def compensated(lib, clock, **kwargs):
    out = feed(lib, clock, up_and_down(10), **kwargs)
    assert out[-1] == (1509 + B_US, 1491 - B_US)
    return status(lib).resets


def test_centring_after_compensation_is_exact_and_restarts(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    before = compensated(lib, clock)
    assert apply(lib, clock, CENTER, CENTER, source=SRC_NONE) == (CENTER, CENTER), "回中就是标定中位"
    assert status(lib).resets == before + 1
    assert apply(lib, clock, 1509, 1491) == (1509, 1491), "重新接手的第一拍原样"


def test_disarming_restarts(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    before = compensated(lib, clock)
    assert apply(lib, clock, 1509, 1491, armed=0) == (1509, 1491)
    assert apply(lib, clock, 1509, 1491, armed=1) == (1509, 1491)
    assert status(lib).resets == before + 1


def test_a_source_switch_restarts(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    before = compensated(lib, clock)
    assert apply(lib, clock, 1509, 1491, source=SRC_SYSID, armed=0) == (1509, 1491)
    assert status(lib).resets == before + 1 and status(lib).source == SRC_SYSID


@pytest.mark.parametrize("gap_ms,restarts", [(50, False), (51, True)])
def test_a_gap_between_calls_restarts(lib, gap_ms, restarts):
    """标定手势期间 commit 整段不调用；断档超过 50 ms 就当有人接管过舵机。"""
    reset(lib)
    enable(lib)
    clock = Clock()
    compensated(lib, clock)
    out = apply(lib, clock, 1509, 1491, dt_ms=gap_ms)
    assert out == ((1509, 1491) if restarts else (1509 + B_US, 1491 - B_US))


def test_a_calibration_change_restarts(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    compensated(lib, clock)
    lib.stub_set_calibration(1510, 1500, 1000, 1000, 2000, 2000, -1, -1)
    assert apply(lib, clock, 1509, 1491) == (1509, 1491)


def test_a_config_change_restarts_and_rescales(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    compensated(lib, clock)
    command(lib, "BACKLASH SET ALPHA 30")
    assert apply(lib, clock, 1510, 1490) == (1510, 1490)
    out = feed(lib, clock, [(1510 + k, 1490 - k) for k in range(1, 8)])
    assert out[-1] == (1517 + 19, 1483 - B_US), "30 mrad = 19.1 µs"


# ---------------------------------------------------------------- 换向、夹限、计数


def test_a_reversal_steps_by_two_b_and_is_counted(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    feed(lib, clock, [(1500 + k, CENTER) for k in range(12)])            # 走到 1511，+13
    assert apply(lib, clock, 1511, CENTER) == (1511 + B_US, CENTER)
    assert apply(lib, clock, 1508, CENTER) == (1508 + B_US, CENTER), "退 3 µs = 4.7 mrad，还在迟滞里"
    assert apply(lib, clock, 1507, CENTER) == (1507 - B_US, CENTER), "退 4 µs = 6.3 mrad：换向，跳 26 µs"
    s = status(lib)
    assert (list(s.reversals), s.direction[0], s.offset_us[0]) == ([1, 0], 1, -B_US)


def test_the_compensated_pulse_is_clamped_to_the_calibrated_endpoints(lib):
    reset(lib)
    lib.stub_set_calibration(1500, 1500, 1490, 1492, 1520, 1520, -1, -1)
    enable(lib)
    out = feed(lib, Clock(), [(1500 + k, 1500 - k) for k in range(15)])
    assert out[-1] == (1520, 1492)
    assert max(a for a, _b in out) == 1520 and min(b for _a, b in out) == 1492
    assert status(lib).clamped > 0


def test_counters_survive_a_config_rebuild(lib):
    reset(lib)
    enable(lib)
    clock = Clock()
    feed(lib, clock, [(1500 + k, CENTER) for k in range(12)] + [(1500, CENTER)])
    assert status(lib).reversals[0] == 1
    command(lib, "BACKLASH SET THR 6")
    feed(lib, clock, [(1500 + k, CENTER) for k in range(12)] + [(1500, CENTER)])
    s = status(lib)
    assert s.reversals[0] == 2 and s.ticks > 0 and s.cfg.thr_mrad == 6


def test_critical_sections_are_balanced_and_never_nested(lib):
    reset(lib)
    enable(lib)
    feed(lib, Clock(), up_and_down(30))
    command(lib, "BACKLASH ?")
    assert lib.stub_critical_balance() == 0
    assert lib.stub_critical_nesting() == 1


# ---------------------------------------------------------------- 命令面


@pytest.mark.parametrize("line", ["BACKLASH ON", "BACKLASH OFF", "BACKLASH SET ALPHA 10",
                                  "BACKLASH SET THR 7"])
def test_changes_are_rejected_while_armed_or_while_sysid_holds_the_rig(lib, line):
    reset(lib)
    disable(lib)     # 从已知的"关、20/20/5"起步（开机默认是开），下面核对被拒绝的命令一项都没改
    lib.stub_set_armed(1)
    assert command(lib, line) == ["BACKLASH event=rejected reason=armed\r\n"]
    lib.stub_set_armed(0)
    lib.stub_set_sysid(1)
    assert command(lib, line) == ["BACKLASH event=rejected reason=sysid\r\n"]
    cfg = Config()
    lib.APP_ServoBacklash_GetConfig(ctypes.byref(cfg))
    assert (cfg.enable, cfg.half_mrad[0], cfg.thr_mrad) == (0, 20, 5), "被拒绝的命令一项都没改"
    assert command(lib, "BACKLASH ?")[0].startswith("BACKLASH cfg "), "看状态什么时候都可以"


@pytest.mark.parametrize("line,key", [
    ("BACKLASH SET ALPHA 88", "ALPHA"), ("BACKLASH SET BETA 1000", "BETA"),
    ("BACKLASH SET THR 0", "THR"), ("BACKLASH SET THR 51", "THR"),
    ("BACKLASH SET ALPHA -1", "ALPHA"),      # 与 app_control_parse_u32 同语义：strtoul 回绕成大数
])
def test_out_of_range_values_are_rejected_by_key(lib, line, key):
    reset(lib)
    assert command(lib, line) == [f"BACKLASH event=rejected reason=range key={key}\r\n"]


@pytest.mark.parametrize("line", ["BACKLASH FOO", "BACKLASH SET GAMMA 5", "BACKLASH SET ALPHA x",
                                  "BACKLASH SET ALPHA 1.5", "BACKLASH ON 1",
                                  "BACKLASH SET THR", "BACKLASH ? ?", "BACKLASH on"])
def test_malformed_commands_get_the_usage(lib, line):
    reset(lib)
    assert command(lib, line) == [
        "BACKLASH event=rejected reason=usage usage=BACKLASH ?|ON|OFF|SET ALPHA|BETA|THR <mrad>\r\n"]


def test_accepted_edges_and_the_reply_shows_the_request_before_the_next_tick(lib):
    reset(lib)
    disable(lib)     # 开机默认是开：先关，下面的 ON 才是一次真实的关→开请求
    for line in ("BACKLASH SET ALPHA 0", "BACKLASH SET BETA 87", "BACKLASH SET THR 1",
                 "BACKLASH SET THR 50"):
        assert command(lib, line)[0].startswith("BACKLASH cfg ")
    reply = command(lib, "BACKLASH ON")
    assert reply[0].startswith("BACKLASH cfg en=1 alpha_mrad=0 beta_mrad=87 thr_mrad=50 active=0 ")
    assert len(reply) == 2
    assert command(lib, "BACKLASH OFF")[0].startswith("BACKLASH cfg en=0 ")


def test_other_commands_are_not_claimed(lib):
    reset(lib)
    assert lib.stub_command(b"RPMNOTCH ?") == 0
    assert lib.stub_command(b"BACKLASHX ?") == 0


def test_status_and_provenance_parse_with_the_sysid_kv_parser(lib):
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from sysid.decode import _parse_kv
    finally:
        sys.path.pop(0)
    reset(lib)
    enable(lib, "BACKLASH SET BETA 25")
    clock = Clock()
    feed(lib, clock, [(1500 + k, 1500 - k) for k in range(12)] + [(1505, 1495)])
    cfg_line, count_line = command(lib, "BACKLASH ?")
    cfg, count = _parse_kv(cfg_line), _parse_kv(count_line)
    assert cfg == {"en": "1", "alpha_mrad": "20", "beta_mrad": "25", "thr_mrad": "5", "active": "1",
                   "src": "controller", "dir_alpha": "1", "dir_beta": "-1", "off_alpha_us": "-13",
                   "off_beta_us": "16"}
    assert count == {"rev_alpha": "1", "rev_beta": "1", "reset": "0", "nonfinite": "0",
                     "ticks": "13", "clamped": "0"}
    lib.stub_clear_text()
    lib.APP_ServoBacklash_ReportProvenance(42)
    assert texts(lib) == ["SYSID BACKLASH run=42 en=1 alpha_mrad=20 beta_mrad=25 thr_mrad=5 servo_hz=50\r\n"]


def _format_strings(source: str) -> list[str]:
    usage = re.search(r'#define BACKLASH_USAGE\s+"([^"]*)"', source).group(1)
    calls = re.findall(r"APP_Control_QueueText\(\s*((?:\"[^\"]*\"\s*|BACKLASH_USAGE\s*)+)", source)
    return ["".join(text if text else usage for text, _m in re.findall(r'"([^"]*)"|(BACKLASH_USAGE)', call))
            for call in calls]


def test_every_line_fits_the_255_byte_text_buffer_with_worst_case_values():
    source = (ROOT / "App/Src/app_cmd_backlash.c").read_text(encoding="utf-8")
    formats = _format_strings(source)
    assert len(formats) == 6
    for fmt in formats:
        assert "%f" not in fmt and "%g" not in fmt and "%e" not in fmt, fmt
        worst = re.sub(r"%l?u", "4294967295", fmt)
        worst = re.sub(r"%d", "-2147483648", worst)
        worst = worst.replace("%s", "controller").replace("\\r\\n", "\r\n")
        assert len(worst.encode()) < 255, (len(worst), fmt)
    assert formats.count(
        "SYSID BACKLASH run=%u en=%u alpha_mrad=%u beta_mrad=%u thr_mrad=%u servo_hz=%lu\\r\\n") == 1


# ---------------------------------------------------------------- 分层与接线（静态）


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
    return re.sub(r"//[^\n]*", "", source)


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(signature)


def block_after(body: str, marker: str) -> str:
    """marker 所在那一行打开的 { ... } 块。"""
    return function_body(body, marker)


def test_the_driver_is_pure():
    for path in ("Driver/Src/drv_servo_backlash.c", "Driver/Inc/drv_servo_backlash.h"):
        includes = re.findall(r'#include\s+[<"]([^>"]+)[>"]', read(path))
        assert set(includes) <= {"drv_servo_backlash.h", "math.h", "stddef.h", "string.h", "stdint.h"}, includes
    for line in strip_comments(read("Driver/Src/drv_servo_backlash.c")).splitlines():
        if line.startswith("static ") and "(" not in line:
            raise AssertionError(f"file-scope static: {line}")


def test_the_policy_does_not_print_from_the_control_task_and_touches_no_hardware():
    source = read("App/Src/app_servo_backlash.c")
    assert "APP_Control_QueueText(" not in source
    assert '#include "app_control.h"' not in source
    includes = re.findall(r'#include\s+[<"]([^>"]+)[>"]', source)
    assert set(includes) == {"app_servo_backlash.h", "bsp_critical.h", "drv_coax_ctrl.h",
                             "drv_servo_backlash.h", "math.h", "stddef.h", "string.h"}


def test_boot_default_is_on_after_the_rig_ab():
    """2026-09-28 作者定为开机即开：杆上 A/B 里 3° 舵机 1 Hz 等效延迟 100 → 66 ms；ANGLE 轮
    不含空程的模型复现从 −0.02 升到 +0.91（空程被吃掉，线性模型才说得通）。幅值与阈值不变。"""
    header = read("App/Inc/app_servo_backlash.h")
    assert re.search(r"#define APP_SERVO_BACKLASH_DEFAULT_ENABLE\s+1U", header)
    assert re.search(r"#define APP_SERVO_BACKLASH_DEFAULT_HALF_MRAD\s+20U", header)
    assert re.search(r"#define APP_SERVO_BACKLASH_DEFAULT_THR_MRAD\s+5U", header)


def test_the_source_is_set_only_where_controller_and_sysid_pulses_are_produced():
    source = strip_comments(read("App/Src/app_stabilizer.c"))
    assert '#include "app_servo_backlash.h"' in source
    assert len(re.findall(r"frame->servo_source = ", source)) == 3
    compute = function_body(source, "static void stabilizer_control_compute(")
    # 光杆辨识：取完目标、确认还在跑才标 sysid（老 IDENT 不标）。
    ident = compute[:compute.index("} else if (frame->rc_control_motor_mix_allowed == 0U) {")]
    assert ident.index("APP_SysId_GetServoTargets(&ident_alpha_us, &ident_beta_us);") < \
        ident.index("frame->moves[1].pulse_us = ident_beta_us;") < \
        ident.index("if ((frame->sysid_running != 0U) && (APP_SysId_IsRunning() != 0U)) {") < \
        ident.index("frame->servo_source = (uint8_t)APP_SERVO_BACKLASH_SRC_SYSID;")
    # 低油门回中与上电无姿态这两支不标（默认 0 = 不补）。
    centred = compute[compute.index("} else if (frame->rc_control_motor_mix_allowed == 0U) {"):
                      compute.index("} else if (frame->imu_control_valid != 0U) {")]
    startup_at = compute.index("} else if (ctx->has_imu_sample == 0U) {")
    hold_at = compute.index("  } else {", startup_at)
    startup = compute[startup_at:hold_at]
    assert "servo_source" not in centred and "servo_source" not in startup
    # 在飞控制器：紧跟在把分配器输出写进 moves 之后；IMU 短暂失效时保持的也是控制器的目标。
    assert re.search(r"frame->moves\[1\]\.pulse_us = frame->ctrl_out\.servo_beta_us;\s*"
                     r"frame->servo_source = \(uint8_t\)APP_SERVO_BACKLASH_SRC_CONTROLLER;", compute)
    hold = compute[hold_at:compute.index("if ((frame->ident_running != 0U) && (frame->sysid_running == 0U)) {")]
    assert "DRV_COAX_CTRL_ResetState();" in hold
    assert "frame->servo_source = (uint8_t)APP_SERVO_BACKLASH_SRC_CONTROLLER;" in hold


def test_compensation_sits_between_the_arbitration_and_the_hardware_write():
    source = strip_comments(read("App/Src/app_stabilizer.c"))
    commit = function_body(source, "static void stabilizer_control_commit(")
    normal = block_after(commit, "if (frame->servo_cal_active == 0U) {")
    apply_at = normal.index("APP_ServoBacklash_Apply(")
    assert normal.index("APP_ServoJog_Apply(") < normal.index("stabilizer_servo_record_target(frame->moves);") \
        < apply_at < normal.index("BSP_PWM_SetServoPulse(1U, frame->moves[0].pulse_us);") \
        < normal.index("BSP_BusServo_MoveManyAsync(frame->moves, 2U,")
    call = normal[apply_at:normal.index(";", apply_at)]
    assert "frame->now_ms, (APP_ServoBacklashSource)frame->servo_source" in call
    assert "frame->rc_armed" in call
    for owner in ("acceptance_override != 0U", "APP_ServoFeedbackBench_IsActive() != 0U",
                  "APP_ServoJog_IsActive() != 0U"):
        assert owner in call, owner
    assert "&frame->moves[0].pulse_us, &frame->moves[1].pulse_us" in call
    assert commit.count("APP_ServoBacklash_Apply(") == 1, "每拍恰好一次"
    init = function_body(source, "static void stabilizer_init(")
    assert init.index("memset(ctx, 0, sizeof(*ctx));") < init.index("APP_ServoBacklash_Init();")


def test_every_consumer_of_the_final_pulses_sees_the_intended_side():
    """保持目标、飞行日志 servo_*_us = 补偿前；servo_*_sent_us、IMU 采集注记 = 补偿后实际发出。"""
    source = strip_comments(read("App/Src/app_stabilizer.c"))
    assert "flog_snapshot.servo_alpha_us = stabilizer_latest_servo_target_us[0];" in source
    assert "flog_snapshot.servo_beta_us = stabilizer_latest_servo_target_us[1];" in source
    assert re.search(r"flog_snapshot\.servo_alpha_sent_us =\s*stabilizer_last_successful_servo_pulse_us\[0\];",
                     source)
    record = function_body(source, "static void stabilizer_servo_record_target(")
    assert "stabilizer_latest_servo_target_us[0] = moves[0].pulse_us;" in record
    # 保持目标只从 record_target 写（外加标定换了的回中），不会被补偿后的脉宽覆盖。
    writes = re.findall(r"stabilizer_latest_servo_target_us\[0\] =\s*([^;]+);", source)
    assert sorted(w.strip() for w in writes) == sorted([
        "moves[0].pulse_us",
        "servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]",
        "servo_calibration.center_us[DRV_COAX_CTRL_SERVO_ALPHA_INDEX]"])


def test_identification_never_sees_the_compensated_pulses():
    sysid = read("App/Src/app_sysid.c")
    assert "backlash" not in sysid.lower(), "app_sysid.c 不依赖回差补偿"
    source = strip_comments(read("App/Src/app_stabilizer.c"))
    assert source.count("APP_SysId_GetServoTargets(") == 1
    compute = function_body(source, "static void stabilizer_control_compute(")
    assert "APP_SysId_GetServoTargets(" in compute


def test_the_command_family_hangs_off_the_fallback_chain_and_cmake_lists_the_sources():
    fallback = read("App/Src/app_cmd_fallback.c")
    assert '#include "app_servo_backlash.h"' in fallback
    chain = fallback[fallback.index("void app_control_handle_unclaimed("):]
    assert chain.index("APP_ServoBacklash_Command(tokens, count)") < chain.index('"ERR unknown cmd %s')
    assert "BACKLASH" not in read("App/Src/app_control.c")
    cmake = read("CMakeLists.txt").replace("\r\n", "\n")
    for path in ("Driver/Src/drv_servo_backlash.c", "App/Src/app_servo_backlash.c",
                 "App/Src/app_cmd_backlash.c"):
        assert f"    {path}\n" in cmake, path


def test_provenance_follows_only_a_successful_start():
    source = strip_comments(read("App/Src/app_cmd_sysid.c"))
    assert re.search(r"if \(APP_SysId_Start\(\) != 0U\) \{\s*"
                     r"APP_RpmNotch_ReportProvenance\(APP_SysId_GetRunId\(\)\);\s*"
                     r"APP_ServoBacklash_ReportProvenance\(APP_SysId_GetRunId\(\)\);\s*\}", source)
