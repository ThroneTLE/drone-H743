"""遥控 CH9 手动 IMU 归零（2026-10-01）。

作者原话："CH9  2000的时候手动IMU标定归零"。动作与 `IMUZERO` 命令相同，只在上锁时生效。

* 宿主 gcc 编真 C（App/Src/app_rc_aux.c，外部依赖用桩头文件替换），逐条覆盖防误触规则：
  先见低位才认上升沿、拨住只触发一次、解锁时拦下且要重新拨、链路断开清状态、两次触发间隔、回差带。
* 源码契约：控制任务每拍调用、通道绑定只在新模块、构建清单、不进 app_control.c。
"""
from __future__ import annotations

import ctypes
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "App/Src/app_rc_aux.c"

NONE, IMUZERO, BLOCKED = 0, 1, 2
HIGH, LOW, MID = 2000, 1000, 1700

STUB_H = {
    "app_control.h": "void APP_Control_QueueText(const char *format, ...);\n",
    "app_sensor.h": "void APP_Sensor_RequestGyroRecal(void);\n",
    "app_stabilizer.h": "void APP_Stabilizer_RequestAttitudeRezero(void);\n",
}
STUB_C = r"""
#include <stdarg.h>
#include <stdio.h>
int gyro_calls, rezero_calls; char last_text[128];
void APP_Sensor_RequestGyroRecal(void) { gyro_calls++; }
void APP_Stabilizer_RequestAttitudeRezero(void) { rezero_calls++; }
void APP_Control_QueueText(const char *f, ...) { va_list a; va_start(a, f); vsnprintf(last_text, sizeof last_text, f, a); va_end(a); }
"""


class State(ctypes.Structure):
    _fields_ = [("low_seen", ctypes.c_uint8), ("holdoff_active", ctypes.c_uint8),
                ("holdoff_until_ms", ctypes.c_uint32)]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("宿主没有 gcc")
    d = tmp_path_factory.mktemp("rcaux")
    for name, text in STUB_H.items():
        (d / name).write_text(text, encoding="utf-8")
    (d / "stub.c").write_text(STUB_C, encoding="utf-8")
    out = d / "rcaux.dll"
    subprocess.run([gcc, "-shared", "-O0", "-Wall", "-Werror", f"-I{d}", f"-I{ROOT / 'App/Inc'}",
                    str(SRC), str(d / "stub.c"), "-o", str(out)], check=True)
    dll = ctypes.CDLL(str(out))
    dll.APP_RcAux_Step.argtypes = [ctypes.POINTER(State), ctypes.c_uint32, ctypes.c_uint16,
                                   ctypes.c_uint8, ctypes.c_uint8]
    dll.APP_RcAux_Step.restype = ctypes.c_uint8
    dll.APP_RcAux_Update.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint16),
                                     ctypes.c_uint8, ctypes.c_uint8]
    return dll


class Sw:
    def __init__(self, lib):
        self.lib, self.st, self.t = lib, State(), 0

    def __call__(self, us, link=1, armed=0, dt=20):
        self.t += dt
        return self.lib.APP_RcAux_Step(ctypes.byref(self.st), self.t, us, link, armed)


def test_rising_edge_after_low_fires_once_while_held(lib):
    s = Sw(lib)
    assert s(LOW) == NONE
    assert s(HIGH) == IMUZERO
    assert all(s(HIGH) == NONE for _ in range(500))     # 拨住 10 s 不重复


def test_switch_already_high_at_power_on_does_not_fire(lib):
    s = Sw(lib)
    assert all(s(HIGH) == NONE for _ in range(100))
    assert s(LOW) == NONE and s(HIGH) == IMUZERO


def test_armed_blocks_and_needs_a_fresh_toggle(lib):
    s = Sw(lib)
    s(LOW)
    assert s(HIGH, armed=1) == BLOCKED
    assert s(HIGH, armed=0) == NONE                     # 上锁后开关仍在高位：不补触发
    s(LOW)
    assert s(HIGH) == IMUZERO


def test_link_loss_clears_low_seen(lib):
    s = Sw(lib)
    s(LOW)
    s(LOW, link=0)
    assert s(HIGH) == NONE
    s(LOW)
    assert s(HIGH) == IMUZERO


def test_hysteresis_band_and_zero_channel_are_ignored(lib):
    s = Sw(lib)
    assert s(0) == NONE and s(HIGH) == NONE             # 0 = 无数据，不算低位
    s(LOW)
    assert s(MID) == NONE and s(1800) == NONE          # 回差带内与恰好 1800 都不触发
    assert s(1801) == IMUZERO
    s(1650)                                             # 回差带：不算拨回低位
    assert s(HIGH) == NONE


def test_holdoff_between_triggers(lib):
    s = Sw(lib)
    s(LOW)
    assert s(HIGH) == IMUZERO
    s(LOW)
    assert s(HIGH) == NONE                              # 40 ms 后再拨：还在 3 s 采样窗口里
    s(LOW, dt=3000)
    assert s(HIGH) == IMUZERO


def test_update_requests_both_resamples_and_reports(lib):
    gyro = ctypes.c_int.in_dll(lib, "gyro_calls")
    rez = ctypes.c_int.in_dll(lib, "rezero_calls")
    text = ctypes.c_char_p(ctypes.addressof(ctypes.c_char.in_dll(lib, "last_text")))
    ch = (ctypes.c_uint16 * 16)(*([1500] * 16))
    g0, r0 = gyro.value, rez.value
    ch[8] = LOW
    lib.APP_RcAux_Update(100000, ch, 1, 0)
    ch[8] = HIGH
    lib.APP_RcAux_Update(100020, ch, 1, 0)
    assert (gyro.value - g0, rez.value - r0) == (1, 1)
    assert text.value.decode() == "IMUZERO state=restarted keep_still_ms=3000 source=rc\r\n"
    ch[8] = LOW
    lib.APP_RcAux_Update(110000, ch, 1, 1)
    ch[8] = HIGH
    lib.APP_RcAux_Update(110020, ch, 1, 1)
    assert (gyro.value - g0, rez.value - r0) == (1, 1)
    assert text.value.decode() == "IMUZERO state=armed_blocked source=rc\r\n"
    ch[7] = LOW
    ch[8] = 1500
    lib.APP_RcAux_Update(120000, ch, 1, 0)
    ch[7] = HIGH                                        # CH8 拨动不触发：绑定的是 CH9
    lib.APP_RcAux_Update(120020, ch, 1, 0)
    assert (gyro.value - g0, rez.value - r0) == (1, 1)


def _src(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


def test_wiring_and_single_binding_point():
    stab = _src("App/Src/app_stabilizer.c")
    call = "APP_RcAux_Update(frame->now_ms, frame->ch, frame->rc_link_ok, frame->rc_armed);"
    assert stab.count(call) == 1
    assert stab.index("frame->rc_armed = stabilizer_rc_update_armed(") < stab.index(call)
    assert "#define APP_RC_AUX_IMUZERO_CHANNEL     8U      /* CH9 */" in _src("App/Inc/app_rc_aux.h")
    assert "ch[8]" not in stab and "APP_RC_AUX_IMUZERO_CHANNEL" not in stab
    assert "App/Src/app_rc_aux.c" in _src("CMakeLists.txt")
    assert "RcAux" not in _src("App/Src/app_control.c")
    assert "%f" not in _src("App/Src/app_rc_aux.c")
