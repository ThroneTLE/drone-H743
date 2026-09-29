"""回差补偿与光杆辨识：辨识记录永远是补偿前的口径 —— 在宿主上跑真 app_sysid.c + 真补偿策略层。

稳定环里的顺序是：APP_SysId_Update（激励、脉宽、采样入环）→ APP_SysId_GetServoTargets 写进
frame->moves → 仲裁 → 记保持目标 → APP_ServoBacklash_Apply 原地改脉宽 → 写硬件。
这里按同一顺序逐拍驱动（平台边界照 tests/fixtures/sysid/app_harness.c），跑同一轮舵机单独
（SERVO，上锁）双脉冲两次——补偿关、补偿开——判据：

* 两次的**辨识记录逐字段相同**（servo_tilt、倾转指令、陀螺……），也就是记录不含补偿；
* 两次交给补偿层之前的脉宽相同；补偿开时发往硬件的脉宽在走动段带 ±13 µs，
  前导回中与结束回中逐字节是标定中位。
"""
from __future__ import annotations

import ctypes

import pytest

from test_sysid_runtime_contract import build_lib, default_airframe
from test_sysid_servo_mode import (
    EXTRA_SOURCES, MODE_FF, MODE_SERVO, SHORT_DOUBLET, BatchHeader, Sample, ServoCal, centre,
    records, run, servo_obs, start_mode,
)

U8 = ctypes.c_uint8
U16 = ctypes.c_uint16
SRC_NONE, SRC_SYSID = 0, 2
B_US = 13


class Config(ctypes.Structure):
    _fields_ = [("enable", U8), ("half_mrad", U16 * 2), ("thr_mrad", U16)]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    handle = build_lib(tmp_path_factory, (*EXTRA_SOURCES, "App/Src/app_servo_backlash.c",
                                          "Driver/Src/drv_servo_backlash.c"), "sysid-backlash")
    handle.harness_command.argtypes = [ctypes.c_char_p]
    handle.harness_command.restype = U8
    handle.APP_SysId_SetMode.argtypes = [ctypes.c_int, ctypes.c_float]
    handle.APP_SysId_SetMode.restype = U8
    handle.APP_SysId_SetServoTilt.argtypes = [ctypes.c_float]
    handle.APP_SysId_SetServoTilt.restype = U8
    handle.DRV_SysIdRecord_Unpack.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t, ctypes.POINTER(BatchHeader),
        ctypes.POINTER(Sample), ctypes.POINTER(ctypes.c_uint32)]
    handle.DRV_COAX_CTRL_GetServoCalibration.argtypes = [ctypes.POINTER(ServoCal)]
    handle.APP_ServoBacklash_RequestConfig.argtypes = [ctypes.POINTER(Config)]
    handle.APP_ServoBacklash_RequestConfig.restype = U8
    handle.APP_ServoBacklash_GetConfig.argtypes = [ctypes.POINTER(Config)]
    handle.APP_ServoBacklash_Apply.argtypes = [ctypes.c_uint32, ctypes.c_int, U8, U8,
                                               ctypes.POINTER(U16), ctypes.POINTER(U16)]
    return handle


@pytest.fixture(autouse=True)
def back_to_ff(lib):
    yield
    lib.APP_SysId_Stop(b"test")
    for _ in range(40):
        lib.APP_SysId_StreamTick()
    assert lib.APP_SysId_SetMode(MODE_FF, ctypes.c_float(0.0523598776)) == 1
    lib.DRV_Airframe_SetParams(ctypes.byref(default_airframe()))
    lib.harness_reset()


def one_servo_run(lib, *, compensate):
    lib.APP_ServoBacklash_Init()
    # 开关两种都显式配置：2026-09-28 起开机默认是开，"补偿关"那轮不能再靠上电默认。
    cfg = Config()
    lib.APP_ServoBacklash_GetConfig(ctypes.byref(cfg))
    cfg.enable = 1 if compensate else 0
    assert lib.APP_ServoBacklash_RequestConfig(ctypes.byref(cfg)) == 1
    start_mode(lib, MODE_SERVO, spec=SHORT_DOUBLET)
    commanded, hardware = [], []

    def on_tick(index):
        alpha, beta = centre(lib)                    # APP_SysId_GetServoTargets（补偿前）
        source = SRC_SYSID if lib.APP_SysId_IsRunning() else SRC_NONE
        a, b = U16(alpha), U16(beta)
        lib.APP_ServoBacklash_Apply(1_000 + index * 2, source, 0, 0, ctypes.byref(a), ctypes.byref(b))
        commanded.append((alpha, beta))
        hardware.append((a.value, b.value))

    stopped = run(lib, 1000, obs=servo_obs, on_tick=on_tick)
    assert stopped < 1000
    for _ in range(8):
        lib.APP_SysId_StreamTick()
    rows = [(flags, t, bytes(sample)) for flags, t, sample in records(lib)]
    return rows, commanded, hardware


def test_identification_records_are_identical_with_and_without_compensation(lib):
    rows_off, commanded_off, hardware_off = one_servo_run(lib, compensate=False)
    lib.APP_SysId_Stop(b"test")
    for _ in range(40):
        lib.APP_SysId_StreamTick()
    lib.harness_reset()
    rows_on, commanded_on, hardware_on = one_servo_run(lib, compensate=True)

    assert rows_off and rows_on == rows_off, "辨识记录（含 servo_tilt、倾转指令）不含补偿"
    assert commanded_on == commanded_off, "交给补偿层之前的脉宽两次一样"
    assert hardware_off == commanded_off, "补偿关：发往硬件的就是辨识给的"

    cal = ServoCal()
    lib.DRV_COAX_CTRL_GetServoCalibration(ctypes.byref(cal))
    centre_pulses = (cal.center_us[0], cal.center_us[1])
    offsets = {(h[0] - c[0], h[1] - c[1]) for c, h in zip(commanded_on, hardware_on)}
    assert {abs(d) for pair in offsets for d in pair} <= {0, B_US}
    assert any(pair != (0, 0) for pair in offsets), "走动段确实补了"
    # 零激励前导（舵机回中）与结束后的回中：逐字节是标定中位。
    assert hardware_on[0] == centre_pulses and hardware_on[100] == centre_pulses
    assert commanded_on[-1] == centre_pulses and hardware_on[-1] == centre_pulses
