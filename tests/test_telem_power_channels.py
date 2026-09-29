"""R-PWR-1：电源遥测通道（batt_v / batt_i）的填充语义，在宿主上跑真代码。

为什么要单独开一个宿主装置，而不是在 Python 里读一遍源码字符串：这条改动的
全部风险都集中在**一个判断**上——快照无效的时候填什么。填 0 的代码和填 NaN
的代码长得一样正常，编译一样过，实机上一样会出数；区别只在上位机能不能分清
"此刻不耗电"和"根本没采到"。0 A 是合法读数，拿它当哨兵值就等于把这两种情况
永久合并，而它们的处置完全相反（前者继续飞，后者该报传感器故障）。

所以这里把 `App/Src/app_telem_port.c` 真编译出来跑：
  * 有效快照 -> 真实值，且 mV -> V 的换算是对的；
  * 电压无效 / 电流无效 -> **NaN**，不是 0、不是上一拍的旧值。

装置只替换 `APP_Battery_GetSnapshot` / `APP_Current_GetSnapshot` 这一层，
因为被测的是 port 的映射而不是 ADC 采样。"无效判据由这两个 Get 负责折进
valid" 这个前提本身也在下面钉住（见 test_the_snapshot_accessors_own_the
_staleness_rule），否则 port 依赖的就是一条没人保证的口头约定。
"""

from __future__ import annotations

import math
import re
import shutil
import struct
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_comments(source: str) -> str:
    """去掉注释再断言"代码里没有 X"。

    否则一句解释"为什么不在这里重造 STALE_MS"的注释，会把钉这件事的断言自己
    弄红——于是下一个人删的是注释，不是违规代码。
    """
    source = re.sub(r"/\*.*?\*/", " ", source, flags=re.S)
    return re.sub(r"//[^\n]*", " ", source)


# 只替换 Core/ 那几个 CubeMX / RTOS 头，业务头一律用仓库里的真货——
# 通道枚举、快照结构体和单位约定必须来自真实契约，否则测的是装置不是固件。
STUB_MAIN_H = """
#ifndef MAIN_H
#define MAIN_H
/* drv_imu.h 只用到这两个 HAL 句柄类型的名字。 */
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
#endif
"""

STUB_CMSIS_OS2_H = """
#ifndef CMSIS_OS2_H
#define CMSIS_OS2_H
#include <stdint.h>
typedef void *osMessageQueueId_t;
typedef void *osThreadId_t;
typedef void *osMutexId_t;
typedef void *osSemaphoreId_t;
typedef void *osEventFlagsId_t;
typedef enum { osOK = 0, osError = -1 } osStatus_t;
osStatus_t osDelay(uint32_t ticks);
osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *msg,
                             uint8_t *prio, uint32_t timeout);
#endif
"""

STUB_RTOS_OBJECTS_H = """
#ifndef RTOS_OBJECTS_H
#define RTOS_OBJECTS_H
#include "cmsis_os2.h"
extern osMessageQueueId_t vofaLogQueueHandle;
#endif
"""

HARNESS = r"""
#include "app_telem_stream.h"
#include "app_telemetry.h"

#include "app_battery.h"
#include "app_current.h"
#include "app_messages.h"
#include "app_optical_flow.h"
#include "app_stabilizer.h"
#include "drv_coax_ctrl.h"

#include "cmsis_os2.h"
#include "rtos_objects.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------ 装置 ------------------------------------ */

osMessageQueueId_t vofaLogQueueHandle = (osMessageQueueId_t)1;

static APP_BatterySnapshot stub_battery;
static APP_CurrentSnapshot stub_current;

void APP_Battery_GetSnapshot(APP_BatterySnapshot *out) { *out = stub_battery; }
void APP_Current_GetSnapshot(APP_CurrentSnapshot *out) { *out = stub_current; }

osStatus_t osDelay(uint32_t ticks) { (void)ticks; return osOK; }

osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *msg,
                             uint8_t *prio, uint32_t timeout)
{
    (void)queue; (void)prio; (void)timeout;
    memset(msg, 0, sizeof(APP_Sensor_SampleMessage));
    return osOK;
}

uint64_t SVC_Timestamp_Us(void) { return 1000ULL; }

void APP_SysId_StreamTick(void) {}
uint8_t APP_SysId_NeedsService(void) { return 0U; }
/* app_telem_port.c 的开跑前检查按模式分流（舵机单独不查推力新鲜度）。本装置不跑辨识。 */
int APP_SysId_GetMode(void) { return 0; }
APP_TelemSink APP_TelemStream_CommandSink(void) { return APP_TELEM_SINK_USB; }
uint8_t APP_ThrustLut_IsFresh(void) { return 1U; }
uint8_t APP_IMU_Capture_IsExportActive(void) { return 0U; }
void APP_IMU_Capture_ExportStep(void) {}
uint8_t APP_FlightLog_IsExportActive(void) { return 0U; }
uint8_t APP_USB_CDC_IsReady(void) { return 1U; }
uint8_t APP_USB_CDC_Write(const uint8_t *data, uint32_t length, uint32_t timeout_ms)
{ (void)data; (void)length; (void)timeout_ms; return 1U; }
uint8_t APP_VOFA_SendRaw(const uint8_t *data, uint16_t length)
{ (void)data; (void)length; return 1U; }
uint8_t APP_VOFA_SendFloats(const float *data, uint8_t count)
{ (void)data; (void)count; return 1U; }
uint8_t APP_MaintUART_WriteRaw(const uint8_t *data, uint16_t length)
{ (void)data; (void)length; return 1U; }
void APP_Control_QueueText(const char *format, ...) { (void)format; }

void APP_OpticalFlow_GetStatus(APP_OPTICAL_FLOW_Status *status)
{ memset(status, 0, sizeof(*status)); }
void APP_Stabilizer_ReadVofaDebug(StabilizerVofaDebug *out)
{ memset(out, 0, sizeof(*out)); }
void DRV_COAX_CTRL_GetLastDebug(DRV_COAX_CTRL_Debug *debug)
{ memset(debug, 0, sizeof(*debug)); }
uint8_t DRV_COAX_CTRL_GetParam(const char *name, float *value)
{ (void)name; *value = 0.0f; return 1U; }

/* ------------------------------ 用例 ------------------------------------ */

static float values[APP_TELEM_CH_COUNT];

static void emit(void)
{
    if (APP_TelemStream_PortSample(values, (uint32_t)APP_TELEM_CH_COUNT) == 0U) {
        printf("sample-refused\n");
        return;
    }
    printf("batt_v=%08lX batt_i=%08lX\n",
           (unsigned long)*(uint32_t *)(void *)&values[APP_TELEM_CH_BATT_V],
           (unsigned long)*(uint32_t *)(void *)&values[APP_TELEM_CH_BATT_I]);
}

int main(void)
{
    /* 1) 两路都有效：12345 mV -> 12.345 V，电流原样透传。 */
    memset(&stub_battery, 0, sizeof(stub_battery));
    memset(&stub_current, 0, sizeof(stub_current));
    stub_battery.state.valid = 1U;
    stub_battery.state.voltage_mv = 12345U;
    /* 2026-09-21：这一路送的是**块平均**（drv_current_filter.h），不是单点
       瞬时值——瞬时值对带 PWM 纹波的电调电流输出没有定义。所以判据看的是
       mean_valid / mean_a，reading.* 只用来证明 port 没有偷偷回退去读它。 */
    stub_current.mean_valid = 1U;
    stub_current.mean_a = -2.5f;
    stub_current.reading.valid = 1U;
    stub_current.reading.current_a = 999.0f;   /* 若 port 读了瞬时值，这里会暴露 */
    emit();

    /* 2) 电流合法地等于 0：必须原样发 0.0f，不许被当成"没数据"。 */
    stub_current.mean_a = 0.0f;
    emit();

    /* 3) 电压快照无效（不新鲜 / 未初始化 / 饱和，都由 valid 收口）。 */
    stub_battery.state.valid = 0U;
    stub_battery.state.voltage_mv = 12345U;   /* 旧值还在结构体里 */
    emit();

    /* 4) 还没凑满第一个块：必须发 NaN，不许拿"上一次的数"或 0 顶上。 */
    stub_battery.state.valid = 1U;
    stub_current.mean_valid = 0U;
    stub_current.mean_a = 7.5f;               /* 旧值还在结构体里 */
    stub_current.reading.valid = 1U;          /* 瞬时值有效也不算数 */
    stub_current.reading.current_a = 7.5f;
    emit();

    return 0;
}
"""


def build_and_run(tmp_path: Path) -> list[str]:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    stubs = tmp_path / "stubs"
    stubs.mkdir(parents=True, exist_ok=True)
    (stubs / "main.h").write_text(STUB_MAIN_H, encoding="utf-8")
    (stubs / "cmsis_os2.h").write_text(STUB_CMSIS_OS2_H, encoding="utf-8")
    (stubs / "rtos_objects.h").write_text(STUB_RTOS_OBJECTS_H, encoding="utf-8")

    harness = tmp_path / "harness.c"
    harness.write_text(HARNESS, encoding="utf-8")

    executable = tmp_path / "power.exe"
    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{stubs}",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            f"-I{ROOT / 'Services' / 'Inc'}",
            str(ROOT / "App" / "Src" / "app_telem_port.c"),
            str(ROOT / "App" / "Src" / "app_proto.c"),
            str(harness),
            "-o", str(executable),
        ],
        check=True, capture_output=True, text=True,
    )

    result = subprocess.run([str(executable)], check=True, capture_output=True)
    return [line for line in result.stdout.decode("ascii").splitlines() if line]


def decode(line: str) -> tuple[float, float]:
    fields = dict(item.split("=", 1) for item in line.split())
    return (
        struct.unpack("<f", struct.pack("<I", int(fields["batt_v"], 16)))[0],
        struct.unpack("<f", struct.pack("<I", int(fields["batt_i"], 16)))[0],
    )


@pytest.fixture(scope="module")
def samples(tmp_path_factory) -> list[tuple[float, float]]:
    lines = build_and_run(tmp_path_factory.mktemp("telem_power"))
    assert len(lines) == 4, lines
    return [decode(line) for line in lines]


def test_valid_snapshots_produce_volts_and_amps(samples) -> None:
    voltage, current = samples[0]
    # mV -> V。写死 12.345 而不是复算，是因为"除以 1000"写成"乘以 1000"照样
    # 编译通过，而遥测上只表现为电压大了六个数量级——没人会当成单位 bug。
    assert voltage == pytest.approx(12.345, abs=1e-6)
    # 电流原样透传，负值（回充）不许被钳。
    assert current == pytest.approx(-2.5, abs=1e-6)


def test_zero_amps_is_a_real_reading_not_a_sentinel(samples) -> None:
    """合法的 0 A 必须原样出去。

    这条和下一条是一对：只有 0 保持"真的没电流"、NaN 保持"没数据"，
    上位机才可能区分它们。任何一边塌陷到另一边，区分能力就永久消失了。
    """
    _, current = samples[1]
    assert current == 0.0
    assert not math.isnan(current)


def test_invalid_voltage_is_nan_not_zero(samples) -> None:
    voltage, current = samples[2]
    assert math.isnan(voltage), "无效电压必须是 NaN；0 V 会被画成一条贴地的假曲线"
    # 另一路不受牵连：一对 ADC 里只坏一个通道是常态。
    assert not math.isnan(current)


def test_invalid_current_is_nan_not_zero_and_not_stale(samples) -> None:
    voltage, current = samples[3]
    assert math.isnan(current), "无效电流必须是 NaN，不是 0，也不是上一拍的旧值"
    assert current != 7.5
    assert not math.isnan(voltage)


def test_the_snapshot_accessors_own_the_staleness_rule() -> None:
    """port 只判 `valid`，前提是两个 Get 已经把"过期"折进了 valid。

    这个前提必须由测试守住，否则哪天有人把新鲜度判断从 Get 里挪走，port 这边
    的 `valid != 0` 会安静地开始放行 10 秒前的电压——曲线照画，数值照变，
    没有任何一层会报错。同时钉住"无效时连数值本身也已经是 NaN"，这样即便
    port 退化成原样透传也不会冒出一个假的 0。
    """
    current = read("App/Src/app_current.c")
    battery = read("App/Src/app_battery.c")
    driver = read("Driver/Src/drv_battery.c")
    port = read("App/Src/app_telem_port.c")

    # 电流：过期 / ADC 异常 / 一个样本都还没有 -> valid=0 且 current_a=NaN。
    accessor = current.split("void APP_Current_GetSnapshot", 1)[1]
    assert "out->age_ms > CURRENT_STALE_MS" in accessor
    assert "out->adc_status != BSP_CURRENT_OK" in accessor
    assert "out->reading.valid = 0U;" in accessor
    assert "out->reading.current_a = NAN;" in accessor

    # 电压：新鲜度 + 未初始化由 DRV_Battery_IsFresh 收口后写回 state.valid。
    assert "out->state.valid=initialized?DRV_Battery_IsFresh(&state,now):0U;" in battery
    assert "DRV_BATTERY_STALE_MS" in driver

    # port 侧不许自己再造一个新鲜度门限——两个数迟早会分叉。
    sample = strip_comments(port).split("uint8_t APP_TelemStream_PortSample", 1)[1]
    assert "STALE_MS" not in sample
    assert "age_ms" not in sample


def test_power_channels_do_not_reach_into_the_hardware_layer() -> None:
    """遥测节拍上只许读缓存快照，不许触发一次 ADC。

    采样已经由 messageTask 以 50 Hz 在跑（ADC1 rank1=电流 / rank2=电压 是同一
    对转换）。在遥测任务里再读一次 ADC 不只是重复工作：那条路径带轮询等待，
    会把遥测周期拉长，而周期抖动正是这条流已经栽过的坑。
    """
    port = read("App/Src/app_telem_port.c")

    assert '#include "app_battery.h"' in port
    assert '#include "app_current.h"' in port
    for forbidden in ("bsp_current.h", "BSP_Current_Read", "APP_Current_Step",
                      "APP_Battery_Step", "stm32h7xx_hal"):
        assert forbidden not in port, forbidden
