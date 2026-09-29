"""严格的 `CURRENT ...` 行解析；无 Tk、无硬件知识，可直接单测。

与 `battery_monitor.py` 同一个形态：解析和显示分开，页面只拿到一个已经自洽的
快照。事实源是固件 `App/Src/app_current.c` 里 `APP_Current_Report()` 的格式串，
不是这份文件的注释。

为什么解析要这么严：`valid=1` 却带着 `adc_status!=0` / 已饱和 / 零样本 / 超龄的
组合是**自相矛盾**的回包，把它当成一个有效读数显示出去，界面就在说谎。整行拒绝
之后页面显示"回包无效"，比显示一个看着正常的电流值安全得多。
"""

from __future__ import annotations

import math

from .proto import parse_kv


UINT32_MAX = 0xFFFFFFFF

#: 固件 `BSP_CurrentStatus` 的取值。名字只用于显示，判据看数字。
ADC_STATUS = {0: "正常", 1: "尚未初始化", 2: "转换超时", 3: "采样错误"}

#: 固件唯一会发的来源标识。换了硬件却沿用旧标识，是"安静失败"的入口。
CURRENT_SOURCE = "AM32_55A_CURR"


def parse_current(line: str) -> dict:
    """A complete firmware CURRENT line is one snapshot; never merge fragments."""
    if not line.startswith("CURRENT "):
        raise ValueError("不是电流回包")
    values = parse_kv(line)
    result = {}
    for key in ("raw", "valid", "calibrated", "saturated", "age_ms", "samples", "errors", "adc_status"):
        text = values[key]
        if not text.isdecimal():
            raise ValueError(key)
        result[key] = int(text)
        maximum = 65535 if key == "raw" else (1 if key in ("valid", "calibrated", "saturated") else UINT32_MAX)
        if result[key] > maximum:
            raise ValueError(key)
    if result["adc_status"] not in ADC_STATUS or values["source"] != CURRENT_SOURCE:
        raise ValueError("不支持的电流来源或状态")
    for key in ("adc_v", "current_a", "nominal_mv_per_a"):
        result[key] = float(values[key])
    if not math.isfinite(result["nominal_mv_per_a"]) or result["nominal_mv_per_a"] <= 0:
        raise ValueError("比例无效")
    if not (math.isnan(result["adc_v"]) or 0 <= result["adc_v"] <= 3.6):
        raise ValueError("ADC 电压越界")
    if result["valid"] and (not math.isfinite(result["current_a"]) or
                            not math.isfinite(result["adc_v"]) or result["saturated"] or
                            result["adc_status"] != 0 or result["samples"] == 0 or
                            result["age_ms"] > 250):
        raise ValueError("回包有效性矛盾")
    result["source"] = values["source"]
    return result


__all__ = ["ADC_STATUS", "CURRENT_SOURCE", "UINT32_MAX", "parse_current"]
