# PX4 出厂固件基准：PC0/PC1 ADC 原始读数（2026-09-21）

## 为什么做这次测量

我们固件的电流读数长期接近 0（外部电流计 0.15 A / 0.5 A 时，面板读 0.000~0.02 A），
而电压读数正常。审核会话据噪声形态推断"PC1 浮空、Curr 信号没接"，作者不认同
（"成品飞控，电流不可能出问题"），要求用出厂 PX4 做已知good基准做 A/B。

## 方法

整片 Flash 先备份（`data/firmware_updates/2026-09-21/micoair743v2-preflash-full-2MB.bin`，
2 MB，含我们的固件与 config_store 标定），再烧
`doc/micoair743v2/baseline/MicoAir743v2-PX4-1.15.4-Bootloader+Firmware.bin`。
经 MAVLink SERIAL_CONTROL 进 NuttX nsh 取 `listener adc_report`。

## 结果（电机未转，板子与电调通电）

```
TOPIC: adc_report
    raw_data:   [11002, 1226, 9864, 0, ...]
    resolution: 65536
    v_ref:      3.30000
    channel_id: [10, 11, 20, -1, ...]
```

`BATTERY_STATUS` / `SYS_STATUS`：`voltage 11.696 V`，`current 2.43~2.48 A`（稳定）。

## 对照

| 通道 | PX4 原始 | 我们固件 | 结论 |
|---|---|---|---|
| ch10 = PC0 电压 | 11002 LSB | 折算 11.695 V，与 PX4 的 11.696 V 一致 | ✅ 一致 |
| ch11 = PC1 电流 | **1226 LSB** | **约 5 LSB** | ❌ 差约 240 倍 |

## 结论

**硬件正常，Curr 信号确实到达 PC1。缺陷在我们固件的 ADC 采样实现。**

"引脚浮空"的推断被推翻。噪声形态（电机开关时抖动增大、读数可回到 0）同时符合
"浮空拾取 EMI"和"采样实现有缺陷"，单凭它无法区分——这是把未能区分的现象当判据
的教训。

## 首要嫌疑

`BSP/Src/bsp_current.c` 每 20 ms 一对采样，**每对结束都 `HAL_ADC_Stop()`（禁用 ADC），
下一对再 `HAL_ADC_Start()` 重新使能**。而我们的读取顺序是 rank1=ch11(电流) 在前、
rank2=ch10(电压) 在后。若"重新使能后的第一次转换不可靠"，症状恰好是
**电流恒为接近 0、电压正常**——与实测完全吻合。PX4 不关 ADC，持续扫描，故无此问题。

待验证，验证方式见后续 REQ。

## 备注

本次测量期间板子运行的是出厂 PX4，非本仓库固件。回退方式：进 DFU 后把上面那份
2 MB 全片备份写回 `0x08000000`。
