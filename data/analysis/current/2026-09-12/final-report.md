# R-CURRENT-1 软件交审

工作树 `D:/stm32hal/drone-H743-dshot`，分支 `codex/dshot300`，基线 `d7cba69f`。
范围：只读电流采样与 STATUS? 诊断；没有烧录、复位或实机采样。

## 硬件来源与实现

- 作者确认 AM32 55A 原配排线包含 Curr/GND。传感器位于电调，飞控 PC1 / ADC1_INP11 读取模拟信号。
- 标称 12.75 mV/A 来自微空 AM32 55A 官方手册，引用见 `doc/current-sensor.md`。
- Driver 纯换算，BSP 负责 ADC 自校准和有界轮询，App 在既有 backgroundTask 以目标 20 ms 周期采样。
- 不新增 DMA、IRQ 或任务，不改 DShot 输出、安全门、推力模型、参数存储和历史校准文件。
- 默认 3.3 V 参考、输入增益 1、零偏 0；这些是标称假设，`calibrated=0`。
- 原始 ADC/电压可诊断；过期、失败或高端饱和时 current_a=NaN、valid=0。零 ADC 不推断线路断开。
- STATUS? 的 CURRENT 行显示原始值、电流、有效性、校准标志、年龄及错误计数；不在命令处理路径做 ADC I/O。

## CubeMX

PC1 共享 ADC 映射必须带 `ADC1_INP11,IN11-Single-Ended`，只有模拟引脚不启用 ADC 外设。
先经 CubeMX 读取/保存回验，再使用 `config load <ioc>` / `project generate` 在原位置生成。
生成 Core/ADC HAL 文件由工具完成，现有 Core 用户任务代码保持原样。
批处理摘要 `cubemx-summary.txt`；原始工具输出本地保留于 `cubemx-generate-auto.txt`，哈希见 validation.json。
中间失败尝试没有作为成功证据；临时生成目录移至忽略的 `.tmp/`，未混入源码。

## 验证原文

| 检查 | 结果 | 文件 |
|---|---|---|
| 全量 pytest | 1525 passed in 333.17s，无 skipped | pytest-full.txt |
| 电流/DShot/生成/隔离专项 | 26 passed | focused-final.txt |
| 默认 DSHOT300 Debug | 编译链接成功，零警告 | build-debug-final.txt |
| PWM Debug | 编译链接成功，零警告 | build-pwm.txt |

测试使用真实 C 模块和模拟 ADC 输入，验证代码，不代表实际电流精度。
物理串口和烧录工具调用均为零；固件 ELF 哈希、默认比例等见 validation.json。
STATUS? 旧函数冻结契约只剥离本次新增的唯一快照报告委托，原函数剩余部分仍按旧哈希验证。

## 工作模式与待审核项

- 默认 REQ：模拟量 Driver/BSP/后台采样接入。
- protocol-telemetry：复用 STATUS? 增加 CURRENT 快照，未增加写命令。
- 本次生成配置调试属于新功能实现；未修改既有安全判据。

R-CURRENT-1 待审核。审核者需要确认实际 VDDA、Curr 输入是否有额外缩放，
并在同一负载下对照外部电流表确定零偏/增益；ADC 芯片内部自校准不等于电流计实测校准。
