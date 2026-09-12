# R-DSHOT-1 软件交审

工作树：`D:/stm32hal/drone-H743-dshot`；分支：`codex/dshot300`；基线：`aeca543e`。
状态：待审核。未烧录、未复位、未连接物理串口、未运行电机。

## 变更

- PX4 v1.16.0 固定提交的单向编码移植；原始快照、BSD 许可、SHA256 和移植差异随代码交付。
- M4/PE9=电调1、M3/PE11=电调2；TIM1_UP + DMA1_Stream2 从 CCR1 连续更新两个通道。
- UART8_RX 保留原来的中断接收，UART8_TX DMA 不变；M7/M8 的 TIM4 舵机 PWM 保持 50 Hz。
- 编译选择 `ESC_PROTOCOL=DSHOT300|PWM`，默认 DSHOT300。正常两路在既有 500 Hz 最终仲裁点统一提交。
- 1100 us -> STOP；1101..1940 -> 48..2047。特殊命令和双向遥测不开放；物理禁用与 STOP 分开。
- 有界禁用、忙拒绝、错误锁存、缓存对齐与 clean、两个零尾槽。不会循环重放旧油门，不自动降级 PWM。
- `PWM?` 的 ESC 行区分协议、等效命令、实际码与 DMA 状态，并明确没有电调 ACK。
- V10 日志头保留区加带版本标记；meta 与 CSV 记录携带协议/命令单位，历史未标记数据不倒推。
- 生成代码由作者提供，实际语义差异独立提交；没有手改 Core/USB 初始化代码。

## 验证原文与范围

| 项目 | 结果 | 原文 |
|---|---|---|
| DSHOT300 上下文完整回归 | 1518 passed in 304.07s | pytest-final-DSHOT300.txt |
| PWM 上下文完整回归 | 1518 passed in 280.02s | pytest-final-PWM.txt |
| 编码/BSP/生成/日志/DFU/时基专项 | 68 passed | targeted-final.txt |
| 默认 Debug 构建 | 实际编译链接成功，零警告 | build-dshot-final.txt |
| PWM Debug 构建 | 实际编译链接成功，零警告 | build-pwm-final.txt |

两次 pytest 均未跳过，硬件护栏记录物理串口及烧录工具调用为零。
pytest 本身是宿主验证，不是运行 ELF；BSP 装置在每次回归中分别以两个编译宏构建并运行真实 C。
时序装置建模 CCR 预装载与 DMA 更新顺序，不能替代实际芯片波形测量。
ELF 路径、SHA256、CMake 协议与测试输出汇总在 `validation.json`。

首次完整回归暴露的旧优先级测试已独立修复，原始失败保留在 `pytest-dshot.txt`。
新工作树缺失的既有 FlightLog 测试夹具从原工作树原样复制，哈希见 `fixture-provenance.json`。

## 工作模式

- 默认 REQ + dshot-esc：作者明确批准的移植、BSP 与输出集成。
- protocol-telemetry：增加诊断行与日志协议来源标记，旧值不重解释。
- 横切修 bug：索引容量越限只压缩摘要密度；优先级测试误读注释改读真实字段。两项均独立 fix 提交。

## 审核与回退

详见 `doc/esc-output.md`。审核者需要拆桨确认两路编号、位时序/校验、停止与失联、
DMA 并发负载、蓝牙及舵机 PWM；记录 AM32 55A 固件版本和配置。
数据数值映射不代表与 PWM 同转速/同推力，不能据此宣称原推力模型或飞行验收成立。
`build/Debug` 是 DSHOT300，`build/PWM` 是 PWM 回退；任何烧录仍归作者/审核者。
