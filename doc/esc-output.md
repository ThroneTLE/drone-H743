# MicoAir743v2 电调输出

软件提供编译期选择，默认单向 DShot300。软件测试不是实机电调验证。

| 输出 | 绑定 | 用途 |
|---|---|---|
| M4 / PE9 | TIM1_CH1 | 电调 1 |
| M3 / PE11 | TIM1_CH2 | 电调 2 |
| M7 / PD12 | TIM4_CH1 | 舵机 1，50 Hz PWM |
| M8 / PD13 | TIM4_CH2 | 舵机 2，50 Hz PWM |

## 构建与回退

```powershell
cmake --preset Debug -DESC_PROTOCOL=DSHOT300
cmake --build --preset Debug
cmake --preset Debug -B build/PWM -DESC_PROTOCOL=PWM
cmake --build build/PWM
```

切换协议必须另行烧录；不新增 Flash 参数。TIM1_UP 独占 DMA1_Stream2，UART8 接收保留中断，
TX 继续使用 Stream3 DMA。PWM 后端启动前恢复 TIM1 的 1 MHz 计数/400 Hz 帧率，TIM4 不参与切换。

## 接口与时序

- `drv_dshot` 的编码与交错数组来自固定 PX4 v1.16.0，来源和许可见 `tests/fixtures/dshot_px4/README.md`。
- `BSP_PWM_SetEscPulse/GetEscPulse` 在 DShot 模式下表达等效微秒命令；setter 暂存，500 Hz 最终仲裁点的 `BSP_PWM_CommitEsc` 一次提交两路。
- 1100 us 对应 DShot 0；1101..1940 us 映射到 48..2047。不保证与原 PWM 有相同转速或推力。
- `BSP_PWM_DisableEsc` 立即物理禁用；0 us 禁用与发送停止码不同。原 RC 丢失、验收、DFU 路径保留。
- `BSP_DShot_Submit` 接受两路码与有效位掩码，返回 OK/BUSY/INVALID/ERROR。无循环 DMA、不排队重播旧油门。
- 错误锁存并关闭输出，只有显式初始化/重启才尝试恢复。准备期间禁用会取消该次提交；完成回调不会打开输出。
- `BSP_DShot_GetSnapshot` 中 code 是最后接受的码，enabled 是通道使能状态；必须连同 fault/busy 解读。
- 120 MHz 内核时钟：PSC=0、ARR=399，bit 0/1 高电平为 150/300 tick。实际时钟来自 RCC 配置。
- 两路 CCR 交错；先 UG 锁存零，再由更新事件搬入预装载。数据前有低电平准备周期，两个零尾槽排空预装载。
- 160 字节缓冲在 `.dma_buffer`，32 字节对齐并在发送前 clean。禁用使用有界寄存器操作，不依赖 HAL tick/IRQ。
- 不支持双向遥测、特殊命令、转向设置或电调刷写；DMA 完成不代表电调收到或执行。

## 诊断与日志

`PWM?` 增加 `ESC protocol=...` 行。DShot 明确 `command_unit=pwm_equivalent_us ack=unavailable`，
报告 code/enabled/busy/fault/submitted/completed/busy_rejected/errors/cancelled。
原 esc_us 是等效命令；定时器诊断中的 CCR 是硬件 tick，不能当作微秒命令。

FlightLog V10 头末尾 4 个保留字节使用 `[0xD5,1,protocol,0]`：1=PWM、2=DSHOT300；
头/记录大小不变，标记受原 CRC 保护。导出 meta 的 sectors 与 CSV 每条记录带 esc_protocol/motor_command_unit。
旧零保留区标为 legacy_unspecified；不根据当前固件推测历史协议，不改变历史 CSV 数值。

## 审核者验证

拆桨测 M4/M3 位序、校验、帧率、末位与中止，同时检查 M7/M8 的舵机 PWM。
记录 AM32 55A 固件版本；验证上电停止、两路低油门、失联禁用、DFU 禁用与重连。
并发 IMU/遥测/日志负载下记录 DMA 错误、控制周期和蓝牙收发；实际转速/推力差异另行实测。
