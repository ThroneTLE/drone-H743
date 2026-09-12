# DShot / Current 执行者自查

- 基线：`af84478d`，工作树 `D:/stm32hal/drone-H743-dshot`，分支 `codex/dshot300`；开工干净。
- 范围：R-DSHOT-1 与 R-CURRENT-1；本次不作实机验收，不操作目标板。
- 模式：默认 REQ（复核交付）、dshot-esc（输出生命周期）、protocol-telemetry（诊断口径复核）、横切修 bug（DMA 错误锁存缺陷）。

## 已复现的 P1：DMA 完成与错误同次中断会漏锁存

真实来源为仓库 `Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c` 的
`HAL_DMA_IRQHandler`：先更新 TE/FE/DME ErrorCode，再处理 TC 并调用完成回调，最后才调用错误回调。
原 BSP 完成回调不查 ErrorCode，先置 busy=0 并累加 completed；后续错误回调因 busy!=2 忽略错误。
结果是故障帧被计为完成，后续 Submit 仍可发帧，违反本 REQ 的错误锁存契约。
运行时影响限于 DShot 后端；PWM 宏装置也直接调用 DShot BSP 做单元验证，不代表 PWM 输出走 DMA。

旧测试只分别调用完成/错误回调，没有经过真实 HAL 的混合标志分支，因此漏检。
新增装置从当前 vendor 文件直接提取 DMA1/2 分支原文编译执行，不复制改写其控制流程；
寄存器常量同时用 ARM 编译器对真实 ST 头文件做静态断言。
宿主只模拟寄存器存储，不模拟总线仲裁、W1C 时序或实际输出波形。

- 修前：`self-review-red.txt`，2 failed / 10 passed，两个协议编译配置均复现。
- 修复：完成回调检查 HAL ErrorCode，有错立即锁存/禁用，且不增加完成数；后续错误回调不重复计错。
- 修后专项：`self-review-focused.txt`，35 passed；包括 TE/DME/FE 单独及伴随 TC、锁存拒绝、显式初始化后恢复。
- Contract：任何带 HAL 错误的活动 DMA 帧都不能进入成功完成状态。
- Boundary：仅 BSP 完成回调；不改 HAL、CubeMX、安全阈值、输出仲裁或协议映射。
- Test seam：真实 BSP + 当前 vendor DMA IRQ 分支 + 宿主寄存器。

## 其余复核范围

- DShot：PX4 固定源/编码、交错缓冲、ARR+1、CCR 预装载与双零尾槽、D2 缓冲/缓存清理、忙拒绝、禁用取消、错误恢复、成对提交、PWM 回退。
- 生成配置：TIM1_UP 独占 Stream2、UART8 RX 中断及 TX DMA、TIM4 舵机；ADC1 PC1 通道11，PLL2 独立时钟、无 ADC DMA。
- Current：后台采样、HAL ADC 校准/超时、标称公式、NaN/饱和/过期、时间戳回绕、STATUS 快照及 calibrated=0。
- DFU：检查实际关中断后两路禁用路径；不执行复位或 DFU。
- 限制：电流增益/零偏/参考电压未实测；DShot 波形、电调响应、并发负载与失联响应仍须审核者实测。

## 最终验证

- DSHOT300 Debug：`self-review-build-dshot.txt`，实际重新编译/链接，零警告；DMA 缓冲位于 map 的 `0x30000000`，大小160字节。
- PWM Debug：`self-review-build-pwm.txt`，实际重新编译/链接，零警告；`build/PWM/CMakeCache.txt` 中 ESC_PROTOCOL=PWM、CMAKE_BUILD_TYPE=Debug。
- 第一轮全量：`self-review-full-dshot.txt`，1 failed / 1526 passed in 408.35s；唯一失败为既有 Tk 重绘性能测试中位数10.64ms超过10ms。
- 单独复查：`self-review-ui-recheck.txt`，1 passed；保留原性能门限和实现，不据此把首轮失败改写成通过。
- 第二轮全量：`self-review-full-final.txt`，1 failed / 1526 passed in 547.58s；唯一失败为既有 Scope 绘图性能测试中位数8.25ms超过5ms，第一轮失败的 Dashboard 用例本轮通过。
- 两项性能测试合并复跑：`self-review-ui-final.txt`，2 passed in 5.76s。门限和 UI 源码均未改动。
- 两轮宿主套件中的 BSP 装置都自行以 PWM/DSHOT300 两种宏编译；不是在目标板运行 ELF。
- 最终结论：已复现的 DShot P1 已修复并有针对性回归；本次未发现其他可确认的 DShot/Current 阻塞代码缺陷。**全量回归门未闭合**，不能把单项重跑通过改写成全量通过。
- 观察到原工程上位机进程运行中，未关闭或操作它；并发桌面负载可能影响墙钟性能，但未证明其为根因。保留两轮原始失败，性能稳定性留作审核关注项，不在当前 REQ 顺手重构 UI。
- REQ 保持待审核，并明确本次回归缺口；硬件调用为零。
- 治理复核：`self-review-governance-verified.txt`，PIPELINE/索引契约10 passed；前两次命令误写索引测试路径，未执行测试，原文保留在本地 governance/governance-final 日志。
- 手写源码/测试/PIPELINE 的 diff whitespace 检查通过；原始构建与 pytest 输出里的空行/空白按原样保留。
