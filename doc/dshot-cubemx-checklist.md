# R-DSHOT-1：CubeMX 生成交接清单

接收工作树：`D:/stm32hal/drone-H743-dshot`，分支 `codex/dshot300`。
基线：`aeca543e`，包含 MicoAir743v2 移植。不要在原工作树的未提交 LED 工作中混入本次生成。

## 当前阶段

纯 Driver 与宿主测试已实现。独立工作树 `.ioc` 已按下列参数准备好，**生成代码尚未同步**。
请重新打开 `D:/stm32hal/drone-H743-dshot/drone-H743.ioc`，核对后 Generate Code。
不要在原 `drone-H743` 工作树点生成，也不要在 BSP 接入完成前烧录这个中间状态。
只有作者完成生成、执行者核对后才进入 BSP 与运行链路接入；当前不得用新接线假定协议已经切换。

## 在本工作树的 drone-H743.ioc 中修改

1. UART8 → DMA Settings：删除 **UART8_RX / DMA1_Stream2**。保留 UART8 全局中断与 **UART8_TX / DMA1_Stream3**。
   蓝牙接收现用 `BSP_UART_MaintRxStart()` → `HAL_UART_Receive_IT()`，不是 DMA；不改波特率和 PE0/PE1。
2. TIM1 → DMA Settings：增加 **TIM1_UP / DMA1_Stream2 / DMA_REQUEST_TIM1_UP**。
   Direction=Memory to peripheral；Mode=Normal；Peripheral increment=Disabled；Memory increment=Enabled；
   Peripheral/Memory data width=Word；Priority=High；FIFO=Disabled；burst=Single。
   这里的 DMA burst=Single 与后续 TIM1 的 DCR/DMAR 两寄存器 burst 不矛盾，两者是不同机制。
3. NVIC：保留/启用 DMA1_Stream2_IRQn，抢占优先级=5，子优先级=0；生成的 IRQ 应只调用 TIM1_UP DMA 句柄。
4. TIM1：内部时钟、向上计数；PE9=AF1/TIM1_CH1，PE11=AF1/TIM1_CH2。
   CH3/CH4 保留既有 CubeMX 配置以减少无关改动，BSP 禁止启动这两路输出。
   CH1/CH2：PWM mode 1、High polarity、Fast mode Disabled、Pulse=0；GPIO speed=Very High、No pull。
   按当前 **TIM1 内核时钟 120 MHz**：PSC=0、Period=399、Repetition counter=0；启用 ARR preload。
   不改全局时钟树。若界面显示的 TIM1 时钟不是 120 MHz，记录实际值；Driver 可验证整周期时序，不能照抄 PSC/ARR。
5. TIM4 保持原样：PD12/PD13，PSC=119、Period=19999，对应 50 Hz 舵机 PWM；不修改其极性和脉宽。
6. Generate Code。保存 `.ioc` 和全部实际重新生成的文件，不手工拼接生成代码。

## 生成结果应包含

- `tim.h/tim.c` 中有 `hdma_tim1_up`，链接到 `htim1.hdma[TIM_DMA_ID_UPDATE]`。
- `usart.c` 不再初始化/链接 `hdma_uart8_rx`，UART8 TX DMA 和全局 IRQ 仍在。
- Stream2 IRQ 改为 `HAL_DMA_IRQHandler(&hdma_tim1_up)`，不能残留 UART8_RX 的处理器。
- `.ioc` 中 Stream2 只有 TIM1_UP 一个所有者；TIM4 与其他 DMA 保持原配置。
- GPIO/定时器初始化不自动启动电机输出。生成代码时可能重写 `freertos.c`，必须与该工作树基线核对。

## 后续由执行者接入（不是 CubeMX 操作）

- 新 BSP 设置 CCR1 起始、两次传输的 TIM DMA burst；使用 `.dma_buffer`、缓存 clean 和私有完成/错误回调。
- 500 Hz 最终仲裁点成对提交；停止码与物理禁用分开，DFU 的禁用在关中断上下文可用。
- CMake 编译选择默认 DSHOT300，PWM 回退初始化恢复 TIM1 的 1 MHz tick / 400 Hz 帧率。
- 完成双协议测试、实际构建及日志语义标识。物理验证交审核者，不由执行者连接设备。
