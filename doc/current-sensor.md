# R-CURRENT-1：AM32 55A 电流采样

## 硬件与标称换算

飞控的 PC1 接收外部 Curr 信号，对应 ADC1 通道 11。电流传感器在配套 AM32 55A 电调上。
作者确认使用包含 Curr/GND 的原配排线；该读数是 Curr 线路测得的总电流，不能拆成各电机电流。

- [飞控官方手册](https://micoair.cn/zh/docs/flight-controller/micoair743/micoair743v2-flight-controller-manual)：PC1 为电流输入。
- [AM32 55A 官方手册](https://micoair.cn/zh/docs/esc/55a-am32-esc)：电流输出 **12.75 mV/A**。
- 标称计算：`I_A = (raw / 65535 * Vref_V * input_scale - offset_V) / 0.01275`。
- 默认参考电压 3.3 V、input_scale=1、offset=0，均为标称配置；不套用其他电调的 40.2 A/V 默认比例。
- 参考电压、输入电路增益、零偏和传感器比例都可由 Driver 配置表达，`calibrated=0` 直到实测确认。
- ADC 零值无法证明电流线连接完好；高端饱和/过期/采样失败不能被当成有效电流。

## 软件边界

默认 REQ 模式负责常规 Driver/BSP 分层，复用 STATUS? 回包时进入 protocol-telemetry 模式。
Driver 只做电压/电流换算，BSP 只做 ADC 校准及原始采样，App 在 backgroundTask 中每 20 ms 尝试一次。
不新增任务、DMA 或定时器；不在 IMU/稳定环执行 ADC 轮询，不改安全门、推力模型或电量保护。
STATUS? 增加 CURRENT 原始值、电压、电流、有效性、年龄、错误数和未校准标识。

## CubeMX 交接

执行者准备独立工作树 `D:/stm32hal/drone-H743-dshot/drone-H743.ioc`。可用 GUI 或 CubeMX 官方 CLI 生成；不手改 Core。
PC1 是共享通道：必须有 `SH.ADCx_INP11.0=ADC1_INP11,IN11-Single-Ended`，只有模拟引脚配置不足以启用 ADC1。
新增 ADC1 / PC1 通道 11，16 位、单端、单次软件触发、DR 读取、无 DMA/IRQ。
独立 PLL2 使用 HSE 8 MHz / M4 * N128 / P8 = 32 MHz，ADC 异步除以 4 得到 8 MHz；
采样时间 387.5 周期。PLL1、PLL3、TIM1 DShot、TIM4 舵机和既有 DMA 保持原配置。
生成后应有 adc.c/h、hadc1、MX_ADC1_Init() 调用，ADC HAL 驱动及模块开关由 CubeMX 添加。

本次已用 CubeMX 官方 CLI 成功生成。脚本使用现有工程位置，不另设 `project path`：

```text
config load D:/stm32hal/drone-H743-dshot/drone-H743.ioc
project generate
exit
```

通过安装目录的 `jre/bin/java -jar STM32CubeMX.exe -q <脚本>` 执行。
不要让另一个仍打开旧配置的 GUI 窗口覆盖 `.ioc`。

## 验证与审核

宿主测试检查公式/边界/NaN/饱和和状态新鲜度；生成门核对真实 ADC 通道/时钟/分辨率；
全量 pytest、Debug 构建及 DShot 回退回归。实测精度需用外部电流表在同一负载下对照，
零偏与增益不能靠上位机读数自身宣称校准。默认不操作目标板。
