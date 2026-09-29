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
Driver 只做电压/电流换算，BSP 负责 ADC 校准及原始采样。现有messageTask每约20ms读取一次PC1电流/PC0电压采样对，使用两rank、单项间断触发，任一rank失败时丢弃整对；开始时刻作为采样时间。不随后台存储或PC命令停顿，250ms过期判据保留，报告仅复制快照。电池低压门与ELRS回传见[bluetooth-battery.md](bluetooth-battery.md)。
不新增任务、DMA 或定时器；不在 IMU/稳定环执行 ADC 轮询，不改安全门、推力模型或电量保护。
STATUS? 增加 CURRENT 原始值、电压、电流、有效性、年龄、错误数和未校准标识。

## 上位机回读（R-CURRENT-2）

「传感器 → 电流计」消费现有 STATUS? 的 CURRENT 行，沿用连接和接收代次，不建立新连接。
提供立即读取和仅页面可见时的每2秒自动回读；慢链路或日志独占期间沿用既有发送拒绝机制。
读数标为最近回读，显示 ADC 原始值、电压、标称电流、回包有效性、饱和、未校准、样本年龄、采样/错误计数及 ADC 状态。
接收年龄以 transport 的 received_at 为准；3秒无 CURRENT、切换连接或断开均不显示有效电流；其他报文不刷新电流年龄。
缺字段/非法字段不沿用上次有效值；旧固件不返回 CURRENT 时说明未收到电流回包。
Contract：只显示本连接最近的完整 CURRENT；Boundary：页面/接收分派，不改固件采样和控制；Test seam：真实 C 报告格式到真实 DronePanel 队列及控件。
带宽按现有 STATUS 格式和字段范围保守估算约3600 B/次（含 CURRENT 与帧开销），0.5Hz约1800 B/s；
叠加既有 ARM 2Hz按800 B/s估算约2600 B/s，占57600/8N1有效5760 B/s的45%，低于60%预算。
这是静态估算，其他手动开启的遥测流另计；不改变当前遥测流参数。

## R-PWR-1：并入「电源」页，实时值改走遥测流

上面这一节描述的独立「电流计」页已退役。它与「电池电压」页都是 0.5 Hz 轮询，
而且两页都显示电流——同一个物理量在同一个分组里有两个可能不一致的读数。
两页合并为 `tools/panel_lib/pages/power.py`（「传感器 → 电源」），按数据的时间
尺度分成两半：

- **实时区**（总压 / 电流 / 平均单节 / 功率）走遥测流推送，通道 `batt_v` `batt_i`，
  页面可见才订阅、不可见就退订。**没有任何定时轮询**，原来两个 2 秒定时器已删除。
- **配置与诊断区**（串数 / 告警恢复阈值 / 电压与电流两路 ADC 计数 / ELRS tx 统计 /
  标称灵敏度与"未校准"提示）改为"打开页面取一次 + 手动『刷新诊断』"，仍走
  `BATTERY? <nonce>` 与 `STATUS?`。`BATTERY SET` 的 nonce 问答确认原样保留——
  写入确认必须能对上是哪一次写入，没法用推送替代。

`parse_current()` 搬进纯解码模块 `tools/panel_lib/current_monitor.py`（无 Tk，
可直接单测），解析严格度与本节原有描述一致。掩码 / 流开关 / 二进制 sink 三件
共享资源的仲裁规则见 [telemetry-protocol.md](telemetry-protocol.md) 的 Dashboard 契约。

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
