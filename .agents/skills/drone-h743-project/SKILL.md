---
name: drone-h743-project
description: Use when working on this repository's STM32H743 firmware project based on STM32CubeMX and FreeRTOS, especially for CubeMX-managed hardware changes, generated-code review, RTOS task design, STM32H7 memory/domain planning, and migrating code from other STM32 projects into this repo.
type: project
---

# drone-H743 Project Skill

## 何时使用

- 用户正在修改这个仓库里的 STM32H743 固件
- 任务涉及 `drone-H743.ioc`、`Core/Src/freertos.c`、`Core/Src/*` 外设初始化、`USB_DEVICE/*`、`App/*`、`BSP/*`
- 任务涉及外设开关、引脚复用、中断、DMA、USB、FreeRTOS 对象、内存布局、性能优化、代码移植
- 用户提到 STM32H7 的 cache、ITCM、DTCM、AXI SRAM、D2/D3 SRAM、域划分、任务栈、DMA buffer

## 项目边界

- 当前目标工程就是当前工作区根目录，也就是这个 `drone-H743` 仓库
- `drone-H743.ioc` 是 CubeMX 配置真源
- 主要由 CubeMX 生成并维护的文件：
  - `Core/Src/gpio.c`
  - `Core/Src/i2c.c`
  - `Core/Src/spi.c`
  - `Core/Src/tim.c`
  - `Core/Src/usart.c`
  - `Core/Src/freertos.c`
  - `Core/Src/main.c`
  - `Core/Src/stm32h7xx_hal_msp.c`
  - `Core/Inc/*` 对应头文件
  - `USB_DEVICE/*`
- 主要用户代码层：
  - `App/*`
  - `BSP/*`
- 除非非常明确，只在 CubeMX 生成文件的 `USER CODE BEGIN/END` 区域内写用户代码

## 强制流程 1: 涉及硬件或 CubeMX 托管对象时

只要改动会影响下面任一项，就必须先走 CubeMX：

- 引脚、复用、GPIO 模式、上下拉、速度
- 外设使能或实例变更，如 I2C/SPI/UART/TIM/USB
- 时钟树、时基、中断优先级、DMA、NVIC
- FreeRTOS 里的任务、队列、信号量、定时器等由 CubeMX 可配置的对象

执行规则：

1. 先告诉用户如何在 CubeMX 里改，写清楚入口页面、选项名、实例名、推荐值
2. 停下来等待用户自己在 CubeMX 里修改并重新生成代码
3. 用户说“已经生成代码”后，再检查：
   - `.ioc` 是否体现了预期配置
   - 生成代码是否出现了预期的 handle、init、IRQ、DMA、RTOS 对象
   - 原有用户代码是否需要回填或迁移
   - 还缺哪些手工修改
4. 明确告诉用户“生成代码检查结果”和“接下来要改哪些手工代码”
5. 只有在用户允许继续后，才编辑依赖这些硬件改动的用户代码

额外约束：

- 不要把手改 `.ioc` 文本当作常规方案；优先指导用户通过 CubeMX GUI 修改
- 不要先改生成的 `Core/Src/*` 再倒推 CubeMX
- 如果 CubeMX 生成结果和现有手写代码冲突，优先保护用户已有逻辑，并提出迁移到 `App/*` 或 `BSP/*` 的方案

## 强制流程 2: FreeRTOS 任务设计

这个项目已经使用 FreeRTOS。默认原则是：不要把复杂逻辑都塞进同一个任务，尤其不要把高栈占用和多职责任务继续做大。

必须遵守：

- `defaultTask` 只保留轻量启动职责，避免变成“什么都往里放”的总任务
- 任务按职责拆开，尤其是下面这类通常应单独任务化：
  - 传感器采集
  - 屏幕显示或 UI 刷新
  - 通讯收发
  - 控制环
  - 日志或存储
- 当某个任务出现下面任一特征时，优先考虑拆分新任务：
  - 栈需求明显增大，尤其接近或达到 `1024` 这一级别
  - 同时做阻塞 I/O、协议解析、格式化输出、显示刷新、控制计算
  - 运行频率、实时性、优先级需求彼此不同
  - 局部大数组、格式化缓冲区、复杂状态机越来越多

通信规则：

- 任务间通信优先使用合适的 RTOS 原语，并说明选择理由
- 典型建议：
  - 传感器采样数据: queue、message buffer、双缓冲
  - ISR 唤醒任务: task notification
  - 多事件聚合: event flags
  - 总线共享: mutex
  - 大块字节流: stream buffer
- ISR 保持短小，重活放到任务上下文

新增任务时的强制步骤：

1. 先告诉用户如何在 CubeMX 的 `Middleware -> FREERTOS` 中添加任务，以及推荐的优先级、栈大小、对象名称
2. 等待用户重新生成代码
3. 检查 `Core/Src/freertos.c` 是否生成正确
4. 再把任务体和业务编排放到 `App/*`

项目内额外约定：

- 当前 `freertos.c` 里 CubeMX 生成的 `stack_size` 形式是 `N * 4` 字节，因此 `.ioc` 里的任务栈数字要按这个习惯理解
- 已存在 `App/Src/app_tasks.c`，后续优先把任务入口包装、业务调度和模块组合放在 `App/*`，保持 `freertos.c` 尽量薄

## 强制流程 3: STM32H7 资源与性能规划

这个项目必须主动利用 H7 的特性，而不是只把它当“大一点的 F4”来写。

每当任务涉及性能、内存、DMA、任务栈、数据吞吐、控制计算时，都要给出一个简短的 H7 资源分配说明，至少覆盖：

- 热点代码放哪里
- 高频数据或任务栈放哪里
- DMA buffer 放哪里
- cache 是否会影响一致性
- 这个模块主要落在项目的哪一块域/内存区域

本项目采用“四块资源区”的思考模型：

1. `ITCM`: 最热、最短、最看重确定性的代码
2. `DTCM`: 只被 CPU 使用的高速数据、任务栈、控制结构体、临时计算缓冲
3. `D1 / AXI SRAM`: 较大的通用数据区、吞吐优先的数据区、共享堆或大 buffer
4. `D2 / D3 SRAM`: 更贴近外设和 DMA 的 buffer、低速共享数据、需要单独规划的常驻状态

必须提醒自己的事实：

- STM32H743 的硬件电源/总线域主要是 `D1/D2/D3`；这里把 TCM 单独拿出来，是为了项目内更好做计算和内存规划
- 不要默认 DMA 能安全访问 `DTCM`
- 只要 cacheable buffer 被 DMA 读写，就必须同步考虑 cache maintenance 或 MPU 非缓存区方案
- 在建议修改 linker、section、MPU、cache 策略之前，先读 `.ioc` 和 `STM32H743XX_FLASH.ld`，先解释方案，再动代码

当任务涉及这些主题时，先读 `references/h7-memory-domains.md`

当前项目联调进度和暂存结论，优先记到 `references/progress-notes.md`

## 强制流程 4: 区分当前工程与其他工程路径

这个项目后面会从其他项目移植代码，所以必须严格区分路径和归属。

每次移植或参考其他工程时，都要明确写出：

- `source path`: 外部工程的源路径
- `target path`: 本工程中的目标路径
- 依赖的外设实例、GPIO、DMA、IRQ、RTOS 对象
- 需要先在 CubeMX 中补齐的配置

禁止做法：

- 把外部工程的 `Core/*` 生成文件整块复制进来
- 不说明路径就说“按原项目那样改”
- 混淆“当前工程已有文件”和“外部项目文件”
- 不核对实例名就直接照搬，比如 `SPIx`、`I2Cx`、`USARTx`、中断名、句柄名

优先做法：

- 外部驱动优先落到 `BSP/*`
- 业务整合优先落到 `App/*`
- 当前工程的 `.ioc` 继续作为唯一硬件配置入口

## Ai-WB2 WiFi/TCP 调试记忆

当用户调 Ai-WB2-12F、CH340、TCP 透明传输、上位机 `tools/*`，或提到 `+SOCKET:97` 时，先按这里处理：

- 当前常用 WiFi: `Xiaomi_11FA`
- 当前常用密码: `2325972824`
- 当前 PC 有线连接 `Xiaomi_11FA` 时的已验证 IP: `192.168.31.189`
- TCP 透明传输默认端口: `6666`
- Ai-WB2 作为 TCP client 自动透明连接 PC: `AT+SOCKETAUTOTT=4,192.168.31.189,6666`

手动 AT 配置时必须提醒：

- 串口参数通常是 `115200 8N1`，发送结尾必须是 `CRLF`
- 不要整段粘贴 AT 命令；一次发一条，等 `OK`/事件/错误后再发下一条
- 如需先扫描环境 WiFi，用 `AT+WSCAN`；若不识别，再试 `AT+WSCANOPT=1` 后 `AT+WSCAN`
- 进入透明传输后，串口里的 `AT...` 会被当成 TCP 数据；退回 AT 模式才发 `+++`，且前后保持约 1 秒空闲

`+SOCKET:97` 排障优先级：

1. `+EVENT:WIFI_GOT_IP` 后出现 `+SOCKET:97 ERROR`，优先判断为 TCP socket 连接 PC 失败，不要先怀疑 SSID/密码。
2. 先确认 PC TCP server 已经在目标端口监听，再让模块重连或 `AT+RST`。常用命令：
   `python D:\stm32hal\drone-H743\tools\aiwb2_net_tool.py tcp-server --bind 0.0.0.0 --port 6666`
3. 若本机 `Test-NetConnection 192.168.31.189 -Port 6666` 不通，说明没有监听或被防火墙拦截。
4. 如端口已监听仍失败，检查目标 IP 是否为 PC 在同一 WiFi/有线 LAN 下的地址，再检查 Windows 防火墙入站规则。
5. 成功标志是 PC 端出现类似 `TCP client connected from 192.168.31.203:<port>`。

## 回答格式偏好

当用户要做一个和硬件相关的功能时，优先按下面顺序回答：

1. `CubeMX 需要怎么改`
2. `生成代码后我会检查什么`
3. `得到你允许后我再改哪些手工代码`

在总结里尽量明确区分：

- `生成文件`
- `USER CODE 区域`
- `用户自管文件`

并始终给出具体路径，避免含糊地说“那个文件”或“原工程里的代码”。
