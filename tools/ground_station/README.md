# drone-H743 专属上位机与飞控自检架构

> **定位**：面向 `drone-H743` 飞控的跨平台可视化调试地面站与自检工作台，基于
> [Serial-Studio](https://github.com/Serial-Studio/Serial-Studio) 做特异性扩展。
>
> 本文记录**架构结论与已知陷阱**；进度、决策时间线与下一步见
> [ROADMAP.md](ROADMAP.md)。

---

## 0. 先读这一节：当前真实状态

这份文档前半部分曾经把"设想"和"已完成"混在一起写，误导过人。现在分清楚：

| 部分 | 状态 |
|:--|:--|
| 固件遥测通道 schema（`TELEM?`） | ✅ **已实现**，有契约测试 |
| 固件 Action 安全状态机（`app_action.c`） | ✅ **已实现并验证**，但**尚未接入命令层**（没有任何命令能启动它） |
| `Drone-H743-GCS.ssproj` | ⚠️ **仿真器演示壳**，见下 |
| 控制脚本 / 自动建表 | ❌ 未做 |
| 交互控件（PID 调参台等） | ❌ 未做，但路线已探明并有原型包 |
| 自检向导 / 日志回放 | ❌ 未做 |

### `Drone-H743-GCS.ssproj` 是对着仿真器做的，不是对着固件

它连的是 `drone_simulator.py`（TCP `127.0.0.1:6666`、`/*$...*/` 包裹的 CSV），
而固件实际发的是 **VOFA JustFloat 二进制**（float32 LE + 尾 `00 00 80 7F`）加裸文本行。

顶部 9 个按钮里，**只有 `PING` 和 `STATUS?` 是固件真实存在的命令**。其余 7 个接上真机
一律返回 `ERR unknown cmd`：

| 按钮 | 固件真实情况 |
|:--|:--|
| `ARM` / `DISARM` | 顶层不存在，只有 `IDENT ARM` / `MOTOR ARM` 这类**子命令** |
| `CALIB IMU` | 实际是 `IMUCAL BEGIN/DATA/END/APPLY/REVERT/COMMIT` 六步流程 |
| `CALIB BARO` | 不存在 |
| `SERVO TEST` | 有 `SERVO`，但没有 `TEST` 子命令 |
| `CONFIG SAVE` | 实际是 `SAVE` |
| `REBOOT` | 全仓库搜不到 |

UI 布局和控件选型是实打实的成果，别推倒；但要清楚它离真机还隔着一整个协议适配层。

---

## 1. 核心工程思想：上位机不持有危险动作的时间轴

判据很简单：**上位机进程被 `kill -9`，物理世界会不会停在危险态？**

```python
# 反模式：时间轴在上位机手里
send("SERVO 1 500"); time.sleep(2); send("SERVO 1 1500")
# 这两行之间进程被杀 / USB 被拔 / 用户 Ctrl+C
# -> 舵机永远停在 500，堵转发热，固件一无所知
```

会 → 这个动作必须是 Action，它的时间轴、超时、急停必须闭环在**固件端**。

```text
[ 上位机 ]  查询状态 / 原子指令 / 发起任务 / 监听进度
     |
[ USB CDC | Wi-Fi | UART ]
     |
[ STM32H743 FreeRTOS ]  状态互锁 / 硬件超时 / 任务状态机闭环
```

### API 四分类

| 类别 | 判据 | 现有例子 |
|:--|:--|:--|
| **Query** | 只读、幂等、随时可调 | `STATUS?` `IMU?` `PID?` `TELEM?` |
| **Command** | 瞬时、原子、无持续危险 | `PARAM k=v` `SAVE` |
| **Action** | 跨时间、独占执行器、中断有危险 | 见第 3 节 |
| **Stream** | 周期推送、可丢包 | VOFA JustFloat |

这个分级的实际作用：**Command 可以无脑重发，Action 绝对不能。** 网络抖动时重发 `SAVE`
无所谓，重发 `MOTOR_SPIN START` 就是灾难 —— 所以 Action 必须带 `action_id` 做幂等。

---

## 2. 遥测通道 schema（已实现）

### 为什么需要它

改造前，"第 N 个 float 是什么"这个映射同时存在于三处：`freertos.c` 的填充代码、
`VOFA_task` 上方的注释、以及 `.ssproj`。没有任何机制保证一致，而且**已经漂移过**
——旧注释漏了 `[22] vel_z_kd`。插入一个通道会让上位机所有曲线静默错位，不报错，
只是数值"有点怪"。

### 现在的做法

唯一事实源是 [`App/Inc/app_telemetry.h`](../../App/Inc/app_telemetry.h) 的
`APP_TELEM_CH_*` 枚举 + [`App/Src/app_telemetry.c`](../../App/Src/app_telemetry.c)
的元数据表。`freertos.c` 用枚举名下标，帧长与周期从表推导。

```text
< TELEM?
> TELEM ver=1 n=28 rate=40 page=6 hash=01297B96

< TELEM CH from=0
> TELEM CH idx=0 name=roll unit=deg min=-180.000 max=180.000 grp=attitude
> ... (至多 6 条)
> TELEM PAGE from=0 count=6 next=6      # next=-1 表示到表尾
```

上位机连上后拉取，用 `ensureDashboard()` 自动建表；缓存 `hash → 已建好的仪表盘`，
重连时 hash 未变就跳过重建。

**两个设计要点，改动前请先理解：**

- **分页不是为了好看。** `uartTxQueue` 深度 32 且**满时丢弃最旧的一条**
  （见 `APP_Control_QueueText`），28 条通道一次推进队列在慢链路上会静默丢掉开头几条，
  而 schema 丢一条就会让上位机建错表。单次回包压到 7 条，且天然可重试。
- **通道表只可追加。** `tests/test_telemetry_schema_contract.py` 抄下了改造前的
  28 通道顺序做前缀断言。既有解码器（`vofa_serial_capture.py`、`drone_tcp_panel.py`）
  和所有历史飞行日志都按位置索引，重排或中间插入会让它们全部错位。

---

## 3. Action 安全契约（已实现，未接线）

[`App/Src/app_action.c`](../../App/Src/app_action.c) 已实现并通过 26 项验收，
但**还没有任何命令能启动它** —— 接线需要在 `app_control.c` 加命令解析，并提供
具体任务的 `begin/step/safe_reset` 回调。

### 为什么它不碰 HAL / RTOS / printf

因为它的正确性**无法在真机上验证**。恰恰是超时、断线、急停这些路径最难在实机可靠
复现，而本项目当前无法实机烧录。所以：

- 时间靠**参数注入**（每个入口收 `now_ms`），模块内不调 `HAL_GetTick`
- 执行器靠**回调注入**（`APP_ActionDescriptor`）
- 零 RTOS 调用、零 printf

代价是调用方要传时间；收益是"第 1.5 秒拔掉 USB，第 4 秒舵机必须回中"变成了确定性
单元测试。**接线时请勿在模块内部引入时钟或阻塞调用**，`tests/test_action_contract.py`
里有源码守卫会直接报错。

### 锁死的不变量

| | 内容 | 为什么重要 |
|:--|:--|:--|
| I1 | 离开 RUNNING 必定且**恰好一次**调用 `safe_reset` | 这是断线/超时路径上唯一的保护，没有第二道防线 |
| I2 | 终态后继续 Tick 不再复位 | |
| I3 | 互锁拒绝的 START 不调 `begin` 也不调 `safe_reset` | 多余的复位本身就是一次意外的执行器动作 |
| I4 | 同一 `action_id` 重复 START 幂等 | 链路抖动时重发 START 是常态，重启一次舵机扫频就是事故 |
| I5 | 时间比较走无符号差值 | `HAL_GetTick` 在 2^32 ms 回绕，写成 `now > start + timeout` 会漏判超时，意味着舵机永远不回中 |

另外：心跳丢失（可配，如 500ms）早于硬超时（如 4000ms）中止；**携带错误 `action_id`
的陈旧心跳不能续命**（否则上个任务残留的心跳会悄悄关掉新任务的断线保护）；上位机请求
的超时会被钳到描述符上限（不能让上位机说服固件放弃自己的安全限制）。

### 接线时的一条硬约束

`APP_Action_Tick()` 必须由一个**不会被通信阻塞**的固定周期任务调用。如果挂在处理
上位机命令的任务上，那个任务因 UART/Flash 阻塞时看门狗会跟着一起死，断线保护就是假的。

### 待实现的具体任务

| 任务 | 要点 |
|:--|:--|
| `SERVO_SWEEP` | 全行程扫频；硬看门狗超时无条件回中并释放 PWM |
| `MOTOR_SPIN` | 油门硬限幅（如 15%）、单次时长硬限幅（如 2000ms）、任何异常立即关 PWM |
| `SENSOR_HEALTH` | I2C/SPI/UART 连通性与底噪合理性 |

固件里已有的 `IMUCAL` / `ACCEPT` / `IDENT` / `FLOG` 是同类多步骤任务，且 `IMUCAL`
已具备两段提交（`APPLY` 生效不落盘 / `COMMIT` 写 Flash）、CRC 校验、
`SetImuCalibrationCandidateArmLock` 硬互锁。**后续应把它们归一化到同一套 Action 外壳，
而不是再造一套**，否则上位机要写两遍状态跟踪。

---

## 4. 安全互锁矩阵

| 场景 | 风险 | 固件处理 | 上位机表现 |
|:--|:--|:--|:--|
| 已解锁时触发自检 | 飞行中误触导致执行器失控或基准归零 | 互锁拦截，返回 `ERR_ARMED`，不启动也不复位 | 按钮置灰 + 危险提示 |
| 通信中断 / 上位机崩溃 | 舵机扫频中途断线 | 心跳丢失先中止；硬超时兜底；`safe_reset` | 显示 Disconnected |
| 校准中机体晃动 | 错误姿态基准导致翻机 | 方差超标拦截，不应用零偏 | "请保持静止后重试" |
| 紧急中止 | 现场险情 | 立即终止并断能 | 显著的红色急停按钮 |

---

## 5. Serial-Studio 能力探底（**最重要的一节，能省下几周**）

结论基于 Serial-Studio 源码、内置扩展包，以及用构建好的二进制
`--dump-api-schema` 实跑导出的 [`ss-api-schema-gpl3.json`](ss-api-schema-gpl3.json)
（GPL3 构建实际注册的 256 条命令）交叉验证。

### 5.1 不要 fork。交互式 UI 用 widget extension

**这是最关键的一条。** 早期计划是"全量 fork 改造"，探底后证明**没有必要**：

- 扩展包 = `info.json` + **一个 QML 文件**，装到工作区目录，不进应用包
- `scope` 支持 `"dataset"` 和 **`"group"`**（整块面板）
- 官方文档原文：**"Widget extensions are not a Pro feature: they load in GPL builds
  and in commercial builds alike"**
- 内置的 `compass` / `datagrid` 就是用这套机制做的（见 `app/rcc/extensions/widget/`）
- 信任模型原文：**"no sandbox, no capability boundary"**，与主程序同权限

**写设备的通路**（结构上闭合，不是猜的）：

```
ModuleManager::registerQmlTypes():
  qmlRegisterType<API::TerminalBridge>("SerialStudio", 1, 0, "ApiTerminalBridge")   <- 无条件
  qmlRegisterType<Widgets::ExtensionData>("SerialStudio", 1, 0, "ExtensionDataModel")
  #ifdef BUILD_COMMERCIAL      <- 商业块在这两行之后才开始
```

扩展**必须** `import SerialStudio` 才能拿到 `ExtensionDataModel`（四个必填属性之一），
而 `ApiTerminalBridge` 就在同一个模块、同一次注册调用里，且在商业门控之上。
`TerminalBridge::run("scope.verb {json}")` 走 `CommandHandler` 的 Trusted origin，
可执行**任意 API 命令**。

原型包见 [`extensions/org.drone-h743.control-panel/`](extensions/org.drone-h743.control-panel/)。
安装：整个目录复制到 `~/Documents/Serial Studio/Extensions/widget/`，重启，
在工程编辑器里给某个 group 选该控件。

> **仍待人工确认**：首次运行的信任对话框、写设备是否弹确认框
> （`io.writeData` 文档称 "in-process scripts are not prompted"，未实测）。

### 5.2 Canvas / Painter 是死路，别规划它

`app/src/UI/Widgets/Painter.cpp` 的头是
`SPDX-License-Identifier: LicenseRef-SerialStudio-Commercial` —— **纯商业授权，不是双授权**，
整个文件被 `#ifdef BUILD_COMMERCIAL` 包住。且 `CMakeLists.txt` 强制 `BUILD_GPL3=ON` 时
`BUILD_COMMERCIAL=OFF`；反过来开商业构建会在 configure 阶段拿 license key 去
Lemon Squeezy 验证，验不过 `FATAL_ERROR`。

它有全套鼠标事件和 `deviceWrite`，看起来非常适合做 PID 滑条 —— **但在 GPL3 构建里不存在**，
而且改 ifdef 属于授权问题不是技术问题。用 5.1 的扩展机制代替。

### 5.3 控制脚本能做 Action Manager，不需要 C++

项目自带 `setup()` / `loop()` 控制脚本，跑在独立工作线程：

| 能力 | 说明 |
|:--|:--|
| `io.writeData()` / `console.send()` | 下发命令（文本协议用后者，会附加行结束符） |
| `io.getLatestFrame()` / `newFrame()` | 读回包，带 `ageMs`（超时判据用它，别拿 `timestampMs` 跟 `Date.now()` 比） |
| `delay(ms)` | 节拍，且**暂停 2000ms 运行时看门狗** |
| `ensureDashboard(spec)` | 声明式建组/建通道，幂等且 memoized —— `TELEM?` 自动建表就用它 |
| `tableGet/tableSet` + `refreshDashboard()` | 共享变量与渲染 |

**生命周期陷阱**：每次连接都是**全新引擎**，顶层变量全部重置、`setup()` 重跑。
不要设计任何跨连接存活的状态。

### 5.4 GPL3 构建缺失的能力

用实跑导出的 schema 与完整 SDK 符号表做差集得出：

| 缺失 | 影响 |
|:--|:--|
| `notifications.*`（全部 8 条） | 通知中心不可用 → 告警改用 LED 控件或扩展包自绘 |
| `sessions.*` | 会话录制/回放不可用 → 用 `csvPlayer.*`（GPL3 里有）或已有 Python 工具 |
| `mdf4Export/Player`、`project.mqtt.*`、`project.painter` | Pro |
| `io.modbus/canbus/opcua/audio/hid/usb/process` | Pro 驱动（本项目用 TCP/串口，不受影响） |

### 5.5 Action 的 txData 没有占位符替换

`DataModel::get_tx_bytes()` 只做转义序列解析，**没有任何变量替换机制**。所以顶部
工具栏的 Action 按钮只能发固定字符串 —— **PID 滑条这类"运行时决定数值"的交互
无法用 Action 实现**，必须走 5.1 的扩展控件。

### 5.6 子模块需要 Qt 6.7 / MinGW 移植补丁

上游针对 Qt 6.8+ 与 MSVC。子模块提交 `4bbf511` 带着一批 `#if QT_VERSION >= 6.8.0`
门控、LuaJIT 在 MinGW 下的 VM 目标格式修正等，**没有它们编译不出来**。
该提交尚未推送到 origin，换机器需先取得。它同时含一批往硬编码绝对路径写日志的
临时调试插桩，去除方式记在其提交信息里。

---

## 6. 已知未解决问题

### 6.1 文本回包与二进制遥测混在一条链路上

`APP_VOFA_SendFloats` 发的是裸浮点 + 4 字节尾，`APP_Control_QueueText` 发的是裸 ASCII 行，
**没有类型标记**。float 的字节里完全可能出现 `\n` 或可打印字符，解析器无法可靠区分。

**连接时取 schema 不受影响** —— `vofaStreamActive` 默认为 `0`，要发 `Sensor_Data:1`
才开流，所以时序天然干净：

```
连接 -> TELEM? / TELEM CH（流还没开，纯文本无歧义）
     -> ensureDashboard 建表
     -> Sensor_Data:1 开流
```

但流跑起来之后任何命令回包仍会混在浮点里。**Action 状态轮询必然要在流开着时发命令，
所以这是接线 Action 之前必须先解决的**。三个选项：

1. 取数据前先 `Sensor_Data:0` 停流 —— 最省事，但停流是异步的，有竞态
2. **给遥测帧加一个与文本不可能冲突的头**（如 `0xAA 0x55 <len16> <payload> <crc16>`），
   解析器先找魔数，找不到按行当文本 —— 改动集中，推荐
3. 全部走统一封包（`app_proto.h` 里那个被注释掉的 `APP_ProtoFrame`）—— 最干净但要
   重写所有现有 Python 工具

### 6.2 命令契约尚未整理

`app_control.c` 有 **51 条顶层命令、56 条子命令、274 个回包发射点**。
建议**不要**追求一次做全 —— 上位机真正要用的约 10~15 条，按需增量整理即可。

### 6.3 无实机验证

Action 的超时/断线/急停已有 host 确定性测试，但从未在真硬件上跑过。
`feat/hil-simulink-validation` 分支上有一套 host 编译的 SIL（`gcc` 直接编译
`drv_coax_ctrl.c` + Simulink codegen），**不需要真机**，但它只复现 `DRV_COAX_CTRL_Run()`，
**不含 RTOS 侧、执行器时序、命令层** —— 覆盖不到 Action。不建议合并那个分支
（提交含 3660 文件 / 71 万行，绝大部分是采集数据），要用就 cherry-pick 或只借模式。

---

## 7. 建议的推进顺序

1. **Phase 1 只读打通（零风险）** —— ssproj 换真数据源 + JustFloat 二进制 frameParser；
   控制脚本做 `连接 → TELEM? → ensureDashboard → Sensor_Data:1`；删掉 7 个假按钮。
   验收：连真机看到姿态曲线，且固件加通道时上位机自动跟随。
2. **Phase 2 交互控件（扩展包，零 C++）** —— 只放安全命令（`PARAM`/`PID`/`SAVE`/查询类），
   不碰执行器。验收：滑条改增益，飞控回显跟随。
3. **Phase 3 Action 接线** —— 先解 6.1 分帧；先做一个真正危险的任务
   （`SERVO_SWEEP` 或 `MOTOR_SPIN`）。
4. **Phase 4** —— `IMUCAL`/`ACCEPT`/`IDENT` 归一化；自检向导；日志回放。

---

## 8. 仓库与环境

- 上游：[Serial-Studio/Serial-Studio](https://github.com/Serial-Studio/Serial-Studio)（GPLv3 / 商业双授权）
- 本工程 fork：[ThroneTLE/Serial-Studio](https://github.com/ThroneTLE/Serial-Studio)
- 子模块路径：`tools/ground_station/Serial-Studio`
- 环境：Qt 6.7.2 MinGW 64-bit、CMake 3.20+、Ninja

```bash
git submodule update --init --recursive
```

| 脚本 | 说明 |
|:--|:--|
| `run_gcs.bat` | 启动地面站（可带 `.ssproj` 参数） |
| `run_sim.bat` | 启动仿真器 + 地面站。**注意**：`controlScriptCode` 里拉起的是 `python3`，Windows 上会返回 `exited with code 9009`（找不到命令），需改成 `python` 或绝对路径 |
