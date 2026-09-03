# 遥测流 v2 与上位机示波器规划（派工稿）

目标：在自己的上位机（`tools/drone_tcp_panel.py` 体系）里实现 Synex 那种"高帧率波形 + 滑块调参实时回显"，
同时解决 Synex/JustFloat 方案的三个硬伤：**参数不变也在回显、包长不能变、变了就错位**。
数传与 USB CDC 两条链路**第一期就都要通**，同一帧格式、同一解码器。

本文是设计裁决 + REQ 拆分。执行者按 §7 的 REQ 领取，类别模式见
[`.agents/skills/drone-h743-project/references/modes/protocol-telemetry.md`](../.agents/skills/drone-h743-project/references/modes/protocol-telemetry.md)。

---

## 1. 现状与约束（改之前必须知道的）

| 项 | 现状 | 出处 |
|---|---|---|
| 帧格式 | VOFA JustFloat：`float32[28]` + 尾 `00 00 80 7F`，定长 116 B | `App/Src/app_vofa.c` |
| 周期 | 25 ms（40 Hz），`VOFA_task` 在 `Core/Src/freertos.c` USER CODE 段里填充 | `Core/Src/freertos.c` |
| 出口 | `uartTxQueue`（深 32，函数号 `VOFA_SOCKET`）→ USART1 **57600 baud** → 数传直连（`APP_AIWB2_DIRECT_SERIAL_MODE=1`） | `App/Src/app_aiwb2.c:56`、`Core/Src/usart.c:229` |
| 带宽 | 116 B × 40 Hz = **4640 B/s，占 5760 B/s 的 80%**；其中 16 路增益回显 = 2560 B/s | 算出来的 |
| 通道表 | `APP_TelemChannelId` 枚举 + `app_telem_channels[]` 元数据 + `TELEM?`/`TELEM CH from=` 分页 + FNV-1a `SchemaHash` | `App/Src/app_telemetry.c` |
| 滑块写入 | Synex 发文本 `name:value`，`app_control_handle_pid_slider_line` 查表 → `DRV_COAX_CTRL_SetParam` → 回 `PARAM` 记录 + `PID` legacy 长文本 | `App/Src/app_control.c:3497` |
| 帧协议 | `$X` + dir + flags + fn(u16) + len(u16) + payload(≤256) + CRC8-DVB-S2；固件 builder 在 `app_proto.c` 里 `#if 0`；上位机 `transport.py::_consume_buffer` 已解析，但把 payload 一律当 UTF-8 文本 | `App/Inc/app_proto.h`、`tools/panel_lib/transport.py` |
| USB CDC | 目前只做命令/回复镜像；`APP_USB_CDC_Write` 单次 ≤1536 B、阻塞等待完成；IMUCAP/FLOG 导出期间独占 | `App/Src/app_usb_cdc.c`、`app_control_core.c` |
| 上位机绘图 | Tk + 可选 matplotlib（`panel_lib/plotting.py`）；`pyqtgraph 0.14 / PySide6 6.11 / numpy 1.26` 本机可用 | 实测 |

硬约束（AGENTS.md）：`drone_tcp_panel.py` 与 `app_control.c` **只减不增**；`freertos.c` 属 CubeMX 文件，
USER CODE 段只允许缩减；新代码进新模块、每文件 ≤800 行；每项行为改动带契约测试。

---

## 2. 设计裁决

### 2.1 一个机制解决三个问题：**自描述掩码帧**

每一帧自带"通道表指纹 + 本帧包含哪些通道的位掩码"。接收端不需要任何会话状态就能解出每个 float 是谁：

- **包长可变**：掩码变了，长度跟着变，接收端按 `popcount(mask)` 算长度并与 `$X` 的 len 交叉校验，不等就整帧丢弃。
- **不错位**：帧带 `schema_hash`（`APP_Telemetry_SchemaHash()`），固件通道表任何改动 → hash 变 → 上位机丢帧并自动重拉 `TELEM?` 表重建。
- **参数不冗余回显**：增益通道平时**不置位**；只有值变了（任何来源：滑块、`PARAM SET`、`LOAD`、`DEFAULTS`）才在下一帧置位带上；另有可配的低频全量刷新帧（默认 1 Hz，`0` 关闭）保证晚加入的上位机能收敛。

否决的替代方案：

- "协商 stream_id，帧只带 id"——省 8 B/帧，但状态在两端，重连/重启后极易错位，正是要消灭的问题。
- "继续 JustFloat，只把参数通道拿掉"——没解决包长可变，加通道仍会静默错位。
- "文本 CSV 行"——40 Hz × 12 通道文本约 8 kB/s，数传直接爆。

### 2.2 帧格式（`$X` 帧，function `APP_PROTO_MSG_TELEM_FRAME = 0x2230`，payload 小端）

```
off  0  u8   ver        = 1
off  1  u8   count      本帧样本数 ≥1（R-T1 固定 1；R-T2 USB 高速档批量）
off  2  u16  seq        帧序号，自然回绕；上位机据此统计丢帧
off  4  u32  schema     APP_Telemetry_SchemaHash()
off  8  u32  t_us       首个样本时间戳（SVC_Timestamp_Us 低 32 位，71 min 回绕，上位机解回绕）
off 12  u16  dt_us      批内样本间隔；count==1 时为 0
off 14  u16  flags      bit0 = 全量刷新帧；其余保留写 0
off 16  u64  mask       bit i = 通道 i 在本帧中出现（i 为 APP_TelemChannelId）
off 24  f32  data[count × popcount(mask)]   按通道索引升序、再按样本序排列
```

- 校验：`payload_len == 24 + 4 × count × popcount(mask)`，否则丢弃并计数；`$X` CRC8 保留不改。
- `_Static_assert(APP_TELEM_CH_COUNT <= 64)`；将来超过 64 路再升 `ver`。
- 数传档估算：12 路实时通道 → 9 + 24 + 48 = **81 B × 40 Hz = 3240 B/s**（较现在省 30%，且再加通道不会错位）。
- `$X` payload ≤256 → 单帧 `count × popcount ≤ 58`；固件对超限配置回 `ERR telem frame too large`，**不得静默截断**。

### 2.3 参数回显与滑块三态

- 通道表增加 `param` 字段（`TELEM CH` 行加 `param=coax.roll_rate_kd`，无参数的通道为 `-`），`APP_TELEM_SCHEMA_VERSION` 升到 2。
  上位机由此**数据驱动**生成滑块：`group=gain` 且 `param!=-` 的通道 → 滑块，量程取表里 `min/max`；其它 → 曲线候选。
- 脏位检测放在遥测模块自己里：每 tick 对每个带 `param` 的通道 `DRV_COAX_CTRL_GetParam` 与影子值按位比较，变了置位。
  不往 `DRV_COAX_CTRL_SetParam` 或任何控制路径挂钩子——零耦合，且现在本来就每 25 ms 查一遍，无新增开销。
- 符号约定不变：遥测里给操作者看的是 UI 值（yaw/roll/pitch 的 `angle_kp`、`yaw_rate_kd` 取反），写入走既有 `app_control_param_from_ui_value`。
- 上位机滑块三态：**pending**（已发、等回显）→ **confirmed**（回显值与发送值在 1e-4 相对容差内）/ **diverged**（固件钳位或拒绝，滑块跳到固件实际值并标黄）。
  拖动节流：拖动中 ≤5 Hz、松手必发；写入仍走现有 `PROTO_REQ_PARAM_SET`，不新增写入命令。
- 固件对每次 `PARAM SET` 仍回 `PARAM` 记录 + `PID` legacy 文本（现有行为，本期不改）；节流后带宽可接受。裁掉 legacy 回显另立 REQ。

### 2.4 命令面（文本，沿用 `TELEM` 族）

| 命令 | 语义 |
|---|---|
| `TELEM?` | 现有表头 + 新增流状态：`stream=<0/1> rate=<hz> mask=<hex16> refresh=<s> fmt=<bin/jf> sink=<usb/uart/auto> active=<usb/uart/-> seq=<n> drop=<n>` |
| `TELEM CH from=<n>` | 现有分页，行内新增 `param=` |
| `TELEM STREAM on` / `off` | 开/关；`Sensor_Data:1/0` 保留为别名（Synex 兼容） |
| `TELEM RATE <hz>` | 1..40（R-T1）；USB 出口在 R-T2 放宽到 1000 |
| `TELEM MASK <hex16>` | 立即生效；帧自描述，切换瞬间不会错位 |
| `TELEM REFRESH <s>` | 全量刷新周期，0 关闭，默认 1 |
| `TELEM FORMAT bin` / `jf` | `jf` = 旧 JustFloat 定长帧（Synex 过渡用），默认 `bin` |
| `TELEM SINK usb` / `uart` / `auto` | 出口选择，默认 `auto`（跟随 `STREAM on` 的来源链路），详见 §3 |

流配置**只存 RAM、不持久化**，上电默认 `stream=0`。上位机示波器页选中时发 `STREAM on`，切走发 `off`（沿用 flow_monitor 的可见性门控范式），不占数传带宽。

### 2.5 固件模块划分

- 新建 `App/Src/app_telem_stream.c` / `App/Inc/app_telem_stream.h`：帧编码、掩码/脏位/刷新计时、出口选择、`APP_TelemStream_Tick()`。
  `VOFA_task` 的填充体整体搬进去，`freertos.c` 里只剩 `APP_TelemStream_Tick()` 一行调用（USER CODE 段净减少）。
- `app_proto.c`：只把 `APP_Proto_BuildFrame` 及其 CRC 从 `#if 0` 里放出来（解析器继续禁用）。
- `app_vofa.c` 退化为 `jf` 格式的发送后端；`bin` 格式 UART 出口经同一 `uartTxQueue`/`VOFA_SOCKET` 原样字节路径出去（帧已含 `$X` 头），USB 出口直接 `APP_USB_CDC_Write`。
- 命令注册：`TELEM` 子命令放到新文件 `App/Src/app_cmd_telem.c`（仿 `app_cmd_flow.c`），`app_control.c` 只允许把现有 `TELEM` 分发行改指向新文件（净不增）。
- `app_telemetry.c` 加 `param` 字段与 hash 覆盖、版本升 2。

### 2.6 上位机模块划分

- `tools/panel_lib/transport.py::_consume_buffer`：function 在 `PROTO_BINARY_FUNCTIONS` 集合内时投递 `("proto_bin", fn, bytes)`，不再按 UTF-8 解码。这是对该文件唯一改动；serial / tcp / udp 三个 Transport 共用这一处，二进制分支一次全部生效。
- 新建 `tools/panel_lib/telem_stream.py`：
  - `TelemSchema`：由 `TELEM?`/`TELEM CH` 分页装配，含 hash 复算（与固件 FNV 逐字节一致，已有 pytest 先例）；
  - `TelemDecoder.feed(payload) -> list[Sample] | None`：长度/版本/hash 校验、seq 丢帧统计、`t_us` 回绕；
  - `TelemRing`：每通道 numpy 环形缓冲（默认 60 s × 最高速率），**在收线程里写入**，Tk 线程只读快照——绘图不受数据速率牵连。
- 新建 `tools/panel_lib/scope.py`：纯 Tk Canvas 示波器控件（无 matplotlib 依赖）。
  每 33 ms 重绘一次；每条曲线按像素列做 min/max 抽稀（numpy reshape），再 `canvas.coords(line_id, ...)` 一次更新；
  支持时间窗口 1/5/10/30/60 s、自动/手动量程、暂停、光标读数。目标：8 曲线 × 10 k 点每帧 ≤5 ms。
- 新建 `tools/panel_lib/pages/scope.py`（Mixin）：通道勾选树（按 `group`）→ 发 `TELEM MASK`；滑块区（三态）；
  统计条（当前出口 / 实测帧率 / seq gap / drop / schema）；CSV 录制到 `data/telemetry/YYYY-MM-DD/telem_*.csv`（含表头行 = 通道名与 hash）。
- `drone_tcp_panel.py`：仅允许 Mixin 挂载 + 页签注册（≤5 行），且必须同时把 `_imu_poll_tick` 里可迁移的可见性判断挪到新页里抵消。

### 2.7 为什么不用 pyqtgraph（以及什么时候用）

pyqtgraph 性能更高，但 Qt 事件循环进不了 Tk 进程；只能开独立进程，链路（尤其是独占的串口）又在面板手里，
就得再加本地 UDP 镜像，滑块和波形分在两个窗口，回到了"外部上位机"的体验。Tk Canvas + numpy 抽稀在 8 路 × 40 Hz～1 kHz
的量级下完全够用，且能用现有 `DronePanel()` 真实构造做端到端测试。留作 R-T3 可选项：面板一行 `sendto` 把二进制帧镜像到
`127.0.0.1:6670`，`tools/telem_scope_qt.py` 用 pyqtgraph 只读显示。

---

## 3. 数传与 USB：同一帧、两个出口，第一期就都要通

同一份 `$X` 二进制帧可以从两个出口发出，上位机三种通道（serial = USB CDC / tcp / udp）解码路径完全相同：

| 出口 | 物理路径 | 速率上限 | count | 典型用途 |
|---|---|---|---|---|
| `uart` | `uartTxQueue` → USART1 → 数传（或 Ai-WB2 透传） | ≤40 Hz | 1 | 飞行中调参、常规监视 |
| `usb` | `APP_USB_CDC_Write` 直接写 `$X` 帧 | R-T1：≤40 Hz；R-T2：≤1000 Hz | R-T1：1；R-T2：批量 | 台架、免数传时的日常调试 |

- `TELEM SINK` 取值 `usb` / `uart` / `auto`，默认 `auto` = **命令从哪条链路进来就往哪条发**（以 `STREAM on` 的来源为准），换链路不用改配置。
  `auto` 的来源判定复用现有"命令来自 USB / UART"的上下文（`app_control_core.c` 已区分 USB 镜像与 `uartTxQueue`）。
- USB 出口规则：IMUCAP/FLOG 导出激活时自动挂起（沿用现有互斥，`VOFA_task` 已有这个判断）；USB 拔出（`APP_USB_CDC_IsReady()==0`）自动 `stream=0`；
  `APP_USB_CDC_Write` 是阻塞等待完成的，只能在遥测任务上下文调用，不得进控制环。
- UART 出口帧长受 `APP_UART_TX_TEXT_SIZE=256` 限制；USB 出口在 R-T2 单独放宽 `APP_TELEM_FRAME_MAX_PAYLOAD 1024`（`$X` len 本就是 u16，线上兼容）。
  任一出口超限配置回 `ERR telem frame too large`，不静默截断。
- 示波器页显示当前出口与实测帧率，方便肉眼确认走的是哪条链路。
- 文本命令/回复的既有双链路镜像行为不变。

---

## 4. 测试与验收

固件（host gcc 装置，仿 `tests/test_flow_nav_service_contract.py` 的编译执行方式）：
- 编码器黄金向量：给定 mask/count/values → 逐字节期望；`popcount` 与 len 一致；`≤64` 静态断言存在。
- 脏位：改一个参数 → 仅该位置位一帧；`REFRESH` 到期 → 全位置位且 `flags.bit0=1`；`REFRESH 0` 永不全量。
- 出口：`SINK auto` 跟随命令来源；`SINK usb` 在 `IsReady()==0` 时自动 `stream=0`；IMUCAP/FLOG 激活时不发帧。
- 超限：`RATE`/`MASK` 组合超过出口上限 → `ERR`，不发帧。
- 契约：`freertos.c` 的 `VOFA_task` 不再含 `vofa_data[APP_TELEM_CH_` 填充；`app_control.c` 行数不增。

上位机（pytest）：
- 解码器：黄金向量对称；**模糊测试**——随机截断/插入/翻转字节的流，断言永不产出错长度样本、一帧内重同步、hash 不符不入环。
- schema：hash 与固件复算一致；表变更（改一个 `max`）→ 重建流程被触发。
- 页面（真实 `DronePanel()` + FakeTransport）：选中页发 `TELEM STREAM on`、切走发 `off`；勾选通道发 `MASK`；
  滑块拖动节流 ≤5 Hz、松手必发；回显一致 → confirmed、不一致 → diverged 并跳到固件值；重置/录制不发协议帧以外的东西。
- 性能基准：`scope.py` 8 × 10 k 点单次重绘 ≤5 ms（无显示环境自动 skip）。

实机（审核者，COM31）：
- 数传 40 Hz、12 路、60 s：`seq` 无缺口，链路统计正常，命令回复不被挤掉；同一配置改从 USB CDC 连接再跑一遍，`sink=auto` 自动切到 USB。
- 滑块→回显往返延迟 < 150 ms；`LOAD/DEFAULTS` 后所有滑块自动收敛到固件值。
- 固件改一处通道表后重连：面板自动重建、曲线名正确，无错位。
- USB 高速档（R-T2）500 Hz × 8 路 60 s：无缺口；IMUCAP DUMP 期间流自动挂起、结束后恢复。

---

## 5. 分期与依赖

```
R-T1-1 固件遥测流模块（含双出口）─┬─→ R-T1-2 上位机解码器 ──→ R-T1-3 示波器页 ──→ R-T1-4〔机〕双链路验收
                                  └─→（R-T1-2 可与 R-T1-1 并行：先按本文帧格式写黄金向量）
R-T2-1 USB 高速批量档（依赖 R-T1-1；USB 40 Hz 档已在 R-T1 里通）──→ R-T2-2〔机〕
R-T3   pyqtgraph 外部示波器（可选，依赖 R-T1-2）
```

R-T1-1 与 R-T1-2 可以派给两个会话并行：帧格式以本文 §2.2 为准，任何一方要改格式先改本文。

---

## 6. 明确不做

- 不持久化流配置；不改 `PARAM SET` 的现有文本回复；不动 `drv_coax_ctrl.c` 与任何控制律/门限。
- 不改 `$X` 帧头与 CRC8；不重新启用固件侧 `$X` 解析器（PC→FC 仍是文本行）。
- 不删 JustFloat（`fmt=jf` 保留一个版本周期供 Synex 过渡；删除另立 REQ）。

---

## 7. 执行需求清单（已登记进 PIPELINE.md 副线 S8）

| ID | 节点 | 内容 | 验收判据 | 状态 |
|---|---|---|---|---|
| R-T1-1 | S8 | 〔码〕固件遥测流 v2：新建 `app_telem_stream.c/h` + `app_cmd_telem.c`，实现 §2.2 掩码帧、§2.3 脏位回显、§2.4 命令族；`VOFA_task` 填充体搬出 `freertos.c`；`app_telemetry` 加 `param` 字段、schema v2；`APP_Proto_BuildFrame` 解禁；`app_vofa.c` 退为 `jf` 后端；`TELEM SINK`（usb、uart、auto）两个出口同帧，USB 拔出自动关流、与 IMUCAP/FLOG 互斥。类别模式：protocol-telemetry | host 装置覆盖黄金向量/脏位/刷新/超限 ERR/双出口选择/拔出关流；`freertos.c` USER CODE 净减少；`app_control.c` 不增；`cmake --build --preset Debug` 零警告；`pytest tests -q` 全绿 | 待做 |
| R-T1-2 | S8 | 〔码〕上位机解码：`transport.py` 二进制分支（唯一改动）；新建 `panel_lib/telem_stream.py`（schema 装配+hash 复算、解码器、seq/drop 统计、numpy 环形缓冲、收线程写入）。类别模式：protocol-telemetry | 黄金向量与固件逐字节一致；模糊测试永不错位；hash 不符触发重拉；不改 `drone_tcp_panel.py` | 待做 |
| R-T1-3 | S8 | 〔码〕示波器页：新建 `panel_lib/scope.py`（Tk Canvas，min/max 抽稀）与 `pages/scope.py`（通道勾选→MASK、滑块三态、统计条、CSV 录制、可见性门控 STREAM on/off）。依赖 R-T1-2。类别模式：protocol-telemetry | 真实 `DronePanel()` 端到端断言 §4 全部条目；重绘基准 ≤5 ms；`drone_tcp_panel.py` 改动 ≤5 行且净不增 | 待做 |
| R-T1-4 | S8 | 〔机〕双链路实机验收：数传 40 Hz 与 USB 40 Hz 各跑一遍 | §4 实机前三条，两条链路各一次 | 待做 |
| R-T2-1 | S8 | 〔码〕USB 高速批量档：count>1、dt_us、USB 出口 `RATE ≤1000`、`APP_TELEM_FRAME_MAX_PAYLOAD 1024`。依赖 R-T1-1。类别模式：protocol-telemetry | 装置覆盖批量编码与 USB/UART 上限差异；UART 出口超限仍 ERR；构建零警告 | 待做 |
| R-T2-2 | S8 | 〔机〕USB 档 500 Hz 验收 | §4 实机第四条 | 待做 |
| R-T3 | S8 | 〔码〕可选：本地 UDP 镜像 + `tools/telem_scope_qt.py`（pyqtgraph 只读） | 不影响 R-T1 任何测试 | ⏸ |
