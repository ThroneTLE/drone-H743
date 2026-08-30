# drone-H743 技术规范与代码落地方案

> 版本 v1 · 2026-08-30 · 与 `PIPELINE.md` 配套：**清单在 PIPELINE（派什么）、规范在本文（怎么做）**。
> 执行者（任何 AI 或人）开工前必须完整读完 PIPELINE.md 与本文；审核由验收会话（Claude，持有 COM31/ST-Link 实机）执行。

## 1. 最终目的与范围

同轴双桨 STM32H743 无人机完成证据驱动的全链路 bring-up：传感健康 → 坐标极性 → IMU 标定 → 舵机机械 → 光流测距 → 无桨端到端 → 带桨首飞（M1~M7），并同步完成两大代码工程：运行时 FLU 坐标迁移（§10，M7 硬前置）与巨文件拆分（§11，副线 S6）。一切进度以 PIPELINE.md 状态图为唯一事实，实现完成 ≠ 测试通过 ≠ 实机验收。

## 2. 角色与工作流

- **作者**：定方向、做需要人手/签字的实机确认（拆桨、方向目视、机体倾斜手势）。
- **执行者**：认领 PIPELINE「执行需求清单」中一个 REQ，按本文规范实现，交 diff+测试输出+证据路径，把 REQ 状态改为**待审核**（执行者无权置 ✅）。
- **审核者**：复核代码与证据、跑全量测试与实机验证、置 ✅/打回、回写 PIPELINE 证据表。

**派单提示词模板**（复制给执行 AI）：

```text
你是 drone-H743 项目的执行工程师。开工前完整阅读仓库根目录 PIPELINE.md 与
doc/technical-spec.md 并遵守全部规范。本次任务：完成 REQ-____。
硬约束：①只做该 REQ 范围内的事；②新代码放新模块，禁止向 tools/drone_tcp_panel.py
与 App/Src/app_control.c 追加内容；③每项行为改动必须带契约测试，并通过
python -m pytest tests -q 全量（当前基线 696+ 项）与 cmake --build --preset Debug；
④改任何非忽略文件后运行仓库索引再生成脚本；⑤完成后把 PIPELINE.md 中该 REQ 状态
改为"待审核"并在证据表追加一行，不许自标完成；⑥不得触碰冻结节点、历史证据文件
（data/calibration/**/2026-08-2*）与 CubeMX 生成代码。
交付物：变更说明、测试输出原文、证据文件路径。未过审不算完成。
```

## 3. 架构与代码规范

- 分层（不可逆）：`App/*` 任务与行为 → `Services/*` 无任务域服务 → `Driver/*` 芯片/协议逻辑 → `BSP/*` 板级绑定 → `Core/*` CubeMX/HAL。芯片协议不进 BSP；CubeMX 拥有 `Core/Src/main.c、freertos.c、外设 init、USB_DEVICE/*`——改引脚/时钟/DMA/NVIC 必须走 CubeMX 重生成，禁止手改。
- 文件规模：C 手写 ≤~1500 行、Python ≤~2000 行；超限 = 抽新模块。`drone_tcp_panel.py`（≈11.4k）与 `app_control.c`（≈6k）已冻结增长，只减不增（§11）。
- 命名：Driver 按器件/器件类命名；App 服务 `app_<domain>.c` + 对应 `App/Inc` 头；测试 `tests/test_<topic>.py`。
- 注释密度与语言随文件现状（本仓库多中文注释）；解释"为什么"，不复述代码。

## 4. 坐标与单位契约

- 唯一规范：`Driver/Inc/drv_frame_contract.h` —— 右手 FLU（+X前/+Y左/+Z上；+roll 右翼下沉、+pitch 机头下俯、+yaw 机头左转）。任何轴/符号工作先读 `.agents/.../references/flu-coordinate-contract.md`，改后必跑 `tests/test_flu_frame_contract.py`。
- 现状：`DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK = 0x00`（6 个 seam 全 legacy）。**在掩码补全前禁止宣称运行时 FLU 达标或放飞**；对外导出若自称 FLU（如光流补偿快照），必须是显式适配转换并有配套测试。
- 单位：遥测 `mg / mdps / cdeg`（IMU? 行内声明 units 字段）；内部控制律 SI（rad、m/s）。新增遥测必须自带 units 与 frame 标注。

## 5. 通信协议规范

- 通道：USB CDC 与 UART 均为**裸 ASCII 文本行** → `APP_Control_ProcessLine`；结构化回复走 `app_control_queue_proto_text(function_id, ...)` 帧镜像。
- ID 注册：所有 REQ/MSG function ID 必须在 `App/Inc/app_proto.h` 登记，主机侧常量与之同提交（历史教训：0x1022/0x1023 曾主机单方面定义）。
- 上下文契约：`APP_Control_QueueText` / `queue_proto_text` **只允许通信任务上下文**调用（会同步阻塞 USB，最坏 3×10ms）。控制环（500Hz/1kHz）模块一律置事件标志由 `app_control_tick_common` 补发（先例：`APP_ServoCal_TakeNotice`；契约测试 `tests/test_control_loop_blocking_contract.py`）。
- 命令族现状：`IMU? RTOS? FLASH? IMUFRAME IMUCAL SERVOCAL RCMAP RC? FLOW? RANGE? ACCEPT BOOT` 等；新增命令需同时更新 CAPS LIST 回复与面板 `VALIDATION_ALLOWED_COMMANDS`（若面板使用）。
- 周期报文：UART/USBCDC 统计每 2s（透传条件下）；解析器必须容忍任意穿插行。

## 6. 持久化规范

- 机制：`SVC_Param` 双槽后台保存；外部 FLASH GD25Q32（JEDEC C8 40 16）。
- FCAL 记录：**恒 160 字节**（`_Static_assert` 锁定），演进只能吃保留区（先例：舵机机械字段占用 `v2_reserved[7]→[3]`）；`valid_mask` 新位需同步区分「持久化支持掩码」与「V1 上传通道掩码」（防越权，先例 `APP_FLIGHT_CAL_V1_VALID_MASK_SUPPORTED=0x0F`）。
- CFG 记录：当前 V17（含 RC 映射）。版本迁移规则：新版本结构读头部判版本 → 旧版本用旧结构体整读并独立校验 checksum → 缺失域装出厂默认；旧版本必须永远可读回（测试锁定）。
- 判据单源：合法性判据归 Driver 层；App 因 host 单测无法链接整驱动而保留副本时，必须有 lockstep 契约测试强制逐条一致（先例：`test_coax_ctrl_contract.py::test_servo_calibration_criteria_stay_in_lockstep_across_layers`）。

## 7. 安全规范

- 解锁四门（不可削弱）：IMU frame arm-lock、IMU 健康、RC 链路、低油门上升沿；标定候选（IMUCAL/SERVOCAL）RAM 预览期一律 arm-lock。
- DFU：主机只做 advisory；拒绝判定归固件 BOOT 处理器（`app_boot.h` 四类原因），`test_firmware_build_gate.py` 锁定其存在。真驱动舵机的路径（V1 采集、V2A、SERVO MOVE/JOG）保留主机硬门（新鲜快照 + armed=0 + 拆桨勾选）。
- 地面点动：机械校准页一律用保持型 `SERVO JOG ch us`（`app_servo_jog.c`）——在稳定环 commit 仲裁点接管输出，优先级 手势标定>验收>反馈台架>解锁>点动，500µs/s 斜坡、120s 超时或 `SERVO JOG STOP` 交还；一次性 `SERVO MOVE` 会被稳定环 500ms 强制刷新覆盖，禁止再用于机械校准（`test_servo_jog_contract.py`）。
- 证据不可变：`data/calibration/**` 历史文件只读；面板仅浏览态禁止 autosave（`test_evidence_write_protection.py`）。执行者不得修改/删除任何既有证据文件。
- 默认拆桨；带桨属 M7，冻结中。

## 8. 数据与证据规范

- 目录唯一来源 `tools/project_paths.py`（新目录先登记常量），布局见 `data/README.md`；运行产物 `category/YYYY-MM-DD/`。
- 证据 JSON 通用头：`format`（`drone-h743-<topic>`）、`schema`、`created_at`（ISO 带时区）、`verdict`（PASS/WARN/FAIL/INCOMPLETE）、`findings[]`；带机上下文的须含 `firmware_crc32`/`cal_generation`/`orientation`。
- 每次验证（成败都算）在 PIPELINE 证据表追加一行；状态翻转另守验收门。

## 9. 测试规范

- 模式：源码契约测试（正则/结构断言，防机制回退）+ host 编译执行装置（gcc 编译目标 C + 桩，跑真逻辑）+ Tk 端到端抽样。每个 REQ 至少一条新契约测试；修 bug 先写复现测试。
- 全量 `python -m pytest tests -q` 必绿；固件 `cmake --build --preset Debug` 零警告；改文件后再生成仓库索引（索引新鲜度本身是测试）。
- 实机证据只能由持硬件方（审核者/作者）出具；执行者产出的一律标「待实机」。

## 10. 运行时 FLU 迁移方案（M7 硬前置，可派单）

目标：把 `DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK` 从 0x00 推到 0x3F。**顺序固定、一 seam 一 REQ 一提交**；每 seam 流程 = ①现状口径盘点写入测试（先红）→ ②实施基变换（完整四元数/矩阵，不许只翻显示欧拉角）→ ③host 契约 + 实机 A/B（disarmed 对比迁移前后姿态/导航输出一致性）→ ④翻掩码位（与测试同提交）→ ⑤回滚 = revert 单提交。

| 位 | seam | 现状（legacy） | 主要文件 | 关键风险 |
|---|---|---|---|---|
| 0 | SENSOR | 芯片轴→中间轴排列 | `App/Src/app_sensor.c` | 与 V0 orientation 映射叠加次序 |
| 1 | ESTIMATOR | x-io Fusion NED 约定 | `Driver/Src/drv_attitude_fusion.c`、`app_stabilizer.c` 融合输入符号 | 四元数方向定义（body↔nav）必须显式写明并测全基变换 |
| 2 | NAVIGATION | 本地水平系 Z-down | `Driver/Src/drv_imu_nav.c` | 重力符号、速度积分 |
| 3 | CONTROLLER | 控制器 Y-右等混合口径 | `app_stabilizer.c` 控制装配、`drv_coax_ctrl.c` 力/角速率系 | 与舵机机械 pulse_sign 分层：机械极性不许进控制符号 |
| 4 | RC/ACTUATOR | RC 意图与分配残留旧符号 | `app_stabilizer.c`、`app_rc_config.c` | 已部分现代化，需盘点残留 |
| 5 | TELEMETRY/LOG | 面板地平仪、`flight_log_rerun_replay.py` X-fwd/Y-right/Z-down | `drone_tcp_panel.py`、日志工具 | 历史数据永不重释义，新数据带 frame 标注 |

迁移期间历史 NED/FRD 数据禁止按 FLU 重读（契约 §4）。每 seam 完成后审核者做一次实机静置+手势符号抽查。

## 11. S6 巨文件拆分方案（可派单，绞杀者模式）

**panel（Python）**：建 `tools/panel_lib/` 包：`transport.py`（TCP/UDP/串口+指纹重连）、`proto.py`（ID 表+帧编解码）、`state.py`（panel_state 持久化）、`pages/<domain>.py`（每页 builder+handlers）、`evidence.py`（会话/证据读写含只读保护）。一次只迁一页/一域：搬运→`drone_tcp_panel.py` 内改为 import 转发→现有源码契约测试同步改指向→全量绿。禁止行为变更混入搬运提交。

**app_control.c（C）**：按命令域拆 `App/Src/app_cmd_imucal.c / app_cmd_servocal.c / app_cmd_rcmap.c / app_cmd_flow.c / app_cmd_system.c` + `app_control_core.c`（QueueText/tick/持久化）。分发表留在 app_control.c；新文件各 ≤800 行；每拆一域：CMake 登记→host 装置照编→固件构建零警告。QueueText 上下文契约与 §5 一致。

## 12. 工具链与环境

- 构建 `cmake --build --preset Debug`；烧录（ST-Link）：`"D:/Program Files/OpenOCD-20240916-0.12.0/bin/openocd.exe" -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/Debug/drone-H743.elf verify reset exit"`；复位同上 `-c "init; reset run; shutdown"`。
- 飞控 CDC：STM VCP（VID 0x0483/PID 0x5740，序列号 335335763233，当前 COM31，会漂移——按指纹找）。
- 实机基线：`python tools/m1_baseline_check.py --port COM31 --seconds 60`（只读）。工作台：`python tools/drone_tcp_panel.py`。

## 13. 审核清单（审核者逐项执行）

1. REQ 范围核对：diff 只含该 REQ；无巨文件追加、无证据文件改动、无 CubeMX 手改。
2. 测试：新契约测试存在且先红后绿可论证；全量 pytest + 固件构建亲自复跑。
3. 语义：安全门/判据/坐标符号未被顺手放宽；放宽类改动必须独立提交并标「作者复核点」。
4. 持久化：ABI/版本迁移规则（§6）符合；lockstep 副本同步。
5. 证据：路径、schema、verdict 真实可复现；PIPELINE 状态与证据表一致（`tests/test_pipeline_contract.py` 绿）。
6. 索引新鲜、提交信息如实（不许"校准完成/可以飞"类越级措辞）。
