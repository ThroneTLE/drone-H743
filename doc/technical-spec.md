# drone-H743 技术规范与代码落地方案

> 稳定工程契约 · 流程修订于 2026-09-21 · 与 `PIPELINE.md` 配套：**清单在 PIPELINE（派什么）、规范在本文（怎么做）**。
> 从项目 SKILL 按角色和任务读取本文相关章节；主控读取当前进度，执行者接收必要任务上下文。不要加载完整历史证据表。

## 1. 最终目的与范围

同轴双桨 STM32H743 无人机完成证据驱动的全链路 bring-up：传感健康 → 坐标极性 → IMU 标定 → 舵机机械 → 光流测距 → 无桨端到端 → 带桨首飞（M1~M7），并同步完成两大代码工程：运行时 FLU 坐标迁移（§10，M7 硬前置）与巨文件拆分（§11，副线 S6）。一切进度以 PIPELINE.md 状态图为唯一事实，实现完成 ≠ 测试通过 ≠ 实机验收。

## 2. 角色与工作流

- **作者**决定功能和有明显取舍的路线，确认安全/实机事项；主控用通俗中文解释效果、成本和限制。
- **主控**维护已采纳设计，组织任务、审查合并、验证及统一进度/索引收尾；普通实现可自己完成。
- **执行者**在独立工作树内只修改分配文件，完成定向验证，交版本、差异、测试原文和缺口，不自行更新总表。
- **审核职责由主模型承担**：子模型完成后交待审核，主模型检查代码、设计和验证证据，全部判据满足后置 ✅，否则返修或保留缺口；可由其他模型协助复核，但最终结论由主模型负责。无需另行指定验收会话。主模型直接完成的小任务也须自查并记录依据。含实机要求的任务仍须取得对应实机证据，审核职责不授予额外设备操作权限。

角色不绑定模型品牌。协作及任务卡只维护在 [主控规则](../.agents/skills/drone-h743-project/references/dispatcher-prompt.md)，测试流程只维护在 [验证策略](../.agents/skills/drone-h743-project/references/validation-policy.md)。不在派单模板复制另一套全量命令。
已采纳设计与理由见 [当前架构](current-architecture.md)；后续任务必须继承，公共职责/接口/数据语义变更由主控协调，改变作者已确认设计或硬边界仍需作者决定。

## 3. 架构与代码规范

- 分层（不可逆）：`App/*` 任务与行为 → `Services/*` 无任务域服务 → `Driver/*` 芯片/协议逻辑 → `BSP/*` 板级绑定 → `Core/*` CubeMX/HAL。芯片协议不进 BSP；CubeMX 拥有 `Core/Src/main.c、freertos.c、外设 init、USB_DEVICE/*`——改引脚/时钟/DMA/NVIC 必须走 CubeMX 重生成，禁止手改。
- 文件规模：C 手写 ≤~1500 行、Python ≤~2000 行；超限即抽新模块。`drone_tcp_panel.py` 与 `app_control.c` 已冻结增长，只减不增；当前行数从索引或工作树读取，不写入长期文档。
- 命名：Driver 按器件/器件类命名；App 服务 `app_<domain>.c` + 对应 `App/Inc` 头；测试 `tests/test_<topic>.py`。
- 注释密度与语言随文件现状（本仓库多中文注释）；解释"为什么"，不复述代码。

## 4. 坐标与单位契约

- 唯一规范：`Driver/Inc/drv_frame_contract.h` —— 右手 FLU（+X前/+Y左/+Z上；+roll 右翼下沉、+pitch 机头下俯、+yaw 机头左转）。任何轴/符号工作先读 [`flu-coordinate-contract.md`](../.agents/skills/drone-h743-project/references/flu-coordinate-contract.md)，改后必跑 `tests/test_flu_frame_contract.py`。
- 当前迁移掩码只从 `drv_frame_contract.h` 读取，不在规范中复制易过期数值。掩码置位不等于物理方向验收或放飞；对外导出若自称 FLU，必须有正确适配与配套测试，阶段证据见 PIPELINE。
- 单位：遥测 `mg / mdps / cdeg`（IMU? 行内声明 units 字段）；内部控制律 SI（rad、m/s）。新增遥测必须自带 units 与 frame 标注。

## 5. 通信协议规范

- 命令入口：USB CDC 与 UART 接收的命令是 ASCII 文本行 → `APP_Control_ProcessLine`；同一链路还可承载 `$X` 二进制遥测帧。结构化文本回复走 `app_control_queue_proto_text(function_id, ...)` 帧镜像，线上格式见 `doc/telemetry-protocol.md`。
- ID 注册：所有 REQ/MSG function ID 必须在 `App/Inc/app_proto.h` 登记，主机侧常量与之同提交（历史教训：0x1022/0x1023 曾主机单方面定义）。
- 上下文契约：`APP_Control_QueueText` / `queue_proto_text` **只允许通信任务上下文**调用（会同步阻塞 USB，最坏 3×10ms）。控制环（500Hz/1kHz）模块一律置事件标志由 `app_control_tick_common` 补发（先例：`APP_ServoCal_TakeNotice`；契约测试 `tests/test_control_loop_blocking_contract.py`）。
- 命令族现状：`IMU? RTOS? FLASH? IMUFRAME IMUCAL SERVOCAL RCMAP RC? FLOW? RANGE? ACCEPT BOOT` 等；新增命令需同时更新 CAPS LIST 回复与面板 `VALIDATION_ALLOWED_COMMANDS`（若面板使用）。
- 周期报文：UART/USBCDC 统计每 2s（透传条件下）；解析器必须容忍任意穿插行。

## 6. 持久化规范

- 机制：`SVC_Param` 双槽后台保存；外部 FLASH GD25Q32（JEDEC C8 40 16）。
- FCAL 记录：**恒 160 字节**（`_Static_assert` 锁定），演进只能吃保留区（先例：舵机机械字段占用 `v2_reserved[7]→[3]`）；`valid_mask` 新位需同步区分「持久化支持掩码」与「V1 上传通道掩码」（防越权，先例 `APP_FLIGHT_CAL_V1_VALID_MASK_SUPPORTED=0x0F`）。
- CFG 记录：当前 V19（四环级联物理参数；含 RC 映射），继续读取 V18/V17/V16/V15。版本迁移规则：新版本结构读头部判版本 → 旧版本用旧结构体整读并独立校验 checksum → 按原物理语义换算或缺失域装安全默认；旧版本必须永远可读回（测试锁定）。
- 判据单源：合法性判据归 Driver 层；App 因 host 单测无法链接整驱动而保留副本时，必须有 lockstep 契约测试强制逐条一致（先例：`test_coax_ctrl_contract.py::test_servo_calibration_criteria_stay_in_lockstep_across_layers`）。

## 7. 安全规范

- 解锁四门（不可削弱）：IMU frame arm-lock、IMU 健康、RC 链路、低油门上升沿；标定候选（IMUCAL/SERVOCAL）RAM 预览期一律 arm-lock。
- DFU：主机只做 advisory；拒绝判定归固件 BOOT 处理器（`app_boot.h` 四类原因），`test_firmware_build_gate.py` 锁定其存在。真驱动舵机的路径（V1 采集、V2A、SERVO MOVE/JOG）保留主机硬门（新鲜快照 + armed=0 + 拆桨勾选）。
- 地面点动：机械校准页一律用保持型 `SERVO JOG ch us`（`app_servo_jog.c`）——在稳定环 commit 仲裁点接管输出，优先级 手势标定>验收>反馈台架>解锁>点动，500µs/s 斜坡、120s 超时或 `SERVO JOG STOP` 交还；一次性 `SERVO MOVE` 会被稳定环 500ms 强制刷新覆盖，禁止再用于机械校准（`test_servo_jog_contract.py`）。
- 证据不可变：`data/calibration/** 全部历史证据`只读；面板仅浏览态禁止 autosave（`test_evidence_write_protection.py`）。执行者不得修改/删除任何既有证据文件。
- 默认拆桨；带桨属 M7，冻结中。

## 8. 数据与证据规范

- 目录唯一来源 `tools/project_paths.py`（新目录先登记常量），布局见 `data/README.md`；运行产物 `category/YYYY-MM-DD/`。
- 证据 JSON 通用头：`format`（`drone-h743-<topic>`）、`schema`、`created_at`（ISO 带时区）、`verdict`（PASS/WARN/FAIL/INCOMPLETE）、`findings[]`；带机上下文的须含 `firmware_crc32`/`cal_generation`/`orientation`。
- 每次验证保留原始输出（包括失败）；主控整批收尾在 PIPELINE 汇总可追溯证据，独立 bug 另列一项。子 Agent 不并发改总表；状态翻转另守验收门。

## 9. 测试规范

- 每项行为变化须有有效覆盖：按需要采用真实 C 编译执行、真实 Tk 交互、接口/兼容性测试及必要结构约束；已有测试足够时复用，不按任务数量机械新增字符串断言。修 bug 先固定复现。
- **小改动定向验证，较大改动和阶段交审才全量**；唯一执行表见 [验证策略](../.agents/skills/drone-h743-project/references/validation-policy.md)。子任务定向，主控对最终组合验证；需要的固件构建须零警告。索引由主控在整批收尾统一生成与检查。
- 实机证据由获授权的持硬件方出具，主模型核对来源和实际判据；只有软件证据时明确列出待实机项，不得把整个实机任务置 ✅。

## 10. 运行时 FLU 与历史数据

`DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK` 的当前值只从 `drv_frame_contract.h` 读取。任何 seam 迁移必须独立立项，以真实数据和拆桨物理方向验证；不得只翻显示符号或通过负增益掩盖问题。历史 NED/FRD 数据永不因当前源码变化而重解释。

## 11. 巨文件永久约束

S6 已完成并从现行实施计划中移除。永久规则仍有效：`drone_tcp_panel.py` 与 `app_control.c` 只减不增；新功能进入新模块。纯提取必须搬移而非复制，安全常量、调用时序和行为保持不变，跨模块私有状态只经窄接口暴露。

## 12. 工具链与环境

- 构建 `cmake --build --preset Debug`；烧录（ST-Link）：`"D:/Program Files/OpenOCD-20240916-0.12.0/bin/openocd.exe" -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/Debug/drone-H743.elf verify reset exit"`；复位同上 `-c "init; reset run; shutdown"`。
- 飞控 CDC：STM VCP（VID 0x0483/PID 0x5740）；端口号会漂移，必须按 USB 指纹识别并向用户确认当前设备。
- 历史 M1 工具仍可复查：`python tools/m1_baseline_check.py --port <COMx> --seconds 60`（只读）。主工作台：`python tools/drone_tcp_panel.py`。任何实机操作仍需当前 REQ 授权。

## 13. 审核清单（主模型在子模型结束后执行）

1. REQ 范围核对：diff 只含该 REQ；无巨文件追加、无证据文件改动、无 CubeMX 手改。
2. 测试：变化有有效覆盖；核对原始输出、版本、配置与验证范围，按验证策略决定复用/补跑，禁止将局部结果当成全量结果。
3. 语义：安全门/判据/坐标符号未被顺手放宽；放宽类改动必须独立提交并标「作者复核点」。
4. 设计：职责、接口、状态/数据语义符合已采纳决定，未另建平行实现；持久化 ABI/版本迁移（§6）及 lockstep 副本一致。
5. 证据：路径、schema、verdict 真实可复现；PIPELINE 状态与证据表一致（`tests/test_pipeline_contract.py` 绿）。
6. 索引新鲜、提交信息如实（不许"校准完成/可以飞"类越级措辞）。
