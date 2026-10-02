---
name: drone-h743-project
description: Apply shared drone-H743 rules for firmware, host tools, reviews, and coordinated agent work, with architecture continuity and risk-based validation. Use only in this repository and its worktrees.
---

# drone-H743 Project Skill

适用于 `D:\stm32hal\drone-H743` 及其 Git 工作树。共用规则只维护这一份；入口文件不复制。
角色由任务决定，不按模型品牌固定。作者当前授权优先；旧模板中的逐人全量、逐次写总表要求由下述流程替代，技术与实机判据不因此放宽。

## 开始与按需阅读

- **只讨论/只读**：回答当前问题；不改文件、不认领任务、不启动测试/构建/索引生成。
- **主控**：直接接作者需求，负责通俗讨论、路线取舍、设计延续、分派、审查、合并和收尾。实施前读 PIPELINE 当前节点、门和相关任务；工程契约按需读 [technical-spec](../../../doc/technical-spec.md)。协调多 Agent 时再读 [主控协作规则](references/dispatcher-prompt.md)。
- **执行者**：只做主控任务卡中的目标，读取相关源码、测试、设计条款和类别模式；不重新遍历全工程、不自行领取下一项、不修改其他人的范围。
- **审核职责**：由主模型（主控）在子模型结束后承担，检查改动、设计一致性和验证证据，决定通过或返修；可请其他模型协助复核，最终结论由主模型负责。主模型直接完成的小任务也须自查并记录依据，无须另开验收会话。
- 同一会话已读且未变化的材料不重复读。Do not load the complete historical evidence table；只读取本任务相关证据。已收官任务不按旧模板重新领取。
- 单人或多人实施前都检查 `git status` 与基线；有他人未提交改动时在干净工作树开工，禁止为此 stash/reset/clean 或覆盖原目录。主控合回原目录时只合自己的差异，并复查目标文件是否已变化。

## 与作者沟通

先讲功能解决什么、怎样使用、有什么限制。少用生僻词和代码细节，必要术语随文解释。
存在明显取舍时，比较 2～3 条路线的使用效果、工作量、可靠性、维护及硬件成本，给出推荐理由和未验证项，再与作者确定。
日常实现细节由主控处理；改变作者已确认设计、显著改变功能/成本或触及授权边界时再请作者决定。已授权的动作不重复确认。
汇报先说结果、验证范围、缺口和需要作者决定的事；命令原文与详细证据留在交付记录。

## 架构延续与修改边界

- [当前架构与已采纳设计](../../../doc/current-architecture.md) 是设计入口；任务卡钉住基线提交和相关决定。继承职责、接口、单位、状态归属；不得悄悄另建一套状态/解析/计算链或恢复旧方案。
- 模块内部实现可在范围内调整；改变跨模块职责/接口/数据语义先报主控。主控在授权内协调，推翻作者已确认设计或硬边界须由作者决定，并更新设计理由、通知受影响任务。
- 新模块、重构、优化先读 [解耦规范](references/decoupling-spec.md)。`App/Services → Driver → BSP → HAL/Core`；Services 同步且不占任务，芯片协议不进 BSP，禁止反向依赖。
- 手写文件参考上限为 C 约 1500 行、Python 约 2000 行。`tools/drone_tcp_panel.py` 与 `App/Src/app_control.c` **只减不增**；新功能进入模块，不固化当前行数。
- `drone-H743.ioc` 是硬件配置源；Core 启动/外设/MSP/RTOS 配置及 `USB_DEVICE/*` 归 CubeMX。引脚、时钟、DMA、NVIC 或生成对象变化，先说明 CubeMX 配置，等待生成后接入手写层；禁止手改生成代码。
- DMA/cache、内存、链接、MPU、大缓冲或任务栈工作先读 [H7 内存规则](references/h7-memory-domains.md)；确认可达性和 cache 维护，不能假定 DMA 可访问 DTCM。ISR 只搬运/唤醒，控制高频路径不做阻塞 I/O。

## Pipeline Governance

`PIPELINE.md` 只管当前进度和验收状态。主控承接作者指定任务；只有作者要求推进主线且未指定任务时，才选择直接需要的待做代码项。新增授权任务由主控登记；不擅自解冻或越过前置。
执行者返回代码版本、范围、测试原文和缺口；**仅主控**在整批收尾时更新本项任务及证据，失败与修复记录也须可追溯，不逐条命令追加总表。
子模型完成后交付为**待审核**，不得自行置 ✅；主模型审核后，全部判据满足才置 ✅，否则返修或保持待审核并列明缺口。需要实机证据的任务，只有软件验证时不能整体置 ✅；审核职责不代替设备操作授权。主线状态变化时同步图、门、指针和日期，未变化时不动其他节点。
PIPELINE 收尾修改后定向跑 `python -m pytest tests/test_pipeline_contract.py -q`。实现、主机测试、烧录、实机验收和飞行放行分别陈述。
未经当前任务明文授权，不烧录、复位、打开设备连接或发送目标板命令；设备按 USB 指纹与作者确认，不能固定猜 COM 号。

## 验证与索引

验证范围的唯一流程定义见 [validation-policy.md](references/validation-policy.md)：**默认定向验证；只有特别大的架构改动或作者当次明确要求才做全量回归**。阶段交审、准备烧录、协议或配置升版本身不触发全量。纯文档不跑固件构建；每项行为有有效覆盖，已有测试足够时复用，不为数量新增空泛测试。
缺陷先固定证据，按 [工作模式](references/work-modes.md) 最小修复；指定类别读对应模式。类别文件保存专属技术判据，不另加逐人全量流程。
多 Agent 的工作树/分支/文件归属/公共资源由主控分配；子任务完成不代表整批验证完成。

定位文件优先使用 [索引入口](references/repository-index/README.md) 的最小相关分片，再读头文件、测试及必要实现。任务卡已有准确路径时可直达；索引无匹配或过期时用窄范围 `rg` 核实，不扫描整个源码/数据/供应商树。
索引由主控在合并和文档收尾后统一生成：`python .agents/skills/drone-h743-project/scripts/update_repository_index.py`，随后 `--check`。子 Agent 不提交各自的生成索引。
普通任务只执行生成器，不读其源码；生成页不手改。保持入口 8 KiB、单分片 32 KiB、总计 128 KiB 上限（2026-10-01 作者“总上限放宽很多就行”，原 96 KiB），超限优先改善摘要。

## Canonical FLU Body Frame

`Driver/Inc/drv_frame_contract.h` 唯一定义右手 FLU：+X 前、+Y 左、+Z 上；正 roll 右翼下沉、正 pitch 机头下俯、正 yaw 机头左转。
涉及 IMU/磁力计/Fusion/导航/控制/RC/光流/执行器坐标或极性，先读 [FLU 契约](references/flu-coordinate-contract.md)，定向跑 `tests/test_flu_frame_contract.py` 及受影响链路测试。不得用 legacy 宏或负增益替代规范。
运行时迁移状态读契约头；掩码或 `IMUFRAME COMMIT` 不等于物理验收。保留 V0 映射、V1 测量、V2 控制/执行器的分段证据要求；安全门/判据/坐标符号变更须作者单独授权。

## 数据与其余资料

- 项目采集、日志和报告放 `data/`，运行文件按类别/日期归档；默认路径来自 `tools/project_paths.py`。新数据类别先读 [data/README](../../../data/README.md)，不重建散落的 tools/data 或根级日志目录；显式用户路径仍有效。
- `data/calibration/**` 历史证据不得修改/删除；算法结论须用作者确认的实录与同一数据上的基线比较，自造输入只用于单测。主机报文夹具锚定真实固件格式/编码器或注明出处的实录。
- 推力模型：飞控与台架验证只用推力查补表 `thrust_lut`，当前版本以 `data/identification/thrust/models/lut/current.json` 为准；多项式 `throttle_model`、实验库 `dataset_model`、`drv_coax_ctrl.c` 21 点旧曲线/`emit.py` 都不是现行换算 → [推力台约定](../../../doc/thrust-bench-contract.md)。
- FLASH/存储分层 → [flash-architecture](references/flash-architecture.md)；Param/后台慢操作 → [runtime-services](references/runtime-services.md)。
- 板级事实 → [hardware-reference](../../../doc/hardware-reference.md)；协议 → [telemetry-protocol](../../../doc/telemetry-protocol.md)；历史资料 → [history/README](../../../doc/history/README.md)，有多份历史输入时先确定日期、数据、固件和硬件来源。

源码与 `.ioc` 用于核实实现事实；与已采纳设计冲突时报告差异并走变更流程，不把未审实现自动升级为新规范。规范机体系始终以契约头为准。
