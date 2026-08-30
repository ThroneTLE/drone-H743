# drone-H743 归零检查 Pipeline

> 最后更新：2026-08-30  
> 当前主线位置：**M5 · 光流与测距校准**  
> 执行/审核工作流与全部技术规范：`doc/technical-spec.md`（执行者开工前必读，含派单提示词模板）  
> 本文件是工程推进状态的唯一入口。代码已经实现、测试通过、固件已经烧录和实机验收通过是不同状态，不得互相替代。

## 状态定义

- `✅ 完成`：该节点规定的实现、验证和证据全部满足，允许作为下一节点的前置条件。
- `🟡 未完成`：实现或验证证据至少有一项缺失；即使软件测试通过，也不能越级宣告完成。
- `⏸ 冻结`：当前不投入实现；只有作者明确决定解冻并更新本图后才能恢复。

## 主线、副线与当前状态

```mermaid
flowchart TB
    subgraph BASE["已完成的工程基础（不等于实机验收或飞行放行）"]
        direction LR
        B1["✅ FLU 坐标系唯一规范<br/>+X前 / +Y左 / +Z上"]
        B2["✅ Python 校准上位机框架<br/>页面按功能命名"]
        B3["✅ 校准参数事务机制<br/>RAM应用 / 撤销 / Flash写入 / 回读"]
        B4["✅ 软件回归基线<br/>696项测试通过"]
    end

    subgraph MAIN["主线：从底层重新取得实机证据"]
        direction TB
        M0["✅ M0 归零基线审计<br/>三路只读审计、主题化提交、过期说明清除（2026-08-30）"]
        M1["✅ M1 底层与原始数据健康<br/>2026-08-30 实机基线 PASS：992Hz、零丢样、无故障"]
        M2["✅ M2 坐标系与极性<br/>2026-08-30 作者当前固件完整 V0 复跑 6/6 PASS + Flash 写入回读一致"]
        M3["✅ M3 IMU 零偏与比例<br/>重分析 WARN 可用 + 漂移 A/B PASS + 免重 APPLY（2026-08-30）"]
        M4["✅ M4 舵机机械校准<br/>2026-08-30 实测：两路中心/方向/行程确认+Flash 写入+ST-Link 复位回读一致（gen=3）"]
        M5["🟡 M5 光流与测距校准 · 当前<br/>分析与补偿输出已完成；坐标、比例、零偏、旋转补偿未实测"]
        M6["🟡 M6 无桨控制链验收<br/>RC意图 → 导航 → 控制器 → 执行器，并验证失控保护"]
        M7["⏸ M7 带桨动力、振动与首飞<br/>M0～M6全部完成以前冻结"]
        M0 --> M1 --> M2 --> M3 --> M4 --> M5 --> M6 --> M7
    end

    B1 --> M0
    B2 --> M0
    B3 --> M0
    B4 --> M0

    subgraph SIDE["副线：保留，但不能抢占主线"]
        direction TB
        S1["🟡 S1 Python 上位机体验优化<br/>只修复主线实测暴露的问题，不无目的增加页面"]
        S2["⏸ S2 Serial-Studio 地面站<br/>以后用于遥测仪表盘和日志展示"]
        S3["⏸ S3 精密陀螺比例、非正交与温漂<br/>等待旋转台和多温点实验条件"]
        S4["⏸ S4 滤波器重新整定<br/>保持现有采集数据和滤波参数"]
        S5["⏸ S5 PID、控制律优化与新飞行功能<br/>底层验收完成后再恢复"]
        S6["🟡 S6 巨型文件拆分<br/>panel≈11k行 / app_control≈6k行；新功能一律新模块（立即生效），存量拆分待 M2 实机复跑后穿插"]
    end

    M0 -.约束.-> S1
    M1 -.具备实验条件后.-> S3
    M5 -.当前不影响校准主线.-> S2
    M6 -.通过后才能恢复.-> S4
    M6 -.通过后才能恢复.-> S5
```

## 主线验收门

| 节点 | 完成条件 | 当前缺口 |
|---|---|---|
| M0 归零基线审计 | 当前混合改动完成只读审计；按主题形成可解释的提交/变更单元；过期说明和状态矛盾清除 | 无（2026-08-30 完成：三路审计报告、7 个主题提交、references 过期条目修订、被污染证据还原并加写保护） |
| M1 底层与原始数据健康 | 当前固件实测确认供电、器件 ID、总线、采样率、时间戳、丢样和温度合理，并保存证据 | 无（2026-08-30 基线 PASS；供电为间接证据——全外设活跃、60s 无欠压复位；万用表实测列为可选加强项 R-M1-2） |
| M2 坐标系与极性 | 映射已知且硬件未动时的复核路径：持久化映射回读一致 + 硬复位重启回读一致 + 粗略正滚转/俯仰/偏航符号验证。六姿态重推导仅在映射未知或 IMU 重新安装时要求（本决定 2026-08-30 记录，理由：V0 为 24 选 1 离散判定，对摆放精度不敏感，历史置信度 99.53%） | 无（2026-08-30 作者以完整 V0 复跑超额完成：6/6 步 PASS、倾角↔Fusion 0.46°、Flash 写入、审核者独立回读 persisted=-x,-y,+z dirty=0） |
| M3 IMU 零偏与比例 | 当前工具对既有六面+静止陀螺证据重分析达 PASS/WARN；机上 persisted 标定与该证据同源；静止漂移 A/B 无恶化。徒手重采仅在出现 FAIL 面时要求；陀螺矩阵/温漂持续冻结在副线 S3 | 无（R-M3-1/2/4 全 ✅；M2 前置已满足，2026-08-30 置 ✅） |
| M4 舵机机械校准 | 拆桨固定条件下确认两路中心、物理方向和安全行程；写入后重启回读一致 | ✅ 2026-08-30：作者三项人工确认+COMMIT；审核者 ST-Link 复位回读逐字段一致（alpha 1530/1000/2000/−1，beta 1500/1000/2000/−1，gen=3，dirty=0） |
| M5 光流与测距校准 | 完成坐标、比例、零偏和旋转补偿验收，证据带 FLU 契约与姿态来源 | 目标端补偿量已可导出，缺实机采样与判定 |
| M6 无桨控制链验收 | RC、导航、控制器、执行器和失控保护端到端方向一致，执行器动作受固件端超时/互锁约束 | 尚未完成当前固件端到端实机验收 |
| M7 带桨动力、振动与首飞 | M0～M6 全部完成；运行时坐标迁移完成；另行批准带桨测试方案 | 冻结 |

## 执行需求清单（派单用）

规则：执行者认领一个 REQ，按 `doc/technical-spec.md` 全部规范实施；完成后只能把状态改为**待审核**并在证据表追加一行；**✅ 只能由审核会话在复核证据后落**。状态词汇：`待做 / 进行中 / 待审核 / ✅ / 条件跳过 / ⏸`。标注：〔码〕纯代码可远程派单；〔机〕需实机（审核者持有）；〔人〕需作者在场。

| REQ | 节点 | 需求 | 验收判据 | 状态 |
|---|---|---|---|---|
| R-M1-1 | M1 | 〔机〕60s 实机健康基线报告 | `tools/m1_baseline_check.py` verdict=PASS，报告入 `data/analysis/m1_baseline/` | ✅ |
| R-M1-2 | M1 | 〔人〕3V3/5V 轨万用表实测（可选加强） | 电压 ±5%，记入证据表 | 待做 |
| R-M2-1 | M2 | 〔机〕硬复位后映射/标定回读一致 | reset 后 orientation=3、frame=canonical_flu、cal_gen 不变 | ✅ |
| R-M2-2 | M2 | 〔人+机〕粗符号手势验证 | 机头下俯→+pitch、右翼下沉→+roll、机头左转→+yaw、回平复零；在线读数符号全对 | ✅（作者超额完成：面板完整 V0 复跑 6/6 PASS 含三个正旋转步骤） |
| R-M3-1 | M3 | 〔机〕8-28 六面会话按当前工具重分析 | verdict ∈ {PASS, WARN}，逐面残差 < 0.075g | ✅（WARN，最差面 0.033g/4.7°） |
| R-M3-2 | M3 | 〔机〕静止漂移 A/B vs 8-29 基线 | 漂移/零偏/温漂无恶化项 | ✅（PASS：偏航与零偏略改善，余项噪声底） |
| R-M3-3 | M3 | 〔人+机〕FAIL 面补采（条件触发） | 仅当 R-M3-1 出现 FAIL 面；手扶 ≥10s/面，WARN 即可用 | 条件跳过（无 FAIL 面） |
| R-M3-4 | M3 | 〔机〕确认免重新 APPLY | persisted cal_generation 与重分析证据同源则免 | ✅（同源链闭合，免重 APPLY） |
| R-M4-1 | M4 | 〔人+机〕SERVOCAL 实机标定 | 拆桨；两路中心/方向/行程人工确认，preview→COMMIT | ✅ 2026-08-30 |
| R-M4-2 | M4 | 〔机〕SERVOCAL 重启回读一致 | reset 后 SERVOCAL? 与提交值一致 | ✅ 2026-08-30 |
| R-M5-1..4 | M5 | 〔人+机〕光流静零/轴向/两点测距/旋转补偿 | 面板六阶段全 PASS；旋转补偿残余 ≤0.08m/s@≥15dps | 待做 |
| R-M6-1 | M6 | 〔人+机〕RCMAP 向导标定并 COMMIT | 12 步向导完成，重启回读一致 | 待做 |
| R-M6-2..4 | M6 | 〔人+机〕V2A 无桨端到端 + 失控保护 | ACCEPT 全阶段通过；断链进入安全态；方向一致性表全对 | 待做 |
| R-F0~F5 | M7前置 | 〔码〕运行时 FLU 迁移六 seam（spec §10，顺序固定） | 每 seam：先红测试→实施→host 绿→实机 A/B→翻掩码位 | 待做 |
| R-S6-1 | S6 | 〔码〕panel 拆分为 tools/panel_lib/ 包（spec §11） | 每次搬一页；行为零变更；全量绿 | 待审核（增量7~10；增量1~6 已过审 2026-08-30；余光流测距页） |
| R-S6-2 | S6 | 〔码〕app_control 按命令域拆分（spec §11） | 每域 ≤800 行；host 装置照编；构建零警告 | 待审核（增量1 SERVOCAL域） |

## 最近验证证据

每次完成任何软件、构建、烧录或实机验证，都必须在同一任务中更新本表；即使节点状态不变，也要更新证据与结论。只有满足对应验收门时才能修改图中的状态。

| 日期 | 范围 | 证据 | 结果 | 对状态的影响 |
|---|---|---|---|---|
| 2026-08-30 | S6 app_control 拆分增量1（R-S6-2） | 新建 `App/Src/app_cmd_servocal.c`（377 行）+ `App/Inc/app_cmd_servocal.h`（43 行）；`app_control.c` 6258→5933 行（−325 行）；SERVOCAL 命令族（clear_preview/set_event/report_servocal_record/report_servocal/parse_servocal/handle_servocal/service_servocal）与 8 个专属 static 变量整体搬移；适配器 `app_cmd_servocal_notify_persisted`/`app_cmd_servocal_is_busy`/`app_cmd_servocal_init` 闭合反向耦合；`tests/test_app_control_split.py` 锁定零残留与分发；全量 `737 passed`；`cmake --build --preset Debug` 零警告；未启动串口/板机 | 软件验证通过，待审核 | R-S6-2 置**待审核（增量1 SERVOCAL域）**；S6 保持 🟡，主线仍为 M5 |
| 2026-08-30 | S6 panel 拆分增量10（R-S6-1） | 父提交 `69bf4158`；`tools/panel_lib/pages/validation_v0.py` 的 `ValidationV0PageMixin` 接管 38 个 V0 builder/handlers 与 `v0_workflow_guidance`，`tools/panel_lib/evidence.py` 的 `EvidenceMixin` 接管 18 个共享安全/证据方法及 4 个证据 helper、10 个 validation 常量；独立比对 56/56 方法、5/5 helper、5/5 类常量、10/10 模块常量 AST 全部相同。`_validation_autosave_session`/`_validation_autosave_workflow` 首条 loaded_history 早退守卫逐 AST 保持；`test_evidence_write_protection.py` 仅 owner 锚点重定向、断言未变。聚焦 `190 passed`；开发期两个聚焦红灯均为旧源码/monkeypatch owner 锚点，纯重定向后归零；全量 `736 passed`；Debug 构建 `ninja: no work to do.`、零警告；索引 current；未启动串口/板机 | 软件验证通过，待审核 | R-S6-1 统一置**待审核（增量7~10）**；panel 8166→5814 行；S6 保持 🟡，主线仍为 M5 |
| 2026-08-30 | S6 panel 拆分增量9（R-S6-1） | 父提交 `4d87d78f`；`tools/panel_lib/pages/acceptance_v2.py` 的 `AcceptanceV2PageMixin` 接管 `_build_v2_page` 与 5 个 `_v2_*` handlers，6/6 方法 AST 相同，builder 保持 22 条语句；V2A 租约/ESC 安全门/证据源码锚点仅重定向。聚焦 `81 passed`；全量 `735 passed`；Debug 构建 `ninja: no work to do.`、零警告；索引 current；未启动串口/板机 | 软件验证通过，待审核 | R-S6-1 保持进行中直至本批统一交审；panel 8285→8166 行；不改变 M5 |
| 2026-08-30 | S6 panel 拆分增量8（R-S6-1） | 父提交 `4eaf9f29`；`tools/panel_lib/pages/vibration.py` 接管 1 个振动占位 builder，`tools/panel_lib/pages/servo_debug.py` 接管 12 个舵机调试 builders/handlers（含 `_send_raw`、`_update_servo_ok_line`）；13/13 方法 AST 相同；参数编辑页共享的 `_param_value_for_servo` 明确保留 legacy。聚焦 `42 passed`；全量 `734 passed`；Debug 构建 `ninja: no work to do.`、零警告；索引 current；未启动串口/板机 | 软件验证通过，待审核 | R-S6-1 保持进行中直至本批统一交审；panel 8472→8285 行；不改变 M5 |
| 2026-08-30 | S6 panel 拆分增量7（R-S6-1） | 父提交 `e88b3d99`；`tools/panel_lib/pages/mechanical.py` 的 `MechanicalPageMixin` 接管 1 个 34 条语句 builder 与 12 个 `_mechanical_*` handlers（含 `_mechanical_nudge_center`/`_mechanical_jog_stop`），13/13 方法 AST 相同；光流页 8 个方法哈希冻结未动。聚焦最终 `43 passed`；首次全量 `2 failed, 731 passed` 仅因增量5/6旧契约仍要求机械 owner 留在 legacy，断言/哈希纯重定向后最终全量 `733 passed`；索引硬限通过缩短本增量测试索引表面解决，未抬限制；Debug 构建 `ninja: no work to do.`、零警告；未启动串口/板机 | 软件验证通过，待审核 | R-S6-1 保持进行中直至本批统一交审；panel 8901→8472 行；光流页继续冻结，不改变 M5 |
| 2026-08-30 | 审核：R-S6-1 增量6 过审（含并发事故复盘） | 审核者独立复核，范围按执行者建议取 `8b8085c6..820c8150`：35/35 方法（34 handlers + 72 条语句 builder）与 9/9 顶层 helper 相对基线 8b8085c6 AST 逐一相同；panel 零遗留、MRO 头插正确；模块 `UI_PALETTE` 为 3 键子集，与方法内实际引用（accent×1/border_strong×3/console×1）精确闭合且被其契约测试逐键锁定等于主调色板；机械/光流/jog 五方法相对真实父提交 8abc4ec0 逐字节不变；`test_rc_mapping.py` 全部为源锚点纯重定向；PIPELINE 只动自己行、零证据触碰；审核者复跑全量 `732 passed`、构建零警告。**并发事故（责任在审核者）**：审核侧样式提交 27149729 `git add` 整文件时卷入执行者未提交的 RC 拆分 WIP（+35/−1079），导致 27149729/9c587ca3/8abc4ec0 三个中间提交的 panel 引用尚不存在的 rc_wizard 模块（checkout 即 import 失败）；执行者如实检测并披露，820c8150 补齐后 HEAD 恢复完整。决定不重写历史（证据链已引用相关 SHA），审核者此后 add 前必须对目标文件做外来 hunk 检查 | **过审** | R-S6-1 置"进行中"（增量6完成）；panel 9962→8900 行 |
| 2026-08-30 | S6 panel 拆分增量6（R-S6-1） | `tools/panel_lib/pages/rc_wizard.py` 的 `RcWizardPageMixin` 接管完整 `_build_rc_page` 与 34 个 `_rc_*` handlers，并接管 9 个 RC 纯 helper/15 个专属常量；相对指定基线 `8b8085c6` 独立比对 35/35 方法 + 9/9 helper AST 全部相同，builder 保持 72 条语句；契约锁定旧路径同对象转发、直接脚本导入及机械/SERVO JOG/光流页相对实际父提交不变；既有 RC 源码契约仅重定向 owner。并发说明：审核侧样式提交 `27149729` 在共享工作树中提前收录了本增量的 panel 转发/删除，故审核范围须按 `8b8085c6..本增量最终提交` 复核；聚焦回归 `80 passed`，全量两次绿色：`732 passed`、`731 passed, 1 skipped`（Tk 条件跳过）；`cmake --build --preset Debug`：`ninja: no work to do.`、零警告；未启动串口/板机 | 软件验证通过，待审核 | R-S6-1 置**待审核（增量6）**；panel 9962→8900 行；S6 保持 🟡，不改变主线位置 M5 |
| 2026-08-30 | **M4 舵机机械校准（作者实测 R-M4-1 + 审核者复位回读 R-M4-2）** | 作者拆桨台架完成两路中心/方向/行程三项人工确认并 preview→COMMIT：alpha 中心1530 限位1000/2000 sign=−1，beta 中心1500 限位1000/2000 sign=−1（两路均"机构标记反向"），证据 `servo_mechanical_20260830_140743.json`（status=PERSISTED_READBACK_MATCH）。审核者 R-M4-2：ST-Link 硬复位前后 `SERVOCAL?` 三行回读，persisted 复位前后逐字段一致、开机 active==persisted、valid=1、dirty=0、gen=3，证据 `servocal_reboot_readback_20260830_141226.json` VERDICT=PASS（首次采集因主机脚本截断 persisted 行误判 FAIL，属工具缺陷，已修复重采并删除误导文件）。另核实作者提出的映射语义疑虑：斜率恒为物理常数 (2500−500)µs/180°（`drv_coax_ctrl.c` `coax_ctrl_tilt_rad_to_servo_pulse`），标定 min/max 仅作输出钳制限位，不参与量程换算；`SERVO ANGLE` 调试路径同 | **M4 PASS** | **R-M4-1、R-M4-2 置 ✅；M4 置 ✅；当前主线位置移至 M5** |
| 2026-08-30 | 审核：R-S6-1 增量5 过审 | 审核者独立复核：范围=单提交 631e1fb1（基于 911a8167），零证据文件触碰；独立 AST 比对 24/24 方法（23 handlers + builder）与父提交逐一相同、builder 54 条语句、panel 零遗留、MRO 头插 V1PageMixin 正确；机械/光流页及 `_mechanical_move` 在其提交点与父提交逐字节相同（遵守"不动作者在用页"指令）；5 个既有测试改动全部为源锚点纯重定向、断言无削弱；新契约测试沿用金标准（AST SHA-256 钉死 + 同对象转发 + 子进程直跑导入）；审核者在含本增量的 HEAD 全量复跑 `725 passed`、构建零警告；零板机接触 | **过审** | R-S6-1 置"进行中"（增量5完成）；panel 10468→9886 行（后续审核者 jog 修复又 +76） |
| 2026-08-30 | M4 阻塞缺陷修复：SERVO JOG 保持型点动（审核者实机） | 作者台架实测：机械页点最小/最大"走到一半被拉回中点"。根因=稳定环 commit 持续流式下发目标（3µs 死区+500ms 强制刷新），一次性 `SERVO MOVE` 慢移必然被覆盖。修复：新模块 `App/Src/app_servo_jog.c` 在 commit 仲裁点接管（优先级 手势标定>验收>反馈台架>解锁>点动），500µs/s 斜坡保持、120s 超时自动交还；机械页改发 `SERVO JOG`、增中点微调（±2/±10 µs 实时跟随）、「结束点动」与两路 FLU 判向文字（alpha：+50µs→左(+Y)倾=正向；beta：+50µs→机尾(−X)倾=正向，源自 `coax_ctrl_body_tilt_to_servo_tilts` 符号链）。`tests/test_servo_jog_contract.py` gcc 宿主编译真模块验证斜坡/保持/重定目标/超时/让位/命令面 + 仲裁顺序源码契约；全量 `725 passed`；构建零警告；OpenOCD 烧录 `Verified OK`；COM31 实测 7/7 PASS（JOG/保持中 IMU? 存活/STOP/非法参数/SERVOCAL? 回读）。附带修复：build/Debug 系统识别被污染（CMakeSystem 记录 Windows/非交叉），清树重配置恢复 | 实机验证通过 | R-M4-1 的点动阻塞解除，M4 台架可继续；主线位置不变 M4 |
| 2026-08-30 | S6 panel 拆分增量5（R-S6-1） | `tools/panel_lib/pages/v1_metrology.py` 的 `V1PageMixin` 接管完整 `_build_v1_page` 与 23 个 `_v1_*` handlers；相对父提交 `911a8167` 独立比对 24/24 方法 AST 全部相同，builder 保持 54 条语句并继续唯一调用漂移页 builder；`tests/test_panel_v1_page_extraction.py` 锁定实现所有权、同对象转发、直接脚本导入、依赖/常量等价及机械/光流页不迁移；既有 V1 源码契约仅重定向到新 owner；聚焦回归 `117 passed`，全量 `719 passed`；`cmake --build --preset Debug`：`ninja: no work to do.`、零警告；未启动串口/板机 | 软件验证通过，待审核 | R-S6-1 置**待审核（增量5）**；panel 10468→9886 行；S6 保持 🟡，不改变主线位置 M4 |
| 2026-08-30 | 审核：R-S6-1 增量4 过审 | 审核者独立复核：范围=单提交 f5ddbe88（基于 7fb9a605）；7/7 handler AST 逐一相同；builder 属"片段抽方法"型搬迁——审核者独立验证 17/17 条语句与父提交 `_build_v1_page` 内**连续片段**逐条相同，宿主 70→54 条（−17+1 调用，算术精确）、调用点唯一；零残留、DriftPageMixin 继承；全量 `713 passed` 与构建复跑一致；零板机接触。执行者以语句级（非函数级）表述一致性，措辞精确 | **过审** | R-S6-1 置"进行中"（增量4完成）；panel 10612→10468 行 |
| 2026-08-30 | S6 panel 拆分增量4（R-S6-1） | 新建 `tools/panel_lib/pages/`，`DriftPageMixin` 接管静止漂移 builder 与全部 7 个 `_drift_*` handlers；原 V1 页位置仅调用继承的 builder，旧模块保留同对象转发；`tests/test_panel_drift_page_extraction.py` 以预拆分 HEAD `88e33205` 的 SHA-256 锁定 17 条 builder AST 语句和 7/7 handler AST，并验证所有权、转发及内存样本配对；聚焦回归 `76 passed`，全量 `713 passed`；`cmake --build --preset Debug`：`ninja: no work to do.`、零警告；未启动串口/板机 | 软件验证通过，待审核 | R-S6-1 置**待审核（增量4）**；panel 10612→10468 行；S6 保持 🟡，不改变主线位置 M4 |
| 2026-08-30 | **M2 完整 V0 复跑（作者实测，R-M2-2 超额）** | 作者在当前固件（面板为 S6 增量3 后版本）完整重跑 V0：RAM 映射复验 6/6 PASS（静止水平66/机头朝上64/左侧朝上62/+roll 52/+pitch 48/+yaw 49 样本）、加速度倾角↔Fusion 中位误差 0.46°（限8°）、候选 `R=diag(-1,-1,+1)` conf=99.53%、`IMUFRAME COMMIT` 写入 Flash；审核者独立串口回读：`active=persisted=-x,-y,+z dirty=0 frame=canonical_flu_persisted arm_lock=1`（arm_lock 因迁移掩码未完成按契约保持）。附注：8-28 归档 session/workflow 被续采路径以逐字节相同内容回写（仅行尾差异，已还原）——归档状态与本次实测状态完全一致，构成跨固件不变性旁证 | **PASS** | **M2 置 ✅（三条腿全满足）；M3 前置解除同时置 ✅；当前位置移至 M4** |
| 2026-08-30 | R-M2-2 判据讨论归档 | 作者质疑手势复验与 8-28 已做的 V0 动态验证重复；审核者核实：运行时姿态链零符号改动（三路审计+FLU 契约测试）+ M6 方向矩阵天然覆盖动态复核，同意免除。随后作者直接以完整 V0 复跑给出更强证据，讨论以超额完成收束。备用工具 `tools/gesture_sign_check.py`（录制+自动分段判号）保留入库，供 M6 方向一致性日与后续 seam 抽查复用 | 归档 | 判据变更记录在案；工具入库 |
| 2026-08-30 | 审核：R-S6-1 增量3 过审 | 审核者独立复核：范围=单提交 1e204ec4；AST 独立比对 2 函数+2 方法+2 日志路径逐一相同；旧位置零残留、PanelStateMixin 继承、4/4 转发；路径全走 project_paths（PANEL_STATE_PATH/LOG_DIR）合规；两个既有测试改动均为源断言重定向（含一处切片边界的机械适配）；tmp_path 声明抽查属实（真路径仅同一性断言+monkeypatch 后 I/O）；全量 `708 passed` 与固件构建审核者复跑一致；零板机接触 | **过审** | R-S6-1 置"进行中"（增量3完成）；panel 10673→10612 行 |
| 2026-08-30 | S6 panel 拆分增量3（R-S6-1） | `tools/panel_lib/state.py` 接管 `_load_panel_state`/`_save_panel_state`、`PANEL_STATE_PATH`/日志路径及 `append_log`/`record_panel_crash`；`tests/test_panel_state_extraction.py` 锁定实现所有权、旧路径同对象转发并仅用 `tmp_path` 验证状态与日志 I/O；与父提交 AST 比对 2 函数+2 方法+2 日志路径全部一致；全量 708 项三次均绿：两次 `708 passed`、一次 `707 passed, 1 skipped`（Tk 条件跳过）；`cmake --build --preset Debug`：通过、零警告 | 软件验证通过，待审核 | R-S6-1 置**待审核（增量3）**；S6 保持 🟡，不改变主线位置 |
| 2026-08-30 | M3 漂移 A/B（R-M3-2） | 新工具 `tools/drift_ab_check.py`（只读，60s/475帧@8Hz）：新固件 0xF12AD9F5 vs 8-29 旧固件 0x15F33731、同标定 gen=2。偏航漂移 1.352→1.333°/min（改善）、陀螺零偏峰 0.0243→0.0211dps（改善）、横滚/俯仰 ±0.01°/min（噪声底）、\|a\| 偏差 15.6→16.1mg（+0.5mg 重复性）；两份报告均 WARN（已知标定残差），无恶化项 | **PASS** | R-M3-2 置 ✅；报告入 `data/calibration/imu_metrology/2026-08-30/stationary_drift/` |
| 2026-08-30 | M3 免重 APPLY（R-M3-4） | 同源链：板上 persisted `cal_generation=2`（今日两次回读含硬复位后）← `imucal_commit_20260829_211321.json` ← `room_temperature_candidate.json`（`candidate_calibration_generation=2`，`session_id=v1-20260828-221343`，来源指纹=该会话六面采集）= R-M3-1 重分析的同一会话。附注：重分析会就地刷新派生的 candidate 文件（原始采集 CSV/meta 未动，属文档化的派生物再生成） | 成立 | R-M3-4 置 ✅；M3 三项证据齐备，仅待 M2 前置 |
| 2026-08-30 | 审核：R-S6-1 增量2 过审 | 审核者独立复核：范围=单提交 00beb5f3；**AST 独立比对 5 函数+2 方法逐一相同、82/82 常量与真源逐字节一致**（校验器曾误报 4 个常量不一致，实为父提交 panel 的增量1转发行覆盖了 transport 真定义的归并优先级问题，肉眼比对排除）；旧位置零残留、面板类继承 ProtocolLineMixin、82/82 转发、transport→proto 单向依赖无环；全量 `704 passed` 与固件构建审核者复跑一致（一次复跑出现 687+17 瞬态条件跳过，总数吻合、与提交无关）；本增量无任何串口/板机接触 | **过审** | R-S6-1 置"进行中"（增量2完成）；panel 10769→10673 行 |
| 2026-08-30 | S6 panel 拆分增量2（R-S6-1） | `tools/panel_lib/proto.py` 接管 82 个 `PROTO_*` 常量、5 个纯行解析函数和 2 个协议行规范化方法；`tests/test_panel_proto_extraction.py` 锁定实现所有权、旧路径同对象转发、transport 常量复用及 build/CRC 留在 transport 的边界；与父提交 AST 比对全部一致；`python -m pytest tests -q`：704 passed；`cmake --build --preset Debug`：通过、零警告 | 软件验证通过，待审核 | R-S6-1 置**待审核（增量2）**；S6 保持 🟡，不改变主线位置 |
| 2026-08-30 | 审核：R-S6-1 增量1 过审 | 审核者独立复核（spec §13）：范围=单提交 e699b789 仅含拆分相关文件；**AST 独立比对 13/13 定义与拆分前逐一相同、旧文件零残留、13/13 转发**；三个既有测试改动均为 move 导致的 monkeypatch/源断言重定向；全量 `700 passed` 与固件构建由审核者亲自复跑。执行者披露的开发期短暂打开 COM31：根因=monkeypatch 打在旧模块（与 diff 证据吻合），终态已修复、无字节收发、板机无影响——记违规一次（AGENTS.md 规则5），因如实披露且无害不打回 | **过审** | R-S6-1 置"进行中"（增量1完成）；panel 11446→10769 行 |
| 2026-08-30 | S6 panel 拆分首增量（R-S6-1） | `tools/panel_lib/transport.py` 接管 TCP/UDP/串口、USB 指纹匹配与重枚举等待；`tests/test_panel_transport_extraction.py` 锁定新模块所有权、旧路径同对象转发及直接脚本导入；13 个搬移定义与拆分前 AST 一致；`python -m pytest tests -q`：700 passed；`cmake --build --preset Debug`：通过、零警告 | 软件验证通过，待审核 | R-S6-1 置**待审核**；S6 保持 🟡，不改变主线位置 |
| 2026-08-30 | M1 实机基线（R-M1-1） | 烧录 98d5637c 后 `m1_baseline_check.py` 60s×10Hz：588→再跑 PASS；均值 991.9Hz、最坏间隙 0.5ms、fault=0、\|a\|=1016mg、陀螺均值<0.04dps、温升 0.57°C；首跑 6 次丢行为采集脚本竞态（health 行未等齐），已修并注释 | PASS | **M1 置 ✅，当前位置移至 M2** |
| 2026-08-30 | M2 重启回读（R-M2-1） | OpenOCD `reset run` 硬复位后 IMU?：orientation=3、frame=canonical_flu_ram、cal_generation=2/mask=0x07 从 Flash 正确重载 | PASS | M2 仅余粗符号手势（R-M2-2） |
| 2026-08-30 | M3 离线重分析（R-M3-1） | 8-28 六面+静止陀螺会话按当前判据重分析：WARN 可用（RMS 0.0268g，超 PASS 线约1.5°；六面残差 0.020~0.033g、1.0~4.7°，全部远低于 0.075g FAIL 线） | 通过 | 免徒手重采；M3 余静止漂移 A/B |
| 2026-08-30 | 执行者入口打通 | 新建根目录 `AGENTS.md`（Codex/GPT 每会话自动加载）：强制先读 SKILL/PIPELINE/spec 三件套、REQ 领取规则、六条硬约束（含"实机默认归审核者"——GPT 环境配有 OpenOCD MCP 但未派单不得碰板）、完成协议 | 生效 | GPT 执行链路与本清单对接完成 |
| 2026-08-30 | 治理升级：执行/审核分工 | 新增本文件「执行需求清单」+ `doc/technical-spec.md`（13 章规范：架构/协议/持久化/安全/测试/FLU迁移方案/巨文件拆分方案/派单模板/审核清单） | 生效 | 执行者只能置"待审核"，✅ 由审核会话落 |
| 2026-08-30 | M0 归零基线审计收口 | 三个并行只读审计报告（固件C/上位机/测试文档数据）；工作区 58 个文件拆成 7 个主题提交；全量 `696 passed`；Debug 构建零警告 | 通过 | **M0 置 ✅，当前位置移至 M1**；M1 作业单 `doc/m1-baseline-runbook.md` |
| 2026-08-30 | 审计发现的缺陷修复 | ①历史验收证据被面板 autosave 覆盖（`target_state_at_save`→`{}`）：已还原并加只读守卫+回归测试；②500Hz 控制环内 4 处同步 USB 阻塞写（最坏~30ms）：改事件通告由通信任务补发+契约测试；③协议 ID 0x1022/0x1023 补登记；④反向通道阈值边界改严格镜像；⑤RC 配置发布前读取回退出厂默认；⑥App/Driver 舵机判据副本加锁步契约测试 | 通过 | 不改变节点状态；提高 M1 一次跑通概率 |
| 2026-08-30 | 治理约束新增 | 作者要求限制单文件规模：新功能一律新模块，panel/app_control 拆分立为副线 S6（SKILL.md 已写入守则） | 生效 | 新增 S6 🟡；对巨文件的追加自即日起被守则禁止 |
| 2026-08-29 | 治理文档实证复核 | 全量回归复跑 `682 passed`；证据表原引用的 `quick_validate.py` 证实全仓不存在，已移除该虚假引用；迁移掩码引用与 `drv_frame_contract.h:54` 核对一致；契约测试并入并刷新索引后全量 `688 passed` | 通过 | 新增 `tests/test_pipeline_contract.py` 机械校验本文件（节点/验收门/当前指针/证据日期/M7 冻结联动）；M0 保持当前节点 |
| 2026-08-29 | Pipeline 治理入口 | SKILL.md「Pipeline Governance」+ `tests/test_pipeline_contract.py` 一致性契约 | 通过 | 建立每次任务先核对主线、验证后回写状态的强制流程；M0 保持当前节点 |
| 2026-08-29 | 全量软件回归 | `python -m pytest tests -q` | `682 passed` | 仅确认软件回归基线；不提升 M1～M7 |
| 2026-08-29 | Debug 固件构建 | `cmake --build --preset Debug` | 构建通过 | 仅确认可构建；不代表已烧录或实机通过 |
| 2026-08-28 | 历史 V0 坐标证据 | `R_FLU<-legacy_intermediate_v1 = diag(-1,-1,+1)` | 历史证据有效 | 当前固件改动后仍须重跑，M2 保持未完成 |
| 2026-08-29 | 运行时 FLU 迁移 | `DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK = 0U` | 未完成 | 禁止宣称运行时 FLU 完成或自由飞行放行 |

## 推进与更新规则

1. 每个工程任务开始前先读取本文件，指出它属于哪个主线节点或副线节点。
2. 默认只推进“当前主线位置”或修复其直接暴露的阻塞问题，不得无理由跨越前置节点。
3. 请求与主线不一致、试图提前推进后续节点或触碰冻结副线时，必须先提醒作者并说明代价；只有作者明确决定后才能作为例外执行。
4. Python 上位机是当前校准与实机验收工作台；Serial-Studio 暂不扩张校准功能。
5. 一次任务只处理一个可验证边界。没有实机证据时，只能写“实现完成”或“待实机”，不能写“校准完成”“验收完成”或“可以飞”。
6. 完成验证后必须更新本文件：更新 Mermaid 节点说明或状态、主线验收门缺口、最近验证证据和“最后更新”日期。
7. 若验证失败，节点保持未完成，并记录失败证据和下一阻塞；不得通过降低判据、翻转符号或隐藏异常来推进状态。
8. 只有当前主线节点完成后，才能把“当前主线位置”移动到下一节点。解冻副线也必须在本图中显式记录。
