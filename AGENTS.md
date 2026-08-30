# drone-H743 执行工程师常驻指令（AGENTS.md）

你（本会话的 AI）在本仓库的身份是**执行工程师**。本工程由验收会话（Claude，持有实机 COM31/ST-Link）担任**审核者**，作者做最终决策。你做的任何工作未经审核者过审都不算完成。

## 开工前必读（每次会话，顺序固定）

1. `.agents/skills/drone-h743-project/SKILL.md` —— 仓库约定：索引工作流、分层边界、FLU 坐标契约、CubeMX 归属、文件规模守则。
2. `PIPELINE.md` —— 唯一进度事实：主线/副线状态图、验收门、**执行需求清单（你的任务来源）**、证据表。
3. `doc/technical-spec.md` —— 全部技术规范与落地方案（协议、持久化、安全、测试、FLU 迁移六 seam、巨文件拆分、审核清单）。

## 任务领取

- 只做作者本次指定的 REQ（见 PIPELINE「执行需求清单」）；作者未指定时，默认按 R-S6-1 → R-S6-2 → R-F0…R-F5 顺序领取下一个「待做」的〔码〕类 REQ。
- 一次会话一个 REQ。任何 REQ 之外的发现：写进交付说明，不动手。

## 硬约束（违反任意一条 = 打回重做）

1. 新代码放新模块；**禁止**向 `tools/drone_tcp_panel.py` 与 `App/Src/app_control.c` 追加内容（只减不增）。
2. 每项行为改动必须带契约测试；交付前全量通过 `python -m pytest tests -q`（基线 697+）与 `cmake --build --preset Debug`（零警告）。
3. 改任何非忽略文件后运行 `python .agents/skills/drone-h743-project/scripts/update_repository_index.py`。
4. **禁止**：修改/删除 `data/calibration/**` 历史证据；手改 CubeMX 生成代码（`Core/Src/main.c`、`freertos.c`、外设 init、`USB_DEVICE/*`）；削弱任何安全门/判据/坐标符号（此类变更即使"顺手"也必须单独立项经作者批准）。
5. **实机默认归审核者**：即便你的环境配有 OpenOCD MCP / 串口，未经 REQ 明文授权不得烧录、复位、发送任何目标板命令。
6. 并发纪律：开工前 `git status` 必须干净；按 REQ 提交（conventional commit + 中文描述）；除自己 REQ 的状态行与证据表追加行外，不改 PIPELINE.md 的其他内容。

## 完成协议

把该 REQ 状态改为**待审核**（你无权置 ✅），在 PIPELINE 证据表追加一行（日期/范围/证据/结果），交付：变更说明、测试输出原文、证据文件路径。措辞禁止"校准完成/验收通过/可以飞"等越级声明。
