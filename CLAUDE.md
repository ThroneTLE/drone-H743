# drone-H743 —— Claude 会话入口

本文件只是**指针**，不存放规则本身。规则的唯一权威在下面这些文件里，改规则去改它们，不要在这里复制一份。

## 每次会话按顺序读

1. [`.agents/skills/drone-h743-project/SKILL.md`](.agents/skills/drone-h743-project/SKILL.md)
   —— 仓库约定的总入口：索引工作流、PIPELINE 治理、`data/` 路径、FLU 坐标契约、
   CubeMX 归属、App/Services/Driver/BSP 分层、文件规模守则、按需引用的 references 路由表。
   它与 Codex 侧读的是**同一份文件**。
2. [`AGENTS.md`](AGENTS.md) —— 执行者/审核者协议、任务领取、硬约束、完成协议。
   本会话（持有实机与 ST-Link）默认是**审核者**；作者明确派活时按作者说的做。
3. [`PIPELINE.md`](PIPELINE.md) —— 主线状态图、验收门、执行需求清单。
   只读本次 REQ 相关的证据行，不要加载整份历史证据表。

写代码 / 重构 / 优化前追加读
[`.agents/skills/drone-h743-project/references/decoupling-spec.md`](.agents/skills/drone-h743-project/references/decoupling-spec.md)。
其余 references 按 SKILL.md 末尾的路由表**按需**读，不要预加载。

## 两条最容易踩的硬约束

- **禁止**手改 CubeMX 生成代码（`Core/Src/main.c`、`freertos.c`、外设 init、`USB_DEVICE/*`）；
  也**禁止**向 `tools/drone_tcp_panel.py` 与 `App/Src/app_control.c` 追加内容（只减不增）。
- 改过任何非忽略文件后跑
  `python .agents/skills/drone-h743-project/scripts/update_repository_index.py`。
