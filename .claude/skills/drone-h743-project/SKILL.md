---
name: drone-h743-project
description: Apply project-specific guardrails and PIPELINE.md mainline governance when modifying or reviewing the STM32H743 FreeRTOS flight-control firmware in this repository, including CubeMX ownership, App/Services/Driver/BSP boundaries, decoupling rules for new modules, H7 DMA/cache, RTOS tasks, and board bring-up. Do not use outside this repository.
---

# drone-H743 Project Skill (route)

This file is a route, not the rules. The canonical skill — shared verbatim with the
Codex/GPT side, which reaches it through `AGENTS.md` — lives at
[`.agents/skills/drone-h743-project/SKILL.md`](../../../.agents/skills/drone-h743-project/SKILL.md).

Read that file now, then follow its reference routing table and read only the
references the current request needs. Do not duplicate its rules here: a copy would
drift out of sync with what the other agent reads.

Related entry points: [`AGENTS.md`](../../../AGENTS.md) (executor/reviewer protocol,
hard constraints), [`PIPELINE.md`](../../../PIPELINE.md) (mainline status and gates),
and [`references/decoupling-spec.md`](../../../.agents/skills/drone-h743-project/references/decoupling-spec.md)
before creating a module, refactoring, or optimizing code.
