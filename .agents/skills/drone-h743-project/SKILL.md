---
name: drone-h743-project
description: Apply project-specific guardrails and PIPELINE.md mainline governance when modifying or reviewing the STM32H743 FreeRTOS flight-control firmware in this repository, including CubeMX ownership, App/Services/Driver/BSP boundaries, H7 DMA/cache, RTOS tasks, and board bring-up. Do not use outside this repository.
---

# drone-H743 Project Skill

Use this skill only for `D:\stm32hal\drone-H743`. Inspect the files relevant to the request; do not load every reference by default.

## Repository Index Workflow

Use the generated repository index before reading project files:

1. Run `python .agents/skills/drone-h743-project/scripts/update_repository_index.py --check`. This checks the working tree without putting file contents into model context.
2. If it is stale, run the same command without `--check` before relying on the index.
3. Read [references/repository-index/README.md](references/repository-index/README.md), then only the smallest task-matching shard linked there.
4. Open the listed public header, test, or document first. Read implementations and dependencies only as the evidence requires.

Execute the generator as a tool; do not read its source during ordinary project work. Use a narrow `rg --files` or `rg -n` query only when the index has no matching entry or a listed dependency must be resolved. Do not dump the whole repository, preload every index shard, or recursively read source, vendor, capture, or data directories.

After adding, changing, moving, or deleting a non-ignored repository file, regenerate the index and run `--check` before finishing. The generator indexes hand-written code per module, aggregates vendor/data trees, and enforces an 8 KiB entry-page, 32 KiB per-shard, and 96 KiB total limit. If a limit is reached, improve grouping or summaries instead of raising the limit by default. Never edit generated index pages manually.

## Pipeline Governance

`PIPELINE.md` at the repository root is the authoritative mainline/sideline status map for bring-up work. After checking the repository index, read `PIPELINE.md` completely before planning or acting on every project task. Work is dispatched through its 执行需求清单 (REQ checklist) under the executor/reviewer protocol in `doc/technical-spec.md`: executors may only set a REQ to 待审核; only the reviewing session marks ✅ after verifying evidence.

- Classify the request against a named mainline or sideline node before changing files.
- Default to the current mainline node or a direct blocker revealed by it. Do not silently skip prerequisites.
- If a request conflicts with the mainline, prematurely advances a later node, or touches a frozen sideline, warn the author before editing and explain the consequence. Proceed as an exception only after the author explicitly chooses it, then record that decision in `PIPELINE.md`.
- Treat `completed`, `incomplete`, and `frozen` as evidence-backed states. Software implementation or host tests cannot complete a node whose gate requires flashing or physical verification.
- After every completed software test, build, flash, or physical validation, update `PIPELINE.md` in the same task. Update the Mermaid node and gate gap when status changes, and always update the latest-evidence table even when status does not change.
- Never claim calibration completion, runtime FLU compliance, flight release, or sideline reactivation unless the corresponding pipeline gate is satisfied and recorded.
- Keep the diagram, gate table, current-mainline pointer, latest-evidence table, and last-updated date mutually consistent. After every edit to `PIPELINE.md`, run `python -m pytest tests/test_pipeline_contract.py -q`; it mechanically enforces that consistency plus the M7 freeze while runtime FLU migration is incomplete.

## Project Data Paths

All project-owned captures, flight logs, identification datasets, calibration files, telemetry exports, and derived reports belong under `data/`. Run artifacts use `category/YYYY-MM-DD/`; undated historical artifacts use `category/undated/`, while stable calibration/reference files remain at the category root. `tools/project_paths.py` is the only source of default host-tool paths; reuse its constants and recursive discovery helpers instead of reconstructing paths in individual tools. Read `data/README.md` before adding a new data category. Do not recreate `tools/data`, root-level `captures*`, `saleae_capture*`, `log`, or `debug_flog_exports` directories. Explicit user-supplied input/output paths may still override defaults.

## Canonical FLU Body Frame

`Driver/Inc/drv_frame_contract.h` is the sole normative, machine-readable body-frame definition: right-handed FLU (`+X` forward, `+Y` left, `+Z` up), with positive roll moving the right wing down, positive pitch moving the nose down, and positive yaw turning the nose left.

Before changing IMU axes, Fusion/attitude adapters, navigation transforms, controller signs, RC direction, optical-flow axes, actuator allocation, servo polarity, or motor/yaw polarity, read [references/flu-coordinate-contract.md](references/flu-coordinate-contract.md). After each such change and before finishing, run `python -m pytest tests\test_flu_frame_contract.py -q`. Do not treat legacy sign macros or as-implemented documentation as an alternative body-frame definition. Keep the header, reference, Skill route, and test synchronized. Do not claim runtime FLU compliance while `DRV_FRAME_RUNTIME_MIGRATION_COMPLETE` evaluates to `0`.

For physical calibration work, preserve the staged acceptance boundary in that reference: V0 frame mapping and RAM/Flash confirmation, V1 sensor metrology, then V2 navigation/controller/RC/actuator validation. Never interpret an `IMUFRAME COMMIT` as flight release.

## Core Boundaries

Treat `drone-H743.ioc` as the hardware-configuration source of truth.

CubeMX owns generated startup and peripheral configuration, including `Core/Src/main.c`, `Core/Src/freertos.c`, peripheral init files, MSP code, matching `Core/Inc/*` headers, and `USB_DEVICE/*`. If a change affects pins, clocks, peripheral instances, DMA, NVIC, or CubeMX-created RTOS objects:

1. Tell the user exactly what to change in CubeMX.
2. Wait for regenerated code.
3. Then repair or connect the hand-written layers.

Hand-written code belongs mainly in `App/*`, `Services/*`, `Driver/*`, `BSP/*`, `tools/*`, and `tests/*`.

## Layering

Preserve this direction:

```text
App tasks / application-facing services / Services
  -> Driver
  -> BSP / board binding
  -> HAL / Core
```

- `App/*`: tasks, behavior, diagnostics, commands, control flow, and application-facing boundaries such as `App/*_service.*`.
- `Services/*`: synchronous, no-task domain/data services.
- `Driver/*`: reusable chip or protocol logic, named after the device or device class.
- `BSP/*`: board resource binding such as bus handles, GPIO/CS, locks, DMA callback registration, and cache-policy hooks.
- `Core/*`: CubeMX/HAL MCU configuration.

Do not put chip protocols in BSP or reverse the Driver-to-BSP dependency. Before changing FLASH or runtime service boundaries, read the corresponding reference below.

## File Size And AI Readability

The author requires bounded file sizes: giant files defeat AI context loading, hunk-level commit splitting, and human review. Guideline ceilings for hand-written code: roughly 1500 lines per C file and 2000 per Python file; when a change would exceed them, extract a module instead of adding a section (follow the `App/Src/app_stabilizer.c` extraction precedent).

Sideline S6 finished the scheduled split (panel 11446→5449, `app_control.c` 6257→4131), but **both files are still 2–3× over ceiling**. The append ban therefore stands: never add new features to `tools/drone_tcp_panel.py` or `App/Src/app_control.c` — create a new module and wire it in. Further shrinking of either file is opportunistic, not scheduled; do not start a new extraction mid-bring-up without the author's go-ahead.

## STM32H743 And FreeRTOS

For any DMA, cache, linker, MPU, section-placement, large-buffer, or task-stack work, read [references/h7-memory-domains.md](references/h7-memory-domains.md) before making changes. Never assume DMA can access DTCM; verify memory reachability and cache maintenance.

Keep ISRs short and wake tasks with RTOS primitives. CubeMX-created tasks and RTOS objects must be configured in CubeMX first; connect their behavior from `App/*` after regeneration.

## Validation

Validate in proportion to the change. Inspect existing tests and build presets before choosing commands. For architecture or naming changes, add or update focused contract tests in `tests/*` so removed dependencies and legacy names cannot return. Task-specific references contain focused validation commands where needed.

Two rules that a green test suite does not satisfy on its own. Any claim about an *algorithm's* behaviour must rest on recorded data under `data/`, compared against a baseline on the same dataset — self-authored inputs are for unit tests, never for the conclusion. And when a host tool parses firmware output, pin the test's fixture to the real emitted format (the firmware source format string or a captured line), because a name that firmware never emits fails silently rather than loudly.

## Task-Specific References

Read only the references relevant to the current request:

- Executor work modes — the default layer, the cross-cutting bug-fix mode, and how a per-class mode under `references/modes/` is mounted: [references/work-modes.md](references/work-modes.md)
- Planning and dispatching work orders, and authoring a new per-class mode before that class is dispatched for the first time: [references/dispatcher-prompt.md](references/dispatcher-prompt.md)
- FLU body axes, IMU/Fusion/navigation transforms, controller/RC/actuator polarity: [references/flu-coordinate-contract.md](references/flu-coordinate-contract.md)
- FLASH/GD25Q32 APIs, ownership, naming, or tests: [references/flash-architecture.md](references/flash-architecture.md)
- `Param`, `backgroundTask`, slow operations, or service/task ownership: [references/runtime-services.md](references/runtime-services.md)
- DMA/cache, memory domains, linker/MPU, buffers, or task stacks: [references/h7-memory-domains.md](references/h7-memory-domains.md)
- Current wiring, diagnostic commands, and board bring-up observations: [references/progress-notes.md](references/progress-notes.md)

Repository code and `.ioc` configuration override stale reference text except for the canonical body-frame definition: during FLU migration, `drv_frame_contract.h` remains normative and legacy runtime code only describes incomplete implementation. When an intentional change makes a reference inaccurate, update that reference in the same task.
