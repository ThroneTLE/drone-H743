---
spec: 0012-project-discoverability
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-16
---

# Plan 0012 — LLM discoverability primitives for large projects

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Grounded in the actual code (all touch-points below confirmed by
> read/grep, including the tool-catalog plumbing, the elision exemption set, the snapshot
> windowing helpers, and the entity structs).

## Approach (one paragraph)

`project.search` and `project.group.get` are implemented as a new per-concern TU
`ProjectHandlerDiscovery.cpp` (following the spec-0002 split pattern; registration stays in
`ProjectHandler.cpp`), using a **one-walk collector**: a single pass over the live
`ProjectModel` accessors (`sources()` → `groups()`/datasets → `actions()` →
`activeWorkspaces()` → `tables()`) in that fixed order, matching case-insensitively, skipping
`offset` matches, emitting up to `limit` compact rows, and counting to the end for
`matchCount` — no cache, no index, deterministic by construction. Existing list commands gain
optional `offset`/`limit` reusing `project.snapshot`'s windowing idiom + `attachProjectEpoch`;
their **on-the-wire default stays unlimited** (TCP/SDK back-compat) while
`ToolDispatcher::executeCommand` injects a default `limit` when the *assistant* calls them
without one. `meta.search` is a new `ToolDispatcher::searchCommands()` sibling of the existing
`listCommands()` (substring over name+description of the same merged catalog, sorted by name
for determinism), dispatched via `Conversation::dispatchMetaTool` like its peers. R9 is three
string-surgery edits in `Conversation.cpp` plus five names added to the existing
`isFsContent`/`isDiscovery` elision exemption.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/API/Handlers/ProjectHandlerDiscovery.cpp` | **New TU**: `projectSearch` + `groupGet` handler impls (one-walk collector, compact row builders). |
| `app/src/API/Handlers/ProjectHandler.h` | Declare `projectSearch`, `groupGet` static handlers + `registerDiscoveryCommands()`. |
| `app/src/API/Handlers/ProjectHandler.cpp` | Register `project.search` / `project.group.get` (schemas via `SchemaBuilder.h`); call `registerDiscoveryCommands()` from `registerListCommands()`. |
| `app/src/API/Handlers/ProjectHandlerBatch.cpp` | Add `offset`/`limit` windowing + `attachProjectEpoch` + window self-identification to `groupsList`, `datasetsList`, `datasetGetExecutionOrder`. |
| `app/src/API/Handlers/ProjectApiSupport.h` | Shared windowing helper (`applyWindow(total, offset, limit)` → start/count/nextOffset), mirrored from `buildSnapshotGroups`' `qBound` idiom, if extraction beats duplication (3 call sites). |
| `app/src/AI/ToolDispatcher.cpp` | `meta.search` in `metaToolRoster()`; new `searchCommands(query, offset, limit)` next to `listCommands()`; default-`limit` injection for the three list commands in `executeCommand`. |
| `app/src/AI/ToolDispatcher.h` | Declare `searchCommands`. |
| `app/src/AI/Conversation.cpp` | `dispatchMetaTool` route + `runMetaSearch`; `makeMetaTool` entry in `appendCommandMetaTools`; R9 wording in `makeTruncatedResult` + `elideAgedToolResult`; add `project.search`, `project.group.get`, `meta.search` (+ paged lists stay elidable) to the `isDiscovery`/`isFsContent` exemption at `ageHistoryToolResults`. |
| `app/rcc/ai/command_safety.json` | `project.search`, `project.group.get` → `"safe"` tier (meta.* stays absent by design — dispatched pre-registry). |
| `app/CMakeLists.txt` | Add `src/API/Handlers/ProjectHandlerDiscovery.cpp` (next to line 262). |
| `doc/help/Identity-Model.md` | R6: two-group-id-spaces section + the workspaces `groupId`-carries-`uniqueId` collision warning. |
| `doc/help/API-Reference.md` | Document the two new project commands + new paging params. |
| `app/rcc/ai/skills/api_semantics.md` | R6 warning + point discovery flows at `project.search`. |
| `app/rcc/ai/skills/dashboard_layout.md` | Strengthen the existing `:330` warning with the `project.group.get` escape hatch. |
| `app/rcc/ai/skills/tool_discovery.md` | Register `meta.search` in the discovery flow (search → describe → execute). |
| `tests/integration/test_project_discovery.py` | **New**: AC1–AC5 (fixture built via `project.dataset.addMany`/`project.batch`, 87 groups / 570 datasets). |
| `tests/scripts/test_ai_assistant_static.py` | AC7/AC9 string-level asserts (safety tiers, exemption names, distinct notice wording). |
| Generated (via `sanitize-commit.py`): `app/rcc/api/*`, `app/rcc/ai/search_index.json` | SDK symbols + AI search index regeneration — expected churn, named here so it isn't scope creep. |

## Architecture & data flow

- **`project.search` / `project.group.get`** (main thread, API command execution): handler →
  `DataModel::ProjectModel::instance()` const accessors → JSON rows → `CommandResponse`.
  Read-only; no signals, no model mutation, no epoch bump. Rows per spec R2; dataset rows add
  `matchedField` only when the hit was alias/units. `groupGet` resolves positional `groupId`
  XOR stable `uniqueId` (two explicit params; both/neither → `InvalidParam` with the two-id-
  spaces hint), returns both ids, and windows its `datasetSummary` with the same
  `offset`/`limit` params (a single group can hold 100+ datasets — an unbounded summary would
  re-create the R5 failure one level down).
- **`meta.search`** (assistant surface only, same availability as `describeCommand` — spec
  constraint honored by construction): `dispatchMetaTool` → `runMetaSearch` →
  `ToolDispatcher::searchCommands`, which merges `assistantToolDefs()` + `fsToolDefs()` +
  `metaToolRoster()` + `API::CommandRegistry::commands()` (skipping `Safety::Blocked`),
  case-insensitive substring on name+description, **sorts by name** (the registry is a QHash —
  unsorted iteration would make `offset` paging incoherent), windows, returns
  `{name, family, snippet}` rows. `family` = text before the first `.`; `snippet` =
  description truncated to ~120 chars.
- **List paging**: `groupsList`/`datasetsList`/`datasetGetExecutionOrder` read `offset`/`limit`
  exactly like `projectSnapshot` (`ProjectHandlerFile.cpp:394-407` idiom), add
  `nextOffset`/total/`attachProjectEpoch`, and a `window: {offset, count, total}` block so a
  windowed reply self-identifies (R4/R9).
- **Assistant default-limit shim**: in `ToolDispatcher::executeCommand`, after safety gating,
  if `name` ∈ {the three list commands} and `args` lacks `limit`, inject the default. TCP/SDK
  callers bypass `ToolDispatcher` entirely and keep today's full-list behavior.

## Hotpath & threading impact

- **Touches the hotpath?** **No.** All code runs in API command handlers / conversation meta
  dispatch on the main thread; only const reads of `ProjectModel`. No `FrameReader` /
  `CircularBuffer` / `FrameBuilder` / `Dashboard` code is touched; no `--benchmark-hotpath`
  delta expected (gate still runs in CI as usual).
- **New cross-thread signal/slot?** No. No new connections at all.
- **New input to a cached hotpath flag?** No.
- **Timestamp ownership** — not applicable; nothing on this path touches frames or stamps.

## Data model & persistence

None. No `Keys::` additions, no schema/writer bump, no project-JSON change, no Sessions DB
change. All new state is request-scoped.

## API / SDK surface

- Two new registered commands (`project.search`, `project.group.get`) — registered in
  `ProjectHandler` alongside the existing list/resolver registrations; **not**
  `BUILD_COMMERCIAL`-gated (discovery is core plumbing, mirrors `project.dataset.list`).
- Three existing commands gain optional params (`offset`, `limit`) — additive, back-compat.
- One new meta tool (`meta.search`) on the assistant surface (roster + makeMetaTool + dispatch;
  meta tools take no safety-JSON entry by design).
- SDK (`app/rcc/api/`) and AI search index regenerate via `scripts/sanitize-commit.py`.
- Safety: both new commands in the `"safe"` tier (read-only, auto-execute — R7). Static test
  pins this.

## QML / UI

None. Headless feature.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Search engine | (a) one-walk collector; (b) epoch-cached index; (c) filter existing list JSON | **(a)** — ~600 items is microseconds per walk; (b) adds a stale-cache bug class for nothing; (c) pays the full-serialization cost the spec exists to kill. |
| Where the bounded default lives | (a) global default limit on the wire; (b) assistant-side injection in `ToolDispatcher::executeCommand`; (c) leave defaults unlimited and rely on docs | **(b)** — (a) silently breaks every existing TCP/SDK client and integration test that expects the full list; (c) fails R5's "in-budget by construction". **Spec-letter note:** R4 says "a default call returns a bounded page" — under (b) this is true from the assistant's seat (the only caller with a byte budget) and false for raw TCP. Flagged for approval as a deliberate scoping of R4, not silent divergence. New commands are bounded-by-default for *all* callers (no compat debt). |
| Impl location | (a) new `ProjectHandlerDiscovery.cpp` TU; (b) grow `ProjectHandlerBatch.cpp` | **(a)** — the god-file split (spec 0002) is per-concern; discovery is a new concern. Registration stays in `ProjectHandler.cpp` per the established pattern. |
| `meta.search` shape | (a) new tool; (b) add `query` substring param to `meta.listCommands` | **(a)** — spec R8 names it, a distinct verb is what an LLM finds when it looks for "search", and its description cross-references `meta.searchDocs` (docs) vs `meta.search` (commands) so the two adjacent names disambiguate each other. `listCommands` behavior untouched. |
| `group.get` selector ambiguity | (a) one param, guess the id space; (b) two mutually exclusive params | **(b)** — positional ids and uniqueIds are both small ints on young projects; guessing recreates the exact confusion this spec fixes. |
| Row field for match provenance | always emit `matchedField` vs only on non-title hits | **Only non-title** — saves ~20 bytes/row on the overwhelmingly common case. |
| Initial paging numbers (Q2) | — | `project.search` 20/100, `meta.search` 25/100, `dataset.list` shim 8/50, `group.list` shim 4/25, `getExecutionOrder` 50/200, `group.get` summary 50/200. **Initial values** — AC5's fixture measurement during `/ss-implement` is the binding check; numbers move if a default page exceeds ~3.5 KB serialized. |

## Risks & mitigations

- **QHash iteration order** (`API::CommandRegistry::commands()`): unsorted paging would return
  incoherent pages. Mitigated: `searchCommands` sorts matches by name before windowing.
- **Group rows are variable-size** (`group.list` rows embed `datasetSummary`): a 4-group page of
  dense groups could still crowd the budget. Mitigated: AC5 measures on the 570-dataset fixture;
  shim default drops if needed (Q2 is explicitly tunable).
- **Elision/truncation string edits may be pinned by tests**: grep `tests/` for `"elided"` /
  `"exceeded the"` before editing; update `test_ai_assistant_static.py` in the same pass (AC9
  makes the *new* strings the pinned ones).
- **Scope creep magnet**: `WorkspacesHandler.cpp:133` (`"groupId" = groupUniqueId`) is the root
  naming collision. **This plan does not touch it** — renaming/aliasing that key changes the
  wire contract for existing clients. Named here as an explicit non-change; if an additive
  `groupUniqueId` alias field is wanted, that is a separate decision (happy to spec it).
- **Docs drift** (per the AI-corpus-drift memory): every new claim in skills/help must match
  registered names exactly; `sanitize-commit.py` rebuilds the search index; the static test
  pins the strings that matter.
- **In-body comment rule / style**: new TU follows the handler-file shape (static free helpers,
  `//---` banners between functions, `[[nodiscard]]`, no in-body comments).

## Test & verification plan

- **Static (I run):** `python scripts/code-verify.py --check` on every touched C++/QML file;
  `python scripts/documentation-verify.py` for the doc edits (AC6);
  `pytest tests/scripts/test_ai_assistant_static.py` (AC7, AC9 string invariants — runnable
  without the app); `qt-cpp-review` before handoff; `sanitize-commit.py` before commit.
- **Integration (maintainer runs, app up with API server on 7777):**
  `tests/integration/test_project_discovery.py`:
  - `test_search_basic` (AC1): fixture, multi-group substring, R2 fields, `matchCount`, a
    returned `path` round-trips through `project.dataset.getByPath`.
  - `test_search_filters_and_types` (AC2): `type` filter; `groupId` filter; group / source /
    workspace / table title hits return correctly typed rows.
  - `test_group_get_both_id_spaces` (AC3): same group via positional and unique id; both ids in
    response; both/neither/unknown → error with the two-id-spaces hint text.
  - `test_list_paging` (AC4): `limit=N` exactness, totals, `projectEpoch`, gap/dup-free walk via
    `nextOffset`, identical repeat pages.
  - `test_default_calls_fit_budget` (AC5): default-parameter responses (with the shim's
    defaults passed explicitly, since pytest speaks raw TCP) serialize under 3.8 KB compact.
  - `test_meta_search` (AC8) runs through the assistant surface where available; the static
    test covers roster/dispatch presence as the runnable fallback.
- **Maintainer observation:** one live assistant session on the large fixture — confirm the
  original failure script ("find the pressure datasets in group X") now resolves in ≤3 tool
  calls, and the three incomplete-result notices read distinctly (AC9's live half).
- **Hotpath:** not touched; no benchmark delta expected (CI gate runs regardless).
