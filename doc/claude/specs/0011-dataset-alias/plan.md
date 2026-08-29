---
spec: 0011-dataset-alias
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-15
---

# Plan 0011 — Dataset aliases for script and API dataset lookup

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

`Dataset` gains an optional persisted `alias` string (empty = none, `color`-pattern
serialization). Resolution lives where the values already live: `DataTableStore::initialize()`
— which is rebuilt from the template frame on every project sync, so renames propagate through
the existing epoch-apply/autosave rebuild with no new wiring — builds an alias → (raw, final)
slot map next to the existing `m_datasetIndex`, and additionally mirrors each alias as
lookup-only `raw:<alias>` / `final:<alias>` keys in the register name index (no extra storage,
not in `snapshot()`), which makes aliases work for free in `tableGet`/`tableHandle`, control
scripts, and `project.dataTable.*`. The two script entry points grow a string branch:
the Lua C closures type-switch on `lua_type()` and resolve via an interned-pointer alias cache
(same pattern as `getByInternedKey`, zero steady-state allocation), and the JS
`TableApiBridge::datasetGetRaw/Final` parameter changes from `int` to `const QJSValue&` so a
JS string can never coerce to a number. API handlers that address a dataset by uniqueId
(`project.dataset.getByUniqueId`, `project.dataset.move`, `dashboard.tailFrames`, the
assistant resolvers) route through one new shared selector helper in `ProjectApiSupport.h`
that accepts number-or-alias-string. The editor gets an Alias `TextField` row with a
uniqueness guard at commit and a "seed aliases from titles" bulk action using the
multi-select batch idiom.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/DataModel/Frame.h` | `Dataset::alias` field (:438 block); `Keys::Alias` (:96 block); optional serialize following the `color` pattern (:1133) |
| `app/src/DataModel/Frame.cpp` | Deserialize `d.alias = ss_jsr(obj, Keys::Alias, "").toString().simplified()` in `read(Dataset&, ...)` (:291) |
| `app/src/DataModel/DataTable.h` | `m_aliasIndex` (`QHash<QString, QPair<int,int>>`), alias-read accessors (`getDatasetRawByAlias`/`getDatasetFinalByAlias`, `QString` + interned `const char*` forms), `m_aliasCache` interned-pointer cache, `m_warnedMissingAliases`; `TableApiBridge::datasetGetRaw/Final` signature → `const QJSValue&` |
| `app/src/DataModel/DataTable.cpp` | Build `m_aliasIndex` + register-name mirrors in `initialize()` (:117 loop); accessors + `noteMissingAlias`; clear alias cache in `clearLookupCache()`; QJSValue type-branch in the bridge methods |
| `app/src/DataModel/FrameBuilder.cpp` | `luaDatasetGetRaw`/`luaDatasetGetFinal` (:2642/:2668): `lua_type()` switch, string → interned alias accessor |
| `app/src/DataModel/Project/ProjectEditorItemIds.h` | `kDatasetView_Alias` enum value |
| `app/src/DataModel/Project/ProjectEditorForms.cpp` | Alias `TextField` row in the dataset General Information form (next to units :904) |
| `app/src/DataModel/Project/ProjectEditorCommit.cpp` | Commit case in `onDatasetCommonItemChanged` (:553 block) + uniqueness guard in `onDatasetItemChanged` (~:836) rejecting duplicates before `pm.updateDataset` |
| `app/src/DataModel/Project/ProjectEditorMultiSelect.cpp` | Exclude the alias row from the multi-selection aggregate form (a shared alias is never valid) |
| `app/src/DataModel/ProjectModel.h/.cpp` (or `Project/ProjectModelCrud.cpp`) | `seedDatasetAliases()` bulk mutator: fill empty aliases from `title.simplified()`, dedupe with `-2`/`-3` suffixes, batch-applied via the `setAutoSaveSuspended`/`m_batchApplying` idiom |
| `app/qml/ProjectEditor/Sections/ProjectStructure.qml` | Context-menu entry (project root) invoking the bulk seed action |
| `app/src/API/Handlers/ProjectApiSupport.h` | New shared selector: resolve a `QJsonValue` that is a number (uniqueId) or string (alias) to a dataset, with an error message that names the unresolved alias |
| `app/src/API/Handlers/ProjectHandlerBatch.cpp` | `datasetGetByUniqueId` (:204) and `datasetMove` (:363) use the shared selector |
| `app/src/API/Handlers/ProjectHandler.cpp` | `applyDatasetTextAndToggleFields` (:1420): accept/validate `alias` in `project.dataset.update` (uniqueness check, same rules as editor); command doc strings (:1175, :962) advertise the alias form |
| `app/src/API/Handlers/DashboardHandler.cpp` | `dashboard.tailFrames` `uniqueIds` filter (:474): string array elements resolve as aliases |
| `app/src/API/Handlers/AssistantHandler.cpp` | `datasetResolve` (:105) + `resolveAddTileDataset` (:508): string `uniqueId` treated as alias via the selector |
| `app/src/AI/ToolDispatcher.cpp` | Tool schemas for `uniqueId` params become integer-or-string (:139, :292); `resolveDataset` (:773) / `scriptTargetDataset` (:1077) pass strings through |
| `app/rcc/api/prelude.js` / `prelude.lua` | Doc comments: `datasetGetRaw(uniqueIdOrAlias)` form (bodies unchanged — pass-through) |
| `app/rcc/api/SerialStudio.js` / `.lua` / `sdk-symbols.json` | Regenerated by `scripts/generate-sdk.py` (via sanitize-commit) |
| `app/rcc/api/api-schema.json` | Regenerated by the maintainer via `--dump-api-schema` after handler changes (Claude cannot run the app) |
| `doc/help/SerialStudio-SDK.md`, `Dataset-Transforms.md`, `Data-Tables.md`, `Identity-Model.md`, `API-Reference.md` | Document the alias parameter form and the alias identity (per the audited mention list) |
| `app/rcc/ai/docs/*.md`, `app/rcc/ai/skills/*.md` (12 files mentioning `datasetGetRaw/Final`) | Corpus mention updates; `search_index.json` rebuilt by `build_search_index.py` (sanitize-commit runs it) |
| `tests/scripts/conftest.py` + `tests/scripts/test_table_handles.py` (or new file) | Extend `TABLE_API_SHIM` with alias-aware `datasetGetRaw/Final`; JS unit for numeric-vs-alias equivalence |
| `tests/integration/` (new `test_dataset_alias.py`) | API acceptance: uniqueId vs alias equivalence, unknown-alias error (maintainer runs) |

Out of lane, named here: translation catalogs (`.ts`/`.qm`) are NOT touched — new tr()
strings land in the catalogs when the maintainer regenerates them.

## Architecture & data flow

Cold path (project load / sync / editor apply): `ProjectModel` → `FrameBuilder::
initializeTableStore()` / `refreshTableStoreFromProjectModel()` → `DataTableStore::
initialize()` walks the template frame's groups/datasets exactly as today; for each dataset
with a non-empty alias it (a) inserts alias → `{rawSlot, finalSlot}` into `m_aliasIndex`, and
(b) inserts `("__datasets__", "raw:<alias>")` / `("final:<alias>")` into `m_index` pointing at
the *existing* slots — skipped with a one-shot warning if the key is already taken (e.g. an
all-digit alias shadowing a uid register name). `m_tableRegNames` is not extended, so
`snapshot()` and table exports are unchanged. Alias renames ride the existing
epoch-gated `syncFromProjectModel` → store rebuild (generation bump) → script handle
re-resolution lifecycle (see `project_api_mutation_apply_path` memory + architecture/project);
no new signals.

Hot path (per script call): Lua string arg → interned-pointer cache probe (array scan of ≤16
entries keyed by Lua's interned `const char*`) → hit: direct slot read via `captureRead` +
`m_storage`; miss: one `QString::fromUtf8` + `m_aliasIndex` lookup, result (found or -1)
cached. JS string arg → `QJSValue::toString()` + `m_aliasIndex` lookup (JS marshalling
already allocates; the JS tiers gate at 64–128 kHz). Numeric args in both languages follow
the byte-identical existing path. Reads still land on the same slots, so read-set capture
(`captureRead`) and change-driven transform skips work unchanged for alias lookups.

API path: handlers resolve string selectors against `ProjectModel::groups()` with a linear
scan in the shared helper (GUI thread, cold, mirrors the existing inline uid loops), then
proceed exactly as if the resolved uniqueId had been passed (spec's safety invariant).

## Hotpath & threading impact

- **Touches the hotpath?** Yes — the `datasetGetRaw`/`datasetGetFinal` call sites run inside
  frame parsers and per-dataset transforms at frame rate. Preservation: the numeric path is
  unchanged except for one integer type-check (`lua_type()` / `QJSValue::isNumber()`) ahead
  of it; the alias path adds no steady-state allocation on Lua (interned cache, mirroring
  the proven `getByInternedKey` pattern) and only the inherent marshalling on JS; the alias
  map is built in `initialize()` (cold). No changes to the span fast lane, frame pool,
  publish sites, or `FrameReader`/`CircularBuffer`. `ss-hotpath` invoked; benchmark plan
  below (benchmark projects carry no aliases, so the gates also prove the unused-cost-zero
  claim).
- **New cross-thread signal/slot?** No. Control scripts keep reaching dataset values through
  the already-marshalled `project.dataTable.getValue` (which now accepts `raw:<alias>` names
  via the register mirror) — no direct bridge is added to the worker engine.
- **New input to a cached hotpath flag?** No. `m_externalTableApiUsers` / capture-flag
  refresh timing is untouched; alias data rides the store rebuild that already sets
  `m_captureFlagsDirty`.
- **Timestamp ownership** — untouched; no frame construction or stamping anywhere in this
  change.

## Data model & persistence

- `Keys::Alias = "alias"` in `Frame.h` (grep-confirmed unused today).
- Serialize only when non-empty (`color` pattern) → pre-alias projects round-trip
  byte-identical (spec AC8); older builds ignore unknown keys on read (schema-compat
  constraint).
- Load applies `.simplified()`; the editor commit path applies the same normalization, so
  the persisted form is always trimmed/collapsed. No writer/schema version bump needed.
- uniqueId lifecycle untouched: the loader's backfill/dedup passes don't interact with
  aliases; alias uniqueness is enforced at the two write surfaces (editor commit, API
  `project.dataset.update`), and `initialize()` is defensive (first-wins + warn) if a
  hand-edited project file smuggles a duplicate in.

## API / SDK surface

- Shared selector in `ProjectApiSupport.h`: number → uniqueId; string → alias scan; error
  text names the alias ("Dataset not found for alias 'X'"). Used by
  `project.dataset.getByUniqueId`, `project.dataset.move`, `dashboard.tailFrames`
  (per-element), `assistant.dataset.resolve`, `assistant.workspace.addTile`.
- `project.dataset.update` learns the `alias` field (whitelist in
  `applyDatasetTextAndToggleFields` + `consumed` set — without this the param is silently
  dropped by `appendUnknownFieldsWarning`), with uniqueness validation returning an error
  string like the `color` validity check.
- ToolDispatcher schemas: `uniqueId` properties become `["integer","string"]` unions with
  description text explaining the alias form; no new tool, so no
  dual-registration/safety-JSON work (per the `assistant.*` dual-registration memory, that
  applies to *new* tools only).
- No Pro gating — GPL + Commercial identical.
- `datasetGetRaw`/`datasetGetFinal` are prelude/bridge functions, not apiCall commands: the
  SDK change is doc-comment-only in the preludes, then `generate-sdk.py` regenerates the
  bundles; `api-schema.json` refresh needs the maintainer's `--dump-api-schema` run.

## QML / UI

- Dataset form: new Alias `TextField` row — rendered by the existing generic
  `TableDelegate`, zero QML changes for the row itself.
- Rejection UX for duplicate aliases: commit guard restores the previous value via the
  existing `syncDatasetItemCache` path and surfaces the reason (exact surface — message box
  vs. inline — pinned at task time to whatever the editor's current validation idiom is).
- All-digit aliases: accepted but warn once at commit (spec open question resolved: warn,
  don't block); the register mirror independently guards against uid-register collisions.
- Bulk seed: context-menu entry on the project root in `ProjectStructure.qml` calling a new
  `ProjectModel` slot; batch idiom from `ProjectEditorMultiSelect.cpp:278` (suspend
  autosave, `m_batchApplying`, one `flushAutoSave()`).

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Where resolution lives | **Store-native** (alias map in `DataTableStore::initialize()`); Prelude-shim (alias→uid dict injected per engine); ProjectModel-central (bridges query the singleton) | **Store-native** — the store already rebuilds from the template frame on every sync (renames propagate for free), keeps hotpath reads inside one object, and serves Lua/JS/API uniformly. Prelude-shim duplicates state per engine, misses the API, and stales mid-session; ProjectModel-central couples hotpath reads to a cross-singleton call. |
| Register-name mirror (`raw:<alias>`) | Mirror in `m_index` (lookup-only); no mirror (aliases only in `datasetGetRaw/Final`) | **Mirror** — two hash entries per aliased dataset buy alias access for control scripts, `tableGet`/handles, and `project.dataTable.*` with zero storage and zero hotpath cost; without it, control scripts (which have no `datasetGetRaw`) get no alias support at all. Collision with existing names skips + warns. |
| JS bridge signature | `QJSValue` param (type-branch); `int`/`QString` Q_INVOKABLE overloads | **QJSValue** — overload resolution can coerce `"128"` to int, violating the spec's strict string=alias rule; a single QJSValue branch is exact. |
| Lua type test | `lua_type() == LUA_TSTRING`; `lua_isnumber()` | **`lua_type()`** — `lua_isnumber` is true for numeric strings, which would silently break the same rule. |
| API selector | Shared helper in `ProjectApiSupport.h`; patch each inline loop | **Shared helper** — three call sites today plus the assistant resolvers; one helper keeps the error shape (naming the alias) consistent and future commands honest. |
| Bulk-seed sanitization | Trim+collapse (`simplified()`), keep case/charset; aggressive slugging (spaces→`-`) | **`simplified()` only** — spec invariant is "deterministic, unique, non-empty"; titles like `ATAM1-CH1` survive verbatim, and users who want slug-style aliases can type them. Dedupe: `-2`, `-3`... suffixes. |

## Risks & mitigations

- **Silent hotpath regression** — the one exposed class from `common-mistakes.md` is
  per-frame lookups creeping into the transform loop; mitigated by the interned cache +
  unchanged numeric path, and gated by `--benchmark-hotpath` (all nine tiers).
- **`QHash::insert` overwrite on the register mirror** — an alias colliding with
  `raw:<uid>`/`final:<uid>` (or another register name) would silently *replace* the index
  entry; `initialize()` must probe `m_index` before inserting and skip+warn (called out in
  the loop design, and the all-digit editor warning shrinks the surface).
- **`operator[]` insert on lookup** — all alias reads use `constFind` (the
  `common-mistakes.md` map-insert class).
- **API param silently dropped** — `project.dataset.update` whitelist + `consumed` set is
  the known trap (agent-confirmed at ProjectHandler.cpp:1685); the task list will pin it.
- **Editor multi-select fan-out** — without the exclusion, multi-editing datasets would
  assign one alias to N datasets and trip the uniqueness guard N-1 times; excluded from the
  aggregate form up front.
- **Docs drift** — 12 assistant-corpus files + 5 help pages mention the functions; the
  sweep list is enumerated above so none is missed, and `documentation-verify.py` +
  search-index rebuild run in sanitize-commit.

## Test & verification plan

- **Unit (Claude can run):** extend `tests/scripts/` shim with alias-aware
  `datasetGetRaw/Final`; cases: alias == uid result equivalence (AC1, JS half),
  `"128"` ≠ `128` discrimination (R3), unknown alias → `undefined` (AC4 shape).
- **Integration (maintainer runs, app up with API server):** new
  `tests/integration/test_dataset_alias.py` — `project.dataset.getByUniqueId` with numeric
  vs alias selector equivalence, unknown alias error naming the alias (AC5);
  `project.dataset.update` setting/rejecting duplicate aliases (R2/AC2 API half);
  `dashboard.tailFrames` alias filter.
- **In-app observations (maintainer):** editor alias row + duplicate rejection (AC2), alias
  survives restructuring (AC3), Lua transform alias read + one-shot warning (AC1 Lua half,
  AC4), bulk seed on a duplicate-title project (AC6), pre-alias project round-trip (AC8).
- **Hotpath:** maintainer/CI `--benchmark-hotpath --min-fps 256000` — all nine gates (AC7).
- **Static:** `python scripts/code-verify.py --check` on every touched C++/QML file;
  `qt-cpp-review` before handoff; `python scripts/sanitize-commit.py` before commit
  (regenerates SDK bundles + search index + doc lint).
