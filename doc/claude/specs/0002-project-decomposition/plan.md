---
spec: 0002-project-decomposition
phase: plan
status: approved      # draft -> approved (gate before /ss-tasks)
updated: 2026-07-06
---

# Plan 0002 - God-class decomposition (ProjectModel / ProjectEditor / ProjectHandler)

> **Phase 2 of 4 - the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this. Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Split each of the three god `.cpp` files into per-responsibility sibling translation units
under a new `app/src/DataModel/Project/` directory, moving whole function bodies verbatim.
The class headers are untouched (S2 alone extracts a private enum block and the `CustomModel`
class into new headers, both pure moves). moc-neutrality (no `.cpp` includes its own
`moc_*.cpp`; the meta-object lives in the untouched header) makes a whole-function move
invisible to moc; the unqualified-lookup trick (out-of-line members resolve helpers in the
enclosing namespace) lets cross-TU file-scope statics become `inline` free helpers in a
shared header with zero call-site edits. S1-S3 are physical-only and land tonight, each a
valid standalone morning state. S4-S5 extract real collaborator objects behind the unchanged
facade and are specified here but deferred. The alternative - a `friend`-heavy single-pass
rewrite, or an interface change to shrink the facade - was rejected: it would break QML's
~60 slot-name bindings and 150+ enum references, and it cannot be verified without a
compiler, whereas a pure move is verifiable by grep symmetry.

## Affected subsystems & files

New directory: `app/src/DataModel/Project/`. All new `.cpp` files carry the SPDX dual-license
banner (copy of `ProjectModel.cpp:1-20`), a verbatim copy of the complete include block of
the origin file, plus the relevant shared header; 98-dash section banners sit *between*
concern groups; `@brief` comments travel with their functions.

| File | Change |
|------|--------|
| `app/src/DataModel/Project/*.cpp` (S1/S2/S3) | New per-responsibility TUs; verbatim moves. |
| `app/src/DataModel/Project/*.h` (S1/S2/S2b) | Shared inline-helper headers + ItemIds enum block + CustomModel class. |
| `app/CMakeLists.txt` | Add new `.cpp` to `SOURCES` (near :308/:309/:256); `CustomModel.h` to `HEADERS` (near :439). |
| `app/src/DataModel/ProjectModel.cpp` | Residual; loses moved bodies. |
| `app/src/DataModel/ProjectEditor.cpp` | Residual; loses moved bodies; gains ItemIds/CustomModel includes. |
| `app/src/API/Handlers/ProjectHandler.cpp` | Residual; loses moved bodies + promoted statics. |
| `app/src/DataModel/Editors/FrameParserModel.cpp` (S2b only) | Add `CustomModel.h` include (only other user). |

Ground truth (validated against the current tree): CMake lists are explicit (no glob).
`SOURCES` at `app/CMakeLists.txt:199` - `ProjectModel.cpp` at :308, `ProjectEditor.cpp` at
:309, `ProjectHandler.cpp` at :256. `HEADERS` at :327 - :439, :440, :382. `CMAKE_AUTOMOC ON`
at :59. Commercial-only lists at :595/:689. `Cpp_JSON_ProjectModel` / `Cpp_JSON_ProjectEditor`
set at `ModuleManager.cpp:693-694`; `qmlRegisterType` at :538-539.

## Stage S1 - TU-split ProjectModel.cpp (TONIGHT)

Header untouched. Whole-function verbatim moves. Every new TU: SPDX banner + copy of the
complete include block `ProjectModel.cpp:22-62` + the shared header.

| New file | Moves (line ranges in original) | ~lines |
|---|---|---|
| `Project/ProjectModelShared.h` | templates `folderExists`, `folderIsSelfOrDescendant`, `sanitizeFolderTree`, `serializeFolders` (:67-154) + inline `nextDuplicateTitle` (:159-194), `seedDefaultFrameParser` (:1503-1516); namespace DataModel, ImporterCommon.h style | ~180 |
| `Project/ProjectModelPersistence.cpp` | :1779-2027 (askSave..serializeToJson), :7639-7868 (autoSave..finalizeProjectSave) | ~700 |
| `Project/ProjectModelLoading.cpp` | :2334-3176 (openJsonFile x2, loadFromJsonDocument, importProjectFromJson, load*, transform-scanner statics :2715-2900, migrateLegacy{LayoutKeys,DashboardLayout,Separator}, emitProjectLoadedSignals, persistLegacyMigration) + :758-905 (seedNextUniqueIdFromGroups, deduplicateUniqueIds, migrateLegacy{WorkspaceRefs,XAxisIds,WaterfallYAxisIds} + static remapWaterfallYAxisId) | ~1,150 |
| `Project/ProjectModelSources.cpp` | :1517-1778 (source CRUD/settings) + :4849-4982 (frame-parser setters) | ~650 |
| `Project/ProjectModelCrud.cpp` | :3366-3591, :3595-4029 (reorder + detail::RefAnchor statics), :4036-4824 (output-widget/group-widget/addDataset + detail::ThreeAxisLayout + populateThreeAxisDatasets statics :275-317), :7877-8055 (stateless id mutators), :8063-8297 (bulk) | ~2,100 |
| `Project/ProjectModelWorkspaces.cpp` | :5055-5224, :5824-6030, :6781-7095, :7122-7480 + statics :196-273, :322-349 (tallyDatasetWidgetTypes, appendDatasetRef, collectGroupDatasetRefs, pushTrackedRef, buildAutoRefsForGroup) | ~1,300 |
| `Project/ProjectModelFolders.cpp` | :6032-6779 (three folder CRUD blocks + folder prompts) + :7097-7121 (sanitize*Folders) | ~800 |
| `Project/ProjectModelTables.cpp` | :5231-5451, :5461-5497 + :5591-5822 (tables/registers + prompts + CSV import/export) | ~650 |
| residual `ProjectModel.cpp` | ctor/singleton, status/lock, getters :725-1512, setupExternalConnections, newJsonFile + scalar setters :2085-2325, selection setters, entity prompts :5499-5590, diagram invokables :7490-7637, clearTransientState, nextDatasetIndex, allocateUniqueId | ~1,900 |

**Partitioning correction (verified):** the "Table folder CRUD" banner range (6536-7482)
also contains the auto-workspace synthesis + hidden-groups machinery (6781-7480). Partition
by *function*, not by banner.

- **CMake:** add 7 `.cpp` to `SOURCES` (near :308); `ProjectModelShared.h` to `HEADERS`
  (near :439).
- **Risk:** Low. Largest single TU (Crud) carries the two `detail::` types - keep both in
  that one TU only.
- **Verification recipe (no compiler):**
  1. Definition symmetry: `grep -cE "DataModel::ProjectModel::"` old vs summed new =
     identical; each header decl defined in exactly one family file.
  2. Static closure: every `^static` / `^template` helper's call sites resolve same-TU or in
     `Shared.h`; only the 6 shared-header helpers may cross TUs.
  3. Include closure: each new TU's includes are a superset of the original's.
  4. Pure-move check: concatenated moved hunks equal deleted hunks modulo removed `static` on
     the 6 promoted helpers (templates keep `template`; non-templates gain `inline`).
  5. detail-namespace ODR: `ThreeAxisLayout` + `RefAnchor` stay in ONE TU (Crud); no
     duplicate type names across TUs.
  6. `grep "\btr(" ProjectModelShared.h` -> zero.
  7. CMake: each new file listed exactly once.

## Stage S2 - TU-split ProjectEditor.cpp + item-id header (+S2b CustomModel) (TONIGHT)

| New file | Moves | ~lines |
|---|---|---|
| `Project/ProjectEditorItemIds.h` | private typedef-enum block `ProjectEditor.cpp:51-273` (TopLevelItem, ProjectItem, kDatasetView_*, kGroupView_*, ...) | ~230 |
| `Project/ProjectEditorShared.h` | inline `folderDisplayPath`, `buildFolderTree`, `accumulateFolderEnabled`, `busTypeIcon` (:168-273; used from >=2 future TUs: :1676/:6053, :5931/:5989/:6065, :1463/:2973) | ~110 |
| `Project/ProjectEditorWiring.cpp` | wire* :281-760 | ~490 |
| `Project/ProjectEditorTree.cpp` | :1310-2021, :2419-2537, :5717-5782 | ~1,000 |
| `Project/ProjectEditorMqtt.cpp` | :1191-1214, :2023-2418 (#ifdef BUILD_COMMERCIAL regions verbatim) | ~450 |
| `Project/ProjectEditorForms.cpp` | :2538-3023, :3205-3443, :3443-4005, :5497-5640 | ~1,700 |
| `Project/ProjectEditorCommit.cpp` | :3024-3204, :4116-4749, :5641-5715 | ~950 |
| `Project/ProjectEditorMultiSelect.cpp` | :4757-5065 | ~310 |
| `Project/ProjectEditorSelection.cpp` | :4088-4114, :5073-5495 | ~470 |
| `Project/ProjectEditorSummaries.cpp` | :5784-6560, :6567-6677 | ~900 |
| residual `ProjectEditor.cpp` | ctor, accessors :765-1290, generateComboBoxModels, transform-editor glue | ~1,300 |

**S2b (optional, low risk):** move `class CustomModel` (`ProjectEditor.h:635-680`) to
`Project/CustomModel.h` (includes `DataModel/ProjectEditor.h` for role enums; `ProjectEditor.h`
already forward-declares `CustomModel` at :40 and uses only pointers). Add the include to
`ProjectEditor.cpp` + `Editors/FrameParserModel.cpp` (the only other user). Add to `HEADERS`
for automoc (it has `Q_OBJECT`). S2b is a prerequisite for S5c.

- **CMake:** add the new `.cpp` to `SOURCES` (near :309); `ItemIds.h`/`Shared.h` are
  non-`Q_OBJECT` (no HEADERS/moc needed); `CustomModel.h` -> `HEADERS` (has `Q_OBJECT`).
- **Risk:** Low-medium. The Q_ENUMs (CANNOT-MOVE #2) and the `wireProjectModelRebuilds()`
  connect topology / QueuedConnection (CANNOT-MOVE #8, `:285-289`) stay put; the ItemIds
  header carries the private enum block so `onDataset*`/`onGroup*` handlers can move
  (CANNOT-MOVE #9).
- **Verification recipe:** S1 recipe, plus
  (a) every `k<View>_*` user file includes the ItemIds header;
  (b) `#ifdef BUILD_COMMERCIAL` open/close balance per TU;
  (c) S2b: exactly one `class CustomModel` definition; `roleNames()` unchanged.

## Stage S3 - TU-split ProjectHandler.cpp + ProjectApiSupport (TONIGHT only if S1/S2 verify clean)

~60 file-scope statics. 14 are verified cross-family -> `ProjectApiSupport.h/.cpp`
(namespace `API::Handlers`, drop `static`): `attachProjectEpoch`, `captureProjectEpoch`,
`appendStaleProjectWarning`, `appendUnknownFieldsWarning`, `buildDatasetObject`,
`datasetOptionsBitflag`, `summarizeProjectJson`, `summarizeCurrentProject`, `takeParam`
(33 sites), `makeScriptEngine`, `detectLanguageMismatch`, `frameParserCompileHint`,
`applySimpleAlarmFields`, `appendDatasetWidgetTypes`. The rest cluster cleanly: snapshot
builders -> File, dataset-field appliers -> Entities, dry-run machinery -> Parser, batch
machinery -> Batch.

Four family TUs:

| New file | Moves | ~lines |
|---|---|---|
| `Handlers/ProjectApiSupport.h/.cpp` | the 14 cross-family statics (namespace API::Handlers, drop `static`) | ~14 helpers |
| `Handlers/ProjectHandlerFile.cpp` | registerFile* + file/snapshot/validate/template + exclusive statics :3683-3860 | ~1,700 |
| `Handlers/ProjectHandlerEntities.cpp` | group/dataset/action/outputWidget + applyDataset*Fields :4773-4930 + member helpers :157-162 | ~1,900 |
| `Handlers/ProjectHandlerParser.cpp` | parser/painter/dry-run + engine statics :5847-6450 | ~1,600 |
| `Handlers/ProjectHandlerBatch.cpp` | batch + list/resolver/move | ~700 |
| residual `ProjectHandler.cpp` | `registerCommands()` stays | remainder |

- **STOP-RULE:** any static with call sites spanning two families and *not* on the 14-list
  -> leave that family unsplit. Partial split is acceptable.
- **Do NOT** rewrite registration as a table (already data-driven; the bulk is irreducible
  doc-string/schema literals). Spec-only for a later run: a `withEpochGuard(...)` wrapper.
- **Risk:** Medium - highest-static-density file; the stop-rule bounds the blast radius.
- **Verification recipe:** per-family S1 recipe, plus the sum of `registry.registerCommand`
  counts across new + residual equals the original, and `BUILD_COMMERCIAL` pairing balances
  per TU.

## Stage S4 (SPEC ONLY) - ProjectModel collaborators

Order and owned state:

- **S4a ProjectFileGuard** - owns `m_fileWatcher`, `m_diskCheckPending`, `m_diskPromptActive`,
  `m_diskFileHash` (~150 ln from :391-397 + :7742-7817). Emits a narrow "disk changed"
  signal; facade re-emits under existing NOTIFY names.
- **S4b AutoSaveController** - owns `m_autoSaveTimer`, `m_autoSaveSuspended` (~90 ln
  :7639-7708). `autoSave()` / `syncRuntime()` / `m_runtimeDirty` STAY in the facade
  (CANNOT-MOVE #4 race area). Depends on S4a landing first.
- **S4c WorkspaceSynthesizer** - pure functions; no owned mutable state.
- **S4d LegacyMigrations** - free functions.
- **S4e ProjectUiStateStore** - owns the UI-state cluster (~350 ln :1165-1512).
- **ProjectSerializer/Loader** (tail, ~1,100 ln) - needs the `ProjectDocument` aggregate
  decision (see spec Open Questions); `friend class` is the cheaper interim.

**Signal strategy (all S4):** each collaborator emits a narrow signal; the facade ctor
connects it to the existing NOTIFY names - zero QML changes. **Depends on:** S1. S4a unlocks
S4b.

## Stage S5 (SPEC ONLY) - ProjectEditor collaborators

- **S5a ComboBoxCatalog** (~250 ln).
- **S5b ProjectTreeController** (~2,100 ln) - tree + selection + expansion move together;
  keep the `QueuedConnection` verbatim (CANNOT-MOVE #8).
- **S5c per-entity FormControllers** (~3,000 ln) - each takes a `CustomModel*`, builds rows,
  and handles `on*ItemChanged` commit. Requires S2b `CustomModel.h`.
- **S5d MultiSelectionController** (~320 ln).

**Invariant restated per controller:** the title-edit rule - in-place item update, never a
per-keystroke model mutation. **Depends on:** S2. S2b is a prerequisite for S5c.

## Architecture & data flow

S1-S3 change no data flow: the same objects, signals, slots, and threads exist; only the file
a definition lives in changes. The project subsystem's control flow (ProjectModel = data,
ProjectEditor = controller with the tree model + 4 form models + selection, all
`buildTreeModel`/`groupsChanged` connections `QueuedConnection`) is preserved exactly. S4-S5
introduce collaborator objects that own a slice of state and emit a narrow signal the facade
re-broadcasts under its existing NOTIFY names, so the QML/API view of the world is unchanged.

## Hotpath & threading impact

- **Touches the hotpath?** No. `ProjectModel`/`ProjectEditor`/`ProjectHandler` are the
  project-editing and API-command subsystem, not the `FrameReader` -> `FrameBuilder` ->
  `Dashboard` data path. No span fast-lane, no cached hotpath flag, no slot pool involved.
- **New cross-thread signal/slot?** No. S1-S3 add none. The existing `QueuedConnection`
  (`ProjectEditor.cpp:285-289`) and `DirectConnection` contracts are moved verbatim, never
  re-typed. S4-S5 collaborators are constructed and wired on the same (GUI) thread as the
  facade; their narrow signals use the same connection types the moved bodies already used.
- **New input to a cached hotpath flag?** No.
- **Timestamp ownership** - unaffected; the source still stamps at the driver boundary.

## Data model & persistence

No `Frame.h` `Keys::` additions, no schema/writer version bump, no project-JSON shape change,
no Sessions DB change. The persistence *code* moves TU (S1 `ProjectModelPersistence.cpp`,
S4-tail serializer) but reads/writes byte-identical output. The watcher re-arm invariant
(`watchProjectFile()` after write/load/new, `ProjectModel.cpp:7735`) is preserved.

## API / SDK surface

No API surface change. `registerCommands()` stays in the residual `ProjectHandler.cpp`; the
command table, doc-strings, and schemas are byte-identical. The 14 cross-family statics move
to `ProjectApiSupport` (namespace `API::Handlers`) as non-`static` helpers, resolved by
unqualified lookup - no registration or handler-signature change. `BUILD_COMMERCIAL` regions
stay inline behind `#ifdef`.

## QML / UI

No QML change. The facade slot names (~60 `Cpp_JSON_ProjectModel.*`) and enums (150+
`ProjectEditor.*`) are byte-identical. `ModuleManager.cpp:693-694` / :538-539 registrations
are untouched. The S2 ItemIds extraction moves a *private* enum block (not a Q_ENUM); the
public Q_ENUMs (CANNOT-MOVE #2) stay in the header.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Split granularity | Physical TU split vs interface-shrinking rewrite | Physical split - verifiable by grep symmetry, zero QML/API risk, each stage a valid morning state. A rewrite would break ~60 slot + 150+ enum bindings and needs a compiler to verify. |
| Cross-TU helpers | Anonymous namespace vs `inline` free fns in a shared header | Shared header (ImporterCommon.h precedent) - the linter flags anon namespaces; unqualified lookup finds enclosing-namespace helpers with zero call-site edits. |
| ProjectHandler statics | Move all vs stop-rule partial split | Stop-rule - any off-list static spanning two families leaves that family unsplit; bounds risk on the highest-static-density file. |
| Serializer/Loader boundary (S4 tail) | `ProjectDocument` aggregate vs `friend class` | Deferred (Open Question); `friend class` is the cheaper interim, aggregate is the cleaner end state. |
| Registration format (S3) | Keep as-is vs table-driven rewrite | Keep as-is - already data-driven; bulk is irreducible schema/doc literals. |

## Risks & mitigations

- **Lost or duplicated definition** - caught by the definition-symmetry grep (AC1) and the
  pure-move diff (AC4).
- **ODR violation from duplicated `detail::` types** - `ThreeAxisLayout` / `RefAnchor` pinned
  to the Crud TU only; ODR grep in the recipe.
- **Reordered ctor side effects** (CANNOT-MOVE #3, fenced :445-450) - the ctor stays in the
  residual facade TU untouched; no ctor line moves.
- **Connection-type drift** (QueuedConnection :285-289, CANNOT-MOVE #8) - connect topology
  stays in its facade TU; only bodies move, never the `connect(...)` shape.
- **Watcher re-arm dropped** - `watchProjectFile()` calls travel with their functions; include
  closure ensures the symbol is visible.
- **BUILD_COMMERCIAL imbalance** - per-TU `#ifdef` open/close balance check.
- **CMake double-registration** - each new file exactly-once check.
- **Silent-breakage classes from `common-mistakes.md`** - this change touches none of the
  cached-hotpath-flag or timestamp-capture classes (it is off the hotpath entirely); the
  relevant class here is scope creep, mitigated by the pure-move discipline (no edits beyond
  moves).

## Test & verification plan

- **Unit (agent can run):** none in `tests/scripts/` (no parser-script logic changes). The
  primary agent-runnable checks are the per-stage grep recipes above (definition symmetry,
  static/include/ODR closure, pure-move diff, BUILD_COMMERCIAL pairing, CMake exactly-once).
- **Integration / security / perf (maintainer runs):** the existing project-editor and API
  handler `pytest tests/integration/` suites should pass unchanged with the app up and the
  API server enabled - a regression there means a body was altered, not moved. No new test is
  required (behavior is unchanged by construction).
- **Hotpath:** not touched; `--benchmark-hotpath` need not be re-run for correctness, though
  the maintainer may spot-check it is not regressed by the added TUs.
- **Static:** `python scripts/code-verify.py --check <new files>` on every new TU/header;
  `qt-cpp-review` before handoff; `python scripts/sanitize-commit.py` before commit.
- **Build (maintainer, the one check the agent cannot run):** Pro and non-Pro configurations
  compile with no new warnings; the running app behaves identically (AC5).
