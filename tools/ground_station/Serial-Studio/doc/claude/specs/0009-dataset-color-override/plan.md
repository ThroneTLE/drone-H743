---
spec: 0009-dataset-color-override
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-14
---

# Plan 0009 — Per-dataset color override in the Project Editor

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this — a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Add an optional `QString color` member to `DataModel::Dataset` (`Frame.h`), mirroring the
existing `AlarmBand::color` convention exactly: empty string = automatic, serialized under
the existing `Keys::Color` key only when non-empty, read back with `ss_jsr`. Resolution
happens at render time through a new override-aware `SerialStudio::getDatasetColor(const
DataModel::Dataset&)` overload — non-empty and valid `QColor` wins, otherwise today's
index-based palette lookup — dropped into the five C++ consumers and the output-panel
accent. The Project Editor gains one `ColorPicker` row in the dataset form (new
`EditorWidget` enum value + one QML `Loader` delegate wrapping a `ColorDialog`, mirroring
the `IconPicker` delegate and the `AlarmBandsEditor` dialog pattern), and the project API
gains a `color` patch field on `project.dataset.update`. No new signals, no schema bump,
no hotpath work: edits ride the existing `updateDataset → syncRuntime →
syncFromProjectModel → invalidateFramePool → reconfigure → widgetCountChanged` pipeline.

## Affected subsystems & files

All touch-points below were confirmed by direct reads/greps this session.

| File | Change |
|------|--------|
| `app/src/DataModel/Frame.h` | `QString color;` on `Dataset` (with the other QString members, ~:441); guarded `obj.insert(Keys::Color, d.color)` in `serialize(Dataset)` (~:1150, `AlarmBand` pattern from :1072) |
| `app/src/DataModel/Frame.cpp` | `d.color = ss_jsr(obj, Keys::Color, "").toString().simplified();` in `read(Dataset&, ...)` (~:315, mirrors band read at :230) |
| `app/src/SerialStudio.h` / `.cpp` | New C++-only `[[nodiscard]] static QColor getDatasetColor(const DataModel::Dataset&)` next to the int overload (SerialStudio.cpp:745): valid non-empty override wins, else `getDatasetColor(d.index)`; invalid strings fall back to auto |
| `app/src/UI/DashboardWidget.cpp` | `widgetColor()` (:135-145) resolves via the new overload — covers Plot/FFT/Bar/Gauge/Meter/etc. accents and QML `widgetColor` consumers |
| `app/src/UI/Widgets/MultiPlot.cpp` | Per-curve colors (:895 loop) via the new overload |
| `app/src/UI/Widgets/LEDPanel.cpp` | Per-LED dataset color (:269) via the new overload; feeds the existing `resolveBandColor` fallback (:242) |
| `app/src/UI/Widgets/GPS.cpp` | Trail color (:647): first non-empty member override in the group, else today's `getDatasetColor(m_index + 1)` |
| `app/src/UI/Widgets/Plot3D.cpp` | Curve color (:588): same group-level resolution as GPS |
| ~~`app/src/UI/Widgets/Output/Panel.h` / `.cpp`~~ | **Dropped mid-build (2026-07-14):** output panels iterate `DataModel::OutputWidget` entries, not `Dataset` objects — there is no dataset (and no `color` field) to resolve. The positional accent stays as-is; per-control colors would be an `OutputWidget` feature outside this spec. |
| ~~`app/qml/Widgets/Dashboard/Output/DashboardOutputPanel.qml`~~ | Dropped with the row above — `accentColor` keeps its positional `getDatasetColor(index + 1)`. |
| `app/src/DataModel/ProjectEditor.h` | `ColorPicker` appended to `EditorWidget` (:226, `Q_ENUM` — QML sees `ProjectEditor.ColorPicker` automatically) |
| `app/src/DataModel/Project/ProjectEditorItemIds.h` | `kDatasetView_Color` in `DatasetItem` (:45-73) |
| `app/src/DataModel/Project/ProjectEditorForms.cpp` | "Widget color" row in `addGeneralSection` (:840, standard role pattern; `EditableValue` = color string, empty = automatic) |
| `app/src/DataModel/Project/ProjectEditorCommit.cpp` | `kDatasetView_Color` case in `onDatasetCommonItemChanged` (:540-562); non-tree field, `rebuildTree` stays false |
| `app/qml/ProjectEditor/Views/TableDelegate.qml` | New `ColorPicker` `Loader` (swatch + "Automatic" state + `ColorDialog` + reset-to-auto), mirroring the `IconPicker` loader (:502-566) and `AlarmBandsEditor.qml`'s `ColorDialog`/`hexFromColor` (:287-298) |
| `app/src/API/Handlers/ProjectHandlerEntities.cpp` | `takeParam` block for `Keys::Color` in `applyDatasetTextAndToggleFields` (:1418-1451); empty string clears; no `rebuildTree` |
| `app/src/API/Handlers/ProjectHandler.cpp` | `project.dataset.update` doc-string (:1176-1208) lists `color` ("#rrggbb, empty = automatic") |
| `tests/integration/test_project_editor.py` | Color round-trip via `project.dataset.update` + save/load; pre-feature project loads with empty color |
| Regen (commit-time) | `api-schema.json` re-dump (`--dump-api-schema`, maintainer runs the binary) + `scripts/generate-sdk.py` + search index — all via `sanitize-commit.py` |

Out of lane, named here deliberately: `app/src/API/GRPC/ConversionUtils.cpp` hand-enumerates
dataset fields (:181-189) and will NOT carry `color` — it serializes telemetry values, not
presentation. The shared `DataModel::serialize(Dataset)` is used by the JSON API/MQTT frame
broadcasts, so those gain the guarded `color` key automatically when set (harmless,
presentation metadata). `app/rcc/ai/*` files have uncommitted foreign modifications in the
working tree — this work will not touch them; corpus/search-index regen happens at
sanitize/commit time under the maintainer's control.

## Architecture & data flow

**Edit flow (editor)**: `TableDelegate` ColorPicker writes `EditableValue` →
`ProjectEditor::onDatasetItemChanged` (`ProjectEditorCommit.cpp:804`) → new
`kDatasetView_Color` case sets `m_selectedDataset.color` →
`ProjectModel::updateDataset` (`ProjectModelCrud.cpp:135`) assigns the whole struct, sets
`m_runtimeDirty`, `setModified(true)`, `scheduleAutoSave()` → debounced autosave flush calls
`syncRuntime()` → `FrameBuilder::syncFromProjectModel()` rebuilds `m_frame` and calls
`invalidateFramePool()` (FrameBuilder.cpp:605) → next frame's `preparePooledSlot` sees the
generation mismatch and full-assigns the slot (color included) → `Dashboard::hotpathRxFrame`
reconfigures and `reconfigureDashboard` ends with an unconditional
`Q_EMIT widgetCountChanged()` (Dashboard.cpp:1810) → QML widget delegates rebuild and re-read
colors. Identical to how a title/units edit propagates today; no new wiring.

**Edit flow (API)**: `project.dataset.update {color}` → `datasetUpdate`
(`ProjectHandlerEntities.cpp:1624`) → `applyDatasetTextAndToggleFields` → same
`updateDataset` chain as above.

**Render flow**: every consumer holds (or can fetch) a `Dataset&` — `DashboardWidget` via
`GET_DATASET`, MultiPlot/LEDPanel via `group.datasets[i]`; GPS/Plot3D are group widgets and
scan their group's members. Resolution is one function:
`getDatasetColor(dataset)` = `QColor(dataset.color)` when non-empty and valid, else
`getDatasetColor(dataset.index)`. Theme switches keep firing the existing
`themeChanged → widgetColorChanged` connections; an overridden dataset re-resolves to the
same fixed color (R4 holds by construction), automatic datasets follow the palette.

**Persistence**: `serialize(Group)` already loops datasets (Frame.h:1159), and the save path
(`ProjectModelPersistence.cpp:266`) / load path (`ProjectModelLoading.cpp:510`) are
transitive — the field travels with zero loader changes. Absent key reads back as empty =
automatic (R5). `Keys::Color` (Frame.h:115) is reused at dataset-object scope; the AlarmBand
use lives in a different JSON scope, so there is no collision and no new key constant.

## Hotpath & threading impact

- **Touches the hotpath?** Only by enlarging a struct the pool copies on *structure changes*.
  Verified this session by reading the pool code: steady-state frames go through
  `copy_frame_values` (Frame.h:924-946), which copies **only** `value`/`rawValue`/
  `numericValue`/`isNumeric`/`rawNumericValue` — `color` is deliberately not added there
  (it is structure, not per-frame state). The span fast lane (`applyDatasetValuesSpans`)
  writes value strings in place through pre-resolved `Dataset*` and never copies structs.
  The only paths that copy `color` are `preparePooledSlot`'s full assign on generation
  mismatch (FrameBuilder.cpp:309, fires once per structure change) and the pool-exhausted
  heap fallback (:325). `sizeof(Dataset)` grows by one QString (24 bytes on 64-bit —
  `QArrayDataPointer` is three words; corrected from the draft's "8" during review;
  8-aligned, the `static_assert` at Frame.h:447 holds). Steady-state per-frame cost: zero.
  `--benchmark-hotpath` gates verify (AC7); no FrameReader/CircularBuffer/Dashboard-ingest
  code is edited.
- **New cross-thread signal/slot?** None. No new connections at all; propagation reuses the
  existing project-sync pipeline.
- **New input to a cached hotpath flag?** None. Color is never read on the parse/publish
  path — only in render-time getters and reconfigure.
- **Timestamp ownership** — untouched; nothing on the stamping path changes.

## Data model & persistence

- `Dataset::color` (QString, default empty = automatic). Doxygen mirrors AlarmBand:
  `///< Optional hex override; empty -> automatic (theme palette by index)`.
- JSON: reuse `Keys::Color` at dataset scope; write-only-when-set (`serialize(Dataset)`),
  `ss_jsr(..., "")` + `.simplified()` on read. No `kSchemaVersion` bump (currently 3), no
  legacy alias — optional additive field, same class as `Waterfall`/`Disabled`/
  `HideOnDashboard`.
- Editor always writes `#rrggbb` (from `ColorDialog`); API accepts any string but the
  resolver guards with `QColor::isValid()`, so garbage degrades to automatic instead of
  rendering black.
- No widgetSettings, no Sessions DB, no example `.ssproj` changes.

## API / SDK surface

- `project.dataset.update` gains optional `color` (string, `#rrggbb`, empty clears →
  automatic): one `takeParam(params, consumed, Keys::Color)` block in
  `applyDatasetTextAndToggleFields` + doc-string mention. Unknown-field warning machinery
  (`appendUnknownFieldsWarning`) needs nothing — the param becomes known.
- `datasetAdd` paths need nothing: a new dataset default-constructs with empty color.
- Free feature — no `BUILD_COMMERCIAL` gating anywhere (maintainer-confirmed in spec).
- Regen after the param lands: `api-schema.json` dump (maintainer runs the app with
  `--dump-api-schema`), then `generate-sdk.py` — both part of the normal
  `sanitize-commit.py` flow.

## QML / UI

- `EditorWidget::ColorPicker` + one `Loader` block in `TableDelegate.qml` gated on
  `model.widgetType === ProjectEditor.ColorPicker`, mirroring the IconPicker loader shape:
  a swatch button showing the current color (or an "Automatic" label with the computed
  palette color when empty), opening one shared `ColorDialog` (the `AlarmBandsEditor`
  `hexFromColor` pattern), plus a clear-to-automatic button. Writes commit through the
  standard `EditableValue` role — no ComboBox restore-race exposure, no new model.
- The dataset form row lives in `addGeneralSection` (visible for every widget type,
  including output-group datasets, which use the same form).
- Dashboard QML is untouched except `DashboardOutputPanel.qml`'s `accentColor` source swap.
- Theme reactivity: unchanged connections; glass/theme surfaces not involved.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Storage | **Struct field on `Dataset`** / widgetSettings sidecar keyed by uniqueId / palette-slot index | Struct field — it is the `AlarmBand::color` precedent verbatim and reaches editor, API, persistence, and all render sites through pipelines that already carry the struct; sidecar would need bespoke plumbing at every one of those seams; palette-slot was rejected by spec R4. |
| Group widgets (GPS, Plot3D) | **First non-empty member override, else today's positional auto** / leave auto-only | These widgets render one color for a whole group and today key it off widget position (`m_index + 1`), not `dataset.index`. First-non-empty keeps the spec's "override applies everywhere" promise with a discoverable rule; auto-only would silently violate the spec's goals list. |
| Editor control | **New `EditorWidget::ColorPicker`** / hex TextField / palette ComboBox | Dedicated picker — arbitrary color is a spec requirement (R2), hex-by-hand is hostile UX, and the enum + delegate extension point is exactly one loader block. |
| Validation | **Store string, resolver guards `QColor::isValid()`** / strict API rejection | Matches AlarmBand semantics, keeps the API forgiving (bad value = automatic, surfaced by the color simply not applying), and avoids a new error path; the editor only ever writes valid `#rrggbb`. |
| Output panel accent | ~~Resolve in `Panel` C++~~ / **keep positional QML call** | Decision reversed mid-build: the panel's entries are `DataModel::OutputWidget`, not `Dataset` — output groups hold no datasets, so there is no override to resolve. Spec-conformant (output controls display no dataset; per-widget styling is a non-goal). |

## Risks & mitigations

- **Subagent-report drift** — the two survey reports were spot-checked against the code
  (Dataset struct, `copy_frame_values`, `preparePooledSlot`/`acquireFrame`,
  `compare_frames`, `serialize(Dataset)`, `Keys::Color`, `takeParam` table,
  `project.dataset.update` registration, `jsonFileMapChanged` consumer,
  `reconfigureDashboard`'s unconditional `widgetCountChanged`); all held. Remaining
  unverified detail (exact MultiPlot/GPS/Plot3D color-caching internals, IconPicker loader
  internals) gets read in full during `/ss-implement` before those edits.
- **Stale color while disconnected** — with no frames flowing, a color edit reaches the
  Dashboard only at the next reconfigure (frame arrival or reset), exactly like title/units
  edits today. AC6 tests the streaming case; the disconnected case is pre-existing
  editor-wide semantics, not a regression. Named here so review doesn't rediscover it.
- **`compare_frames` ignores `color`** (Frame.h:972 compares only `index`) — safe by
  design: color edits always arrive via `syncFromProjectModel`, which calls
  `invalidateFramePool()`; the generation bump forces the full slot re-assign regardless of
  structural equality. No change to `compare_frames` (keeping it minimal is a hotpath
  property).
- **Key reuse across scopes** — `Keys::Color` now appears in both band objects and dataset
  objects; readers are scope-local (`ss_jsr` on the specific QJsonObject), so no collision.
  Older app versions ignore the unknown dataset key on load and would drop it on re-save —
  acceptable additive-format behavior (same as any new optional key).
- **Editor form regressions** — the new row uses the exact `QStandardItem` role pattern of
  its siblings; commit goes through the existing `onDatasetCommonItemChanged` switch with
  `rebuildTree = false` (color is not a tree-visible attribute), so no
  `buildTreeModel`-in-handler exposure (common-mistakes "ProjectModel" row).
- **Style/safety** — new code clears `code-verify.py` (no in-body comments, `[[nodiscard]]`
  on the new getter, ctor-init only), and the resolver adds the usual assertion density.

## Test & verification plan

- **Unit (runnable here):** none — no JS-parser surface. `tests/scripts/` untouched.
- **Integration (maintainer runs, app up with API server on :7777):**
  - `tests/integration/test_project_editor.py` — new cases: (1) `project.dataset.update`
    with `color: "#ff0000"` → `project.dataset.get`/save/reload round-trips the value
    (AC5, R5, R7); (2) update with `color: ""` clears back to automatic (R3); (3) loading
    an existing pre-feature example project yields no `color` keys / empty fields (AC5).
- **In-app maintainer observations:** AC1 (fresh project = automatic everywhere), AC2 (red
  override recolors every widget bound to the dataset, siblings untouched), AC3 (revert
  restores palette color), AC4 (theme switch: override fixed, automatic follows), AC6
  (live recolor while streaming, no reconnect).
- **Hotpath:** `--benchmark-hotpath` all nine gates (AC7) — CI runs it per push; maintainer
  can run locally on the PGO binary for confidence before pushing.
- **Static:** `python scripts/code-verify.py --check` on every touched file; `qt-cpp-review`
  before handoff; `python scripts/sanitize-commit.py` before commit (drives clang-format,
  SDK regen, search-index rebuild).
