---
spec: 0013-widget-title-overrides
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-16
---

# Plan 0013 — Per-widget title overrides and freeze-titlebar visibility

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this — a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

`ProjectModel` persists one new root JSON object, `widgetDisplay`, holding a title-override
map (`titles: { uniqueId → string }` — dataset and group uids share one counter, so one map)
and a freeze-title mode map (`freezeTitle: { "type:uniqueId" → "bar" | "painted" |
"hidden" }`, absent = per-type default). Titles propagate via **copy-patch**: `Dashboard::buildWidgetGroups` /
`processDatasetIntoWidgetMaps` already copy every group/dataset into the display-only
`m_widgetGroups` / `m_widgetDatasets` maps — overrides are applied to those copies at copy
time, so every downstream display surface (WidgetRegistry → taskbar/search,
`GET_DATASET`/`GET_GROUP` → painted instrument titles, `DashboardWidget::widgetTitle()` →
captions, freeze header, external windows) inherits the override with zero per-surface code,
while `m_lastFrame` — the thing exports and `dashboard.getData` serialize — is never touched,
so R3 holds by construction. A live edit re-patches the copies in place, pushes new titles
through `WidgetRegistry::updateWidget` (signal exists, currently unconsumed), and notifies
QML — no dashboard rebuild (R8). Freeze-title modes are pure QML-read state consumed by
`WidgetDelegate` and the three instrument widgets. Editing lands in the workspace editor
rows and a caption menu button; `project.dashboard.*` API commands give R10 parity.

**Spec amendment required (R4/R5 phrasing):** a boolean cannot express today's frozen
instrument look (painted title, no header) *and* "gauge can show the panel header" *and*
"label-free face". The per-widget setting is a **freeze-title mode** with only explicit
values: `bar` (panel header shown, painted title suppressed while frozen; default for
non-instrument types), `painted` (title on the instrument face, no header; Bar/Gauge/Meter
only, and their default), and `hidden` (no header, painted title suppressed while frozen).
Name shown at most once in every state (R5's invariant). *(Revised during implementation,
2026-07-16: the original tri-state carried an `auto` value resolving per widget type; it
was removed as confusing — UI/API always present the resolved mode and `SerialStudio::
dashboardWidgetPaintsTitle()` is the single per-type-default predicate. Writing a widget's
default removes its stored entry.)*

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/DataModel/Frame.h` | New `Keys::` constants: `WidgetDisplay`, `Titles`, `FreezeTitle` (key names only — Frame structs untouched) |
| `app/src/DataModel/ProjectModel.h` | `m_widgetDisplay` member; getters `displayTitle(uid)`, `displayTitles()`, `freezeTitleMode(type, uid)`; slots `setDisplayTitle`, `setFreezeTitleMode`; signal `widgetDisplayChanged()` |
| `app/src/DataModel/ProjectModel.cpp` | Setter bodies (guard-return, `setModified(true)`, emit); wire `widgetDisplayChanged` → `scheduleAutoSave` next to the existing `widgetSettingsChanged` hookup (:155); reset in `newJsonFile()` |
| `app/src/DataModel/Project/ProjectModelPersistence.cpp` | Serialize `widgetDisplay` beside `widgetSettings` (:322); omit when empty |
| `app/src/DataModel/Project/ProjectModelLoading.cpp` | Load in `loadWidgetSettingsAndWorkspaces` (:807); absent key = empty maps |
| `app/src/DataModel/ProjectEditor.h` | `ResolvedWidget` gains `uniqueId` + `isGroupWidget` fields |
| `app/src/DataModel/Project/ProjectEditorSummaries.cpp` | `buildResolvedWidgetLookup` fills the new fields; `widgetsForWorkspace` rows gain `uniqueId`, `isGroupWidget`, `displayTitle`, `freezeTitleMode` |
| `app/src/UI/Dashboard.h` | Declare `applyDisplayTitles()` (patch helper) + `refreshDisplayTitles()` slot + `displayTitlesChanged()` signal |
| `app/src/UI/Dashboard.cpp` | Apply overrides when copying into `m_widgetGroups`/`m_widgetDatasets` (incl. LED-panel synthesized group title built from the *displayed* group title); live re-patch slot: update copies + `WidgetRegistry::updateWidget` + emit; connect `ProjectModel::widgetDisplayChanged` |
| `app/src/UI/DashboardWidget.h/.cpp` | `widgetTitle` gains a real `widgetTitleChanged` NOTIFY, emitted from Dashboard's `displayTitlesChanged` |
| `app/src/UI/Taskbar.h/.cpp` | Consume `WidgetRegistry::widgetUpdated` → `findItemByWidgetId` → `setData(WidgetNameRole)` (today items are built once and never retitled) |
| `app/qml/MainWindow/Panes/Dashboard/WidgetDelegate.qml` | `windowRoot.title` becomes a live binding to `widgetTitle`; `frozenPanel` bool replaced by the explicit mode read from ProjectModel (resolved per-type default, no "auto"); header/painted arbitration |
| `app/qml/MainWindow/Panes/Dashboard/ExternalWidgetWindow.qml` | Title follows `widgetTitleChanged` (today one-shot at :119) |
| `app/qml/Widgets/Dashboard/Bar.qml`, `Gauge.qml`, `Meter.qml` | Painted-title elements gate on `windowRoot.frozen` + effective mode (suppressed under `bar`/`hidden` while frozen); title text prefers `windowRoot.title` (live) over `model.title` (ctor snapshot); drop the hardcoded `frozenPanel = false` opt-outs |
| `app/qml/ProjectEditor/Views/WorkspaceView.qml` | Per-row editable display-title field (placeholder = canonical) + freeze-title mode combo (Title bar / Painted title [instruments only] / Hidden); refresh on `widgetDisplayChanged` |
| `app/qml/Widgets/MiniWindow.qml` + `WidgetDelegate.qml` | Visible caption menu button (left edge, `menu.svg`) replaces the external-window button; menu hosts "Rename Widget…", the freeze-title submenu, and "Open in External Window", every entry with a mono `icons/buttons` icon |
| `app/src/API/Handlers/DashboardHandler.cpp` | Register `project.dashboard.setWidgetTitle`, `project.dashboard.getWidgetTitles`, `project.dashboard.setWidgetFreezeTitle` |
| `app/rcc/ai/command_safety.json` | Safety tiers for the two new mutating commands |
| `doc/help/API-Reference.md` | Document the three commands |
| `tests/integration/test_widget_display.py` | New: round-trip, reorder survival, canonical-title isolation (AC1/AC3/AC7) |
| `tests/scripts/test_ai_assistant_static.py` | Safety-registration assertions for the new commands (AC7) |

Out of lane, named here: `app/rcc/ai/search_index.json` and the generated SDK regenerate via
`sanitize-commit.py` — expected diff noise, not hand-edited.

## Architecture & data flow

**Persistence:** `widgetDisplay` lives beside `widgetSettings` in the project root
(ProjectModelPersistence/Loading). Setters follow the `saveWidgetSetting` shape: guard-return
on no-op, `setModified(true)`, emit `widgetDisplayChanged`. That signal joins the
autosave debounce (ProjectModel.cpp:155 pattern) and — for the API path — the epoch-gated
apply wiring that all project mutators need (see `project_api_mutation_apply_path`
conventions: the new mutating commands go through `CommandRegistry` so the debounced
autosave + `syncFromProjectModel` epoch machinery already applies).

**Title flow (static):** `reconfigureDashboard` → `buildWidgetGroups` copies groups/datasets
into `m_widgetGroups`/`m_widgetDatasets`, patching `copy.title` from the override map (one
`QHash` lookup per widget, reconfigure-time only) → `registerWidgets` seeds the
`WidgetRegistry` with the already-patched titles → taskbar items and search read registry
info; `DashboardWidget::widgetTitle()` and the widget-model ctors (`Bar.cpp:52` et al.) read
the patched copies via `GET_DATASET`/`GET_GROUP`. `m_lastFrame` keeps canonical titles.

**Title flow (live edit, R8):** `ProjectModel::widgetDisplayChanged` →
`Dashboard::refreshDisplayTitles()` (direct, main thread): re-patch the copies in place,
`WidgetRegistry::updateWidget(id, title)` per changed widget, emit
`Dashboard::displayTitlesChanged`. Consumers: Taskbar's new `widgetUpdated` handler retitles
its `QStandardItem`s; each `DashboardWidget` re-emits `widgetTitleChanged` so QML captions,
freeze headers, and external windows rebind; instrument QML reads `windowRoot.title`, which
follows the same notify. No `widgetCountChanged`, no delegate re-instantiation, no frame
interruption.

**Freeze-title mode flow:** pure QML read. `ProjectModel::freezeTitleMode()` returns the
resolved mode: explicit map entry wins, else the per-type default from
`SerialStudio::dashboardWidgetPaintsTitle()` (`painted` for Bar/Gauge/Meter, `bar`
otherwise). `frozenHeaderVisible` becomes `frozen && effectiveMode === "bar" &&
title.length > 0`; the delegate exposes the effective mode via `windowRoot` so instruments
gate their painted titles (`frozen && effectiveMode !== "painted"` → suppress). The
hardcoded `windowRoot.frozenPanel = false` lines in the three instruments are removed.

## Hotpath & threading impact

- **Touches the hotpath?** Dashboard yes, frame path no. All title work happens in
  `reconfigureDashboard` (already a cold, full-rebuild context) or in the explicit
  `refreshDisplayTitles` slot (user-edit cadence). `updateDashboardData`'s per-frame walk,
  the push tables, and the span lane are untouched. `m_lastFrame` titles stay canonical, so
  per-frame `compare_frames()`/generation semantics see identical data. One `QHash<int,QString>`
  lookup per *widget* per *reconfigure* is the total added cost. `--benchmark-hotpath` gates
  must stay green (AC8); the `lua+dashboard` floor covers the reconfigure path indirectly.
- **New cross-thread signal/slot?** No. ProjectModel, Dashboard, WidgetRegistry, Taskbar are
  all main-thread; new connections are default (direct) same-thread.
- **New input to a cached hotpath flag?** No. The override map is not read on the frame
  path, so no cache/refresh wiring is needed. Explicitly *not* adding a cached flag.
- **Timestamp ownership** — untouched; no export/report code changes at all.

## Data model & persistence

- New `Keys::` constants in `Frame.h` (`namespace Keys` is the single source of truth for
  project JSON keys): `WidgetDisplay("widgetDisplay")`, `Titles("titles")`,
  `FreezeTitle("freezeTitle")`. Frame structs and `Frame::serialize` are untouched — the
  map is ProjectModel state, never frame state.
- Shape: `"widgetDisplay": { "titles": { "614": "Chamber Pressure", "9:614": "Pressure
  Spectrum" }, "freezeTitle": { "9:614": "hidden" } }`. Titles hold two key scopes
  (amended 2026-07-16): bare decimal uniqueId = entity-level (all widgets of that
  dataset/group), `<int widgetType>:<uniqueId>` = widget-level, which wins on resolution
  (widget → entity → canonical). FreezeTitle keys are always widget-level. UI surfaces
  (workspace editor rows, caption menu) edit the widget-level scope; the API sets either
  (widgetType present = widget-level).
- Empty-string title = remove entry (spec R1). Empty maps are omitted from the save.
- **Migration story:** absent key loads as empty maps (today's behavior, R4 default); no
  schema-version bump — older Serial Studio ignores unknown root keys (loader reads known
  keys only), satisfying the loader-tolerance constraint. `newJsonFile()` resets the member.
- Orphaned entries (deleted dataset/group) are inert: resolution is uid-lookup at patch
  time, a miss just keeps the canonical title. No pruning pass (matches how stale
  `widgetSettings` entries are handled — they persist harmlessly).
- QuickPlot/DeviceDefined: `saveWidgetSetting` gates on `operationMode == ProjectFile`; the
  new setters gate identically. Out-of-scope modes never read the map because their
  ProjectModel map is empty.

## API / SDK surface

Registered in `DashboardHandler.cpp` beside `dashboard.setTimeRange` /
`project.dashboard.setTimeRange`:

- `project.dashboard.setWidgetTitle { uniqueId, title }` — empty/absent `title` clears.
  Mutating: safety-tier entry in `command_safety.json`, rich response echoing old/new value.
- `project.dashboard.getWidgetTitles {}` — returns the map plus, per entry, the canonical
  title it shadows (resolvability aid for the assistant).
- `project.dashboard.setWidgetFreezeTitle { widgetType, uniqueId, mode }` — mode ∈
  `bar | painted | hidden`; `painted` only valid for Bar/Gauge/Meter; writing the
  per-type default removes the entry. Mutating, same conventions.

All three flow through `CommandRegistry` (epoch + debounced autosave apply automatically).
SDK (`SerialStudio.js`/`.lua`) and the AI search index regenerate via `sanitize-commit.py`.
No `EnumLabels` slugs (mode strings live in the handler). Nothing commercial-gated — spec
says no new tier gate; the freeze *header* only renders inside Pro-gated freeze mode anyway.

## QML / UI

- **WidgetDelegate:** replace `frozenPanel` (bool, instrument opt-out) with
  `freezeTitleMode` resolved from ProjectModel (`Q_INVOKABLE` read + `Connections` on
  `widgetDisplayChanged`, the `widgetSettings(root.widgetId)` restore pattern). Effective
  mode drives `frozenHeaderVisible`; exposed on `windowRoot` for instruments. The caption's
  left-edge button becomes a menu button (`menu.svg`, replacing the external-window button;
  MiniWindow emits `menuClicked`): "Rename Widget…" (`rename.svg`), a "Freeze Title"
  submenu (`freeze.svg`) with icon-free checkable options Title Bar / Painted Title
  (instruments only) / Hidden, and "Open in External Window" (`expand.svg`, absorbing
  the removed button). *(Revised 2026-07-16 from the original hidden right-click menu.)*
- **Instruments (Bar/Gauge/Meter QML):** painted `WidgetTitleBar` strips and in-face title
  labels add a freeze gate (`!windowRoot.frozen || effectiveMode === "painted"`); text
  prefers `windowRoot.title`. Normal-mode appearance is untouched.
- **WorkspaceView:** the row table gains an editable "Display Title" TextField (placeholder
  shows the canonical title; commit on editingFinished → `setDisplayTitle(uid, text)`) and
  a mode ComboBox. Both hidden when `customizeWorkspaces` is off, matching the existing
  row-edit gating. Rows referencing the same uid share one override — edits refresh
  sibling rows via the changed signal. LED-panel rows: title field disabled (synthesized
  `LED Panel (%1)` label derives from the group's *display* title automatically); mode
  combo active.
- ComboBox restore-race: guard with the `restoringPage`-style suppression flag when
  programmatically syncing the mode combo.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Title propagation | Frame-Injection (patch `m_lastFrame`) / Per-Surface Resolver (helper at ~10 sites) / **Copy-Patch** | Copy-Patch: one resolution point where Dashboard already copies for display; R3 safe by construction because `m_lastFrame` (the export/API source) is never touched; per-surface misses impossible |
| Freeze-title control shape | Boolean show/hide / tri-state with `auto` / **explicit mode enum (bar/painted/hidden)** / two independent booleans | Explicit enum: a boolean cannot express today's default *and* header-on-gauge *and* label-free; two booleans allow the forbidden "both titles" state R5 bans; `auto` (the first tri-state shape) hid the per-type resolution from the user and was dropped 2026-07-16 as confusing. Requires a small R4/R5 wording amendment |
| Live-update mechanism | Piggyback on full dashboard rebuild / **In-place re-patch + registry update + notify** | R8 explicitly bans a visible rebuild; the fan-out is contained (one Dashboard slot, one unused registry signal finally consumed, one QML notify) |
| Override keying | `widgetSettings` widgetId (`type:groupId:datasetIndex`, positional) / **uniqueId (+ type for modes)** | Spec R9 demands reorder survival; the positional widgetId key breaks on any reorder (known wart of `widgetSettings`) |
| Storage location | Per-dataset/group JSON fields / **Separate root map** | User asked for a map; per-entity fields would ride `Frame` serialization and leak into API/frame payloads, violating R3's boundary |
| API namespace | `project.workspace.*` / **`project.dashboard.*`** | Overrides are dashboard presentation, not workspace membership; `project.dashboard.setTimeRange` set the precedent |

## Risks & mitigations

- **Caption input routing:** the menu opens from a real caption `ToolButton` (2026-07-16
  revision), the same pattern as the old external-window button, so
  `WindowManager::childMouseEventFilter` treats it like the other caption controls; the
  drag-region geometry reads `menuControlWidth` (renamed from `externControlWidth`) and
  must keep excluding the button, or caption drags swallow its clicks.
- **Stale-copy drift:** `m_widgetGroups`/`m_widgetDatasets` copies are rebuilt on every
  reconfigure, so a patch bug self-heals on rebuild; the live re-patch reuses the same
  apply function as the copy-time patch (single implementation, can't diverge).
- **AlarmMonitor notification text** resolves datasets from Dashboard state by uniqueId; if
  it reads the patched copies its notification text shows the display title. That is
  arguably correct (display surface) — verify which container it reads during
  implementation and note the observed behavior at handoff; no code change planned.
- **Two-titles regression (R5):** the arbitration lives in one place (`WidgetDelegate`
  effective-mode + the instrument gate reading the same `windowRoot` value), not in
  per-widget booleans that can disagree. AC4 observation covers it.
- **Silent-breakage classes from `common-mistakes.md` this change is exposed to:** setter
  guard-returns (all new setters); `%n`-vs-`.arg()` in new translated strings (menu items);
  no `buildTreeModel()` from item handlers (workspace editor edits go through ProjectModel
  slots, not tree mutation); QML `Shortcut` — none added.
- **Trust/lane:** the file list above is the lane; `Taskbar` gains one slot (named here
  because today nothing consumes `widgetUpdated`) — no other opportunistic changes.

## Test & verification plan

- **Integration (maintainer runs, app up with API server on `localhost:7777`):**
  `tests/integration/test_widget_display.py` — AC1: set override via
  `project.dashboard.setWidgetTitle`, save/reload project, assert round-trip; reorder
  groups via `project.group.move`-equivalents and assert the override still targets the
  same dataset; delete the dataset, assert project loads and entry is inert. AC3: with
  overrides set, assert CSV export headers and `project.dataset.get` /
  `dashboard.getData` return canonical titles. AC7: command presence, mutation semantics,
  default-mode-clears-entry, invalid-mode and painted-on-non-instrument rejection.
- **Static (I run):** `tests/scripts/test_ai_assistant_static.py` — safety-tier entries
  exist for both mutating commands; API-Reference documents all three.
  `python scripts/code-verify.py --check` on every touched file; `sanitize-commit.py`
  before commit.
- **Maintainer observations:** AC2 (override visible on caption, freeze header, painted
  gauge title, taskbar, pop-out, second widget on same dataset), AC4 (tri-state defaults
  match today; flips work; never two titles), AC5 (workspace editor edit → modified →
  save/restore), AC6 (live rename mid-stream, no rebuild flicker, persists).
- **Hotpath:** AC8 — `--benchmark-hotpath` run by maintainer/CI; expectation: no delta
  outside the historical noise band (gates pass/fail only, per the CI-noise memory).
