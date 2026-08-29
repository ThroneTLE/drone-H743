---
spec: 0007-dashboard-freeze
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-14
---

# Plan 0007 — Dashboard Freeze Mode

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

The stored frozen flag becomes a first-class project property (`ProjectModel::frozen` +
`Keys::Frozen`), mirroring the `plotTimeRange` pattern: serialized in the `.ssproj`, reset in
`newJsonFile()`, loaded unconditionally so unlicensed round-trips preserve it (R8). The
*effective* state is a read-only `UI::Dashboard::frozen` property computed as
`ProjectModel::frozen && SerialStudio::proWidgetsEnabled()`, with its notify wired to both
`ProjectModel::frozenChanged` and `LemonSqueezy::activatedChanged` — late activation re-derives
for free (R9). QML consumes only the effective property: `WidgetDelegate` hides caption/shadow,
and toolbars are centralized into one new `WidgetToolbar` component (R13) that owns the frozen
gate and scrolls horizontally when the widget is too narrow — replacing the per-widget
size-driven `hasToolbar` logic and its imperative resize handlers. `WindowManager` gets a
`frozen` Q_PROPERTY whose early-outs in the mouse/hover handlers make the layout inert in both
layout modes, explicitly including manual-mode *body* dragging of unfocused windows and edge
resizing, not just caption drags (R3). The toggle fans out from
`DashboardLayout.toggleFreeze()` to a taskbar button (doubling as the passive indicator, R11),
the `Ctrl+Shift+F` shortcut, and a StartMenu item (R1). This shape was chosen over a
layout-blob field (per-workspace, not dashboard-wide) and a QSettings flag (machine-scoped —
the one thing the spec forbids).

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/DataModel/Frame.h` | Add `Keys::Frozen("frozen")` to the project-level `KeyView` list (near `PlotTimeRange`, :177). |
| `app/src/DataModel/ProjectModel.h` / `ProjectModel.cpp` | `m_frozen` member (ctor-init `false`), `frozen()` getter, `setFrozen()` setter (license-gated, `setModified(true)`), `frozenChanged` signal, `Q_PROPERTY`; reset in `newJsonFile()`. |
| `app/src/DataModel/Project/ProjectModelPersistence.cpp` | `json.insert(Keys::Frozen, m_frozen)` in `serializeToJson` (beside `PlotTimeRange`, :246). |
| `app/src/DataModel/Project/ProjectModelLoading.cpp` | Read `Keys::Frozen` (default `false`) in the load path **without** the license gate — direct member write, not `setFrozen()` — and emit `frozenChanged` with the other post-load signals (:1069 block). |
| `app/src/UI/Dashboard.h` / `Dashboard.cpp` | Read-only `frozen` Q_PROPERTY: `ProjectModel::frozen && proWidgetsEnabled() && operationMode-aware`; notify wired to `frozenChanged`, `activatedChanged` (`BUILD_COMMERCIAL`), `operationModeChanged`. Plus `setFrozen(bool)` slot that forwards to `ProjectModel::setFrozen` (single QML entry point). |
| `app/src/UI/WindowManager.h` / `WindowManager.cpp` | `frozen` Q_PROPERTY (plain bool, set from QML). Early-outs: `startManualPress()` **first statement** (`if (m_frozen) return false` — this single gate closes caption drag, manual-mode body drag of unfocused windows, and edge resize, since all three start there), `childMouseEventFilter()`, `mousePressEvent()` (both branches, incl. auto-mode caption drag/reorder), `mouseDoubleClickEvent()`, `updateHoverCursor()` (unset cursor). |
| `app/qml/MainWindow/Panes/Dashboard/WidgetDelegate.qml` | `readonly property bool frozen: Cpp_UI_Dashboard.frozen`; `headerVisible: !frozen`; `shadowEnabled: !frozen && (existing expr)`; the `windowRoot.hasToolbar` mirror assignment becomes `widgetInstance.hasToolbar && !frozen` (re-evaluated on `frozenChanged`). |
| `app/qml/MainWindow/Panes/Dashboard/DashboardCanvas.qml` | Bind `windowManager.frozen`; gate the manual-resize hover MouseArea (`enabled: !_wm.autoLayoutEnabled && !frozen`) and the canvas right-click context menu on frozen. |
| `app/qml/MainWindow/Panes/Dashboard/Taskbar.qml` | Freeze `IconButton` next to the auto-layout button (:909): checked-style highlight = passive indicator (R11); unlicensed click opens the licensing dialog instead of toggling (R7). |
| `app/qml/MainWindow/Panes/Dashboard/DashboardLayout.qml` | `toggleFreeze()`; gate `closeActiveWindow()` / `minimizeActiveWindow()` / `toggleAutoLayout()` on frozen. |
| `app/qml/MainWindow/Panes/Dashboard.qml` | Delegation stub `toggleFreeze()` (matches :50-:56 pattern). |
| `app/qml/MainWindow/Panes/Dashboard/StartMenu.qml` | "Freeze Dashboard" menu item (checked state + Pro gating consistent with the taskbar button). |
| `app/qml/MainWindow/MainWindow.qml` | `Shortcut` `Ctrl+Shift+F` → `dashboard.toggleFreeze()`; add `&& !Cpp_UI_Dashboard.frozen` to the `Ctrl+Shift+W` / `Ctrl+Shift+M` / `Ctrl+Shift+L` / `Ctrl+Home` enables. |
| `app/qml/Widgets/Dashboard/WidgetToolbar.qml` (new) | The central lever (R13): 48 px band hosting toolbar buttons in a horizontal `Flickable` (`contentWidth` = row implicit width, clipped, wheel-scrollable, no scrollbar chrome; subtle edge-fade affordance when overflowing). Owns the visibility policy: `shown = !frozen && widget height ≥ minWidgetHeight` (per-widget threshold property; **width no longer hides the toolbar — it scrolls**). Frozen read: `windowRoot && windowRoot.frozen === true` (undefined in external pop-outs ⇒ not frozen). Collapses to height 0 when hidden so content anchored to its bottom needs no per-widget margin math. |
| `app/qml/Widgets/Dashboard/{Plot,MultiPlot,FFTPlot,Plot3D,Waterfall,Accelerometer,GPS,DataGrid,ImageView,Terminal}.qml` | Migrate each bespoke toolbar block into `WidgetToolbar` (buttons unchanged, `DashboardToolButton` as-is); **delete** the imperative `onWidthChanged`/`onHeightChanged` `hasToolbar` assignments; `hasToolbar` becomes `readonly property bool hasToolbar: toolbar.shown` (keeps the delegate/band mirror contract intact). Terminal derives `hasToolbar` from its console toolbar — verify during implement and adapt rather than force-fit. |
| `app/rcc/icons/buttons/freeze.svg` (new) + its qrc registration | Taskbar/StartMenu icon (snowflake/lock glyph). |

## Architecture & data flow

Single direction, all main thread:

1. **Toggle**: taskbar button / shortcut / StartMenu → `DashboardLayout.toggleFreeze()` →
   `Cpp_UI_Dashboard.setFrozen(!frozen)` → `ProjectModel::setFrozen()` (guard-return,
   license-gated, `setModified(true)`, `frozenChanged`).
2. **Derive**: `Dashboard::frozen` recomputes on `frozenChanged` / `activatedChanged` /
   `operationModeChanged`; QML bindings fan out from that one property.
3. **Chrome**: `WidgetDelegate.headerVisible=false` collapses `captionHeight` to 0 (existing
   MiniWindow contract — `MiniWindow.qml` is not edited); each widget's `WidgetToolbar.shown`
   goes false ⇒ its `hasToolbar` mirror collapses the band and the toolbar itself; shadows off.
4. **Input**: `WindowManager.frozen` early-outs return before any grab/drag/resize/raise
   state is created, so presses fall through to widget content (R4 comes free —
   `childMouseEventFilter` returning false is the existing pass-through path). The
   `startManualPress` gate is the load-bearing one: caption drag, unfocused-window body drag,
   and edge resize all originate there, in both event entry points (filter + own press).
5. **Persistence**: `serializeToJson` writes the stored flag; load path restores it and the
   existing layout blob already round-trips per-window `state` (so a maximized-at-freeze
   widget persists as-is, R12 — no new code).

Ctor-order note: the new `Dashboard` connections reference `ProjectModel` and `LemonSqueezy`,
both pinned before Dashboard in `instantiateCoreModules()` — no ctor-edge change. Nothing in
ProjectModel's protected ctor closure is touched (`setFrozen` is user-driven, post-init).

## Hotpath & threading impact

- **Touches the hotpath?** No. `Dashboard::frozen` is bound by QML at change time only;
  `hotpathRxFrame` / ingest / push tables are untouched. No per-frame reads added anywhere.
- **New cross-thread signal/slot?** No. Every connection is main-thread ↔ main-thread,
  default (direct) connection type.
- **New input to a cached hotpath flag?** No. `frozen` is not read on the frame path and
  does not feed `streamAvailable`/`anyAsyncSink`/etc.
- **Timestamp ownership** — unaffected; no data-path code changes.
- `--benchmark-hotpath` runs once before handoff as the AC7 no-regression check (expected
  no-op; CI gates it regardless).

## Data model & persistence

- **New key**: `Keys::Frozen` = `"frozen"`, project-root level, omitted-means-false on read.
  No schema/writer version bump: unknown-key tolerance already covers old apps reading new
  files, and new apps reading old files default to unfrozen.
- **R8 invariant**: the loader writes `m_frozen` directly from JSON (no license gate), and
  `serializeToJson` writes `m_frozen` unconditionally — an unlicensed load/save cycle
  round-trips the flag. Only `setFrozen()` (the user-facing mutation) is license-gated.
- **QuickPlot / non-project modes**: `ProjectModel::m_frozen` still holds the session value;
  with no `.ssproj` there is nowhere to persist it, which matches R5's session-only rule.
  `newJsonFile()` resets it to `false`.
- Layout/maximized state: already persisted per-group by `Taskbar::saveLayout()` →
  `serializeLayout()` (`winGeom["state"]`); freeze adds nothing there.

## API / SDK surface

None added. The flag rides the existing whole-project serialization, so `project.*` API
reads/saves already expose it for the pytest round-trip (AC4); no new command, no
`EnumLabels` entry, no SDK regeneration. (A dedicated `dashboard.setFrozen` API command is a
possible follow-up, out of scope here.)

## QML / UI

- **Taskbar button** (next to auto-layout, `Taskbar.qml:909` block): icon-highlight when
  frozen — this *is* the passive indicator (R11). When `!proWidgetsEnabled`, the button stays
  visible at reduced opacity and `onClicked` opens the license/upgrade dialog (same UX as
  other Pro touch points) instead of toggling.
- **StartMenu item**: checkable "Freeze Dashboard" entry mirroring the button.
- **Shortcut**: `Ctrl+Shift+F`, `enabled: root.dashboardVisible` (one binding per window —
  the ambiguous-shortcut gotcha in `common-mistakes.md`).
- **Frozen look**: caption + band collapse reclaims 28/48 px for content automatically via
  the existing `captionHeight`/`hasToolbar` anchor math in `WidgetDelegate`.
- New user-visible strings via `qsTr()`; **no `.ts`/`.qm` files are touched** (trust
  contract — translation regeneration is the maintainer's).
- Theme: no new colors; reuses `taskbar_highlight`/`taskbar_text` roles.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Stored-flag home | ProjectModel first-class key · layout blob (`serializeLayout`) · QSettings singleton | **ProjectModel key** — dashboard-wide and travels with the `.ssproj`; layout blobs are per-group/workspace (would fragment freeze per tab); QSettings is machine-scoped, which the spec forbids. |
| Effective-state owner | `UI::Dashboard` C++ property · per-file QML expression (`sweepAllowed` pattern) | **Dashboard property** — one source of truth for ~8 QML consumers; the license predicate stays in C++ behind `SS_LICENSE_GUARD`, and the `activatedChanged` re-derive is wired once instead of per binding. |
| Toolbar hiding | Central `WidgetToolbar` component (scrollable) · per-widget `toolbarVisible` gate · force `hasToolbar` from the delegate | **Central `WidgetToolbar`** (maintainer-directed, R13) — one place owns visibility + the frozen gate, and making narrow toolbars *scroll* instead of hide deletes the `width >= toolbar.implicitWidth` dependency that forced the imperative per-widget handlers (the binding-loop hazard disappears with the policy, not despite it). The per-widget gate was the low-touch fallback; rejected as 10 copies of the same logic. |
| Input locking | `WindowManager` early-outs · QML MouseArea overlays per delegate | **WindowManager early-outs** — all drag/resize/raise/double-click paths already funnel through 5 handlers; an overlay would also block widget-content interaction, violating R4. |
| License predicate | `proWidgetsEnabled()` · new bespoke `freezeEnabled()` | **`proWidgetsEnabled()`** — freeze gates at the same Pro/Trial boundary; a new predicate adds guard-self-test surface for no behavioral difference. |

## Risks & mitigations

- **Toolbar migration regressions** (10 files move to `WidgetToolbar`): behavior change is
  intentional (narrow ⇒ scroll, not hide) but must be verified per widget: frozen/unfrozen,
  height below `minWidgetHeight`, narrow-width scrolling, and toolbar buttons that pop
  dialogs (settings, trigger) still working inside the Flickable. Flick vs. button-press
  gesture conflict is the classic Flickable trap — buttons must still click reliably
  (`pressDelay`/interactive tuning if needed). External pop-outs reuse widget QML, so they
  gain scrollable toolbars too (accepted; they never gain the frozen gate because
  `windowRoot.frozen === undefined` evaluates not-frozen via the `=== true` comparison).
- **Terminal.qml is the odd one out** (`hasToolbar` derives from its console toolbar):
  adapt rather than force-fit; if it can't adopt `WidgetToolbar` cleanly, it keeps its
  structure and gains only the frozen gate — named here so it's not silent scope drift.
- **Body-drag leak in manual mode**: unfocused windows are draggable by body
  (`startManualPress`), not just caption — the frozen early-out sits at the top of
  `startManualPress`, before focus/raise, closing every entry (filter, own press, hover).
- **Setter-guard omission**: `setFrozen` needs the guard-return + `setModified(true)` +
  runtime license gate; loading must bypass the setter (direct member write) or R8 breaks.
- **Late-activation gap (the Plot3D lesson)**: covered structurally — the effective property
  recomputes on `activatedChanged`; verify AC6 explicitly with an offline `.sslic`.
- **Taskbar minimize path**: taskbar window buttons and `Ctrl+Shift+W/M` bypass the chrome —
  gated at `DashboardLayout` functions + shortcut enables; the taskbar *button-click restore*
  behavior (clicking a taskbar entry) only focuses when frozen, never minimizes.
- **Ambiguous shortcut**: `Ctrl+Shift+F` bound exactly once in `MainWindow.qml`
  (`common-mistakes.md` Qt/QML row 2).
- **Scope discipline**: the file table above is the lane; anything discovered outside it
  (e.g. an ExternalWidgetWindow interaction) gets named in chat before touching.

## Test & verification plan

- **Unit (I can run):** none applicable — no parser/JS logic. Static checks only.
- **Integration (maintainer runs, app up + API server):**
  - AC4: extend `tests/integration/` with a project round-trip case — `project.*` load of a
    frozen `.ssproj`, assert `frozen: true` in the returned project JSON, save, re-read,
    assert preserved. Direct `.ssproj` inspection covers the licensed/unlicensed variants.
- **Maintainer in-app observations:**
  - AC1 (three entry points stay in sync), AC2 (chrome hidden/restored, arrangement intact),
    AC3 (drag/resize/restack inert in both layout modes; output button still transmits),
    AC5 (unlicensed: disabled toggle + upsell, opens unfrozen, flag survives save),
    AC6 (offline late activation freezes without reload).
  - R13: shrink a Plot/GPS/DataGrid widget until buttons overflow — toolbar scrolls
    horizontally instead of vanishing; buttons remain clickable; below the height
    threshold it still hides.
- **Hotpath:** one `--benchmark-hotpath` run before handoff (AC7); expected no-delta.
- **Static:** `python scripts/code-verify.py --check` on every touched C++/QML file;
  `qt-cpp-review` on the C++ diff; `python scripts/sanitize-commit.py` before commit.
