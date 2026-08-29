---
spec: 0026-project-navigation
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-21
---

# Plan 0026 — Project navigation overhaul

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Give `DataModel::ProjectEditor` a transient back/forward history of *logical tree identities*
(`{ItemKind kind, int id, int parentId, QString key}`), captured inside the existing
`onCurrentSelectionChanged` (which today throws `previous` away) and replayed by re-selecting
through a single centralized resolver that reuses the same item-map lookups the `selectX`
slots already use — so history is stable across the tree rebuilds that discard row indices,
and dead entries (deleted items) are skipped, not crashed on. Expose `canGoBack`/`canGoForward`
+ `navigateBack()`/`navigateForward()` to QML, and drive them from three input surfaces added
in QML only: a `Qt.BackButton|Qt.ForwardButton` `MouseArea`, `Alt+Left`/`Alt+Right` shortcuts,
and a tree-focus-scoped `Backspace`. The nav buttons plus the four folder-operation buttons
live in the **Project Structure pane caption** via `Widgets.Pane`'s existing `actionComponent`
slot (zero added height), calling the folder backend that the tree context menu already uses
but targeting the *current* selection. Phase 2 adds a directional slide over the destructive
dashboard workspace switch using a frozen `ShaderEffectSource` snapshot captured from a new
`Taskbar::aboutToChangeWorkspace` signal emitted *before* the canvas is cleared; Phase 3 adds
a searchable, keyboard-drivable virtual-desktop switcher overlay fed by the existing
`workspaceModel`/`workspaceTree()`. All three phases are independent; **Phase 1 ships first**.

## Affected subsystems & files

### Phase 1 — Project editor navigation (ships first)

| File | Change |
|------|--------|
| `app/src/DataModel/ProjectEditor.h` | Add `NavEntry` struct; `m_navHistory`/`m_navCursor`/`m_navigatingHistory` members; `Q_PROPERTY canGoBack/canGoForward` + `navHistoryChanged` signal; `public slots navigateBack()/navigateForward()`; private `captureNavEntry`, `resolveNavEntry`, `pushNavEntry`, `clearNavHistory` helpers. |
| `app/src/DataModel/Project/ProjectEditorSelection.cpp` | In `onCurrentSelectionChanged`: after resolving `item`, capture the entry and push it (guarded by `m_navigatingHistory`). Implement `navigateBack/Forward`, `resolveNavEntry` (centralized map lookup incl. `m_userTableItems` path key), `pushNavEntry` (truncate-forward + dedup + cap), `clearNavHistory`. |
| `app/src/DataModel/Project/ProjectEditorTree.cpp` | Clear nav history when the tree is rebuilt for a *different project* (load/new), not on in-project structural rebuilds — wire to the project-load reset path (see Data model note). |
| `app/qml/Widgets/Pane.qml` | No change expected — already exposes `actionComponent` Loader in the caption. (Listed to confirm the extension point; edit only if the caption RowLayout needs a spacer tweak.) |
| `app/qml/ProjectEditor/Sections/ProjectStructure.qml` | Set `actionComponent:` to a `Component` holding the nav + folder-op `Widgets.IconButton` cluster (right-aligned in caption); add `Backspace` handling to the existing `TreeView.Keys.onPressed`; reuse existing `moveContextItemBy`/`newTopFolderForCtx`/`rebuildMoveMenu` logic retargeted to the current selection. |
| `app/qml/ProjectEditor/ProjectEditor.qml` | Add page-level `MouseArea { acceptedButtons: Qt.BackButton \| Qt.ForwardButton }` → navigate; add `Shortcut` items for `Alt+Left`/`Alt+Right`. |
| `app/rcc/rcc.qrc` | Register `icons/navigation/{left,right,up,down}.svg` individually (no globbing), placed alphabetically in the `icons/` block. |

### Phase 2 — Dashboard workspace slide

| File | Change |
|------|--------|
| `app/src/UI/Taskbar.h` / `Taskbar.cpp` | Add `void aboutToChangeWorkspace(int fromIndex, int toIndex)` signal; `Q_EMIT` it at the very top of `setActiveGroupId` **before** `saveLayout`/`clear` (covers every switch path, since all funnel here). |
| `app/qml/MainWindow/Panes/Dashboard/DashboardCanvas.qml` | Wrap the canvas in a container; add a frozen-snapshot overlay (`ShaderEffectSource`) + slide/fade transition driven by `aboutToChangeWorkspace` (freeze) and `activeGroupIdChanged` (animate); direction from index delta. |
| `app/qml/MainWindow/Panes/Dashboard/DashboardLayout.qml` | Only if the container/anchoring must move up a level; confirm during implementation. |

### Phase 3 — Virtual-desktop switcher overlay

| File | Change |
|------|--------|
| `app/qml/MainWindow/Panes/Dashboard/WorkspaceSwitcherOverlay.qml` (new) | Searchable `GridView` of all workspaces (from `taskBar.workspaceModel`, folder context from `workspaceTree()`), arrow/Enter/Escape keys, type-to-filter; activates via `taskBar.selectWorkspaceById`. |
| `app/qml/MainWindow/Panes/Dashboard/Taskbar.qml` | Open the overlay from the existing `_switcher` control (click opens overview instead of / alongside the dropdown) and expose an `open()` hook. |
| `app/qml/MainWindow/MainWindow.qml` | Add the switcher-overlay keyboard shortcut (key TBD — see Tradeoffs OQ3). |

## Architecture & data flow

**Phase 1 selection + history.** Today: tree click / programmatic `setCurrentIndex` →
`QItemSelectionModel::currentChanged` (connected at `ProjectEditorTree.cpp:171`,
`m_currentSelectionConnection`) → `ProjectEditor::onCurrentSelectionChanged(current, previous)`
(`ProjectEditorSelection.cpp:352`, `previous` discarded) → `selectXItem()` cascade →
`setCurrentView()` → `currentViewChanged` → right-pane `Loader` switch (`ProjectEditor.qml`).

New: at the end of `onCurrentSelectionChanged`, once `item` is resolved, build a `NavEntry`
from the item's roles — `TreeItemKind` (UserRole+17), `TreeItemId` (UserRole+18),
`TreeItemParentId` (UserRole+19), and for `KindUserTable` the path string from
`m_userTableItems` (its `TreeItemId` is `-1`). If `m_navigatingHistory` is false, `pushNavEntry`
(truncate forward history at cursor, dedup vs current, append, advance cursor, cap length);
emit `navHistoryChanged`. `navigateBack/Forward` set `m_navigatingHistory=true`, step the
cursor over any entries `resolveNavEntry` can't resolve (bounded loop), call
`m_selectionModel->setCurrentIndex(resolved->index(), ClearAndSelect)`, then clear the guard.
Because replay goes through the same selection signal, the view switch and all
`selected*Id`/form-model updates happen exactly as for a manual click — no divergent path.

`resolveNavEntry(entry)` centralizes the reverse lookup, mirroring the `selectX` slots but
returning `QStandardItem*` (nullptr if gone): dispatch on `kind` over the existing maps
(`m_groupItems`, `m_datasetItems` keyed by (groupId,datasetId), `m_actionItems`,
`m_outputWidgetItems`, `m_sourceItems`, `m_workspaceItems`, `m_workspaceFolderItems`,
`m_groupFolderItems`, `m_tableFolderItems`, `m_userTableItems` by path, singletons
`m_mqttPublisherItem`/`m_controlScriptItem`, root → `ProjectView`).

**Phase 1 UI.** `Widgets.Pane` caption (`Pane.qml:110`) already renders a `Loader {
sourceComponent: root.actionComponent }` after the fill-width title label — the buttons
right-align there. The `actionComponent` is defined in `ProjectStructure.qml`, so it can reach
`treeView.selectionModel` and `Cpp_JSON_ProjectModel`/`Cpp_JSON_ProjectEditor`. Back/forward
bind enabled to `Cpp_JSON_ProjectEditor.canGoBack/canGoForward`; folder buttons read the
current selection's kind/id from `selectionModel.currentIndex` roles and enable per the same
conditions the context menu already uses, calling the same `Cpp_JSON_ProjectModel` move/add
slots on the current selection.

**Phase 2.** `Taskbar::setActiveGroupId` (`Taskbar.cpp:390`) is the sole sink for every switch
path. Emitting `aboutToChangeWorkspace(fromIndex,toIndex)` before it clears lets QML freeze the
outgoing canvas into a `ShaderEffectSource` overlay; after the C++ repopulates and emits
`activeGroupIdChanged`, QML slides the frozen overlay out (direction = sign(toIndex-fromIndex))
while the rebuilt canvas slides/fades in, then discards the snapshot.

**Phase 3.** Pure additive overlay reading `taskBar.workspaceModel` / `workspaceTree()` and
calling `selectWorkspaceById`; no model or switch-path change.

## Hotpath & threading impact

- **Touches the hotpath?** **No.** Nothing here runs on `FrameReader`/`CircularBuffer`/
  `FrameBuilder`/the Dashboard *draw* frame or the span fast lane. Editor navigation is
  main-thread model/UI. The Phase-2 snapshot/slide runs only on a user-initiated workspace
  switch (rare, human-paced), never per frame. `--benchmark-hotpath` is unaffected (AC7).
- **New cross-thread signal/slot?** **No.** `aboutToChangeWorkspace` is a same-thread
  (`Taskbar` ↔ QML) signal; default `AutoConnection` resolves to direct. History replay is
  same-thread. No mutexes, no SPSC changes.
- **New input to a cached hotpath flag?** **No.** None of `m_operationMode`, `m_anyAsyncSink`,
  `m_captureLatestFrame`, `m_streamAvailable`, `m_changeDriven`, `m_playerOpen` gain an input.
- **Timestamp ownership** — unchanged; no frame is created, copied, or re-stamped.

## Data model & persistence

- **No persistence.** Navigation history is transient runtime state on `ProjectEditor`; it is
  **not** written to `.ssproj`/JSON. No `Keys::` additions, no `schemaVersion`/writer-version
  bump, no Sessions DB change, no legacy-alias work.
- **Lifecycle:** history is cleared when a *different* project is loaded or a new project is
  created (so stale entries from a previous file don't linger), but **not** on ordinary
  in-project structural rebuilds (add/delete/move/rename) — those keep history valid because
  entries resolve by stable id/path and dead entries are skipped. Wire `clearNavHistory` to the
  project load/new reset (the same reset that rebuilds the tree for a new document); exact
  signal confirmed in implementation. Dashboard workspace identity is unchanged.

## API / SDK surface

None. No new API handlers, `EnumLabels` slugs, SDK changes, or `apiCall` reach. No
`BUILD_COMMERCIAL` gating — navigation is core UX in all builds. (Phase-2 `Taskbar` and Phase-3
overlay involve dashboard workspaces, some of which are Pro widgets, but the navigation
mechanics themselves are not license-gated.)

## QML / UI

- **New icons:** `qrc:/icons/navigation/{left,right,up,down}.svg` (18×18), after registration.
  `left`=back, `right`=forward, `up`=move up, `down`=move down; Add Folder / Move-to-folder
  reuse existing `icons/project-editor/*` icons. Buttons use `Widgets.IconButton`
  (`iconSize` default 18, `icon.color: "transparent"` to keep SVG colors, tooltip via attached
  `ToolTip.text: qsTr(...)`), grouped in a `RowLayout` inside the caption `actionComponent`.
- **Enablement:** `canGoBack/canGoForward` drive the nav buttons; folder buttons mirror the
  existing context-menu visibility conditions per selection kind.
- **Phase 2 overlay:** `ShaderEffectSource` frozen snapshot + `NumberAnimation` slide (short,
  ~150–220 ms to match existing dashboard easing) with graceful fallback (crossfade / instant)
  when no snapshot is available (R10).
- **Phase 3 overlay:** `GridView` + `Widgets.SearchField` + `Keys` navigation, themed via
  `Cpp_ThemeManager.colors[...]`; scales via scroll + type-to-filter.
- No ComboBox restore-race surfaces are introduced (the workspace switcher already exists).

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| History identity | (A) `QPersistentModelIndex` (B) logical `{kind,id,parentId,key}` (C) full state snapshot | **B** — indices die on the fresh `selectionModel` built each tree rebuild; ids/paths already exist as item roles and survive, and replay reuses tested `selectX` map lookups. |
| Replay mechanism | (A) walk tree matching roles (B) centralized `resolveNavEntry` reusing existing maps | **B** — one small helper reuses the exact maps `selectX` already uses (incl. `m_userTableItems` path), so behavior can't diverge from a normal click; O(n) map scan is trivial at human cadence. |
| Button placement | (A) new thin toolbar row (B) `Widgets.Pane` caption `actionComponent` | **B** — the maintainer's call: zero added vertical height, uses existing unused caption space, and the tree pane is the natural home for tree navigation + folder ops. |
| Backspace scope (OQ1) | (A) tree-focus only (B) any non-text control in editor | **A** — tree-focus only; zero risk of eating Backspace while typing in a field; widen later if it feels narrow. |
| Folder buttons (OQ4) | (A) always shown, enable/disable per kind (B) show/hide per kind | **A** — fixed button positions are more predictable than buttons that appear/vanish. |
| Phase-2 snapshot trigger | (A) grab in QML per switch path (B) new `aboutToChangeWorkspace` emitted before clear | **B** — the canvas is already the *new* workspace by the time `activeGroupIdChanged` fires; a pre-clear signal at the single C++ sink is the only place to freeze the outgoing canvas, and it covers all trigger paths at once. |
| Phase-3 invocation (OQ3) | (A) reuse `_switcher` click to open overview (B) new dedicated shortcut | **A + a shortcut to confirm** — open the overview from the existing switcher control (discoverable), plus a keyboard shortcut; exact key deferred (PgUp/PgDown + Ctrl+1..9 are taken — candidate `Ctrl+K` or `F2`), confirm in `/ss-tasks`. |

## Risks & mitigations

- **History replay re-entrancy** — programmatic `setCurrentIndex` re-fires
  `onCurrentSelectionChanged`; without the `m_navigatingHistory` guard it would corrupt the
  stack. Mitigation: set/clear the guard around every replay; unit-reason the truncate/dedup/cap
  invariants. (Silent-breakage class: signal re-entrancy.)
- **Dead / stale entries** — an item in history gets deleted. Mitigation: `resolveNavEntry`
  returns nullptr and the navigator skips it in a **bounded** loop (NASA fixed-bound); never
  lands on a blank pane (AC2).
- **Unbounded growth** — long sessions. Mitigation: cap history at a fixed N (e.g. 128), drop
  oldest and adjust cursor.
- **Backspace stealing keystrokes** — scoped to `TreeView` focus only; text fields keep native
  Backspace (AC3).
- **Phase-2 snapshot timing** — `grabToImage` is async and may miss the pre-clear window;
  `ShaderEffectSource` freeze must be captured in the `aboutToChangeWorkspace` handler while the
  old canvas is still valid. Mitigation: freeze on the pre-clear signal; if the freeze isn't
  ready, fall back to crossfade/instant (R10) — never a broken frame. This is the phase's main
  risk and why Phase 2 ships after Phase 1.
- **Independent (external-window) dashboards** — each owns its own `Taskbar`
  (`independentWorkspace`); the new signal/overlay must be per-instance, not global, so external
  windows animate their own switches.
- **Scope creep** — folder buttons must call the *existing* backend slots, not new move logic;
  the diff stays in the file list above.

## Test & verification plan

- **Unit (you can run):** none — this is C++/QML UI state with no `tests/scripts/` JS-parser
  surface. History invariants (truncate-forward, dedup, cap, skip-dead) are verified by
  maintainer observation per AC1–AC2; logic is kept small and guard-flagged for review.
- **Integration / security / perf (maintainer runs):** no new `pytest` target is strictly
  required, but AC1–AC6 are concrete in-app observations the maintainer performs with the app
  running (history back/forward incl. after delete; mouse + Alt + tree-scoped Backspace; caption
  buttons enablement + parity with context menu; directional slide across all switch paths;
  switcher overlay open/scroll/filter/keys/activate).
- **Hotpath:** `--benchmark-hotpath` must still pass its gate (AC7) — expected no delta since
  nothing touches the parse/publish path.
- **Static:** `python scripts/code-verify.py --check` on every touched C++/QML file; run the
  `qt-cpp-review` / `qt-qml-review` skills before handoff; `python scripts/sanitize-commit.py`
  before any commit.
