---
spec: 0015-operator-mode-hardening
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-17
---

# Plan 0015 — Operator mode hardening

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this — a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Two independent strands. (1) **Gating**: extend the existing per-control operator gates
(`!app.runtimeMode` / `CLI_RUNTIME_MODE`, the idiom already used for "New Workspace…" and
the pin registry) to the four leaked surfaces: taskbar freeze button, Start-menu freeze
entry, taskbar edit-workspace button + right-click "Remove from Workspace", and the
caption menu's project-mutating entries. All controls become *absent* in operator mode
(Q3), no C++ changes. (2) **Search-bar option**: a new negative CLI flag
`--no-taskbar-search`, emitted by the deployment generator when the author switches the
new "Search Bar" toggle off; `CLI::applyOperatorTaskbarSettings()` derives
`TaskbarSettings::searchEnabled` from the flag on every operator launch, making the
deployment deterministic and old shortcuts (no flag) resolve to search-on per Q2. The
Ctrl+F shortcut and the taskbar field already key off `searchEnabled`, so R5 falls out of
the one setter call.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/qml/MainWindow/Panes/Dashboard/Taskbar.qml` | Hide `_freezeButton` and the edit-workspace `IconButton` in runtime mode; guard the widget-button `TapHandler` so `_tbContextMenu` never pops in runtime mode. |
| `app/qml/MainWindow/Panes/Dashboard/StartMenu.qml` | Hide `_freezeBt` in runtime mode (its `visible:` gains `&& !app.runtimeMode`). |
| `app/qml/MainWindow/Panes/Dashboard/WidgetDelegate.qml` | Fold runtime mode into `displayEditable`; hide the "Rename Widget…" item, the "Freeze Title" submenu, and their trailing `MenuSeparator` in runtime mode so only "Open in External Window" remains. |
| `app/qml/Dialogs/ShortcutGenerator.qml` | New "Search Bar" `Switch` (default checked) in the Taskbar tab's Visibility section, disabled/dimmed when mode is `hidden` (R6); value threaded into the `generate(...)` call. |
| `app/src/Misc/ShortcutGenerator.h` | `generate(...)` and `buildArguments(...)` gain a `bool taskbarSearch` parameter (single QML caller, confirmed by grep). |
| `app/src/Misc/ShortcutGenerator.cpp` | `buildArguments` appends `--no-taskbar-search` when `taskbarSearch` is false. |
| `app/src/Misc/CLI.h` | New `QCommandLineOption noTaskbarSearchOpt{"no-taskbar-search", ...}` in `m_opts`. |
| `app/src/Misc/CLI.cpp` | Register the option; `applyOperatorTaskbarSettings()` adds `tbs.setSearchEnabled(!m_parser.isSet(m_opts.noTaskbarSearchOpt))`. |

Translations (`app/translations/ts/*.ts`) pick up the new "Search Bar" string via the
normal `sanitize-commit.py` / lupdate flow — not hand-edited.

## Architecture & data flow

No new objects, signals, or data paths. Control flow for the search option:

- Author: ShortcutGenerator.qml `Switch` → `Cpp_ShortcutGenerator.generate(...,
  taskbarSearch, ...)` → `buildArguments()` writes `--no-taskbar-search` into the shortcut
  file (same mechanism as `--taskbar-mode` / `--taskbar-buttons`).
- Operator launch: `CLI::applyOperatorRuntimeSettings()` (runtime mode only) →
  `applyOperatorTaskbarSettings()` → `TaskbarSettings::setSearchEnabled(...)` with
  persistence already off (`setSettingsPersistent(false)` runs first in the same
  function), so the author's on-disk `Taskbar/SearchEnabled` QSettings value is never
  touched. `searchEnabledChanged` then drives the existing QML bindings
  (`Taskbar.qml` field visibility + `focusSearch()` guard, `MainWindow.qml` Ctrl+F
  `enabled:`) — R5 needs no QML edits.

Gating strand: pure QML `visible:`/guard edits consuming the existing
`app.runtimeMode` context property (`main.qml`, backed by `CLI_RUNTIME_MODE`);
`Taskbar.qml` and `WidgetDelegate.qml` can reach `app` the same way `pinVisible()`
already does. Freeze *state* continues to flow untouched from project JSON through
`ProjectModel::frozen` → `UI::Dashboard::frozen` (spec 0007 wiring, dashboard.md
"Dashboard Freeze Mode").

## Hotpath & threading impact

- **Touches the hotpath?** No. All edits are dashboard-chrome QML, the deployment dialog,
  and CLI startup plumbing; no `FrameReader`/`CircularBuffer`/`FrameBuilder` code, no
  Dashboard draw path, no cached-flag inputs.
- **New cross-thread signal/slot?** No — no new connections at all;
  `setSearchEnabled` is a main-thread call during startup, before frames flow.
- **New input to a cached hotpath flag?** No. `searchEnabled` is UI-only;
  `frozen` inputs are unchanged (we remove UI triggers, not wiring).
- **Timestamp ownership** — untouched.

## Data model & persistence

None. No `Keys::` additions, no project-JSON shape change (freeze stays as spec 0007
stored it), no schema/writer bump, no new QSettings keys. The deployment shortcut file
gains one optional argument; absence keeps today's meaning (search on), so old shortcuts
need no migration (AC4/Q2).

## API / SDK surface

None. No API handlers, enum labels, or SDK surface touched. The new CLI option rides the
existing operator-mode option block (Pro-gated the same way `--taskbar-mode` is; the
ShortcutGenerator dialog is already commercial-only).

## QML / UI

- **ShortcutGenerator.qml**: one `Label` + `Switch` row ("Search Bar", default checked)
  in the Taskbar tab's Visibility section, `enabled: root.taskbarMode !== "hidden"` with
  the same 0.5-opacity dimming the Pinned Buttons rows use (R6). Reset to checked in
  `onVisibleChanged` alongside the other per-open resets.
- **Taskbar.qml**: `_freezeButton.visible` gains `&& !app.runtimeMode`; the
  edit-workspace button gains `visible: !app.runtimeMode` (hidden, not disabled — Q3);
  the `TapHandler` right-click guard adds `!app.runtimeMode` so the remove-from-workspace
  menu never opens.
- **StartMenu.qml**: `_freezeBt.visible` gains `&& !app.runtimeMode`. Its neighbors
  (Auto Layout above, separator + Full Screen below) remain, so no separator orphaning.
- **WidgetDelegate.qml**: `displayEditable` gains `&& !app.runtimeMode`; "Rename
  Widget…" and the "Freeze Title" submenu are hidden (not merely disabled) in runtime
  mode, plus their `MenuSeparator`, using the `visible` + zero-height idiom already used
  for the "Painted Title" entry; if the nested `Menu` cannot be height-collapsed that
  way, remove it at `Component.onCompleted` via `Menu.removeMenu()` in runtime mode —
  either way the end state is: operator menu = "Open in External Window" only.
- No new fonts, themes, or ComboBox restore paths.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| CLI shape for search | `--no-taskbar-search` negative flag; `--taskbar-search <on\|off>` valued; `search` pseudo-id in `--taskbar-buttons` | **Negative flag** — absence-means-on makes the Q2 backward-compat rule structural (old shortcuts need no special case); valued option adds parsing for no expressiveness gain; pseudo-pin overloads pin semantics for a non-pin. |
| Gating mechanism | Per-control `!app.runtimeMode` point gates; new central "operator lock" context property | **Point gates** — matches the established idiom in these exact files (`pinVisible()`, "New Workspace…", Start-menu context menu); a central property is a larger surface for zero behavior difference. Revisit only if a real permissions system (spec non-goal) ever lands. |
| Caption-menu suppression | Hide entries per-item; skip building the menu entirely in runtime mode | **Hide entries** — the menu keeps "Open in External Window" (R7 keeps non-mutating entries), so the menu itself must survive; per-item hiding is the minimal change. |
| Where search-off is enforced | Startup-only `setSearchEnabled` in CLI; extra QML `!runtimeMode` belt-and-braces on the field | **Startup-only** — `searchEnabled` is the single source the field, `focusSearch()`, and Ctrl+F already consult; duplicating the gate in QML adds a second source of truth the next reader must reconcile. Settings persistence is already off in runtime mode, so the author's stored preference cannot be clobbered. |

## Risks & mitigations

- **Bundled scope creep** (common-mistakes.md, Process & Trust): the audit found the
  caption-menu leak mid-flight; it was surfaced and approved into scope (spec Q1) rather
  than slipped in. The file list above is the lane; nothing else gets touched.
- **Signature change on `generate()`**: single caller confirmed by grep
  (`ShortcutGenerator.qml:129`). Parameter ordering will place `taskbarSearch` adjacent
  to the other taskbar params to keep the long positional call readable; QML and C++ must
  change in the same task.
- **Translated-string breakage** (`%n`/`.arg()` class): the only new user-visible strings
  are plain `qsTr("Search Bar")`-style labels with no placeholders — nothing to mangle.
- **Separator orphaning in menus**: hiding items can leave doubled separators; each hide
  site's neighbors are enumerated in QML/UI above, and the caption-menu separator is
  hidden together with its group.
- **Old-deployment behavior shift**: a machine whose author preference was search-off now
  shows search in old deployments — accepted and documented in spec Q2/AC4, not a bug.
- **QQC2 nested-Menu hiding quirk**: `visible: false` on a nested `Menu` may not remove
  its generated menu item on all Qt versions; mitigation is the explicit
  `removeMenu()`-at-completion fallback named in QML/UI, verified by the AC5 in-app
  check.

## Test & verification plan

No pytest/API surface is touched (UI + CLI startup only), so verification is static
checks plus maintainer in-app observations, mapped per acceptance criterion:

- **AC1 (no freeze toggle, frozen honored)** — maintainer: launch an operator deployment
  of a frozen project → no freeze control in taskbar or Start menu, layout locked;
  repeat with unfrozen project → live layout, still no control. Taskbar search for
  "freeze" (search-on deployment) returns nothing (no such Start-menu search entry
  exists — confirmed in `searchableItems()`).
- **AC2 (no workspace editing)** — maintainer: operator deployment with a user workspace
  (id ≥ 1000, the `Taskbar::deleteWorkspace` branch line) active → edit button absent,
  right-click on a taskbar widget button inert, switcher offers switching only. Author
  mode on the same machine retains all three.
- **AC3 (deployment option)** — maintainer: New Deployment dialog shows "Search Bar"
  default-on, dimmed when mode = Hidden; generate one search-on and one search-off
  deployment on a machine whose Preferences toggle is set opposite each time → field and
  Ctrl+F follow the deployment, not the machine.
- **AC4 (old shortcuts)** — maintainer: launch a pre-change shortcut → runs, search bar
  visible regardless of the machine's stored preference.
- **AC5 (caption menu)** — maintainer: operator deployment → widget caption menu shows
  only "Open in External Window"; author mode shows Rename/Freeze Title/pop-out.
- **Hotpath:** not touched; no `--benchmark-hotpath` run required beyond the CI gate that
  runs anyway.
- **Static (I run):** `python scripts/code-verify.py --check` on all six touched files;
  `qt-cpp-review`-relevant C++ delta is small but the review skill runs before handoff;
  `python scripts/sanitize-commit.py` before commit (also regenerates translations for
  the new strings).
