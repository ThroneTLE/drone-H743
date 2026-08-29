---
spec: 0028-icon-registry
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-21
---

# Plan 0028 — Centralized icon & command registry

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Gate: do not start `/ss-tasks` until a human marks this
> `approved`.

## Approach (one paragraph)

Two small GUI-thread singletons, each deriving its catalog from resources rather than code.
`Misc::IconRegistry` scans `:/icons/<category>/<tier>/<name>.svg` once and answers
`icon(category, name, px)` with the nearest tier at-or-above `px` (R1-R4); the icon tree is
consolidated to that layout by a manifest-driven migration script, with qrc **aliases**
keeping every old path alive so the 804 references migrate surface-by-surface instead of
big-bang. `UI::CommandRegistry` loads per-context command manifests and per-surface layout
manifests from `:/commands/`, translates titles through the existing
`QT_TRANSLATE_NOOP`-dummy-source pipeline, and exposes plain `QVariantList` queries.
Behavior never enters JSON: each context ships one QML bindings file mapping command id to
`{run, enabled, checked, visible}` (real bindings, so reactivity is native), with
commercial-only entries in a Loader-guarded file so GPL builds evaluate nothing Pro (R14).
Surfaces become renderers: a shared `CommandToolbar` draws both ribbon toolbars from layout
JSON, the Start menu draws its static structure from layout JSON (dynamic submenus stay
named slots), the palette/taskbar-search/start-search swap their three copy-pasted
providers for registry queries, and per-window `Instantiator`s of `Shortcut` items replace
the 31 hand blocks (R9-R16).

## Affected subsystems & files

New C++ / resources / scripts:

| File | Change |
|------|--------|
| `app/src/Misc/IconRegistry.h/.cpp` | NEW — catalog scan of `:/icons`, `icon(category, name, px)` resolution, placeholder + once-per-key warning. Modeled on `Misc::IconEngine` (singleton shape). |
| `app/src/UI/CommandRegistry.h/.cpp` | NEW — loads/validates `:/commands/*.json` + `:/commands/layouts/*.json`; per-context command queries, layout trees, shortcut display text; retranslate hook. (Namespace-distinct from `API::CommandRegistry` — accepted.) |
| `app/src/UI/CommandStrings.cpp` | NEW, GENERATED — `QT_TRANSLATE_NOOP("Commands", ...)` dummy source so lupdate sees manifest strings (pattern: `NativeTemplates/TextTemplates.cpp:582+`). Committed like other generated artifacts. |
| `app/rcc/commands/app.json`, `dashboard.json`, `projecteditor.json` | NEW — command manifests (content lifted 1:1 from the three `*Actions.qml` + StartMenu + toolbars + Shortcut blocks). |
| `app/rcc/commands/layouts/main-toolbar.json`, `project-toolbar.json`, `start-menu.json` | NEW — surface layout manifests. |
| `app/rcc/icons/**` | RESTRUCTURED — `<category>/<tier>/<name>.svg`, duplicates dropped, placeholder added (`system/16/missing.svg`); `buttons/` untouched. |
| `app/rcc/rcc.qrc` | REWRITTEN (icons block) — new tree + temporary `<file alias="icons/<old-path>">` compat aliases; `:/commands/*` entries added. |
| `scripts/icon-migrate.py` | NEW — audit + apply modes over a reviewed CSV manifest (tu-cutter discipline: refuses to act unless the manifest reconstructs the tree exactly). |
| `scripts/registry-verify.py` | NEW — icon-tree lint (layout conformance, byte-dup sweep, qrc<->disk sync, alias audit) + manifest lint (schema, unique ids, icon refs resolve, per-context duplicate shortcuts). |
| `scripts/generate-command-strings.py` | NEW — manifests → `CommandStrings.cpp`; wired into `sanitize-commit.py` next to the SDK/search-index steps (`sanitize-commit.py:185` area). |
| `doc/claude/specs/0028-icon-registry/icon-map.csv` | NEW — the audit/migration manifest (old path → category/tier/name, dup/merge verdicts); maintainer sign-off artifact for Q2/Q3. |

Edited C++ (grep-confirmed touch-points):

| File | Change |
|------|--------|
| `app/src/Misc/ModuleManager.cpp` | Two context properties in `registerCoreContextProperties()` (:734): `Cpp_Misc_IconRegistry`, `Cpp_UI_CommandRegistry`. Lazy first-touch like `Cpp_Misc_IconEngine` (:772) — **not** added to `instantiateCoreModules()`. |
| `app/src/SerialStudio.cpp` | `dashboardWidgetIcon(w, large)` (:259-339, decl `.h:308`) keeps its signature; body becomes enum→name switch + `IconRegistry::icon("widgets", name, large ? 32 : 16)`. 11 call sites (8 C++, 3 QML) untouched. |
| `app/src/UI/Taskbar.cpp` | Folder/workspace fallback icons (:1509, :1522-1527) resolve via registry; model gains a logical `iconId` role ("category/name") alongside the resolved-URL role (contract below). |
| `app/src/DataModel/Project/ProjectEditorTree.cpp`, `ProjectEditorForms.cpp`, `ProjectEditorSummaries.cpp`, `app/src/UI/Widgets/DataGrid.cpp` | Icon literals → registry calls. |
| `app/src/Platform/CSD.cpp` | `:/icons/csd/*` (:204-207) → registry (`window` category), merging the csd/miniwindow duplicate glyphs. |
| `app/CMakeLists.txt` | Register new QML files (~line 532 block) and new C++ sources. |

New / edited / deleted QML:

| File | Change |
|------|--------|
| `app/qml/Commands/CommandModel.qml` | NEW — joins registry metadata + context bindings; the single provider the palette/menus/toolbars consume. |
| `app/qml/Commands/AppCommandBindings.qml`, `DashboardCommandBindings.qml`, `ProjectEditorCommandBindings.qml` | NEW — id → `{run, enabled, checked, visible}` entries (~5 lines each); host refs injected as required properties (as `ToolActions` does today). |
| `app/qml/Commands/CommercialCommandBindings.qml` | NEW — commercial-only entries; instantiated behind a `Loader` gated on `Cpp_CommercialBuild` (replaces per-button Loader+Component scaffolding; R14). |
| `app/qml/Widgets/CommandToolbar.qml` | NEW — Repeater-driven RibbonSection/ToolbarButton renderer over a layout tree. |
| `app/qml/MainWindow/Panes/Toolbar.qml` | 636 → thin host: `CommandToolbar` + layout `main-toolbar` (24 buttons / 4 sections today). |
| `app/qml/ProjectEditor/Sections/ProjectToolbar.qml` | 580 → thin host, layout `project-toolbar` (34 buttons / 8 sections today). |
| `app/qml/MainWindow/Panes/Dashboard/StartMenu.qml` | Static buttons + Tools submenu rendered from layout `start-menu`; Workspaces/Actions/Export submenus stay QML behind named slots; string-id if-chain dispatch (:679-705) deleted. |
| `app/qml/MainWindow/Panes/Dashboard/PaletteModel.qml` | `toolActions`/`extraTools` provider properties → `CommandModel` queries; activation for tools = `entry.run()` as today. 0027 contract intact. |
| `app/qml/MainWindow/Panes/Dashboard/Taskbar.qml` | Search popup reads the same `CommandModel`; delegates resolve icons at their own px via `Cpp_Misc_IconRegistry`. |
| `app/qml/Widgets/CommandPalette.qml` | Rows gain a right-aligned shortcut label (from registry display text). |
| `app/qml/MainWindow/MainWindow.qml` | 24 `Shortcut` blocks (:416-516) → one `Instantiator` over app-context commands (+ the few non-command shortcuts that remain, see Risks). |
| `app/qml/ProjectEditor/ProjectEditor.qml` | 7 `Shortcut` blocks (:146-172) → same treatment for the editor context. |
| `app/qml/MainWindow/Panes/Dashboard/ToolActions.qml`, `app/qml/MainWindow/MainWindowActions.qml`, `app/qml/ProjectEditor/ProjectEditorActions.qml` | DELETED (R9). |
| `app/qml/Widgets/MiniWindow.qml`, `app/qml/ProjectEditor/Views/FlowDiagram.qml`, remaining icon-literal QML | Literal → registry sweeps; FlowDiagram alone holds 90 refs (mechanical map swap). |

## Architecture & data flow

- **Icon catalog (pull, built once):** `IconRegistry` ctor runs a `QDirIterator` over
  `:/icons/`, parsing `<category>/<tier>/<name>.svg` into
  `QHash<category, QHash<name, sorted tier list>>` (~450 entries, trivial). `icon(cat,
  name, px)` returns the full `qrc:/icons/<cat>/<tier>/<name>.svg` URL for the smallest
  tier >= px, else the largest available (R2). Unknown key: `qWarning()` once per key
  (dedup set) + return the placeholder URL (R4). Files under non-tier paths (e.g.
  `buttons/`) are ignored by the scan.
- **Migration strategy (the keystone):** the audit produces `icon-map.csv`; `icon-migrate
  .py --apply` moves files and rewrites the qrc icons block emitting, for every old path,
  `<file alias="icons/<old>/<name>.svg">icons/<cat>/<tier>/<name>.svg</file>` — all 804
  existing references keep resolving unchanged. Consumer migration then proceeds
  surface-by-surface; the final task deletes the alias block and the grep gate (AC3)
  proves nothing needed it.
- **Command load:** `CommandRegistry` ctor parses the three command manifests + three
  layout manifests with the ThemeManager error pattern (`QJsonDocument::fromJson` +
  `qWarning` + skip; `ThemeManager.cpp:120-131`). `#ifndef BUILD_COMMERCIAL` drops
  `"pro": true` entries at load, so GPL surfaces never list them. Validation failures are
  loud but never fatal.
- **Command queries:** `commands(context)` returns `QVariantList` of maps `{id, title,
  tooltip, icon ("category/name"), shortcut, shortcutText, kind}`; `layout(surface)`
  returns the layout tree; titles/tooltips translated at query time via
  `QCoreApplication::translate("Commands", key)` (pattern:
  `ScriptTemplates.cpp:93-112`). `Misc::Translator::languageChanged`
  (`Translator.cpp:411-420`) → registry emits `commandsChanged` → surfaces re-pull (the
  QML engine's own `retranslate` hook at `ModuleManager.cpp:582-585` covers qsTr text;
  this covers registry-supplied text).
- **QML join:** `CommandModel` takes `{context, bindings}` and merges: for each registry
  command with a bindings entry, expose `{...metadata, run, enabled, checked, visible}`;
  commands without a bindings entry in this context are dropped (R11). Bindings entries
  are tiny QtObjects, so `enabled`/`checked` stay live QML bindings that delegates bind
  to directly.
- **Rendering:** `CommandToolbar` walks `layout(surface)`: section nodes → `RibbonSection`
  (title, `collapsedIcon` resolved via registry); item nodes → `ToolbarButton` with
  presentation hints from the node (`iconSize`, `horizontalLayout`, grid `columns`) and
  behavior from the joined entry; separator and named-slot nodes pass through. StartMenu
  renders static/tools nodes the same way; slot nodes (`workspaces`, `actions`, `export`)
  keep their existing QML implementations.
- **Shortcuts:** each host window instantiates `Instantiator { model:
  commandsWithShortcut(context) ; Shortcut { sequence; enabled: entry.visible &&
  entry.enabled && host gating; onActivated: entry.run() } }`. Context-sensitive
  sequences stay single commands whose `run()` branches (e.g. today's Ctrl+K branch at
  `MainWindow.qml:510-515` moves into the `palette.open` binding) — one enabled owner per
  sequence per window (R12). ProjectEditor is its own window, so its sequences are
  naturally scoped.
- **Taskbar model icon contract (R15):** `TaskbarModel` keeps its resolved-URL role for
  user-picked workspace icons (`IconEngine::resolveActionIconSource`, inline SVG — size
  independent) and gains a logical `iconId` role for fixed glyphs (folder, workspace
  fallback, widget icons). Delegate rule: `icon || IconRegistry.icon(iconId, ownPx)` —
  the taskbar resolves at 16-18, palette/switcher cells at 32, search rows at 18, from
  the same model data. `buildTreeModel`/`groupsChanged` queued-connection conventions are
  untouched (the palette still pulls on open).

## Hotpath & threading impact

- **Touches the hotpath?** No. Both registries are GUI-thread, touched at startup, on
  open/keystroke of navigation surfaces, and on model (re)population. No connection to
  frame/data signals; no `FrameReader`/`CircularBuffer`/`FrameBuilder`/Dashboard-draw
  code is modified. `--benchmark-hotpath` not required; AC6 keeps a perceived-startup
  check anyway.
- **New cross-thread signal/slot?** No. `Translator::languageChanged` →
  `CommandRegistry` is same-thread (both GUI).
- **New input to a cached hotpath flag?** No.
- **Timestamp ownership** — untouched.
- **Construction order:** neither registry enters `instantiateCoreModules()`
  (ModuleManager.cpp:617). Both are lazy Meyers singletons first touched at
  context-property registration (after `Translator` exists, satisfying translate-at-query
  anyway since translation happens per call, not in ctors). No ctor-closure exposure.

## Data model & persistence

None. No `Frame.h` `Keys::`, no project-JSON, no `widgetSettings`, no Sessions schema, no
`QSettings` changes. `UI::TaskbarSettings` (pinned buttons) is deliberately untouched —
aligning its button ids with command ids is named follow-up work, not scope creep here.

## API / SDK surface

None. No API handlers, no `EnumLabels`, no SDK regeneration (the sanitize-commit hook adds
a *translation-strings* generator, which is build tooling, not API surface).

## QML / UI

- Manifest schema (validated by `registry-verify.py` and at load):
  - Command: `id` (`scope.verb`, unique), `title`, `tooltip?`, `icon`
    (`"category/name"`), `shortcut?` (portable sequence string or `StandardKey.*` name),
    `kind` (`"action" | "toggle"`), `pro?` (bool).
  - Layout node: `{"type": "section", "title", "collapsedIcon", "items": [...]}` |
    `{"type": "command", "id", presentation hints...}` | `{"type": "separator"}` |
    `{"type": "group", "columns", "items"}` | `{"type": "slot", "name"}`.
- Presentation preserved, not redesigned (R8/AC9): `ToolbarButton`/`RibbonSection` remain
  the rendering primitives; active-driver bold-font behavior generalizes to "checked →
  bold" on toolbar buttons (matches today's per-button `font:` bindings); driver-select
  buttons become ten `toggle` commands (`driver.uart`...) with `checked` bound to
  `Cpp_IO_Manager.busType`.
- Palette rows: shortcut text right-aligned, low-opacity, elided first at narrow widths
  (R8 rules from 0027 still hold). Browse cells unchanged.
- Theme/RTL: colors via `Cpp_ThemeManager.colors`; layout manifests carry no colors or
  geometry beyond the existing component hints; RTL unaffected (same components).
- No ComboBoxes introduced (no restore race). Mode boundary: dashboard-context bindings
  keep the same `visible` guards `ToolActions` has today (QuickPlot/ConsoleOnly show the
  reduced set, exactly as now).

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Behavior binding | C++ dispatch / JS-in-JSON / **QML bindings per context** | **Maintainer-decided**: JSON stays declarative; closures keep direct `Cpp_*`/host access; no eval, no string reflection; reactivity is native QML. |
| Shortcut scope | declare+display only / **full migration** | **Maintainer-decided**: single source now; risk contained by doing it last with a per-sequence checklist (AC11). |
| Icon tree layout | `cat/name@tier.svg` / `cat/name/tier.svg` / **`cat/tier/name.svg`** | **Maintainer-decided**: same filename across tier folders = logical id; trivial scan; mirrors dashboard-small/large intuition. |
| Migration style | big-bang path swap / **qrc aliases + per-surface migration** | Aliases keep every old path resolving, so each surface is an independently verifiable diff and the final alias drop proves completeness. |
| Registry naming | `UI::Commands` / `CommandCenter` / **`UI::CommandRegistry`** | Namespace disambiguates from `API::CommandRegistry`; the parallel is intentional (same registry idiom, different domain). |
| Manifest granularity | one commands.json / **per-context + per-surface layouts** | Mirrors the context model, smaller diffs, GPL/pro filtering per entry either way. |
| Icon size resolution point | model resolves URLs / **delegates resolve via registry** | The same model feeds 16 px and 32 px surfaces (R15); per-delegate resolution is a cheap hash lookup at instantiation. |
| Dup canonical home | commands first / **code-bound categories first, commands last** | AMENDED during T1: C++ resolves `widgets`/`window`/`editor` names from fixed category strings (`dashboardWidgetIcon`, CSD, editor models), so those categories must retain their glyphs; command manifests are data and can reference any category (found via the `notification-log`/`notifications` byte-dup, which the original order would have silently broken). |
| Widget-icon helper | migrate 11 call sites / **keep `dashboardWidgetIcon` as shim** | Signature survives, body shrinks to enum→name + registry; call sites stay stable. |

## Risks & mitigations

- **GPL evaluation of commercial symbols (R14/AC13):** commercial bindings isolated in
  `CommercialCommandBindings.qml` behind a `Loader` gated on `Cpp_CommercialBuild`;
  `registry-verify.py` greps the non-commercial bindings files for `Cpp_Licensing_`/
  commercial-only ids; GPL console check in AC13.
- **Shortcut regressions (AC11):** migration ordered last; per-sequence checklist built
  from the 31 existing blocks *before* deletion; context-sensitive sequences stay single
  commands with branching `run()`; lint rejects duplicate sequences per context; any
  shortcut that is not a command (pure focus/navigation chords, if found during
  implementation) stays a hand block, explicitly listed.
- **Start menu parity (AC10):** dynamic submenus are not migrated (named slots); the
  static/tools cutover is one task with a before/after screenshot pass; the deleted
  if-chain's behavior moves into the same bindings the palette uses, so drift between
  menu and palette becomes impossible rather than merely unlikely.
- **Retranslation of registry-supplied text (R13/AC12):** `commandsChanged` on language
  switch + surfaces re-pull; verified live. Generated `CommandStrings.cpp` must be in the
  lupdate source list (`app/translations/translation_manager.py` uses an explicit
  `@list` — task includes it); maintainer runs the existing translation refresh flow.
- **Icon visual parity (R8/AC1/AC6):** consolidation only re-tiers where the audit says
  the glyph is identical; merge verdicts carry maintainer sign-off (Q2); before/after
  screenshots per migrated surface.
- **viewBox is export scale, not detail tier (maintainer, 2026-07-21):** a 40x40-viewBox
  export can be 16 px-detail art. Initial tiers from viewBox are provisional; task T20b
  re-tiers from ground truth (the pre-migration render sizes read out of git HEAD's QML
  `icon.width`/`iconSize` near each old literal) and rewrites call-site px arguments to
  display sizes. Interim call sites stay correct because nearest-at-or-above resolution
  serves the file wherever its tier folder sits.
- **Alias-drop timing:** aliases removed only after phases C-E land and the grep gate
  passes; `registry-verify.py` reports alias-hit counts (a qrc alias that no source
  references is safe to drop; one that is referenced blocks the drop task).
- **Silent-breakage classes** (`common-mistakes.md`): mode boundary — bindings keep
  today's guards; ComboBox restore race — N/A; macOS file-dialog reentrancy — command
  `run()` bodies move verbatim, no new dialog flows; queued-vs-direct — no new
  cross-thread wiring; `operator[]` inserts — registry lookups are const `value()` paths.
- **CMake registration misses:** every new QML file registered in the ~:532 block (house
  rule); missing registration fails visibly at startup, checked in the per-task verify.

## Test & verification plan

No pytest path exists for QML surfaces; verification is scripted lint + static sweeps +
maintainer observation, mapped per AC:

- **Scripted (implementer runs):** `python3 scripts/registry-verify.py` — icon-tree
  conformance, byte-dup sweep vs the 64-group baseline (AC2), qrc<->disk sync, alias
  audit, manifest schema/id/icon/shortcut lint (AC13 static half);
  `python3 scripts/generate-command-strings.py --check` (drift gate);
  `python3 scripts/code-verify.py --check` on every touched file;
  grep gates — no `qrc:/icons/`|`:/icons/` literals outside exempt sets (AC3), no
  `ToolActions|MainWindowActions|ProjectEditorActions` references (AC7), no
  `WorkspaceSwitcherOverlay`-style dead refs after deletions.
- **Static review:** `qt-cpp-review` on the C++ diff (registries, Taskbar, SerialStudio,
  CSD, project-editor models) before handoff.
- **Maintainer observations (app):** AC1 (palette/switcher crisp at 32 px, taskbar/start
  unchanged), AC4 (scratch icon), AC5 (bad id → warning + placeholder), AC6 (full visual
  pass + startup feel), AC8 (scratch command end-to-end), AC9 (toolbar parity shots +
  manifest reorder demo), AC10 (start menu parity), AC11 (31-sequence shortcut checklist
  + palette shortcut display + no ambiguity warnings), AC12 (live language switch), AC13
  (GPL console clean).
- **Hotpath:** not touched; no `--benchmark-hotpath` run required.
- **Commit:** `python3 scripts/sanitize-commit.py` before any commit (maintainer-gated),
  which now also regenerates `CommandStrings.cpp`.
