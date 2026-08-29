---
spec: 0005-console-dedup-search
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-11
---

# Plan 0005 — Console duplicate-line collapsing and in-console search

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this — a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Both features live in the view layer, `Widgets::Terminal`, which already owns the display
line buffer (`m_data`, `MAX_LINES = 1000`), the paint pipeline, selection, and scrolling —
the receive path (`Console::Handler::hotpathRxData` → `m_pendingDisplay` → UI-tick
`displayString`) is untouched, so the raw text buffer, console export, and every downstream
consumer keep full fidelity for free. Duplicate collapsing runs at line-completion inside
the widget's append path (UI-tick rate, batched text): when a completed non-empty line
equals the previous line (timestamp prefix excluded), the new row is dropped and a parallel
per-row repeat counter increments; the "× N" badge is painted after the line text at paint
time and never enters `m_data`, so copy/selection are unaffected. Search is a small match
engine in the same widget (query + case flag → list of (row, col) matches over `m_data`),
exposed to QML via properties/slots; a QML search bar overlay in `Terminal.qml` opens on
StandardKey.Find, and match highlights are painted in the existing per-segment paint pass.
Two persisted toggles (`collapseDuplicates`, `searchCaseSensitive`) live in
`Console::Handler` beside the other console options.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/Console/Handler.h` | Two new `Q_PROPERTY(bool)`: `collapseDuplicates`, `searchCaseSensitive`; getters, setter slots, change signals; two `bool` members. |
| `app/src/Console/Handler.cpp` | Ctor loads `Console/CollapseDuplicates` (default false) and `Console/SearchCaseSensitive` (default false) from `QSettings`; guarded setters persist + emit. |
| `app/src/UI/Widgets/Terminal.h` | Dedup: `m_repeatCounts` (`QList<int>`, lockstep with `m_data`), `m_collapseDuplicates` cache, line-completion hook decl, badge paint helper. Search: `m_searchQuery`, `m_searchCaseSensitive`, `m_searchMatches` (`QList<QPoint>` col,row), `m_searchCurrent`, `m_searchDirty`; `Q_PROPERTY` for `searchMatchCount`, `searchCurrentMatch`, `searchActive`; slots `setSearchQuery`, `searchNext`, `searchPrevious`, `clearSearch`. |
| `app/src/UI/Widgets/Terminal.cpp` | Dedup logic at line completion in the `processText('\n')` / `appendString` flow; lockstep trim of `m_repeatCounts` (matching the existing `m_colorData` pattern) in `appendString`/`initBuffer`/erase paths; badge painting in `paintTextContent`; search: rescan-on-dirty match computation, match navigation (wrap, `setAutoscroll(false)` + `setScrollOffsetY` on jump), match highlight painting before text. |
| `app/qml/Widgets/Dashboard/Terminal.qml` | "Collapse Duplicates" `CheckBox` in the output-options row (bound to `Cpp_Console_Handler.collapseDuplicates`); search bar overlay (TextField, prev/next buttons, case toggle, "N of M" label, close); `Shortcut` on `StandardKey.Find` gated like Copy/SelectAll; Escape closes and refocuses the terminal. |
| `app/src/API/Handlers/ConsoleHandler.cpp` | Optional parity (decision below): `console.setCollapseDuplicates` + include both new flags in `console.getConfig`. |
| `doc/claude/specs/0005-console-dedup-search/*` | Workflow artifacts. |

No new files. `TerminalWidget` registration (`ModuleManager.cpp:528`) is unchanged.

## Architecture & data flow

Current flow (unchanged): driver → `IO::ConnectionManager` → `Console::Handler::hotpathRxData`
/ `hotpathRxDeviceData` (per-chunk, main thread) → line assembly + timestamp stamping into
`m_textBuffer` (raw, feeds export) and `m_pendingDisplay` → `Misc::TimerEvents::uiTimeout`
flush → `displayString(QString)` → `Widgets::Terminal::append` (VT-100 state machine) →
`appendString` → `m_data` rows → `paint()`.

**Dedup insertion point:** the moment a line *completes* — `processText` on `'\n'` — with
`m_collapseDuplicates` on and `vt100emulation()` off (R7): compare the just-completed row
against the row above using a comparison that skips a well-formed `HH:mm:ss.zzz -> ` prefix
on both sides when `Console::Handler::showTimestamp()` is on (R3; fixed 16-char shape
check: digits/colons/dot at fixed offsets, so a data line that merely looks similar never
false-strips). Empty / whitespace-only lines never collapse (keeps the welcome guide and
blank spacing intact). On match: drop the new row, reuse it as the fresh cursor row,
`++m_repeatCounts[prev]`, `m_stateChanged = true`. On mismatch: normal append, count 1. The
widget caches `m_collapseDuplicates` from `Console::Handler` at ctor and via its change
signal (same pattern as `fontChanged`).

`m_repeatCounts` mirrors `m_data` row-for-row and is trimmed/erased in lockstep in
`appendString`'s MAX_LINES drop, `initBuffer()`, and the CSI erase-display paths — exactly
the `m_colorData` discipline already documented in the erase code.

**Search:** `setSearchQuery(query, caseSensitive)` stores state and marks `m_searchDirty`;
matches are (re)computed by a single `m_data` scan (`QString::indexOf`, ≤1000 lines, only
while `searchActive`) — on query edit immediately, and on buffer change lazily via
`m_searchDirty` consumed in the next UI-tick `update()`, so arriving data costs one bool
store. `searchNext()`/`searchPrevious()` wrap, set the current index, call
`setAutoscroll(false)` and `setScrollOffsetY` so the current match's visual row is on
screen (R12). Match highlighting paints per visible segment before the text pass (all
matches: theme search color; current match: distinct color), same segment-walk geometry as
`drawSegmentSelection`. `clearSearch()` empties state and repaints. Since dedup keeps one
displayed row per run, a collapsed line naturally matches once (R13).

All objects are main-thread; every new connection is default (direct) same-thread.

## Hotpath & threading impact

- **Touches the hotpath?** No. `FrameReader`, `CircularBuffer`, `FrameBuilder`, `Dashboard`,
  and the span fast lane are untouched. `Console::Handler::hotpathRxData` (per-chunk) gains
  zero instructions; the two new Handler booleans are read only by the widget. Dedup +
  search work runs in `Widgets::Terminal`, which consumes the *batched* `displayString`
  flush at `Misc::TimerEvents::uiTimeout` rate — display-rate, not data-rate. With both
  toggles off the added cost is one branch per completed line. `--benchmark-hotpath` runs
  headless (no Terminal widget instantiated), but AC5 still runs it as the regression gate.
- **New cross-thread signal/slot?** No. Handler and Terminal are both main-thread; new
  connections (`collapseDuplicatesChanged` → widget cache refresh) are same-thread direct.
- **New input to a cached hotpath flag?** No. Neither flag feeds `FrameBuilder`/`Dashboard`
  caches. (The widget-side `m_collapseDuplicates` cache is refreshed by its change signal —
  same-thread, wired at ctor.)
- **Timestamp ownership** — unchanged; the console keeps stamping display text in
  `Handler::append`/`appendToDevice`; dedup only *compares around* the stamp, never writes
  one.

## Data model & persistence

Two new `QSettings` keys, following the existing `Console/*` family: `Console/CollapseDuplicates`
(bool, default false — R1) and `Console/SearchCaseSensitive` (bool, default false). No
`Frame.h` keys, no project-JSON, no schema, no migration. Search query/current-match are
per-widget-instance runtime state, deliberately not persisted.

## API / SDK surface

Optional parity only (tradeoff below): `console.setCollapseDuplicates` in
`API::Handlers::ConsoleHandler` mirroring `setShowTimestamp`'s shape, and both new flags
added to `console.getConfig`'s payload. No SDK regeneration beyond what
`sanitize-commit.py` already does. Search is view-local widget state and gets no API
surface.

## QML / UI

- **Toggle:** one `CheckBox { text: qsTr("Collapse Duplicates") }` in the existing
  output-options `RowLayout` in `Terminal.qml`, bound with the same guarded
  `onCheckedChanged` pattern as `timestampCheck`. Greyed (`enabled: false`) while
  `Cpp_Console_Handler.vt100Emulation` is on — the "visibly unavailable" branch of R7,
  matching how `ansiColorsCheck` gates on VT-100 today.
- **Search bar:** a compact overlay anchored to the terminal's top-right (inside the
  `TerminalWidget` item, above the border `Rectangle`): `TextField` (mono font, console
  palette), "Aa" case `ToolButton` (checkable, bound to `Cpp_Console_Handler.searchCaseSensitive`),
  `Label` "%1 of %2" (or `qsTr("No results")`), prev/next `IconButton`s, close button.
  Enter/Shift+Enter in the field → `searchNext()`/`searchPrevious()`; Escape → close +
  `terminal.forceActiveFocus()`. Bar visibility drives `terminal.clearSearch()` on close.
- **Shortcut:** `Shortcut { sequences: [StandardKey.Find] }` (Ctrl+F / Cmd+F per platform,
  R8) gated `enabled: terminal.activeFocus && !root.vt100Interactive` — the exact gate the
  existing Copy/SelectAll shortcuts use, which also keeps the two live `Terminal.qml`
  instances (Console pane + dashboard tool window) from ever double-binding the sequence
  (common-mistakes: ambiguous-shortcut silent no-op). Search therefore ships in **both**
  the Console pane and the dashboard Terminal window — same component, resolving the
  spec's first open question.
- **Badge rendering:** painted in `paintTextContent` after the final wrapped segment of a
  row with `m_repeatCounts[row] > 1`: small rounded rect, `console_highlight`-derived
  fill, `× N` text in the console font at a smaller point size. Pure paint-time decoration:
  never in `m_data`, so selection geometry, `copy()`, and `positionToCursor` are untouched
  (R4).
- Theme colors come from `Misc::ThemeManager` (`console_*` group; reuse `console_highlight`
  and `console_text` — no new theme keys unless review wants a dedicated search color).
- All new strings `qsTr()`-wrapped; **no `.ts`/`.qm` files are touched** (user regenerates).

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Dedup placement | (a) Handler stream rewrite; (b) interposer filter model; (c) view-level collapse in `Widgets::Terminal` | **(c)** — (a) adds per-chunk hotpath work and poisons the raw buffer export reads (R4); (b) duplicates line-assembly state the VT-100 machine already owns; (c) is UI-tick rate, keeps raw data lossless by construction, and owns the paint surface the badge needs. |
| Search match updates on live data | Incremental per-append patching vs. dirty-flag full rescan | **Dirty-flag rescan** — `m_data` is ≤1000 short lines; a rescan at UI-tick rate is microseconds, while incremental patching must handle row trimming, CSI erases, and dedup row reuse — high bug surface for zero measurable win. |
| Search in VT-100 interactive mode | Allow vs. disable | **Allow** (amended 2026-07-11 after maintainer testing) — originally disabled to preserve Ctrl+F = 0x06 forwarding, but macOS maps Cmd to Qt's ControlModifier, so Cmd+F fell through to the VT-100 translator and sent a stray 0x06. The Find shortcut now always wins while the terminal or search field has focus; 0x06 no longer reaches the device via the keyboard shortcut. |
| Toggle-state home | Per-widget QML `Settings` vs. `Console::Handler` + `QSettings` | **Handler** — it already owns every console option with the same persist-in-setter pattern; both Terminal instances stay in sync for free. |
| API parity for the new toggle | Skip vs. add `console.setCollapseDuplicates` + `getConfig` fields | **Add** (small, mirrors `setShowTimestamp`) — `console.getConfig` advertises "all console settings"; letting it silently drift from the real setting set costs API users more than the ~30 lines cost us. Flagged here so review can strike it if it reads as scope creep. |
| Device-switch counter fidelity | Persist per-device repeat counts vs. recompute on replay | **Recompute on replay** — switching devices already rebuilds the view from the 10 KB per-device retained buffer; counts beyond that buffer are unknowable without a new per-device side structure. R5's "current run" survives MAX_LINES trimming (counter rides the row); a device *switch* is a view rebuild. Named as the one R5 caveat for the maintainer to accept. |

## Risks & mitigations

- **Parallel-array drift (`m_repeatCounts` vs `m_data`)** — the exact class of bug the
  existing `m_colorData` lockstep-trim comment guards. Mitigation: every site that erases
  or resizes `m_data` (append trim, `initBuffer`, CSI erase-display 0/1, `clear`) updates
  `m_repeatCounts` in the same statement block; task list will enumerate all sites found by
  grep on `m_data.erase|m_data.resize|m_data.clear`.
- **False timestamp strip** (R3 comparison) — fixed-shape check (16 chars,
  `dd:dd:dd.ddd -> `) applied only when `showTimestamp()` is on; a line that fails the
  shape check compares whole.
- **Ambiguous QML shortcut** (common-mistakes) — single `Shortcut` per `Terminal.qml`
  instance, gated on `terminal.activeFocus`, which is exclusive across instances.
- **Badge vs. selection geometry** — badge is paint-only and drawn past the end of the last
  segment; `positionToCursor`/`copy()` read `m_data` and never see it. Risk of badge
  overlapping wrapped text at exactly-full lines: clamp badge to the right border and skip
  it when the last segment fills the row width.
- **Search index invalidation on trim/erase** — rescan-on-dirty recomputes from scratch;
  current-match index clamps to the new match count (wraps to last).
- **Welcome-guide regression** — dedup skips empty/whitespace lines and the guide has no
  consecutive identical non-empty lines; verified by loading it with the toggle on.
- **Scope discipline** — files above are the lane; the API-parity row is the only
  borderline item and is pre-flagged for review.

## Test & verification plan

- **Unit (I can run):** none applicable — no JS-parser logic. Static checks only.
- **Integration (maintainer runs, app + API server up):** extend
  `tests/integration/` console coverage with a case that sets
  `console.setCollapseDuplicates` via API (if the parity row survives review), streams
  duplicate lines over TCP, and asserts `console.getConfig` round-trips the flag — display
  collapse itself is view-state and stays a manual observation.
- **Maintainer observations (map to ACs):**
  - AC1/AC3: UDP Function Generator + a repeated-line stream; toggle on/off; timestamps on.
  - AC2: restart; toggle persists.
  - AC4: console export on with collapsing on → exported file has full duplicates; copy a
    collapsed line → no badge text.
  - AC5: `--benchmark-hotpath` unchanged (CI gate); UDP Function Generator at max rate with
    collapsing on → no stall/loss.
  - AC6: two-source project; per-device counters; device switch.
  - AC7: VT-100 emulation on → toggle greyed, rendering identical to today.
  - AC8–AC10: Ctrl+F/Cmd+F flow, counts, wrap navigation, case toggle, Escape; live-stream
    search with autoscroll suspension; collapsed line counted once.
- **Hotpath:** `--benchmark-hotpath` via CI (`ci.yml`) — no code on the measured path, gate
  must stay green.
- **Static:** `python scripts/code-verify.py --check` on all touched files; `qt-cpp-review`
  before handoff; `python scripts/sanitize-commit.py` before commit.
