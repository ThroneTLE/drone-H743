---
spec: 0005-console-dedup-search
phase: tasks
status: approved     # draft -> approved (gate before /ss-implement)
updated: 2026-07-11
---

# Tasks 0005 — Console duplicate-line collapsing and in-console search

> **Phase 3 of 4 — the ordered checklist.** Decompose [`plan.md`](./plan.md) into units that
> are small, ordered, and *individually verifiable* — each one a coherent diff a reviewer
> could read in isolation. `/ss-implement` works this list top to bottom and keeps the status
> boxes current. Gate: do not start `/ss-implement` until a human marks this `approved`.

## Conventions

- One task = one focused, reviewable change. If a task touches >3 files or needs a paragraph
  to describe, split it.
- **Verify** is how *this* unit is confirmed before moving on — usually
  `python scripts/code-verify.py --check <files>`, plus a test or a read-back where one fits.
- **Deps** lists task IDs that must land first.
- Order so the tree compiles (conceptually) after each task where practical.

## Tasks

### T1 — Handler toggles: `collapseDuplicates` + `searchCaseSensitive`

- **Files:** `app/src/Console/Handler.h`, `app/src/Console/Handler.cpp`
- **Does:** Adds two `Q_PROPERTY(bool)` with getters, guarded setter slots, and change
  signals, following the exact `setShowTimestamp` pattern (guard → assign → persist →
  `Q_EMIT`). Ctor loads `Console/CollapseDuplicates` and `Console/SearchCaseSensitive`
  (both default `false`) from `QSettings`. Header order per code-style (`Q_PROPERTY` block,
  `signals:`, `public slots:`); ctor-init-list only, no in-header init. Handler is
  constructed pre-AppState (composition root) — the new code touches nothing beyond
  `m_settings`, so the ctor closure stays inert.
- **Verify:** `python scripts/code-verify.py --check app/src/Console/Handler.h
  app/src/Console/Handler.cpp`; read-back that setters are guarded and persist.
- **Deps:** none
- [x] done

### T2 — Terminal dedup core: repeat counts + line-completion collapse

- **Files:** `app/src/UI/Widgets/Terminal.h`, `app/src/UI/Widgets/Terminal.cpp`
- **Does:** Adds `m_repeatCounts` (`QList<int>`, row-for-row with `m_data`) and the
  collapse-at-line-completion logic in `processText('\n')`. **Binding invariants:**
  (1) `m_repeatCounts` mutates in lockstep at every `m_data` mutation site —
  `appendString` MAX_LINES drop (Terminal.cpp:1261), `initBuffer` (1349-1350), CSI
  erase-display cases 0/1 (1664, 1673), `replaceData` grow (2022) — same discipline as the
  existing `m_colorData` lockstep-trim comment; (2) collapse runs only when the cached
  `m_collapseDuplicates` is on **and** `vt100emulation()` is off (R7), so the toggle-off
  cost is one branch per completed line; (3) comparison strips the fixed-shape 16-char
  `HH:mm:ss.zzz -> ` prefix only when `Console::Handler::showTimestamp()` is on and both
  lines pass the digit/colon/dot shape check (R3); (4) empty/whitespace-only lines never
  collapse. On duplicate: drop the new row, reuse it as the cursor row, increment the
  previous row's count, `m_stateChanged = true`. Cache `m_collapseDuplicates` from the
  Handler at ctor + on its change signal (same-thread, like `fontChanged`).
- **Verify:** `python scripts/code-verify.py --check app/src/UI/Widgets/Terminal.h
  app/src/UI/Widgets/Terminal.cpp`; grep read-back that all six `m_data` mutation sites
  touch `m_repeatCounts` in the same block.
- **Deps:** T1
- [x] done

### T3 — Badge painting ("× N")

- **Files:** `app/src/UI/Widgets/Terminal.cpp`
- **Does:** Paints the repeat badge in `paintTextContent` after the final wrapped segment
  of any visible row with `m_repeatCounts[row] > 1`: rounded rect from
  `console_highlight`, count text in the console font at reduced point size. Paint-only —
  nothing enters `m_data`, so `copy()`, selection, and `positionToCursor` stay untouched
  (R4). Clamp to the right border; skip when the last segment fills the row width.
- **Verify:** `python scripts/code-verify.py --check app/src/UI/Widgets/Terminal.cpp`;
  read-back: badge code touches only painters, no buffer writes.
- **Deps:** T2
- [x] done

### T4 — Search engine in `Widgets::Terminal`

- **Files:** `app/src/UI/Widgets/Terminal.h`, `app/src/UI/Widgets/Terminal.cpp`
- **Does:** Adds search state (`m_searchQuery`, `m_searchCaseSensitive`, `m_searchMatches`
  as `QList<QPoint>` (col,row), `m_searchCurrent`, `m_searchDirty`), `Q_PROPERTY`s
  `searchMatchCount` / `searchCurrentMatch` / `searchActive` (+ `searchResultsChanged`
  signal), and slots `setSearchQuery(QString, bool)`, `searchNext()`, `searchPrevious()`,
  `clearSearch()`. **Binding invariants:** (1) arriving data costs one bool store —
  buffer changes only set `m_searchDirty`, consumed at the next UI-tick `update()`;
  (2) full rescan over `m_data` (≤1000 lines) only while `searchActive`; (3) navigation
  wraps, clamps `m_searchCurrent` after a rescan shrinks the match list, and calls
  `setAutoscroll(false)` + `setScrollOffsetY` so the current match's visual row is on
  screen (R12) — visual row accounts for line wrapping like `positionToCursor` does.
- **Verify:** `python scripts/code-verify.py --check app/src/UI/Widgets/Terminal.h
  app/src/UI/Widgets/Terminal.cpp`; read-back: no per-append scan, no allocation while
  search inactive.
- **Deps:** T2 (row indices must be post-dedup so a collapsed line matches once, R13)
- [x] done

### T5 — Search highlight painting

- **Files:** `app/src/UI/Widgets/Terminal.cpp`
- **Does:** Paints match highlights for the visible line range before the text pass using
  the same segment-walk geometry as `drawSegmentSelection`: all matches in a
  `console_highlight`-derived fill, current match visually distinct (R11). No repaint cost
  when `searchActive` is false.
- **Verify:** `python scripts/code-verify.py --check app/src/UI/Widgets/Terminal.cpp`.
- **Deps:** T4
- [x] done

### T6 — QML: Collapse Duplicates checkbox

- **Files:** `app/qml/Widgets/Dashboard/Terminal.qml`
- **Does:** Adds `CheckBox { text: qsTr("Collapse Duplicates") }` to the output-options
  row with the guarded `onCheckedChanged` pattern of `timestampCheck`, bound to
  `Cpp_Console_Handler.collapseDuplicates`; `enabled: !Cpp_Console_Handler.vt100Emulation`
  (visibly-unavailable branch of R7, mirroring `ansiColorsCheck`). QML comment-sandwich
  style; string translatable.
- **Verify:** `python scripts/code-verify.py --check app/qml/Widgets/Dashboard/Terminal.qml`.
- **Deps:** T1
- [x] done

### T7 — QML: search bar overlay + Find shortcut

- **Files:** `app/qml/Widgets/Dashboard/Terminal.qml`
- **Does:** Adds the top-right search overlay (mono `TextField`, checkable "Aa" button
  bound to `Cpp_Console_Handler.searchCaseSensitive`, "%1 of %2" / `qsTr("No results")`
  label, prev/next/close buttons) and one `Shortcut { sequences: [StandardKey.Find] }`.
  **Binding invariants:** (1) shortcut gated
  `enabled: terminal.activeFocus && !root.vt100Interactive` — the Copy/SelectAll gate, so
  the two live Terminal instances never double-bind the sequence (ambiguous-shortcut
  silent no-op from common-mistakes); (2) Enter/Shift+Enter → `searchNext()` /
  `searchPrevious()`; Escape closes, calls `terminal.clearSearch()` and
  `terminal.forceActiveFocus()` (R8); (3) closing the bar always clears search state.
  Numbered `.arg()` placeholders in translated strings only — never `%n`.
- **Verify:** `python scripts/code-verify.py --check app/qml/Widgets/Dashboard/Terminal.qml`;
  read-back: exactly one `StandardKey.Find` binding in the file.
- **Deps:** T4, T5, T6 (shares the options row / overlay z-order)
- [x] done

### T8 — API parity: `console.setCollapseDuplicates` + `getConfig` fields

- **Files:** `app/src/API/Handlers/ConsoleHandler.h`, `app/src/API/Handlers/ConsoleHandler.cpp`
- **Does:** Registers `console.setCollapseDuplicates` (static handler mirroring
  `setShowTimestamp`: missing-param error, bool param, echo result) and adds
  `collapseDuplicates` + `searchCaseSensitive` to `console.getConfig`'s payload. Free
  (non-commercial) surface — no `#ifdef BUILD_COMMERCIAL`.
- **Verify:** `python scripts/code-verify.py --check app/src/API/Handlers/ConsoleHandler.h
  app/src/API/Handlers/ConsoleHandler.cpp`.
- **Deps:** T1
- [x] done

### T9 — Integration test: toggle round-trip over the API

- **Files:** `tests/integration/test_console_configuration.py`
- **Does:** Adds a test that sets `console.setCollapseDuplicates` true/false via the API
  client and asserts `console.getConfig` round-trips both new flags. Follows the file's
  existing fixture/marker conventions (read `tests/README.md` first). Maintainer runs it
  (needs the app + API server); I only author it.
- **Verify:** read-back against existing tests in the file for convention match;
  maintainer runs `pytest tests/integration/test_console_configuration.py -v`.
- **Deps:** T8
- [x] done

### T10 — Self-review sweep + handoff

- **Files:** none new (whole diff)
- **Does:** Re-reads the full diff against the plan's file lane (no foreign files, no
  scope creep); counterfactual check named in chat (most-at-risk rule + concrete evidence
  it holds — expected: the `m_repeatCounts` lockstep invariant); runs the static pipeline
  and `qt-cpp-review` on the C++ diff; leaves maintainer AC checklist (AC1-AC10) in chat.
- **Verify:** `python scripts/code-verify.py --check` clean on all touched files;
  `qt-cpp-review` findings addressed or noted; `python scripts/sanitize-commit.py` run.
- **Deps:** T1-T9
- [x] done

## Definition of Done

- [x] Every code-verifiable acceptance criterion is implemented; AC1-AC10 runtime
      observations listed for the maintainer at handoff (app must be run — see chat).
- [x] `python scripts/code-verify.py --check` is clean on all changed files (no new errors).
- [x] `qt-cpp-review` run on the C++ diff (6 agents); 4 confirmed + 3 actionable findings
      all fixed in-tree: counter saturation at INT_MAX, per-paint QFontMetrics hoist,
      cached badge font/metrics, `console.setSearchCaseSensitive` parity, tick refresh
      gated on `isVisible()`, `lineContentView(QStringView)`, match-cursor advance before
      the empty-row skip, +2 invariant asserts.
- [x] `--benchmark-hotpath` not regressed (CI gate; no code on the measured path — Agent 3
      confirmed `hotpathRxData`/`appendToDevice` byte-identical).
- [x] Relevant `pytest` target identified for the maintainer:
      `tests/integration/test_console_configuration.py`.
- [x] `python scripts/sanitize-commit.py` run; working tree clean of lint debt.
- [x] Diff is *what was asked, and only that* — plan lane + `search_index.json` (regenerated
      by the sanitize pipeline itself); no foreign files, no `.ts`/`.qm` regenerated.
- [x] `spec.md` status set to `done` (implementation; runtime ACs pending maintainer run).
