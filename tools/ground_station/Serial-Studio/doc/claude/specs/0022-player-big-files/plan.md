---
spec: 0022-player-big-files
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-19
---

# Plan 0022 — Big-file CSV & MDF4 replay

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Clone the Sessions player's proven split — resident timestamp index + on-demand data access +
a `PlayerLoaderWorker` thread — into both file players, each with the storage model that fits
its format. CSV keeps the file on disk via `QFile::map` and holds only a row-offset vector and
a per-row seconds vector, built in the background with `DSP::simdForEachByteMatch` + fast_float;
rows are split into cells on demand by a new byte-level RFC-4180 splitter that mirrors
`splitReplayRow` exactly. MDF4 keeps its full decode (maintainer decision) but moves it onto a
worker that owns the `MdfReader` and every `mdf::*` pointer, and lands the samples in contiguous
per-channel columnar vectors instead of today's map-of-per-row-vectors. A new replay-only
FrameBuilder ingestion surface (UTF-8 span cells for CSV, typed numeric/text cells for MDF4)
writes `Dataset::numericValue` directly and the display string in place, eliminating the
per-frame `QStringList` churn and the `double -> QString -> double` round trip while leaving
the live parse path byte-for-byte untouched.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/CSV/Player.h/.cpp` | Storage rework: drop `m_csvData`; mapped file + row-offset/seconds vectors; frontier-clamped playback/scrub; virtualized header-skip, interval, and datetime-column modes; worker wiring + progress properties. |
| `app/src/CSV/PlayerLoaderWorker.h/.cpp` (new) | Background indexer: newline scan (`simdForEachByteMatch`), row validity, per-row seconds (fast_float numeric or fixed-format datetime), batched progress signals, atomic cancel. |
| `app/src/MDF4/Player.h/.cpp` | Columnar caches (per-channel contiguous vectors + sorted frame index + packed active bits); numeric-lane injection replacing `buildRowCells`'s per-cell `QString::number`; worker wiring + progress properties. |
| `app/src/MDF4/PlayerLoaderWorker.h/.cpp` (new) | Worker-owned `MdfReader` + observers; decode into the columnar payload; progress from `NofSamples()` totals; single payload handoff; atomic cancel between records. |
| `app/src/DataModel/FrameBuilder.h/.cpp` | Replay-only entry points `replayChannelSpans` (UTF-8 views) and `replayChannelsTyped` (numeric/text cells) beside `replayChannels`; shared apply core; live-path branches of `applyDatasetValue` untouched. |
| `app/src/DataModel/Scripting/FrameParserPipeline.h/.cpp` | Byte-level RFC-4180 row splitter (`QByteArrayView` -> cell views) with semantics identical to `splitReplayRow`; existing UTF-16 splitter kept for its remaining callers. |
| `app/CMakeLists.txt` | Register the four new files. |
| Player transport QML (exact file pinned in tasks; candidates: `app/qml/MainWindow/Panes/Toolbar.qml` and the pane hosting the timeline) | Thin indexing-progress indication bound to the new properties. |

Sessions player files are deliberately not touched (spec non-goal); the shared-base extraction
of the copy-pasted scrub machinery across the three players is named future work, not part of
this pass.

## Architecture & data flow

**CSV open:** `openFile(path)` runs the existing dialogs synchronously on a small foreground
read (header row + first data row only): header registration, timestamp-format detection, and
the interval / datetime-column prompts all resolve before the worker starts. The player then
maps the file (`QFile::map`; the `QFile` member stays open, so `isOpen()` semantics are
unchanged) and starts `CSV::PlayerLoaderWorker` on its own `QThread` (mirroring
`Sessions::PlayerLoaderWorker`): one pass over the mapped bytes building `rowOffsets`
(`QVector<quint64>`, one entry per valid row) and `rowSeconds` (`QVector<double>`), emitting
batched `indexProgress(rowsIndexed, bytesDone)` queued signals. The main thread appends the
batches, so `frameCount()` grows monotonically — that growing count *is* the frontier.
`openChanged` fires at accept time (R1); play/scrub clamp to the frontier (R2).

**CSV row access:** `rowCells(i)` slices the mapped bytes at `[rowOffsets[i], rowOffsets[i+1])`
and splits with the byte-level splitter into `QByteArrayView` cells. The three legacy
mutations become virtual: the header is excluded from `rowOffsets`; interval mode computes
`rowSeconds[i] = i * interval` with no synthetic cells (display formats from seconds); the
datetime-column mode records the timestamp column index and the cell accessor skips/redirects
it. ProjectFile injection passes the views to `replayChannelSpans`; QuickPlot slices the raw
row bytes after the timestamp field verbatim (datetime-column mode, the rare rebuild case,
joins cells as today). `buildSeekWindow` parses views directly with
`SerialStudio::toDouble(QByteArrayView)` — no `QString` materialization anywhere on the scrub
path.

**MDF4 open:** the worker owns `MdfReader`, `ReadEverythingButData`, the observer decode, and
the ns-quantized cache-key merge (kept bit-identical — it is the multi-channel-group frame
contract). It accumulates directly into the columnar payload: per-channel
`std::vector<double>` / `std::vector<QString>` (text channels only), packed per-channel active
bits, plus the sorted frame index. Progress = records seen / sum of `NofSamples()` (resolving
the spec's open question: fractional progress is real). On completion one
`std::shared_ptr` payload crosses to the main thread via queued signal; the frontier jumps
0 -> 100% (single handoff — R2 is satisfied degenerately for MDF4: the transport clamps at
frame 0 until decode lands; this interpretation is called out for review). `closeFile` and
re-open set the worker's atomic cancel and join; no `mdf::*` pointer ever crosses threads.

**Numeric replay lane:** ground truth (verified): `Dataset` already carries
`numericValue`/`isNumeric` beside `value`; the dashboard per-frame path consumes only the
double; the only replay parse sites are the two `SerialStudio::toDouble` calls in
`applyDatasetValue` (`FrameBuilder.cpp:1337/:1353`); value strings must stay populated because
every dataset has string targets (`m_lastFrame` copies, DataGrid, LED). Therefore:
`replayChannelSpans` assigns `dataset.value` via `assign_utf8_in_place` (allocation-free steady
state) and parses the view once; `replayChannelsTyped` sets `numericValue`/`isNumeric` directly
and formats the display string in place (`%.10g` into a stack buffer +
`assign_utf8_in_place`, chosen to match today's `QString::number(v, 'g', 10)` — see
Tradeoffs). Both funnel into one shared replay-apply helper; `applyDatasetValue`'s live
branches are not edited. Timing is unchanged: players still anchor the steady base and stamp
recorded deltas; `publishReplayFrame` still fans out to dashboard + read-only observers only.

## Hotpath & threading impact

- **Touches the hotpath?** FrameBuilder gains replay-only functions; the live parse path
  (`trySpanLane`, `applyDatasetValues` live branches, `applyDatasetValue`'s non-replay code)
  is not edited. `ss-hotpath` invoked; `--benchmark-hotpath` must pass unchanged (AC5). The
  replay lane itself follows the same no-alloc discipline (`acquireFrame` slots, in-place
  string writes) it has today.
- **New cross-thread signal/slot?** Yes: two loader workers on dedicated `QThread`s,
  queued-connection signals carrying `std::shared_ptr` payloads / plain progress ints —
  exactly the `Sessions::PlayerLoaderWorker` pattern (registered metatypes, atomic cancel,
  join-with-timeout on shutdown). Injection, seeks, and all FrameBuilder/Dashboard calls stay
  main-thread.
- **New input to a cached hotpath flag?** No. `openChanged`/`playerStateChanged` fire from the
  same places with the same meanings; the existing player lambdas keep the caches fresh. The
  only semantic shift — `openChanged` now fires before indexing completes — does not feed any
  cached flag differently (player-open flags are boolean).
- **Timestamp ownership:** unchanged — players stamp recorded deltas via the anchored steady
  base; nothing downstream re-stamps.

## Data model & persistence

None. No `Keys::` additions, no schema or writer-version changes, no project-JSON or
`widgetSettings` shape changes, no Sessions DB involvement. Both players read existing files
exactly as before.

## API / SDK surface

None. No new handlers, slugs, or SDK surface. (`dashboard.getData` continues to serialize
`m_lastFrame` strings, which the numeric lane keeps populated.)

## QML / UI

Two new Q_PROPERTYs per player (`indexing` bool, `indexProgress` 0..1, notify via existing or
one new signal) and a thin progress indication on the transport UI. `frameCount`/`progress`
already notify via `playerStateChanged`/`timestampChanged`, so the growing CSV frontier
animates the existing timeline without new plumbing. No ComboBox/restore-race surface.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| CSV storage | Mapped-Spans; SQLite-Import (temp session DB, reuse Sessions player); Resident-Columnar | **Mapped-Spans** — bounded memory + verbatim text parity for free; SQLite-Import explodes rows-per-cell (10M x N readings), doubles disk, and imports as slowly as today's parse; Resident-Columnar stays O(file) RAM, killing R3. |
| MDF4 depth | Columnar full decode off-thread; true windowed decode | **Columnar off-thread** — maintainer decision in spec (2026-07-19); windowed decode fights mdflib's whole-data-group reads. |
| Replay API shape | Typed-Cells entry points; Parallel-Arrays (`QStringList` + `QVector<double>` + mask); Dataset-Direct (players write frames) | **Typed-Cells** (`replayChannelSpans` UTF-8 views for CSV, `replayChannelsTyped` for MDF4) — zero intermediate lists, live path untouched; Parallel-Arrays keeps the QStringList churn; Dataset-Direct bypasses the slot-pool/publish contract. |
| MDF4 numeric display string | `%.10g` stack-buffer + in-place assign; keep `QString::number(v,'g',10)` per cell | **`%.10g` in place** — allocation-free; both are 10-significant-digit trimmed formats. Parity risk is named in Risks; fallback is one line (revert to `QString::number` inside the lane) if A/B shows any corner-case divergence (inf/nan casing, exponent form). |
| CSV row validity in the indexer | Exact (byte-split each row, any non-empty cell); heuristic byte test | **Exact** — the byte splitter runs anyway and R8 parity is absolute; a heuristic would misclassify pathological quoted rows. |
| MDF4 progressive playback | Single payload handoff; per-data-group incremental handoffs | **Single handoff** — SS-exported files are effectively one data group, so incrementality buys nothing for the common case and adds cross-thread cache staging; R2 satisfied degenerately (explicitly flagged for review). |
| Shared player base class | Extract now; keep three copies | **Keep copies** — extraction touches the Sessions player (spec non-goal) and triples the parity surface of this already-large change; named as future work. |

## Risks & mitigations

- **Byte splitter vs `splitReplayRow` divergence** (R8/AC3-AC4 killer). Mitigation: the byte
  splitter is written as the semantic mirror with a shared quote-state table, reviewed
  side-by-side; separators/quotes are ASCII so decode-after-split == split-after-decode; the
  UTF-16 splitter stays for its existing callers so QuickPlot's downstream re-split uses the
  same rules as today.
- **Worker vs mmap lifetime** — unmap/`QFile` teardown only after cancel+join (also on
  reopen-while-indexing, per spec constraint). The Sessions worker's shutdown shape
  (cancel -> quit -> wait-with-timeout) is copied verbatim.
- **`mdf::*` thread confinement** — reader and channel pointers never leave the worker;
  playback reads only the columnar payload. `isOpen()` re-based on player state, not
  `m_reader->IsOk()`.
- **Frontier edge cases** — every `Q_ASSERT(row < frameCount())` site gains frontier clamps;
  play at frontier pauses-and-resumes as batches land (CSV) instead of running past the index.
- **`%.10g` display parity** — corner set (inf/nan/±0/exponent) A/B'd during implementation;
  one-line fallback documented above.
- **macOS file-dialog reentrancy** (common-mistakes) — the existing queued-invoke open shape
  is preserved; prompts run before worker start, from the queued context.
- **Silent breakage classes on notice:** `qt-todouble-direct` (all parses via
  `SerialStudio::toDouble`), `operator[]` inserts on source maps, stale `Q_ASSERT`s that
  assumed complete data, and no caching of views into reused buffers (mapped file memory is
  stable for the map's lifetime, unlike frame-pool slots — the plan relies on that stability
  and documents it where views are held).

## Test & verification plan

- **Unit (runnable here):** none applicable (`tests/scripts/` is JS-parser only). Static:
  `python scripts/code-verify.py --check` on every touched file; `qt-cpp-review` before
  handoff.
- **Integration (maintainer, app + API server up):** existing replay/dashboard integration
  tests must pass unmodified (AC3); if a player-replay pytest exists it runs on both a small
  reference CSV and MDF4 file, else AC3 falls to the A/B observation.
- **Maintainer observations:** AC1 (big-file open, responsive UI, memory bound), AC2
  (>10M-row CSV plays past the old cap), AC4 (verbatim text spot-check), AC6 (close/quit
  mid-index, repeated), AC7 (scrub clamps at frontier then advances).
- **Hotpath:** `--benchmark-hotpath` at current gates (AC5) — expected untouched; any
  FrameBuilder diff makes this mandatory anyway.
- **Numeric parity:** during implementation, a temporary debug assertion compares the lane's
  `%.10g` output against `QString::number(v,'g',10)` over the corner set + fuzzed doubles;
  removed before handoff.
