---
spec: 0019-fft-freq-markers
phase: plan
status: approved     # maintainer delegated gate approval for this run (2026-07-17)
updated: 2026-07-17
---

# Plan 0019 — FFT frequency markers and alarm bands

> **Phase 2 of 4 — the HOW.** Satisfies [`spec.md`](./spec.md). Grounded in:
> `doc/claude/architecture/dashboard.md` + `project.md` + `common-mistakes.md` (read in
> full), `FFTPlot.h/.cpp`, `PlotWidget.qml`, `FFTPlot.qml`, `Waterfall.h/.cpp/.qml`,
> `AlarmBandsEditor.qml`, `DatasetView.qml`, `Frame.h/.cpp` schema paths,
> `ProjectEditorForms/Commit.cpp`, `ProjectHandlerEntities.cpp`.

## Approach (one paragraph)

Clone the alarm-bands architecture into the frequency domain. A new
`DataModel::FrequencyMarker` struct (`freq`, optional `endFreq`, `label`, `color`,
optional `warningDb`/`alarmDb`) rides on `Dataset` as `std::vector<FrequencyMarker>
fftMarkers`, serialized under a new `Keys::FFTMarkers` array exactly like
`Keys::AlarmBands` (additive, omitted when empty). The Project Editor gets a
`FrequencyMarkersEditor.qml` dialog cloned from `AlarmBandsEditor.qml` (signal →
lazy-Loader → commit slot), launched from a new ribbon button in `DatasetView.qml`. The
`FFTPlot` C++ model loads markers at construction, precomputes each marker's bin window,
and after `computeBinSpectrum` each tick evaluates the peak display dB per marker
(ballistics-processed, so what you see is what is judged), exposing config via a
`markers` QVariantList property and live peak/state via allocation-free invokables plus
a `markerValuesChanged` signal; `FFTPlot.qml` renders DAW-EQ-style gradient bands and
labeled lines in a new overlay inside `PlotWidget.curveLayer` (under the curve) with
label chips above. `Waterfall` (Pro, `QQuickPaintedItem`) draws the same markers in its
`paint()` using the identical zoom/pan frequency mapping as `drawXAxis`. The API adds an
`fftMarkers` key to the dataset-update allow-list plus atomic
`project.dataset.getFFTMarkers` / `setFFTMarkers` commands mirroring the alarm-bands
pair. Zero hotpath contact.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/DataModel/Frame.h` | `Keys::FFTMarkers/Frequency/EndFrequency/WarningDb/AlarmDb`; `struct FrequencyMarker`; `Dataset::fftMarkers`; inline `serialize(const FrequencyMarker&)`; hook into `serialize(const Dataset&)`; `read(FrequencyMarker&, ...)` decl |
| `app/src/DataModel/Frame.cpp` | `read(FrequencyMarker&, obj)` (needs `SerialStudio::toDouble`); call in `read(Dataset&)` |
| `app/src/DataModel/ProjectEditor.h` | `openFrequencyMarkersEditor(...)` signal; `openFrequencyMarkersEditorForSelection()` / `commitFrequencyMarkers(...)` slots |
| `app/src/DataModel/Project/ProjectEditorForms.cpp` | `openFrequencyMarkersEditorForSelection()` (mirrors the alarm-bands launcher; range = 0…Nyquist from `fftSamplingRate`) |
| `app/src/DataModel/Project/ProjectEditorCommit.cpp` | `commitFrequencyMarkers(const QVariantList&)` (mirrors `commitAlarmBands`) |
| `app/src/API/Handlers/ProjectHandler.h` | `datasetGetFFTMarkers` / `datasetSetFFTMarkers` decls |
| `app/src/API/Handlers/ProjectHandler.cpp` | register both commands (next to the alarm-bands registrations, ~line 533) |
| `app/src/API/Handlers/ProjectHandlerEntities.cpp` | `fftMarkers` branch in `applyDatasetNumericFields`; the two command impls (clone of the alarm-bands pair at :1007/:1056) |
| `app/src/UI/Widgets/FFTPlot.h/.cpp` | marker storage, bin-window cache, per-tick peak/state evaluation, `markers` property + `markerPeakDb(i)` / `markerState(i)` invokables + `markerValuesChanged` |
| `app/src/UI/Widgets/Waterfall.h/.cpp` | marker storage, per-row peak/state, marker drawing pass in `paint()`, `markersVisible` property |
| `app/qml/ProjectEditor/Dialogs/FrequencyMarkersEditor.qml` | **new** — dialog cloned from AlarmBandsEditor (rows: Start Hz, End Hz, Label, Color, Warn dB, Alarm dB; presets; preview strip) |
| `app/qml/ProjectEditor/Views/DatasetView.qml` | lazy Loader + `Connections` for the new dialog; "Freq. Markers" ribbon button (Behavior section, enabled when FFT or Waterfall on) |
| `app/qml/Widgets/Dashboard/FFTPlot.qml` | marker overlay (bands under curve via `curveLayer`, chips above), toolbar toggle persisted as `showFrequencyMarkers` |
| `app/qml/Widgets/Dashboard/Waterfall.qml` | toolbar toggle wired to `model.markersVisible`, persisted |
| `app/CMakeLists.txt` | register `FrequencyMarkersEditor.qml` (near AlarmBandsEditor's entry) |
| `tests/integration/test_fft_markers.py` | **new** — persistence + API round-trip + validation (maintainer runs) |
| `doc/claude/architecture/dashboard.md` | short subsection on FFT frequency markers |

## Architecture & data flow

- **Persistence**: `Dataset.fftMarkers` ↔ `Keys::FFTMarkers` JSON array; read in
  `DataModel::read(Dataset&)` (after alarm bands), written in `serialize(const Dataset&)`
  only when non-empty (byte-identical untouched projects). `read(FrequencyMarker&)`
  validates: finite `freq > 0`; `endFreq` kept only when `> freq`; thresholds default
  NaN (= unset); returns false on junk → entry dropped (same contract as `AlarmBand`).
- **Editor**: ribbon button → `ProjectEditor::openFrequencyMarkersEditorForSelection()`
  → builds `QVariantList` from `m_selectedDataset.fftMarkers` + Nyquist
  (`fftSamplingRate * 0.5`) → `Q_EMIT openFrequencyMarkersEditor(...)` → DatasetView
  `Connections` shows the lazy-loaded dialog → Apply calls
  `Cpp_JSON_ProjectEditor.commitFrequencyMarkers(list)` → rebuilds
  `m_selectedDataset.fftMarkers`, `pm.updateDataset(...)`, `buildDatasetModel(...)` —
  the exact `commitAlarmBands` shape. Live dashboard pickup rides the existing
  modified→autosave→`syncRuntime()` path (same as alarm bands; widgets re-instantiate on
  dashboard reconfigure).
- **FFT widget (all main thread, UI tick cadence)**: ctor copies the dataset's markers,
  resolves each to `[binLo, binHi]` (point markers get a ±2-bin neighborhood, clamped to
  the spectrum) — recomputed in `rebuildFftPlan` since bin width changes with size. In
  `updateData()` after `computeBinSpectrum`: for each marker, peak over `m_binDb[binLo..
  binHi]` → store peak + state (0 normal / 1 warn / 2 alarm via NaN-aware threshold
  compare) in a pre-sized vector; emit `markerValuesChanged()`. QML reads
  `markerPeakDb(i)` / `markerState(i)` (double/int returns — no per-tick containers).
- **FFT QML overlay**: a clipped `Item` parented into `plot.curveLayer` with `z: -1`
  (bands render *under* the `PlotCurve` stroke) holding a `Repeater` over
  `model.markers`; each delegate maps `freq → x` via the widget's own world→pixel math
  (`logX ? log10(freq) : freq`, using `plot.xVisibleMin/xVisibleRange` — same transform
  as `PlotCurve`), tracking zoom/pan for free. Label chips live in a second overlay
  above the curve (pattern of `_triggerLine` in `PlotWidget.qml`), showing
  `label · live dB`, escalation recolor + alarm blink via a `SequentialAnimation`.
- **Waterfall (Pro)**: ctor copies markers; `updateData()` computes per-marker peak from
  the freshly-written `m_smoothed` row (few comparisons); `paint()` draws band fills /
  marker lines / label chips after the axis-layer composite, mapping Hz→x with the
  identical `xMinHz/xMaxHz` zoom-pan math already in `drawXAxis` (extract a tiny shared
  helper inside Waterfall.cpp). Not in the cached axis layer — escalation tint changes
  per row, and a handful of lines per paint is cheap next to the image blit.

## Hotpath & threading impact

- **Touches the hotpath?** **No.** No changes to `FrameReader` / `CircularBuffer` /
  `FrameBuilder` / `Dashboard` ingest or the span lane. All new computation runs inside
  widget models on the GUI thread at UI-tick cadence, downstream of the existing
  `fftData()` ring read. `--benchmark-hotpath` expected byte-identical (AC7).
- **New cross-thread signal/slot?** No. `markerValuesChanged` is same-thread
  (widget → QML); editor/API paths are main-thread as today.
- **New input to a cached hotpath flag?** No.
- **Timestamp ownership** — untouched; feature never sees frames.
- **Steady-state allocation**: marker/bin/state vectors sized at ctor /
  `rebuildFftPlan`; the per-tick loop only compares floats. The `markers` QVariantList
  is built once at construction (config, not per-tick data).

## Data model & persistence

- New `Keys::` entries (single source of truth): `FFTMarkers("fftMarkers")`,
  `Frequency("freq")`, `EndFrequency("endFreq")`, `WarningDb("warningDb")`,
  `AlarmDb("alarmDb")`; reuse `Keys::Label` / `Keys::Color`.
- Marker JSON: `{"freq": 800, "endFreq": 850, "label": "Gear mesh", "color": "#ff5722",
  "warningDb": -40, "alarmDb": -25}` — every field but `freq` optional; optional fields
  omitted on write (NaN thresholds, empty strings, `endFreq <= freq` never serialized).
- No schema-version bump: additive key, old builds ignore it on read (same policy as
  0014/0017). Untouched projects serialize byte-identically (array omitted when empty).
- No legacy aliases needed (new key). No Sessions DB impact.

## API / SDK surface

- `applyDatasetNumericFields`: `fftMarkers` branch — array parsed through
  `DataModel::read(FrequencyMarker&, ...)`, invalid entries dropped (mirrors alarmBands;
  `takeParam` keeps the unknown-field warning honest).
- New commands `project.dataset.getFFTMarkers` / `project.dataset.setFFTMarkers`
  (atomic array read/write, `droppedInvalid` count on set), registered beside the
  alarm-bands pair in `ProjectHandler.cpp`. Free feature — no `BUILD_COMMERCIAL` guard
  (schema and FFT widget are GPL; only the Waterfall *renderer* is Pro and already
  gated at the widget level).
- SDK regenerated by `scripts/sanitize-commit.py` (generate-sdk step) at the end.

## QML / UI

- **FrequencyMarkersEditor.qml** (new, `Widgets.SmartDialog`): same section grammar as
  AlarmBandsEditor — Presets card (a few honest ones: 50 Hz / 60 Hz mains hum with 2
  harmonics as point markers; "1x / 2x / 3x shaft orders" scaffold from a base
  frequency prompt is out — keep presets static), markers table (Start Hz, End Hz
  (blank = point marker), Label, Color swatch + ColorDialog + right-click reset,
  Warn dB, Alarm dB, move up/down, delete), live preview strip over a 0…Nyquist scale
  (bands as translucent regions, points as ticks), footer Cancel / Apply IconButtons.
  Inline validation: Hz clamped 0…Nyquist on edit, `warn <= alarm` normalized at
  collect, blank/invalid numbers → unset.
- **DatasetView.qml**: Loader + `Connections` (`onOpenFrequencyMarkersEditor`) cloned
  from the alarm-bands wiring; ribbon `ToolbarButton` "Freq. Markers" in the Behavior
  section, `enabled` when `DatasetFFT || DatasetWaterfall` option set, icon
  `alarm-bands.svg` sibling (reuse `fft.svg` action icon to avoid new assets).
- **FFTPlot.qml**: band delegate = `Rectangle` with horizontal `Gradient` (edge 0.04 →
  core 0.22 opacity of marker color) + 1 px edge strokes; point delegate = 2 px line
  with a soft 8 px glow rect behind; chip = pill `Label` (marker color background at
  0.9 opacity, `widget_base` text — the cursor-label idiom from PlotWidget.qml), text
  `label · −42.1 dB`; warning state re-tints chip/line with
  `Cpp_ThemeManager.alarmColorForSeverity(2)`, alarm with severity 3 + opacity blink.
  Chips stack top-aligned; when two chips would overlap horizontally the later one
  drops a row (simple greedy row assignment in JS over marker x positions, few items).
  Toolbar toggle (icon `labels.svg`) persisted via
  `saveWidgetSetting(widgetId, "showFrequencyMarkers", ...)`, default on.
- **Waterfall.qml**: toolbar toggle (same icon/setting name) → `model.markersVisible`.
- Theme-reactive for free (QML bindings on `Cpp_ThemeManager.colors`); Waterfall's
  painted colors refresh via its existing `onThemeChanged`.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Where markers live | dataset JSON vs widgetSettings vs new top-level map | Dataset JSON (`fftMarkers`), like `alarmBands` — engineering config, API-addressable, per-dataset by nature |
| Monitoring locus | widget-local vs AlarmMonitor-central | Widget-local — FFT exists only inside the visible widget; central = headless FFT engine (spec non-goal) |
| FFT rendering | QML overlay vs C++ painted layer vs QtGraphs series | QML overlay in `curveLayer` — theme/zoom/pan free, matches PlotCurve layering, no scene-graph work |
| Waterfall rendering | in cached axis layer vs per-paint pass | Per-paint — escalation tint changes per row; axis cache only rebuilds on zoom/theme, would go stale |
| Threshold semantics | display dB (post-ballistics) vs raw dB | Display dB — WYSIWYG with the plot; raw would alarm on peaks the user cannot see |
| Live state to QML | per-tick QVariantList property vs invokable polling on a signal | Invokables + `markerValuesChanged` — no per-tick container churn |
| Point-marker window | exact bin vs ±2 bins | ±2 bins — a slightly detuned line still registers; constant, documented in the @brief |
| Editor entry | form-model rows vs dedicated dialog | Dialog — list-of-structs doesn't fit the row grammar; AlarmBands set the precedent |

## Risks & mitigations

- **Ctor-closure / startup**: none — no ProjectModel-ctor-reachable code touched; the
  editor launcher runs on user click only.
- **Byte-identical serialization regression** (0014/0017 discipline): `FFTMarkers`
  written only when non-empty; optional sub-keys omitted; covered by the new pytest
  round-trip which saves an untouched project and diffs.
- **`%n` / `.arg()` translation trap** (common-mistakes): chip text built with
  numbered placeholders only.
- **QML overlay under zoom**: mapping uses `xVisibleMin/xVisibleRange` (identical to
  PlotCurve/cursors) — verified against the existing cursor math; clip to the layer so
  panned-out markers never bleed.
- **Waterfall Hz mapping drift**: extract one local helper used by both `drawXAxis` and
  the marker pass so the two can't disagree.
- **Editor commits on multi-select**: `commitFrequencyMarkers` operates on
  `m_selectedDataset` exactly like `commitAlarmBands` (single-select; the ribbon button
  follows the same enablement as Alarm Bands so multi-select behavior matches).
- **Dashboard staleness after Apply**: relies on the same updateDataset→autosave→
  `syncRuntime` path alarm bands use; AC5 verifies live pickup, and if the FFT widget
  needs a nudge the fix is the existing reconfigure signal, not a new path.
- **NaN handling**: thresholds use `std::isnan` guards before compare; JSON writer
  omits NaN (never emits `null`).

## Test & verification plan

- **AC1 (persistence round-trip)** + **AC2 (API)**: new
  `tests/integration/test_fft_markers.py` (maintainer runs; app up with API server):
  set markers via `project.dataset.update` and via `setFFTMarkers`; `getFFTMarkers`
  echoes; save/load preserves; invalid payloads (negative freq, junk types, reversed
  band, warn > alarm) → dropped/normalized with `droppedInvalid`; project without
  markers → saved JSON contains no `fftMarkers` key.
- **AC3/AC4 (rendering + escalation)**: maintainer observation with audio input —
  checklist embedded in tasks.md.
- **AC5 (editor)**: maintainer observation against the AlarmBands dialog side-by-side.
- **AC6 (waterfall)**: maintainer observation, zoom/pan + Campbell mode sanity.
- **AC7 (hotpath)**: `--benchmark-hotpath` unchanged — no ingest-path diffs to gate,
  CI run confirms.
- **Static (I run)**: `python scripts/code-verify.py --fix/--check` on every touched
  file; `qt-cpp-review` pass over the C++ diff; `python scripts/sanitize-commit.py` at
  the end (sanitize only — **no commit**, per instruction).
