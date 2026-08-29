---
spec: 0019-fft-freq-markers
phase: tasks
status: approved     # maintainer delegated gate approval for this run (2026-07-17)
updated: 2026-07-17
---

# Tasks 0019 — FFT frequency markers and alarm bands

> **Phase 3 of 4 — the ordered checklist.** Decomposes [`plan.md`](./plan.md).
> `/ss-implement` works top to bottom and keeps the boxes current.

## Conventions

- One task = one focused, reviewable change.
- **Verify** = how this unit is confirmed before moving on.
- **Deps** = task IDs that must land first.

## Tasks

### T1 — Schema: FrequencyMarker struct + keys + serialize/read

- **Files:** `app/src/DataModel/Frame.h`, `app/src/DataModel/Frame.cpp`
- **Does:** Adds `Keys::FFTMarkers/Frequency/EndFrequency/WarningDb/AlarmDb`;
  `struct alignas(8) FrequencyMarker` (freq, endFreq, warningDb/alarmDb = NaN,
  color, label) + static_assert; `Dataset::fftMarkers` vector; inline
  `serialize(const FrequencyMarker&)` (optional fields omitted; never emits NaN);
  hook in `serialize(const Dataset&)` (array only when non-empty — byte-identical
  untouched projects); `read(FrequencyMarker&, obj)` in Frame.cpp (toDouble, finite
  freq > 0 gate, endFreq kept only when > freq, warn/alarm normalized) + call in
  `read(Dataset&)` after `readDatasetAlarmBands`.
- **Verify:** `python scripts/code-verify.py --check app/src/DataModel/Frame.h
  app/src/DataModel/Frame.cpp`; read-back of serialize/read symmetry.
- **Deps:** none
- [x] done

### T2 — Editor C++: launcher signal + commit slot

- **Files:** `app/src/DataModel/ProjectEditor.h`,
  `app/src/DataModel/Project/ProjectEditorForms.cpp`,
  `app/src/DataModel/Project/ProjectEditorCommit.cpp`
- **Does:** Declares `openFrequencyMarkersEditor(groupId, datasetId, nyquist,
  markers)` signal + `openFrequencyMarkersEditorForSelection()` /
  `commitFrequencyMarkers(QVariantList)` slots; launcher builds the QVariantList from
  `m_selectedDataset.fftMarkers` with Nyquist = `fftSamplingRate * 0.5` (fallback
  guard for rate <= 0); commit rebuilds the vector (same validation as read),
  `pm.updateDataset(..., false)` + `buildDatasetModel` — exact `commitAlarmBands`
  shape, no ProjectModel-ctor-reachable code.
- **Verify:** code-verify on the three files; signature cross-check against the QML
  wiring plan (T6).
- **Deps:** T1
- [x] done

### T3 — API: dataset-update key + atomic get/set commands

- **Files:** `app/src/API/Handlers/ProjectHandler.h`,
  `app/src/API/Handlers/ProjectHandler.cpp`,
  `app/src/API/Handlers/ProjectHandlerEntities.cpp`
- **Does:** `fftMarkers` branch in `applyDatasetNumericFields` (array through
  `read(FrequencyMarker&)`, invalid entries dropped, key consumed so the
  unknown-field warning stays honest); `datasetGetFFTMarkers` /
  `datasetSetFFTMarkers` impls (clones of the alarm-bands pair, `droppedInvalid`
  count) + registration as `project.dataset.getFFTMarkers` / `setFFTMarkers` beside
  the alarm-bands registrations. Free feature — no BUILD_COMMERCIAL guard.
- **Verify:** code-verify on the three files; registration/table read-back.
- **Deps:** T1
- [x] done

### T4 — FFT widget model: marker config, monitoring, QML surface

- **Files:** `app/src/UI/Widgets/FFTPlot.h`, `app/src/UI/Widgets/FFTPlot.cpp`
- **Does:** Ctor copies dataset markers, clamps to 0…Nyquist, resolves per-marker
  `[binLo, binHi]` (point = ±2 bins) — re-resolved in `rebuildFftPlan` (bin width
  changes with size); `updateData()` post-`computeBinSpectrum` evaluates peak display
  dB + state (NaN-aware warn/alarm compare) into pre-sized vectors and emits
  `markerValuesChanged()`; exposes `markers` QVariantList (built once, CONSTANT-style
  config) + `markerPeakDb(int)` / `markerState(int)` invokables. **Binding
  invariants:** zero steady-state allocation in the per-tick path; display-dB
  (post-ballistics) values only; no hotpath contact; assertion density on the new
  functions.
- **Verify:** code-verify on both files; read-back that per-tick loop allocates
  nothing and indices are bounds-guarded.
- **Deps:** T1
- [x] done

### T5 — FFT QML: overlay rendering + toolbar toggle

- **Files:** `app/qml/Widgets/Dashboard/FFTPlot.qml`
- **Does:** Marker overlay: bands as gradient rectangles + edge strokes and points as
  glow lines in a clipped `Item` parented to `plot.curveLayer` with `z: -1` (under
  the curve stroke); label chips (pill, marker-color bg, `label · dB` readout,
  warning/alarm re-tint via `Cpp_ThemeManager.alarmColorForSeverity`, alarm opacity
  blink, greedy row drop on horizontal overlap) in an upper overlay; Hz→x mapping via
  `plot.xVisibleMin/xVisibleRange` with `logX ? log10 : identity` — the PlotCurve
  transform, so zoom/pan track for free. Toolbar `DashboardToolButton` toggle
  (`labels.svg`), persisted as `showFrequencyMarkers` (default on). Chip text uses
  numbered `.arg()` placeholders only.
- **Verify:** code-verify (QML rules) on the file; read-back of the transform vs
  PlotCurve/cursor math; maintainer AC3/AC4 checklist below.
- **Deps:** T4
- [x] done

### T6 — Editor QML: FrequencyMarkersEditor dialog + DatasetView wiring

- **Files:** `app/qml/ProjectEditor/Dialogs/FrequencyMarkersEditor.qml` (new),
  `app/qml/ProjectEditor/Views/DatasetView.qml`, `app/CMakeLists.txt`
- **Does:** Dialog cloned from AlarmBandsEditor grammar (SmartDialog; presets card —
  50 Hz / 60 Hz mains + harmonics; table: Start Hz, End Hz (blank = point), Label,
  Color swatch + shared ColorDialog + right-click reset, Warn dB, Alarm dB, move
  up/down, delete; 0…Nyquist preview strip; Cancel/Apply footer → 
  `commitFrequencyMarkers`); inline clamping to 0…Nyquist and warn/alarm
  normalization at collect. DatasetView: lazy Loader + `Connections`
  (`onOpenFrequencyMarkersEditor`) + "Freq. Markers" ribbon button in the Behavior
  section, enabled when `DatasetFFT || DatasetWaterfall`. CMake registers the new
  QML file next to AlarmBandsEditor's entry.
- **Verify:** code-verify on both QML files; CMake entry read-back; maintainer AC5.
- **Deps:** T2
- [x] done

### T7 — Waterfall: marker rendering + per-row state + toggle

- **Files:** `app/src/UI/Widgets/Waterfall.h`, `app/src/UI/Widgets/Waterfall.cpp`,
  `app/qml/Widgets/Dashboard/Waterfall.qml`
- **Does:** Ctor copies markers; extracts one local Hz→x visible-window helper used
  by BOTH `drawXAxis` and the new marker pass (single source of truth for the
  mapping); `updateData()` computes per-marker peak/state from the fresh `m_smoothed`
  row; `paint()` draws band fills / lines / label chips after the axis composite
  (per-paint, NOT the cached axis layer — escalation tint changes per row);
  `markersVisible` Q_PROPERTY (default true). QML toolbar toggle persisted as
  `showFrequencyMarkers`. Commercial-licensed files — keep the SPDX headers intact.
- **Verify:** code-verify on the three files; read-back that `drawXAxis` output is
  unchanged when markers are absent; maintainer AC6.
- **Deps:** T1
- [x] done

### T8 — pytest: persistence + API round-trip

- **Files:** `tests/integration/test_fft_markers.py` (new)
- **Does:** Round-trip via `project.dataset.update` and `setFFTMarkers`/
  `getFFTMarkers`; save/load persistence; no-marker project serializes without the
  key; invalid payloads (negative freq, junk types, reversed band, warn > alarm)
  dropped/normalized with `droppedInvalid`; follows `tests/utils/api_client.py`
  patterns + existing markers/fixtures per `tests/README.md`.
- **Verify:** `python -m py_compile`; maintainer runs
  `pytest tests/integration/test_fft_markers.py -v` with the app + API server up.
- **Deps:** T3
- [x] done

### T9 — Docs: architecture note + spec close-out

- **Files:** `doc/claude/architecture/dashboard.md`,
  `doc/claude/specs/0019-fft-freq-markers/spec.md`
- **Does:** Short "FFT Frequency Markers (spec 0019)" subsection in dashboard.md
  (schema key, widget-local monitoring, overlay layering, waterfall mapping helper);
  spec ACs checked off where verifiable, status flipped per Definition of Done.
- **Verify:** documentation read-back; no doc-verify targets touched (doc/claude is
  exempt from documentation-verify.py).
- **Deps:** T1–T8
- [x] done

## Definition of Done

- [x] Every acceptance criterion in `spec.md` is met or explicitly left to the
      maintainer (AC3–AC7 runtime checks; AC1/AC2 tests written, maintainer runs).
- [x] `python scripts/code-verify.py --check` clean on all changed files (no new
      errors).
- [x] `qt-cpp-review` run on the C++ diff (6 agents); confirmed findings fixed
      (double-domain bin clamp, spectrumSize==0 guards, freq ceiling, chip-row
      hardening, hasThresholds wired); accepted-as-is items noted in chat handoff.
- [x] Hotpath untouched (no ingest-path diffs); `--benchmark-hotpath` left to CI.
- [x] `pytest tests/integration/test_fft_markers.py` identified for the maintainer.
- [x] `python scripts/sanitize-commit.py` run (sanitize only — **no commit**).
- [x] Diff is what was asked, and only that — no foreign files touched (the
      pre-existing 0016–0018 spec-doc edits in the working tree are the
      maintainer's, untouched).
- [x] `spec.md` status set to `done`.
