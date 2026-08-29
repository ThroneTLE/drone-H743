---
spec: 0014-log-axes
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-17
---

# Plan 0014 — Logarithmic axis scales for Plot, FFT, and MultiPlot

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

QtGraphs 2D (Qt 6.11) has no logarithmic axis type, so log scaling is implemented as
**log-world carriers**: each widget model (`Widgets::Plot`, `FFTPlot`, `MultiPlot`)
transforms its render data and axis bounds into log10 space at draw time, and
`PlotWidget.qml` keeps its linear `ValueAxis` — now spanning log10 units — while formatting
decade tick labels (`10^n` via the existing engineering formatter) and converting cursor /
crosshair / range-dialog values back through `pow10`. Everything downstream of the model
(PlotCurve GPU rendering, PlotAreaFill, zoom/pan, visible-window push) already operates on
"world coordinates" and needs no changes, because log-world is just a different world.
Persistence is three new optional dataset bools (`plotLogX`, `plotLogY`, `fftLogX` — the
JSON key `log` is already taken by CSV logging); MultiPlot follows the existing group→
dataset fan-out convention that `xAxisId` uses. The hotpath is untouched: transforms run in
widget `updateData()` at UI cadence (≤240 Hz, O(rendered points)).

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/DataModel/Frame.h` | `Dataset` fields `pltLogX`, `pltLogY`, `fftLogX` (slot into the existing bool run ~421-430, keeps `sizeof` static_assert happy); `Keys::PltLogX("plotLogX")`, `Keys::PltLogY("plotLogY")`, `Keys::FFTLogX("fftLogX")` (~line 111); guarded `obj.insert` in `serialize(Dataset)` (~1109, `if (d.pltLogX)` pattern so old files stay byte-identical) |
| `app/src/DataModel/Frame.cpp` | `ss_jsr` reads with `false` defaults in `read(Dataset&, ...)` (~line 311) |
| `app/src/DataModel/Project/ProjectEditorItemIds.h` | `kDatasetView_Plt_LogX`, `kDatasetView_Plt_LogY`, `kDatasetView_FFT_LogX` in `DatasetItem` (~65); `kGroupView_LogX`, `kGroupView_LogY` in `GroupItem` (~101) |
| `app/src/DataModel/Project/ProjectEditorForms.cpp` | Two CheckBox rows in `addPlotSection` (~1042): Y-log editable on `dataset.plt`, X-log editable on `dataset.plt && xAxisId != kXAxisTime`; one CheckBox in `buildFftRangeRows` (~1129) gated on `dataset.fft` (not waterfall); two CheckBox rows next to `buildGroupXAxisRow` (~254) for MultiPlot groups, X-log editable only in Samples mode |
| `app/src/DataModel/Project/ProjectEditorCommit.cpp` | Bool cases in `onDatasetFlagItemChanged` (~807); group fan-out in `applyGroupEdit` mirroring `kGroupView_xAxis` (~274-282): write flag into every member dataset |
| `app/src/UI/Widgets/Plot.h/.cpp` | `logX`/`logY` CONSTANT Q_PROPERTYs (resolved in ctor: `logX = dataset.pltLogX && !m_timeAxis`, structurally impossible on time axis per spec); log transforms in `updateData()` + log-clamped bounds in `updateRange()` / `calculateAutoScaleRange()`; scratch X buffer for pre-downsample X transform |
| `app/src/UI/Widgets/FFTPlot.h/.cpp` | `logX` CONSTANT Q_PROPERTY; ctor sets `m_minX = log10(freqStep)`, `m_maxX = log10(fs/2)` when log; `computeSmoothedSpectrum()` pushes `log10(freq)` (DC bin clamped to `m_minX`) |
| `app/src/UI/Widgets/MultiPlot.h/.cpp` | Same as Plot: `logX` (Samples mode only) / `logY` from `group.datasets.front()` (fan-out read-back convention); per-curve Y transform, one shared scratch X |
| `app/qml/Widgets/PlotWidget.qml` | `logX`/`logY` props; decade-aware `smartIntervalX/Y` (integer decades ≥1); `tickAnchor = ceil(min)` in log mode; label delegates format `pow(10, v)`; cursor/crosshair/delta labels in true units |
| `app/qml/Widgets/Dashboard/Plot.qml` | Forward `model.logX/logY`; trigger-level line mapped `log10 ↔ pow10` when `logY`; area-fill baseline = `minY` when `logY` |
| `app/qml/Widgets/Dashboard/FFTPlot.qml` | Forward `model.logX` |
| `app/qml/Widgets/Dashboard/MultiPlot.qml` | Forward `model.logX/logY`; same trigger/baseline handling as Plot.qml |
| `app/qml/Dialogs/AxisRangeDialog.qml` | Display/edit true values: `pow10` on read, `log10(clamp)` on write when the axis is log |
| `app/src/API/Handlers/ProjectHandlerEntities.cpp` | `takeParam` blocks for the three bools in `applyDatasetTextAndToggleFields` (~1436-1491) |
| `app/src/API/Handlers/ProjectHandler.cpp` | Field names added to the `project.dataset.update` doc string (~1279-1300) |
| `tests/integration/test_project_editor.py` | Round-trip test mirroring `test_dataset_color_round_trip` (~273-292) |
| `app/rcc/api/api-schema.json` (+ generated SDK) | Refreshed via `--dump-api-schema` by the developer, then `sanitize-commit.py` regenerates `SerialStudio.js`/`.lua` + AI search index |

## Architecture & data flow

Unchanged upstream: driver → FrameReader → FrameBuilder → Dashboard ingest → rings. The
change lives entirely in the widget-model draw layer and QML:

1. **Config**: editor/API writes the bools into `Dataset`; project save/load round-trips
   them via `Keys::`. MultiPlot group toggles fan into every member dataset (the exact
   `kGroupView_xAxis` convention, read back from `datasets.front()` — same encoding wart,
   same slated group-level-field cleanup).
2. **Model resolve (construction)**: each widget reads its dataset/group in the ctor
   (Plot.cpp:59, FFTPlot.cpp:57, MultiPlot.cpp:81) and freezes `logX`/`logY` as CONSTANT
   properties — widgets are recreated on every project reconfigure, same lifetime as
   `timeAxis`. Plot forces `logX = false` when `m_timeAxis`; MultiPlot forces it unless
   Samples mode — a project JSON that forces log-X onto a time plot renders linear (AC5).
3. **Draw transform (`updateData()`, UI cadence)**:
   - **Y (exact)**: log10 is monotonic, so per-column min/max envelopes commute with the
     transform — apply `y = log10(max(y, floor))` to the downsampler *output* (`m_data`),
     after `downsampleMonotonic` / `downsampleTimeWindow` / `downsampleWindowAbsolute`.
     Envelope-exact, O(columns).
   - **X (pre-downsample)**: bucketing must be uniform in *log* pixel space or low decades
     collapse into a few columns. FFTPlot already owns its X buffer — push `log10(freq)`
     in `computeSmoothedSpectrum` (bin 0 clamped to `m_minX`, keeping X non-decreasing).
     Plot/MultiPlot's samples/dataset-X rings are Dashboard-shared and immutable, so the
     widget copies X through a member scratch `DSP::AxisData` applying log10, then calls
     the `(x, y)` overload of `downsampleMonotonic` (the one FFTPlot uses). The
     non-monotonic XY path applies log10 inline in its existing copy loop.
   - **Ranges**: `updateRange` / `calculateAutoScaleRange` compute linear bounds as today,
     then map through log10 with the floor policy below. QML's visible-window push
     (`setVisibleXWindow`) round-trips log-space values consistently — no change.
4. **QML render**: `ValueAxis` min/max/tickInterval now carry log10 units. Tick labels
   format `pow(10, value)` through the existing `engineeringFormat` (decades render as
   1, 10, 100, 1K, 10K…); `tickAnchor = Math.ceil(min)` lands ticks on decade boundaries;
   `subTickCount` stays — uniform subticks in log space sit at geometric positions
   (10^(n+k/(m+1))), which are legitimate log minors. Cursors, crosshairs, deltas, the
   trigger-level line, and AxisRangeDialog convert at the display boundary
   (`pow10` out, `log10` in); PlotCurve/PlotAreaFill/zoom/pan are world-space and
   untouched.

**Floor policy (R5)**: data values clamp at `kLogFloor = 1e-12`. Range bounds: a
configured `pltMin/pltMax ≤ 0` is unusable in log mode, so the bound falls back to
data-derived auto-scale over *positive* samples (smallest positive datum), with
`max(maxBound × 1e-6, kLogFloor)` as the empty-data fallback — this avoids a 12-decade
mostly-empty axis when a user leaves `min = 0` configured. One constexpr, one helper
(`logClamp`), shared by the three models.

## Hotpath & threading impact

- **Touches the hotpath?** **No.** All transforms run in widget `updateData()` /
  `updateRange()` on the main thread at `TimerEvents::uiTimeout` cadence (60 Hz default,
  240 max), operating on the widget-local render buffers (`m_data`, `m_xData`, scratch).
  Ingest (`hotpathRxFrame`, push tables, TimeRing/SweepEngine `advance`) is untouched;
  `SweepEngine` keeps triggering on raw values. The scratch `AxisData` reaches steady-state
  size after the first frame — no per-tick allocation. `--benchmark-hotpath` runs as the
  regression gate anyway (AC6) since Dashboard draw sits in the benchmarked consumer rows.
- **New cross-thread signal/slot?** No. No new connections at all — the flags ride the
  existing dataset/group read at widget construction.
- **New input to a cached hotpath flag?** No. The bools are read at widget construction,
  not per frame; project changes already rebuild widgets via reconfigure.
- **Timestamp ownership** — unaffected; no timestamps touched.

## Data model & persistence

- `Dataset` gains `bool pltLogX = false`, `bool pltLogY = false`, `bool fftLogX = false`,
  placed in the existing bool run (~Frame.h:421-430) so `sizeof(Dataset)` is unchanged
  (padding absorbs them; the alignment static_assert at :455 stays green).
- `Keys::PltLogX("plotLogX")`, `Keys::PltLogY("plotLogY")`, `Keys::FFTLogX("fftLogX")` —
  names chosen because `Keys::Log("log")` already means CSV logging, and the `plot*`/`fft*`
  prefixes match the existing `plotMin`/`fftSamples` families.
- Serialize: guarded inserts (`if (d.pltLogX) obj.insert(...)`) — projects that never use
  the feature serialize byte-identically to today (AC7: no modified-flag, no diff noise).
- Deserialize: `ss_jsr(obj, Keys::PltLogX, false).toBool()` — absent keys read linear (AC4).
- **No Group key, no schema bump**: MultiPlot persists via the per-dataset fan-out
  (identical to `xAxisId`), so schemaVersion stays 1 and older Serial Studio versions
  simply ignore the unknown dataset keys.

## API / SDK surface

- `project.dataset.update` gains `plotLogX`, `plotLogY`, `fftLogX` (bools) via `takeParam`
  blocks in `applyDatasetTextAndToggleFields`; unknown-field warnings stop firing once
  wired. Doc string updated in `ProjectHandler.cpp`. Not commercial-gated (spec R8).
- Mutations flow through the existing epoch-gated `CommandRegistry::execute` apply path
  (syncFromProjectModel + debounced autosave) — dataset-field updates are already wired;
  nothing new to wire.
- Developer refreshes `app/rcc/api/api-schema.json` with `SerialStudio --dump-api-schema`
  (needs a build — maintainer runs it); `sanitize-commit.py` then regenerates the JS/Lua
  SDK and the AI search index.

## QML / UI

- **Editor**: five CheckBox rows using the existing form-item pattern (WidgetType
  CheckBox + `Active`/`setEditable` gating, translated `ParameterName`/`Description`).
  Bool rows need no multi-select combo guard — `onDatasetFlagItemChanged` is reused by
  multi-select automatically. Row editability re-resolves on the form rebuild that any
  commit already triggers (same mechanism the waterfall-conditional rows use).
- **PlotWidget.qml**: `logX`/`logY` props plus the decade tick/label/readout changes above.
  No new components; no ComboBox restore races (checkboxes only).
- **Wrappers**: Plot/FFTPlot/MultiPlot.qml forward the props; trigger-level and area-fill
  baseline adjustments as listed in the file table. `dataBipolar`-driven baseline logic is
  bypassed when `logY` (clamped log data is single-signed by construction; baseline pins
  to `minY`, matching FFT's floor convention).

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Where the log mapping lives | (a) log-world carriers in the models; (b) QML/render-side remap (PlotCurve, fill, axes each transform); (c) custom log-axis item | **(a)** — QtGraphs has no log axis, and (a) needs one transform site per model while every world-space consumer (curve GPU path, fill, cursors, zoom/pan, visible-window push) works unchanged; (b) breaks C++ downsample bucketing and multiplies transform sites; (c) re-implements GraphsView grid/theming for no visual gain |
| MultiPlot persistence | Group-level JSON field vs. fan-out into member datasets | **Fan-out** — matches the existing `xAxisId` group convention (read back from `datasets.front()`), avoids a Group schema addition; inherits the same known wart, and joins the slated group-level-`xAxisId` cleanup when that happens |
| Editor control shape | Linear/Log combo vs. checkbox per axis | **Checkbox** ("Logarithmic X/Y Axis") — matches the editor's other bool toggles, commits through the existing flag handler, zero multi-select guard code |
| X transform vs. downsampler | Post-downsample X remap vs. pre-downsample transform (scratch copy / FFT-side push) | **Pre-downsample** — post-remap buckets uniformly in linear space, collapsing the low decades (the headline FFT use case) into a handful of pixel columns; the scratch copy is O(n) at UI cadence, allocation-free at steady state |
| Y transform placement | Pre-downsample vs. post-downsample | **Post** — log10 is monotonic so min/max envelopes commute exactly; transforming O(columns) beats O(samples) with identical output |
| Non-positive range bounds | Clamp bound to `kLogFloor` verbatim vs. fall back to positive-data auto-scale | **Auto-scale fallback** — a literal `log10(1e-12)` floor on a user-left `min = 0` yields a 12-decade empty axis; deriving from positive data gives a usable viewport, floor only as last resort |
| FFT DC bin | Skip bin 0 in log mode vs. clamp to first-bin frequency | **Clamp** — keeps buffer sizes/loops identical and X non-decreasing (downsampler precondition); the DC point collapses onto the left edge instead of shifting every buffer size by one |

## Risks & mitigations

- **Fan-out coupling**: a dataset that is both in a MultiPlot group and has its own Plot
  widget shares `pltLogY` across both — identical to how `xAxisId` already behaves;
  documented here, resolved whenever the group-level-field refactor lands.
- **Downsampler preconditions**: `downsampleMonotonic` requires monotonic X — log10 of a
  monotonic positive sequence is monotonic, and the DC/floor clamps produce non-decreasing
  (never decreasing) runs. `clampToVisibleX`'s `Q_ASSERT(lo <= hi)` holds since log
  preserves ordering.
- **Stale form gating**: the X-log checkbox's editability depends on the current
  `xAxisId`; it re-resolves on the form rebuild every commit triggers — same lifecycle as
  the existing waterfall-conditional rows, no new mechanism.
- **AxisRangeDialog writes**: it assigns `plotWidget.xMin/…` directly (overriding model
  bindings); log conversion must happen at that boundary or a typed "100" becomes decade
  100. The dialog change is small but load-bearing — it gets its own task and AC2 check.
- **`%n`/`.arg()` translation trap** (common-mistakes): new editor strings use plain
  `tr()` with no plurals — low risk, but the checklist item stays.
- **Silent editor no-op**: forgetting the `ProjectHandlerEntities` wiring makes the API
  field warn `unknown_field` instead of applying — the round-trip pytest catches exactly
  this.

## Test & verification plan

- **Unit (runnable here)**: none — no JS-parser surface. Verification is read-based +
  `code-verify.py` (below).
- **Integration (maintainer runs, app up with API server on 7777)**:
  - New `test_log_axis_round_trip` in `tests/integration/test_project_editor.py`
    (mirrors `test_dataset_color_round_trip`): sets the three flags via
    `project.dataset.update`, exports, reloads, re-exports, asserts persistence **(AC4)**;
    plus an assert that a JSON without the keys reads back `false` **(AC4, R6)**.
- **Maintainer observations (running app)**:
  - FFT @ 44.1 kHz, log frequency on: decade ticks 10/100/1K/10K, octaves evenly spaced;
    toggle off restores today's render **(AC1)**.
  - Plot fed 0.01→1000 with log Y: equal-height decades, cursor readouts in true units
    **(AC2)** — including after editing ranges via AxisRangeDialog.
  - Zero/negative samples into a log-Y plot: clamped curve, no crash/blank **(AC3)**.
  - Time-axis Plot/MultiPlot: no X-log checkbox offered; hand-edited JSON forcing
    `plotLogX` onto a time plot renders linear **(AC5)**.
  - Existing example projects open unmodified, render identically **(AC7)**.
- **Hotpath**: `--benchmark-hotpath` at existing gates (CI + maintainer) **(AC6)** —
  expected no-op since ingest is untouched; gate proves it.
- **Static**: `python scripts/code-verify.py --check` on every touched file;
  `qt-cpp-review` before handoff; `python scripts/sanitize-commit.py` before commit
  (regenerates SDK + AI index; developer refreshes `api-schema.json` via
  `--dump-api-schema` first).
