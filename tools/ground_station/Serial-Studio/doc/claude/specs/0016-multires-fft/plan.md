---
spec: 0016-multires-fft
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-17
---

# Plan 0016 — Multi-resolution FFT display for the logarithmic frequency axis

> **Superseded by [spec 0018](../0018-smooth-log-fft/spec.md)** — the design below shipped,
> then was replaced by the single-window approach the same day (uniform latency beat
> extra low-band resolution). Kept as a reference for the decimate-and-stitch technique:
> the stage math, crossover margins, and calibration analysis remain valid if the
> approach is ever revisited as an opt-in "high-resolution low end" mode.

## Approach (one paragraph)

**Decimation cascade + stitched same-size FFTs**, fully inside `Widgets::FFTPlot` at UI
draw cadence. When `dataset.fftLogX` is set, `Dashboard::configureFftSeries` sizes that
widget's sample ring at `E = 16` times the configured window (capped; the per-sample
ingest push is O(1) and unchanged). Each `updateData` tick the widget takes the ring
tail and produces three analysis stages: the newest `N` raw samples (stage 0, effective
rate `fs`), the newest `4N` samples decimated ×4 (stage 1, `fs/4`), and the newest `16N`
samples decimated ×16 via a second ×4 pass (stage 2, `fs/16`). All three run the *same*
`N`-point FFT (one kissfft plan, one window buffer, same `1/N²` power normalization), so
stage 2 delivers `Δf = fs/(16N)` at the low end — a 16× true-resolution gain (R1/R2, e.g.
2.7 Hz at 44.1 kHz / N=1024) — while stage 0 keeps today's short-window responsiveness up
top. The spectra stitch by frequency region at fixed crossovers (each stage used only up
to 0.4× its own Nyquist, keeping the decimator transition band out of the displayed
range), matched normalization keeps the dB trace continuous across seams (R3/R4), and the
stitched bins feed the existing log10-X push, smoothing, downsampling, and render path
from spec 0014 unchanged. Log axis off ⇒ every branch short-circuits to today's code
(R5). No schema, editor, API, or QML changes at all.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/UI/Dashboard.cpp` | `configureFftSeries()` (:2459): size the ring `dataset.fftLogX ? extendedFftCapacity(dataset.fftSamples) : dataset.fftSamples` (:2470). `extendedFftCapacity` = `min(16 * N, kMaxExtendedFftSamples)` as a file-local helper. **Only touch in this file** — the push table build (:2474-2484) and the per-frame `updateFftSeries` walk (:2272-2283) are byte-identical. |
| `app/src/UI/Widgets/FFTPlot.h` | Members for the multi-res path: stage scratch buffers (`std::vector<float>` × 2 for decimated histories, one shared FIR work buffer), stitched-spectrum size, expected ring capacity, per-stage crossover bin indices; declarations for the new private helpers below. No new Q_PROPERTY — QML contract unchanged. |
| `app/src/UI/Widgets/FFTPlot.cpp` | The bulk: (a) ctor + `rebuildFftPlan` compute stage layout (stage count from available ring span, crossover bins, expected capacity) and size all scratch at configure time; (b) `updateData` log branch: normalize+copy ring tail once, run the ×4/×4 decimation cascade (constexpr symmetric FIR taps, unity DC gain), run the three N-point FFTs through the one plan, `computeSmoothedSpectrum` generalized to emit the stitched (freq, dB) sequence with per-stage `freqStep`; (c) `applyLogFrequencyBounds` lower bound becomes the finest stage's first bin `fs/(16N)`; (d) the ring-capacity plan-rebuild check compares against the *expected* (possibly extended) capacity instead of assuming capacity == FFT size. Linear path: untouched code, guarded by `m_logX`. |
| `doc/claude/architecture/dashboard.md` | One paragraph documenting the extended FFT ring + stage stitching (CLAUDE.md: record architectural change). |

Confirmed by grep: `fftData()` has exactly one consumer (`FFTPlot.cpp:392`); Waterfall
uses its own `m_waterfallValues` ring (Dashboard.cpp:2524) and `floorPow2Bounded(dataset.
fftSamples)` (Waterfall.cpp:225), so the capacity extension cannot leak into it.

## Architecture & data flow

1. **Configure time** (project/reconfigure, main thread): `configureFftSeries` sizes the
   ring at `E·N` for log-axis FFT datasets. Widget is recreated (existing reconfigure
   flow); its ctor reads `fftSamples`/`fftSamplingRate`/`fftWindow`/`fftLogX` as today
   and additionally computes: `m_stageCount` (3 when the extended span fits `16N`, else
   2 or 1 — degrades gracefully at huge N under the cap), per-stage decimation `D_k ∈
   {1, 4, 16}`, per-stage used-bin ranges from the crossovers `f_k = 0.4 · (fs/D_k)/2`,
   the stitched bin total, and the expected ring capacity. All scratch vectors resize
   here — steady state is allocation-free.
2. **Per tick** (`updateData`, `TimerEvents::uiTimeout` via QML `draw()`, main thread):
   - Copy + normalize (existing `m_center`/`m_halfRange` scaling) the newest `min(span,
     16N)` ring samples into the raw scratch, oldest→newest.
   - Cascade: FIR decimate ×4 (31-tap constexpr windowed-sinc, cutoff with margin below
     `0.5·fs/4`, unity DC gain) → stage-1 history (≤ 4N); same filter ×4 again → stage-2
     history (≤ N). Stateless per tick — no incremental filter state to invalidate.
   - Three FFTs through the single `N`-point plan (window applied per stage from the one
     `m_window`); short histories zero-pad exactly as the current filling-ring path does.
   - Stitch into `m_xData`/`m_yData`: stage 2 bins up to `f_2`, stage 1 bins `f_2..f_1`,
     stage 0 bins above `f_1`; X pushed as `log10(freq)` with the DC clamp at the finest
     `freqStep` (0014 convention); dB smoothing (3-bin boxcar) runs per stage segment so
     it never averages across a bin-spacing discontinuity.
   - Existing `downsampleMonotonic` → `PlotCurve` render, untouched. X stays monotonic
     non-decreasing across the stitch by construction (each region's bins are increasing
     and regions are disjoint and ordered).
3. **Ingest** (unchanged): `updateFftSeries` pushes `numericValue` per frame into the
   ring exactly as today; a bigger ring changes only how much history survives.

**Why the history may exceed the configured window (spec-constraint note):** the spec
said extra history must come from the widget's own retention; this design instead
lengthens the *existing Dashboard ring* at configure time because the widget has no way
to detect "new samples since last tick" from a ring snapshot (no monotonic push counter),
so widget-side incremental accumulation would require adding a counter to the ingest
path — strictly worse for the hotpath than a passive capacity change. The ring *is* the
widget's retention buffer in this architecture; only its length changes, the ingest loop
does not. Flagged here for explicit gate approval rather than silently reinterpreted.

## Hotpath & threading impact

- **Touches the hotpath?** One configure-time line in `Dashboard.cpp`
  (`configureFftSeries` ring sizing). The per-frame ingest walk (`updateFftSeries`) is
  untouched; `FixedQueue::push` is O(1) independent of capacity, so per-sample cost is
  identical. All new analysis (filters + 3 FFTs) runs in `FFTPlot::updateData` at UI
  cadence on the main thread — the same layer as 0014. `--benchmark-hotpath` gates it
  (AC5); memory ceiling: `kMaxExtendedFftSamples = 262144` doubles = 2 MB/widget worst
  case (typical audio: N=8192 → 1 MB).
- **New cross-thread signal/slot?** No. No new connections anywhere.
- **New input to a cached hotpath flag?** No. `fftLogX` is read at configure/ctor time
  only; reconfigure already fires on any project change.
- **Timestamp ownership** — untouched; the FFT path carries no timestamps.

## Data model & persistence

None. Zero new keys, no schema change, no migration — the feature keys off `fftLogX`
(spec 0014). Projects saved before/after this change are identical.

## API / SDK surface

None. No new fields; `--dump-api-schema` output is unchanged by this spec.

## QML / UI

None. The widget's QML contract (`minX/maxX/minY/maxY/logX` + carrier series) is
unchanged; the smoother low end comes purely from denser, finer bins in the carrier.
`PlotWidget.qml` ticks/cursors from 0014 apply as-is.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Analysis architecture | (a) decimation cascade + stitched same-size FFTs; (b) one 16N-point FFT; (c) constant-Q/Goertzel filterbank | **(a)** — 16× low-end Δf at ~3 same-size FFTs + cheap FIRs, keeps the high end on a short window (the "dynamic" behavior asked for); (b) drags 16× latency/smear onto the highs and 16× memory for one knob-free display; (c) is a new analysis engine with the hardest calibration story |
| Where the extra history lives | Widget-side incremental buffers (needs a new-sample counter on the ingest path) vs. lengthening the existing Dashboard ring at configure time | **Ring extension** — passive capacity change, ingest loop byte-identical; the counter alternative adds hotpath code to avoid a configure-time line. Deviates from the spec-constraint letter; flagged above for gate sign-off |
| Extension factor / stages | E=16, 3 stages (fixed) vs. adaptive stage count | **E=16 fixed, capped at 262144 samples total** — 16× covers the audible blocky region (two decades); above the cap (N > 16384) stages degrade to ×4/×1 gracefully. No knob (spec: no new configuration) |
| Seam treatment | Hard handoff at crossover bins vs. crossfade band | **Hard handoff first** — matched windows + normalization + the 0.4·Nyquist margin should make seams invisible after smoothing; a short linear crossfade is the named fallback if AC3 shows a step (isolated in the stitch function, cheap to add) |
| Decimator | Constexpr 31-tap windowed-sinc FIR ×4 (reused for both hops) vs. cascaded halfbands vs. IIR | **Single FIR ×4 reused** — stateless per tick (no history invalidation on reconfigure), linear phase (no seam phase tricks), unity DC gain for calibration; halfbands save ~half the MACs but double the code for a cost that is already negligible at UI cadence |
| Smoothing across the stitch | Global 3-bin boxcar vs. per-segment | **Per-segment** — averaging across a 4× bin-spacing jump would bias levels at the seam and break R3/R4 |

## Risks & mitigations

- **Seam visibility (R3/AC3)**: crossovers sit at 0.4× each stage's Nyquist, inside the
  decimator's flat passband; identical window + normalization on all stages. Fallback:
  linear crossfade over a few bins, isolated in the stitch step.
- **Level calibration (R4/AC4)**: unity-DC-gain FIR taps (normalized at definition) and
  the shared `1/N²` power norm; a swept-sine check is the acceptance gate.
- **Plan-rebuild path**: `updateData` currently assumes ring capacity == FFT size; with
  the extension that assumption breaks (the 0014 review note becomes load-bearing). The
  plan makes the expected capacity an explicit member; the rebuild path recomputes the
  full stage layout, so the runtime fallback stays coherent even though QML's CONSTANT
  bounds still only refresh on widget recreation (unchanged 0014 caveat).
- **Zero-pad startup**: cold-start ticks have short histories; stages zero-pad like the
  current filling-ring path, so the spectrum fades in rather than glitching.
- **Memory**: 2 MB/widget worst case at the cap; typical audio configs ≤ 1 MB. Named
  constant, documented in dashboard.md.
- **Scope creep guard**: Waterfall confirmed on a separate ring; the single
  `Dashboard.cpp` line is the entire blast radius outside the widget.

## Test & verification plan

- **Unit (runnable here)**: none — pure C++ display path, no JS surface. Verification is
  read-back + `code-verify.py`.
- **Maintainer observations (running app, audio source)**:
  - Broadband audio: smooth detailed low end, no straight-segment runs below 200 Hz
    **(AC1)**; compare against the reference analyzer character.
  - Two-tone 100 Hz + 126 Hz: two peaks with log on; one lump with log off **(AC2, R2,
    R5)**.
  - White noise: no visible seam/step across the axis **(AC3, R3)**.
  - Constant-amplitude sweep 50 Hz → 5 kHz: steady peak dB **(AC4, R4)**.
  - Linear-axis FFT projects render identically; `fftLogX: true` projects need no
    re-edit **(AC6)**; Task-Manager CPU sanity with one log FFT at default refresh
    **(AC7, R6)**.
- **Hotpath**: `--benchmark-hotpath` at existing gates **(AC5)** — the one-line configure
  change and unchanged push walk predict a no-op; the gate proves it.
- **Static**: `python scripts/code-verify.py --check` on the three C++ files;
  code review on the diff before handoff; the standard `sanitize-commit.py` pass runs
  before commit.
