---
spec: 0016-multires-fft
phase: tasks
status: approved     # draft -> approved (gate before /ss-implement)
updated: 2026-07-17
---

# Tasks 0016 — Multi-resolution FFT display for the logarithmic frequency axis

> **Superseded by [spec 0018](../0018-smooth-log-fft/spec.md).** All tasks below were
> completed and the feature worked as specified; the implementation was then removed the
> same day in favor of the single-window approach (see the 0018 spec for the rationale).
> The checklist is kept as the record of what was built and reviewed.

## Conventions

- One task = one focused, reviewable change. If a task touches >3 files or needs a paragraph
  to describe, split it.
- **Verify** is how *this* unit is confirmed before moving on — usually
  `python scripts/code-verify.py --check <files>`, plus a test or a read-back where one fits.
- **Deps** lists task IDs that must land first.
- Order so the tree compiles (conceptually) after each task where practical.

## Tasks

### T1 — Extended FFT ring capacity (the single Dashboard touch)

- **Files:** `app/src/UI/Dashboard.cpp`
- **Does:** In `configureFftSeries()` (~:2470), size the ring
  `dataset.fftLogX ? extendedFftCapacity(dataset.fftSamples) : dataset.fftSamples`, with a
  file-local `extendedFftCapacity(N) = min(16 * N, kMaxExtendedFftSamples = 262144)` helper
  (named constant + brief). **Binding invariants: this is the ONLY line in Dashboard.cpp
  that changes — the push-table build (:2474-2484) and the per-frame `updateFftSeries`
  walk stay byte-identical; `FixedQueue::push` is O(1) regardless of capacity, which is
  what keeps the ingest path a provable no-op for `--benchmark-hotpath`.**
- **Verify:** `python scripts/code-verify.py --check app/src/UI/Dashboard.cpp`; `git diff
  app/src/UI/Dashboard.cpp` shows exactly the sizing expression + helper, nothing else.
- **Deps:** none
- [x] done (constant hoisted to the Constants banner per linter)

### T2 — FFTPlot stage layout + expected-capacity plumbing

- **Files:** `app/src/UI/Widgets/FFTPlot.h`, `app/src/UI/Widgets/FFTPlot.cpp`
- **Does:** Adds the multi-res state and configure logic, no analysis yet: members for
  stage count, per-stage decimation {1,4,16}, per-stage used-bin crossover indices
  (0.4× each stage's Nyquist), expected ring capacity, and the scratch buffers
  (raw-normalized copy, two decimated histories, stitched x/y sizing) — **all sized in
  ctor/`rebuildFftPlan`, allocation-free at steady state; stage count degrades (3→2→1)
  when the capped ring span can't cover 16N.** Fixes the plan-rebuild check in
  `updateData` (:363-365): compare ring capacity against the *expected* capacity member
  instead of assuming capacity == FFT size (**binding invariant: in log mode capacity is
  E·N while the analysis size stays N — deriving `m_size` from capacity would silently
  16× the FFT**). `applyLogFrequencyBounds` lower bound moves to the finest stage's first
  bin `fs/(16N)` (recomputed with the stage layout on rebuild). Linear mode: every new
  member inert, behavior byte-identical.
- **Verify:** `python scripts/code-verify.py --check` on both files; read-back that the
  linear path hits no new branch and the rebuild check covers both modes.
- **Deps:** T1 (capacity contract exists)
- [x] done (log mode never called rebuildFftPlan — layout recomputed from capacity while
  the plan stayed at N; this also resolved the CONSTANT-bounds review note carried over
  from 0014)

### T3 — Decimation cascade

- **Files:** `app/src/UI/Widgets/FFTPlot.cpp` (+ declarations in `FFTPlot.h`)
- **Does:** The stateless per-tick cascade: copy + normalize (existing
  `m_center`/`m_halfRange` scaling) the newest `min(span, 16N)` ring samples
  oldest→newest into the raw scratch; FIR-decimate ×4 into stage-1 history and ×4 again
  into stage-2, using one constexpr symmetric 31-tap windowed-sinc lowpass (**binding
  invariants: unity DC gain — taps normalized at definition — so calibration survives
  (R4); linear phase; stateless per tick so reconfigure can't leave stale filter state;
  fixed loop bounds from the configure-time sizes**). Short histories leave leading
  zeros (cold-start fade-in, matching the existing filling-ring behavior).
- **Verify:** `python scripts/code-verify.py --check` on both files; read-back: taps sum
  to 1 (comment the normalization), no allocation inside the tick path.
- **Deps:** T2
- [x] done (39-tap Blackman-windowed sinc, unity DC, -77 dB at the alias-fold band)

### T4 — Three-stage analysis + stitched spectrum

- **Files:** `app/src/UI/Widgets/FFTPlot.cpp` (+ declarations in `FFTPlot.h`)
- **Does:** The log-mode `updateData` branch: run the newest-N window of each stage
  history through the **single** N-point kissfft plan (shared `m_window`, shared `1/N²`
  power norm — **binding invariant: identical window + normalization per stage is what
  makes the dB trace continuous across seams, R3/R4**); generalize
  `computeSmoothedSpectrum` to emit the stitched (freq, dB) sequence — stage 2 bins up
  to f₂, stage 1 to f₁, stage 0 above — with per-stage `freqStep`, per-segment 3-bin
  smoothing (**never average across the bin-spacing jump**), log10 X with the DC clamp
  at the finest `freqStep`, and **X monotonic non-decreasing across the stitch
  (downsampler precondition; regions are disjoint and ordered by construction — assert
  it)**. Linear mode: the existing single-stage path runs unchanged (R5).
- **Verify:** `python scripts/code-verify.py --check` on both files; read-back of the
  stitch bounds (each stage used only below 0.4× its own Nyquist) and the monotonic
  assert.
- **Deps:** T3
- [x] done (sine calibration confirmed; spec R3/AC3 amended for the physical
  noise-density shelf)

### T5 — Architecture doc note

- **Files:** `doc/claude/architecture/dashboard.md`
- **Does:** One paragraph under the FFT/plot material: extended FFT ring (16×, capped,
  log-axis datasets only, FFTPlot sole consumer), the three-stage decimate+stitch
  analysis in FFTPlot, and the capacity≠FFT-size contract for anyone touching
  `configureFftSeries` or the rebuild check later.
- **Verify:** read-back for accuracy against the landed code.
- **Deps:** T4 (describe what actually landed)
- [x] done (section later rewritten by spec 0018 when the approach changed)

### T6 — Realtime feel + axis floor (field feedback, R9/R10)

- **Files:** `app/src/UI/Widgets/FFTPlot.cpp` (+ constants)
- **Does:** Bounds the finest stage by observation time (`kMaxFinestWindowSec = 0.4`):
  stage count = min(capacity-allowed, time-allowed), so audio-sized windows stop
  watching multi-second histories (**binding invariant: Δf = 1/T — the budget IS the
  resolution choice; a window already ≥ 0.4 s stays single-stage by user intent**).
  Axis lower bound: the finest stage's first bin, superseding an interim ~100 Hz
  default — nothing the analysis captures is cropped.
- **Verify:** `python scripts/code-verify.py --check` on the file; read-back of the
  stage-budget math at N=1024/8192/65536 with fs=44100.
- **Deps:** T4
- [x] done (time budget also structurally resolves the review's large-N per-tick cost
  concern: 3 stages exist only when the whole span is ≤ 0.4 s of samples)

## Definition of Done

Closed as superseded (see header note); the items below record the state at handoff.

- [x] `python scripts/code-verify.py --check` clean on all changed files (0 errors,
      0 advisories).
- [x] Code review run on the C++ diff. Fixed: hostile-`fftSamples` clamp in
      `configureFftSeries` (unclamped project input could overflow the 16x extension
      into a negative ring capacity; the clamp outlived the feature and remains in the
      0018 code); live-reconfigure re-derivation of the analysis config; assertion
      additions. Accepted-as-noted: the pre-existing unclamped Waterfall ring sizing
      (`configureWaterfallSeries`, out of this spec's scope, tracked separately);
      CONSTANT-bounds staleness under live reconfigure (accepted 0014 caveat); the
      noise-density seam shelf (deliberate sine calibration, spec R3).
- [x] No runnable pytest target (display-only feature; no API/schema delta).
- [x] Diff limited to `FFTPlot.h/.cpp`, `Dashboard.cpp` (`configureFftSeries` only), and
      one dashboard.md section.
- [x] `spec.md` status: shelved (superseded by 0018).
