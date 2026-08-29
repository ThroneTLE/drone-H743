---
spec: 0017-fft-ballistics
phase: plan
status: approved     # approved 2026-07-17
updated: 2026-07-17
---

# Plan 0017 — FFT display ballistics

## Approach

Two dataset fields (`bool fftBallistics = false`, `int fftBallisticsRelease = 300`)
following the exact 0014 plumbing (Keys:: → guarded serialize → ss_jsr read → editor
rows → commit cases → API `takeParam` → round-trip pytest).
`Widgets::FFTPlot` keeps a per-bin display state `m_displayDb` and blends each fresh dB
value: `disp = fresh >= disp ? fresh : disp + (fresh - disp) * alpha` with
`alpha = 1 - exp(-dt / tau)` from a `QElapsedTimer` (wall-clock, so any refresh rate
decays identically; the first frame jumps straight to the fresh value). Release clamped
`qBound(50, ms, 5000)` at the widget. `m_displayDb` resizes wherever the bin count is
(re)computed. Display-only; no Dashboard change.

> Post-0018 note: the blend now lives in `computeBinSpectrum`, applied per FFT bin
> upstream of both the linear and log emit paths (the multi-res per-stage indexing this
> plan originally described was removed with spec 0018); the editor rows landed in
> `buildFftGeneralRows`. Behavior is as specified here.

## Affected files

| File | Change |
|------|--------|
| `app/src/DataModel/Frame.h/.cpp` | fields, `Keys::FFTBallistics`/`FFTBallisticsRelease`, guarded serialize (both keys only when enabled), reads (false / 300) |
| `app/src/DataModel/Project/ProjectEditorItemIds.h` | `kDatasetView_FFT_Ballistics`, `kDatasetView_FFT_BallisticsRelease` |
| `app/src/DataModel/Project/ProjectEditorForms.cpp` | CheckBox + IntField rows in `buildFftRangeRows`, editable on `dataset.fft` |
| `app/src/DataModel/Project/ProjectEditorCommit.cpp` | bool case in `onDatasetFlagItemChanged`; int case in `onDatasetFftItemChanged` |
| `app/src/UI/Widgets/FFTPlot.h/.cpp` | members (`m_ballistics`, `m_releaseMs`, `m_releaseAlpha`, `m_displayDb`, `QElapsedTimer`, stage emit offsets), `applyBallistics()` blend at both push sites, per-frame alpha update, resize hooks in ctor/`rebuildFftPlan`/`computeStageLayout`/`resyncLogModeConfig` |
| `app/src/API/Handlers/ProjectHandlerEntities.cpp` + `ProjectHandler.cpp` | `takeParam` blocks + doc string |
| `tests/integration/test_project_editor.py` | `test_fft_ballistics_round_trip` |
| `doc/claude/architecture/dashboard.md` | one sentence in the 0016 section |

## Hotpath & threading impact

None. Widget draw layer only, wall-clock display animation (not frame timestamping —
source-owns-time is untouched). No new connections, no cached flags, no allocation at
steady state (`m_displayDb` sized at configure/layout time).

## Test & verification

AC1/AC2 maintainer observation; AC3 = the new pytest (app + API server); AC4 =
benchmark gate. Static: code-verify + clang-format on the changed files, then the
standard `sanitize-commit.py` pass before commit.
