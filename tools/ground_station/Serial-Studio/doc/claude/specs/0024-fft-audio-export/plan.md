---
spec: 0024-fft-audio-export
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-20
---

# Plan 0024 — Audio generation output for the FFT widget

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this — a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

A new commercial-only `Widgets::AudioExport` singleton mirrors `Widgets::ImageExport`
structurally (`DataModel::FrameConsumer<AudioExportItem>` + worker `QThread`, registered as
`Cpp_Audio_Export` in `ModuleManager`'s `BUILD_COMMERCIAL` block), but writes plain WAV
(float32 PCM) through `QFile` on the worker instead of Qt Multimedia — WAV is the only
container that accepts the default 100 Hz FFT rate, and it kills the dependency question.
Samples are captured at the single point where they are gap-free: the ingest tap inside
`Dashboard::updateFftSeries` / `updateWaterfallSeries`, where each `SeriesPush` gains a
commercial-guarded `record` flag + `sessionKey`; when armed, the push loop calls
`AudioExport::enqueueSample(key, value)` (lock-free `try_enqueue`, no allocation) right where
`p.buf->push(*p.value)` already runs. FFT and Waterfall widgets each gain an
`audioRecordingEnabled` property (commercial-guarded) whose setter opens/closes a session —
carrying sample rate and `fftMin`/`fftMax` scale resolved at session start — and arms/disarms
the Dashboard tap. Finalize writes the true-rate WAV (rescaling once when peak-normalized)
plus, for sub-8 kHz rates, an "audible" companion with the same data chunk at an
integer-multiplied header rate.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/UI/Widgets/AudioExport.h` **(new)** | Commercial SPDX. `AudioExportItem {float value; quint32 sessionKey;}` (8 bytes), `AudioSessionConfig` (rate, scale mode, center/halfRange, dataset/project titles), `AudioSession` worker state (QFile, sample count, running peak, drop counter), `AudioExportWorker : FrameConsumerWorker<AudioExportItem>`, `AudioExport : FrameConsumer<AudioExportItem>` singleton facade (`openSession`/`closeSession`/`closeAllSessions`/`enqueueSample`/`audioPath`/`setupExternalConnections`). |
| `app/src/UI/Widgets/AudioExport.cpp` **(new)** | Worker: WAV header write/patch, float32 append, finalize (peak rescale pass, zero-sample delete, audible companion emit). Facade: session registry keyed by `(widgetType, index)` packed key, Dashboard tap arm/disarm, auto-stop wiring. Queue config `{65536, 4096, 33}`. |
| `app/src/UI/Dashboard.h` | `SeriesPush` gains `bool record; quint32 sessionKey;` under `#ifdef BUILD_COMMERCIAL`; declare `setFftAudioTap(int index, bool, quint32)` / `setWaterfallAudioTap(...)` (commercial-guarded). |
| `app/src/UI/Dashboard.cpp` | Tap branch in `updateFftSeries` (~line 2469) and `updateWaterfallSeries` (~2686) push loops; taps default off in `configureFftSeries`/`configureWaterfallSeries` (~2669/~2723); arm/disarm setters. |
| `app/src/UI/Widgets/FFTPlot.h` / `.cpp` | `#ifdef BUILD_COMMERCIAL`: `Q_PROPERTY(bool audioRecordingEnabled ...)`, setter (runtime `FeatureTier` gate, parity with sweep setters), session open using ctor-resolved `m_samplingRate`/`m_scaleIsValid`/`m_center`/`m_halfRange` + dataset title; dtor closes the session (dashboard rebuild destroys widgets). |
| `app/src/UI/Widgets/Waterfall.h` / `.cpp` | Same property/lifecycle as FFTPlot (already commercial-only files). |
| `app/qml/Widgets/Dashboard/FFTPlot.qml` | Two `DashboardToolButton`s (record toggle + open-folder), `visible: Cpp_CommercialBuild`, alongside the existing buttons at ~line 170. |
| `app/qml/Widgets/Dashboard/Waterfall.qml` | Same two buttons. |
| `app/src/Misc/ModuleManager.cpp` | `#include` at ~124 and singleton + `setupExternalConnections()` + `setContextProperty("Cpp_Audio_Export", ...)` in the `BUILD_COMMERCIAL` block at ~864-872, adjacent to `ImageExport`. |
| `app/CMakeLists.txt` | `AudioExport.h`/`.cpp` in the commercial header/source lists (adjacent to `ImageExport` entries, ~664/~764). |

No `tests/` files: there is no pytest path to widget toolbars or workspace files; verification
is maintainer ACs + the benchmark gate (below).

## Architecture & data flow

```
FrameBuilder → Dashboard::hotpathRxFrame → updateDashboardData → updateFftSeries /
updateWaterfallSeries  [main thread]
        └─ per SeriesPush: p.buf->push(*p.value); if (p.record)
           AudioExport::enqueueSample(p.sessionKey, *p.value)   ← lock-free SPSC enqueue
                                    ↓ moodycamel queue (FrameConsumer spine)
        AudioExportWorker::processItems  [worker QThread, 33 ms / 4096-item drain]
                └─ per session: append float32 → session QFile
        closeSession → finalize: patch WAV header; if peak-normalized, one rescale pass;
        if rate < 8000 Hz, emit companion WAV (same data chunk, header rate = rate × factor,
        factor = ceil(8000 / rate), suffix "-audible-<factor>x.wav"); delete zero-sample files.
```

- **Session lifecycle (main thread)**: `FFTPlot::setAudioRecordingEnabled(true)` →
  `AudioExport::openSession(key, config)` (config marshaled to the worker via queued invoke,
  the `setSnapshotIntervalMs` precedent) → `Dashboard::setFftAudioTap(m_index, true, key)`.
  Disable/dtor reverses. Session keys pack `(SerialStudio::DashboardWidget kind, index)` so an
  FFT and a Waterfall on the same dataset record independent files (spec AC6).
- **Reconfigure**: `configureFftSeries`/`configureWaterfallSeries` rebuild push tables with
  taps off; dashboard rebuilds destroy and recreate widgets, whose dtors close their sessions
  — no re-arm path needed, no stale index can fire.
- **Auto-stop** (`setupExternalConnections`, mirroring `ImageExport.cpp:370-382`):
  `ConnectionManager::connectedChanged` + `pausedChanged` → `closeAllSessions()`. Additionally
  the three player `openChanged` signals → `closeAllSessions()`: replay frames re-enter
  `hotpathRxFrame`, so recording must disarm *before* replay pushes reach the rings —
  a per-sample `isAnyPlayerOpen()` poll is banned by the cached-flag rule.
- **Output paths**: `Misc::WorkspaceManager::instance().path("Audio Recordings")` /
  project title / dataset title / `yyyy-MM-dd_HH-mm-ss-zzz.wav` (the `ensureSession()` /
  `imagesPath()` shape from `ImageExport.cpp:184-196,340-346`); `audioPath()` Q_INVOKABLE
  backs the QML open-folder button.
- **Amplitude mapping (worker)**: scale-mode sessions map `(value - center) / halfRange`
  (same fields FFTPlot's ctor already derives from `fftMin`/`fftMax`, clamped to ±1);
  fallback sessions store raw float32 and finalize with one deterministic rescale to
  peak × 10^(-1/20) (~1 dB headroom). Float32 WAV (format tag 3) sidesteps quantization
  order-of-operations entirely.

## Hotpath & threading impact

- **Touches the hotpath?** Yes — dashboard ingest (`updateFftSeries` / `updateWaterfallSeries`).
  Preserved as follows: the tap is one `bool` test per push entry when disarmed (fields live in
  the already-walked `SeriesPush`, no new lookup, no map, no signal hop); armed cost is one
  `try_enqueue` of an 8-byte POD into a preallocated moodycamel SPSC queue — no allocation, no
  lock, no Frame copy, and `FrameReader`/`CircularBuffer`/`FrameBuilder`/span lane untouched.
  Implementer must invoke `ss-hotpath` and read `Dashboard.cpp` regions in full before editing.
  Gate: `--benchmark-hotpath` unchanged (the `lua+dashboard` 0.5× floor covers this path;
  taps-off is the benchmark state, satisfying spec R9/AC8).
- **New cross-thread signal/slot?** Yes — facade→worker config/close hops via
  `QMetaObject::invokeMethod(..., Qt::QueuedConnection)`, identical to the ImageExport/CSV
  worker precedent. Sample flow itself is the lock-free queue, not signals. All auto-stop
  connections are main-thread→main-thread on the facade.
- **New input to a cached hotpath flag?** The `record` flags in `SeriesPush` *are* a new
  cached ingest input. Refresh discipline: they are written only by the main-thread
  arm/disarm setters, cleared unconditionally on every push-table rebuild, and every
  external condition that must stop capture (disconnect, pause, player open, license loss on
  `activatedChanged`) routes through `closeAllSessions()` → disarm — wired in
  `setupExternalConnections` with direct (default, same-thread) connections. No existing
  cached flag (`m_streamAvailable`, `m_anyAsyncSink`, ...) gains a new input.
- **Timestamp ownership** — no timestamps are captured or re-stamped anywhere: sample time is
  implied by position × 1/rate (that is what a WAV is). The worker never calls a clock.

## Data model & persistence

None. No `Frame.h` keys, no project JSON, no `widgetSettings`, no Sessions schema. The toggle
is runtime-only widget state (dies with the widget, parity with ImageView's export toggle);
a persisted master flag can follow later if wanted. Output artifacts are ordinary files under
the workspace tree.

## API / SDK surface

None in v1. No handler, no EnumLabels, no SDK regen. (A `fftAudio.start/stop` API can be a
follow-up; nothing here blocks it.)

## QML / UI

Two `DashboardToolButton`s per widget QML (record toggle bound to
`root.model.audioRecordingEnabled`, checked-state icon parity with ImageView's camcorder
button; open-folder calling `Cpp_Misc_Utilities.revealFile(Cpp_Audio_Export.audioPath(...))`),
both `visible: Cpp_CommercialBuild` (the `Plot.qml:89` sweep-gating pattern — build-time gate
in QML, runtime `FeatureTier` gate in the C++ setter, so GPL builds render nothing and
unlicensed commercial builds get the license dialog behavior of other Pro setters). No new
components, no ComboBox, no theme surface.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Capture point | **Ingest-tap** (flag in `SeriesPush` at the ring-push site) vs Widget-delta (read fresh ring tail per UI tick) vs Builder-tap (`FrameBuilder` dataset hook) | **Ingest-tap.** Widget-delta silently drops audio whenever frames-per-UI-tick exceeds ring capacity (`fftSamples`, default **256** — ~15 kHz suffices to overflow at 60 Hz); Builder-tap sits on the parse hotpath proper for zero benefit. |
| Encoder | **Plain WAV via QFile** vs `QMediaRecorder`+`QAudioBufferInput` | **Plain WAV.** Backend AAC encoders reject sub-8 kHz rates (default FFT rate is 100 Hz), and the recorder's async start/stop event-loop dance (ImageExport needs 1.5 s/3 s `QEventLoop` waits) buys nothing for uncompressed PCM. |
| Sample format | **Float32 PCM (fmt 3)** vs int16 PCM | **Float32.** Deterministic mapping without dither/quantization decisions, native for Audacity/MATLAB/Python, and finalize-rescale is exact. 4 bytes/sample is irrelevant at these rates. |
| Duplicate semantics | **Ring fidelity** (record exactly what the FFT ring receives, incl. `dashboardTick`/`reprocessFrames` republishes) vs wire fidelity (only genuine device frames — needs provenance plumbed through `TimestampedFrame`) | **Ring fidelity.** The recording equals the sample stream the widget analyzed (WYSIWYG, same philosophy as marker monitoring); republishes only occur in ProjectFile mode via control-script/API ticks, where the tick *is* the data clock for table-fed datasets. Zero hotpath surface added. |
| Normalization | **Finalize-pass rescale** (store raw, rescale once at close) vs streamed running gain | **Finalize-pass.** A running gain bakes a loudness ramp into the file — non-deterministic and fails spec R4's determinism; one linear pass over ≤ minutes of float32 at close is negligible. |
| Queue overflow | **Count + warn** (drop counter per session, `qWarning` at finalize) vs blocking producer | **Count + warn.** Blocking the GUI thread on disk I/O is forbidden; `{65536, 4096, 33 ms}` absorbs 48 kHz × 40+ s of worker stall, so drops indicate real pathology, not steady state. |

## Risks & mitigations

- **Stale/misaligned tap after reconfigure** (widget indices shift): taps are cleared on every
  push-table rebuild and only widget setters re-arm; widget destruction closes its session via
  dtor. No path leaves a `record` flag pointing at a re-indexed widget.
- **Replay contamination** (AC9): player `openChanged` → `closeAllSessions()` fires before
  replay frames flow; recording is disarmed, not filtered per-sample (cached-flag rule,
  `common-mistakes.md` row 18).
- **License loss / late activation**: `LemonSqueezy::activatedChanged` → close sessions when
  Pro drops (the license-gated-state rule from CLAUDE.md "Startup"); enable path is already
  runtime-gated in the setter.
- **moc + `#ifdef BUILD_COMMERCIAL`** on FFTPlot's new Q_PROPERTY: moc honors preprocessor
  conditionals and `Dashboard.h` already ships `#ifdef`'d members — follow that exact shape;
  GPL builds compile FFTPlot without the property (spec R10/AC10).
- **WAV header correctness on abnormal end** (AC5): write the header with placeholder sizes at
  open, patch RIFF/data sizes on every flush batch (cheap seek+write), so even a crash leaves
  a file Audacity can open; finalize does the authoritative patch.
- **`Q_INVOKABLE` ban**: `audioPath` mirrors `ImageExport::imagesPath` — note that file uses
  `Q_INVOKABLE` with a return value, which is the sanctioned exception (the ban is on
  `Q_INVOKABLE void`; those become slots).
- **Scope discipline**: the Waterfall negative-size ring hole flagged in dashboard.md stays
  out of lane; nothing outside the file table above gets touched.

## Test & verification plan

- **Unit (runnable here):** none — no JS-parser surface. WAV-writer correctness is covered by
  a maintainer spot-check (AC1/AC2) because the writer lives behind Qt file I/O; if review
  wants more, a follow-up could extract the header math into a header testable via
  `tests/scripts`-style tooling, but v1 keeps the surface minimal.
- **Integration (maintainer, app running):** spec AC1–AC7 as in-app observations — rocket
  demo / sine TCP feed, two rates (100 Hz, 8 kHz), scale vs peak-normalized datasets,
  FFT+Waterfall same-dataset parallel sessions, disconnect mid-recording, replay inertness,
  workspace folder reveal.
- **Hotpath:** `--benchmark-hotpath` (user-run/CI) must pass unchanged — taps-off state; the
  `lua+dashboard` floor row exercises the modified push loops (AC8).
- **GPL build check (maintainer):** compile with `BUILD_COMMERCIAL=OFF` — no toolbar buttons,
  no symbols (AC10).
- **Static:** `python scripts/code-verify.py --check` on every touched file (hotpath
  violations are blockers); `qt-cpp-review` before handoff; `python scripts/sanitize-commit.py`
  before commit.
