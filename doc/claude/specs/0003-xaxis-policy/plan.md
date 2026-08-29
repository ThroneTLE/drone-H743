---
spec: 0003-xaxis-policy
phase: plan
status: approved      # draft -> approved (gate before /ss-tasks)
updated: 2026-07-06
---

# Plan 0003 - Plot X-axis pipeline unification

> **Phase 2 of 4 - the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read `doc/claude/architecture.md` (Plot X-Axis, TimeRing, push
> tables) and the *actual code* at the cited lines before touching anything.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Resolve the Time/Samples/Dataset mode decision once and cache it, instead of re-deriving it
from raw `xAxisId` sentinels in ~8 UI-layer sites. Introduce an `XAxisMode` enum + an
`XAxisPolicy` POD in `SerialStudio.h`, a single `datasetXAxisEnabled()` Pro predicate, and a
single `groupXAxisMode()` reader of the front-dataset group encoding. The already
mode-specialized render machinery (feed lambdas, TimeRing, SweepEngine, downsamplers, push
tables) is left exactly as-is; only the decision above it is unified. Tonight (C-S1..C-S5) is
low-risk, additive, and hotpath-neutral: doc fix, licensing predicate, additive policy types +
single-reader swaps, a brace-scope on `PlotClock`, and removal of a dead Y-ring allocation.
C-S6..C-S9 are compile-gated follow-on specs that adopt the policy at the remaining fork
sites, add a real group-level field, reconcile the range engines, and (only under a hard
benchmark gate) unify push-struct storage.

## Affected subsystems & files

Line refs live in the per-stage sections; this table is the file-to-stage map.

| File | Stages | Role |
|------|--------|------|
| `doc/claude/architecture.md` | C-S1 | Rewrite "Plot X-Axis" bullet (`:438-473`). |
| `app/src/SerialStudio.h` | C-S2, C-S3 | Declare predicate + policy types (~`:250`). |
| `app/src/SerialStudio.cpp` | C-S2, C-S3 | Define predicate + helpers (mirrors `:44-68`). |
| `app/src/UI/Widgets/Plot.cpp` | C-S2, C-S6, C-S8 | Gates; policy adoption; range engine. |
| `app/src/UI/Dashboard.cpp` | C-S2..C-S6, C-S9 | Gates, group helper, brace, Y-ring, ring key. |
| `app/src/UI/Dashboard.h` | C-S5, C-S6 | `m_pltNullY` (`:424`); `m_xPolicy` cache. |
| `app/src/DataModel/Project/` | C-S3, C-S7 | `buildGroupXAxisRow` (Forms); dual-write (Commit). |
| `app/src/DataModel/Project/` | C-S7 | `migrateGroupXAxisIds()` (Loading, after :158). |
| `app/src/UI/Widgets/MultiPlot.cpp` | C-S6, C-S8 | `groupXAxisMode` ctor; range engine. |
| `app/src/DataModel/Frame.h` | C-S7 | `Group::xAxisId` (~`:465`), sparse serialize. |
| `app/src/API/.../DashboardHandler.cpp` | C-S9 | Ring-key routing (C-S5 verify target). |
| `app/CMakeLists.txt` | C-S8 | New source entry (`:230`/`:242`). |

## Architecture & data flow

The X-axis mode is a property of a dataset (per-dataset combo = Time | Samples | every other
dataset; `ProjectModel::xDataSources` at `:501`) or, for a multiplot, a property of the group
(GROUP combo = Time | Samples only, today fanned into member datasets and read back via
`front()`). At configure/reconfigure time `Dashboard` builds its plot carriers and picks the
render arm; at draw time (60 Hz UI, and per-frame push at the hotpath) the already-specialized
machinery runs. `resolveXAxisPolicy` / `groupXAxisMode` sit strictly at the configure/UI layer:
they translate sentinels into an enum once, and every consumer branches on the enum. The three
render arms (sweep-ring / time-ring / sample-ring) and their push tables are unchanged; see
`doc/claude/architecture.md` "Plot X-Axis" and the TimeRing / push-table sections.

## Hotpath & threading impact

- **Touches the hotpath?** No, for C-S1..C-S8. All edits are at configure/construct time or UI
  draw rate; the feed lambdas, TimeRing, SweepEngine, downsamplers, and push tables are
  untouched. **C-S9 is the only stage that touches per-fire push cost** and is gated on
  `--benchmark-hotpath` (see C-S9). Per-stage impact table below.
- **New cross-thread signal/slot?** No. No new connections in any stage.
- **New input to a cached hotpath flag?** No. `resolveXAxisPolicy`/`groupXAxisMode` are pure
  reads of the project model at configure time; C-S6 caches `m_xPolicy` in `Plot`, refreshed on
  the same configure path that already rebuilds carriers - it is not a per-frame flag.
- **Timestamp ownership** - unchanged; the source still stamps at the driver boundary.

### Per-stage hotpath-impact table

| Stage | Impact |
|-------|--------|
| C-S1 | none (docs) |
| C-S2 | none (configure/draw-rate; cost unchanged) |
| C-S3 | none (configure/ctor/editor) |
| C-S4 | lexical only (`diff -w` ~= braces) |
| C-S5 | none (`configureLineSeries` only) |
| C-S6 | configure only (push tables provably identical + benchmark belt-and-braces) |
| C-S7 | none |
| C-S8 | none (draw-rate) |
| C-S9 | **YES** (hard benchmark gate) |

## Data model & persistence

Only C-S7 touches persistence. `Group::xAxisId` (default `kXAxisTime`) reuses `Keys::XAxis`
("xAxis", `Frame.h:94`; group and dataset scopes are disjoint so no key collision), serialized
sparsely and read with a clamp. No `SchemaVersion` bump; forward/backward compatibility is
handled by a dual-write window and the fact that old apps ignore an unknown group key. Full
schema evolution in the C-S7 section below.

## API / SDK surface

C-S7 only: the API `groupUpdate` handler accepts `xAxisId` (`-2`/`-1` only, warn otherwise)
with a help string (~`ProjectHandler.cpp`). No new handler, no `EnumLabels` change, no SDK
regeneration beyond the added field. Commercial dataset-X stays behind `BUILD_COMMERCIAL` via
`datasetXAxisEnabled()`.

## QML / UI

No new QML components. `Plot.qml:84` already gates sweep on `model.timeAxis`; that invariant is
preserved. Tick formatting stays presentation-only. The multiplot group combo (Time | Samples)
is unchanged in C-S3; in C-S7 it reads/writes the real group field.

---

## Stages

### C-S1 (TONIGHT) - architecture.md truth restore

**Transformation.** Rewrite the "Plot X-Axis" bullet (`architecture.md:438-473`) to match the
code. Doc-fix list, exact:

1. Retitle "Plot X-Axis (Time / Samples / Dataset)". Replace the false claim (`:439-441`,
   "kXAxisSamples removed as user option... deserialize maps -1 -> -2") with: three live modes;
   Samples is live + free (shared monotonic index ring `m_pltXAxis` `fillRange` + per-dataset
   Y ring + `downsampleMonotonic`); deserialize preserves `-1` verbatim (`Frame.cpp:301`);
   `migrateLegacyXAxisIds` keeps Time AND Samples, remaps legacy positive frame-indices to
   uniqueIds, maps other `<=0`/unresolvable to Time (`ProjectModelLoading.cpp:158`).
2. Add selector reality: per-dataset combo = Time | Samples | every dataset
   (`ProjectModel::xDataSources` `:501`); multiplot GROUP combo = Time | Samples only, fans the
   value into every member dataset's `xAxisId` with `front()` canonical read
   (`ProjectEditorCommit.cpp:268`, `useTimeXAxisGroup`) - a known encoding wart slated for a
   group-level field.
3. `:473` "Dataset-X plots, FFT, GPS, 3D keep the raw-ring + downsample path" -> add Samples
   plots (consistent with `:484`).
4. Add: unlicensed / Trial-expired dataset-X silently degrades to Samples (gates ->
   `datasetXAxisEnabled()` after C-S2).
5. Add carrier invariant: `m_pltValues` holds one LineSeries per plot widget INCLUDING time
   plots (index alignment + `size() != plotCount` reconfigure trigger `:2007`); time carriers
   are placeholders (empty Y after C-S5).

**Files.** `doc/claude/architecture.md`. **Risk.** None (docs). **Hotpath.** None.
**Verify.** `python scripts/documentation-verify.py`; re-read `:438-473` against `Frame.cpp:301`
and `ProjectModelLoading.cpp:158`.

### C-S2 (TONIGHT) - datasetXAxisEnabled() + 5-site replacement

**Transformation.** Add `SerialStudio::datasetXAxisEnabled()` and route all five copy-pasted
gates through it. `#ifdef BUILD_COMMERCIAL` internals:
`tk.isValid() && SS_LICENSE_GUARD() && tk.featureTier() >= Licensing::FeatureTier::Trial` -
**no** `variantName()` clause (that is `proWidgetsEnabled()`'s extra check). Replace at
`Plot.cpp:94-102`, `:690-703`, `Dashboard.cpp:2361-2368`, `:2591-2594`, `:2645-2653`.

Compile-safety, per site: the `#ifdef` pairs collapse because the helper returns `false` in GPL
- the Dataset branch never fires and `m_datasets.contains(-1)` is always false. Remove
now-unused `tk`/`tk2` locals in the same edit (unused-var warnings are fatal). `Plot.cpp:701`'s
`if (false) {}` `#else` stub disappears; the `calculateAutoScaleRange` chain becomes a plain
`if / else-if`. `registerXAxisIfNeeded` keeps its early return; drop `Q_UNUSED(dataset)`. Remove
the `Licensing/CommercialToken.h` include + its guard from `Plot.cpp` once both sites convert.

**Files.** `SerialStudio.h` (declare ~`:251`), `SerialStudio.cpp` (define), `Plot.cpp`,
`Dashboard.cpp`. **Risk.** Low; the two Sweep-setter gates (`Dashboard.cpp:1369`, `:1402`) share
the predicate but gate Sweep - leave them. **Hotpath.** None (configure/draw-rate; cost
unchanged). **Verify.** `grep -n "FeatureTier::Trial" Plot.cpp Dashboard.cpp` -> only the two
Sweep setters remain; the replaced predicate is token-identical to the helper body; no orphaned
`tk` locals.

### C-S3 (TONIGHT) - additive policy types + groupXAxisMode single-reader

**Transformation.** Add the `XAxisMode`/`XAxisPolicy` types and three declarations;
`resolveXAxisPolicy` is added-not-consumed tonight (no call sites). Swap the two group-mode
inline reads to the single reader:

- `Dashboard.cpp:326` (`useTimeXAxisGroup`):
  `return !group.datasets.empty()
   && SerialStudio::groupXAxisMode(group) == SerialStudio::XAxisMode::Time;`
- `ProjectEditorForms.cpp:203` (`buildGroupXAxisRow`):
  `const bool samples =
   SerialStudio::groupXAxisMode(group) == SerialStudio::XAxisMode::Samples;`

**Files.** `SerialStudio.h`/`.cpp` (+types +3 decls), `Dashboard.cpp:326`,
`ProjectEditorForms.cpp:203`. **Risk.** Low but behavior-critical - the swap must be byte-for-byte
behavior-identical. **Hotpath.** None (configure/ctor/editor).

**Verify.** `grep -rn "datasets.front().xAxisId" app/src` -> only inside `groupXAxisMode`;
`resolveXAxisPolicy` has no call sites yet (grep).

#### C-S3 truth-table derivation (empty-group asymmetry preservation) - IN FULL

`groupXAxisMode(group)` returns:

- **Samples** iff `front().xAxisId == kXAxisSamples`
- **Time** iff `front().xAxisId == kXAxisTime`
- **Dataset otherwise** (`front().xAxisId >= 0`)

The two call sites compare against different enum values, and *that* is what preserves the
existing asymmetry:

| Group state | `groupXAxisMode` | Dashboard (`== Time`) | ProjectEditor (`== Samples`) |
|-------------|------------------|-----------------------|------------------------------|
| front `== kXAxisTime` | Time | true -> time path | samples=false -> "Time" combo |
| front `== kXAxisSamples` | Samples | false -> samples path | samples=true -> "Samples" combo |
| front `>= 0` (dataset) | Dataset | false -> samples path | samples=false -> "Time" combo |
| **empty group** | Time | `!empty` guard -> samples path | samples=false -> "Time" combo |

Both `>= 0` and empty-group rows land on the same behavior as today; the "(preserved)" note is
in the prose below rather than the cells, to keep the rows under 100 columns.

So: Dashboard compares `== Time`, meaning a `front >= 0` group is "not Time" and takes the
samples path exactly as before; ProjectEditor compares `== Samples`, meaning a `front >= 0`
group is "not Samples" and shows the "Time" combo exactly as before. The empty group returns
`Time` from the helper, but Dashboard keeps its `!empty` guard so it still lands on the samples
path, and ProjectEditor gets `samples == false` so it still shows "Time". **Both sites are
byte-for-byte behavior-identical to today; the P10 asymmetry is now visible in one documented
place instead of two inline reads.**

### C-S4 (TONIGHT) - PlotClock brace scope

**Transformation.** In `Dashboard.cpp` `hotpathRxFrame`, wrap `:1463-1501` (from the
`PlotClock& clk` declaration through `m_plotDisplayTimeSec = displayNext`) in a brace scope so
`clk` structurally cannot outlive `reconfigureDashboard`'s move-assign (`:1710-1711`). The
`// code-verify off` fence comment shrinks. Pure code motion, zero codegen.

**Files.** `Dashboard.cpp`. **Risk.** Minimal (lexical). **Hotpath.** Lexical only; `diff -w`
is essentially just braces. **Verify.** `git diff -w` shows only braces + the shrunk comment;
nothing after the closing brace references `clk`.

The stronger form - extract a static `advancePlotClock` - is deferred to C-S6: it must be a
member to name the private nested struct and to see the file-local `kSmoothMax*` constants
(`:53-54`), and "almost certainly inlined" is not "provable".

### C-S5 (TONIGHT-OPTIONAL, LAST, own backup point) - dead Y-ring removal

**Transformation.** Remove the dead per-dataset time-plot Y-ring allocation. Two edits, both in
`configureLineSeries`, that **must land together**:

- **(1)** First loop (`:2561-2573`): after `if (!d->plt) continue;` add
  `if (useTimeXAxis(*d)) continue;`. `registerXAxisIfNeeded` is verified a no-op for time
  datasets (`xSource == -2` fails `m_datasets.contains`).
- **(2)** Time branch (`:2586-2589`): `series.y = &m_pltNullY;`. This MUST land with (1) or the
  QMap `operator[]` re-creates the entry.

Add `DSP::AxisData m_pltNullY;` to `Dashboard.h` near `m_pltXAxis` (`:424`); its default ctor is
capacity-100 size-0, allocated once.

**Files.** `Dashboard.h` (+`m_pltNullY`), `Dashboard.cpp` (`configureLineSeries` only).
**Risk.** Low but trap-laden (see below); take a backup point first. **Hotpath.** None
(`configureLineSeries` only). **Verify.** reader-set grep
(`plotData(|m_pltValues|m_yAxisData`) unchanged: `Plot.cpp:546` non-time only;
`DashboardHandler.cpp:483` guarded; the `clearPlotData` `m_yAxisData` loop is a no-op for absent
keys. All three `m_pltValues.append` branches still execute.

#### C-S5 trap notes - IN FULL

- **Alias to `m_pltNullY`, NOT `m_pltXAxis`.** Do not alias `series.y` to `m_pltXAxis`: it is
  `fillRange`-filled to size 1001, so `tailFrames` would emit the X ramp as Y garbage. A
  dedicated size-0 null ring is required.
- **Both edits are the same change-set.** Edit (1) skips the `registerXAxisIfNeeded`/QMap
  insert; edit (2) points the carrier at the shared null ring. If (2) lands without (1), the
  QMap `operator[]` re-creates the very entry being removed. Never split them.
- **The carrier keeps a valid pointer.** A stale-widget transient that reaches
  `downsampleMonotonic` with `n == 0` hits the early return and is safe; a null pointer would
  crash. The carrier (one LineSeries per plot widget) must always hold a valid Y pointer - the
  size-0 null ring provides that.
- **MultiPlot is NOT dead - curve-count reads.** `MultiPlot::updateRange`/`updateData` read
  `multiplotData(index).y.size()` as a *curve count* in all modes (`MultiPlot.cpp:643-653`,
  `:673-676`). This edit is `Plot`/`Dashboard` only; do not extend the null-ring idea to
  MultiPlot sample buffers without first adding a curve-count accessor.

---

### C-S6 (SPEC - compile-gated) - full policy adoption at 8 fork sites

**Transformation.** Adopt the cached policy at the remaining forks: `Plot` ctor caches
`m_xPolicy` (via `resolveXAxisPolicy`); `resolveXAxis`/`updateData`/`updateRange`/
`calculateAutoScaleRange` switch on it; `MultiPlot` ctor uses `groupXAxisMode`;
`configureLineSeries`'s second loop becomes a three-case switch; `m_monotonicData`/`m_timeAxis`
become derived, then die. The C-S4 `advancePlotClock` extraction lands here (member fn, sees
`kSmoothMax*`).

**Files.** `Plot.cpp`, `Plot.h` (`m_xPolicy`), `MultiPlot.cpp`, `Dashboard.cpp`. **Risk.**
Medium - this is the behavioral heart; needs compile + the full smoke matrix + benchmark
belt-and-braces. **Hotpath.** Configure only (push tables provably identical; benchmark
belt-and-braces). **Verify.** After: `grep xAxisId app/src/UI` -> only `resolveXAxisPolicy`
sites + `Dashboard` configure internals. Smoke matrix: time / samples / XY / unlicensed-degrade
/ sweep. Run `--benchmark-hotpath`.

### C-S7 (SPEC - compile-gated) - group-level xAxisId field

**Transformation + JSON schema evolution - all nine points IN FULL:**

1. **Group field.** Add `int xAxisId = kXAxisTime` to `Group` (`Frame.h` ~`:465`).
2. **Reuse `Keys::XAxis`.** Reuse the existing `"xAxis"` key (`Frame.h:94`); group and dataset
   scopes are disjoint, so no collision.
3. **Serialize sparse.** After the `sourceId` line (`Frame.h:1158-1199`):
   `if (g.xAxisId != kXAxisTime) obj.insert(Keys::XAxis, g.xAxisId);` - default stays implicit.
4. **Read + clamp.** On read, clamp: any value `!= kXAxisSamples` -> `kXAxisTime` (group combo
   is Time | Samples only).
5. **Migration.** Add `migrateGroupXAxisIds()` after `migrateLegacyXAxisIds()`
   (`ProjectModelLoading.cpp:158`): multiplot groups without a group key whose
   `front().xAxisId == kXAxisSamples` are promoted to Samples; accel/gyro groups are excluded
   (they have no selector and default to Time).
6. **Dual-write window.** `onGroupItemChanged` `kGroupView_xAxis` (`ProjectEditorCommit.cpp:268`)
   sets the group field AND keeps the per-dataset fan-out (old apps ignore the unknown group
   key, so both must be written during the window).
7. **Readers move to the field.** `buildGroupXAxisRow` reads `group.xAxisId`; `groupXAxisMode`
   becomes a one-line field read (the designed seam); `useTimeXAxisGroup` ->
   `group.xAxisId == kXAxisTime` (keep the `!empty` guard for one release).
8. **API.** `groupUpdate` accepts `xAxisId` (`-2`/`-1` only, warn otherwise) + a help string
   (~`ProjectHandler.cpp`); **no `SchemaVersion` bump needed**.
9. **Retire.** Drop the per-dataset fan-out in a later release, once the dual-write window has
   shipped.

**Files.** `Frame.h`, `ProjectModel.cpp`, `ProjectEditor.cpp`, `Dashboard.cpp` (helper
collapse). **Risk.** Medium - persistence + migration; the dual-write window is the safety net.
**Hotpath.** None. **Verify.** New build's project loads on an old build (fan-out present) and
vice versa (unknown key ignored); migration promotes the right groups and leaves accel/gyro on
Time; `groupXAxisMode` is now a field read.

### C-S8 (SPEC - compile-gated, honest medium risk) - shared range engine

**Transformation.** Reconcile the two autoscale padding engines. **Honest-risk note, IN FULL:**
`Plot::applyAxisPadding` pads +/-10% of range then floor/ceil (`Plot.cpp:834-860`);
`MultiPlot::applyDerivedYBounds` pads the midpoint by `halfRange * 1.1` + floor/ceil + a
degenerate re-split (`MultiPlot.cpp:793-828`). **The formulas differ**, so a naive dedup changes
on-screen autoscale for one of the two widgets. This is a **maintainer decision: parameterize
(behavior-preserving, safe) vs unify the visuals (a product change)** - the plan does not
pre-decide it. Plot's `dataBipolar`/`updateDataExtremes` (area-fill baseline) stays in Plot.
`computeMinMaxValues` is a member template, so hoisting it changes its instantiation context.
Any new source file needs a `CMakeLists.txt` entry (`Plot.cpp` at `:230`, `MultiPlot.cpp` at
`:242`). Ship compiled + a screenshot compare.

**Files.** `Plot.cpp`, `MultiPlot.cpp`, possibly a new shared source + `app/CMakeLists.txt`.
**Risk.** Medium (visual regression class). **Hotpath.** None (draw-rate). **Verify.** Screenshot
compare against the pre-change baseline (parameterize) or the agreed target (unify).

### C-S9 (SPEC - benchmark-gated) - push-struct unification

**Transformation.** Index-address push structs everywhere, and collapse the manual
`(sourceId, uniqueId)` ring key into a single `RingKey{sourceId, uniqueId}` struct
(`qHash`/`operator==`) or route the four call sites (`Dashboard.cpp:2388-2503`) through the
existing helper (P5; off-hotpath, low risk, bundled here).

**Benchmark gate rationale - LinePush QMap-node dangling analysis, IN FULL:**
`TimePush`/`MultiPush` already use the defensively-correct index discipline. `LinePush` caches
raw `AxisData*` pointers into QMap nodes; that survives **only** because the maps are rebuilt
before the pushes resolve AND are never detached - a COW detach via non-const access would
dangle the cached pointers. Converting `LinePush` to key-lookup adds a per-fire QMap `find` on
the HOT `updateLineSeries` loop, which lives in `datasets+publish` (~70-80% of per-frame time).
**That cost MUST be measured** - the alternative is to stabilize storage (`std::vector` + index)
rather than key-lookup. The pick is decided by `--benchmark-hotpath`, not on paper.

**Files.** `Dashboard.cpp` (push structs + ring key), `DashboardHandler.cpp` (ring-key routing).
**Risk.** High for the LinePush change (hotpath); low for the ring-key struct. **Hotpath.**
**YES** - hard benchmark gate. **Verify.** All seven `--benchmark-hotpath` tiers pass at the
default `--min-fps 256000`; C-S9 lands only if `datasets+publish` is not regressed.

## Tradeoffs & alternatives considered

Recorded as a list rather than a table so each rationale wraps under 100 columns.

- **Fork unification scope** - merge render arms vs one predicate decides. Chosen: one
  predicate decides; the three data sources genuinely differ, so merging arms is impossible.
- **Range engine (b)** - parameterize vs unify vs drop. Chosen: demoted to C-S8 as a maintainer
  product decision; the padding formulas differ, so "unify" is a visual change, not a refactor.
- **Group mode** - keep front-dataset encoding vs a real group field. Chosen: real
  `Group::xAxisId` (C-S7) with a dual-write window; retires the P3 wart without a schema bump.
- **LinePush storage** - key-lookup vs `std::vector` + index. Undecided by design;
  `--benchmark-hotpath` picks. Key-lookup adds a per-fire find on the hottest loop.
- **PlotClock hardening** - brace-scope now vs extract static now. Chosen: brace-scope now
  (C-S4, zero codegen); the extraction needs member access to `kSmoothMax*`, deferred to C-S6.

## Risks & mitigations

- **Silent-breakage class: cached flag not refreshed** - `m_xPolicy` (C-S6) must refresh on the
  same configure path that rebuilds carriers, or a mode change would not take. Mitigation: wire
  it into the existing reconfigure, not a new signal.
- **QMap `operator[]` re-insert (C-S5)** - pointing the carrier at the null ring without also
  skipping the insert re-creates the entry. Mitigation: land both edits together (trap note).
- **Aliasing to the filled X ring (C-S5)** - would emit the X ramp as Y garbage. Mitigation:
  dedicated size-0 `m_pltNullY`.
- **Behavior drift in the group swap (C-S3)** - the empty-group asymmetry must be preserved.
  Mitigation: the documented truth table; the two sites compare against different enum values.
- **Persistence incompat (C-S7)** - old/new build interop. Mitigation: sparse key + dual-write
  window; no schema bump.
- **Autoscale visual regression (C-S8)** - differing formulas. Mitigation: screenshot compare;
  maintainer chooses parameterize vs unify before implementation.
- **Hotpath regression (C-S9)** - LinePush key-lookup on the hottest loop. Mitigation: hard
  `--benchmark-hotpath` gate; storage-stabilization fallback.

## Test & verification plan

- **Unit (you can run):** none directly - this is UI-layer C++; no `tests/scripts/` JS cases
  apply.
- **Smoke matrix (maintainer runs in the app):** time / samples / XY / unlicensed-degrade /
  sweep, plus a multiplot group toggled Time<->Samples and an empty group. Required for C-S5 and
  C-S6; re-run for C-S7/C-S8.
- **Persistence (maintainer):** save on new build, load on old build and vice versa; confirm
  C-S7 migration promotes the right multiplot groups and leaves accel/gyro on Time.
- **Hotpath:** `--benchmark-hotpath` (all seven tiers, default `--min-fps 256000`) - required
  for C-S9, belt-and-braces for C-S6.
- **Static:** `python scripts/code-verify.py --check <files>` per stage;
  `python scripts/documentation-verify.py` for C-S1; `qt-cpp-review` on the C++ diff before
  handoff; `python scripts/sanitize-commit.py` before commit.
