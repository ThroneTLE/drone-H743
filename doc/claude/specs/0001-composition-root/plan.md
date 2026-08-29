---
spec: 0001-composition-root
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-06
---

# Plan 0001 - Composition Root & De-Singleton-ization

> **Phase 2 of 4 - the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this - a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Formalize `Misc::ModuleManager` as the composition root and convert loose `X::instance()`
call sites to captured dependencies, in seven gated stages. S1 (this spec package) and S2
(the regression-guard linter rule) land tonight and are marked done below. S3 pins the
core-module construction order with a new `instantiateCoreModules()` - its load-bearing
effect is constructing ProjectModel before AppState, which kills the settings-conditional
re-entrancy edge. S4 captures true-leaf and near-leaf dependencies as constructor init-list
reference members, wave by wave. S5 captures the five core modules (the pentagon) as
deferred pointers assigned at the top of their own `setupExternalConnections()`, one class
per morning build. S6 converts the hotpath statics last, benchmark-gated. S7 ratchets the
sanctioned surface down. The end state is structurally identical to constructor injection
minus the wiring explosion, with the hotpath strictly no worse (a member load beats a Meyers
guard).

## Affected subsystems & files

| File | Change |
|------|--------|
| `scripts/code_verify_rules.py` | S2 (done): new `arch-singleton-instance` advisory rule in `analyze()` |
| `scripts/code-verify.py` | S2 (done): add the new kind to `_ADVISORY_KINDS` (~line 1889) |
| `doc/claude/specs/0001-composition-root/*` | S1 (done): this spec/plan/tasks package |
| `doc/claude/architecture.md` | S1: new subsection "Composition Root & Construction Order -- ModuleManager" |
| `app/src/Misc/ModuleManager.cpp` | S3: add `instantiateCoreModules()`, call first in `setupCrossModuleConnections()` |
| `app/src/Misc/ModuleManager.h` | S3: declare private `void instantiateCoreModules();` |
| `app/src/AppState.{h,cpp}` | S5: `ProjectModel* m_projectModel` deferred capture; first pentagon target |
| `app/src/DataModel/ProjectModel.{h,cpp}` | S5: deferred pointers; ctor/newJsonFile surface keeps direct calls |
| `app/src/DataModel/FrameBuilder.{h,cpp}` | S5: deferred pointers; hotpath members deferred to S6 |
| `app/src/IO/ConnectionManager.{h,cpp}` | S5 + S6: deferred pointers; hotpath statics + per-frame AppState poll |
| `app/src/UI/Dashboard.{h,cpp}` | S5 + S6: deferred pointers; `updateStreamAvailable` static |
| `app/src/Misc/*`, `app/src/UI/Widgets/*` | S4 wave 1: capture CommonFonts/TimerEvents/ThemeManager |
| `app/src/Console/*`, `app/src/CSV/*`, `app/src/MDF4/*` | S4 wave 2 |
| `app/src/DataModel/*` (editors) | S4 wave 3 |

## Architecture & data flow

Two-phase startup, all through the composition root (see `doc/claude/architecture.md`,
"AppState" ordering section, and the new "Composition Root & Construction Order" subsection
added in S1):

1. **Construct.** `instantiateCoreModules()` runs first inside
   `ModuleManager::setupCrossModuleConnections()` and forces each core module into existence
   in the pinned topological order (below). Each ctor only self-initializes, reads
   `QSettings`, and connects to objects it itself forces.
2. **Wire.** The 17 ordered `setupExternalConnections()` calls run, cross-wiring modules.
   The pentagon captures its deferred pointers at the top of each of these.
3. **Restore.** `restoreLastProject()` runs after all wiring (INV-1).
4. **Expose.** ~60 `Cpp_*` context properties are registered after wiring, before QML load
   (INV-2). The Qt message handler is installed only after `Console::Handler` and
   `NotificationCenter` exist (INV-3).

Dependency access shifts from a Meyers-guarded `X::instance()` at each call site to a
captured member: a ctor init-list reference (leaves) or a pointer assigned during wiring
(pentagon). No signal/slot connection types change; the hotpath stays `DirectConnection`.

## Hotpath & threading impact

- **Touches the hotpath?** Yes, but only in **S6** (benchmark-gated, last). S3/S4/S5 stay off
  the per-frame path - pentagon ctor bodies and any method reachable before wiring keep
  direct `instance()`. S6 converts the `static auto&` cache statics in
  `ConnectionManager::{onFrameReady,onRawDataReceived,processPayload,processMultiSourcePayload}`
  and `Dashboard::updateStreamAvailable` into S5 pointer members (a captured member load is
  cheaper than the Meyers atomic-load-plus-branch it replaces), and replaces the per-frame
  `AppState::instance().operationMode()` poll in `onFrameReady` with a cached
  `m_operationMode` refreshed on `operationModeChanged` (the existing FrameBuilder pattern).
  The `ss-hotpath` skill applies; read `ConnectionManager` and `Dashboard` draw paths in full
  before S6.
- **New cross-thread signal/slot?** No. S6 adds one cache-refresh connection for
  `m_operationMode` (`operationModeChanged` -> refresh slot), same-thread, matching the
  FrameBuilder cached-flag pattern.
- **New input to a cached hotpath flag?** Yes at S6: `m_operationMode` becomes a cached flag.
  Its change signal (`AppState::operationModeChanged`) must wire into the refresh slot or the
  operation-mode branch in `onFrameReady` goes stale (silent breakage; see
  `common-mistakes.md`).
- **Timestamp ownership** - unchanged. The source still stamps at the driver boundary;
  nothing here re-stamps downstream.

## Data model & persistence

None. No `Frame.h` `Keys::` additions, no schema/writer bumps, no `widgetSettings` or
project-JSON changes, no Sessions DB changes. This is a startup-ordering and
dependency-acquisition refactor; on-disk formats are untouched.

## API / SDK surface

None. No API handlers, `EnumLabels` slugs, or SDK changes. `API::Server` and `CommandHandler`
are captured like any other module but their external surface is unchanged.

## QML / UI

None functional. The ~60 `Cpp_*` context properties keep the same names and addresses (INV-2
guarantees stable, fully-wired `QObject` addresses at registration time). No new components,
no ComboBox restore-race surface.

## Implementation stages

S1 and S2 are done tonight (autonomous overnight run, 2026-07-06). S3 is gated on the
orchestrator's constructor re-verification decision. S4-S7 are morning work under the build
gate below.

### Build gate (all implementation stages)

- **One stage per build.** Land exactly one stage (or one wave / one pentagon class) per
  build-and-launch cycle; never batch two ordering-sensitive changes into one build.
- **Launch all three operation modes** after any stage that changes construction order
  (S3) or module wiring (S5): QuickPlot, ProjectFile, Console-only.
- **`--benchmark-hotpath` after S6** - all seven tiers, compared against the pre-S6 baseline.
  S3/S4/S5 do not require the benchmark (off the per-frame path) but a launch smoke test is
  still required.

### S1 - Dependency census + init-order contract (docs) - DONE (tonight)

- **Transformation:** Write this spec/plan/tasks package and the architecture.md subsection
  "Composition Root & Construction Order -- ModuleManager": the verified ctor-edge graph
  (spec appendix table), the pinned instantiation order for S3, and the three standing
  invariants INV-1..INV-3. Include the per-file `::instance()` inventory (1,852 sites, 59
  singleton headers) as an appendix so the morning waves are pre-scoped.
- **Files:** `doc/claude/specs/0001-composition-root/{spec,plan,tasks}.md`,
  `doc/claude/architecture.md`.
- **Risk:** Low (docs only). Risk is a mis-stated ctor edge propagating into S3/S5.
- **Verification:** `python3 scripts/documentation-verify.py`; cross-check every claimed ctor
  edge against the source before relying on it in a later stage.

### S2 - Regression guard: `arch-singleton-instance` advisory rule (Python) - DONE (tonight)

- **Transformation:** New advisory rule in `scripts/code_verify_rules.py` `analyze()`; add
  its kind to `_ADVISORY_KINDS` in `scripts/code-verify.py`. Trigger: regex
  `\b([A-Za-z_][\w:]*)::instance\(\)` on `.cpp`/`.h` lines, tree-sitter-scoped like the
  existing hotpath rules. Sanctioned (no finding): (1) files `Misc/ModuleManager.cpp`,
  `main.cpp`; (2) the line inside a class's own `X& X::instance()` definition; (3) call sites
  inside a function named `setupExternalConnections`; (4) the
  `static (const )?auto[&*] name = (&)?X::instance();` cache idiom (interim, removed at S7);
  (5) `// code-verify off/on` fences. Message points at
  `doc/claude/specs/0001-composition-root/`.
- **Files:** `scripts/code_verify_rules.py`, `scripts/code-verify.py`.
- **Risk:** Low; advisory only, never blocks CI.
- **Verification:** Run `python3 scripts/code-verify.py app/src` before and after; the
  blocking-error count is identical and the advisory report gains only the new kind. Drop a
  synthetic `Foo::instance()` snippet in the scratchpad and confirm the rule fires; confirm
  each sanctioned pattern does not.

### S3 - Pin the construction order (formalize the composition root)

- **Transformation:** Add private `void instantiateCoreModules();` to
  `Misc::ModuleManager`; call it as the **first line** of `setupCrossModuleConnections()`
  (`ModuleManager.cpp:604`). Body: one `(void)X::instance();` per module, in the verified
  topological order below. Scope rule: list **only** modules `setupCrossModuleConnections`
  already constructs transitively today; modules first built at context-property time
  (`ProjectEditor`, `ProtoImporter`, `Examples`, `HelpCenter`, ...) stay there.
- **Pinned instantiation order** (exactly as designed; `ProjectModel` before `AppState` is
  load-bearing):
  1. `Translator`
  2. `TimerEvents`
  3. `CommonFonts`
  4. `WorkspaceManager`
  5. `NotificationCenter`
  6. `ThemeManager`
  7. `ExtensionManager`
  8. `ControlScript`
  9. `ProjectModel`   (before `AppState` -- kills the settings-conditional edge)
  10. `AppState`
  11. `LemonSqueezy` / `MachineID`   [commercial]
  12. `FrameBuilder`
  13. `IO::ConnectionManager`
  14. `Console::Handler`
  15. `API::Server`
  16. `CSV::Player`
  17. `MDF4::Player`
  18. `[Sessions::Player]`   [commercial / optional]
  19. the four exports
  20. `FrameParser`
  21. `UI::Dashboard`   (last -- its ctor touches five core modules + two players +
      `TimerEvents`)
- **Files:** `app/src/Misc/ModuleManager.{h,cpp}`.
- **Risk:** Medium. Behavior-preserving because every listed ctor only self-initializes,
  reads `QSettings`, and connects to objects it itself forces; the riskiest reorder
  (ProjectModel before AppState) is the order production ProjectFile machines already run
  every day. Gated on the orchestrator's ctor re-verification decision.
- **Verification (grep symmetry):** every class named in `instantiateCoreModules` also
  appears later in `setupCrossModuleConnections` or was already transitively constructed;
  the order matches the spec table exactly; `restoreLastProject()` is still after all
  `setupExternalConnections()` (INV-1). Build gate: build + launch in all three operation
  modes + `--benchmark-hotpath`.

### S4 - Leaf ctor-reference capture (wide, wave-based)

- **Transformation:** Per consumer class, add a reference member (e.g.
  `Misc::TimerEvents& m_timerEvents;`), init it in the ctor init list
  (`m_timerEvents(Misc::TimerEvents::instance())`), and replace non-hotpath in-file calls
  with the member. Leaves verified to have zero singleton out-edges: `Translator`,
  `TimerEvents`, `CommonFonts`, `NotificationCenter`, `WorkspaceManager`, `CSV::Player`,
  `MDF4::Player`, `CommandHandler` (plus `ThemeManager` as a near-leaf). `-Wreorder` plus
  zero-warnings catches init-order slips at build.
- **Waves:** (1) `Misc/*` + `UI/Widgets/*` consumers of CommonFonts/TimerEvents/ThemeManager;
  (2) `Console/`, `CSV/`, `MDF4/`; (3) `DataModel/` editors.
- **Files:** as listed per wave in the table above.
- **Risk:** Low-medium; init-order mistakes are compile-time (`-Wreorder`), not runtime.
- **Verification (per file):** (a) `grep -c "X::instance()"` in the `.cpp` is 0 outside the
  ctor init list; (b) the header gained exactly one member per dependency; (c) the init-list
  entry position matches member declaration order; (d) the advisory `::instance()` count
  strictly decreases. Build gate: one wave per build + launch smoke test.

### S5 - Pentagon deferred pointer capture

- **Transformation:** `AppState`, `ProjectModel`, `FrameBuilder`, `ConnectionManager`,
  `Dashboard` each gain an `X* m_x` (nullptr in the ctor init list), assigned at the top of
  their own `setupExternalConnections()`, with `Q_ASSERT(m_x)` at use sites. Method-body call
  sites convert to `m_x->`. **Ctor bodies and any method reachable before wiring keep direct
  `instance()`** - the pre-wiring surfaces: AppState `deriveFrameConfig()`; ProjectModel
  `newJsonFile()` / `setModified()` / `watchProjectFile()` and anything they reach; Dashboard
  everything its ctor connects; ConnectionManager the `m_uiDriverSaveTimer` lambda.
- **Order (one per morning build):** `AppState` -> `ConnectionManager` -> `FrameBuilder` ->
  `ProjectModel` -> `Dashboard`. AppState may ctor-capture `ProjectModel` post-S3 (safe
  because S3 constructs ProjectModel first) but nothing else; **never ctor-capture AppState**
  from ProjectModel (the live A<->B hazard).
- **Files:** `app/src/AppState.{h,cpp}`, `app/src/DataModel/ProjectModel.{h,cpp}`,
  `app/src/DataModel/FrameBuilder.{h,cpp}`, `app/src/IO/ConnectionManager.{h,cpp}`,
  `app/src/UI/Dashboard.{h,cpp}`.
- **Risk:** Medium. The trap is converting a pre-wiring call site (fires before
  `setupExternalConnections` assigns `m_x`) - the `Q_ASSERT(m_x)` catches it in debug; the
  pre-wiring surface list above is the guard.
- **Verification (per class):** the header gained exactly the pointer members; every
  converted call site is in a method proven to run after wiring; the pre-wiring surface still
  uses direct `instance()`; advisory count decreases. Build gate: one pentagon class per
  build + launch in all three modes.

### S6 - Hotpath capture (benchmark-gated, last)

- **Transformation:** Convert the `static auto&` cache statics in
  `ConnectionManager::{onFrameReady,onRawDataReceived,processPayload,processMultiSourcePayload}`
  and `Dashboard::updateStreamAvailable` to the S5 pointer members. Replace the per-frame
  `AppState::instance().operationMode()` poll in `onFrameReady` with a cached
  `m_operationMode` refreshed on `AppState::operationModeChanged` (FrameBuilder pattern). All
  connections stay `Qt::DirectConnection`.
- **Files:** `app/src/IO/ConnectionManager.{h,cpp}`, `app/src/UI/Dashboard.{h,cpp}`.
- **Risk:** Medium-high (per-frame path). Silent breakage if `m_operationMode` is not wired
  to `operationModeChanged`. Read the hotpath files in full; invoke `ss-hotpath`.
- **Verification:** grep that the converted call sites are members, not statics;
  `m_operationMode` refresh is wired to `operationModeChanged`. Build gate:
  **`--benchmark-hotpath`, all seven tiers**, no regression vs the pre-S6 baseline.

### S7 - Ratchet

- **Transformation:** Shrink the sanctioned surface of `arch-singleton-instance` to
  `{ModuleManager.cpp, main.cpp, the class's own instance() definition,
  setupExternalConnections bodies}` - dropping the interim `static auto&` cache-idiom
  exemption now that S6 removed it. Optionally promote the pentagon to blocking per-class.
  New services take ctor params from here on.
- **Files:** `scripts/code_verify_rules.py`, `scripts/code-verify.py`.
- **Risk:** Low; linter-only. Promotion to blocking is a policy call, deferred here.
- **Verification:** the four remaining sanctioned patterns still produce no finding; every
  interim `static auto&` site removed in S6 now either reports or is gone; blocking-error
  count unchanged unless a per-class blocking promotion is intentionally enabled.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Overall shape | Container DI / service locator / composition root + capture | Composition root + capture -- ModuleManager already IS the root; two-phase construct/wire is DI's split without the ~60-param explosion or 1,852-site flag-day; hotpath strictly improves |
| ProjectModel vs AppState order | keep settings-conditional / pin PM-first / pin AppState-first | Pin ProjectModel first -- it is the order production ProjectFile machines already run daily, and it kills the re-entrancy edge |
| Leaf capture mechanism | ctor init-list reference / deferred pointer / lazy accessor | Ctor init-list reference for true leaves -- `-Wreorder` makes init-order slips compile-time errors; deferred pointer only where a class is reachable before wiring |
| Hotpath timing | convert with S5 / defer to a gated last stage | Defer to S6, benchmark-gated -- isolates the only per-frame-visible change behind `--benchmark-hotpath` |
| Guard severity | blocking now / advisory then ratchet | Advisory then ratchet -- 1,852 existing sites make a blocking flag-day untenable; advisory lets the surface shrink wave by wave |

## Risks & mitigations

- **Re-entrancy during construction (the core hazard).** Mitigation: S3 pins ProjectModel
  before AppState so the settings-conditional edge cannot form; the `ProjectModel.cpp:446`
  fenced surface and all pre-wiring surfaces keep direct `instance()` (never captured).
- **Init-order slip in a leaf reference member.** Mitigation: `-Wreorder` + zero-warnings
  makes it a build failure (S4 verification (c)).
- **Pre-wiring use of an unassigned pentagon pointer.** Mitigation: `Q_ASSERT(m_x)` at use
  sites; the explicit pre-wiring surface list in S5; one class per build.
- **Silent hotpath breakage from a stale cached flag** (`common-mistakes.md`). Mitigation:
  S6 wires `m_operationMode` to `operationModeChanged`; `--benchmark-hotpath` gate.
- **Behavior change from force-instantiation.** Mitigation: scope rule -- only pin modules
  the root already constructs transitively; grep symmetry check in S3.
- **Scope creep across the 1,852 sites.** Mitigation: strict wave boundaries; advisory count
  must strictly decrease per wave and never increase.

## Test & verification plan

- **Unit (agent can run):** none directly (no `tests/scripts/` JS surface). The S2 linter
  rule is self-tested with a synthetic `Foo::instance()` snippet in the scratchpad and a
  before/after `code-verify.py` blocking-count diff.
- **Static (agent runs):** `python3 scripts/documentation-verify.py` on the S1 docs;
  `python3 scripts/code-verify.py --check` on every changed C++ file per wave; the grep
  symmetry recipes per stage; `qt-cpp-review` on each C++ diff before handoff.
- **Integration (maintainer runs):** launch in QuickPlot, ProjectFile, and Console-only after
  S3 and after each S5 pentagon class; confirm normal startup, project restore, and console
  output. No new `pytest` files; existing `pytest tests/integration/` may be run against the
  live app as a smoke check.
- **Hotpath (maintainer runs):** `--benchmark-hotpath`, all seven tiers, after S6; compare to
  the pre-S6 baseline. `deploy.yml` gates the shipped PGO binary as the final backstop.
- **Commit:** `python3 scripts/sanitize-commit.py` before each commit; working tree clean of
  lint debt.
