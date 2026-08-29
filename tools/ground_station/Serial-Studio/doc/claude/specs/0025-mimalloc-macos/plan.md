---
spec: 0025-mimalloc-macos
phase: plan
status: approved
updated: 2026-07-20
---

# Plan 0025 — Enable mimalloc on macOS via static interpose

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this — a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Add a macOS branch to `cmake/MiMalloc.cmake` that links mimalloc **statically** into the
executable with `MI_OVERRIDE=ON`, forced in via `-Wl,-force_load` so its Mach-O
`__DATA,__interpose` entries survive `-dead_strip` and override `malloc`/`free`
process-wide (app + Qt + linked libs) with no injected dylib and no `DYLD_*` reliance — the
one macOS override path that survives SIP, hardened runtime, library validation, and
notarization because it lives inside the signed binary. It plugs in exactly where the
existing Windows/Linux paths do (`target_link_mimalloc()` called once on the executable in
`app/CMakeLists.txt`), keeps mimalloc out of LTO/PGO via the existing include-before-
`Optimization.cmake` ordering, and reuses the existing sanitizer skip. Because the shipped
default is benchmark-gated (spec R7) while the single user-facing switch stays
`SS_USE_MIMALLOC`, macOS participation is guarded by an **internal, non-headline** cache
variable that defaults OFF for bring-up/benchmarking and is flipped to ON in a one-line final
task only if `--benchmark-hotpath` confirms a win on both arches. Static-archive + force_load
is chosen over the object-file variant (fewer moving parts, single link flag) and over a
bundled shared dylib (which would require DYLD injection to reach Qt — a spec non-goal).

## Affected subsystems & files

| File | Change |
|------|--------|
| `cmake/MiMalloc.cmake` | Add the macOS static-override branch: extend the platform gate to include `APPLE` behind an internal `SS_MIMALLOC_ENABLE_APPLE` default (OFF at bring-up); on Apple build `mimalloc-static` with `MI_OVERRIDE=ON` (not shared); in `target_link_mimalloc()` add an Apple arm linking with `-Wl,-force_load,$<TARGET_FILE:mimalloc-static>` and **no** dylib copy/install POST_BUILD step. Update the module header comment's macOS line from "opts out" to describe the static-interpose path. |
| `CMakeLists.txt` | Likely **no change** — `SS_USE_MIMALLOC` already exists and stays the single user switch. Touch only if the internal `SS_MIMALLOC_ENABLE_APPLE` advanced var is surfaced here instead of inside the module (decision below). |
| `.claude/skills/cpp-compiler-flags/` (skill body) | Update the MiMalloc description: it currently says the module is "Windows/MSVC + Linux". Doc-only, AI-facing; keep in lane, flagged as a task not a silent edit. |
| `doc/claude/specs/0025-mimalloc-macos/plan.md` | This file. |

No C++/QML/`.h` source changes. No `app/` runtime code, no `Frame.h`, no API, no QML.

## Architecture & data flow

Pure build-time change — there is no runtime object graph, signal, or thread involved. The
"data flow" is the allocator override path at process start:

1. `target_link_mimalloc(${PROJECT_EXECUTABLE})` (already called at `app/CMakeLists.txt:1136`)
   force-loads the mimalloc static archive into the app executable on Apple.
2. mimalloc's `MI_OVERRIDE` object contributes `__DATA,__interpose` entries. dyld resolves
   these at load, redirecting `malloc`/`free`/`realloc`/… process-wide, **including calls made
   from the prebuilt Qt frameworks** — the same transparency Windows gets from
   `mimalloc-redirect.dll` and Linux from `-Wl,--no-as-needed`, achieved here with zero runtime
   injection.
3. Foreign pointers (allocated before mimalloc is live, or through a system malloc zone) are
   detected by mimalloc (`mi_is_in_heap_region`) on free and delegated to the system allocator
   — the safety net that makes a full-process override non-crashing (spec R2).

The universal binary is assembled downstream by CI: each per-arch slice
(`build-macos-arm64`, `build-macos-intel`) statically contains its own arch's mimalloc; the
`build-macos` job `lipo -create`s the two executables, then `codesign --options runtime`
signs the single fat binary. No extra artifact enters the bundle, so signing/notarization is
unaffected.

## Hotpath & threading impact

- **Touches the hotpath?** Not in source. mimalloc *serves* the frame-parse hotpath's small
  `QString`/`QByteArray`/Lua allocations, but this change adds **no** code on the
  `FrameReader` / `CircularBuffer` / `FrameBuilder` / `Dashboard` path — no new call, no
  allocation shape change, no signal. The SPSC / `Qt::DirectConnection` / no-alloc /
  slot-pool / cached-flag invariants are untouched. The impact is measured, not structural:
  `--benchmark-hotpath` on macOS must show **no regression** on either arch (spec AC3), and
  the shipped default flips ON only on a measured win (AC4).
- **New cross-thread signal/slot?** None.
- **New input to a cached hotpath flag?** None.
- **Timestamp ownership** — unchanged; the source still stamps at the driver boundary.

## Data model & persistence

None. No `Frame.h` `Keys::` additions, no schema/version bump, no `widgetSettings`, no project
JSON, no Sessions DB change, no migration.

## API / SDK surface

None. No API handler, no `EnumLabels` slug, no SDK regeneration, no script `apiCall` reach.
Not gated by `BUILD_COMMERCIAL` — this is a platform build option, not a Pro feature.

## QML / UI

None. Headless build-system change; no component, model, ComboBox guard, theme, or font work.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Static override mechanism | (A) static archive + `-Wl,-force_load`; (B) object file (`MI_BUILD_OBJECT` → `mimalloc.o` linked directly); (C) shared `libmimalloc.dylib` bundled in the `.app` | **A** — single link flag, forces the interpose object past `-dead_strip`, no bundle/sign/notarize surface. B is equally valid but adds an object-library target to thread through the exe link. **C is rejected**: shared override on macOS needs `DYLD_INSERT_LIBRARIES`/flat-namespace to interpose Qt — a spec non-goal and the exact fragility being avoided. |
| Shipped default on macOS | (i) internal `SS_MIMALLOC_ENABLE_APPLE` advanced var, default OFF, flipped ON post-benchmark; (ii) extend the existing gate to `APPLE` and default ON immediately with `SS_USE_MIMALLOC` | **(i)** — satisfies spec R7 (default decided by benchmark, not assumption) while keeping `SS_USE_MIMALLOC` the *only* end-user toggle (an internal `mark_as_advanced` var used for bring-up is not a competing user switch). The mechanism lands regardless; the default is a one-line change in the final task, gated on AC4. |
| Where the internal var lives | (a) inside `cmake/MiMalloc.cmake`; (b) as an `option()` in root `CMakeLists.txt` | **(a)** — keeps the whole macOS concern in one module and root `CMakeLists.txt` untouched; `mark_as_advanced()` keeps it out of the default cache UI. |
| CI macOS benchmark threshold | (a) leave the existing `--min-fps 1` smoke run; (b) promote macOS CI to the real 256k gate | **(a)** — changing CI benchmark thresholds is out of this spec's lane and macOS runners are variable; the real AC3 gate is maintainer-run locally. Noted, not silently changed. |

## Risks & mitigations

- **`-dead_strip` drops the interpose object** (override silently does nothing) → `-Wl,-force_load`
  forces the whole archive in; AC1 (runtime confirms mimalloc active, no `DYLD_*` set) catches
  a regression.
- **mimalloc swept into LTO/PGO** (spec R8) → preserved by the existing include ordering
  (`MiMalloc.cmake` before `Optimization.cmake`); the macOS target is created before the
  directory-level `-flto` flags apply, exactly like today's Win/Linux targets. Verify the
  Apple `mimalloc-static` target carries no `-flto`.
- **Foreign-pointer free crash** (spec R2) → mimalloc's `mi_is_in_heap_region` delegation;
  AC2 stress run exercises it. Note: mimalloc is skipped under ASan (existing sanitizer guard),
  so AC2's heap-corruption check runs as a normal (non-ASan) stress; a separate plain ASan run
  covers the non-mimalloc allocator path — the two do not combine by design.
- **Notarization / hardened runtime rejects the binary** → static link adds nothing injectable;
  the override is inside the already-signed executable. AC5 runs the real
  lipo→sign→`--options runtime`→notarize pipeline to confirm.
- **Universal-binary breakage** → each slice statically embeds its own arch's mimalloc; `lipo`
  merges executables, not libraries. AC5's `lipo -info` (arm64 + x86_64 present) + launch
  guards it.
- **mimalloc CMake target names differ from assumption** (`mimalloc-static` / `mimalloc-obj`
  vs `mimalloc`) → confirm the exact static target name against the fetched mimalloc v3.4.1
  `CMakeLists.txt` at implement time before wiring `$<TARGET_FILE:...>`.
- **pac-ret / Lua unwind interaction** → none; pac-ret is Linux-only here (Hardening.cmake),
  and mimalloc has no throw path. No change to the unwind-table invariants.

## Test & verification plan

Build-system feature — no `pytest` or `tests/scripts/` case applies; verification is
maintainer build/run/benchmark observation, mapped per acceptance criterion:

- **AC1** (override live, no `DYLD_*`) — maintainer: default macOS build, confirm mimalloc
  version symbol present / stats reachable at runtime with no env var set.
- **AC2** (no crash / corruption, foreign frees safe) — maintainer: hotpath stress run, clean;
  plus a separate plain-ASan run for the non-mimalloc path.
- **AC3** (hard gate: no regression) — maintainer: `--benchmark-hotpath` passes all gates on
  `arm64` **and** `x86_64`.
- **AC4** (record delta, set default) — maintainer: record the benchmark delta vs system heap
  per arch; the final task flips `SS_MIMALLOC_ENABLE_APPLE` default per the result.
- **AC5** (universal + notarized) — CI/maintainer: `build-macos` lipo→sign→notarize pipeline
  succeeds; `lipo -info` shows both slices; the notarized app launches.
- **AC6** (`SS_USE_MIMALLOC=OFF`) — maintainer: OFF build links no mimalloc; system allocator
  active.
- **AC7** (sanitizer skip) — maintainer: macOS sanitizer build emits "mimalloc disabled for
  sanitizer build" and links no override.
- **Static:** `scripts/code-verify.py` does not lint `.cmake`; `scripts/sanitize-commit.py`
  before commit. No `qt-cpp-review` (no C++ changed).
