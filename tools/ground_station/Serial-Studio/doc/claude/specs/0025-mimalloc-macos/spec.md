---
spec: 0025-mimalloc-macos
title: Enable mimalloc on macOS via static interpose
status: done
created: 2026-07-20
author: Alex Spataru
---

# Spec 0025 — Enable mimalloc on macOS via static interpose

> **Phase 1 of 4 — the WHAT and the WHY.** No implementation detail; no file paths, no
> class names, no signal wiring (that is `plan.md`). Gate: do not start `/ss-plan` until
> a human marks this `approved`.

## Problem / Motivation

The frame-parse hotpath allocates many small `QString` / `QByteArray` / Lua buffers per
frame. On Windows and Linux the build overrides the system allocator with mimalloc because
the MSVC CRT and glibc serve that pattern more slowly — glibc in particular thrashes its
per-thread arenas under the cross-thread alloc/free pattern where the main thread allocates
and exporter workers free. mimalloc is a measured hotpath win on those two platforms and is
enabled by default under `SS_USE_MIMALLOC`.

macOS is currently the only supported desktop platform left on the system heap. The exclusion
was made for a real reason — the dynamic-override path on macOS (`DYLD_INSERT_LIBRARIES`,
flat-namespace interposing) is fragile: System Integrity Protection strips `DYLD_*` from
signed binaries, hardened runtime with library validation blocks the injection, and a partial
override crashes. But that reasoning only rules out the *dynamic injection* mechanism.
mimalloc also supports a **static override** on macOS: the allocator is linked into the
executable and installs Mach-O `__interpose` entries that dyld resolves process-wide,
including into the prebuilt Qt libraries — with no environment variables, no injected dylib,
and nothing for SIP or notarization to reject. That path is not currently attempted, so macOS
users may be leaving a hotpath allocation win on the table that Windows and Linux already
capture. Whether the win is real on macOS's `magazine_malloc` is itself unknown and must be
measured, not assumed.

The deciding constraint: this feature ships enabled on macOS **only if** the repo's
`--benchmark-hotpath` gate confirms a real macOS speedup on both architectures; safety alone
is not sufficient justification to route allocations through a third-party allocator in a
signed, notarized binary.

## Goals

- macOS gains a first-class, statically-linked mimalloc override that requires no runtime
  environment variables and no injected dylib.
- The override is transparent and process-wide: allocations made by the app, Qt, and other
  linked libraries all route through mimalloc, and any foreign pointer (allocated before the
  override is live, or through a system malloc zone) is freed safely without a crash.
- The macOS path composes with every existing `SS_USE_MIMALLOC` behavior: the single option
  still turns the whole feature on/off, sanitizer builds still skip the override, and the
  offline/no-network build still degrades to the system heap.
- The decision to enable macOS mimalloc by default is settled by a measured
  `--benchmark-hotpath` result, not by assumption — the mechanism lands regardless, the
  default flips only on a proven win (on both Apple Silicon and Intel).
- A shipped universal, hardened-runtime, notarized macOS build launches and runs correctly
  with the override active.

## Non-Goals

- No dynamic override on macOS (`DYLD_INSERT_LIBRARIES`, `DYLD_FORCE_FLAT_NAMESPACE`, injected
  `libmimalloc.dylib`). That path is explicitly rejected as unsafe for signed/notarized
  builds and is not revisited here.
- No change to the Windows or Linux mimalloc paths, their build shapes, or their default
  enablement.
- No change to what `SS_USE_MIMALLOC=OFF` means on any platform.
- No new user-facing setting, UI, or runtime toggle — this is a build-time allocator choice
  only.
- No custom mimalloc configuration/tuning (arena sizes, purge policy, security flags) beyond
  what is needed to make the static override correct on macOS.
- No change to the mimalloc dependency's source or version beyond what macOS support requires.

## Requirements

1. **R1** — When building on macOS with `SS_USE_MIMALLOC=ON` (the default), the produced
   binary links mimalloc statically and overrides the process allocator; no `DYLD_*`
   environment variable and no separately-shipped mimalloc dylib is required for the override
   to take effect.
2. **R2** — With the override active, memory allocated by Qt and other linked libraries is
   served by mimalloc, and freeing a pointer that mimalloc did not allocate (a foreign/system
   pointer) does not crash — it is delegated to the system allocator.
3. **R3** — `SS_USE_MIMALLOC=OFF` on macOS produces a binary that uses the system allocator
   with no mimalloc linked, identical in allocator behavior to today.
4. **R4** — On macOS, a sanitizer build (ASan / UBSan / TSan) skips the mimalloc override, as
   it already does on the other platforms.
5. **R5** — The override works in a **universal** macOS binary: both the `arm64` and
   `x86_64` slices link and override correctly.
6. **R6** — A macOS build produced through the real release path — hardened runtime, code
   signing, and notarization — launches and runs with the override active and passes
   notarization.
7. **R7** — The default enablement of macOS mimalloc is decided by `--benchmark-hotpath`
   results: it defaults ON only if the benchmark shows a real speedup versus the system heap
   on both Apple Silicon and Intel; otherwise the mechanism remains in-tree but macOS defaults
   to the system heap. Either way, macOS must not *regress* the hotpath benchmark gate.
8. **R8** — The mimalloc dependency must not be swept into the build's LTO/PGO optimization,
   consistent with how it is kept separate on the other platforms.

## Acceptance Criteria

- [x] **AC1** — On macOS, a default build (`SS_USE_MIMALLOC=ON`) runs and reports mimalloc as
      the active allocator at runtime (mimalloc version symbol present / its stats reachable),
      confirming the override is live without any `DYLD_*` variable set. *(Maintainer
      observation on a running build.)*
- [x] **AC2** — Stress run of the frame-parse hotpath on macOS shows no allocator-related
      crash and no leak/heap-corruption report, including freeing system-origin pointers.
      *(Maintainer run; clean under a normal + a separate ASan session.)*
- [x] **AC3** — `--benchmark-hotpath` on macOS passes all gates (no regression versus the
      current system-heap baseline) on both `arm64` and `x86_64`. *(Maintainer-run benchmark;
      this is the hard gate.)*
- [x] **AC4** — The measured `--benchmark-hotpath` delta versus the system heap is recorded
      for Apple Silicon and Intel, and the macOS default (ON vs system-heap) is set to match
      that result per R7. *(Maintainer records numbers; default follows the data.)*
- [x] **AC5** — A universal (`arm64` + `x86_64`) build links and overrides in both slices, and
      the same binary passes hardened-runtime code signing and notarization and launches
      correctly. *(Maintainer runs the release/notarization pipeline.)*
- [x] **AC6** — `SS_USE_MIMALLOC=OFF` on macOS produces a binary with no mimalloc linked and
      the system allocator active. *(Maintainer observation / link inspection.)*
- [x] **AC7** — A macOS sanitizer build emits the existing "mimalloc disabled for sanitizer
      build" status and links no override. *(Build-log observation.)*

## Constraints & Invariants

- **Value is a gate, not an assumption.** macOS's `magazine_malloc` already handles the
  cross-thread pattern well; the feature ships enabled only on a measured `--benchmark-hotpath`
  win. No regression to the 256 kHz hotpath gate is permitted on any platform.
- **Static override only.** No injected dylib and no `DYLD_*` reliance — the override must
  survive SIP, hardened runtime, library validation, and notarization because it lives inside
  the signed binary.
- **Foreign-pointer safety is mandatory.** Any pointer not allocated by mimalloc must free
  correctly; a partial override that crashes is unacceptable.
- **One option governs all platforms.** `SS_USE_MIMALLOC` remains the single on/off switch;
  the macOS path must not introduce a second competing toggle for end users.
- **Sanitizer and offline builds keep their current behavior.** ASan/UBSan/TSan skip the
  override; a no-network build still degrades to the system heap.
- **Keep mimalloc out of LTO/PGO**, consistent with the existing build ordering.
- **Windows/Linux untouched.** Their build shapes, defaults, and override mechanisms do not
  change.

## Open Questions

- None blocking. The two maintainer decisions are settled: (1) macOS defaults to a
  **benchmark-gated** enablement — the mechanism lands, the default flips ON only on a proven
  win; (2) acceptance requires the **universal + notarized release** shape, not just the local
  build arch.
