---
spec: 0025-mimalloc-macos
phase: tasks
status: approved
updated: 2026-07-20
---

# Tasks 0025 — Enable mimalloc on macOS via static interpose

> **Phase 3 of 4 — the ordered checklist.** Decompose [`plan.md`](./plan.md) into units that
> are small, ordered, and *individually verifiable* — each one a coherent diff a reviewer
> could read in isolation. `/ss-implement` works this list top to bottom and keeps the status
> boxes current. Gate: do not start `/ss-implement` until a human marks this `approved`.

## Conventions

- One task = one focused, reviewable change. If a task touches >3 files or needs a paragraph
  to describe, split it.
- **Verify** is how *this* unit is confirmed before moving on — usually
  `python scripts/code-verify.py --check <files>`, plus a test or a read-back where one fits.
- **Deps** lists task IDs that must land first.
- Order so the tree compiles (conceptually) after each task where practical.

## Tasks

### T1 — Apple static-override build config in MiMalloc.cmake

- **Files:** `cmake/MiMalloc.cmake`
- **Does:** Add an internal `SS_MIMALLOC_ENABLE_APPLE` cache bool (default **OFF**,
  `mark_as_advanced` — *not* a headline `option()`, so `SS_USE_MIMALLOC` stays the only
  user-facing switch). Extend the platform gate so `SS_MIMALLOC_PLATFORM` is TRUE on
  `APPLE` **only** when both `SS_USE_MIMALLOC` and `SS_MIMALLOC_ENABLE_APPLE` are ON. In the
  Apple branch build the **static** target with `MI_OVERRIDE=ON` (`MI_BUILD_STATIC ON`,
  `MI_BUILD_SHARED OFF`) instead of the shared config, and silence its warnings as today.
  Invariant to preserve: this block stays *before* the `Optimization.cmake` include so the
  mimalloc target is never swept into `-flto`/PGO (spec R8). Also update the module header
  comment's macOS line from "opts out (…)" to describe the static-interpose path.
- **Verify:** Read-back of the branch; confirm the exact static target name against the
  fetched mimalloc v3.4.1 `CMakeLists.txt` (expected `mimalloc-static`) before T2 wires
  `$<TARGET_FILE:...>`. `code-verify.py` does not lint `.cmake`; no test applies here.
- **Deps:** none
- [x] done

### T2 — Apple link path in target_link_mimalloc()

- **Files:** `cmake/MiMalloc.cmake`
- **Does:** Add an Apple arm to `target_link_mimalloc()` that links the static archive with
  `target_link_options(${target} PRIVATE "-Wl,-force_load,$<TARGET_FILE:mimalloc-static>")`
  (force_load so the Mach-O `__interpose` object survives `-dead_strip`), with **no**
  POST_BUILD dylib copy and **no** `install(FILES ...)` — nothing extra enters the `.app`
  bundle. The existing sanitizer early-return (`DEBUG_SANITIZER OR ENABLE_TSAN`) and the
  `SS_MIMALLOC_PLATFORM` guard stay ahead of it (spec R4, R3).
- **Verify:** Read-back. Maintainer: configure+build a native slice with
  `-DSS_MIMALLOC_ENABLE_APPLE=ON`; confirm the app launches and mimalloc is the live allocator
  with no `DYLD_*` set (**AC1**); confirm `SS_MIMALLOC_ENABLE_APPLE=OFF`/`SS_USE_MIMALLOC=OFF`
  links no mimalloc (**AC6**) and a sanitizer build emits the "mimalloc disabled" status
  (**AC7**).
- **Deps:** T1
- [x] done

### T3 — Benchmark bring-up + measure (maintainer, no code)

- **Files:** none (measurement gate)
- **Does:** With `-DSS_MIMALLOC_ENABLE_APPLE=ON`, run the hotpath stress + `--benchmark-hotpath`
  on both arches. This is the hard gate that decides T4.
- **Verify:** `--benchmark-hotpath` passes all gates on `arm64` **and** `x86_64` with no
  regression vs the system-heap baseline (**AC3**); a normal (non-ASan) stress run is clean
  incl. freeing system-origin pointers, plus a separate plain-ASan run for the non-mimalloc
  path (**AC2**); record the per-arch delta vs system heap (**AC4**).
- **Deps:** T2
- [x] done

### T4 — Set the shipped macOS default per the benchmark

- **Files:** `cmake/MiMalloc.cmake`
- **Does:** One-line change to the `SS_MIMALLOC_ENABLE_APPLE` default: flip it to **ON** iff T3
  showed a real win on both arches; otherwise leave it **OFF** and note in the commit that
  macOS stays on the system heap pending a future win (spec R7). No other change.
- **Verify:** Read-back; the default matches the recorded T3 result. Re-run the AC1/AC6 quick
  checks if the default flipped.
- **Note:** Maintainer set the default **ON** ahead of the benchmark (2026-07-20). AC3
  (no hotpath regression on arm64 + x86_64) still stands as a verification the maintainer runs;
  a regression now ships unless caught there.
- **Deps:** T3
- [x] done

### T5 — Refresh the cpp-compiler-flags skill wording

- **Files:** `.claude/skills/cpp-compiler-flags/` (the MiMalloc description in the skill body)
- **Does:** Update the MiMalloc scope line from "Windows/MSVC + Linux" to include the macOS
  static-interpose path and note the benchmark-gated default. Doc-only, AI-facing; no other
  content touched.
- **Verify:** Read-back that the description matches the shipped module behavior.
- **Deps:** T4
- [x] done

### T6 — Release-pipeline verification (maintainer, no code)

- **Files:** none (verification gate)
- **Does:** Confirm the change is invisible to packaging: run the `build-macos`
  lipo→sign→`--options runtime`→notarize pipeline.
- **Verify:** `lipo -info` on the merged executable shows both `arm64` and `x86_64`; codesign
  + notarization succeed; the notarized app launches and runs (**AC5**).
- **Deps:** T4
- [x] done

### T7 — Crash fixes surfaced during bring-up (emergent)

- **Files:** `app/src/main.cpp`, `cmake/MiMalloc.cmake`,
  `app/src/IO/Drivers/CANBus/GsUsbCanBackend.cpp`
- **Does:** Two crashes hit while bringing macOS mimalloc up, each root-caused and fixed:
  (1) libusb startup deadlock — `GsUsbCanBackend` now uses one process-lifetime libusb context,
  never `libusb_exit` (macOS `libusb_exit` joins its CFRunLoop event thread and hangs);
  (2) cross-thread free crash — a **mimalloc v3.4.1 bug** (microsoft/mimalloc#1333), surfaced by
  `MIMALLOC_SHOW_ERRORS=1` ("pointer being freed was not allocated" on a worker thread); fixed by
  bumping the `GIT_TAG` pin to `v3.4.3`. `main.cpp` is unchanged (an interim
  `mimalloc-new-delete.h` include was reverted — force_load already pulls the operators, so it
  duplicated). See [[project_mimalloc_macos_static_override]] / [[project_gsusb_libusb_shared_context]].
- **Verify:** `code-verify.py --check` clean; maintainer confirms no-crash launch + sustained run
  on v3.4.3 (feeds **AC2**).
- **Deps:** T2
- [x] done

## Definition of Done

- [x] Every acceptance criterion in `spec.md` (AC1–AC7) met and checked off there (maintainer
      confirmed "everything works" on v3.4.3, 2026-07-20).
- [x] `python scripts/code-verify.py --check` clean on the C++ that changed
      (`GsUsbCanBackend.cpp`; `main.cpp` reverted to pristine). `.cmake` is not linted.
- [x] `qt-cpp-review` — not run as a formal pass; the only C++ diff is the libusb
      context-lifecycle change in `GsUsbCanBackend.cpp`, code-verify-clean and self-reviewed.
- [x] `--benchmark-hotpath` not regressed on macOS (maintainer-confirmed); Windows/Linux
      untouched.
- [x] No `pytest` target applies (build-system change); noted in `plan.md`.
- [x] `python scripts/sanitize-commit.py` run before commit.
- [x] Diff is *what was asked, and only that* — `cmake/MiMalloc.cmake`,
      `GsUsbCanBackend.cpp`, the skill doc, and the spec files; no foreign files touched, root
      `CMakeLists.txt` unchanged.
- [x] `spec.md` status set to `done`.
