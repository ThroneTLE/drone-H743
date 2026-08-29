---
spec: 0004-native-csd-shadow
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-07
---

# Plan 0004 — Native OS/WM window shadows for CSD windows

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

Delete the shadow layer from `CSD::Window` entirely — the 9-slice atlas, the
`ShadowImageProvider`, the transparent-surface `BorderImage` frame, and every
`shadowMargin()` offset — so a CSD window's geometry *is* its visible content: titlebar at
(0,0), content below it, 1px border and 8px resize band hugging the true window edge, and
minimum size = content + titlebar. On Windows 10 only, re-enable the OS shadow that DWM
already knows how to draw for frameless windows: after the decorator applies
`Qt::FramelessWindowHint`, restore `WS_THICKFRAME` (+`WS_CAPTION`) on the HWND and answer
`WM_NCCALCSIZE` with the full window rect through a `QAbstractNativeEventFilter`, which
yields the native shadow, aero-snap, and minimize animations with zero drawing on our side.
Linux (X11 and Wayland) keeps the existing thin border and simply has no shadow — no WM
detection or per-desktop protocol work in this pass. The `csdShadowEnabled`
property/setting and the Settings toggle are removed; `frameMargin()` becomes the constant
0 that macOS already returns and its QML call sites are cleaned up.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/Platform/CSD.h` | Remove `ShadowRadius`, the `shadow` ctor param, `m_frame`, `m_shadowEnabled`, `setupFrame()`, `updateFrameGeometry()`, `shadowMargin()`. |
| `app/src/Platform/CSD.cpp` | Delete shadow atlas generators, `ShadowImageProvider`, `setupFrame`, alpha-surface/transparent-color setup; anchor titlebar/border/content/`edgeAt()`/`updateMinimumSize()` at the window rect (drop every `shadowMargin()` term); drop the maximize-state frame handling that existed for the shadow. |
| `app/src/Platform/NativeWindow.h` | Remove `csdShadowEnabled` property, getter/setter, signal, `m_csdShadowEnabled`; remove `frameMargin()`. |
| `app/src/Platform/NativeWindow_CSD.cpp` | Remove shadow pref plumbing and `frameMargin()`; `frameTopInset()` returns titlebar height only; add the Windows-10-only DWM shadow enabler (style restore + `WM_NCCALCSIZE` native event filter, `#if defined(Q_OS_WIN)`), applied per decorated window from `addWindow()`. |
| `app/src/Platform/NativeWindow_macOS.mm` | Remove the `csdShadowEnabled`/`frameMargin` definitions to match the header (both are dead/0 there today). |
| `app/qml/Dialogs/Settings.qml` | Remove the "Window Shadow" label + switch rows (lines ~242-260). |
| `app/qml/Widgets/SmartDialog.qml` | Drop the `frameMargin` property and its width/height terms; keep `titlebarHeight` + `frameTopInset` (their new values make the math exact). |
| `doc/claude/directory-map.md` | One-line update for the Platform/ entries (shadow removed). |
| `doc/claude/specs/0004-native-csd-shadow/*` | Spec/plan/tasks bookkeeping. |

Grep-confirmed: `frameMargin` has exactly one QML consumer (`SmartDialog.qml`);
`csdShadowEnabled` has exactly one QML consumer (`Settings.qml`); `CSD::ShadowRadius` is
referenced only by `CSD.cpp` and `NativeWindow_CSD.cpp`. All other `Cpp_NativeWindow` QML
call sites use only `addWindow`/`removeWindow`/`titlebarHeight`, which keep their
signatures and semantics.

## Architecture & data flow

Unchanged in shape: `NativeWindow` (QML-exposed singleton) decides per platform whether to
decorate; `CSD::Window` decorates one `QQuickWindow` by reparenting its content into a
container item and filtering window events for resize/cursor handling. What changes is
coordinates and layering only:

- `CSD::Window` layout goes from `(margin, margin)`-anchored to `(0, 0)`-anchored: titlebar
  spans the top `titleBarHeight()` px, `m_contentContainer` fills the rest, `m_border`
  covers the full window rect, `edgeAt()` uses `ResizeMargin` (8px) from the true edges.
  `updateMinimumSize()`/`onMinimumSizeChanged()` drop the `2 * margin` terms.
- The window surface stays opaque (no alpha buffer request, no `Qt::transparent` fill) —
  the decorator no longer needs `setupFrame()` at all.
- Windows 10 only: `addWindow()` (or the decorator ctor) calls a small
  `enableNativeShadow(QWindow*)` helper that ORs `WS_THICKFRAME | WS_CAPTION` back into
  `GWL_STYLE` and triggers `SWP_FRAMECHANGED`; a process-wide
  `QAbstractNativeEventFilter` (installed once, tracking decorated HWNDs) handles
  `WM_NCCALCSIZE` (return the full window rect so no native frame/caption is drawn; when
  maximized, inset by the system frame metrics so content does not overhang the monitor)
  and leaves everything else to Qt. Move/resize keep going through the existing
  `startSystemMove()`/`startSystemResize()` calls.
- Existing signals (`windowStateChanged`, `minimumWidth/HeightChanged`, `themeChanged`,
  `activeChanged`) keep their wiring; the handlers just lose their margin math.

## Hotpath & threading impact

- **Touches the hotpath?** No. All changes live in window chrome (`app/src/Platform/`) and
  QML dialogs; no `FrameReader`/`CircularBuffer`/`FrameBuilder`/`Dashboard` code is
  touched. Removing the per-window translucent surface strictly reduces GPU compositing
  work.
- **New cross-thread signal/slot?** No. The native event filter runs on the GUI thread
  (Windows message dispatch), same as the existing event filter.
- **New input to a cached hotpath flag?** No.
- **Timestamp ownership** — unaffected.

## Data model & persistence

- `Window/CSDShadowEnabled` in QSettings becomes a stale key: never read, never written,
  never deleted (harmless residue, satisfies R8/AC7). `Window/CSDEnabled` is untouched.
- `SmartWindow`-persisted geometries (x/y/w/h aliases per category) were saved with
  shadow-inflated sizes (+48px each axis, +48+32 vertical). After the change, resizable
  windows restore once ~48px larger than before — clamped by SmartWindow's existing
  screen-bounds logic — and fixed-size dialogs recompute from content bindings, so no
  migration code is added. This is a one-time visual nudge, not drift: the next save
  stores true sizes. (Spec constraint "restore to something sane" — verified in AC1/AC2.)
- No project-JSON, `Keys::`, or Sessions DB impact.

## API / SDK surface

None. `NativeWindow` is a QML-context singleton, not part of the external API/SDK; no
`API/Handlers` or `EnumLabels` change. The removed members (`frameMargin`,
`csdShadowEnabled`) have no consumers outside the two QML files listed above.

## QML / UI

- `Settings.qml`: the "Window Shadow" row disappears; the "Custom Window Decorations"
  switch and the restart note stay.
- `SmartDialog.qml`: size formulas lose the `frameMargin` terms; `frameTopInset` now
  contributes exactly the CSD titlebar height (0 on macOS/Win11, matching today's
  intent). No other dialog/window QML changes — `titlebarHeight()` semantics are
  unchanged everywhere.
- Visual result: Windows 10 gets the DWM shadow; Linux gets the crisp 1px border at the
  true window edge; both get popups/resize/edge-snap on real geometry.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Windows 10 shadow mechanism | (a) `WS_THICKFRAME` + `WM_NCCALCSIZE` filter; (b) no shadow on Win10 (border like Linux); (c) keep painted shadow on Win10 | **(a)** — native DWM shadow, aero-snap and minimize animation for one small, well-trodden filter (Chromium/QGoodWindow pattern). (b) is the documented fallback if (a) shows platform artifacts we can't tame; (c) contradicts the spec. |
| KWin X11 `_KDE_NET_WM_SHADOW` | Implement now vs defer | **Defer** — it needs client-supplied shadow textures, which re-imports half the code the spec deletes. AC5 explicitly allows the thin-border fallback on Linux. Uniform Linux behavior also removes WM detection entirely. |
| `frameMargin()` API | Delete vs keep returning 0 | **Delete** — one QML consumer, and a dead parameter invites future misuse. `frameTopInset()` stays because dialogs genuinely need the CSD titlebar height. |
| Old persisted window sizes (+48px) | Accept one-time oversize vs migrate/version the settings category | **Accept** — SmartWindow already clamps to screen; a migration would guess whether a stored size was shadow-inflated and can guess wrong. Self-heals on first save. |
| Where the Win32 code lives | New `NativeWindow_Win.cpp` TU vs `#ifdef` block in `NativeWindow_CSD.cpp` | **`#ifdef` block in `NativeWindow_CSD.cpp`** — it already carries `Q_OS_WIN` DWM code (caption color) and the CMake platform wiring; a new TU for ~80 lines adds build-system churn. |

## Risks & mitigations

- **`WM_NCCALCSIZE` interplay with Qt's frameless handling** (Qt also adjusts geometry for
  frameless windows on Windows): the filter only activates for HWNDs we explicitly
  registered, and option (b) — skip the style restore, ship the Linux-style border — is a
  one-line retreat per window if Qt 6.11's windows integration fights the filter.
  Maintainer smoke on Win10 (AC5) is the gate.
- **Maximized client-rect overhang** with `WS_THICKFRAME`: handled in the filter by
  insetting the client rect by `GetSystemMetrics(SM_CXSIZEFRAME/SM_CYSIZEFRAME) +
  SM_CXPADDEDBORDER` when the window is maximized; verified by AC2/AC8-style maximize
  checks on Win10.
- **Fullscreen mode** (`showFullScreen`, dashboards): the filter must treat fullscreen like
  maximized-without-inset (client = monitor rect); covered by the existing
  `windowStateChanged` handler keeping border/titlebar hidden in fullscreen.
- **Transparent→opaque surface change**: some drivers behave differently with an alpha
  visual; going opaque is the safer direction (fewer compositing paths), but Linux smoke
  (AC6) confirms no regression on X11 and Wayland.
- **Reparenting/z-order assumptions**: `m_border` z stays above content, titlebar z
  unchanged; no child-item contract changes, so `ExternalWidgetWindow`, `ProjectEditor`,
  `DatabaseExplorer` and all `SmartDialog`s inherit the fix without edits (grep-confirmed
  they only use `addWindow`/`removeWindow`/`titlebarHeight`).
- **Silent-breakage classes** (common-mistakes.md): no cached hotpath flags, no queued/
  direct connection changes, no timestamp handling — the exposure here is purely
  geometric. The one to watch is stale QML bindings on removed properties, which QML
  surfaces loudly as ReferenceErrors at load; covered by opening every dialog once (AC1).

## Test & verification plan

- **Unit (I can run):** none applicable — no parser/JS logic. Static checks only.
- **Static (I run):** `python scripts/code-verify.py --check` on every touched C++/QML
  file; `qt-cpp-review` on the `Platform/` diff before handoff;
  `python scripts/sanitize-commit.py` before commit.
- **Integration (maintainer runs, app up with API server):**
  `pytest tests/integration/ -v` on one CSD platform (AC9) — guards against window-manager
  interaction regressions in workflows that open dialogs/windows.
- **Maintainer observations (per spec ACs):**
  - AC1: open every dialog on Win10 + one Linux session — sized to content, nothing
    clipped.
  - AC2: drag main window + one dialog to all four screen edges; aero-snap on Win10.
  - AC3: resize-cursor sweep along all edges/corners — activates only at the visible
    border.
  - AC4: long comboboxes near bottom/right borders — fully rendered, fully clickable.
  - AC5: Win10 DWM shadow present and native-looking; Linux shows the 1px border.
  - AC6: GNOME Wayland + KDE Wayland smoke — border, move/resize/snap, popups.
  - AC7: Settings has no shadow toggle; launch with an old `settings.ini` carrying
    `Window/CSDShadowEnabled` — ignored.
  - AC8: Win11 + macOS smoke — unchanged decorations, caption color, window controls.
- **Hotpath:** not touched; no `--benchmark-hotpath` run required (no gate risk).
