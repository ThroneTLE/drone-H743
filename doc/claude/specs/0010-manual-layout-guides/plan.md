---
spec: 0010-manual-layout-guides
phase: plan
status: approved     # draft -> approved (gate before /ss-tasks)
updated: 2026-07-15
---

# Plan 0010 — Figma-style alignment guides and small-window support in manual layout mode

> **Phase 2 of 4 — the HOW.** The technical design that satisfies every requirement in
> [`spec.md`](./spec.md). Read the relevant `doc/claude/` sub-docs and the *actual code*
> before writing this — a plan grounded in a stale mental model is worse than no plan.
> Gate: do not start `/ss-tasks` until a human marks this `approved`.

## Approach (one paragraph)

A new pure, stateless snap resolver (`UI::SnapGuides`, new TU) computes the snapped
geometry plus guide/spacing/size-match visuals for a move or resize gesture;
`UI::WindowManager` gathers the inputs at gesture start (cached sibling rects, canvas
size, grid settings, Alt state from the mouse event), calls the resolver on every
`handleDragMove`/`handleResizeMove`, applies the snapped rect, and publishes the visuals
through new notify properties that `DashboardCanvas.qml` renders — the same one-way
pattern the existing `snapIndicator` uses. The manual-mode half/quarter-canvas Aero snap
(`computeSnapRect`, `updateManualSnapIndicator`, `applyManualSnap`) is deleted; the
auto-layout drag-to-swap indicator keeps `snapIndicator` untouched. The 48×48 manual
floor comes from making `WidgetDelegate`'s `minimumWidth/Height` (which feed
`implicitWidth/Height`, the clamp source in `computeResizedGeometry`) mode-dependent,
plus lowering `constrainWindows`' hardcoded 100×80 fallback to 48×48 in manual mode and
removing the drag "tear-off to implicit size" block that would now shrink near-fullscreen
windows to 48×48. `MiniWindow`'s caption collapses progressively so the chrome stays
usable at 48 px.

## Affected subsystems & files

| File | Change |
|------|--------|
| `app/src/UI/SnapGuides.h` (new) | Pure value types (`SnapInput`, `SnapResult`, guide/spacing/size-match structs) + free-function resolvers `resolveMoveSnap()` / `resolveResizeSnap()` / `snapToGrid()`. No Qt GUI deps beyond QRect/QVector. |
| `app/src/UI/SnapGuides.cpp` (new) | Resolver implementation: edge/center candidates, equal-gap detection, size matching, grid quantization, priority merge. |
| `app/src/UI/WindowManager.h` | New Q_PROPERTYs (`alignmentGuides`, `spacingIndicators`, `sizeMatchRect`+visible, `manualGestureActive`, `manualGestureGeometry`, `gridEnabled`, `gridSize`); members for cached sibling rects + published visuals; drop `updateManualSnapIndicator`/`applyManualSnap` declarations. |
| `app/src/UI/WindowManager.cpp` | Gesture integration in `handleDragMove`/`handleResizeMove` (snap → apply → publish, Alt bypass); sibling-rect caching in `startManualPress`; clear visuals in `mouseReleaseEvent`/`setFrozen`/`clear`; delete `computeSnapRect` + manual-snap plumbing and the near-fullsize tear-off block; 48-px fallback floors in `constrainWindows` (manual mode); grid settings persisted via `m_settings`. |
| `app/CMakeLists.txt` | Register `SnapGuides.cpp` (and header, matching how sibling TUs are listed). |
| `app/qml/MainWindow/Panes/Dashboard/DashboardCanvas.qml` | Overlay layer above windows: guide-line Repeater, spacing indicators with px labels, size-match highlight, geometry badge (x, y, w×h), grid rendering (QML `Canvas`, repaint only on size/gridSize/theme change); context-menu entries "Show Grid" (checkable) + "Grid Size" preset submenu. |
| `app/qml/MainWindow/Panes/Dashboard/WidgetDelegate.qml` | `minimumWidth/Height` become mode-dependent: 48/48 when `!windowManager.autoLayoutEnabled`, 356/320 otherwise. |
| `app/qml/Widgets/MiniWindow.qml` | Progressive caption collapse: external-window button, title, minimize, maximize hide as width shrinks; close stays. Thresholds chosen so all hit targets remain ≥ caption-height sized. |
| `app/rcc/themes/default.json`, `fluent-dark.json`, `fluent-light.json` | Two new color keys: `alignment_guide` (guide lines, spacing, size-match, badge accent) and `canvas_grid` (grid lines). ThemeManager loads keys straight from JSON (`jsonObjectToVariantMap`), no C++ registration. |
| `CLAUDE.md` / `doc/claude/architecture/dashboard.md` | One short note: manual-mode snapping lives in `SnapGuides`, Aero snap is auto-mode-only, 48×48 manual floor. |

Out of lane (explicitly untouched): `ExternalWidgetWindow.qml`, auto-layout tiling
helpers, `Taskbar`, freeze plumbing, everything under `app/src/DataModel/`.

## Architecture & data flow

All on the QML/main thread at interaction rate; nothing touches ingest.

1. **Gesture start** — `startManualPress()` (already the single entry point for both
   event paths) additionally snapshots `m_snapSiblings`: the geometry of every visible
   `"normal"` window except the target, plus the canvas size. One O(N) walk per press.
2. **Move** — `handleDragMove()` manual branch: compute the unsnapped candidate rect from
   the delta (existing code), then, unless Alt is held (`event->modifiers()`), call
   `Snap::resolveMoveSnap({candidate, m_snapSiblings, canvas, kSnapThreshold,
   gridEnabled, gridSize})`. Apply `result.snapped` via `setX/setY`, publish
   `result.guides` / `result.spacings` into the notify properties. The existing
   `qBound` canvas clamp runs after snapping so a snap can never push a window
   off-canvas.
3. **Resize** — `handleResizeMove()`: `computeResizedGeometry()` produces the candidate;
   `Snap::resolveResizeSnap(..., m_resizeEdge)` adjusts only the moving edge(s), adds
   size-match candidates (sibling widths/heights), then the existing canvas clamp runs.
   Non-moving edges are never emitted by the resolver (R2).
4. **Publish** — visuals are plain properties: `alignmentGuides` (QVariantList of
   `{x, y, w, h, kind}` maps — guides are 1-px rects in canvas coords),
   `spacingIndicators` (QVariantList of `{x, y, w, h, gap}`), `sizeMatchRect` (QRect +
   visible flag), `manualGestureActive` + `manualGestureGeometry` (QRect; QML formats
   the badge text, keeping strings/translation out of C++). Each has one guarded
   notify signal; `DashboardCanvas.qml` binds Repeaters/Rectangles at
   `z: _wm.zCounter + 9999` (same layer the snap indicator uses).
5. **Gesture end / abort** — `mouseReleaseEvent` manual branch commits geometry as today
   (minus `applyManualSnap`) and clears all visuals; `setFrozen(true)` and `clear()`
   clear them too (freeze already aborts in-flight gestures).
6. **Grid** — `gridEnabled`/`gridSize` live on WindowManager, persisted in `m_settings`
   (like `WindowManager_Wallpaper`), toggled from the canvas context menu. The grid
   itself renders in QML (`Canvas`), only in manual mode, under the windows and above
   the wallpaper.

Alt state is sampled from each mouse-move event's modifiers, so pressing/releasing Alt
mid-gesture takes effect on the next pointer move (R7's "within the same gesture").

**Resolver semantics** (all in `SnapGuides.cpp`, pure):

- *Candidates (move):* for X — moving left/center/right vs each sibling's
  left/center/right, canvas 0/center/width; same for Y. Nearest candidate within
  `kSnapThreshold` (6 px) wins per axis; ties prefer edges over centers. Guides span
  from the moving rect to the farthest aligned sibling.
- *Equal spacing (move only, per spec):* for siblings that overlap the moving rect's
  cross-axis range, existing neighbor-pair gaps become gap candidates; snapping equalizes
  the new gap with the matched one and reports both gap rects with their px value.
- *Size match (resize only):* sibling widths/heights within threshold of the candidate
  size; on snap, report the matched sibling rect for the highlight.
- *Priority:* smart-guide candidate beats grid candidate on the same axis (spec R6);
  spacing beats plain edge snap only when strictly closer.
- *Bounds:* O(N) candidate collection with N = sibling count; no allocation beyond the
  result vectors (reserved); loops bounded by window count.

## Hotpath & threading impact

- **Touches the hotpath?** No. `WindowManager` is a UI-thread QQuickItem; every new call
  path is inside mouse-event handlers or property reads. Nothing on
  `FrameReader`/`CircularBuffer`/`FrameBuilder`/`Dashboard` ingest or draw. The
  `--benchmark-hotpath` gate is run once as a regression formality (AC12).
- **New cross-thread signal/slot?** None. All new signals are same-thread QML bindings.
- **New input to a cached hotpath flag?** None.
- **Timestamp ownership** — untouched; no export/report code in scope.

## Data model & persistence

- **No project-JSON or `Keys::` changes.** Manual geometries keep the exact
  `serializeLayout()` shape; sub-356×320 sizes already round-trip as plain ints (R11).
  Older builds loading a small-window layout clamp up via their own `implicitWidth`
  floor — degraded but safe, as the spec accepts.
- **QSettings:** `WindowManager_GridEnabled` (bool, default false),
  `WindowManager_GridSize` (int, default 16), read in the ctor next to the wallpaper key.
- No schema/writer version bump, no Sessions DB impact.

## API / SDK surface

None. No API handlers, no EnumLabels, no SDK regeneration. (If a future ask wants
`dashboard.setGrid` via API, it composes on the existing property setters.)

## QML / UI

- **DashboardCanvas.qml** — new sibling of `_snapIndicator`: an overlay `Item` visible
  only when `_wm.manualGestureActive && !_wm.autoLayoutEnabled && !_wm.frozen` (R10),
  containing (a) `Repeater` over `alignmentGuides` drawing 1-px rects in
  `alignment_guide`; (b) `Repeater` over `spacingIndicators` drawing the gap bars with a
  centered px `Label`; (c) size-match highlight rect around `sizeMatchRect`; (d) the
  geometry badge — a small rounded `Rectangle` + `Label` reading `x, y  ·  w × h`,
  anchored above the gesture rect and clamped into the canvas. Grid: a `Canvas` under
  the Instantiator (above wallpaper), painting vertical/horizontal lines in
  `canvas_grid` every `gridSize` px; `requestPaint()` only on size/gridSize/theme
  change. Context menu gains `Show Grid` (checkable) and a `Grid Size` submenu
  (8/16/24/32/64 px).
- **WidgetDelegate.qml** — `readonly property int minimumWidth:
  windowManager.autoLayoutEnabled ? 356 : 48` (same for height, 320/48). `width/height/
  implicitWidth/implicitHeight` already bind to these; `implicitWidth` is exactly what
  `computeResizedGeometry`/`constrainWindows`/`cascadeLayout` read, so the C++ floor
  follows the mode with no property round-trips. `cascadeLayout`'s `qMax(implicit, 200)`
  keeps first-placement sane; `autoLayout()` ignores implicit size entirely, so
  toggling back to auto-layout packs normally (AC10).
- **MiniWindow.qml** — collapse ladder driven by `root.width`: `< 200` hide the
  external-window button (its width already feeds `externControlWidth`, which the drag
  hit-test reads — the binding collapses to 0 automatically); `< 148` hide the title
  label; `< 120` hide minimize; `< 88` hide maximize; close never hides.
  `windowControlsWidth` is already the sum of the *visible* button MouseAreas, so the
  caption drag region stays correct at every step. At 48 px the caption is: drag strip +
  close button. RTL: the caption is a RowLayout, `LayoutMirroring` handles it as today;
  guides/badge live in canvas coordinates (the delegate container already pins
  `LayoutMirroring.enabled: false`).
- **Theme:** both new keys added to the three dashboard themes; guide color picked as
  the theme accent family (Figma-pink in default, accent-blue in fluent) — final values
  at implementation, validated against wallpapers via opacity 1.0 lines + the badge's
  solid background.

## Tradeoffs & alternatives considered

| Decision | Options | Chosen + why |
|----------|---------|--------------|
| Where snap math lives | QML handlers / inline in WindowManager / new pure TU | **New `SnapGuides` TU** — WindowManager already owns the gesture state machine; a pure resolver keeps the 2.2k-line TU from growing and makes the geometry math reviewable in isolation. |
| Guide rendering | QVariantList + QML Repeater / QQuickPaintedItem / custom SG node | **QVariantList + Repeater** — a handful of rects at interaction rate; matches the existing `snapIndicator` pattern; theme-aware for free. A paint item is warranted only if profiling shows binding churn, which ~6 guides won't. |
| Badge text | Composed in C++ / QRect published, QML formats | **QML formats** — keeps `tr()`/locale and string churn out of the C++ move handler; C++ publishes one QRect. |
| Grid toggle home | Canvas context menu / Settings dialog / dashboard toolbar | **Canvas context menu** (open question from spec, recommendation) — it's a canvas-scoped tool used mid-layout, next to the existing wallpaper/tile actions; Settings would hide it two dialogs away. |
| Grid scope | Per-project / global QSettings | **Global QSettings** — matches the wallpaper precedent; per-project grid would need project-JSON schema and migration for marginal value. |
| Caption at 48 px | Shrink all buttons to fit / progressive collapse | **Progressive collapse, close last** — three squeezed 16-px targets fail hit-target minimums and clip icons; minimize/maximize remain reachable via resize-larger or the taskbar. "At least the caption buttons can be shown" is read as *the caption stays functional*, not *all three fit*. Flagged for review here since the spec sentence allows either reading. |
| Near-fullsize drag tear-off | Keep (reset to implicit on drag) / remove in manual mode | **Remove** — it existed to un-stick Aero-snapped half/full windows; with Aero gone it would shrink a large window to 48×48 mid-drag. Auto-layout path unaffected. |
| Alt sampling | Event modifiers per move / QGuiApplication::keyboardModifiers | **Event modifiers** — deterministic per-event, testable, no global keyboard state queries; Alt takes effect on the next pointer move, which satisfies R7. |

## Risks & mitigations

- **`implicitWidth` is multi-consumer.** It feeds `computeResizedGeometry` (intended),
  but also `cascadeLayout` (safe: `qMax(…, 200)`), `constrainWindows` (fixed: manual
  floor 48), and the tear-off block (removed). Each consumer is named in tasks so none
  is silently left reading a 48 floor it can't handle. This is the diff's highest-risk
  edge and maps to AC9/AC10.
- **Snap fighting the canvas clamp.** Snapping runs before the existing
  `qBound`/edge clamps, so a clamped result may end 1-5 px off a reported guide.
  Mitigation: resolver receives the canvas rect and never emits candidates outside it.
- **Guided-property churn.** Publishing QVariantLists per mouse move re-evaluates QML
  bindings ~120 Hz worst case. Mitigation: guarded setters (skip emit when the guide
  set is unchanged — common while sliding along a held snap), reserve()d vectors, and
  the 30-window stutter check in verification.
- **Frozen-mode regression** (spec 0007's input gate). No new event entry points are
  added; all new logic sits inside branches already gated by
  `m_frozen`/`autoLayoutEnabled()` early-outs, and `setFrozen(true)` clears the new
  visuals exactly where it clears the snap indicator today. Counterfactual check at
  handoff names this rule.
- **Setter-guard misses** (`common-mistakes.md` Qt/QML row): every new property setter
  gets the `if (m_x == x) return;` guard; `code-verify.py` plus `qt-cpp-review` before
  handoff.
- **Old-layout compatibility** (R11): no serialization change at all — the risk is nil
  by construction; AC round-trips a small-window layout to prove it.
- **Theme key typos fail silent** (missing key → default-constructed color = black).
  Mitigation: keys added to all three themes in one task, eyeballed in light + dark.

## Test & verification plan

No pytest surface reaches canvas gestures (the API drives data, not mouse input), so
acceptance is maintainer-observed in-app, backed by static checks:

- **Maintainer in-app (maps 1:1 to spec ACs):** AC1-AC2 edge/center snap on move+resize
  with badge-verified 0-px deltas; AC3 three-window equal-gap; AC4 size-match; AC5
  badge lifecycle; AC6 grid toggle + persistence across restart; AC7 Alt bypass
  mid-gesture; AC8 no Aero rect at any canvas edge in manual mode; AC9 48×48 resize
  floor + caption usability + save/reload round-trip; AC10 auto-layout toggle round-trip
  with small windows; AC11 freeze/auto-mode show none of the new visuals. Plus the
  30-window drag-smoothness spot check (constraint) and an RTL-locale pass over badge
  and caption.
- **Hotpath:** one `--benchmark-hotpath` run post-implementation (AC12) — expected
  no-op; gate pass/fail only per the CI-noise memory.
- **Static (I run):** `python scripts/code-verify.py --check` on every touched C++/QML
  file; `qt-cpp-review` on `SnapGuides.*`/`WindowManager.*`; `scripts/sanitize-commit.py`
  before commit.
