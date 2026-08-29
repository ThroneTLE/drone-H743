# Shortcut checklist — spec 0028 (AC11)

Snapshot of every keyboard shortcut that existed before the registry migration (24 blocks in
`MainWindow.qml:416-516`, 7 in `ProjectEditor.qml:146-172`), with its disposition. After T20,
each row is re-verified in the running app by the maintainer.

Legend: **command** = sequence now declared on the named manifest command and instantiated by
the per-window registry `Instantiator`; **hand block** = deliberately retained QML `Shortcut`
(parameterized family or focus-dependent guard the manifest cannot express).

## Main window (24)

| # | Sequence | Old behavior | Disposition | Verified |
|---|----------|--------------|-------------|----------|
| 1 | StandardKey.Preferences | showSettingsDialog (if !runtimeMode) | command `app.preferences` | [ ] |
| 2 | StandardKey.Quit | quitApplication | command `app.quit` | [ ] |
| 3 | StandardKey.Open | CSV Player openFile (if !runtimeMode) | command `csv.open` | [ ] |
| 4 | Ctrl+F | focus taskbar search (dashboard + searchEnabled) | command `dashboard.focusSearch` | [ ] |
| 5 | Ctrl+M (application) | toggle start menu (dashboard) | command `dashboard.startMenu` | [ ] |
| 6 | PgDown | cycleWorkspace(+1) (dashboard) | command `workspace.next` | [ ] |
| 7 | PgUp | cycleWorkspace(-1) (dashboard) | command `workspace.previous` | [ ] |
| 8 | Tab | cycleWindow(-1) (dashboard, focus guard) | **hand block** (focus-owns-Tab guard) | [ ] |
| 9 | Backtab / Shift+Tab | cycleWindow(+1) (dashboard, focus guard) | **hand block** (focus-owns-Tab guard) | [ ] |
| 10 | Ctrl+Shift+W | closeActiveWindow (dashboard) | command `window.closeActive` | [ ] |
| 11 | Ctrl+Shift+M | minimizeActiveWindow (dashboard) | command `window.minimizeActive` | [ ] |
| 12 | Ctrl+Shift+L | toggleAutoLayout (dashboard) | command `dashboard.autoLayout` | [ ] |
| 13 | Ctrl+Shift+F | toggleFreeze (dashboard) | command `dashboard.freeze` | [ ] |
| 14 | Ctrl+Home | clearActiveWindow (dashboard) | command `window.clearActive` | [ ] |
| 15-23 | Ctrl+1 .. Ctrl+9 | jumpToWorkspaceIndex(0..8) (dashboard) | **hand blocks** (parameterized family) | [ ] |
| 24 | Ctrl+K | dashboard: switcher, else main palette | command `palette.open` (run branches in binding) | [ ] |

## Project editor (7)

| # | Sequence | Old behavior | Disposition | Verified |
|---|----------|--------------|-------------|----------|
| 25 | Ctrl+K | toggle editor palette | command `palette.open` (editor window instance) | [ ] |
| 26 | StandardKey.Open | ProjectModel openJsonFile | command `editor.open` | [ ] |
| 27 | StandardKey.New | ProjectModel newJsonFile | command `editor.new` | [ ] |
| 28 | StandardKey.Save | ProjectModel saveJsonFile | command `editor.save` | [ ] |
| 29 | StandardKey.Quit | quitApplication | command `app.quit` (shortcutWindows incl. editor) | [ ] |
| 30 | StandardKey.Back | ProjectEditor navigateBack | command `editor.navigateBack` | [ ] |
| 31 | StandardKey.Forward | ProjectEditor navigateForward | command `editor.navigateForward` | [ ] |

## Rules encoded (from plan.md)

- Shortcuts gate on the binding's `enabled` only; `visible` gates menus/toolbars/palette.
  (`app.quit` stays enabled always even where its Start-menu row is hidden.)
- Retained hand blocks: Tab, Backtab, Ctrl+1..9 in `MainWindow.qml` (11 of 31). Everything
  else is manifest-declared (20 of 31).
- Duplicate-sequence lint runs per window (`registry-verify.py`); Ctrl+K is one command
  instantiated in both windows, so no ambiguity.
