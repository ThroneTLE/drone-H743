# Command palette

The command palette is a single search box that reaches every workspace, widget, tool, and command in Serial Studio without hunting through menus and toolbars. Press **Ctrl+K** anywhere in the main window or the Project Editor to open it, start typing, and press Enter on the result you want.

The palette is available in every edition; nothing on it is Pro-gated, though individual entries that open Pro tools are hidden in the free build just as they are everywhere else.

## Opening and closing

| Action | Keys |
|--------|------|
| Open (toggle) | **Ctrl+K** |
| Close | **Esc**, **Ctrl+W** (**Cmd+W** on macOS), or click outside the dialog |

**Ctrl+K** is a toggle: press it again while the palette is open to dismiss it. The taskbar's workspace switcher button opens the same palette on the dashboard.

The palette centers over the current window as a floating dialog. While it is open it holds the keyboard, so the dashboard behind it does not steal focus; the scene behind stays inert until you close the palette.

## What the palette lists

The palette shows different things depending on where you open it:

- **On the dashboard** it lists your [workspaces](Toolbar-Reference.md#a-note-on-workspaces) and workspace folders, every open widget, and the dashboard commands (Start menu tools, export toggles, pause, reset, and so on).
- **Before the dashboard opens** (Console view, or with no data yet) it lists the application commands: file actions, operation-mode switches, connection, preferences, and help.
- **In the [Project Editor](Project-Editor.md)** it lists the project commands: new, open, save, and the add-item actions from the editor toolbar.

## Browsing versus searching

The palette has two modes, and it switches between them automatically based on whether the search box is empty.

### Browse (empty search box)

With no text typed, the palette shows a grid of large cells. On the dashboard the first row is **Workspaces**, ending in an **Add Workspace** cell; below it, tools and commands are grouped into labeled sections. A cell marked **Folder** drills into a workspace folder, and a breadcrumb bar appears at the top so you can step back out. The **Back** button and the breadcrumb crumbs both navigate up the folder tree.

### Search (any text typed)

Start typing and the grid collapses into a dense list of result rows, grouped into sections. Each row shows the item's icon, its name, a dimmed path or category subtitle, and its keyboard shortcut where one exists. On the dashboard, a search spans these sections, each hidden when it has no matches:

| Section | Contains |
|---------|----------|
| **Folders** | Workspace folders whose name matches. |
| **Workspaces** | Workspaces whose name matches. |
| **Groups** | Dataset groups whose name matches. |
| **Widgets** | Open widgets whose name matches. |
| **Tools** | Commands and tools, grouped into the categories listed next. |

Command and tool results are grouped into fixed categories, shown in this order: **File**, **Operation Mode**, **Connection**, **View**, **Data Export**, **Console**, **Project**, **License**, **Tools**, **Help**.

## Keyboard navigation

The search box keeps focus the whole time, so you never have to reach for the mouse:

| Key | Browse mode | Search mode |
|-----|-------------|-------------|
| **Up / Down** | Move the highlight one row of cells | Move one result up or down |
| **Left / Right** | Move the highlight one cell | Move one result up or down |
| **Enter** | Open the highlighted cell (or the first cell) | Open the highlighted result (or the first result) |
| **Esc** | Close the palette | Close the palette |

Opening a workspace switches to it; opening a widget reveals it inside its workspace, or opens it in a single-widget window if it has none; opening a command runs it and closes the palette.

## See also

- [Toolbar & Button Reference](Toolbar-Reference.md): every button the palette's commands mirror, and the workspace switcher that also opens it.
- [Getting Started](Getting-Started.md): the main-window layout and a first connection walkthrough.
- [Operation Modes](Operation-Modes.md): how the Console and Dashboard views are chosen.
