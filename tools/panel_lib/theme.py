"""Shared dark theme and narrow chart styling seam for the Tk panel."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk


UI_PALETTE = {
    "canvas": "#171B22",
    "surface": "#1E232B",
    "panel": "#262C36",
    "raised": "#323945",
    "ink": "#EDF1F7",
    "ink_dim": "#C6CEDA",
    "muted": "#8D97A6",
    "border": "#39414E",
    "border_strong": "#4C5666",
    "navy": "#262C36",
    "accent": "#4DA3F5",
    "accent_hover": "#7BBEFF",
    "accent_ink": "#0A1A29",
    "accent_soft": "#28374A",
    "blue": "#4DA3F5",
    "blue_hover": "#7BBEFF",
    "blue_soft": "#28374A",
    "green": "#4ADE97",
    "green_soft": "#22352C",
    "amber": "#F2B441",
    "amber_soft": "#38311F",
    "amber_hover": "#FFC960",
    "red": "#FF8B82",
    "red_hover": "#FFA9A2",
    "red_soft": "#3B2729",
    "disabled": "#242A33",
    "disabled_ink": "#6B7482",
    "console": "#171B21",
}
UI_FONT = "Microsoft YaHei UI"
UI_MONO = "Consolas"
UI_SIZE = 10
UI_SIZE_SM = 9
UI_SIZE_TITLE = 16


def apply_matplotlib_theme(figure, palette: dict[str, str] = UI_PALETTE) -> None:
    """Apply only visual defaults to an existing matplotlib figure.

    Page modules keep ownership of samples, axes, and redraw policy. This helper
    owns no state and deliberately does not call ``clear`` or alter any data.
    """
    figure.patch.set_facecolor(palette["panel"])
    for axis in figure.axes:
        axis.set_facecolor(palette["panel"])
        axis.tick_params(colors=palette["ink_dim"], labelcolor=palette["ink_dim"])
        axis.xaxis.label.set_color(palette["ink_dim"])
        axis.yaxis.label.set_color(palette["ink_dim"])
        axis.title.set_color(palette["ink"])
        axis.grid(True, color=palette["border"], alpha=0.65, linewidth=0.7)
        for spine in axis.spines.values():
            spine.set_color(palette["border_strong"])


class PanelThemeMixin:
    """Install the one shared semantic ttk theme used by every page."""

    def _configure_style(self) -> None:
        palette = UI_PALETTE
        self.ui_palette = palette
        self.configure(background=palette["canvas"])
        self.option_add("*Font", (UI_FONT, UI_SIZE))
        self.option_add("*Text.Font", (UI_FONT, UI_SIZE))
        self.option_add("*Listbox.background", palette["panel"])
        self.option_add("*Listbox.foreground", palette["ink"])
        self.option_add("*Listbox.selectBackground", palette["accent"])
        self.option_add("*Listbox.selectForeground", palette["accent_ink"])
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure(".", font=(UI_FONT, UI_SIZE), background=palette["surface"],
                        foreground=palette["ink"])

        # 容器
        style.configure("TFrame", background=palette["surface"])
        style.configure("Shell.TFrame", background=palette["canvas"])
        style.configure("Page.TFrame", background=palette["surface"])
        style.configure("Card.TFrame", background=palette["panel"])
        style.configure("AccentBar.TFrame", background=palette["accent"])
        style.configure("Rule.TFrame", background=palette["border"])

        # 文字
        style.configure("TLabel", background=palette["surface"], foreground=palette["ink"])
        style.configure(
            "PageTitle.TLabel", background=palette["surface"], foreground=palette["ink"],
            font=(UI_FONT, UI_SIZE_TITLE, "bold"),
        )
        style.configure(
            "Eyebrow.TLabel", background=palette["surface"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE_SM, "bold"),
        )
        style.configure(
            "SectionTitle.TLabel", background=palette["surface"], foreground=palette["ink_dim"],
            font=(UI_FONT, UI_SIZE, "bold"),
        )
        style.configure("Muted.TLabel", background=palette["surface"], foreground=palette["muted"])
        style.configure("CardMuted.TLabel", background=palette["panel"], foreground=palette["muted"])
        style.configure("Card.TLabel", background=palette["panel"], foreground=palette["ink"])
        style.configure(
            "Mono.TLabel", background=palette["surface"], foreground=palette["ink_dim"],
            font=(UI_MONO, UI_SIZE),
        )
        style.configure(
            "CardMono.TLabel", background=palette["panel"], foreground=palette["ink_dim"],
            font=(UI_MONO, UI_SIZE),
        )
        style.configure(
            "Guide.TLabel", background=palette["accent_soft"], foreground=palette["ink"],
            font=(UI_FONT, UI_SIZE), padding=(10, 6),
        )
        for name, color in (
            ("Pass", palette["green"]),
            ("Warn", palette["amber"]),
            ("Fail", palette["red"]),
        ):
            style.configure(
                f"{name}.TLabel", background=palette["surface"], foreground=color,
                font=(UI_FONT, UI_SIZE, "bold"), padding=(2, 2),
            )
            style.configure(
                f"Card{name}.TLabel", background=palette["panel"], foreground=color,
                font=(UI_FONT, UI_SIZE, "bold"), padding=(2, 2),
            )
        style.configure(
            "Idle.TLabel", background=palette["surface"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE), padding=(2, 2),
        )
        style.configure(
            "Active.TLabel", background=palette["surface"], foreground=palette["accent"],
            font=(UI_FONT, UI_SIZE, "bold"), padding=(2, 2),
        )
        style.configure(
            "CardIdle.TLabel", background=palette["panel"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE), padding=(2, 2),
        )

        # 分组框
        style.configure(
            "TLabelframe", background=palette["surface"], bordercolor=palette["border"],
            lightcolor=palette["border"], darkcolor=palette["border"], relief=tk.SOLID,
        )
        style.configure(
            "TLabelframe.Label", background=palette["surface"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE_SM, "bold"), padding=(4, 0),
        )

        # 按钮
        style.configure(
            "TButton", background=palette["surface"], foreground=palette["ink_dim"],
            bordercolor=palette["border_strong"], focusthickness=1,
            focuscolor=palette["accent"], relief=tk.SOLID,
            padding=(11, 5), font=(UI_FONT, UI_SIZE),
        )
        style.map(
            "TButton",
            background=[("disabled", palette["disabled"]), ("active", palette["raised"])],
            foreground=[("disabled", palette["disabled_ink"]), ("active", palette["ink"])],
            bordercolor=[("disabled", palette["border"])],
        )
        style.configure(
            "Primary.TButton", background=palette["accent"], foreground=palette["accent_ink"],
            bordercolor=palette["accent"], padding=(14, 6), font=(UI_FONT, UI_SIZE, "bold"),
        )
        style.map(
            "Primary.TButton",
            background=[("disabled", palette["disabled"]), ("pressed", palette["accent"]),
                        ("active", palette["accent_hover"])],
            foreground=[("disabled", palette["disabled_ink"]), ("!disabled", palette["accent_ink"])],
            bordercolor=[("disabled", palette["border"]), ("!disabled", palette["accent"])],
        )
        for name, color, soft, hover in (
            ("Success", palette["green"], palette["green_soft"], palette["green"]),
            ("Warning", palette["amber"], palette["amber_soft"], palette["amber_hover"]),
            ("Danger", palette["red"], palette["red_soft"], palette["red_hover"]),
        ):
            style.configure(
                f"{name}.TButton", background=soft, foreground=color,
                bordercolor=color, padding=(12, 5), font=(UI_FONT, UI_SIZE, "bold"),
            )
            style.map(
                f"{name}.TButton",
                background=[("disabled", palette["disabled"]), ("active", palette["raised"])],
                foreground=[("disabled", palette["disabled_ink"]), ("active", hover)],
                bordercolor=[("disabled", palette["border"]), ("!disabled", color)],
            )
        style.configure(
            "Secondary.TButton", background=palette["surface"], foreground=palette["muted"],
            bordercolor=palette["border"], padding=(11, 5), font=(UI_FONT, UI_SIZE),
        )
        style.map(
            "Secondary.TButton",
            background=[("active", palette["raised"]), ("disabled", palette["disabled"])],
            foreground=[("disabled", palette["disabled_ink"]), ("active", palette["ink_dim"])],
        )
        style.configure(
            "Link.TButton", background=palette["surface"], foreground=palette["accent"],
            bordercolor=palette["surface"], relief=tk.FLAT, padding=(6, 4),
        )
        style.map(
            "Link.TButton",
            background=[("active", palette["accent_soft"]), ("disabled", palette["surface"])],
            foreground=[("disabled", palette["disabled_ink"])],
        )

        # 页签
        style.configure(
            "TNotebook", background=palette["canvas"], bordercolor=palette["border"],
            lightcolor=palette["border"], darkcolor=palette["border"],
            tabmargins=(2, 4, 2, 0),
        )
        style.configure(
            "TNotebook.Tab", background=palette["canvas"], foreground=palette["muted"],
            bordercolor=palette["canvas"], padding=(11, 6), font=(UI_FONT, UI_SIZE_SM),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", palette["surface"]), ("active", palette["raised"])],
            foreground=[("selected", palette["accent"]), ("active", palette["ink_dim"])],
            font=[("selected", (UI_FONT, UI_SIZE_SM, "bold"))],
        )

        # 表格
        style.configure(
            "Treeview", background=palette["panel"], fieldbackground=palette["panel"],
            foreground=palette["ink_dim"], bordercolor=palette["border"],
            rowheight=21, font=(UI_MONO, UI_SIZE),
        )
        style.map(
            "Treeview", background=[("selected", palette["accent_soft"])],
            foreground=[("selected", palette["ink"])],
        )
        style.configure(
            "Treeview.Heading", background=palette["raised"], foreground=palette["muted"],
            bordercolor=palette["border"], relief=tk.FLAT, padding=(8, 5),
            font=(UI_FONT, UI_SIZE_SM, "bold"),
        )
        style.map("Treeview.Heading", background=[("active", palette["border_strong"])])

        # 输入控件：名称按语义而不是页面归属复用。所有状态均给出完整可见底色，
        # invalid 只改变强调色，具体错误仍必须由页面文字说明。
        for name, base in (
            ("TEntry", "TEntry"),
            ("Numeric.TEntry", "TEntry"),
        ):
            del base
            style.configure(
                name, fieldbackground=palette["panel"], foreground=palette["ink"],
                insertcolor=palette["accent"], bordercolor=palette["border_strong"],
                lightcolor=palette["border"], darkcolor=palette["border"], padding=(6, 4),
            )
            style.map(
                name,
                fieldbackground=[("disabled", palette["disabled"]), ("readonly", palette["panel"]),
                                 ("invalid", palette["amber_soft"]), ("focus", palette["panel"])],
                foreground=[("disabled", palette["disabled_ink"]), ("readonly", palette["ink"]),
                            ("invalid", palette["ink"])],
                bordercolor=[("focus", palette["accent"]), ("invalid", palette["amber"])],
            )

        spinbox_style = {
            "fieldbackground": palette["panel"], "background": palette["raised"],
            "foreground": palette["ink"], "insertcolor": palette["accent"],
            "bordercolor": palette["border_strong"], "lightcolor": palette["border"],
            "darkcolor": palette["border"], "arrowcolor": palette["muted"],
            "selectbackground": palette["accent_soft"], "selectforeground": palette["ink"],
            "padding": (6, 4), "arrowsize": 12,
        }
        for name in ("TSpinbox", "Numeric.TSpinbox"):
            style.configure(name, **spinbox_style)
            style.map(
                name,
                fieldbackground=[("disabled", palette["disabled"]), ("readonly", palette["panel"]),
                                 ("invalid", palette["amber_soft"]), ("focus", palette["panel"])],
                foreground=[("disabled", palette["disabled_ink"]), ("readonly", palette["ink"]),
                            ("invalid", palette["ink"])],
                bordercolor=[("focus", palette["accent"]), ("invalid", palette["amber"])],
                arrowcolor=[("disabled", palette["muted"]), ("active", palette["ink"])],
                background=[("disabled", palette["disabled"]), ("active", palette["raised"])],
            )

        style.configure(
            "TCombobox", fieldbackground=palette["panel"], background=palette["panel"],
            foreground=palette["ink"], arrowcolor=palette["muted"],
            bordercolor=palette["border_strong"], lightcolor=palette["border"],
            darkcolor=palette["border"], padding=(5, 3), arrowsize=13,
        )
        style.configure("Numeric.TCombobox", **style.configure("TCombobox"))
        style.map(
            "TCombobox",
            fieldbackground=[("disabled", palette["disabled"]), ("readonly", palette["panel"]),
                             ("invalid", palette["amber_soft"]), ("focus", palette["panel"])],
            foreground=[("disabled", palette["disabled_ink"]), ("readonly", palette["ink"]),
                        ("invalid", palette["ink"])],
            selectbackground=[("readonly", palette["panel"])],
            selectforeground=[("readonly", palette["ink"])],
            bordercolor=[("focus", palette["accent"]), ("invalid", palette["amber"])],
        )
        style.map(
            "Numeric.TCombobox",
            fieldbackground=[("disabled", palette["disabled"]), ("readonly", palette["panel"]),
                             ("invalid", palette["amber_soft"]), ("focus", palette["panel"])],
            foreground=[("disabled", palette["disabled_ink"]), ("readonly", palette["ink"]),
                        ("invalid", palette["ink"])],
            bordercolor=[("focus", palette["accent"]), ("invalid", palette["amber"])],
        )
        style.configure(
            "Numeric.Horizontal.TScale", troughcolor=palette["panel"],
            background=palette["accent"], bordercolor=palette["border_strong"],
            lightcolor=palette["border_strong"], darkcolor=palette["border_strong"],
            gripcount=0, sliderlength=18, sliderthickness=12,
        )
        style.map(
            "Numeric.Horizontal.TScale",
            background=[("active", palette["accent_hover"])],
        )
        style.configure(
            "Horizontal.TProgressbar", troughcolor=palette["panel"], background=palette["accent"],
            bordercolor=palette["border"], lightcolor=palette["accent"],
            darkcolor=palette["accent"], thickness=6,
        )
        style.configure(
            "TCheckbutton", background=palette["surface"], foreground=palette["ink_dim"],
            indicatorbackground=palette["panel"], indicatorforeground=palette["accent_ink"],
            bordercolor=palette["border_strong"], indicatormargin=(0, 0, 6, 0), padding=(3, 2),
        )
        style.map(
            "TCheckbutton",
            background=[("disabled", palette["surface"]), ("active", palette["surface"])],
            indicatorbackground=[("disabled", palette["disabled"]), ("selected", palette["accent"]),
                                 ("active", palette["raised"])],
            foreground=[("disabled", palette["disabled_ink"]), ("active", palette["ink"])],
        )
        style.configure("TSeparator", background=palette["border"])
        style.configure("TPanedwindow", background=palette["canvas"], sashwidth=5)
        style.configure(
            "Vertical.TScrollbar", background=palette["raised"], troughcolor=palette["surface"],
            bordercolor=palette["surface"], arrowcolor=palette["muted"], width=11,
        )
        style.map("Vertical.TScrollbar", background=[("active", palette["border_strong"])])
        style.configure(
            "Horizontal.TScrollbar", background=palette["raised"], troughcolor=palette["surface"],
            bordercolor=palette["surface"], arrowcolor=palette["muted"],
        )


__all__ = [
    "PanelThemeMixin",
    "UI_FONT",
    "UI_MONO",
    "UI_PALETTE",
    "UI_SIZE",
    "UI_SIZE_SM",
    "UI_SIZE_TITLE",
    "apply_matplotlib_theme",
]
