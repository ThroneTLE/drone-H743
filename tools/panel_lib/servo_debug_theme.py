"""Scoped dark controls for the servo debug page; no state or command changes."""

from tkinter import ttk


def apply_servo_debug_theme(parent, palette):
    """Use the panel palette for every input, slider and nested page border.

    Clam's Spinbox does not inherit TEntry colors. Its field, arrow and disabled
    state must all be specified; notebook light/dark edges also retain defaults
    unless explicitly overridden. Styles are assigned only within this page.
    """
    style = ttk.Style(parent)
    for kind in ("TSpinbox", "TEntry"):
        name = f"ServoDebug.{kind}"
        style.configure(
            name, fieldbackground=palette["panel"], background=palette["raised"],
            foreground=palette["ink_dim"], insertcolor=palette["accent"],
            bordercolor=palette["border_strong"], lightcolor=palette["border"],
            darkcolor=palette["border"], arrowcolor=palette["muted"],
            selectbackground=palette["accent_soft"], selectforeground=palette["ink"],
            padding=(7, 4), arrowsize=12,
        )
        style.map(
            name,
            fieldbackground=[("disabled", palette["disabled"]), ("readonly", palette["panel"])],
            foreground=[("disabled", palette["muted"]), ("readonly", palette["ink_dim"])],
            bordercolor=[("disabled", palette["border"]), ("focus", palette["accent"])],
            arrowcolor=[("disabled", palette["muted"]), ("active", palette["ink"])],
            background=[("disabled", palette["disabled"]), ("active", palette["raised"])],
        )
    for kind in ("TNotebook", "TLabelframe"):
        style.configure(
            f"ServoDebug.{kind}", background=palette["surface"],
            bordercolor=palette["border"], lightcolor=palette["border"],
            darkcolor=palette["border"], borderwidth=1, relief="solid",
        )
    style.configure("ServoDebug.TLabelframe.Label", background=palette["surface"],
                    foreground=palette["muted"])
    style.configure(
        "ServoDebug.TNotebook.Tab", background=palette["canvas"], foreground=palette["muted"],
        bordercolor=palette["border"], lightcolor=palette["border"], darkcolor=palette["border"],
        padding=(14, 7),
    )
    style.map(
        "ServoDebug.TNotebook.Tab",
        background=[("selected", palette["panel"]), ("active", palette["raised"])],
        foreground=[("selected", palette["accent"]), ("active", palette["ink_dim"])],
        lightcolor=[("selected", palette["border"]), ("active", palette["border_strong"])],
        darkcolor=[("selected", palette["border"]), ("active", palette["border_strong"])],
        bordercolor=[("selected", palette["border"]), ("active", palette["border_strong"])],
    )
    style.configure("ServoDebug.TCheckbutton", background=palette["surface"],
                    lightcolor=palette["border"], darkcolor=palette["border"])
    style.map(
        "ServoDebug.TCheckbutton",
        background=[("disabled", palette["surface"]), ("active", palette["surface"])],
        foreground=[("disabled", palette["muted"]), ("active", palette["ink_dim"])],
        indicatorbackground=[("disabled", palette["disabled"]),
                             ("selected", palette["accent"]), ("active", palette["raised"])],
    )
    style.configure(
        "ServoDebug.Horizontal.TScale", troughcolor=palette["panel"],
        background=palette["accent"], bordercolor=palette["border_strong"],
        lightcolor=palette["border_strong"], darkcolor=palette["border_strong"],
        gripcount=0, sliderlength=18, sliderthickness=12,
    )
    style.map("ServoDebug.Horizontal.TScale", background=[("active", palette["accent_hover"])])

    styles = {"TSpinbox": "ServoDebug.TSpinbox", "TEntry": "ServoDebug.TEntry",
              "TNotebook": "ServoDebug.TNotebook", "TLabelframe": "ServoDebug.TLabelframe",
              "TCheckbutton": "ServoDebug.TCheckbutton",
              "TScale": "ServoDebug.Horizontal.TScale"}

    def apply(widget):
        name = styles.get(widget.winfo_class())
        if name is not None:
            widget.configure(style=name)
        for child in widget.winfo_children():
            apply(child)

    apply(parent)
