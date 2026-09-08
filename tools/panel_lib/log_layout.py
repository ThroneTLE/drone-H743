"""Wrap existing toolbars without copying their controls or actions."""


def wrap_toolbars(owner):
    for frame in owner.winfo_children():
        wrap_toolbars(frame)
        children = frame.pack_slaves()
        if not children or any(child.winfo_class() not in {
            "TButton", "TLabel", "TEntry", "TCombobox", "TCheckbutton", "TRadiobutton",
        } for child in children):
            continue
        for child in children:
            child.pack_forget()
        frame.pack_propagate(False)

        def layout(event=None, frame=frame, children=children):
            width = max(200, frame.winfo_width())
            y = used = row_height = 0
            for child in children:
                needed = min(width, child.winfo_reqwidth() + 8)
                if used and used + needed > width:
                    y += row_height
                    used = row_height = 0
                height = child.winfo_reqheight() + 6
                child.place(x=used + 4, y=y + 3, width=max(1, needed - 8), height=height - 6)
                used += needed
                row_height = max(row_height, height)
            frame.configure(height=y + row_height)

        frame.bind("<Configure>", layout, add="+")
        layout()
