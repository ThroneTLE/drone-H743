"""Compact current-parameter editor; wire names come from the capability table."""

import tkinter as tk
from tkinter import ttk

from .proto import PROTO_REQ_PARAMS


def build_cascade_editor(panel, parent):
    frame = ttk.LabelFrame(parent, text="四环参数 · P → PID → P → PID", padding=8)
    frame.pack(fill=tk.X, pady=(10, 0))
    tabs = ttk.Notebook(frame)
    tabs.pack(fill=tk.X)
    panel.cascade_notebook = tabs
    panel.quick_parameter_vars = {}
    panel.pid_vars = {}
    for title, prefix, axes, terms in (
        ("位置 P", "pos", ("x", "y", "z"), ("kp",)),
        ("速度 PID", "vel", ("x", "y", "z"), ("kp", "ki", "kd")),
        ("角度 P", "att", ("roll", "pitch", "yaw"), ("kp",)),
        ("角速度 PID", "rate", ("roll", "pitch", "yaw"), ("kp", "ki", "kd")),
    ):
        page = ttk.Frame(tabs, padding=6)
        tabs.add(page, text=title)
        for col, term in enumerate(terms, 1):
            ttk.Label(page, text=term.upper()).grid(row=0, column=col, sticky="w")
            page.columnconfigure(col, weight=1)
        for row, axis in enumerate(axes, 1):
            ttk.Label(page, text=axis.upper()).grid(row=row, column=0, sticky="w")
            for col, term in enumerate(terms, 1):
                name = f"coax.{prefix}_{axis}_{term}"
                variable = tk.StringVar(value="")
                panel.quick_parameter_vars[name] = variable
                entry = ttk.Entry(page, textvariable=variable, width=8)
                entry.grid(row=row, column=col, sticky="ew", padx=4, pady=3)
                # Stage user edits immediately so incoming telemetry preserves drafts.
                entry.bind("<KeyRelease>", lambda _e, n=name, v=variable:
                           panel._set_param(n, v.get(), "local", dirty=True))
    buttons = ttk.Frame(frame)
    buttons.pack(fill=tk.X, pady=(6, 0))
    ttk.Button(buttons, text="发送修改", command=panel._send_pid_values).pack(side=tk.LEFT)
    ttk.Button(buttons, text="读取参数", command=lambda:
               panel._send_proto(PROTO_REQ_PARAMS, "PARAM?")).pack(side=tk.LEFT, padx=6)
    ttk.Label(frame, textvariable=panel.pid_ki_status_var,
              style="Muted.TLabel").pack(anchor="w")

