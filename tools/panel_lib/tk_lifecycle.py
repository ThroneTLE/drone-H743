"""Release Tcl variables and images on their owning UI thread."""
import gc
import tkinter as tk
from tkinter import font, ttk


class _ClosedInterpreter:
    """Closed widgets keep normal TclError behavior without retaining Tcl."""
    def call(self, *args, **kwargs):
        raise tk.TclError("application has been destroyed")

    def __getattr__(self, name):
        return self.call


_CLOSED = _ClosedInterpreter()


def release_resources(interpreter):
    # Python cycles can survive Tk.destroy(), then be collected by a TCP worker.
    # Variable.__del__ would call Tcl from that worker and block/raise. Finalize
    # Tcl resources now, while the root's destroy caller still owns the thread.
    # Compare interpreter identity so another open Tk root remains untouched.
    for variable in gc.get_objects():
        if issubclass(type(variable), tk.Variable) and variable._tk is interpreter:
            tk.Variable.__del__(variable)
            variable._tk = None  # Tkinter's destructor explicitly accepts None.
            variable._root = None
        elif issubclass(type(variable), tk.Image) and variable.tk is interpreter:
            tk.Image.__del__(variable)
            variable.name = None  # Image.__del__ skips an already released name.
            variable.tk = None
        elif issubclass(type(variable), font.Font) and variable._tk is interpreter:
            font.Font.__del__(variable)
            variable.delete_font = False
            variable._tk = None
            variable._call = variable._split = _CLOSED.call
        elif issubclass(type(variable), (tk.Misc, ttk.Style)) and variable.tk is interpreter:
            # Widgets form Python cycles (root -> child -> master -> root).
            # Merely deleting Tcl variables leaves the interpreter in those
            # cycles; a worker's GC could then destroy the interpreter itself.
            variable.tk = _CLOSED
