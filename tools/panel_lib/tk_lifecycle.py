"""Release Tcl variables on their owning UI thread."""
import gc
import tkinter as tk


def release_variables(interpreter):
    # Python cycles can survive Tk.destroy(), then be collected by a TCP worker.
    # Variable.__del__ would call Tcl from that worker and block/raise. Finalize
    # Tcl resources now, while the root's destroy caller still owns the thread.
    # Compare interpreter identity so another open Tk root remains untouched.
    for variable in gc.get_objects():
        if issubclass(type(variable), tk.Variable) and variable._tk is interpreter:
            tk.Variable.__del__(variable)
            variable._tk = None  # Tkinter's destructor explicitly accepts None.
            variable._root = None
