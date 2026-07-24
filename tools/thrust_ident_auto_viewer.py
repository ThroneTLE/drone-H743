#!/usr/bin/env python3
"""Viewer for thrust_ident_auto CSV files."""

from __future__ import annotations

import csv
import sys
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import pressure_rs485_gui as pressure_gui


@dataclass
class Dataset:
    label: str
    path: Path
    raw_rows: list[pressure_gui.LossRow]
    net_rows: list[pressure_gui.LossRow]
    baselines: dict[int, pressure_gui.IdentPoint | None]


def default_label(path: Path) -> str:
    name = path.stem
    prefix = "thrust_ident_auto_"
    return name[len(prefix):] if name.startswith(prefix) else name


def load_dataset(path: Path) -> Dataset:
    single1 = pressure_gui.load_ident_points(path, 1)
    single2 = pressure_gui.load_ident_points(path, 2)
    dual = pressure_gui.load_ident_points(path, 0)
    raw_rows = pressure_gui.compute_loss_rows(
        single1,
        single2,
        dual,
        pressure_gui.DEFAULT_MOTOR_KV,
        pressure_gui.DEFAULT_BATTERY_VOLTAGE,
        pressure_gui.DEFAULT_LOAD_FACTOR,
        pressure_gui.DEFAULT_PROP,
    )
    net_rows = pressure_gui.compute_loss_rows(
        pressure_gui.zero_baseline_points(single1),
        pressure_gui.zero_baseline_points(single2),
        pressure_gui.zero_baseline_points(dual),
        pressure_gui.DEFAULT_MOTOR_KV,
        pressure_gui.DEFAULT_BATTERY_VOLTAGE,
        pressure_gui.DEFAULT_LOAD_FACTOR,
        pressure_gui.DEFAULT_PROP,
    )
    if not raw_rows:
        raise ValueError(f"{path.name}: no common M1/M2/Dual pct points")
    return Dataset(
        label=default_label(path),
        path=path,
        raw_rows=raw_rows,
        net_rows=net_rows,
        baselines={
            1: pressure_gui.ident_baseline(single1),
            2: pressure_gui.ident_baseline(single2),
            0: pressure_gui.ident_baseline(dual),
        },
    )


class ThrustAutoViewer(tk.Tk):
    def __init__(self, initial_paths: list[Path]) -> None:
        super().__init__()
        self.title("Thrust Auto CSV Viewer")
        self.geometry("1280x820")
        self.minsize(1040, 680)

        self.datasets: list[Dataset] = []
        self.summary_var = tk.StringVar(value="Load thrust_ident_auto_*.csv files")
        self.zero_baseline_var = tk.BooleanVar(value=True)

        self._build_ui()
        for path in initial_paths:
            self._add_dataset(path)
        self._refresh_view()

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X)
        ttk.Button(top, text="Add CSV", command=self.add_csv).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(top, text="Remove Selected", command=self.remove_selected).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Clear", command=self.clear_all).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Export Table CSV", command=self.export_table_csv).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Export Plot PNG", command=self.export_plot_png).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(
            top,
            text="0% baseline",
            variable=self.zero_baseline_var,
            command=self._refresh_view,
        ).pack(side=tk.LEFT, padx=(16, 6))

        main = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(main)
        main.add(left, weight=1)
        ttk.Label(left, text="Datasets").pack(anchor="w")
        self.dataset_list = tk.Listbox(left, height=8, exportselection=False)
        self.dataset_list.pack(fill=tk.X, pady=(4, 8))
        self.dataset_list.bind("<<ListboxSelect>>", lambda _event: self._refresh_view())
        ttk.Label(left, textvariable=self.summary_var, justify=tk.LEFT, wraplength=320).pack(
            fill=tk.X, pady=(0, 8)
        )

        table_frame = ttk.LabelFrame(left, text="Aligned Data")
        table_frame.pack(fill=tk.BOTH, expand=True)
        columns = (
            "dataset",
            "pct",
            "pulse",
            "m1",
            "m2",
            "sum",
            "dual",
            "coeff",
            "loss",
            "rpm",
        )
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings", height=18)
        headings = {
            "dataset": "Data",
            "pct": "%",
            "pulse": "us",
            "m1": "M1 g",
            "m2": "M2 g",
            "sum": "M1+M2 g",
            "dual": "Dual g",
            "coeff": "Coeff",
            "loss": "Loss %",
            "rpm": "RPM est",
        }
        widths = {
            "dataset": 120,
            "pct": 48,
            "pulse": 58,
            "m1": 78,
            "m2": 78,
            "sum": 88,
            "dual": 78,
            "coeff": 70,
            "loss": 70,
            "rpm": 76,
        }
        for column in columns:
            self.table.heading(column, text=headings[column])
            self.table.column(column, width=widths[column], anchor=tk.E)
        self.table.column("dataset", anchor=tk.W)
        yscroll = ttk.Scrollbar(table_frame, orient=tk.VERTICAL, command=self.table.yview)
        xscroll = ttk.Scrollbar(table_frame, orient=tk.HORIZONTAL, command=self.table.xview)
        self.table.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self.table.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

        right = ttk.Frame(main)
        main.add(right, weight=3)
        self.figure = Figure(figsize=(8, 6), dpi=100, constrained_layout=True)
        self.ax_thrust = self.figure.add_subplot(2, 1, 1)
        self.ax_loss = self.figure.add_subplot(2, 1, 2)
        self.canvas = FigureCanvasTkAgg(self.figure, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, right, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(fill=tk.X)

    def add_csv(self) -> None:
        paths = filedialog.askopenfilenames(
            title="Open thrust auto CSV",
            initialdir=Path(__file__).parent,
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        for text in paths:
            self._add_dataset(Path(text))
        self._refresh_view()

    def _add_dataset(self, path: Path) -> None:
        try:
            dataset = load_dataset(path)
        except Exception as exc:
            messagebox.showerror("Load failed", str(exc))
            return
        if any(existing.path.resolve() == dataset.path.resolve() for existing in self.datasets):
            return
        self.datasets.append(dataset)
        self.dataset_list.insert(tk.END, dataset.label)

    def remove_selected(self) -> None:
        selected = list(self.dataset_list.curselection())
        for index in reversed(selected):
            self.dataset_list.delete(index)
            del self.datasets[index]
        self._refresh_view()

    def clear_all(self) -> None:
        self.datasets.clear()
        self.dataset_list.delete(0, tk.END)
        self._refresh_view()

    def selected_datasets(self) -> list[Dataset]:
        selected = list(self.dataset_list.curselection())
        if not selected:
            return self.datasets
        return [self.datasets[index] for index in selected]

    def rows_for(self, dataset: Dataset) -> list[pressure_gui.LossRow]:
        if self.zero_baseline_var.get():
            return dataset.net_rows
        return dataset.raw_rows

    def view_mode(self) -> str:
        return "net_0pct_baseline" if self.zero_baseline_var.get() else "raw"

    def _refresh_view(self) -> None:
        datasets = self.selected_datasets()
        self._refresh_summary(datasets)
        self._refresh_table(datasets)
        self._refresh_plot(datasets)

    def _refresh_summary(self, datasets: list[Dataset]) -> None:
        if not datasets:
            self.summary_var.set("No dataset loaded")
            return
        lines = []
        for dataset in datasets:
            rows = self.rows_for(dataset)
            usable = [row for row in rows if row.pct > 0 and row.single_sum_g > 0.0]
            selected = usable if usable else rows
            mean_coeff = sum(row.loss_coeff for row in selected) / len(selected)
            max_dual = max(rows, key=lambda row: row.dual_g)
            max_sum = max(rows, key=lambda row: row.single_sum_g)
            lines.append(
                f"{dataset.label}: mode={self.view_mode()}, points={len(rows)}, "
                f"dual_max={max_dual.dual_g:.1f}g@{max_dual.pct}%, "
                f"single_sum_max={max_sum.single_sum_g:.1f}g@{max_sum.pct}%, "
                f"loss={((1.0 - mean_coeff) * 100.0):.1f}%"
            )
            if self.zero_baseline_var.get():
                baseline_text = []
                for motor in (1, 2, 0):
                    baseline = dataset.baselines[motor]
                    if baseline is not None:
                        baseline_text.append(
                            f"{pressure_gui.motor_name(motor)} {baseline.thrust_g:.1f}g@{baseline.pct}%"
                        )
                lines.append("  baselines: " + ", ".join(baseline_text))
        self.summary_var.set("\n".join(lines))

    def _refresh_table(self, datasets: list[Dataset]) -> None:
        for item in self.table.get_children():
            self.table.delete(item)
        for dataset in datasets:
            for row in self.rows_for(dataset):
                coeff_text = "" if row.pct == 0 or abs(row.single_sum_g) <= 1e-9 else f"{row.loss_coeff:.3f}"
                loss_text = "" if row.pct == 0 or abs(row.single_sum_g) <= 1e-9 else f"{row.loss_pct:.1f}"
                self.table.insert(
                    "",
                    tk.END,
                    values=(
                        dataset.label,
                        row.pct,
                        row.pulse_us,
                        f"{row.single1_g:.1f}",
                        f"{row.single2_g:.1f}",
                        f"{row.single_sum_g:.1f}",
                        f"{row.dual_g:.1f}",
                        coeff_text,
                        loss_text,
                        f"{row.rpm_loaded_est:.0f}",
                    ),
                )

    def _refresh_plot(self, datasets: list[Dataset]) -> None:
        self.ax_thrust.clear()
        self.ax_loss.clear()
        if not datasets:
            self.ax_thrust.set_title("Load one or more thrust_ident_auto CSV files")
            self.canvas.draw_idle()
            return

        for dataset in datasets:
            rows = self.rows_for(dataset)
            pct = [row.pct for row in rows]
            self.ax_thrust.plot(pct, [row.single1_g for row in rows], "--", label=f"{dataset.label} M1")
            self.ax_thrust.plot(pct, [row.single2_g for row in rows], "--", label=f"{dataset.label} M2")
            self.ax_thrust.plot(pct, [row.single_sum_g for row in rows], ":", label=f"{dataset.label} M1+M2")
            self.ax_thrust.plot(pct, [row.dual_g for row in rows], "-", linewidth=2, label=f"{dataset.label} Dual")
            loss_rows = [row for row in rows if row.pct > 0 and row.single_sum_g > 0.0]
            self.ax_loss.plot(
                [row.pct for row in loss_rows],
                [row.loss_coeff for row in loss_rows],
                "o-",
                label=f"{dataset.label} coeff",
            )

        if self.zero_baseline_var.get():
            self.ax_thrust.set_title("Net Thrust vs PWM Command (0% baseline removed)")
            self.ax_thrust.set_ylabel("Net thrust (g)")
        else:
            self.ax_thrust.set_title("Raw Thrust vs PWM Command")
            self.ax_thrust.set_ylabel("Raw thrust (g)")
        self.ax_thrust.set_xlabel("PWM command (%)")
        self.ax_thrust.grid(True, alpha=0.3)
        self.ax_thrust.legend(fontsize=8, ncols=2)

        self.ax_loss.axhline(1.0, color="0.4", linestyle="--", linewidth=1)
        self.ax_loss.set_title("Dual Loss Coefficient = Dual / (M1 + M2)")
        self.ax_loss.set_xlabel("PWM command (%)")
        self.ax_loss.set_ylabel("Coefficient")
        self.ax_loss.grid(True, alpha=0.3)
        self.ax_loss.legend(fontsize=8)
        self.canvas.draw_idle()

    def export_table_csv(self) -> None:
        datasets = self.selected_datasets()
        if not datasets:
            messagebox.showinfo("No data", "Load a CSV first.")
            return
        path_text = filedialog.asksaveasfilename(
            title="Export aligned table",
            initialdir=Path(__file__).parent,
            initialfile="thrust_auto_view_table.csv",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path_text:
            return
        path = Path(path_text)
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "view_mode",
                "dataset",
                "pct",
                "pulse_us",
                "m1_g",
                "m2_g",
                "single_sum_g",
                "dual_g",
                "loss_coeff",
                "loss_pct",
                "rpm_loaded_est",
            ])
            for dataset in datasets:
                for row in self.rows_for(dataset):
                    coeff_text = "" if row.pct == 0 or abs(row.single_sum_g) <= 1e-9 else f"{row.loss_coeff:.6f}"
                    loss_text = "" if row.pct == 0 or abs(row.single_sum_g) <= 1e-9 else f"{row.loss_pct:.3f}"
                    writer.writerow([
                        self.view_mode(),
                        dataset.label,
                        row.pct,
                        row.pulse_us,
                        f"{row.single1_g:.3f}",
                        f"{row.single2_g:.3f}",
                        f"{row.single_sum_g:.3f}",
                        f"{row.dual_g:.3f}",
                        coeff_text,
                        loss_text,
                        f"{row.rpm_loaded_est:.1f}",
                    ])

    def export_plot_png(self) -> None:
        path_text = filedialog.asksaveasfilename(
            title="Export plot",
            initialdir=Path(__file__).parent,
            initialfile="thrust_auto_view.png",
            defaultextension=".png",
            filetypes=(("PNG image", "*.png"), ("All files", "*.*")),
        )
        if not path_text:
            return
        self.figure.savefig(path_text, dpi=160)


def main() -> None:
    paths = [Path(arg) for arg in sys.argv[1:]]
    app = ThrustAutoViewer(paths)
    app.mainloop()


if __name__ == "__main__":
    main()
