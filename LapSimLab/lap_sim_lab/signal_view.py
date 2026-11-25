"""Search and inspect the signals of the latest calculation in memory."""

import tkinter as tk
from tkinter import ttk

import numpy as np

from .charts import COLORS, FONT, PlotCanvas, TrackMap
from .planner import GRAVITY
from .signals import Signal


MAP_VIEW = "賽道彩虹圖"


class SignalChart(PlotCanvas):
    def __init__(self, master, seek):
        super().__init__(master)
        self.configure(width=360, height=340)
        self.seek = seek
        self.plot = None
        self.empty_message = "完成圈速計算後選取訊號"
        self.bounds = None
        self.sample = 0
        self.bind("<Button-1>", self.on_click)
        self.bind("<B1-Motion>", self.on_click)

    def set_plot(self, plot):
        self.plot = plot
        if self.winfo_ismapped():
            self.draw()

    def draw_content(self, w, h):
        if self.plot is None:
            self.bounds = None
            self.label(w / 2, h / 2, self.empty_message, size=14, color="#e2e8f0")
            return
        self.bounds = self.draw_lines(w, h, *self.plot)
        self.draw_cursor()

    def draw_cursor(self):
        self.delete("sample")
        if self.bounds is None or self.plot is None:
            return
        left, right, top, bottom = self.bounds
        x_values = self.plot[3]
        fraction = (x_values[self.sample] - x_values[0]) / (x_values[-1] - x_values[0])
        x = left + fraction * (right - left)
        self.create_line(x, top, x, bottom, fill="#f8fafc", dash=(4, 4), tags="sample")

    def on_click(self, event):
        if self.bounds is None or self.plot is None:
            return
        left, right, _, _ = self.bounds
        x_values = self.plot[3]
        fraction = min(1.0, max(0.0, (event.x - left) / (right - left)))
        target = x_values[0] + fraction * (x_values[-1] - x_values[0])
        index = min(int(np.searchsorted(x_values, target)), len(x_values) - 1)
        if index > 0 and target - x_values[index - 1] < x_values[index] - target:
            index -= 1
        self.seek(index)


class SignalView(ttk.Frame):
    def __init__(self, master, export):
        super().__init__(master, padding=(8, 6))
        self.result = None
        self.speed_signal = None
        self.available_signals = {}
        self.keys = []
        self.checked = []
        self.export_enabled = False
        self.sample = 0
        self.note = tk.StringVar(value="按「計算」取得逐點訊號。")
        self.readout = tk.StringVar()
        self.filter = tk.StringVar()
        self.axis = tk.StringVar(value=MAP_VIEW)
        self.selection_note = tk.StringVar()

        summary = ttk.Frame(self)
        summary.pack(fill="x", pady=(0, 8))
        self.metrics = []
        self.time_label = tk.StringVar(value="預估圈速")
        labels = ("預估圈速", "取樣路徑長度", "最高車速", "最大橫向加速度")
        for column, label in enumerate(labels):
            summary.columnconfigure(column, weight=1, uniform="metric")
            card = ttk.Frame(summary, style="Panel.TFrame", padding=(12, 7))
            card.grid(row=0, column=column, sticky="ew", padx=(0, 6 if column < len(labels) - 1 else 0))
            heading = ttk.Label(card, text=label, style="Hint.TLabel")
            if column == 0:
                heading.configure(textvariable=self.time_label)
            heading.pack(anchor="w")
            value = tk.StringVar(value="—")
            self.metrics.append(value)
            ttk.Label(card, textvariable=value, style="Metric.TLabel").pack(anchor="w")

        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", pady=(0, 6))
        self.selected_export_button = ttk.Button(
            toolbar, text="匯出勾選訊號 CSV", command=lambda: export(signal_keys=tuple(self.checked)), state="disabled",
        )
        self.selected_export_button.pack(side="right")
        self.export_button = ttk.Button(toolbar, text="匯出完整 CSV", command=export, state="disabled")
        self.export_button.pack(side="right", padx=(0, 6))
        ttk.Label(toolbar, textvariable=self.note, style="Muted.TLabel").pack(side="left", fill="x")

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        picker = ttk.Frame(body)
        picker.grid(row=0, column=0, sticky="ns", padx=(0, 8))
        ttk.Label(picker, text="搜尋名稱、欄位或單位").pack(anchor="w")
        ttk.Entry(picker, textvariable=self.filter).pack(fill="x", pady=(3, 5))
        list_box = ttk.Frame(picker)
        list_box.pack(fill="both", expand=True)
        list_box.columnconfigure(0, weight=1)
        list_box.rowconfigure(0, weight=1)
        self.list = tk.Listbox(list_box, width=24, height=12, font=(FONT, 9),
                               exportselection=False, selectmode="browse")
        self.list.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(list_box, orient="vertical", command=self.list.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(list_box, orient="horizontal", command=self.list.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.list.configure(yscrollcommand=scroll.set, xscrollcommand=horizontal.set)
        selection_controls = ttk.Frame(picker)
        selection_controls.pack(fill="x", pady=(5, 0))
        self.clear_button = ttk.Button(selection_controls, text="清除勾選", command=self.clear_selection)
        self.clear_button.pack(side="right")
        ttk.Label(selection_controls, textvariable=self.selection_note, style="Muted.TLabel").pack(side="left")
        ttk.Label(picker, text="點一下或按空白鍵勾選／取消", style="Muted.TLabel").pack(anchor="w")

        view = ttk.Frame(body)
        view.grid(row=0, column=1, sticky="nsew")
        controls = ttk.Frame(view)
        controls.pack(fill="x", pady=(0, 4))
        ttk.Label(controls, text="檢視方式").pack(side="left", padx=(0, 6))
        axis = ttk.Combobox(controls, textvariable=self.axis, values=(MAP_VIEW, "時間", "距離"),
                            state="readonly", width=12)
        axis.pack(side="left")
        plots = ttk.Frame(view)
        plots.pack(fill="both", expand=True)
        plots.columnconfigure(0, weight=1)
        plots.rowconfigure(0, weight=1)
        self.chart = SignalChart(plots, self.seek)
        self.track_map = TrackMap(plots, seek=self.seek)
        self.track_map.configure(width=360, height=340)
        self.axis.trace_add("write", lambda *_: self.render())
        self.slider = ttk.Scale(view, from_=0, to=1, command=self.on_slide, state="disabled")
        self.slider.pack(fill="x", pady=4)
        readout = ttk.Label(view, textvariable=self.readout, style="Muted.TLabel", wraplength=360)
        readout.pack(fill="x")
        view.bind("<Configure>", lambda event: readout.configure(wraplength=max(150, event.width - 5)))

        self.filter.trace_add("write", self.rebuild_list)
        self.list.bind("<Button-1>", self.on_signal_click)
        self.list.bind("<space>", lambda _event: self.toggle_signal(self.list.index(tk.ACTIVE)))
        self.update_selection_controls()
        self.render()

    def set_result(self, result, *, stale=False):
        self.result = result
        self.time_label.set("加速時間" if result.track_source == "acceleration" else "預估圈速")
        self.speed_signal = result.signals.get("speed_kmh") or Signal("車速", "km/h", result.plan.speed * 3.6)
        self.available_signals = {
            key: signal for key, signal in result.signals.items()
            if signal.values.size and np.any(signal.values != signal.values[0])
        }
        for variable, value in zip(self.metrics, (
            f"{result.lap_time:.3f} s",
            f"{result.track.s[-1]:,.1f} m",
            f"{np.max(result.plan.speed) * 3.6:.1f} km/h",
            f"{np.max(np.abs(result.lateral_acceleration)) / GRAVITY:.2f} g",
        )):
            variable.set(value)
        self.checked = [key for key in self.checked if key in self.available_signals]
        self.sample = 0
        self.track_map.sample = 0
        self.slider.configure(to=max(len(result.time) - 1, 1), state="normal")
        self.slider.set(0)
        self.note.set(f"{len(self.available_signals)} 個變動訊號 · {len(result.time):,} 個取樣點")
        self.set_export_enabled(not stale)
        if not stale and self.axis.get() != MAP_VIEW:
            self.axis.set(MAP_VIEW)
        self.rebuild_list()
        if stale:
            self.mark_stale()

    def mark_stale(self):
        if self.result is not None:
            self.note.set("上次計算的訊號 · 請重新計算以更新")
            self.set_export_enabled(False)

    def set_export_enabled(self, enabled):
        self.export_enabled = enabled
        self.export_button.state(["!disabled" if enabled else "disabled"])
        self.update_selection_controls()

    def rebuild_list(self, *_):
        if self.result is None:
            return
        query = self.filter.get().strip().casefold()
        self.keys = []
        self.list.delete(0, tk.END)
        for key, signal in self.available_signals.items():
            text = f"{signal.label} [{signal.unit}]  ·  {key}"
            if query in text.casefold():
                self.keys.append(key)
                self.list.insert(tk.END, self.signal_row(key))
        self.render()

    def signal_row(self, key):
        signal = self.available_signals[key]
        mark = "☑" if key in self.checked else "☐"
        return f"{mark}  {signal.label} [{signal.unit}]  ·  {key}"

    def on_signal_click(self, event):
        index = self.list.nearest(event.y)
        bounds = self.list.bbox(index)
        if bounds is not None and bounds[1] <= event.y < bounds[1] + bounds[3]:
            self.list.focus_set()
            self.toggle_signal(index)
        return "break"

    def toggle_signal(self, index):
        if not 0 <= index < len(self.keys):
            return "break"
        self.list.activate(index)
        key = self.keys[index]
        if key in self.checked:
            self.checked.remove(key)
        elif len(self.checked) == len(COLORS):
            self.selection_note.set(f"已達上限 {len(COLORS)}")
            return "break"
        else:
            self.checked.append(key)
        self.list.delete(index)
        self.list.insert(index, self.signal_row(key))
        self.list.activate(index)
        self.list.selection_clear(0, tk.END)
        self.list.selection_set(index)
        self.update_selection_controls()
        self.render()
        return "break"

    def clear_selection(self):
        self.checked.clear()
        self.update_selection_controls()
        self.rebuild_list()

    def update_selection_controls(self):
        self.selection_note.set(f"已選 {len(self.checked)} / {len(COLORS)}")
        self.clear_button.configure(state="normal" if self.checked else "disabled")
        self.selected_export_button.state(["!disabled" if self.export_enabled and self.checked else "disabled"])

    def selected_signals(self):
        return [self.available_signals[key] for key in self.checked]

    def map_signal(self):
        return self.available_signals[self.checked[-1]] if self.checked else self.speed_signal

    @staticmethod
    def comparison_series(signals):
        unit = signals[0].unit
        if all(signal.unit == unit for signal in signals):
            return unit, tuple((str(index), signal.values) for index, signal in enumerate(signals, 1))
        series = []
        for index, signal in enumerate(signals, 1):
            low, high = float(signal.values.min()), float(signal.values.max())
            relative = (signal.values - low) * (100 / (high - low)) if high > low else np.zeros_like(signal.values)
            series.append((str(index), relative))
        return "各訊號相對幅度 %", tuple(series)

    def render(self):
        show_map = self.axis.get() == MAP_VIEW
        visible, hidden = (self.track_map, self.chart) if show_map else (self.chart, self.track_map)
        hidden.grid_remove()
        visible.grid(row=0, column=0, sticky="nsew")
        if self.result is None:
            return
        if show_map:
            self.track_map.sample = self.sample
            self.track_map.set_signal(self.result.track, self.map_signal())
        elif selected := self.selected_signals():
            title = selected[0].label if len(selected) == 1 else "訊號比較"
            xlabel, x = ("時間 [s]", self.result.time) if self.axis.get() == "時間" else ("距離 [m]", self.result.track.s)
            unit, series = self.comparison_series(selected)
            self.chart.sample = self.sample
            self.chart.set_plot((title, unit, xlabel, x, series))
        else:
            if not self.available_signals:
                self.chart.empty_message = "本次結果沒有變動訊號"
            else:
                self.chart.empty_message = "請勾選要顯示的訊號" if self.keys else "找不到相符訊號"
            self.chart.set_plot(None)
        self.update_readout()

    def seek(self, index):
        self.slider.set(index)

    def on_slide(self, value):
        if self.result is None:
            return
        self.sample = min(round(float(value)), len(self.result.time) - 1)
        plot = self.track_map if self.axis.get() == MAP_VIEW else self.chart
        plot.sample = self.sample
        plot.draw_cursor()
        self.update_readout()

    def update_readout(self):
        index = self.sample
        point = f"取樣 {index + 1} · {self.result.time[index]:.3f} s · {self.result.track.s[index]:.2f} m"
        if self.axis.get() == MAP_VIEW:
            signal = self.map_signal()
            values = [f"{signal.label} {signal.values[index]:.5g} {signal.unit}"]
        else:
            values = [
                f"{number} {signal.label} {signal.values[index]:.5g} {signal.unit}"
                for number, signal in enumerate(self.selected_signals(), 1)
            ]
        self.readout.set("  |  ".join((point, *values)))
