"""Search and inspect the signals of the latest calculation in memory."""

import tkinter as tk
from tkinter import ttk

import numpy as np

from .charts import FONT, PlotCanvas, SpeedMap


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
        self.keys = []
        self.active = []
        self.sample = 0
        self.note = tk.StringVar(value="按「計算」取得逐點訊號。")
        self.readout = tk.StringVar()
        self.filter = tk.StringVar()
        self.axis = tk.StringVar(value="時間")

        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", pady=(0, 6))
        self.export_button = ttk.Button(toolbar, text="匯出完整 CSV", command=export, state="disabled")
        self.export_button.pack(side="right")
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
                               exportselection=False, selectmode="extended")
        self.list.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(list_box, orient="vertical", command=self.list.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(list_box, orient="horizontal", command=self.list.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.list.configure(yscrollcommand=scroll.set, xscrollcommand=horizontal.set)
        ttk.Label(picker, text="Ctrl 或 Shift 可疊圖\n同單位最多 4 條", style="Muted.TLabel").pack(anchor="w", pady=(5, 0))

        view = ttk.Frame(body)
        view.grid(row=0, column=1, sticky="nsew")
        controls = ttk.Frame(view)
        controls.pack(fill="x", pady=(0, 4))
        ttk.Label(controls, text="檢視方式").pack(side="left", padx=(0, 6))
        axis = ttk.Combobox(controls, textvariable=self.axis, values=("時間", "距離", "賽道彩虹圖"),
                            state="readonly", width=12)
        axis.pack(side="left")
        plots = ttk.Frame(view)
        plots.pack(fill="both", expand=True)
        plots.columnconfigure(0, weight=1)
        plots.rowconfigure(0, weight=1)
        self.chart = SignalChart(plots, self.seek)
        self.speed_map = SpeedMap(plots, seek=self.seek)
        self.speed_map.configure(width=360, height=340)
        self.chart.grid(row=0, column=0, sticky="nsew")
        self.axis.trace_add("write", lambda *_: self.render())
        self.slider = ttk.Scale(view, from_=0, to=1, command=self.on_slide, state="disabled")
        self.slider.pack(fill="x", pady=4)
        readout = ttk.Label(view, textvariable=self.readout, style="Muted.TLabel", wraplength=360)
        readout.pack(fill="x")
        view.bind("<Configure>", lambda event: readout.configure(wraplength=max(150, event.width - 5)))

        self.filter.trace_add("write", self.rebuild_list)
        self.list.bind("<<ListboxSelect>>", lambda _event: self.render())

    def set_result(self, result, *, stale=False):
        selected = [self.keys[i] for i in self.list.curselection()]
        self.result = result
        self.sample = 0
        self.speed_map.sample = 0
        self.speed_map.set_result(result)
        self.slider.configure(to=max(len(result.time) - 1, 1), state="normal")
        self.slider.set(0)
        self.note.set(f"{len(result.signals)} 個訊號 · {len(result.time):,} 個取樣點")
        self.export_button.state(["!disabled"])
        self.rebuild_list(selected=selected)
        if stale:
            self.mark_stale()

    def mark_stale(self):
        if self.result is not None:
            self.note.set("上次計算的訊號 · 請重新計算以更新")
            self.export_button.state(["disabled"])

    def rebuild_list(self, *_, selected=None):
        if self.result is None:
            return
        if selected is None:
            selected = [self.keys[i] for i in self.list.curselection()]
        query = self.filter.get().strip().casefold()
        self.keys = []
        self.list.delete(0, tk.END)
        for key, signal in self.result.signals.items():
            text = f"{signal.label} [{signal.unit}]  ·  {key}"
            if query in text.casefold():
                self.keys.append(key)
                self.list.insert(tk.END, text)
                if key in selected:
                    self.list.selection_set(tk.END)
        if self.keys and not self.list.curselection():
            index = self.keys.index("speed_kmh") if "speed_kmh" in self.keys else 0
            self.list.selection_set(index)
            self.list.see(index)
        self.render()

    def render(self):
        show_map = self.axis.get() == "賽道彩虹圖"
        visible, hidden = (self.speed_map, self.chart) if show_map else (self.chart, self.speed_map)
        hidden.grid_remove()
        visible.grid(row=0, column=0, sticky="nsew")
        if self.result is None:
            return
        selected = [self.result.signals[self.keys[i]] for i in self.list.curselection()]
        self.active = [signal for signal in selected if signal.unit == selected[0].unit][:4] if selected else []
        if show_map:
            self.speed_map.sample = self.sample
            self.speed_map.draw_cursor()
        elif self.active:
            title = self.active[0].label if len(self.active) == 1 else "訊號比較"
            xlabel, x = ("時間 [s]", self.result.time) if self.axis.get() == "時間" else ("距離 [m]", self.result.track.s)
            series = tuple((str(i), signal.values) for i, signal in enumerate(self.active, 1))
            self.chart.sample = self.sample
            self.chart.set_plot((title, self.active[0].unit, xlabel, x, series))
        else:
            self.chart.empty_message = "請選取訊號" if self.keys else "找不到相符訊號"
            self.chart.set_plot(None)
        self.update_readout()

    def seek(self, index):
        self.slider.set(index)

    def on_slide(self, value):
        if self.result is None:
            return
        self.sample = min(round(float(value)), len(self.result.time) - 1)
        self.chart.sample = self.sample
        self.chart.draw_cursor()
        self.speed_map.sample = self.sample
        self.speed_map.draw_cursor()
        self.update_readout()

    def update_readout(self):
        index = self.sample
        point = f"取樣 {index + 1} · {self.result.time[index]:.3f} s · {self.result.track.s[index]:.2f} m"
        values = [
            f"{number} {signal.label} {signal.values[index]:.5g} {signal.unit}"
            for number, signal in enumerate(self.active, 1)
        ]
        self.readout.set("  |  ".join((point, *values)))
