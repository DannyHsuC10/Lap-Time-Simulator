"""Tk view for comparing speed slices of the GGV envelope."""

from queue import Empty, SimpleQueue
from threading import Thread
import tkinter as tk
from tkinter import ttk

import numpy as np

from .charts import GRID, PlotCanvas, speed_color
from .ggv import build_ggv
from .ggv_surface import GGVSurface


class GGVChart(PlotCanvas):
    def __init__(self, master):
        super().__init__(master)
        self.slices = []
        self.selection = 0

    def draw_content(self, w, h):
        if not self.slices:
            self.label(w / 2, h / 2, "按「計算 GGV」查看車輛加速度上限", size=14, color="#e2e8f0")
            return

        left, right, top, bottom = 64, w - 24, 54, h - 53
        ay_limit = max(0.1, max(float(np.max(np.abs(part.ay_g))) for part in self.slices)) * 1.08
        ax_min = min(-0.1, min(float(part.brake_g.min()) for part in self.slices)) * 1.08
        ax_max = max(0.1, max(float(part.drive_g.max()) for part in self.slices)) * 1.08
        self.label(18, 18, "縱向加速度 ax [g]", anchor="w", bold=True, color="#e2e8f0")
        self.label((left + right) / 2, h - 12, "橫向加速度 ay [g]   右彎 ←  → 左彎")
        for index in range(7):
            fraction = index / 6
            x = left + fraction * (right - left)
            self.create_line(x, top, x, bottom, fill="#64748b" if index == 3 else GRID)
            self.label(x, bottom + 17, f"{(2 * fraction - 1) * ay_limit:.2f}")
        for index in range(6):
            fraction = index / 5
            y = bottom - fraction * (bottom - top)
            self.create_line(left, y, right, y, fill=GRID)
            self.label(left - 8, y, f"{ax_min + fraction * (ax_max - ax_min):.2f}", anchor="e")
        zero = bottom + ax_min / (ax_max - ax_min) * (bottom - top)
        self.create_line(left, zero, right, zero, fill="#64748b")

        indices = (
            np.linspace(0, len(self.slices) - 1, 5, dtype=int)
            if self.selection == 0 else (self.selection - 1,)
        )
        for position, index in enumerate(indices):
            part = self.slices[index]
            color = speed_color(index / (len(self.slices) - 1))
            legend_x = left + position * (right - left) / len(indices)
            self.create_line(legend_x, 37, legend_x + 10, 37, fill=color, width=3)
            self.label(legend_x + 13, 37, f"{part.speed_kmh:g}", color=color, anchor="w", size=9)
            xs = left + (part.ay_g + ay_limit) / (2 * ay_limit) * (right - left)
            for branch in (part.drive_g, part.brake_g):
                ys = bottom - (branch - ax_min) / (ax_max - ax_min) * (bottom - top)
                self.create_line(*np.column_stack((xs, ys)).ravel().tolist(), fill=color, width=2)
        self.label(right, 18, "色標 [km/h]", anchor="e")


class GGVPage(ttk.Frame):
    def __init__(self, master, read_settings):
        super().__init__(master, padding=(8, 6))
        self.read_settings = read_settings
        self.revision = 0
        self.results = SimpleQueue()
        self.note = tk.StringVar(value="可用目前車輛設定計算，無需先跑圈速。")
        controls = ttk.Frame(self)
        controls.pack(fill="x", pady=(0, 5))
        self.run_button = ttk.Button(controls, text="計算 GGV", command=self.calculate)
        self.run_button.pack(side="left", padx=(0, 8))
        self.view = ttk.Combobox(controls, width=10, values=("3D 曲面", "2D 切片"), state="readonly")
        self.view.current(0)
        self.view.pack(side="left", padx=(0, 8))
        self.view.bind("<<ComboboxSelected>>", self.select_view)
        self.speeds = ttk.Combobox(controls, width=15, values=("代表車速疊圖",), state="disabled")
        self.speeds.current(0)
        self.speeds.bind("<<ComboboxSelected>>", self.select_speed)
        plot = ttk.Frame(self)
        plot.pack(fill="both", expand=True)
        plot.columnconfigure(0, weight=1)
        plot.rowconfigure(0, weight=1)
        self.chart = GGVChart(plot)
        self.surface = GGVSurface(plot)
        for canvas in (self.chart, self.surface):
            canvas.configure(width=350, height=280)
        self.instructions = tk.StringVar()
        ttk.Label(self, textvariable=self.instructions).pack(anchor="w", pady=(5, 0))
        ttk.Label(self, textvariable=self.note, style="Muted.TLabel").pack(anchor="w")
        self.select_view()

    def select_view(self, _event=None):
        surface = self.view.current() == 0
        if surface:
            self.chart.grid_remove()
            self.surface.grid(row=0, column=0, sticky="nsew")
            self.speeds.pack_forget()
            self.instructions.set("拖曳旋轉，雙擊重設視角。正 ax 為加速，負 ax 為煞車。")
        else:
            self.surface.grid_remove()
            self.chart.grid(row=0, column=0, sticky="nsew")
            self.speeds.pack(side="left")
            self.instructions.set("正 ax 為加速，負 ax 為煞車。ay 正值為左彎。")
        if self.chart.slices:
            self.select_speed()

    def invalidate(self):
        self.revision += 1
        self.chart.slices = []
        self.surface.slices = []
        self.chart.selection = 0
        self.speeds.configure(state="disabled")
        self.speeds.current(0)
        for canvas in (self.chart, self.surface):
            if canvas.winfo_ismapped():
                canvas.draw()
        self.note.set("設定已變更，按「計算 GGV」更新。")

    def calculate(self):
        if self.run_button.instate(["disabled"]):
            return
        try:
            settings = self.read_settings()
        except ValueError as exc:
            self.note.set(f"輸入有誤。{exc}")
            return
        self.invalidate()
        revision = self.revision
        self.run_button.state(["disabled"])
        self.note.set("正在計算車輛加速度上限…")

        def work():
            try:
                self.results.put(build_ggv(settings, speed_samples=13))
            except Exception as exc:
                self.results.put(exc)

        Thread(target=work, daemon=True).start()
        self.after(50, self.poll, revision)

    def poll(self, revision):
        try:
            result = self.results.get_nowait()
        except Empty:
            self.after(50, self.poll, revision)
            return
        self.run_button.state(["!disabled"])
        if self.revision != revision:
            return
        if isinstance(result, Exception):
            self.note.set(f"GGV 計算失敗。{result}")
            return
        self.chart.slices = result
        self.surface.slices = result
        self.speeds.configure(
            values=("代表車速疊圖", *(f"{part.speed_kmh:g} km/h" for part in result)), state="readonly",
        )
        self.speeds.current(0)
        self.select_speed()

    def select_speed(self, _event=None):
        if self.view.current() == 0:
            self.surface.draw()
            self.note.set(f"由 {len(self.surface.slices)} 個車速切片連成曲面，可切換到 2D 查看峰值。")
            return
        index = self.speeds.current()
        self.chart.selection = index
        self.chart.draw()
        if index <= 0:
            self.note.set("疊圖顯示五個代表車速，選擇單一車速可查看峰值。")
            return
        part = self.chart.slices[index - 1]
        self.note.set(
            f"左彎 {part.ay_g[-1]:.2f} g　右彎 {-part.ay_g[0]:.2f} g　"
            f"加速 {part.drive_g.max():.2f} g　煞車 {-part.brake_g.min():.2f} g"
        )
