"""Independent input-data and attitude analysis pages for the Tk workbench."""

import tkinter as tk
from tkinter import ttk

import numpy as np

from .analysis import AttitudeSweep, sweep_aero, sweep_tires
from .charts import LineChart


AERO_VIEWS = {
    "下壓力與阻力": ("N", (("下壓力", "F_down"), ("阻力", "F_drag"))),
    "前後軸下壓力": ("N", (("前軸", "front"), ("後軸", "rear"))),
    "CL 與 CD": ("無因次", (("CL", "CL"), ("CD", "CD"))),
}
TIRE_VIEWS = {
    "側向峰值 Fy": "fy_peak",
    "驅動上限 Fx": "fx_drive",
    "煞車下限 Fx": "fx_brake",
}
WHEEL_VIEWS = ("四輪合計", "FL", "FR", "RL", "RR")


class MotorPage(ttk.Frame):
    def __init__(self, master, read_motor_curve):
        super().__init__(master)
        self.read_motor_curve = read_motor_curve
        self.dirty = True
        header = ttk.Frame(self, padding=(10, 7))
        header.pack(fill="x")
        self.note = tk.StringVar(value="顯示左側選用的單馬達轉速／扭矩資料。")
        ttk.Label(header, textvariable=self.note, style="Muted.TLabel", wraplength=440).pack(side="left")
        ttk.Button(header, text="更新曲線", command=self.refresh).pack(side="right")
        self.chart = LineChart(self)
        self.chart.pack(fill="both", expand=True)

    def invalidate(self):
        self.dirty = True
        self.chart.set_lines()
        self.note.set("馬達資料已變更，請更新曲線。")

    def refresh(self):
        try:
            path, curve = self.read_motor_curve()
        except (OSError, ValueError) as exc:
            self.invalidate()
            self.note.set(f"無法讀取馬達曲線。{exc}")
            return
        self.dirty = False
        self.note.set(f"{path.name} · {len(curve.rpm)} 筆資料 · 峰值扭矩 {curve.torque.max():.2f} N·m")
        self.chart.set_lines(
            "單馬達扭矩", "N·m", "馬達轉速  [RPM]", curve.rpm, (("CSV 扭矩", curve.torque),)
        )


class SweepPage(ttk.Frame):
    def __init__(self, master, read_settings, *, tire=False):
        super().__init__(master)
        self.read_settings = read_settings
        self.tire = tire
        self.dirty = True
        self.result = None
        self.variables = {}
        self.note = tk.StringVar(value="可先使用預設條件，再調整參數並更新曲線。")

        form = ttk.Frame(self, padding=(10, 5))
        form.pack(fill="x")
        for col in range(3):
            form.columnconfigure(col, weight=1, uniform="sweep")
        fields = [
            ("extent_deg", "Roll 幅度 ± [°]", ()),
            ("ay_g", "橫向加速度需求 [g]", ()),
        ] if tire else [
            ("axis", "掃描軸", ("Roll", "Pitch")),
            ("extent_deg", "幅度 ± [°]", ()),
            ("fixed_deg", "固定另一角 [°]", ()),
            ("speed_kmh", "車速 [km/h]", ()),
            ("heave_mm", "Heave [mm]", ()),
        ]
        defaults = AttitudeSweep()
        for index, (key, label, choices) in enumerate(fields):
            variable = tk.StringVar(value=str(getattr(defaults, key)))
            self.variables[key] = variable
            self.control(form, index, label, variable, choices)
            variable.trace_add("write", self.invalidate)

        metrics = tuple(TIRE_VIEWS if tire else AERO_VIEWS)
        self.metric = tk.StringVar(value=metrics[0])
        self.control(form, len(fields), "曲線", self.metric, metrics).bind(
            "<<ComboboxSelected>>", lambda _event: self.render()
        )
        if tire:
            self.wheel = tk.StringVar(value=WHEEL_VIEWS[0])
            self.control(form, len(fields) + 1, "輪胎", self.wheel, WHEEL_VIEWS).bind(
                "<<ComboboxSelected>>", lambda _event: self.render()
            )

        header = ttk.Frame(self, padding=(10, 4))
        header.pack(fill="x")
        feedback = ttk.Label(header, textvariable=self.note, style="Muted.TLabel", wraplength=420)
        feedback.pack(side="left", fill="x", expand=True, padx=(0, 8))
        ttk.Button(header, text="更新曲線", command=self.refresh).pack(side="right")
        header.bind("<Configure>", lambda event: feedback.configure(wraplength=max(220, event.width - 115)))
        self.chart = LineChart(self)
        self.chart.pack(fill="both", expand=True)

    @staticmethod
    def control(form, index, label, variable, choices):
        cell = ttk.Frame(form)
        cell.grid(row=index // 3, column=index % 3, sticky="ew", padx=(0, 8), pady=2)
        ttk.Label(cell, text=label).pack(side="left", padx=(0, 5))
        if choices:
            widget = ttk.Combobox(cell, textvariable=variable, values=choices, state="readonly", width=10)
        else:
            widget = ttk.Entry(cell, textvariable=variable, width=7)
        widget.pack(side="left", fill="x", expand=True)
        return widget

    def invalidate(self, *_):
        if not self.dirty:
            self.chart.set_lines()
        self.dirty = True
        self.result = None
        self.note.set("條件已變更，請更新曲線。")

    def refresh(self):
        try:
            sweep = AttitudeSweep(**{
                key: variable.get() if key == "axis" else float(variable.get())
                for key, variable in self.variables.items()
            })
        except ValueError:
            self.invalidate()
            self.note.set("請確認掃描條件的數值格式。")
            return
        try:
            settings = self.read_settings()
            result = sweep_tires(settings, sweep) if self.tire else sweep_aero(settings, sweep)
        except (OSError, ValueError, ArithmeticError) as exc:
            self.invalidate()
            self.note.set(f"無法更新曲線。{exc}")
            return
        self.result = result
        self.sweep = sweep
        self.aero_enabled = settings.aero_enabled
        self.dirty = False
        self.render()

    def render(self):
        if self.result is None:
            return
        sweep = self.sweep
        metric = self.metric.get()
        if self.tire:
            xlabel = "Roll  [deg]"
            notes = ["固定靜態輪載，Roll 只用於換算外傾角"]
            result = self.result
            wheel = self.wheel.get()
            signal = getattr(result, TIRE_VIEWS[metric])
            if wheel == WHEEL_VIEWS[0]:
                values = signal.sum(axis=1)
            elif result.point_mass:
                values = np.zeros(len(result.angles))
                notes.append("質點模型無單輪資料")
            else:
                values = signal[:, WHEEL_VIEWS.index(wheel) - 1]
            if metric == "側向峰值 Fy":
                notes.append("Fx = 0 時的峰值" if result.point_mass else "Fx = 0，按前後軸偏航力矩平衡取側向上限")
            else:
                notes.append(f"固定 ay = {sweep.ay_g:g} g 的剩餘抓地")
                if failures := np.count_nonzero(~result.feasible):
                    notes.append(f"{failures} 點的指定 Fy 無法滿足，Fx 顯示 0")
            self.chart.set_lines(f"{wheel} · {metric}", "N", xlabel, result.angles, ((metric, values),))
        else:
            xlabel = f"{sweep.axis}  [deg]"
            other = "Pitch" if sweep.axis == "Roll" else "Roll"
            notes = [f"{other} 固定 {sweep.fixed_deg:g}°"]
            if not self.aero_enabled:
                notes.append("空力已關閉")
            angles, signals = self.result
            unit, curves = AERO_VIEWS[metric]
            series = tuple((label, signals[key]) for label, key in curves)
            self.chart.set_lines(metric, unit, xlabel, angles, series)
        self.note.set(" · ".join(notes))
