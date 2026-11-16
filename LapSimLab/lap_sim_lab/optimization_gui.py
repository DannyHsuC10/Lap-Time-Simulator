"""Configuration and results for whole-lap parameter optimization."""

import math
import tkinter as tk
from tkinter import ttk

from .optimization import PARAMETERS, active_parameters
from .settings import MODEL_CHOICES


class OptimizationPage(ttk.Frame):
    def __init__(self, parent, baseline_vars, run, cancel, apply):
        super().__init__(parent, padding=(10, 8))
        self.result = None
        self.busy = False
        self.optimizing = False
        self.active = ()
        self.sources = tk.StringVar()
        self.summary = tk.StringVar(value="")
        self.evaluations = tk.StringVar(value="64")
        self.margins = {p.key: tk.StringVar(value=str(p.margin)) for p in PARAMETERS}

        ttk.Label(self, text="固定模型  ·  質點  /  煞車機械利益  /  固定空力係數  /  全圈迭代").pack(anchor="w")
        ttk.Label(self, textvariable=self.sources, style="Muted.TLabel").pack(anchor="w", pady=(2, 6))

        table = ttk.Frame(self)
        table.pack(fill="x")
        for column, title in enumerate(("參數", "基準值", "±M", "搜尋範圍", "最佳值", "單項圈速效益 (秒)")):
            table.columnconfigure(column, weight=1)
            ttk.Label(table, text=title, style="Section.TLabel").grid(row=0, column=column, sticky="w", padx=4)
        self.rows = {}
        self.entries = []
        self.outputs = {}
        for row, p in enumerate(PARAMETERS, 1):
            lower_upper, best, benefit = (tk.StringVar(value="—") for _ in range(3))
            baseline = ttk.Entry(table, width=8, textvariable=baseline_vars[p.key])
            margin = ttk.Entry(table, width=7, textvariable=self.margins[p.key])
            widgets = [ttk.Label(table, text=p.label), baseline, margin]
            widgets.extend(ttk.Label(table, textvariable=value) for value in (lower_upper, best, benefit))
            for column, widget in enumerate(widgets):
                widget.grid(row=row, column=column, sticky="w", padx=4, pady=1)
            self.rows[p.key] = widgets
            self.entries.extend((baseline, margin))
            self.outputs[p.key] = (lower_upper, best, benefit)
        self.baseline_hint = tk.StringVar(value="M = 0 固定該值。CL、CD 留空時從空力 JSON 讀取。")
        ttk.Label(self, textvariable=self.baseline_hint, style="Muted.TLabel").pack(anchor="w", pady=(4, 2))
        ttk.Label(self, text="每次只套用該列最佳值，其餘維持基準。正值代表縮短圈速，負值代表變慢。",
                  style="Muted.TLabel").pack(anchor="w")

        actions = ttk.Frame(self)
        actions.pack(fill="x", pady=(9, 6))
        ttk.Label(actions, text="搜尋評估上限").pack(side="left")
        limit = ttk.Entry(actions, width=5, textvariable=self.evaluations)
        limit.pack(side="left", padx=(4, 8))
        self.entries.append(limit)
        self.run_button = ttk.Button(actions, text="開始最佳化", style="Primary.TButton", command=run)
        self.run_button.pack(side="left")
        self.cancel_button = ttk.Button(actions, text="取消", command=cancel)
        self.cancel_button.pack(side="left", padx=5)
        self.apply_button = ttk.Button(actions, text="套用最佳設定並查看圈速", command=apply)
        self.apply_button.pack(side="right")
        ttk.Label(self, textvariable=self.summary, justify="left").pack(anchor="w", fill="x")

        for variable in (self.evaluations, *self.margins.values()):
            variable.trace_add("write", self.invalidate)
        self.refresh_states()

    def refresh_sources(self, models, track_name):
        tire = MODEL_CHOICES["tire_model"][1][models["tire_model"]]
        motor = MODEL_CHOICES["motor_model"][1][models["motor_model"]]
        self.sources.set(f"沿用左側選擇  ·  {tire}  /  {motor}  /  {track_name}")
        self.active = active_parameters(models["motor_model"])
        for widget in self.rows["dc_voltage"]:
            if models["motor_model"] == "lp03":
                widget.grid()
            else:
                widget.grid_remove()
        self.refresh_states()

    def read_options(self):
        def read(variable, label, parse=float):
            try:
                return parse(variable.get())
            except ValueError as error:
                raise ValueError(f"請檢查「{label}」的數值。") from error

        return dict(
            margins={p.key: read(self.margins[p.key], f"{p.label}的 M") for p in self.active},
            max_evaluations=read(self.evaluations, "搜尋評估上限", int),
        )

    def refresh_states(self):
        self.run_button.state(["disabled" if self.busy else "!disabled"])
        self.cancel_button.state(["!disabled" if self.optimizing else "disabled"])
        self.apply_button.state(["!disabled" if self.result is not None and not self.busy else "disabled"])
        for widget in self.entries:
            widget.state(["disabled" if self.busy else "!disabled"])

    def set_busy(self, busy, *, optimizing=False):
        self.busy = busy
        self.optimizing = optimizing
        self.refresh_states()

    def invalidate(self, *_):
        if self.result is None:
            return
        self.result = None
        for outputs in self.outputs.values():
            for output in outputs:
                output.set("—")
        self.summary.set("設定已變更，請重新最佳化。")
        self.baseline_hint.set("M = 0 固定該值。CL、CD 留空時從空力 JSON 讀取。")
        self.refresh_states()

    def show_result(self, result):
        self.result = result
        baseline = result.baseline_time
        for bound in result.ranges:
            key = bound.parameter.key
            lower_upper, best, benefit = self.outputs[key]
            lower_upper.set(f"{bound.lower:.5g} ~ {bound.upper:.5g}")
            best.set(f"{getattr(result.settings, key):.6g}")
            single_time = result.single_parameter_times.get(key)
            if single_time is None:
                benefit.set("無法比較")
            elif not math.isfinite(single_time):
                benefit.set("無法完賽")
            else:
                saved = round(baseline - single_time, 3)
                benefit.set(f"{saved:+.3f}" if saved else "0.000")
        lap_time = result.simulation.lap_time
        aero = {bound.parameter.key: bound.baseline for bound in result.ranges}
        self.baseline_hint.set(f"M = 0 固定該值。本次基準 CL = {aero['aero_cl0']:g}，CD = {aero['aero_cd0']:g}。")
        comparison = (f"基準 {baseline:.3f} 秒  →  最佳 {lap_time:.3f} 秒  ·  縮短 {baseline - lap_time:.3f} 秒"
                      if math.isfinite(baseline) else f"基準無法完賽  ·  最佳 {lap_time:.3f} 秒")
        self.summary.set(f"{comparison}  ·  搜尋評估 {result.evaluations} 組")
        self.refresh_states()
