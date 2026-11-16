"""Configuration and results for whole-lap parameter optimization."""

import math
import tkinter as tk
from tkinter import ttk

from .aero import AERO_COEFFICIENT_KEYS, read_aero_coefficients
from .optimization import DEFAULT_EQUIVALENT_KEYS, PARAMETERS, active_parameters
from .settings import MODEL_CHOICES, PERCENTAGE_FIELDS, parse_percentage


class OptimizationPage(ttk.Frame):
    def __init__(self, parent, baseline_vars, run, cancel, apply):
        super().__init__(parent, padding=(10, 8))
        self.result = None
        self.busy = False
        self.optimizing = False
        self.active = ()
        self.sources = tk.StringVar()
        self.summary = tk.StringVar(value="")
        self.equivalent_summary = tk.StringVar()
        self.aero_note = tk.StringVar()
        self.aero_baseline = None
        self.aero_baseline_vars = {key: tk.StringVar(value="—") for key in AERO_COEFFICIENT_KEYS}
        self.evaluations = tk.StringVar(value="64")
        self.margins = {p.key: tk.StringVar() for p in PARAMETERS}
        self.equivalent_enabled = {p.key: tk.BooleanVar(value=p.key in DEFAULT_EQUIVALENT_KEYS) for p in PARAMETERS}
        self.edited_margins = set()
        self.updating_default_margin = False

        ttk.Label(self, text="固定模型  ·  質點  /  煞車機械利益  /  固定空力係數  /  全圈迭代").pack(anchor="w")
        ttk.Label(self, textvariable=self.sources, style="Muted.TLabel").pack(anchor="w", pady=(2, 6))

        table = ttk.Frame(self)
        table.pack(fill="x")
        titles = ("參數", "基準值", "±M", "搜尋範圍", "最佳值", "單項效益 (秒)", "等效值（調整量）")
        for column, title in enumerate(titles):
            table.columnconfigure(column, weight=1)
            ttk.Label(table, text=title, style="Section.TLabel").grid(row=0, column=column, sticky="w", padx=4)
        self.rows = {}
        self.option_widgets = []
        self.outputs = {}
        for row, p in enumerate(PARAMETERS, 1):
            outputs = tuple(tk.StringVar(value="—") for _ in range(4))
            baseline_var = self.aero_baseline_vars[p.key] if p.key in self.aero_baseline_vars else baseline_vars[p.key]
            baseline = ttk.Label(table, textvariable=baseline_var)
            margin = ttk.Entry(table, width=7, textvariable=self.margins[p.key])
            self.update_default_margin(p, baseline_var)
            baseline_var.trace_add("write", lambda *_args, parameter=p, source=baseline_var:
                                   self.update_default_margin(parameter, source))
            self.margins[p.key].trace_add("write", lambda *_args, key=p.key: self.margin_changed(key))
            label = f"{p.label} (%)" if p.key in PERCENTAGE_FIELDS else p.label
            widgets = [ttk.Label(table, text=label), baseline, margin]
            widgets.extend(ttk.Label(table, textvariable=value) for value in outputs[:-1])
            equivalent_cell = ttk.Frame(table)
            enabled = self.equivalent_enabled[p.key]
            checkbox = ttk.Checkbutton(equivalent_cell, variable=enabled)
            checkbox.pack(side="left", padx=(0, 4))
            enabled.trace_add("write", self.invalidate)
            ttk.Label(equivalent_cell, textvariable=outputs[-1]).pack(side="left")
            widgets.append(equivalent_cell)
            for column, widget in enumerate(widgets):
                widget.grid(row=row, column=column, sticky="w", padx=4, pady=1)
            self.rows[p.key] = widgets
            self.option_widgets.extend((margin, checkbox))
            self.outputs[p.key] = outputs
        aero_hint = ttk.Label(self, textvariable=self.aero_note, style="Muted.TLabel")
        aero_hint.pack(fill="x", pady=(4, 2))
        aero_hint.bind("<Configure>", lambda event: aero_hint.configure(wraplength=event.width))
        ttk.Label(self, text="輪胎使用率與傳動效率的 M 及調整量以百分點表示。",
                  style="Muted.TLabel").pack(anchor="w")
        ttk.Label(self, text="每次只套用該列最佳值，其餘維持基準。正值代表縮短圈速，負值代表變慢。",
                  style="Muted.TLabel").pack(anchor="w")
        ttk.Label(self, text="在「等效值」欄勾選要做等效比較的參數。", style="Muted.TLabel").pack(anchor="w")
        equivalent_hint = ttk.Label(self, textvariable=self.equivalent_summary, style="Muted.TLabel")
        equivalent_hint.pack(fill="x", pady=(3, 0))
        equivalent_hint.bind("<Configure>", lambda event: equivalent_hint.configure(wraplength=event.width))

        actions = ttk.Frame(self)
        actions.pack(fill="x", pady=(9, 6))
        ttk.Label(actions, text="搜尋評估上限").pack(side="left")
        limit = ttk.Entry(actions, width=5, textvariable=self.evaluations)
        limit.pack(side="left", padx=(4, 8))
        self.option_widgets.append(limit)
        self.run_button = ttk.Button(actions, text="開始敏感性分析", style="Primary.TButton", command=run)
        self.run_button.pack(side="left")
        self.cancel_button = ttk.Button(actions, text="取消", command=cancel)
        self.cancel_button.pack(side="left", padx=5)
        self.apply_button = ttk.Button(actions, text="套用最佳設定並查看圈速", command=apply)
        self.apply_button.pack(side="right")
        ttk.Label(self, textvariable=self.summary, justify="left").pack(anchor="w", fill="x")

        self.evaluations.trace_add("write", self.invalidate)
        self.refresh_states()

    def update_default_margin(self, parameter, source):
        key = parameter.key
        if key in self.edited_margins:
            return
        try:
            baseline = float(source.get())
        except ValueError:
            baseline = math.nan
        margin = parameter.default_margin(baseline)
        scale = 100 if key in PERCENTAGE_FIELDS else 1
        text = f"{margin * scale:.15g}" if math.isfinite(margin) else ""
        if self.margins[key].get() == text:
            return
        self.updating_default_margin = True
        try:
            self.margins[key].set(text)
        finally:
            self.updating_default_margin = False

    def margin_changed(self, key):
        if not self.updating_default_margin:
            self.edited_margins.add(key)
            self.invalidate()

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

    def refresh_aero_baseline(self, path):
        try:
            coefficients = read_aero_coefficients(path)
        except (OSError, ValueError) as error:
            coefficients = None
            self.aero_note.set(f"無法讀取 CL、CD 基準值。{error}")
        else:
            self.aero_note.set(f"CL、CD 基準讀自 {path.name}。M = 0 固定該值。")
        if coefficients != self.aero_baseline:
            self.invalidate()
            self.aero_baseline = coefficients
        for key, variable in self.aero_baseline_vars.items():
            variable.set(str(coefficients[key]) if coefficients is not None else "—")

    def read_options(self):
        def read(variable, label, parse=float):
            try:
                return parse(variable.get())
            except ValueError as error:
                raise ValueError(f"請檢查「{label}」的數值。") from error

        return dict(
            margins={
                p.key: read(self.margins[p.key], f"{p.label}的 M",
                            parse_percentage if p.key in PERCENTAGE_FIELDS else float)
                for p in self.active if p.key in self.edited_margins
            },
            max_evaluations=read(self.evaluations, "搜尋評估上限", int),
            equivalent_keys=tuple(p.key for p in self.active if self.equivalent_enabled[p.key].get()),
        )

    def refresh_states(self):
        self.run_button.state(["disabled" if self.busy else "!disabled"])
        self.cancel_button.state(["!disabled" if self.optimizing else "disabled"])
        self.apply_button.state(["!disabled" if self.result is not None and not self.busy else "disabled"])
        for widget in self.option_widgets:
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
        self.summary.set("設定已變更，請重新分析。")
        self.equivalent_summary.set("")
        self.refresh_states()

    def show_result(self, result):
        self.result = result
        baseline = result.baseline_time
        for bound in result.ranges:
            key = bound.parameter.key
            if key in self.aero_baseline_vars:
                self.aero_baseline_vars[key].set(str(bound.baseline))
            lower_upper, best, benefit, equivalent = self.outputs[key]
            scale, unit = (100, "%") if key in PERCENTAGE_FIELDS else (1, "")
            lower_upper.set(f"{bound.lower * scale:.5g}{unit} ~ {bound.upper * scale:.5g}{unit}")
            best.set(f"{result.parameter_value(key) * scale:.6g}{unit}")
            single_time = result.single_parameter_times.get(key)
            if single_time is None:
                benefit.set("無法比較")
            elif not math.isfinite(single_time):
                benefit.set("無法完賽")
            else:
                saved = round(baseline - single_time, 3)
                benefit.set(f"{saved:+.3f}" if saved else "0.000")
            change = result.equivalent_changes.get(key)
            if key not in result.equivalent_keys:
                equivalent.set("未勾選")
            elif change is not None:
                equivalent.set(f"{change.value * scale:.8g}{unit} ({(change.value - bound.baseline) * scale:+.5g})\n"
                               f"縮短 {baseline - change.lap_time:.5g} 秒")
            else:
                equivalent.set("最佳值未達標" if result.equivalent_gain is not None else "—")
        if result.equivalent_gain is not None:
            self.equivalent_summary.set(f"等效目標為單獨縮短至少 {result.equivalent_gain:.5g} 秒，"
                                        "取勾選項目中最小的正向單項效益。"
                                        "括號為相對基準的調整量。")
        elif not result.equivalent_keys:
            self.equivalent_summary.set("未勾選等效比較項目。")
        elif math.isfinite(baseline):
            self.equivalent_summary.set("沒有可作為等效目標的正向單項效益。")
        else:
            self.equivalent_summary.set("基準無法完賽，無法計算等效目標。")
        lap_time = result.simulation.lap_time
        comparison = (f"基準 {baseline:.3f} 秒  →  最佳 {lap_time:.3f} 秒  ·  縮短 {baseline - lap_time:.3f} 秒"
                      if math.isfinite(baseline) else f"基準無法完賽  ·  最佳 {lap_time:.3f} 秒")
        self.summary.set(f"{comparison}  ·  搜尋評估 {result.evaluations} 組")
        self.refresh_states()
