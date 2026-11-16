"""Two-pane Tk workbench for configuring and reviewing one LapSim run."""

from pathlib import Path
from queue import Empty, SimpleQueue
import threading
import tkinter as tk
from tkinter import filedialog, ttk

import numpy as np

from .car import Car
from .analysis_gui import MotorPage, SweepPage
from .charts import FONT, ResultChart, SpeedMap
from .ggv_gui import GGVPage
from .model_help import ModelHelpPage, describe_models
from .planner import GRAVITY
from .settings import MODEL_CHOICES, Settings
from .settings_profile import load_profile, save_profile
from .signal_view import SignalView
from .simulation import simulate, write_csv
from .tire import build_tire_model
from .timeout_recovery import describe_model_changes, lower_model_choices, timeout_suggestions


FILE_FIELDS = {
    "track_path": ("賽道分段", ".csv"),
    "tire_path": ("輪胎係數", ".json"),
    "ellipse_path": ("摩擦橢圓 JSON", ".json"),
    "motor_path": ("馬達曲線", ".csv"),
    "aero_path": ("空力係數", ".json"),
    "tire_table_path": ("輪胎力資料表", ".npz"),
    "lp03_path": ("LP03 電壓包絡", ".csv"),
}
OPTIONAL_NUMBERS = {"aero_cl0", "aero_cd0"}
SETTING_TOGGLES = ("aero_enabled", "yaw_correction", "closed_lap", "braking_lookahead")
PROFILE_FILETYPES = [("LapSimLab 設定 JSON", "*.json")]

SURFACE = "#f1f5f9"
WHITE = "#ffffff"
INK = "#17253a"
MUTED = "#52657b"


class LapSimLab(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("LapSimLab — 圈速模擬")
        self.geometry("1280x790")
        self.minsize(1040, 665)
        self.configure(bg=SURFACE)
        self.default = Settings()
        self.profile_path = None
        self.applying_inputs = False
        self.paths = {key: getattr(self.default, key) for key in FILE_FIELDS}
        self.file_vars = {key: tk.StringVar(value=path.name) for key, path in self.paths.items()}
        self.entries = {}
        self.field_labels = {}
        self.input_rows = {}
        self.input_groups = {}
        self.active_inputs = set()
        self.model_vars = {}
        self.aero_enabled = tk.BooleanVar(value=self.default.aero_enabled)
        self.yaw_correction = tk.BooleanVar(value=self.default.yaw_correction)
        self.closed_lap = tk.BooleanVar(value=self.default.closed_lap)
        self.braking_lookahead = tk.BooleanVar(value=self.default.braking_lookahead)
        self.status = tk.StringVar(value="已載入內附資料，可直接計算。")
        self.result_title = tk.StringVar(value="結果預覽")
        self.result = None
        self.result_revision = None
        self.revision = 0
        self.busy = False
        self.timed_out_settings = None
        self.events = SimpleQueue()

        self.configure_styles()
        self.build_layout()
        self.refresh_input_rows()
        self.refresh_model_help()
        self.aero_enabled.trace_add("write", self.inputs_changed)
        self.yaw_correction.trace_add("write", self.inputs_changed)
        self.closed_lap.trace_add("write", self.inputs_changed)
        self.braking_lookahead.trace_add("write", self.inputs_changed)
        self.bind("<F5>", lambda _event: self.calculate())

    def configure_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure(".", font=(FONT, 10), foreground=INK, background=SURFACE)
        style.configure("TNotebook", borderwidth=0)
        style.configure("TNotebook.Tab", padding=(12, 7))
        style.map("TNotebook.Tab", background=[("selected", WHITE)])
        style.configure("Panel.TFrame", background=WHITE)
        style.configure("Panel.TLabel", background=WHITE)
        style.configure("Panel.TCheckbutton", background=WHITE)
        style.map("Panel.TCheckbutton", background=[("active", WHITE)])
        style.configure("Header.TLabel", font=(FONT, 22, "bold"), foreground=INK)
        style.configure("Section.TLabel", font=(FONT, 12, "bold"), foreground=INK)
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Hint.TLabel", background=WHITE, foreground=MUTED, font=(FONT, 9))
        style.configure("Metric.TLabel", background=WHITE, font=(FONT, 17, "bold"), foreground="#16385e")
        style.configure("Primary.TButton", background="#1d4ed8", foreground=WHITE,
                        bordercolor="#1d4ed8", padding=(10, 7), font=(FONT, 11, "bold"))
        style.map("Primary.TButton", background=[("disabled", "#94a3b8"), ("active", "#1e40af")],
                  foreground=[("disabled", WHITE)])

    def build_layout(self):
        header = ttk.Frame(self, padding=(19, 12, 19, 8))
        header.pack(fill="x")
        ttk.Label(header, text="LapSimLab", style="Header.TLabel").pack(side="left")
        ttk.Label(header, text="FSAE  圈速模擬", style="Muted.TLabel").pack(side="left", padx=14)
        ttk.Button(header, text="儲存設定", command=self.save_settings).pack(side="right")
        ttk.Button(header, text="載入設定", command=self.load_settings).pack(side="right", padx=(0, 8))

        body = ttk.Frame(self, padding=(19, 2, 19, 0))
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        controls = ttk.Frame(body)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 17))
        ttk.Label(controls, text="模擬輸入", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        tabs = ttk.Notebook(controls)
        tabs.pack(fill="both", expand=True)

        self.build_inputs(tabs)

        buttons = ttk.Frame(controls)
        buttons.pack(fill="x", pady=(11, 0))
        self.run_button = ttk.Button(buttons, text="計算  F5", style="Primary.TButton", command=self.calculate)
        self.run_button.pack(side="left", fill="x", expand=True)
        ttk.Button(buttons, text="恢復預設", command=self.reset).pack(side="left", padx=(8, 0))

        self.result_pages = ttk.Notebook(body)
        self.result_pages.grid(row=0, column=1, sticky="nsew")
        results = ttk.Frame(self.result_pages, padding=(0, 7, 0, 0))
        self.result_pages.add(results, text="圈速結果")
        result_bar = ttk.Frame(results)
        result_bar.pack(fill="x", pady=(0, 8))
        ttk.Label(result_bar, textvariable=self.result_title, style="Section.TLabel").pack(side="left")
        self.export_button = ttk.Button(result_bar, text="匯出完整 CSV", command=self.export, state="disabled")
        self.export_button.pack(side="right")

        summary = ttk.Frame(results)
        summary.pack(fill="x", pady=(0, 10))
        self.metrics = []
        for index, label in enumerate(("預估圈速", "取樣路徑長度", "最高車速", "最大橫向加速度")):
            summary.columnconfigure(index, weight=1, uniform="metric")
            card = ttk.Frame(summary, style="Panel.TFrame", padding=(12, 7))
            card.grid(row=0, column=index, sticky="ew", padx=(0, 6 if index < 3 else 0))
            ttk.Label(card, text=label, style="Hint.TLabel").pack(anchor="w")
            value = tk.StringVar(value="—")
            self.metrics.append(value)
            ttk.Label(card, textvariable=value, style="Metric.TLabel").pack(anchor="w")

        views = ttk.Notebook(results)
        views.pack(fill="both", expand=True)
        self.charts = []
        for kind, label in (
            ("map", "賽道速度圖"), ("speed", "速度曲線"), ("accel", "加速度"),
            ("loads", "四輪輪荷"), ("attitude", "車身姿態"), ("aero", "空力"),
        ):
            chart = SpeedMap(views) if kind == "map" else ResultChart(views, kind)
            views.add(chart, text=label)
            self.charts.append(chart)

        def read_vehicle():
            return self.read_settings(vehicle_only=True)

        self.aero_page = SweepPage(self.result_pages, read_vehicle)
        self.motor_page = MotorPage(self.result_pages, self.read_motor_curve)
        self.tire_page = SweepPage(
            self.result_pages, lambda: self.read_settings(vehicle_only=True, include_aero=False), tire=True,
        )
        self.ggv_page = GGVPage(self.result_pages, self.read_settings)
        self.signal_page = SignalView(self.result_pages, self.export)
        self.model_help = ModelHelpPage(self.result_pages)
        for page, label in (
            (self.signal_page, "訊號檢視"),
            (self.aero_page, "空力掃描"), (self.motor_page, "馬達曲線"), (self.tire_page, "輪胎力掃描"),
            (self.ggv_page, "GGV"), (self.model_help, "模型說明"),
        ):
            self.result_pages.add(page, text=label)
        self.result_pages.bind("<<NotebookTabChanged>>", self.refresh_analysis)

        footer = ttk.Frame(self, padding=(19, 9, 19, 11))
        footer.pack(fill="x")
        status_row = ttk.Frame(footer)
        status_row.pack(fill="x")
        self.retry_button = ttk.Button(status_row, text="使用低階模型重算", command=self.retry_lower_models)
        self.progress = ttk.Progressbar(status_row, mode="indeterminate", length=140)
        self.status_label = ttk.Label(status_row, textvariable=self.status, style="Muted.TLabel", justify="left")
        self.status_label.pack(side="left", fill="x", expand=True)
        self.status_label.bind("<Configure>", lambda event: self.status_label.configure(wraplength=event.width))
        self.model_changes = ttk.Label(footer, style="Muted.TLabel", justify="left")
        self.model_changes.bind("<Configure>", lambda event: self.model_changes.configure(wraplength=event.width))

    def refresh_analysis(self, _event=None):
        page = self.nametowidget(self.result_pages.select())
        if page is self.model_help:
            self.refresh_model_help()
        elif page in (self.aero_page, self.motor_page, self.tire_page) and page.dirty:
            page.refresh()

    def refresh_model_help(self):
        options = self.selected_models()
        sections = describe_models(
            options,
            {key: var.get().strip() or "未填" for key, var in self.entries.items()},
            {key: path.name for key, path in self.paths.items()},
            **self.active_toggles(),
        )
        try:
            self.read_settings()
        except ValueError as error:
            status = f"目前設定有待修正。{error}"
        else:
            status = "說明會隨左側設定更新。"
            if self.busy:
                status += " 計算中的工作使用按下計算時的設定。"
            elif self.result is not None and self.result_revision != self.revision:
                status += " 已顯示的圈速使用上次計算的設定。"
        self.model_help.show(sections, status)

    def build_inputs(self, tabs):
        tire = self.input_tab(tabs, "輪胎")
        self.model_row(tire, "tire_model")
        self.file_row(tire, "tire_path")
        self.file_row(tire, "ellipse_path")
        self.file_row(tire, "tire_table_path")
        self.number_row(tire, "tire_radius", "自由半徑", "m")
        self.model_row(tire, "rolling_model")
        self.number_row(tire, "rolling_coefficient", "固定滾阻係數")
        self.model_hint(tire, "rolling_model_hint", "目前輪胎未提供滾阻資料，滾阻為 0。")

        suspension = self.input_tab(tabs, "懸吊與車體")
        self.model_row(suspension, "load_model")
        self.number_row(suspension, "mass", "總質量（含車手）", "kg")
        self.number_row(suspension, "tire_utilization", "輪胎使用率", "1 = 100%")
        self.number_row(suspension, "wheelbase", "軸距", "m")
        self.number_row(suspension, "cg_from_front", "重心距前軸", "m")
        self.number_row(suspension, "cg_height", "重心高度", "m")
        self.number_row(suspension, "front_track", "前輪距", "m")
        self.number_row(suspension, "rear_track", "後輪距", "m")
        self.number_row(suspension, "front_heave_stiffness", "前軸升沉剛性", "N/m")
        self.number_row(suspension, "front_roll_stiffness", "前軸側傾剛性（每輪）", "N/m")
        self.number_row(suspension, "rear_stiffness_ratio", "後／前剛性比")
        self.number_row(suspension, "tire_stiffness", "輪胎垂直剛性", "N/m")
        self.toggle_row(suspension, "yaw_correction", "啟用偏航慣量修正")
        self.number_row(suspension, "yaw_inertia", "偏航慣量", "kg·m²")

        motor = self.input_tab(tabs, "馬達")
        self.model_row(motor, "motor_model")
        self.file_row(motor, "motor_path")
        self.file_row(motor, "lp03_path")
        self.number_row(motor, "dc_voltage", "直流電壓", "V")
        self.number_row(motor, "gear_ratio", "固定減速比")
        self.number_row(motor, "gear_efficiency", "傳動效率", "0–1")
        self.number_row(motor, "front_torque_scale", "前輪限扭比例", "0–1")
        self.number_row(motor, "rear_torque_scale", "後輪限扭比例", "0–1")
        brake = self.input_group(motor, "煞車")
        self.model_row(brake, "brake_model")
        self.number_row(brake, "front_brake_bias", "前軸煞車比例", "0–1")
        self.number_row(brake, "pedal_force", "踏板力", "N")
        self.number_row(brake, "brake_leverage", "機械利益")
        self.number_row(brake, "pedal_ratio", "踏板比")
        self.number_row(brake, "master_diameter", "總泵直徑", "m")
        self.number_row(brake, "front_caliper_diameter", "前輪卡鉗活塞直徑", "m")
        self.number_row(brake, "rear_caliper_diameter", "後輪卡鉗活塞直徑", "m")
        self.number_row(brake, "front_pistons", "前輪活塞數（每輪）", "個")
        self.number_row(brake, "rear_pistons", "後輪活塞數（每輪）", "個")
        self.number_row(brake, "disc_diameter", "有效碟盤直徑", "m")
        self.number_row(brake, "pad_mu", "來令片摩擦係數")

        aero = self.input_tab(tabs, "空力")
        self.toggle_row(aero, "aero_enabled", "啟用空力")
        self.model_row(aero, "aero_model")
        self.file_row(aero, "aero_path")
        self.number_row(aero, "aero_cl0", "基準 CL", "留空用 JSON")
        self.number_row(aero, "aero_cd0", "基準 CD", "留空用 JSON")

        track = self.input_tab(tabs, "賽道")
        self.model_row(track, "curvature_model")
        self.file_row(track, "track_path")
        self.number_row(track, "curvature_sigma_m", "曲率平滑尺度", "m")
        self.number_row(track, "track_width", "賽道寬度", "m")
        self.number_row(track, "step_m", "取樣間距", "m")
        solver = self.input_group(track, "圈速求解")
        self.model_row(solver, "solver")
        self.number_row(solver, "max_speed", "速度上限", "m/s")
        self.number_row(solver, "time_limit_s", "計算時間上限", "秒，0＝不限")
        self.number_row(solver, "safety_factor", "側向安全係數", "0–1")
        self.number_row(solver, "passes", "最大掃描次數", "次")
        self.toggle_row(solver, "closed_lap", "封閉圈首尾等速")
        self.number_row(solver, "initial_speed", "起始車速", "m/s")
        self.toggle_row(solver, "braking_lookahead", "使用單次煞車前瞻")

    def input_tab(self, notebook, title):
        tab = ttk.Frame(notebook, style="Panel.TFrame")
        notebook.add(tab, text=title)
        canvas = tk.Canvas(tab, background=WHITE, highlightthickness=0, width=330, height=1)
        scrollbar = ttk.Scrollbar(tab, command=canvas.yview)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)
        panel = ttk.Frame(canvas, style="Panel.TFrame", padding=(8, 6))
        window = canvas.create_window(0, 0, anchor="nw", window=panel)
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        panel.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))

        def scroll(event):
            target = event.widget
            while target is not None and target != tab:
                target = target.master
            if target == tab and not isinstance(event.widget, ttk.Combobox):
                canvas.yview_scroll(-int(event.delta / 120), "units")

        tab.bind("<MouseWheel>", scroll)
        panel.input_scroll_tag = str(tab)
        self.bind_input_scroll(canvas, tag=str(tab))
        self.bind_input_scroll(panel)
        return panel

    def input_group(self, panel, title):
        group = ttk.LabelFrame(panel, text=title, padding=(6, 3))
        self.input_groups[panel] = group
        self.bind_input_scroll(group)
        return group

    def input_row(self, panel, key, label=""):
        row = ttk.Frame(panel, style="Panel.TFrame")
        row.columnconfigure(1, weight=1)
        self.input_rows[key] = row
        if label:
            ttk.Label(row, text=label, style="Panel.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 6))
        return row

    def model_hint(self, panel, key, text):
        row = self.input_row(panel, key)
        label = ttk.Label(row, text=text, style="Hint.TLabel", wraplength=290)
        label.grid(row=0, column=0, columnspan=3, sticky="w")
        self.bind_input_scroll(label)

    def bind_input_scroll(self, widget, *, tag=None):
        parent = widget
        while tag is None and parent is not None:
            tag = getattr(parent, "input_scroll_tag", None)
            parent = parent.master
        if tag is not None:
            widget.bindtags((*widget.bindtags(), tag))

    def number_row(self, panel, key, label, unit=""):
        default = getattr(self.default, key)
        var = tk.StringVar(value="" if default is None else str(default))
        var.trace_add("write", self.inputs_changed)
        self.entries[key] = var
        self.field_labels[key] = label
        row = self.input_row(panel, key, label)
        entry = ttk.Entry(row, textvariable=var, width=9)
        entry.grid(row=0, column=1, sticky="ew")
        ttk.Label(row, text=unit, style="Hint.TLabel").grid(row=0, column=2, sticky="w", padx=(5, 0))
        for widget in (row, *row.winfo_children()):
            self.bind_input_scroll(widget)

    def model_row(self, panel, key):
        label, choices = MODEL_CHOICES[key]
        row = self.input_row(panel, key, label)
        variable = tk.StringVar(value=choices[getattr(self.default, key)])
        self.model_vars[key] = variable
        ttk.Combobox(row, textvariable=variable, values=tuple(choices.values()), width=17,
                     state="readonly").grid(row=0, column=1, sticky="ew")
        variable.trace_add("write", self.inputs_changed)
        for widget in (row, *row.winfo_children()):
            self.bind_input_scroll(widget)

    def file_row(self, panel, key):
        label, suffix = FILE_FIELDS[key]
        row = self.input_row(panel, key, label)
        ttk.Entry(row, textvariable=self.file_vars[key], state="readonly", width=12).grid(
            row=0, column=1, sticky="ew",
        )
        ttk.Button(row, text="選擇", width=4, command=lambda: self.choose_file(key, suffix)).grid(
            row=0, column=2, sticky="e", padx=(5, 0),
        )
        for widget in (row, *row.winfo_children()):
            self.bind_input_scroll(widget)

    def toggle_row(self, panel, key, label):
        row = self.input_row(panel, key)
        toggle = ttk.Checkbutton(row, text=label, variable=getattr(self, key), style="Panel.TCheckbutton")
        toggle.grid(row=0, column=0, sticky="w")
        for widget in (row, toggle):
            self.bind_input_scroll(widget)

    def refresh_input_rows(self):
        models = self.selected_models()
        tire = models["tire_model"]
        four_wheels = models["load_model"] != "point_mass"
        lp03 = models["motor_model"] == "lp03"
        hydraulic = models["brake_model"] == "hydraulic"
        lap = models["solver"] == "lap"
        aero = self.aero_enabled.get()
        fixed_aero = aero and models["aero_model"] == "quadratic"
        missing_rolling_table = models["rolling_model"] == "table" and tire != "table"
        conditional = {
            "tire_path": tire == "superellipse", "ellipse_path": tire == "ellipse",
            "tire_table_path": tire == "table",
            "tire_stiffness": four_wheels, "rolling_coefficient": models["rolling_model"] == "constant",
            "rolling_model_hint": missing_rolling_table,
            "wheelbase": four_wheels, "front_track": four_wheels, "rear_track": four_wheels,
            "cg_from_front": four_wheels, "cg_height": four_wheels,
            "front_heave_stiffness": four_wheels, "front_roll_stiffness": four_wheels,
            "rear_stiffness_ratio": four_wheels,
            "yaw_correction": four_wheels,
            "yaw_inertia": four_wheels and self.yaw_correction.get(),
            "motor_path": not lp03, "lp03_path": lp03, "dc_voltage": lp03,
            "brake_leverage": not hydraulic,
            **dict.fromkeys(("pedal_ratio", "master_diameter", "front_caliper_diameter", "rear_caliper_diameter",
                             "front_pistons", "rear_pistons", "disc_diameter", "pad_mu"), hydraulic),
            "aero_model": aero, "aero_path": aero, "aero_cl0": fixed_aero, "aero_cd0": fixed_aero,
            "curvature_sigma_m": models["curvature_model"] == "smooth", "passes": lap,
            "closed_lap": lap, "initial_speed": not lap, "braking_lookahead": not lap,
        }
        active = {key for key in self.input_rows if conditional.get(key, True)}
        changed = active ^ self.active_inputs
        if not changed:
            return
        self.active_inputs = active
        rows = {}
        for key, row in self.input_rows.items():
            rows.setdefault(row.master, []).append((key, row))
        for panel, siblings in rows.items():
            if not any(key in changed for key, _ in siblings):
                continue
            for _, row in siblings:
                row.pack_forget()
            group = self.input_groups.get(panel)
            if group is not None:
                group.pack_forget()
            for key, row in siblings:
                if key in active:
                    row.pack(fill="x", pady=2)
            if panel in self.input_groups:
                group.pack(fill="x", pady=(6, 0))

    def inputs_changed(self, *_):
        if self.applying_inputs:
            return
        self.clear_timeout_recovery()
        self.refresh_input_rows()
        self.revision += 1
        self.aero_page.invalidate()
        self.tire_page.invalidate()
        self.ggv_page.invalidate()
        self.motor_page.invalidate()
        self.export_button.state(["disabled"])
        self.signal_page.mark_stale()
        if self.result is not None:
            self.result_title.set("上次計算結果")
        if not self.busy:
            self.status.set("設定已變更，按「計算」更新結果。")
        if self.result_pages.select() == str(self.model_help):
            self.refresh_model_help()

    def choose_file(self, key, suffix):
        path = filedialog.askopenfilename(
            parent=self, initialdir=self.paths[key].parent,
            filetypes=[(suffix[1:].upper(), f"*{suffix}"), ("所有檔案", "*.*")],
        )
        if path:
            self.paths[key] = Path(path)
            self.file_vars[key].set(self.paths[key].name)
            self.inputs_changed()
            if key in {"motor_path", "lp03_path"}:
                if self.result_pages.select() == str(self.motor_page):
                    self.motor_page.refresh()

    def save_settings(self):
        selected = filedialog.asksaveasfilename(
            parent=self, title="儲存 LapSimLab 設定", defaultextension=".json", filetypes=PROFILE_FILETYPES,
            initialdir=self.profile_path.parent if self.profile_path else None,
            initialfile=self.profile_path.name if self.profile_path else "lapsimlab_settings.json",
        )
        if not selected:
            return
        path = Path(selected)
        snapshot = dict(self.paths, **self.selected_models())
        snapshot.update({key: getattr(self, key).get() for key in SETTING_TOGGLES})
        snapshot.update({key: entry.get() for key, entry in self.entries.items()})
        try:
            save_profile(path, snapshot)
        except (OSError, ValueError) as exc:
            self.status.set(f"無法儲存設定。{exc}")
        else:
            self.profile_path = path
            self.status.set(f"已儲存設定至 {path}")

    def load_settings(self):
        selected = filedialog.askopenfilename(
            parent=self, title="載入 LapSimLab 設定", filetypes=PROFILE_FILETYPES,
            initialdir=self.profile_path.parent if self.profile_path else None,
        )
        if not selected:
            return
        path = Path(selected)
        try:
            snapshot = load_profile(path)
        except (OSError, ValueError) as exc:
            self.status.set(f"無法載入設定。{exc}")
            return
        self.apply_inputs(snapshot)
        self.profile_path = path
        missing = []
        for input_path in self.paths.values():
            try:
                available = input_path.is_file()
            except OSError:
                available = False
            if not available:
                missing.append(input_path.name)
        detail = f"。請重新選擇找不到的資料檔：{'、'.join(missing)}。" if missing else "。按「計算」更新結果。"
        self.status.set(f"已載入設定 {path.name}{detail}")

    def reset(self):
        self.apply_inputs({})

    def apply_inputs(self, snapshot):
        self.applying_inputs = True
        try:
            for key, var in self.entries.items():
                value = snapshot.get(key, getattr(self.default, key))
                var.set("" if value is None else str(value))
            for key in SETTING_TOGGLES:
                getattr(self, key).set(snapshot.get(key, getattr(self.default, key)))
            for key, var in self.model_vars.items():
                value = snapshot.get(key, getattr(self.default, key))
                var.set(MODEL_CHOICES[key][1][value])
            for key, var in self.file_vars.items():
                self.paths[key] = snapshot.get(key, getattr(self.default, key))
                var.set(self.paths[key].name)
        finally:
            self.applying_inputs = False
        self.inputs_changed()
        if self.result_pages.select() == str(self.motor_page):
            self.motor_page.refresh()

    def read_motor_curve(self):
        model = self.model_vars["motor_model"].get()
        selected = "lp03" if model == MODEL_CHOICES["motor_model"][1]["lp03"] else "curve"
        settings = Settings(
            motor_model=selected, motor_path=self.paths["motor_path"], lp03_path=self.paths["lp03_path"],
            dc_voltage=float(self.entries["dc_voltage"].get()) if selected == "lp03" else self.default.dc_voltage,
            aero_enabled=False,
        )
        path = settings.lp03_path if selected == "lp03" else settings.motor_path
        return path, Car(settings, preload_motor=False).motor_curve

    def selected_models(self):
        return {
            key: next(value for value, label in MODEL_CHOICES[key][1].items() if label == var.get())
            for key, var in self.model_vars.items()
        }

    def active_toggles(self):
        return {
            key: getattr(self, key).get() if key in self.active_inputs else getattr(self.default, key)
            for key in SETTING_TOGGLES
        }

    def read_settings(self, *, vehicle_only=False, include_aero=True):
        values = dict(self.paths, **self.active_toggles())
        values.update(self.selected_models())
        if not include_aero:
            values["aero_enabled"] = False
        for key, entry in self.entries.items():
            if key not in self.active_inputs:
                continue
            if not include_aero and key in {"aero_cl0", "aero_cd0"}:
                continue
            if vehicle_only and key in {
                "max_speed", "time_limit_s", "safety_factor", "passes", "initial_speed", "track_width",
                "step_m", "curvature_sigma_m", "gear_ratio", "gear_efficiency",
                "front_torque_scale", "rear_torque_scale", "front_brake_bias",
                "brake_leverage", "pedal_force", "pedal_ratio", "master_diameter",
                "front_caliper_diameter", "rear_caliper_diameter", "front_pistons", "rear_pistons",
                "disc_diameter", "pad_mu", "dc_voltage",
            }:
                continue
            raw = entry.get().strip()
            if key in OPTIONAL_NUMBERS and not raw:
                continue
            try:
                parse = float if key in OPTIONAL_NUMBERS else type(getattr(self.default, key))
                values[key] = parse(raw)
            except ValueError as exc:
                raise ValueError(f"請檢查「{self.field_labels[key]}」的數值。") from exc
        settings = Settings(**values)
        settings.validate()
        if settings.tire_model == "ellipse":
            build_tire_model(settings)
        return settings

    def calculate(self):
        if self.busy:
            return
        try:
            settings = self.read_settings()
        except ValueError as exc:
            self.status.set(f"輸入有誤。{exc}")
            return
        self.clear_timeout_recovery(clear_changes=False)
        self.busy = True
        if self.result_pages.select() == str(self.model_help):
            self.refresh_model_help()
        revision = self.revision
        self.run_button.state(["disabled"])
        self.export_button.state(["disabled"])
        self.signal_page.mark_stale()
        if self.result is not None:
            self.result_title.set("上次計算結果")
        self.status.set("正在計算…")
        self.progress.pack(side="right", padx=(8, 0))
        self.progress.start(18)

        def work():
            try:
                result = simulate(settings, progress=lambda text: self.events.put(("progress", text)))
            except TimeoutError as exc:
                self.events.put(("timeout", (revision, settings, exc)))
            except Exception as exc:
                self.events.put(("error", exc))
            else:
                self.events.put(("result", (revision, settings.track_path.stem, result)))

        threading.Thread(target=work, daemon=True).start()
        self.after(50, self.poll)

    def poll(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except Empty:
                self.after(50, self.poll)
                return
            if kind == "progress":
                self.status.set(payload + "…")
                continue
            self.busy = False
            self.progress.stop()
            self.progress.pack_forget()
            self.run_button.state(["!disabled"])
            if kind == "timeout":
                self.result_revision = None
                self.show_timeout(*payload)
            elif kind == "error":
                self.result_revision = None
                self.status.set(f"計算失敗。{payload}")
            else:
                self.show_result(*payload)
            if self.result_revision == self.revision:
                self.export_button.state(["!disabled"])
            if self.result_pages.select() == str(self.model_help):
                self.refresh_model_help()
            return

    def clear_timeout_recovery(self, *, clear_changes=True):
        self.timed_out_settings = None
        self.retry_button.pack_forget()
        if clear_changes:
            self.model_changes.pack_forget()

    def show_timeout(self, revision, settings, error):
        message = f"計算逾時。{error}"
        if revision != self.revision:
            message += " 設定已變更，請重新計算。"
        elif lower_model_choices(settings):
            self.timed_out_settings = settings
            self.retry_button.pack(side="right", padx=(8, 0), before=self.status_label)
        else:
            message += " " + timeout_suggestions(settings)
        self.status.set(message)

    def retry_lower_models(self):
        settings = self.timed_out_settings
        if self.busy or settings is None:
            return
        changes = lower_model_choices(settings)
        for key, value in changes.items():
            if key == "yaw_correction":
                self.yaw_correction.set(value)
            elif key == "ellipse_path":
                self.paths[key] = value
                self.file_vars[key].set(value.name)
                self.inputs_changed()
            else:
                self.model_vars[key].set(MODEL_CHOICES[key][1][value])
        self.model_changes.configure(text=describe_model_changes(settings, changes))
        self.model_changes.pack(fill="x", pady=(5, 0))
        self.calculate()

    def show_result(self, revision, track_name, result):
        self.result = result
        self.result_revision = revision
        updated = self.revision == revision
        self.result_title.set("模擬結果" if updated else "上次計算結果")
        for variable, value in zip(self.metrics, (
            f"{result.lap_time:.3f} s",
            f"{result.track.s[-1]:,.1f} m",
            f"{np.max(result.plan.speed) * 3.6:.1f} km/h",
            f"{np.max(np.abs(result.lateral_acceleration)) / GRAVITY:.2f} g",
        )):
            variable.set(value)
        for chart in self.charts:
            chart.set_result(result)
        self.signal_page.set_result(result, stale=not updated)
        text = f"{track_name}  ·  {len(result.time):,} 個取樣點"
        if result.plan.converged is False:
            text += f"。{result.plan.passes_used} 輪掃描內未達收斂門檻"
        if result.overspeed_points:
            text += f"。{result.overspeed_points} 點超過彎速上限"
        self.status.set(text if updated else f"{text}。輸入已變更，請重新計算。")

    def export(self):
        if self.busy or self.result is None or self.result_revision != self.revision:
            return
        path = filedialog.asksaveasfilename(
            parent=self, initialfile="lap_result.csv", defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
        )
        if path:
            try:
                write_csv(Path(path), self.result)
            except OSError as exc:
                self.status.set(f"無法匯出。{exc}")
            else:
                self.status.set(f"已匯出 {path}")
