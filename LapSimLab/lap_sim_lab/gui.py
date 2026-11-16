"""Two-pane Tk workbench for configuring and reviewing one LapSim run."""

from dataclasses import replace
from decimal import Decimal
import math
from pathlib import Path
from queue import Empty, SimpleQueue
from shutil import copyfile
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .aero import AERO_COEFFICIENT_KEYS, read_aero_coefficients
from .car import Car
from .analysis_gui import MotorPage, SweepPage
from .charts import FONT
from .ggv_gui import GGVPage
from .model_help import ModelHelpPage, describe_models
from .optimization import FIXED_MODELS, optimize_lap
from .optimization_gui import OptimizationPage
from .settings import MODEL_CHOICES, PERCENTAGE_FIELDS, Settings, parse_percentage
from .settings_profile import load_profile, save_profile, write_settings_json
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
SETTING_TOGGLES = ("aero_enabled", "yaw_correction", "braking_lookahead")
PROFILE_FILETYPES = [("LapSimLab 設定 JSON", "*.json")]

SURFACE = "#f1f5f9"
WHITE = "#ffffff"
INK = "#17253a"
MUTED = "#52657b"


def format_number_input(key, value):
    if key in PERCENTAGE_FIELDS:
        try:
            number = float(value)
        except (ValueError, OverflowError):
            return str(value)
        if math.isfinite(number):
            return format(Decimal(str(number)).scaleb(2).normalize(), "f")
    return str(value)


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
        self.model_controls = {}
        self.analysis_pages = {}
        self.control_sim_window = None
        self.aero_enabled = tk.BooleanVar(value=self.default.aero_enabled)
        self.yaw_correction = tk.BooleanVar(value=self.default.yaw_correction)
        self.braking_lookahead = tk.BooleanVar(value=self.default.braking_lookahead)
        self.status = tk.StringVar(value="已載入內附資料，可直接計算。")
        self.result = None
        self.result_revision = None
        self.revision = 0
        self.busy = False
        self.optimization_cancel = None
        self.timed_out_settings = None
        self.events = SimpleQueue()

        self.configure_styles()
        self.build_layout()
        self.refresh_input_rows()
        self.refresh_model_help()
        self.aero_enabled.trace_add("write", self.inputs_changed)
        self.yaw_correction.trace_add("write", self.inputs_changed)
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
        style.configure("Panel.TLabelframe", background=WHITE)
        style.configure("Panel.TLabelframe.Label", background=WHITE)
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
        ttk.Button(header, text="車輛控制模擬", command=self.open_control_sim).pack(side="right", padx=(0, 8))

        body = ttk.Frame(self, padding=(19, 2, 19, 0))
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        controls = ttk.Frame(body)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 17))
        ttk.Label(controls, text="模擬輸入", style="Section.TLabel").pack(anchor="w", pady=(0, 8))
        self.build_inputs(self.input_panel(controls))

        buttons = ttk.Frame(controls)
        buttons.pack(fill="x", pady=(11, 0))
        self.run_button = ttk.Button(buttons, text="計算  F5", style="Primary.TButton", command=self.calculate)
        self.run_button.pack(side="left", fill="x", expand=True)
        ttk.Button(buttons, text="恢復預設", command=self.reset).pack(side="left", padx=(8, 0))

        self.result_pages = ttk.Notebook(body)
        self.result_pages.grid(row=0, column=1, sticky="nsew")

        self.ggv_page = GGVPage(self.result_pages, self.read_settings)
        self.signal_page = SignalView(self.result_pages, self.export)
        self.model_help = ModelHelpPage(self.result_pages)
        self.optimization_page = OptimizationPage(
            self.result_pages, self.entries, self.start_optimization, self.cancel_optimization, self.apply_optimization,
        )
        self.optimization_page.refresh_sources(self.selected_models(), self.paths["track_path"].name)
        for page, label in (
            (self.signal_page, "訊號檢視"),
            (self.optimization_page, "敏感性分析"),
            (self.ggv_page, "GGV"), (self.model_help, "模型說明"),
        ):
            self.result_pages.add(page, text=label)
        self.result_pages.bind("<<NotebookTabChanged>>", self.result_tab_changed)

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

    def result_tab_changed(self, _event=None):
        page = self.nametowidget(self.result_pages.select())
        self.update_fixed_models(page is self.optimization_page)
        if page is self.optimization_page:
            self.optimization_page.refresh_aero_baseline(self.paths["aero_path"])
        elif page is self.model_help:
            self.refresh_model_help()

    def open_control_sim(self):
        if self.control_sim_window is None or not self.control_sim_window.winfo_exists():
            try:
                from .control_gui import ControlSimWindow
            except ModuleNotFoundError as error:
                if error.name != "pygame":
                    raise
                messagebox.showerror(
                    "車輛控制模擬",
                    "請先在 LapSimLab 資料夾執行 python -m pip install -r requirements.txt 安裝 Pygame。",
                    parent=self,
                )
                return
            try:
                self.control_sim_window = ControlSimWindow(self, self.read_control_settings())
            except (ValueError, OSError) as error:
                messagebox.showerror("車輛控制模擬", str(error), parent=self)
                return
        self.control_sim_window.show()

    def open_analysis(self, title, create_page):
        page = self.analysis_pages.get(title)
        if page is None:
            window = tk.Toplevel(self)
            window.withdraw()
            window.title(title)
            window.transient(self)
            window.geometry("780x520")
            window.minsize(660, 370)
            window.protocol("WM_DELETE_WINDOW", window.withdraw)
            page = create_page(window)
            page.pack(fill="both", expand=True)
            self.analysis_pages[title] = page
        window = page.master
        if page.dirty:
            page.refresh()
        window.deiconify()
        window.lift()
        window.focus_set()

    def refresh_open_motor_page(self):
        page = self.analysis_pages.get("馬達曲線")
        if page is not None and page.winfo_ismapped():
            page.refresh()

    def update_fixed_models(self, locked):
        for key in FIXED_MODELS:
            self.model_controls[key].state(["disabled" if locked else "!disabled"])
        if not locked:
            return
        current = self.selected_models()
        current.update({key: getattr(self, key).get() for key in SETTING_TOGGLES})
        if all(current[key] == value for key, value in FIXED_MODELS.items()):
            return
        self.applying_inputs = True
        try:
            self.apply_model_choices(FIXED_MODELS)
        finally:
            self.applying_inputs = False
        self.inputs_changed()

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

    def build_inputs(self, parent):
        tire = self.input_section(parent, "輪胎")
        self.model_row(tire, "tire_model")
        self.file_row(tire, "tire_path")
        self.file_row(tire, "ellipse_path")
        self.file_row(tire, "tire_table_path")
        self.number_row(tire, "tire_radius", "自由半徑", "m")
        self.number_row(tire, "rolling_coefficient", "固定滾阻係數")
        self.model_hint(tire, "rolling_table_hint", "滾動阻力隨 Pacejka 查表。")
        self.analysis_button(
            tire, "輪胎力掃描", "開啟輪胎力掃描介面",
            lambda window: SweepPage(window, lambda: self.read_settings(vehicle_only=True, include_aero=False), tire=True),
        )
        self.example_button(tire, lambda: {
            "table": self.default.tire_table_path,
            "superellipse": self.default.tire_path,
            "ellipse": self.default.ellipse_path,
        }[self.selected_models()["tire_model"]])

        suspension = self.input_section(parent, "懸吊與車體")
        self.model_row(suspension, "load_model")
        self.number_row(suspension, "mass", "總質量（含車手）", "kg")
        self.number_row(suspension, "tire_utilization", "輪胎使用率", "%")
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

        motor = self.input_section(parent, "馬達")
        self.model_row(motor, "motor_model")
        self.file_row(motor, "motor_path")
        self.file_row(motor, "lp03_path")
        self.number_row(motor, "dc_voltage", "直流電壓", "V")
        self.number_row(motor, "gear_ratio", "固定減速比")
        self.number_row(motor, "gear_efficiency", "傳動效率", "%")
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
        self.analysis_button(motor, "馬達曲線", "開啟馬達曲線", lambda window: MotorPage(window, self.read_motor_curve))
        self.example_button(motor, lambda: {
            "curve": self.default.motor_path, "lp03": self.default.lp03_path,
        }[self.selected_models()["motor_model"]])

        aero = self.input_section(parent, "空力")
        self.toggle_row(aero, "aero_enabled", "啟用空力")
        self.model_row(aero, "aero_model")
        self.file_row(aero, "aero_path")
        self.analysis_button(aero, "空力掃描", "開啟空力掃描介面",
                             lambda window: SweepPage(window, lambda: self.read_settings(vehicle_only=True)))
        self.example_button(aero, lambda: {
            "quadratic": self.default.aero_path.with_name("quadratic_aero.json"),
            "attitude": self.default.aero_path,
        }[self.selected_models()["aero_model"]])

        track = self.input_section(parent, "賽道")
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
        self.number_row(solver, "initial_speed", "起始車速", "m/s")
        self.toggle_row(solver, "braking_lookahead", "使用單次煞車前瞻")
        self.example_button(track, lambda: self.default.track_path)

    def input_panel(self, parent):
        viewport = ttk.Frame(parent, style="Panel.TFrame")
        viewport.pack(fill="both", expand=True)
        canvas = tk.Canvas(viewport, background=WHITE, highlightthickness=0, width=330, height=1)
        scrollbar = ttk.Scrollbar(viewport, command=canvas.yview)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        canvas.configure(yscrollcommand=scrollbar.set)
        panel = ttk.Frame(canvas, style="Panel.TFrame", padding=(8, 6))
        window = canvas.create_window(0, 0, anchor="nw", window=panel)
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        panel.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))

        def scroll(event):
            canvas.yview_scroll(-int(event.delta / 120), "units")
            return "break"

        def show_focused_row(event):
            widget = event.widget
            if not isinstance(widget, (ttk.Entry, ttk.Checkbutton, ttk.Button)):
                return
            top = widget.winfo_rooty() - panel.winfo_rooty()
            bottom = top + widget.winfo_height()
            view_top = canvas.canvasy(0)
            view_height = canvas.winfo_height()
            if top < view_top:
                canvas.yview_moveto(top / panel.winfo_height())
            elif bottom > view_top + view_height:
                canvas.yview_moveto((bottom - view_height) / panel.winfo_height())

        self.input_scroll_tag = str(viewport)
        viewport.bind("<MouseWheel>", scroll)
        viewport.bind("<FocusIn>", show_focused_row)
        self.bind_input_scroll(canvas)
        self.bind_input_scroll(panel)
        return panel

    def input_section(self, parent, title):
        section = ttk.LabelFrame(parent, text=title, style="Panel.TLabelframe", padding=(7, 4))
        section.pack(fill="x", pady=(0, 7))
        self.bind_input_scroll(section)
        fields = ttk.Frame(section, style="Panel.TFrame")
        fields.pack(fill="x")
        self.bind_input_scroll(fields)
        return fields

    def analysis_button(self, panel, title, text, create_page):
        self.section_button(panel, text, lambda: self.open_analysis(title, create_page))

    def example_button(self, panel, source):
        self.section_button(panel, "下載範例檔案", lambda: self.save_example(source()))

    def section_button(self, panel, text, command):
        button = ttk.Button(panel.master, text=text, command=command)
        button.pack(fill="x", pady=(6, 1))
        self.bind_input_scroll(button)

    def input_group(self, panel, title):
        group = ttk.LabelFrame(panel, text=title, style="Panel.TLabelframe", padding=(6, 3))
        self.input_groups[panel] = group
        self.bind_input_scroll(group)
        return group

    def input_row(self, panel, key, label=""):
        row = ttk.Frame(panel, style="Panel.TFrame")
        self.bind_input_scroll(row)
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

    def bind_input_scroll(self, widget):
        own, *tags = widget.bindtags()
        widget.bindtags((own, self.input_scroll_tag, *tags))

    def number_row(self, panel, key, label, unit=""):
        default = getattr(self.default, key)
        var = tk.StringVar(value=format_number_input(key, default))
        var.trace_add("write", self.inputs_changed)
        self.entries[key] = var
        self.field_labels[key] = label
        row = self.input_row(panel, key, label)
        entry = ttk.Entry(row, textvariable=var, width=9)
        entry.grid(row=0, column=1, sticky="ew")
        ttk.Label(row, text=unit, style="Hint.TLabel").grid(row=0, column=2, sticky="w", padx=(5, 0))
        for widget in row.winfo_children():
            self.bind_input_scroll(widget)

    def model_row(self, panel, key):
        label, choices = MODEL_CHOICES[key]
        row = self.input_row(panel, key, label)
        variable = tk.StringVar(value=choices[getattr(self.default, key)])
        self.model_vars[key] = variable
        control = ttk.Combobox(row, textvariable=variable, values=tuple(choices.values()),
                               width=17, state="readonly")
        control.grid(row=0, column=1, sticky="ew")
        self.model_controls[key] = control
        variable.trace_add("write", self.inputs_changed)
        for widget in row.winfo_children():
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
        for widget in row.winfo_children():
            self.bind_input_scroll(widget)

    def toggle_row(self, panel, key, label):
        row = self.input_row(panel, key)
        toggle = ttk.Checkbutton(row, text=label, variable=getattr(self, key), style="Panel.TCheckbutton")
        toggle.grid(row=0, column=0, sticky="w")
        self.model_controls[key] = toggle
        self.bind_input_scroll(toggle)

    def visible_input_keys(self, models, *, aero, yaw):
        tire = models["tire_model"]
        four_wheels = models["load_model"] != "point_mass"
        lp03 = models["motor_model"] == "lp03"
        hydraulic = models["brake_model"] == "hydraulic"
        lap = models["solver"] == "lap"
        conditional = {
            "tire_path": tire == "superellipse", "ellipse_path": tire == "ellipse",
            "tire_table_path": tire == "table",
            "tire_stiffness": four_wheels, "rolling_coefficient": tire != "table",
            "rolling_table_hint": tire == "table",
            "wheelbase": four_wheels, "front_track": four_wheels, "rear_track": four_wheels,
            "cg_from_front": four_wheels, "cg_height": four_wheels,
            "front_heave_stiffness": four_wheels, "front_roll_stiffness": four_wheels,
            "rear_stiffness_ratio": four_wheels,
            "yaw_correction": four_wheels,
            "yaw_inertia": four_wheels and yaw,
            "motor_path": not lp03, "lp03_path": lp03, "dc_voltage": lp03,
            "brake_leverage": not hydraulic,
            **dict.fromkeys(("pedal_ratio", "master_diameter", "front_caliper_diameter", "rear_caliper_diameter",
                             "front_pistons", "rear_pistons", "disc_diameter", "pad_mu"), hydraulic),
            "aero_model": aero, "aero_path": aero,
            "curvature_sigma_m": models["curvature_model"] == "smooth", "passes": lap,
            "initial_speed": not lap, "braking_lookahead": not lap,
        }
        return {key for key in self.input_rows if conditional.get(key, True)}

    def refresh_input_rows(self):
        active = self.visible_input_keys(self.selected_models(), aero=self.aero_enabled.get(), yaw=self.yaw_correction.get())
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
        for page in self.analysis_pages.values():
            page.invalidate()
        self.ggv_page.invalidate()
        self.ggv_page.set_lap_result(None)
        self.optimization_page.invalidate()
        self.optimization_page.refresh_sources(self.selected_models(), self.paths["track_path"].name)
        self.signal_page.mark_stale()
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
                self.refresh_open_motor_page()
            elif key == "aero_path":
                self.optimization_page.refresh_aero_baseline(path)

    def save_example(self, source):
        suffix = source.suffix
        target = filedialog.asksaveasfilename(
            parent=self, title="儲存範例檔案", initialfile=f"{source.stem}_example{suffix}",
            defaultextension=suffix, filetypes=[(suffix[1:].upper(), f"*{suffix}")],
        )
        if target:
            try:
                copyfile(source, target)
            except OSError as exc:
                self.status.set(f"無法儲存範例檔案。{exc}")
            else:
                self.status.set(f"已儲存範例檔案至 {target}")

    def save_settings(self):
        selected = filedialog.asksaveasfilename(
            parent=self, title="儲存 LapSimLab 設定", defaultextension=".json", filetypes=PROFILE_FILETYPES,
            initialdir=self.profile_path.parent if self.profile_path else None,
            initialfile=self.profile_path.name if self.profile_path else "lapsimlab_settings.json",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            save_profile(path, self.input_snapshot())
        except (OSError, ValueError) as exc:
            self.status.set(f"無法儲存設定。{exc}")
        else:
            self.profile_path = path
            self.status.set(f"已儲存設定至 {path}")

    def input_snapshot(self):
        snapshot = dict(self.paths, **self.selected_models())
        snapshot.update({key: getattr(self, key).get() for key in SETTING_TOGGLES})
        snapshot.update({key: entry.get() for key, entry in self.entries.items()})
        for key in PERCENTAGE_FIELDS:
            try:
                value = parse_percentage(snapshot[key])
            except ValueError:
                continue
            if math.isfinite(value):
                snapshot[key] = value
        return snapshot

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
        choices = {key: snapshot.get(key, getattr(self.default, key)) for key in self.model_controls}
        if self.result_pages.select() == str(self.optimization_page):
            choices.update(FIXED_MODELS)
        self.applying_inputs = True
        try:
            for key, var in self.entries.items():
                value = snapshot.get(key, getattr(self.default, key))
                var.set(format_number_input(key, value))
            self.apply_model_choices(choices)
            for key, var in self.file_vars.items():
                self.paths[key] = snapshot.get(key, getattr(self.default, key))
                var.set(self.paths[key].name)
        finally:
            self.applying_inputs = False
        self.inputs_changed()
        self.optimization_page.refresh_aero_baseline(self.paths["aero_path"])
        self.refresh_open_motor_page()

    def apply_model_choices(self, choices):
        for key, value in choices.items():
            if key in self.model_vars:
                self.model_vars[key].set(MODEL_CHOICES[key][1][value])
            else:
                getattr(self, key).set(value)

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

    def read_control_settings(self):
        models = self.selected_models()
        drivetrain_fields = {
            "gear_ratio", "gear_efficiency", "front_torque_scale", "rear_torque_scale", "dc_voltage",
            "front_brake_bias", "pedal_force", "brake_leverage", "pedal_ratio", "master_diameter",
            "front_caliper_diameter", "rear_caliper_diameter", "front_pistons", "rear_pistons",
            "disc_diameter", "pad_mu",
        }
        keys = (drivetrain_fields & self.active_inputs) | {
            "mass", "wheelbase", "cg_from_front", "yaw_inertia", "tire_radius", "max_speed", "initial_speed",
        }
        if models["tire_model"] != "table":
            keys.add("rolling_coefficient")
        settings = replace(
            self.default, aero_enabled=self.aero_enabled.get(),
            aero_model=models["aero_model"], aero_path=self.paths["aero_path"],
            motor_model=models["motor_model"], brake_model=models["brake_model"],
            motor_path=self.paths["motor_path"], lp03_path=self.paths["lp03_path"],
            tire_model=models["tire_model"], tire_table_path=self.paths["tire_table_path"],
            **{key: self.read_number(key) for key in self.entries if key in keys},
        )
        settings.validate()
        return settings

    def read_number(self, key):
        raw = self.entries[key].get().strip()
        parse = parse_percentage if key in PERCENTAGE_FIELDS else type(getattr(self.default, key))
        try:
            return parse(raw)
        except ValueError as error:
            raise ValueError(f"請檢查「{self.field_labels[key]}」的數值。") from error

    def read_settings(self, *, vehicle_only=False, include_aero=True, optimization=False):
        values = dict(self.paths, **self.active_toggles())
        values.update(self.selected_models())
        active = self.active_inputs
        if optimization:
            values.update(FIXED_MODELS)
            active = self.visible_input_keys(values, aero=values["aero_enabled"], yaw=values["yaw_correction"])
        if not include_aero:
            values["aero_enabled"] = False
        for key in self.entries:
            if key not in active:
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
            values[key] = self.read_number(key)
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
        revision = self.revision
        self.signal_page.mark_stale()

        def work():
            try:
                result = simulate(settings, progress=lambda text: self.events.put(("progress", text)))
            except TimeoutError as exc:
                self.events.put(("timeout", (revision, settings, exc)))
            except Exception as exc:
                self.events.put(("error", exc))
            else:
                self.events.put(("result", (revision, settings.track_path.stem, result)))

        self.start_work(work, "正在計算…")

    def start_optimization(self):
        if self.busy:
            return
        self.optimization_page.refresh_aero_baseline(self.paths["aero_path"])
        try:
            settings = self.read_settings(optimization=True)
            options = self.optimization_page.read_options()
        except ValueError as exc:
            self.status.set(f"敏感性分析輸入有誤。{exc}")
            return
        self.clear_timeout_recovery()
        revision = self.revision
        cancelled = threading.Event()
        self.optimization_cancel = cancelled

        def check_cancelled():
            if cancelled.is_set():
                raise InterruptedError

        def work():
            try:
                result = optimize_lap(settings, **options, check_cancelled=check_cancelled,
                                      progress=lambda text: self.events.put(("progress", text)))
            except InterruptedError:
                self.events.put(("optimization_cancelled", None))
            except Exception as exc:
                self.events.put(("optimization_error", exc))
            else:
                self.events.put(("optimization_result", (revision, result)))

        self.start_work(work, "正在進行敏感性分析…")

    def cancel_optimization(self):
        if self.optimization_cancel is not None:
            self.optimization_cancel.set()
            self.status.set("正在取消敏感性分析…")

    def apply_optimization(self):
        result = self.optimization_page.result
        if self.busy or result is None:
            return
        snapshot = self.input_snapshot()
        snapshot.update(FIXED_MODELS)
        source = result.settings.aero_path
        try:
            source_coefficients = read_aero_coefficients(source)
        except (OSError, ValueError):
            source_coefficients = None
        if result.aero_coefficients != source_coefficients:
            selected = filedialog.asksaveasfilename(
                parent=self, title="另存最佳空力係數", defaultextension=".json",
                initialdir=source.absolute().parent, initialfile=f"{source.stem}_optimized.json",
                filetypes=[("JSON", "*.json")],
            )
            if not selected:
                return
            try:
                write_settings_json(Path(selected), result.aero_coefficients)
            except (OSError, ValueError) as error:
                self.status.set(f"無法儲存最佳空力係數。{error}")
                return
            snapshot["aero_path"] = Path(selected)
        for bound in result.ranges:
            key = bound.parameter.key
            if key not in AERO_COEFFICIENT_KEYS:
                snapshot[key] = result.parameter_value(key)
        self.apply_inputs(snapshot)
        self.show_result(self.revision, result.settings.track_path.stem, result.simulation)

    def start_work(self, work, message):
        self.busy = True
        if self.result_pages.select() == str(self.model_help):
            self.refresh_model_help()
        self.optimization_page.set_busy(True, optimizing=self.optimization_cancel is not None)
        self.run_button.state(["disabled"])
        self.signal_page.set_export_enabled(False)
        self.status.set(message)
        self.progress.pack(side="right", padx=(8, 0))
        self.progress.start(18)
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
                if self.optimization_cancel is None or not self.optimization_cancel.is_set():
                    self.status.set(payload + "…")
                continue
            self.busy = False
            self.optimization_cancel = None
            self.optimization_page.set_busy(False)
            self.progress.stop()
            self.progress.pack_forget()
            self.run_button.state(["!disabled"])
            if kind == "timeout":
                self.result_revision = None
                self.show_timeout(*payload)
            elif kind == "error":
                self.result_revision = None
                self.status.set(f"計算失敗。{payload}")
            elif kind == "optimization_cancelled":
                self.status.set("已取消敏感性分析。")
            elif kind == "optimization_error":
                self.status.set(f"敏感性分析失敗。{payload}")
            elif kind == "optimization_result":
                revision, result = payload
                if revision == self.revision:
                    self.optimization_page.show_result(result)
                    self.status.set(f"敏感性分析完成，最低圈速 {result.simulation.lap_time:.3f} 秒。")
                else:
                    self.status.set("分析期間設定已變更，請重新分析。")
            else:
                self.show_result(*payload)
            self.signal_page.set_export_enabled(self.result_revision == self.revision)
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
        self.signal_page.set_result(result, stale=not updated)
        self.ggv_page.set_lap_result(result if updated else None)
        if updated:
            self.result_pages.select(self.signal_page)
        text = f"{track_name}  ·  {len(result.time):,} 個取樣點"
        if result.plan.converged is False:
            text += f"。{result.plan.passes_used} 輪掃描內未達收斂門檻"
        if result.overspeed_points:
            text += f"。{result.overspeed_points} 點超過彎速上限"
        self.status.set(text if updated else f"{text}。輸入已變更，請重新計算。")

    def export(self, *, signal_keys=None):
        if self.busy or self.result is None or self.result_revision != self.revision:
            return
        if signal_keys is not None and not signal_keys:
            return
        filename = "lap_result.csv" if signal_keys is None else "lap_selected_signals.csv"
        path = filedialog.asksaveasfilename(
            parent=self, initialfile=filename, defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
        )
        if path:
            try:
                write_csv(Path(path), self.result, signal_keys=signal_keys)
            except OSError as exc:
                self.status.set(f"無法匯出。{exc}")
            else:
                self.status.set(f"已匯出 {path}")
