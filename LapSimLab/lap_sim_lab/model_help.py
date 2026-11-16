"""Show how the current workbench settings feed the lap solver."""

import tkinter as tk
from tkinter import ttk

from .charts import FONT
from .planner import BRAKING_SPEED_TOLERANCE, SPEED_PLAN_CONVERGENCE_TOLERANCE
from .settings import MODEL_CHOICES


METHODS = {
    "tire_model": {
        "ellipse": "讀取 JSON 的 mu_x、mu_y，分別乘以輪載取得純縱向及純側向峰值。選擇擬合 JSON 時，峰值隨輪載與外傾角改變。兩種格式都用 n = 2 的橢圓算指定 Fy 下剩餘的 Fx。",
        "superellipse": "從輪胎 JSON 依輪載與外傾角算純縱向、純側向峰值，再用指數 n 算超橢圓。剩餘縱向力為 Fx 峰值 × [1 − (|Fy| / Fy 峰值)ⁿ]^(1/n)。",
        "table": "用 Pacejka 資料的 Fz 與 Fy 插值，分別取得驅動與煞車的 Fx 邊界。資料表以零外傾角為基準。含滑移資料的表也能事後反推側滑角、滑移率、輪胎力矩與力殘差。",
    },
    "load_model": {
        "compliant": "以 m × ax × 重心高 ÷ 軸距在前後軸間轉移輪載，按前後側傾剛性分配左右載重轉移。輪胎垂直壓縮量 Fz / Kt 的左右差和前後差，會再加到側傾與俯仰。",
        "point_mass": "整台車以 Fz = max(mg + 下壓力, 0) 求抓地。固定摩擦係數直接乘總輪載；擬合與查表模型以四分之一的總輪載、零外傾角估算單輪抓地並合計。四輪輪載、單輪扭矩與懸吊訊號填 0。",
    },
    "brake_model": {
        "leverage": "踏板力 × 機械利益得到等效總煞車力，再乘自由半徑得到總輪端扭矩。依前後配比拆到兩軸，每軸左右輪各分一半。",
        "hydraulic": "踏板力經踏板比和平衡桿分到兩軸。總泵面積決定油壓，油壓乘單輪卡鉗活塞總面積得到夾力，再乘來令片摩擦係數與碟盤有效半徑得到輪端扭矩。",
    },
    "motor_model": {
        "curve": "求解時由車速、輪胎自由半徑與齒比換算馬達 RPM，從 CSV 線性插值，超出轉速範圍用端點線段外插。",
        "lp03": "求解時由車速、輪胎自由半徑與齒比換算馬達 RPM，在包絡表中依固定直流電壓和 RPM 插值。每圈使用同一電壓，電池電量與溫度沒有逐步更新。",
    },
    "aero_model": {
        "attitude": "用升沉、側傾、俯仰及 JSON 係數算 CL、CD、下壓力分配與空力中心。逐輪求解時反覆用空力更新輪載與姿態，再由姿態重算空力。",
        "quadratic": "使用 JSON 在零姿態下的 CL、CD 和下壓力分配。下壓力與風阻皆隨車速平方變化，側傾、俯仰和升沉對係數的影響為 0。",
    },
    "curvature_model": {
        "raw": "直接用分段 CSV 的曲率。直線 κ = 0，圓弧 κ = ±1 / 半徑，正負代表左右彎。",
        "smooth": "依沿程距離平滑原始曲率，使用設定的高斯尺度。車輛能力計算使用平滑後的 κ，中心線座標和計時距離沿用原始路徑。",
    },
}


def describe_models(options, inputs, files, *, aero_enabled, yaw_correction, braking_lookahead):
    """Describe selected methods using live input text, even during incomplete edits."""
    def section(key, *paragraphs, selection=None):
        label, choices = MODEL_CHOICES[key]
        return f"{label} · {selection or choices[options[key]]}", "\n".join(paragraphs)

    point_mass = options["load_model"] == "point_mass"
    tire = options["tire_model"]
    if options["solver"] == "lap":
        solver = section(
            "solver",
            "在每個賽道點用二分法找彎速，再前掃加速、後掃煞車。反覆降低不符合限制的速度，直到最大速度改變達到門檻，或用完掃描次數。",
            f"最多 {inputs['passes']} 輪，收斂門檻 {SPEED_PLAN_CONVERGENCE_TOLERANCE:g} m/s。",
            "路徑首尾各自求速。",
        )
    else:
        braking = ("先做一次反向煞車前瞻，再用局部縱向力限制向前推進。" if braking_lookahead else
                   "以當下減速度與距離下一個彎速低點的長度，估算何時煞車。")
        solver = section(
            "solver", f"從 {inputs['initial_speed']} m/s 出發，用 v下一點² = v² + 2 × ax × Δs 推進。{braking}",
            f"來不及煞車時保留可達的車速，超過彎速上限 {BRAKING_SPEED_TOLERANCE:g} m/s 以上的點會標示。",
        )
    time_limit = inputs["time_limit_s"]
    try:
        unlimited = float(time_limit) == 0
    except ValueError:
        unlimited = False
    time_note = "計算時間不限。" if unlimited else f"計算時間上限 {time_limit} 秒，逾時停止本次計算。"
    solver = (solver[0], solver[1] + (
        f"\n兩種求解都沿距離估算準穩態車輛能力。側向安全係數 {inputs['safety_factor']}，"
        f"車速上限 {inputs['max_speed']} m/s。圈速為各段距離 ÷ 兩端平均車速的加總。"
        "\n縱向能力由輪胎剩餘抓地與馬達或煞車的上限共同決定，再用風阻、滾阻與車重參數求加減速度。"
        f"\n{time_note}"
    ))

    curvature = section(
        "curvature_model", METHODS["curvature_model"][options["curvature_model"]],
        f"賽道檔：{files['track_path']}。取樣間距設定 {inputs['step_m']} m。"
        + (f" 平滑尺度 {inputs['curvature_sigma_m']} m。" if options["curvature_model"] == "smooth" else ""),
    )
    tire_data = {
        "table": f"資料表：{files['tire_table_path']}。輪載超出範圍時用表格端點。",
        "ellipse": f"摩擦橢圓 JSON：{files['ellipse_path']}。可用固定摩擦係數或擬合係數。",
        "superellipse": (
            f"輪胎檔：{files['tire_path']}。輪載及外傾角以擬合範圍的端點為限。"
            "指數 n 直接採用所選輪胎 JSON 的值。"
        ),
    }
    tire_section = section(
        "tire_model", METHODS["tire_model"][tire], tire_data[tire],
        f"輪胎使用率為 {inputs['tire_utilization']} %，整個 Fx／Fy 抓地包絡等比例縮放。",
    )

    load = options["load_model"]
    load_inputs = f"總質量 {inputs['mass']} kg。"
    if not point_mass:
        load_inputs += (
            f" 前軸 Heave {inputs['front_heave_stiffness']} N/m，"
            f"前軸 Roll {inputs['front_roll_stiffness']} N/m／輪，後／前剛性比 {inputs['rear_stiffness_ratio']}。"
            f" 輪胎垂直剛性 {inputs['tire_stiffness']} N/m，用於由輪載換算受載半徑。"
        )
    loads = section("load_model", METHODS["load_model"][load], load_inputs)

    if aero_enabled:
        aero_text = METHODS["aero_model"][options["aero_model"]]
        if options["aero_model"] == "attitude" and point_mass:
            aero_text = "目前輪載模型的升沉、側傾、俯仰皆為 0，因此圈速計算使用 JSON 的零姿態空力。"
        coefficients = f"空力檔：{files['aero_path']}。"
        if options["aero_model"] == "quadratic":
            coefficients += "CL0 與 CD0 必填，其他常數可使用預設值。"
        aero = section("aero_model", aero_text, coefficients)
    else:
        aero = section("aero_model", "本次計算的下壓力、風阻與空力中心皆為 0。", selection="關閉")

    if point_mass:
        lateral = ("側向力分配 · 整車合力", "以整車 m × v² × κ 求側向需求。質點只求整車力，沒有前後軸力矩。")
    else:
        lateral = (
            "側向力分配 · 前後軸偏航力矩平衡",
            "先按重心到前後軸的力臂分配 Fy，使兩軸偏航力矩相抵；同軸左右輪則按輪荷比例分配。"
            "彎速受最先達到側向極限的輪胎限制。加速或煞車時，任一輪無法提供分到的 Fy，該工作點就不可行。",
        )

    motor = options["motor_model"]
    motor_data = (f"包絡檔：{files['lp03_path']}，匯流排 {inputs['dc_voltage']} V。" if motor == "lp03" else
                  f"扭矩表：{files['motor_path']}。")
    motor_section = section(
        "motor_model", METHODS["motor_model"][motor],
        motor_data + f" 齒比 {inputs['gear_ratio']}，傳動效率 {inputs['gear_efficiency']} %，"
        f"前／後輪限扭倍率 {inputs['front_torque_scale']}／{inputs['rear_torque_scale']}。",
    )
    brake = options["brake_model"]
    brake_inputs = f"踏板力 {inputs['pedal_force']} N，前軸配比 {inputs['front_brake_bias']}。"
    if brake == "hydraulic":
        brake_inputs += (
            f" 踏板比 {inputs['pedal_ratio']}，總泵直徑 {inputs['master_diameter']} m，"
            f"前／後卡鉗活塞直徑 {inputs['front_caliper_diameter']}／{inputs['rear_caliper_diameter']} m，"
            f"前／後活塞數 {inputs['front_pistons']}／{inputs['rear_pistons']}，"
            f"碟盤直徑 {inputs['disc_diameter']} m，來令片摩擦係數 {inputs['pad_mu']}。"
        )
    else:
        brake_inputs += f" 機械利益 {inputs['brake_leverage']}。"
    brakes = section("brake_model", METHODS["brake_model"][brake], brake_inputs)

    if tire == "table":
        resistance = ("滾動阻力 · 隨 Pacejka 查表", "按輪載插值零滑移的滾阻扭矩，再除以受載半徑換成阻力。")
    else:
        resistance = ("滾動阻力 · 固定滾阻係數", f"滾阻力 = Crr × Fz，目前 Crr = {inputs['rolling_coefficient']}。")

    yaw = ("已勾選，但質點模型沒有逐輪輪載。請關閉修正或改選逐輪模型。" if point_mass else
           f"已啟用，Izz = {inputs['yaw_inertia']} kg·m²。以曲率變化率估算偏航力矩需求，依逐輪側向力可提供的力矩折減彎速。")
    output = ("兩種求解器都由速度規劃重建逐點訊號。位移所衍生的速度與加速度用時間差分。"
              "\n目前使用摩擦力包絡，輪胎滑移、力矩與反推殘差填 0。" if tire != "table" else
              "兩種求解器都由速度規劃重建逐點訊號。滑移與力矩由資料表反推；所選 NPZ 若只有抓地包絡，這些欄位填 0。")
    return [
        solver, curvature, loads, aero, tire_section, lateral, motor_section, brakes, resistance,
        ("偏航慣量彎速修正", yaw if yaw_correction else "關閉，彎速直接使用側向力限制。"),
        ("逐點訊號與限制", output),
    ]


class ModelHelpPage(ttk.Frame):
    def __init__(self, master):
        super().__init__(master, padding=(12, 8))
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(header, text="目前模型組合", style="Section.TLabel").pack(side="left")
        self.jump = ttk.Combobox(header, width=18, state="readonly")
        self.jump.pack(side="right")
        self.jump.bind("<<ComboboxSelected>>", self.jump_to_section)
        self.status = tk.StringVar()
        message = ttk.Label(self, textvariable=self.status, style="Muted.TLabel", justify="left")
        message.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        message.bind("<Configure>", lambda event: message.configure(wraplength=event.width))
        panel = ttk.Frame(self)
        panel.grid(row=2, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(panel)
        scrollbar.pack(side="right", fill="y")
        self.text = tk.Text(
            panel, wrap="char", state="disabled", font=(FONT, 10), width=1, height=1,
            padx=14, pady=9, relief="flat", background="#ffffff", foreground="#17253a",
            yscrollcommand=scrollbar.set, spacing2=3,
        )
        self.text.pack(side="left", fill="both", expand=True)
        scrollbar.configure(command=self.text.yview)
        self.text.tag_configure("heading", font=(FONT, 11, "bold"), foreground="#16385e", spacing1=10, spacing3=4)
        self.text.tag_configure("body", spacing3=5)

    def jump_to_section(self, _event=None):
        self.text.yview(f"section_{self.jump.current()}")

    def show(self, sections, status):
        top = self.text.index("@0,0")
        self.status.set(status)
        self.jump.configure(values=tuple(title.split(" · ")[0] for title, _ in sections))
        if self.jump.current() < 0:
            self.jump.current(0)
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        for index, (title, body) in enumerate(sections):
            mark = f"section_{index}"
            self.text.mark_set(mark, "end-1c")
            self.text.mark_gravity(mark, "left")
            self.text.insert("end", title + "\n", "heading")
            self.text.insert("end", body + "\n\n", "body")
        self.text.configure(state="disabled")
        self.text.yview(top)
