from dataclasses import dataclass
from decimal import Decimal, DecimalException
import math
from pathlib import Path


INPUT_DIR = Path(__file__).resolve().parents[1] / "input_data"
DEFAULT_ELLIPSE_PATH = INPUT_DIR / "friction_ellipse.json"
PERCENTAGE_FIELDS = {"tire_utilization", "gear_efficiency"}


def parse_percentage(text):
    try:
        return float(Decimal(text).scaleb(-2))
    except DecimalException as error:
        raise ValueError(f"無法讀取百分比「{text}」。") from error

# Display order follows the detail in each model. Settings below define defaults separately.
# For the solver and track, prefer whole-lap planning and fidelity to the input geometry.
MODEL_CHOICES = {
    "solver": ("求解方式", {"lap": "全圈迭代", "distance": "距離域前向積分"}),
    "tire_model": ("輪胎", {
        "table": "Pacejka 資料插值", "superellipse": "超橢圓",
        "ellipse": "摩擦橢圓",
    }),
    "load_model": ("輪載與姿態", {
        "compliant": "剛性轉移＋輪胎壓縮",
        "point_mass": "質點（全車抓地）",
    }),
    "brake_model": ("煞車", {"hydraulic": "液壓機構", "leverage": "等效機械利益"}),
    "motor_model": ("馬達", {"lp03": "LP03 電壓包絡查表", "curve": "轉速／扭矩查表"}),
    "aero_model": ("空力", {"attitude": "姿態空力", "quadratic": "固定係數 v²"}),
    "curvature_model": ("賽道曲率", {"raw": "原始曲率", "smooth": "距離域平滑"}),
}


@dataclass(frozen=True)
class Settings:
    track_path: Path = INPUT_DIR / "fsae_A_2025.csv"
    tire_path: Path = INPUT_DIR / "hoosier_43075_r20_w8.json"
    motor_path: Path = INPUT_DIR / "motor_speed_vs_torque.csv"
    aero_path: Path = INPUT_DIR / "modal_aero.json"
    track_width: float = 3.0
    step_m: float = 0.5
    max_speed: float = 30.0
    safety_factor: float = 0.98
    passes: int = 3
    aero_enabled: bool = True
    mass: float = 320.0
    wheelbase: float = 1.53
    cg_from_front: float = 0.7498400000000001
    cg_height: float = 0.3016451511
    front_track: float = 1.25
    rear_track: float = 1.24
    tire_radius: float = 0.203
    tire_stiffness: float = 80185.0
    front_heave_stiffness: float = 140000.0
    front_roll_stiffness: float = 100000.0
    rear_stiffness_ratio: float = 1.0
    gear_ratio: float = 11.2
    gear_efficiency: float = 0.95
    front_torque_scale: float = 0.67
    rear_torque_scale: float = 1.0
    front_brake_bias: float = 0.7
    brake_leverage: float = 12.6
    pedal_force: float = 500.0
    solver: str = "lap"
    tire_model: str = "superellipse"
    load_model: str = "compliant"
    brake_model: str = "leverage"
    motor_model: str = "curve"
    aero_model: str = "attitude"
    curvature_model: str = "raw"
    rolling_coefficient: float = 0.015
    tire_table_path: Path = INPUT_DIR / "pacejka_envelope.npz"
    lp03_path: Path = INPUT_DIR / "lp03_voltage_envelope.csv"
    dc_voltage: float = 530.0
    curvature_sigma_m: float = 0.75
    initial_speed: float = 0.0
    braking_lookahead: bool = True
    yaw_correction: bool = False
    yaw_inertia: float = 130.0
    pedal_ratio: float = 2.8
    master_diameter: float = 0.012
    front_caliper_diameter: float = 0.0254
    rear_caliper_diameter: float = 0.0254
    front_pistons: int = 4
    rear_pistons: int = 2
    disc_diameter: float = 0.17
    pad_mu: float = 0.55
    ellipse_path: Path = DEFAULT_ELLIPSE_PATH
    tire_utilization: float = 1.0
    time_limit_s: float = 120.0

    @property
    def rear_heave_stiffness(self) -> float:
        return self.front_heave_stiffness * self.rear_stiffness_ratio

    @property
    def rear_roll_stiffness(self) -> float:
        return self.front_roll_stiffness * self.rear_stiffness_ratio

    def validate(self):
        for key, (label, choices) in MODEL_CHOICES.items():
            if getattr(self, key) not in choices:
                raise ValueError(f"無法辨識「{label}」的選項。")
        if self.load_model == "point_mass" and self.yaw_correction:
            raise ValueError("偏航慣量修正需要逐輪輪載。")
        for name, label in POSITIVE_FIELDS:
            if not math.isfinite(value := getattr(self, name)) or value <= 0:
                raise ValueError(f"{label}須為有限的正數。")
        for name, label in (
            ("cg_height", "重心高度"),
            ("pedal_force", "踏板力"),
            ("initial_speed", "起始車速"),
            ("tire_utilization", "輪胎使用率"),
            ("time_limit_s", "計算時間上限"),
        ):
            if not math.isfinite(value := getattr(self, name)) or value < 0:
                raise ValueError(f"{label}須為有限的非負數。")
        if self.tire_model != "table" and (not math.isfinite(self.rolling_coefficient) or self.rolling_coefficient < 0):
            raise ValueError("固定滾阻係數須為有限的非負數。")
        if self.load_model != "point_mass" and not 0 < self.cg_from_front < self.wheelbase:
            raise ValueError("重心距前軸須大於零且小於軸距。")
        if not 0 < self.safety_factor <= 1:
            raise ValueError("側向安全係數須大於零且不超過 1。")
        if not 0 <= self.gear_efficiency <= 1:
            raise ValueError("傳動效率須介於 0% 和 100%。")
        for name, label in (
            ("front_torque_scale", "前輪限扭比例"),
            ("rear_torque_scale", "後輪限扭比例"),
            ("front_brake_bias", "前軸煞車比例"),
        ):
            if not 0 <= getattr(self, name) <= 1:
                raise ValueError(f"{label}須介於 0 和 1。")
        if type(self.passes) is not int or not 1 <= self.passes <= 30:
            raise ValueError("前後向掃描次數須為 1 到 30 的整數。")
        if self.solver == "distance" and self.initial_speed > self.max_speed:
            raise ValueError("起始車速不得超過速度上限。")
        if any(type(count) is not int or count <= 0 for count in (self.front_pistons, self.rear_pistons)):
            raise ValueError("卡鉗活塞數須為正整數。")


POSITIVE_FIELDS = (
    ("track_width", "賽道寬度"),
    ("step_m", "取樣間距"),
    ("max_speed", "速度上限"),
    ("mass", "總質量"),
    ("wheelbase", "軸距"),
    ("front_track", "前輪距"),
    ("rear_track", "後輪距"),
    ("tire_radius", "輪胎自由半徑"),
    ("tire_stiffness", "輪胎垂直剛性"),
    ("front_heave_stiffness", "前軸 Heave 剛性"),
    ("front_roll_stiffness", "前軸 Roll 剛性"),
    ("rear_stiffness_ratio", "後／前剛性比"),
    ("rear_heave_stiffness", "後軸 Heave 剛性"),
    ("rear_roll_stiffness", "後輪 Roll 剛性"),
    ("gear_ratio", "固定減速比"),
    ("brake_leverage", "煞車機械增益"),
    ("curvature_sigma_m", "曲率平滑尺度"),
    ("yaw_inertia", "偏航慣量"),
    ("pedal_ratio", "煞車踏板比"),
    ("master_diameter", "總泵直徑"),
    ("front_caliper_diameter", "前輪卡鉗活塞直徑"),
    ("rear_caliper_diameter", "後輪卡鉗活塞直徑"),
    ("disc_diameter", "碟盤直徑"),
    ("pad_mu", "來令片摩擦係數"),
    ("dc_voltage", "直流匯流排電壓"),
)
