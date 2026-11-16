import json
import math
from pathlib import Path

class ModalAeroModel:

    @classmethod
    def from_json(cls, path: str | Path, *, wheelbase: float, track: float,
                  cl0: float | None = None, cd0: float | None = None):
        path = Path(path)
        try:
            coefficients = json.loads(path.read_text(encoding="utf-8-sig"))
            if not isinstance(coefficients, dict):
                raise ValueError("expected an object with aerodynamic coefficients")
            for name, value in coefficients.items():
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(f"{name} must be a finite number")
            if cl0 is not None:
                coefficients["CL0"] = cl0
            if cd0 is not None:
                coefficients["CD0"] = cd0
            return cls(**coefficients, wheelbase=wheelbase, track=track)
        except (ValueError, TypeError, OverflowError) as error:
            raise ValueError(f"Invalid aero coefficients in {path}: {error}") from error

    def __init__(
        self,
        *,
        rho,                    # 空氣密度 (kg/m3)
        area,                   # 參考面積 (m2)
        CL0,
        CD0,
        Kh,                     # Heave 敏感度 (1/m)
        Kp,                     # Pitch 敏感度 (1/rad)
        Kr,                     # Roll 敏感度 (1/rad)
        Khh,
        Kpp,
        Krr,
        Kd_h,
        Kd_p,
        Kd_r,
        front_balance,
        K_balance_pitch,
        K_balance_roll,
        wheelbase,              # 軸距 (m)
        track,                  # 平均輪距 (m)
        cop_z,                  # 空力中心高度 (m)
    ):

        self.rho = rho
        self.area = area

        self.CL0 = CL0

        self.Kh = Kh
        self.Kp = Kp
        self.Kr = Kr

        self.Khh = Khh
        self.Kpp = Kpp
        self.Krr = Krr

        self.CD0 = CD0
        self.Kd_h = Kd_h
        self.Kd_p = Kd_p
        self.Kd_r = Kd_r

        # aero balance sensitivity
        self.front_balance = front_balance
        self.K_balance_pitch = K_balance_pitch
        self.K_balance_roll = K_balance_roll

        self.L = wheelbase
        self.track = track

        self.cop_z = cop_z

    def calculate_from_modal(self, speed, qh, qp, qr):
        cl = (
            self.CL0 + self.Kh * abs(qh) + self.Kp * abs(qp) + self.Kr * abs(qr)
            + self.Khh * qh**2 + self.Kpp * qp**2 + self.Krr * qr**2
        )
        cd = self.CD0 + self.Kd_h * abs(qh) + self.Kd_p * abs(qp) + self.Kd_r * abs(qr)
        downforce = -0.5 * self.rho * self.area * abs(speed)**2 * cl
        drag = 0.5 * self.rho * self.area * abs(speed)**2 * cd

        front_ratio = self.front_balance + self.K_balance_pitch * qp
        left_ratio = 0.5 + self.K_balance_roll * qr
        front, rear = downforce * front_ratio, downforce * (1.0 - front_ratio)
        fl, fr = front * left_ratio, front * (1.0 - left_ratio)
        rl, rr = rear * left_ratio, rear * (1.0 - left_ratio)

        cop_x = cop_y = 0.0
        if downforce != 0:
            x, y = self.L / 2, self.track / 2
            cop_x = (x * fl + x * fr - x * rl - x * rr) / downforce
            cop_y = (y * fl - y * fr + y * rl - y * rr) / downforce

        return {
            "F_down": downforce,
            "F_drag": drag,
            "F_FL": fl,
            "F_FR": fr,
            "F_RL": rl,
            "F_RR": rr,
            "COP_x": cop_x,
            "COP_y": cop_y,
            "CL": cl,
            "CD": cd,
            "qh": qh,
            "qp": qp,
            "qr": qr,
        }


class QuadraticAeroModel(ModalAeroModel):
    """Use the zero-attitude coefficients and balance at every speed."""

    def calculate_from_modal(self, speed, qh, qp, qr):
        return super().calculate_from_modal(speed, 0.0, 0.0, 0.0)
