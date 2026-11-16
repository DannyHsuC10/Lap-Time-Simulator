"""Embed the Control_Sim Pygame surface in a Tk window."""

from copy import copy
import math
from time import perf_counter
import tkinter as tk
from tkinter import ttk

import pygame

from .car import Car
from .control_vehicle import VehicleParameters, VehicleState, draw_vehicle, handle_input, update_vehicle
from .planner import GRAVITY, PowerBrakeSystemLimit
from .tire import TabulatedTire


VEHICLE_FIELDS = (
    ("m", "車輛質量 (kg)"),
    ("Iz", "偏航轉動慣量 (kg·m²)"),
    ("lf", "重心至前軸 (m)"),
    ("lr", "重心至後軸 (m)"),
    ("Cf", "前軸側偏剛性 (N/rad)"),
    ("Cr", "後軸側偏剛性 (N/rad)"),
    ("engine_force", "驅動力 (N)"),
    ("brake_force", "煞車力 (N)"),
)
POSITIVE_FIELDS = {"m", "Iz", "lf", "lr"}
DRIVING_KEYS = {"Up": pygame.K_UP, "Down": pygame.K_DOWN, "Left": pygame.K_LEFT, "Right": pygame.K_RIGHT}
PIXELS_PER_METER = 15


class ControlSimWindow(tk.Toplevel):
    def __init__(self, master, settings):
        car = Car(settings)
        power_brake = PowerBrakeSystemLimit(car)
        params = VehicleParameters()
        params.m = car.m
        params.Iz = settings.yaw_inertia
        params.lf = car.lf
        params.lr = car.lr
        params.engine_force = float(power_brake.max_drive_force_distribution(0.0).sum())
        params.brake_force = float(-power_brake.max_brake_force_distribution().sum())
        params.max_speed = settings.max_speed
        self.tire_radius = settings.tire_radius
        self.rolling_coefficient = settings.rolling_coefficient
        self.rolling_tire = TabulatedTire(settings.tire_table_path) if settings.tire_model == "table" else None
        params.rolling_force = self.rolling_force_for(params)
        if car.aero is not None:
            params.drag_factor = car.aero.calculate_from_modal(1.0, 0.0, 0.0, 0.0)["F_drag"]
            if not math.isfinite(params.drag_factor) or params.drag_factor < 0:
                raise ValueError("空氣阻力係數須讓阻力為有限的非負值，請檢查空力係數檔。")

        if not pygame.font.get_init():
            pygame.font.init()
        super().__init__(master)
        self.title("LapSimLab — 車輛控制模擬")
        self.geometry("1040x600")
        self.minsize(780, 485)
        self.params = params
        self.initial_speed = min(settings.initial_speed, params.max_speed)
        self.vehicle = VehicleState(self.initial_speed)
        self.keys = dict.fromkeys(DRIVING_KEYS.values(), False)
        self.frame_job = None
        self.failed = False
        self.surface = None
        self.font = pygame.font.Font(None, 24)
        self.status = tk.StringVar(self)
        self.inputs = {}

        body = ttk.Frame(self, padding=14)
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        controls = ttk.LabelFrame(body, text="車輛參數", padding=12, style="Panel.TLabelframe")
        controls.grid(row=0, column=0, sticky="ns", padx=(0, 14))
        for row, (key, label) in enumerate(VEHICLE_FIELDS):
            ttk.Label(controls, text=label, style="Panel.TLabel").grid(row=row, column=0, sticky="w", pady=5)
            value = tk.StringVar(self, value=str(getattr(self.params, key)))
            entry = ttk.Entry(controls, textvariable=value, width=12)
            entry.grid(row=row, column=1, padx=(10, 0), pady=5)
            self.inputs[key] = (value, entry)

        speed_limit = f"速度上限 {params.max_speed:g} m/s（{params.max_speed * 3.6:g} km/h）"
        ttk.Label(controls, text=speed_limit, style="Hint.TLabel").grid(
            row=len(VEHICLE_FIELDS), column=0, columnspan=2, sticky="w", pady=(8, 0),
        )
        buttons = ttk.Frame(controls, style="Panel.TFrame")
        buttons.grid(row=len(VEHICLE_FIELDS) + 1, column=0, columnspan=2, sticky="ew", pady=(12, 8))
        ttk.Button(buttons, text="套用參數", command=self.apply_parameters).pack(side="left")
        ttk.Button(buttons, text="重設車輛", command=self.reset_vehicle).pack(side="left", padx=(8, 0))
        ttk.Label(controls, textvariable=self.status, style="Hint.TLabel", wraplength=250).grid(
            row=len(VEHICLE_FIELDS) + 2, column=0, columnspan=2, sticky="w",
        )

        view = ttk.Frame(body)
        view.grid(row=0, column=1, sticky="nsew")
        ttk.Label(view, text="點擊畫面後使用方向鍵駕駛。編輯參數時會暫停。", style="Muted.TLabel").pack(
            anchor="w", pady=(0, 6),
        )
        self.viewport = tk.Canvas(view, background="#f0f0f0", highlightthickness=0, takefocus=True)
        self.viewport.pack(fill="both", expand=True)
        self.image = tk.PhotoImage(master=self.viewport)
        self.viewport.create_image(0, 0, image=self.image, anchor="nw")
        self.viewport.bind("<Configure>", self.resize_surface)
        self.viewport.bind("<Button-1>", lambda _event: self.viewport.focus_set())
        self.viewport.bind("<FocusIn>", self.start_driving)
        self.viewport.bind("<FocusOut>", self.stop_driving)
        self.viewport.bind("<KeyPress>", self.press_key)
        self.viewport.bind("<KeyRelease>", self.release_key)

    def show(self):
        self.deiconify()
        self.lift()
        self.viewport.focus_set()

    def rolling_force_for(self, params):
        weight = params.m * GRAVITY
        if self.rolling_tire is None:
            return self.rolling_coefficient * weight
        wheelbase = params.lf + params.lr
        front_load = 0.5 * weight * (params.lr / wheelbase)
        rear_load = 0.5 * weight * (params.lf / wheelbase)
        front_torque, rear_torque = self.rolling_tire.rolling_torque([front_load, rear_load])
        return float(2 * (front_torque + rear_torque) / self.tire_radius)

    def apply_parameters(self):
        params = copy(self.params)
        for key, label in VEHICLE_FIELDS:
            variable, entry = self.inputs[key]
            try:
                value = float(variable.get())
            except ValueError:
                value = math.nan
            positive = key in POSITIVE_FIELDS
            if not math.isfinite(value) or value < 0 or (positive and value == 0):
                self.status.set(f"{label}請輸入{'正數' if positive else '非負數'}。")
                entry.focus_set()
                return
            setattr(params, key, value)
        params.rolling_force = self.rolling_force_for(params)
        self.params = params
        self.failed = False
        self.status.set("已套用車輛參數。")
        self.resume_driving()

    def reset_vehicle(self):
        self.stop_driving()
        self.vehicle = VehicleState(self.initial_speed)
        self.failed = False
        self.status.set("車輛已回到起點。")
        self.render()
        self.resume_driving()

    def resume_driving(self):
        self.viewport.focus_set()
        if self.focus_get() is self.viewport:
            self.start_driving()

    def press_key(self, event):
        if event.keysym in DRIVING_KEYS:
            self.keys[DRIVING_KEYS[event.keysym]] = True
            return "break"

    def release_key(self, event):
        if event.keysym in DRIVING_KEYS:
            self.keys[DRIVING_KEYS[event.keysym]] = False
            return "break"

    def start_driving(self, _event=None):
        if self.frame_job is None and not self.failed:
            self.last_frame = perf_counter()
            self.frame_job = self.after(16, self.advance)

    def stop_driving(self, _event=None):
        if self.frame_job is not None:
            self.after_cancel(self.frame_job)
            self.frame_job = None
        self.keys = dict.fromkeys(DRIVING_KEYS.values(), False)

    def advance(self):
        self.frame_job = None
        now = perf_counter()
        dt = min(now - self.last_frame, 0.05)
        self.last_frame = now
        vehicle = copy(self.vehicle)
        throttle, brake = handle_input(self.keys, vehicle, dt)
        # Smaller Euler steps keep the original model stable near zero speed.
        steps = max(1, math.ceil(dt * 600))
        try:
            for _ in range(steps):
                update_vehicle(vehicle, self.params, throttle, brake, dt / steps)
        except (ValueError, OverflowError):
            self.stop_after_error()
            return
        if not all(math.isfinite(value) for value in vars(vehicle).values()):
            self.stop_after_error()
            return
        self.vehicle = vehicle
        self.render()
        self.frame_job = self.after(16, self.advance)

    def stop_after_error(self):
        self.failed = True
        self.stop_driving()
        self.status.set("車輛狀態超出計算範圍。請調整參數或重設車輛。")

    def resize_surface(self, event):
        self.surface = pygame.Surface((event.width, event.height))
        self.render()

    def render(self):
        if self.surface is None:
            return
        width, height = self.surface.get_size()
        camera = (self.vehicle.x, self.vehicle.y)
        draw_vehicle(
            self.surface, self.vehicle, PIXELS_PER_METER, self.font, width, height,
            camera=camera, params=self.params,
        )
        # Tk accepts the RGB buffer directly as a binary PPM, without a second SDL window.
        header = f"P6\n{width} {height}\n255\n".encode("ascii")
        self.image.configure(data=header + pygame.image.tobytes(self.surface, "RGB"), format="PPM")

    def destroy(self):
        self.stop_driving()
        super().destroy()
