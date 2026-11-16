"""Map workbench inputs to the original LapSim vehicle parameters."""

from functools import cached_property

import numpy as np

from .aero import ModalAeroModel, QuadraticAeroModel
from .motor import MotorCurve, VoltageMotorCurve
from .settings import Settings


class Car:
    def __init__(self, settings: Settings, *, preload_motor: bool = True, aero_coefficients=None):
        self.settings = settings
        self.m = settings.mass
        self.tf = settings.front_track
        self.tr = settings.rear_track
        self.l = settings.wheelbase
        self.lf = settings.cg_from_front
        self.lr = self.l - self.lf
        self.h_cg = settings.cg_height
        self.free_radius = self.R_tire = settings.tire_radius
        self.K_tire = settings.tire_stiffness

        if preload_motor:
            self.motor_curve = self.load_motor_curve()
        self.gear_eta = settings.gear_efficiency
        self.train_rate = settings.gear_ratio
        front, rear = settings.front_torque_scale, settings.rear_torque_scale
        self.T_rate = np.array([front, front, rear, rear])

        self.brake_bias = np.array([settings.front_brake_bias, 1.0 - settings.front_brake_bias])
        self.brake_mechanical_advantage = settings.brake_leverage
        self.F_diver = settings.pedal_force

        self.k_heave_front = settings.front_heave_stiffness
        self.k_heave_rear = settings.rear_heave_stiffness
        self.k_roll_front = settings.front_roll_stiffness
        self.k_roll_rear = settings.rear_roll_stiffness
        self.aero = None
        if settings.aero_enabled:
            aero_class = QuadraticAeroModel if settings.aero_model == "quadratic" else ModalAeroModel
            track = (self.tf + self.tr) / 2.0
            if aero_coefficients is None:
                self.aero = aero_class.from_json(settings.aero_path, wheelbase=self.l, track=track)
            else:
                self.aero = aero_class.from_coefficients(aero_coefficients, wheelbase=self.l, track=track)

    @cached_property
    def motor_curve(self):
        return self.load_motor_curve()

    def load_motor_curve(self):
        if self.settings.motor_model == "lp03":
            return VoltageMotorCurve(self.settings.lp03_path, self.settings.dc_voltage)
        return MotorCurve(self.settings.motor_path)

    def get_motor_torque_data(self, speed_rpm):
        return self.motor_curve.torque_at(speed_rpm)
