from pathlib import Path
import pandas as pd
from scipy.interpolate import interp1d

import numpy as np
from Aero import DEFAULT_AERO_JSON, ModalAeroModel

motor_data_path = (
    Path(__file__).resolve().parents[0] / "input_data" / "motor_speed_vs_torque.csv"
)

g = 9.81

def datasetup(file_name):
    df = pd.read_csv(file_name)
    return interp1d(df["speed"], df["torque"], bounds_error=False, fill_value="extrapolate")

motor_torque_lookup = datasetup(motor_data_path)


class Car_simplify:
    """
    Minimal vehicle parameters used by theoretical_lap_simulator.py.
    """

    def __init__(self, aero_path: str | Path = DEFAULT_AERO_JSON):
        
        self.tf = 1.25 # front_track
        self.tr = 1.24 # rear_track
        # ============================= Geometry
        self.l = 1.53 # wheelbase
        self.lf = 0.7498400000000001 # CG>>fa
        self.lr = self.l-self.lf # CG>>ra

        self.h_cog = 0.3016451511 # 重心高
        self.h_cg = self.h_cog#

        # ============================= Mass
        self.m = 320#
        self.Izz = 130#

        # ============================(輪胎)
        self.free_radius = 205.73999999999998/1000 # m
        self.R_tire = self.free_radius# 
        Kt = 80.185*1000 # N/mm W8
        self.K_tire = Kt# 

        # =================================(控制器+傳動)
        self.motor_table = motor_torque_lookup# 
        self.gear_eta = 0.95# 
        self.train_rate = 11.2# 根據直線加速

        self.T_rate = [0.67,0.67,1.0,1.0]

        # =================================(煞車系統)
        self.brake_bias = np.array([0.7, 0.3])  # front/rear brake ratio
        self.brake_mechanical_advantage = 12.6
        self.F_diver = 500.0  # backward compatible name used by brake.py

        # =================================(空力系統)
        self.aero = ModalAeroModel.from_json(
            aero_path,
            wheelbase=self.l,
            track=(self.tf + self.tr) / 2.0,
        )

        # =========================================== 穩態懸吊剛性
        # Heave stiffness is per axle; roll stiffness is per wheel and is
        # converted to axle roll stiffness in Suspension_sys.sus_load.
        self.k_heave_front =  140000  # N/m
        self.k_heave_rear =  140000  # N/m
        self.k_roll_front = 100000  # N/m per wheel
        self.k_roll_rear = 100000  # N/m per wheel

    def get_motor_torque_data(self,speed_rpm):#取得馬達資料
        return motor_torque_lookup(speed_rpm)
