from pathlib import Path
import os
import pandas as pd
from scipy.interpolate import interp1d

import numpy as np
from aero.Aero import ModalAeroModel

# 取得目前這個 .py 檔案的絕對路徑
base_path = Path(__file__).resolve().parent

# 組合資料路徑（以目前檔案為基準）
motor_data_path = base_path  / "power_brake_sys" / "excel" / "motor_speed_vs_torque.xlsx"

g = 9.81

def datasetup(file_name): 

    df_raw = pd.read_excel(file_name, header=None)
    name_a = df_raw.iloc[0, 0]  # A1
    name_b = df_raw.iloc[0, 1]  # B1
    df = pd.read_excel(file_name)
    # 取出欄位
    a_array = df[name_a].values
    b_array = df[name_b].values
    lookup = interp1d(a_array, b_array, bounds_error=False, fill_value="extrapolate")
    return lookup

motor_torque_lookup = datasetup(motor_data_path)


class Car_simplify:
    """
    Minimal vehicle parameters used by theoretical_lap_simulator.py.
    """

    def __init__(self):
        
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
        self.aero = ModalAeroModel(
            rho=1.225,
            area=1.2,

            CL0=-2.4,
            CD0=1.8,

            Kh=-5.0,
            Kp=25.0,
            Kr=5.0,
            Khh=50.0,
            Kpp=40.0,

            Kd_h=0.0,
            Kd_p=0.0,
            Kd_r=0.0,

            front_balance=0.45,
            K_balance_pitch=0.20,
            K_balance_roll=0.10,

            wheelbase=self.l,
            track=(self.tf + self.tr) / 2.0,
            cop_z=0.0
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
