import numpy as np
from pathlib import Path
import os
import pandas as pd
from scipy.interpolate import interp1d

import numpy as np
from Suspension_sys.KC_sys import *
from Suspension_sys.sus_geo import *
from aero.Aero import ModalAeroModel
from power_brake_sys.Controller import TractionController

# 取得目前這個 .py 檔案的絕對路徑
base_path = Path(__file__).resolve().parent

# 組合資料路徑（以目前檔案為基準）
motor_data_path = base_path  / "power_brake_sys" / "excel" / "motor_speed_vs_torque.xlsx"

g = 9.81

def file_search(filename):#尋找檔案
    print("Opening file...")
    if not os.path.exists(filename):
        print("File not found:",filename)
    else:
        print("Successfully found:",filename)

def datasetup(file_name): 
    file_search(file_name)
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

class Car:
    """
    Vehicle parameters
    """

    def __init__(self):

        # ============================= Geometry
        self.l = 1.53 # wheelbase
        self.lf = 0.7498400000000001 # CG>>fa
        self.lr = self.l-self.lf # CG>>ra

        self.tf = 1.28 # front_track
        self.tr = 1.24 # rear_track

        self.h_cog = 0.3016451511 # 重心高 
        self.h_cg = self.h_cog

        self.wheel_pos = np.array([# 輪胎位置
            [ self.lf,  self.tf/2, 0.0],   # FL
            [ self.lf, -self.tf/2, 0.0],   # FR
            [-self.lr,  self.tr/2, 0.0],   # RL
            [-self.lr, -self.tr/2, 0.0],   # RR
        ])

        self.steer_limit_deg = 30.0
        self.steer_rate_limit_deg_s = 360.0
        
        # =============================(慣性)
        mu_f = 13
        mu_r = 13
        self.mu_list = np.array([mu_f,mu_f,mu_r,mu_r])

        self.m = 320
        self.ms = self.m-mu_f*2-mu_r*2

        self.I_rate = np.array([self.lr/self.l,self.lf/self.l])

        Ixx = 42
        Iyy = 105
        Izz = 130
        
        self.I = np.array([Ixx*self.I_rate[0],Ixx*self.I_rate[1],Iyy,Izz])

        self.w = self.m * g
        self.W = self.w

        self.cv_yaw = 0.0 # yaw damping
        # ============================(輪胎)
        self.tir = "tire.tir"
        self.free_radius = 205.73999999999998/1000 # m
        self.R_tire = self.free_radius
        Kt = 80.185*1000 # N/mm W8
        self.K_tire = Kt
        self.I_w = 0.2
        # W7 : 99.1699 N/mm

        # Tire relaxation length
        # 輪胎需要多少距離建立力量
        self.sigma_alpha = np.ones(4) * 0.4   # m
        self.sigma_kappa = np.ones(4) * 0.4   # m

        # =================================(控制器+傳動)
        self.motor_table = motor_torque_lookup
        self.gear_eta = 0.95
        self.train_rate = 11.2

        self.kp_sc = 250
        self.ki_sc = 0
        self.kd_sc = 0
        
        self.integral = 0.0
        self.prev_error = 0.0

        self.start_resis = 20

        self.T_rate = [1.0,1.0,1.0,1.0]
        self.drive_controllers = [
            TractionController(
                self,
                method="off",
                target=0.1,
                torque_min=0.0,
                torque_max=250
            )
            for _ in range(4)
        ]

        self.brake_controllers = [
            TractionController(
                self,
                method="off",
                target=-0.1,
                torque_min=-1000,
                torque_max=0.0
            )
            for _ in range(4)
        ]
        # =================================(煞車系統)
        # Initial SI-unit brake parameters. Replace these with measured
        # vehicle values when the brake model is calibrated.
        self.bb = np.array([0.65, 0.35])  # front/rear balance bar ratio
        self.mc = np.array([0.012, 0.012])  # master cylinder diameter (m)
        self.Disc = np.array([0.55, 0.17])  # pad friction coefficient, disc diameter (m)
        self.caliper = np.array([0.0254, 4, 0.0254, 2])  # piston diameter (m), piston count
        self.F_driver = 500.0  # driver pedal force (N)
        self.F_diver = self.F_driver  # backward compatible name used by brake.py
        self.PR = 2.8  # pedal ratio

        # =================================(空力系統)
        self.aero = ModalAeroModel(
            rho=1.225,
            area=1.2,

            CL0=-2.4,
            CD0=1.8,

            Kh=-5.0,
            Kp=20.0,
            Kr=5.0,
            Khh=50.0,
            Kpp=30.0,

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


        # =========================================== 避震器
        """
        1. 測試的時候先用LinearKC進行,之後要改其他的還有更多擴充
        2. corner_fl is for 直推,可以用這個來驗證車體的左右耦合是否正常
        3. 車輛單輪禿然受到大量衝擊是有可能扭轉1deg以上的
        """

        heave_f = LinearKC(k=56000/4,c=2700/4,name="heave_f")# 30000
        heave_r = LinearKC(k=44800/4,c=2240/4,name="heave_r")# 28000
        roll_f = LinearKC(k=80000/4,c=3500/4,name="roll_f")#25000
        roll_r = LinearKC(k=64000/4,c=3150/4,name="roll_r")#22000


        roll_c = NullKC()
        warp = NullKC()

        # 扭轉剛性需要換成rad 車架設定一個小小阻尼
        self.body_twist = LinearKC(k=180000,c=50,name="body_twist")
        
        corner_fl = NullKC()
        corner_fr = NullKC()
        corner_rl = NullKC()
        corner_rr = NullKC()

        # 輪胎保持一個小小阻尼
        tire_fl = LinearKC(k=Kt,c=50,name="tire_fl")
        tire_fr = LinearKC(k=Kt,c=50,name="tire_fr")
        tire_rl = LinearKC(k=Kt,c=50,name="tire_rl")
        tire_rr = LinearKC(k=Kt,c=50,name="tire_rr")



        self.shock_list = [heave_f,heave_r,roll_f,roll_r,roll_c,warp,
                           corner_fl,corner_fr,corner_rl,corner_rr]
        self.tire_list = [tire_fl,tire_fr,tire_rl,tire_rr]

        # ================================================= 懸吊幾何
        """
        1. 為了不在懸吊模型裡面玩方向猜猜樂必須要強制把方向在這邊訂好
        2. 幾何傳力設定到100%還是會發生姿態改變，這是因為輪胎被壓縮
        3. roll 和 pitch 設定到100% 給相同的正交輸入姿態角度會不同，這是因為軸距輪距差異和motion ratio 一樣成2次正比
        4. pitch 沒有輸入的時候也有初始姿態這是彈簧剛性與重心位置的問題
        """
        self.geometry = create_basic_geometry(

            self,# 注意傳入順序建議整個function往後掉
            # rc
            h_front=0.025,
            h_rear=0.05,
            # anti
            dive_front=0.15,
            lift_front=0.15,

            squat_rear=0.15,
            lift_rear=0.15,

            camber=np.zeros(4)
        )

        #self.geometry = create_fast_geometry(self)


    def get_motor_torque_data(self,speed_rpm):#取得馬達資料
        return motor_torque_lookup(speed_rpm)
