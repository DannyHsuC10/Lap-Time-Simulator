# 順態輪胎模型
if __name__ == "__main__":
    from model import api as tire_model
else:
    from Pacejka_tire_model.model import api as tire_model
    from car import Car
    from N_order_hold import FirstOrder
import numpy as np
import matplotlib.pyplot as plt



g = 9.81

class tire:
    """
    tire parameters
    """

    def __init__(self,car,transient = True):

        self.enable_transient = transient

        self.car = car
        self.tir_params = tire_model.load_tir(car.tir)
        self.K_tire = car.K_tire
        self.free_radius = car.free_radius
        
        self.r_w = self.loaded_radius(np.zeros(4))# 初始半徑先定義成無負載工作半徑
        
        self.vx_w = np.zeros(4)
        self.vy_w = np.zeros(4)
        self.speed_w = np.zeros(4)

        self.alpha = np.zeros(4)
        self.kappa = np.zeros(4)

        # =====================================
        # Tire relaxation length
        # =====================================
        # 輪胎需要多少距離建立力量

        self.sigma_alpha = car.sigma_alpha   # m
        self.sigma_kappa = car.sigma_kappa   # m

        self.dt = 0.001
        # =====================================
        # Transient tire state
        # =====================================

        self.alpha_filter = FirstOrder(tau=np.ones(4),x0=np.zeros(4))
        self.kappa_filter = FirstOrder(tau=np.ones(4),x0=np.zeros(4))

        
    def loaded_radius(self,Fz):# 工作半徑
        self.r_w = np.array(self.free_radius - Fz / self.K_tire)
        return self.r_w

    def wheel_ground_velocity(self,vx, vy, yaw_rate):# 取得地面速度
        
        for i, (x, y, z) in enumerate(self.car.wheel_pos):

            self.vx_w[i] = vx - yaw_rate * y
            self.vy_w[i] = vy + yaw_rate * x

            self.speed_w[i] = np.hypot(self.vx_w[i], self.vy_w[i])

        return self.vx_w, self.vy_w, self.speed_w

    def transient_slip(self, SA_cmd, SL_cmd, dt):

        # 避免低速時 tau 無限大
        V = np.maximum(self.speed_w, 0.5)

        # ============================
        # relaxation time constant
        # ============================
        self.alpha_filter.tau = (self.sigma_alpha / V)
        self.kappa_filter.tau = (self.sigma_kappa / V)

        # ============================
        # first order response
        # ============================

        SA = self.alpha_filter.update(SA_cmd,dt)
        SL = self.kappa_filter.update(SL_cmd,dt)

        return SA, SL

    def slip_angle_rad(self, steer): # 滑移角
        """
        steer: 前輪轉角
        vx: x前進速度
        vy: y側向速度
        omega_yaw: 車輛偏航角速度
        l: 車輛重心到軸距
        """
        # array運算
        steer = np.deg2rad(np.array(steer))
        vx_safe = np.maximum(np.abs(self.vx_w), 0.1)*np.sign(self.vx_w)

        #alpha = steer - np.arctan((vy + l * omega_yaw) / vx) # 車輛坐標系(直接用轉換過的)
        #mask = np.abs(self.vx_w) < 0.1
        alpha = steer - np.arctan2(self.vy_w, vx_safe) # 車輪坐標系

        alpha_lim = np.deg2rad(12)
        #alpha = np.clip(alpha,-alpha_lim,alpha_lim)
        #alpha[mask] = 0.0

        self.alpha = alpha

        return self.alpha

    def slip_ratio(self, omega,steer): # 滑移率
        """
        vx: x前進速度
        omega: 車輪角速度
        r_w: 車輪半徑
        """
        # array運算
        omega = np.array(omega)
        # omega 項主導
        steer = np.deg2rad(np.array(steer))
        vx = self.vx_w*np.cos(steer) + self.vy_w*np.sin(steer)# 加入轉向角度的影響

        self.kappa = (omega*self.r_w-vx)/np.maximum(np.abs(vx), 0.01)
        self.kappa = np.clip(self.kappa,-1,1)
        return self.kappa

    def Four_wheel_output(self,FZ_list, SA_list, SL_list, IA_list = None, P_list = None):# 四輪輸出
        """
        輸入:
            FZ_list : 四輪的垂直載荷 [fl, fr, rl, rr]
            SA_list : 四輪的側滑角 [fl, fr, rl, rr] 只吃degrees
            SL_list : 四輪的縱向滑移 [fl, fr, rl, rr]
            IA_list : 四輪的傾斜角 [fl, fr, rl, rr] (可選)
            P_list  : 四輪的胎壓 [fl, fr, rl, rr] (可選)
            V_list  : 四輪的速度 [fl, fr, rl, rr] (可選)
            Ro_list : 四輪的接地半徑 [fl, fr, rl, rr] (可選)
            tire_params : 輪胎參數物件
            check : 是否輸出中間結果 (預設 False)
        輸出:
            Fx, Fy, Mx, Mz, RRT : 四輪的力矩
        """

        
        SA_list_copy = np.array(SA_list, copy=True)
        
        SA_list_copy[0] = -SA_list_copy[0] # 右輪轉角與左輪相反
        SA_list_copy[2] = -SA_list_copy[2] # 右輪轉角與左輪相反

        SA_list_copy = np.degrees(SA_list_copy)

        MF = tire_model.CalculationInput( # 建立MF物件函數
                    FZ=FZ_list,
                    SA=SA_list_copy,
                    SL=SL_list,     
                    IA=IA_list,
                    pressure=P_list,
                    V = self.speed_w,# 特殊項
                    Ro = self.r_w,# 特殊項
                    tir_params=self.tir_params
                )

        Fx = tire_model.MF_Universal_solver("FX", MF) # 同向
        Fy = tire_model.MF_Universal_solver("FY", MF)
        Fz = np.array(FZ_list) # 同向
        Mx = tire_model.MF_Universal_solver("MX", MF)
        Mz = tire_model.MF_Universal_solver("MZ", MF)
        RRT = tire_model.MF_Universal_solver("RRT", MF) # 同向

        Fy = np.array([Fy[0],-Fy[1],Fy[2],-Fy[3]]) # 統一坐標系方向
        Mz = np.array([Mz[0],-Mz[1],Mz[2],-Mz[3]]) # 統一坐標系方向
        Mx = np.array([-Mx[0],Mx[1],-Mx[2],Mx[3]]) # 統一坐標系方向

        return Fx, Fy, Fz, Mx, RRT, Mz



    def tire_force(
        self,
        vx,
        vy,
        yaw_rate,
        steer,
        omega_w,
        FZ,
        IA=None,
        pressure=None,
        SL=None,
        SA=None,
        dt=None,
        state=None
    ):
        
        """
        Parameters
        ----------
        vx, vy : CG velocity
        yaw_rate : vehicle yaw rate
        steer : [FL, FR, RL, RR] steering angle (rad)
        wheel_speed : wheel angular velocity (rad/s)
        Fz : vertical load (N)
        """
        vx_w, vy_w, speed_w = self.wheel_ground_velocity(vx, vy, yaw_rate)

        self.loaded_radius(FZ)# 輪胎半徑
        if state is not None:
            state.loaded_radius = self.r_w.copy()

        if SA is None:
            SA = self.slip_angle_rad(steer)# 滑移角度
        if SL is None:
            SL = self.slip_ratio(omega_w,steer)# 滑移率

        if dt is not None:
            self.dt = dt

        if self.enable_transient:
            SA, SL = self.transient_slip(SA,SL,self.dt)

        # Pacejka
        Fx, Fy, Fz, Mx, RRT, Mz = self.Four_wheel_output(FZ, SA, SL, IA_list = IA, P_list = pressure)

        return Fx, Fy, Fz, Mx, Mz, RRT, SA, SL

# test
# =====================================================
import numpy as np
import matplotlib.pyplot as plt

#from tire import tire
from car import Car


if __name__ == "__main__":


    # =================================
    # simulation parameters
    # =================================

    dt = 0.001
    t_end = 0.5

    time = np.arange(
        0,
        t_end,
        dt
    )


    # =================================
    # create tire
    # =================================

    car = Car()

    tire_steady = tire(
        car,
        transient=False
    )


    tire_dynamic = tire(
        car,
        transient=True
    )


    # =================================
    # vehicle condition
    # =================================

    vx = 20.0
    vy = 0.0
    yaw_rate = 0.0


    loaded_radius = tire_dynamic.loaded_radius(
        np.zeros(4)
    )
    omega = np.ones(4) * (
        vx / loaded_radius
    )


    Fz = np.array([800,800,800,800])


    # =================================
    # steering input
    # =================================

    steer = np.zeros((len(time),4))


    steer[
        time>=0.1,
        0
    ] = np.radians(5)


    steer[
        time>=0.1,
        1
    ] = np.radians(5)



    # =================================
    # storage
    # =================================
    Fy_steady=[]
    Fy_dynamic=[]
    SA_steady=[]
    SA_dynamic=[]
    # =================================
    # simulation loop
    # =================================

    for i,t in enumerate(time):


        result_s = tire_steady.tire_force(
            vx=vx,
            vy=vy,
            yaw_rate=yaw_rate,
            steer=steer[i],
            omega_w=omega,
            FZ=Fz,
            dt=dt
        )

        result_d = tire_dynamic.tire_force(
            vx=vx,
            vy=vy,
            yaw_rate=yaw_rate,
            steer=steer[i],
            omega_w=omega,
            FZ=Fz,
            dt=dt
        )

        Fy_steady.append(
            result_s[1]
        )

        Fy_dynamic.append(
            result_d[1]
        )

        SA_steady.append(
            result_s[6]
        )

        SA_dynamic.append(
            result_d[6]
        )

    Fy_steady=np.array(Fy_steady)
    Fy_dynamic=np.array(Fy_dynamic)
    SA_steady = np.array(SA_steady)
    SA_dynamic = np.array(SA_dynamic)

    # =================================
    # plot alpha
    # =================================


    plt.figure(figsize=(8,4))


    plt.plot(
        time,
        np.degrees(SA_steady[:,0]),
        label="alpha command"
    )


    plt.plot(
        time,
        np.degrees(SA_dynamic[:,0]),
        label="alpha transient"
    )


    plt.xlabel("Time (s)")
    plt.ylabel("Slip angle (deg)")


    plt.grid()
    plt.legend()


    plt.show()

    # =================================
    # plot Fy
    # =================================


    plt.figure(figsize=(8,4))


    plt.plot(
        time,
        Fy_steady[:,0],
        label="steady"
    )


    plt.plot(
        time,
        Fy_dynamic[:,0],
        label="transient"
    )


    plt.xlabel("Time (s)")
    plt.ylabel("Lateral force (N)")


    plt.grid()
    plt.legend()


    plt.show()
