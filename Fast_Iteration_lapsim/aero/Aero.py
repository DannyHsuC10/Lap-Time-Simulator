import numpy as np


class ModalAeroModel:

    def __init__(
        self,
        rho=1.225,              # 空氣密度 (kg/m3)
        area=1.2,               # 參考面積 (m2)
        
        CL0=-2.4,               # 基本CL
        
        Kh=-5.0,               # Heave敏感度 (1/m)
        Kp=0.2,                # Pitch敏感度 (1/m)
        Kr=0.0,                 # Roll敏感度
        
        Khh=50.0,              # Heave二階項
        Kpp=30.0,              # Pitch二階項
        Krr=8.0,              # Roll二階項

        CD0=1.8,                # 基本CD

        Kd_h=0.0,
        Kd_p=0.0,
        Kd_r=0.0,
        
        front_balance=0.45,     # 45%前翼、55%後翼
        K_balance_pitch=0.20,   # Pitch造成Balance改變
        K_balance_roll=0.10,    # Roll造成左右Balance改變
        
        wheelbase=1.55,         # 軸距 (m)
        track=1.25,              # 軌距 (m)
        cop_z=0.0,              # 空力中心高度 (m)
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

        self.cop_z=cop_z

    def modal_coordinate(self,z):

        zFL,zFR,zRL,zRR = z


        heave = np.mean(z)


        pitch = (
            (zFL+zFR)/2 -
            (zRL+zRR)/2
        )


        roll = (
            (zFL+zRL)
            -
            (zFR+zRR)
        )/2


        return heave,pitch,roll



    def state_modal_coordinate(self, state):

        heave = state.s[2]
        pitch = state.theta[2]
        roll = (
            state.theta[0]
            +
            state.theta[1]
        ) / 2.0

        return heave,pitch,roll


    def calculate_from_modal(self,V,qh,qp,qr):


        V = abs(V)


        # ==========================
        # Total CL
        # ==========================

        CL = (
            self.CL0
            +
            self.Kh*abs(qh)
            +
            self.Kp*abs(qp)
            +
            self.Kr*abs(qr)

            +
            self.Khh*qh**2
            +
            self.Kpp*qp**2
            +
            self.Krr*qr**2
        )

        CD = (
            self.CD0
            +
            self.Kd_h*abs(qh)
            +
            self.Kd_p*abs(qp)
            +
            self.Kd_r*abs(qr)
        )


        F_down = (
            -0.5
            *
            self.rho
            *
            self.area
            *
            V**2
            *
            CL
        )

        F_drag = (
            0.5
            *
            self.rho
            *
            self.area
            *
            V**2
            *
            CD
        )


        # ==========================
        # Front / Rear distribution
        # ==========================

        front_ratio = (
            self.front_balance
            +
            self.K_balance_pitch*qp
        )


        rear_ratio = 1-front_ratio



        F_front = F_down*front_ratio
        F_rear  = F_down*rear_ratio



        # ==========================
        # Left / Right distribution
        # ==========================

        left_ratio = (
            0.5
            +
            self.K_balance_roll*qr
        )


        right_ratio = 1-left_ratio



        # 四輪 aero load

        F_FL = F_front*left_ratio
        F_FR = F_front*right_ratio

        F_RL = F_rear*left_ratio
        F_RR = F_rear*right_ratio



        # ==========================
        # COP calculation
        # ==========================

        x_FL = self.L/2
        x_FR = self.L/2

        x_RL = -self.L/2
        x_RR = -self.L/2


        y_FL = self.track/2
        y_RL = self.track/2

        y_FR = -self.track/2
        y_RR = -self.track/2


        if F_down == 0:
            COP_x = 0
        else:
            COP_x = (
            x_FL*F_FL
            +
            x_FR*F_FR
            +
            x_RL*F_RL
            +
            x_RR*F_RR
        ) / F_down



        if F_down == 0:
            COP_y = 0
        else:
            COP_y = (
                y_FL*F_FL
                +
                y_FR*F_FR
                +
                y_RL*F_RL
            +
            y_RR*F_RR
        ) / F_down



        return {


            "F_down":F_down,
            "F_drag":F_drag,

            "F_FL":F_FL,
            "F_FR":F_FR,
            "F_RL":F_RL,
            "F_RR":F_RR,


            "COP_x":COP_x,
            "COP_y":COP_y,


            "CL":CL,
            "CD":CD,

            "qh":qh,
            "qp":qp,
            "qr":qr

        }


    def calculate(self,V,z):


        qh,qp,qr = self.modal_coordinate(z)

        return self.calculate_from_modal(V,qh,qp,qr)


    def calculate_from_state(self,state):

        qh,qp,qr = self.state_modal_coordinate(state)
        V = np.hypot(state.v[0], state.v[1])

        return self.calculate_from_modal(V,qh,qp,qr)
