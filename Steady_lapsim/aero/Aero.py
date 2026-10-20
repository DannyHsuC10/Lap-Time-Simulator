import numpy as np


class ModalAeroModel:

    def __init__(
        self,
        rho=1.225,
        area=1.2,
        CL0=-3.0,

        Kh=0.0,
        Kp=0.0,
        Kr=0.0,

        Khh=0.0,
        Kpp=0.0,
        Krr=0.0,

        CD0=1.0,
        Kd_h=0.0,
        Kd_p=0.0,
        Kd_r=0.0,

        # Aero balance
        front_balance=0.45,

        K_balance_pitch=0.0,
        K_balance_roll=0.0,

        # vehicle geometry
        wheelbase=1.55,
        track=1.25,
        cop_z=0.0
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
