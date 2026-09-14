import numpy as np

class TractionController:

    def __init__(
        self,
        car,
        method="bangbang",
        target=None,
        torque_min=None,
        torque_max=None
    ):

        self.method = method

        self.target = car.target_slip if target is None else target

        self.kp = car.kp_sc
        self.ki = car.ki_sc
        self.kd = car.kd_sc

        self.torque_max = car.torque_max if torque_max is None else torque_max
        self.torque_min = car.torque_min if torque_min is None else torque_min

        self.integral = car.integral
        self.prev_error = car.prev_error

    def reset(self):

        self.integral = 0
        self.prev_error = 0

    def update(self, slip, dt, base_torque):
        
        if base_torque == 0.0:
            return base_torque
        
        error = self.target - slip

        if self.method == "off":
            torque = base_torque

        # ------------------------
        # Bang Bang
        # ------------------------

        if self.method == "bangbang":

            if base_torque > 0:
                if slip > self.target:
                    torque = 0

                else:
                    torque = base_torque

            elif base_torque < 0:
                if slip < self.target:
                    torque = 0

                else:
                    torque = base_torque

            else:
                torque = 0

        # ------------------------
        # P
        # ------------------------

        elif self.method == "P":

            torque = base_torque + self.kp * error

        # ------------------------
        # PI
        # ------------------------

        elif self.method == "PI":

            self.integral += error * dt

            torque = (base_torque
                      + self.kp * error
                      + self.ki * self.integral)

        # ------------------------
        # PID
        # ------------------------

        elif self.method == "PID":

            self.integral += error * dt

            derivative = (error-self.prev_error)/dt

            torque = (base_torque
                      + self.kp * error
                      + self.ki * self.integral
                      + self.kd * derivative)

            self.prev_error = error

        else:

            torque = base_torque

        torque = np.clip(torque,
                         self.torque_min,
                         self.torque_max)

        return torque
