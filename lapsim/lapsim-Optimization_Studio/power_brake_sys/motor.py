import numpy as np


def wheel_outputforce_omega(omega,car):#車輪輸出力量 分別輸出
    # 輪速轉馬達轉速 (前後分別計算)
    rpm = omega*car.train_rate * 60 / (2*np.pi)  # rpm
    tau_motor = car.get_motor_torque_data(rpm)# 查扭力
    # 輸出到輪上 (T * gear / R)
    tau_wheel = tau_motor*car.train_rate*car.gear_eta
    return tau_wheel

def Throttle_command(omega, car, cmd = 1):
    """
    依照油門命令輸出輪上扭矩

    Parameters
    ----------
    omega : array_like
        四輪輪速 (rad/s)

    throttle : float or array_like
        油門命令 0 ~ 1

    car : Car
        車輛參數

    Returns
    -------
    tau_wheel : ndarray
        四輪輪上扭矩 (N·m)
    """

    tau_max = wheel_outputforce_omega(omega,car)

    if cmd == 0.0:
        return tau_max * 0.0
    else:
        cmd = np.asarray(cmd, dtype=float)

        # 限制油門範圍
        cmd = np.clip(cmd,0.0,1.0)

        return tau_max * cmd

def Motor_output_torque(T_out,car):
    T_motor = T_out/car.train_rate/car.gear_eta
    return T_motor

