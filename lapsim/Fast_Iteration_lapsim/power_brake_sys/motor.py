import numpy as np

from power_brake_sys.TTR_LP03_EV_System_Model.ev_rom import LP03Model


MOTOR_MODEL_MODE = "excel"  # "excel" or "ev"
DEFAULT_EV_DT = 0.01
DEFAULT_EV_TORQUE_LIMIT_NM = 22.0


def _get_ev_model(car):
    ev = getattr(car, "_lp03_ev_model", None)
    if ev is None:
        ev = LP03Model(
            n_series=getattr(car, "ev_n_series", 144),
            n_parallel=getattr(car, "ev_n_parallel", 1),
            soc0=getattr(car, "ev_soc0", 0.95),
            T_cell0=getattr(car, "ev_T_cell0", 30.0),
            T_motor=getattr(car, "ev_T_motor", 70.0),
            T_air=getattr(car, "ev_T_air", 35.0),
        )
        car._lp03_ev_model = ev
    return ev


def reset_ev_model(car, soc0=None, T_cell0=None):
    """Reset the LP03 EV model state attached to a car object."""
    ev = getattr(car, "_lp03_ev_model", None)
    if ev is None:
        ev = _get_ev_model(car)
    ev.reset(
        getattr(car, "ev_soc0", 0.95) if soc0 is None else soc0,
        getattr(car, "ev_T_cell0", 30.0) if T_cell0 is None else T_cell0,
    )
    return ev


def wheel_outputforce_omega(omega,car):#車輪輸出力量 分別輸出
    # 輪速轉馬達轉速 (前後分別計算)
    rpm = omega*car.train_rate * 60 / (2*np.pi)  # rpm
    tau_motor = car.get_motor_torque_data(rpm)# 查扭力
    # 輸出到輪上 (T * gear / R)
    tau_wheel = tau_motor*car.train_rate*car.gear_eta
    return tau_wheel

def Throttle_command(omega, car, cmd = 1):
    """
    依照油門命令輸出輪上扭矩.

    Set MOTOR_MODEL_MODE at the top of this file to switch models:
    - "excel": original motor torque Excel interpolation
    - "ev": LP03 EV ROM model

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

    omega = np.asarray(omega, dtype=float)
    cmd = np.asarray(cmd, dtype=float)

    # 限制油門範圍
    cmd = np.clip(cmd,0.0,1.0)

    if not np.any(cmd):
        return np.zeros_like(omega, dtype=float)

    if MOTOR_MODEL_MODE == "excel":
        tau_max = wheel_outputforce_omega(omega, car)
        return tau_max * cmd

    if MOTOR_MODEL_MODE != "ev":
        raise ValueError(
            f"Unsupported MOTOR_MODEL_MODE {MOTOR_MODEL_MODE!r}; use 'excel' or 'ev'."
        )

    rpm = omega * car.train_rate * 60 / (2 * np.pi)
    torque_limit = getattr(car, "ev_torque_limit_nm", DEFAULT_EV_TORQUE_LIMIT_NM)
    T_cmd = np.broadcast_to(cmd * torque_limit, omega.shape)
    dt = getattr(car, "ev_dt", getattr(car, "power_dt", getattr(car, "dt", DEFAULT_EV_DT)))

    ev = _get_ev_model(car)
    tau_motor = ev.step(float(dt), T_cmd=T_cmd, rpm=rpm)
    tau_wheel = tau_motor * car.train_rate * car.gear_eta
    return tau_wheel

def Motor_output_torque(T_out,car):
    T_motor = T_out/car.train_rate/car.gear_eta
    return T_motor

