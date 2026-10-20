import numpy as np
from pathlib import Path
from dataclasses import dataclass

OUTPUT_DIR = Path(__file__).parent


@dataclass(frozen=True)
class WheelLoadStiffnessResult:
    loads: np.ndarray
    theta: np.ndarray
    heave: float
    roll_angle: float
    pitch_angle: float
    qh: float
    qp: float
    qr: float


def wheel_load_stiffness(
    mass,
    g,
    aero,
    ax,
    ay,
    cg_height,
    wheelbase,
    front_ratio,
    track_front,
    track_rear,
    k_heave_front,
    k_heave_rear,
    k_roll_front,
    k_roll_rear,
    yaw=0.0,
    return_state=False,
):
    """
    Steady-state wheel loads using suspension stiffness.

    Parameters
    ----------
    mass : float
    g : float
    aero : float
        Total downforce [N]
    ax : float
    ay : float
    cg_height : float
    wheelbase : float
    front_ratio : float
        Static front weight ratio
    track_front : float
    track_rear : float

    k_heave_front : float
        Front axle heave stiffness [N/m]

    k_heave_rear : float
        Rear axle heave stiffness [N/m]

    k_roll_front : float
        Front axle roll stiffness [Nm/rad]

    k_roll_rear : float
        Rear axle roll stiffness [Nm/rad]

    yaw : float
        Vehicle heading [rad], passed through to theta[3].

    return_state : bool
        False keeps the historical return value: ndarray [FL, FR, RL, RR].
        True returns WheelLoadStiffnessResult with loads and estimated attitude.

    Returns
    -------
    ndarray or WheelLoadStiffnessResult
        Wheel loads, optionally with simplified steady-state attitude.
    """

    # -------------------------------------------------------
    # Static load
    # -------------------------------------------------------

    W = mass * g + aero

    F_front_static = W * front_ratio
    F_rear_static = W - F_front_static

    # -------------------------------------------------------
    # Pitch load transfer
    # -------------------------------------------------------

    M_pitch = mass * ax * cg_height

    K_pitch = k_heave_front + k_heave_rear

    pitch_front = M_pitch * k_heave_front / K_pitch
    pitch_rear = M_pitch * k_heave_rear / K_pitch

    dF_front = pitch_front / wheelbase
    dF_rear = pitch_rear / wheelbase

    F_front = F_front_static - dF_front
    F_rear = F_rear_static + dF_rear

    # -------------------------------------------------------
    # Roll load transfer
    # -------------------------------------------------------

    M_roll = mass * ay * cg_height

    K_roll = k_roll_front + k_roll_rear

    roll_front = M_roll * k_roll_front / K_roll
    roll_rear = M_roll * k_roll_rear / K_roll

    dF_front_roll = roll_front / track_front
    dF_rear_roll = roll_rear / track_rear

    # +ay = 左轉
    FL = F_front / 2 - dF_front_roll
    FR = F_front / 2 + dF_front_roll

    RL = F_rear / 2 - dF_rear_roll
    RR = F_rear / 2 + dF_rear_roll

    K_roll_front = 2 * k_roll_front * (track_front/2)**2
    K_roll_rear  = 2 * k_roll_rear * (track_rear/2)**2

    roll_angle = M_roll / (K_roll_front + K_roll_rear)

    K_pitch = k_heave_front + k_heave_rear

    pitch_angle = M_pitch / K_pitch

    heave = aero / K_pitch

    loads = np.array([FL, FR, RL, RR])
    theta = np.array([roll_angle, roll_angle, pitch_angle, yaw])

    if return_state:
        return WheelLoadStiffnessResult(
            loads=loads,
            theta=theta,
            heave=heave,
            roll_angle=roll_angle,
            pitch_angle=pitch_angle,
            qh=heave,
            qp=pitch_angle,
            qr=roll_angle,
        )

    return loads

