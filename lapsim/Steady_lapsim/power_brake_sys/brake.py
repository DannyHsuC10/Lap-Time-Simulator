import numpy as np


def brake(v, car):
    """Return per-wheel brake torques in FL, FR, RL, RR order."""
    front_ratio, rear_ratio = car.brake_bias
    axle_torque = (
        car.F_diver
        * car.brake_mechanical_advantage
        * car.free_radius
        * np.sign(v)
    )
    front_wheel_torque = axle_torque * front_ratio / 2
    rear_wheel_torque = axle_torque * rear_ratio / 2
    return (
        -front_wheel_torque,
        -front_wheel_torque,
        -rear_wheel_torque,
        -rear_wheel_torque,
    )
