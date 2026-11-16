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


def hydraulic_brake(v, car):
    """Convert pedal force through the balance bar, hydraulics and discs."""
    settings = car.settings
    diameters = np.array([settings.front_caliper_diameter, settings.rear_caliper_diameter])
    hydraulic_ratio = (diameters / settings.master_diameter) ** 2
    wheel_torque_by_axle = (
        settings.pedal_force * settings.pedal_ratio * car.brake_bias
        * hydraulic_ratio * np.array([settings.front_pistons, settings.rear_pistons])
        * settings.pad_mu * (settings.disc_diameter / 2) * np.sign(v)
    )
    # Piston counts describe one caliper on each side of the axle.
    return -np.repeat(wheel_torque_by_axle, 2)
