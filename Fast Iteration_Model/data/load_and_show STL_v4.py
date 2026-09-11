import numpy as np
import pyvista as pv
import vtk
from pathlib import Path

from Simulation_Logger import SimulationLogger


scale = 1000.0
camera_distance_scale = 0.4
Attitude_gain = 5
force_arrow_scale = 1
force_arrow_tip_length = 0.25
force_arrow_tip_radius = 0.08
force_arrow_shaft_radius = 0.025

def require_key(result, key):
    if key not in result:
        available = "\n".join(sorted(result.keys()))
        raise KeyError(f"Missing result key: {key}\n\nAvailable keys:\n{available}")
    return np.asarray(result[key])


def optional_key(result, key, default):
    if key not in result:
        return np.asarray(default)
    return np.asarray(result[key])


def angle_to_degrees(angle):
    angle = np.asarray(angle)

    if angle.size == 0:
        return angle

    if np.nanmax(np.abs(angle)) <= 2.0 * np.pi:
        return np.degrees(angle)

    return angle


def rotate_xy(x, y, yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)

    return (
        c * x - s * y,
        s * x + c * y,
    )


def make_polyline(points):
    polyline = pv.PolyData(points)
    cells = np.hstack((
        [points.shape[0]],
        np.arange(points.shape[0])
    ))
    polyline.lines = cells

    return polyline


def tire_force_to_body(Fx, Fy, steer_deg):
    steer_rad = np.radians(steer_deg)

    Fx_body = Fx * np.cos(steer_rad) - Fy * np.sin(steer_rad)
    Fy_body = Fx * np.sin(steer_rad) + Fy * np.cos(steer_rad)

    return Fx_body, Fy_body


def tire_force_components_to_world(Fx, Fy, Fz, steer_deg, yaw):
    heading = yaw + np.radians(steer_deg)

    Fx_component = np.array([
        Fx * np.cos(heading),
        Fx * np.sin(heading),
        0.0
    ])

    Fy_component = np.array([
        -Fy * np.sin(heading),
        Fy * np.cos(heading),
        0.0
    ])

    Fz_component = np.array([
        0.0,
        0.0,
        Fz
    ])

    return Fx_component, Fy_component, Fz_component


def make_force_arrow(start, direction, color):
    arrow = pv.Arrow(
        start=np.zeros(3),
        direction=np.array([1.0, 0.0, 0.0]),
        scale=1.0,
        tip_length=force_arrow_tip_length,
        tip_radius=force_arrow_tip_radius,
        shaft_radius=force_arrow_shaft_radius
    )

    actor = plotter.add_mesh(
        arrow,
        color=color,
        smooth_shading=True
    )

    update_force_arrow(actor, start, direction)

    return actor


def update_force_arrow(actor, start, direction):
    direction = np.asarray(direction, dtype=float)
    start = np.asarray(start, dtype=float)
    length = float(np.linalg.norm(direction))

    if length < 1e-9:
        actor.SetVisibility(False)
        return

    actor.SetVisibility(True)

    x_axis = direction / length
    reference = np.array([0.0, 0.0, 1.0])

    if abs(float(np.dot(x_axis, reference))) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])

    y_axis = np.cross(reference, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)

    matrix = vtk.vtkMatrix4x4()

    basis = np.column_stack((
        x_axis * length,
        y_axis * length,
        z_axis * length
    ))

    for row in range(3):
        for col in range(3):
            matrix.SetElement(row, col, float(basis[row, col]))

        matrix.SetElement(row, 3, float(start[row]))

    matrix.SetElement(3, 0, 0.0)
    matrix.SetElement(3, 1, 0.0)
    matrix.SetElement(3, 2, 0.0)
    matrix.SetElement(3, 3, 1.0)

    actor.SetUserMatrix(matrix)


# =====================================================
# Load simulation result
# =====================================================

current_dir = Path(__file__).resolve().parent
result_file = current_dir / "data.pkl"

logger = SimulationLogger.load(result_file)
result = logger.result()

print("Loaded result:")
for key, value in result.items():
    value = np.asarray(value)
    print(f"{key}: shape={value.shape}")


# =====================================================
# Extract vehicle motion
# =====================================================

time_data = require_key(result, "t")

s_body = require_key(result, "s")
s_world = optional_key(result, "s_world", s_body)

x_world = s_world[:, 0]
y_world = s_world[:, 1]
z_body = s_body[:, 2]

x_origin = x_world[0]
y_origin = y_world[0]

x_path = x_world - x_origin
y_path = y_world - y_origin

theta = require_key(result, "theta")

roll = (
    theta[:, 0]
    + theta[:, 1]
) / 2.0

pitch = theta[:, 2]
yaw = theta[:, 3]


# =====================================================
# Wheel motion
# =====================================================

zu = require_key(result, "zu")
IA = optional_key(result, "IA", np.zeros_like(zu))
steer = optional_key(result, "steer", np.zeros_like(zu))
Fx_data = require_key(result, "Fx")
Fy_data = require_key(result, "Fy")
Fz_data = optional_key(result, "N", optional_key(result, "Fz", np.zeros_like(Fx_data)))

IA_deg = IA
steer_deg = steer


# =====================================================
# Wheel static position
# =====================================================

wheel_base_position = {
    "FL": (0.45,  1.25 / 2.0, 0.259),
    "FR": (0.45, -1.25 / 2.0, 0.259),
    "RL": (-1.1,  1.25 / 2.0, 0.259),
    "RR": (-1.1, -1.25 / 2.0, 0.259),
}

wheel_id = {
    "FL": 0,
    "FR": 1,
    "RL": 2,
    "RR": 3,
}


# =====================================================
# Load STL
# =====================================================

stl_dir = current_dir / "STL"

chassis_mesh = pv.read(
    stl_dir / "LP02.stl"
)

wheel_mesh = {
    name: pv.read(stl_dir / f"W_{name}.stl")
    for name in wheel_id.keys()
}


# =====================================================
# Create Plotter
# =====================================================

plotter = pv.Plotter(
    window_size=(1280, 720)
)

car = plotter.add_mesh(
    chassis_mesh,
    color="lightgray",
    smooth_shading=True
)

wheels = {}
force_arrows = {}

for name, mesh in wheel_mesh.items():
    wheels[name] = plotter.add_mesh(
        mesh,
        color="black",
        smooth_shading=True
    )

    force_arrows[name] = {
        "Fx": make_force_arrow(
            np.zeros(3),
            np.array([1e-9, 0.0, 0.0]),
            "red"
        ),
        "Fy": make_force_arrow(
            np.zeros(3),
            np.array([0.0, 1e-9, 0.0]),
            "green"
        ),
        "Fz": make_force_arrow(
            np.zeros(3),
            np.array([0.0, 0.0, 1e-9]),
            "blue"
        ),
        "F": make_force_arrow(
            np.zeros(3),
            np.array([1e-9, 1e-9, 1e-9]),
            "orange"
        )
    }

path_points = np.column_stack((
    x_path * scale,
    y_path * scale,
    np.zeros_like(x_path)
))

path_line = make_polyline(path_points)

plotter.add_mesh(
    path_line,
    color="dodgerblue",
    line_width=4,
    render_lines_as_tubes=True
)

plotter.add_mesh(
    pv.Sphere(radius=120.0, center=path_points[0]),
    color="limegreen"
)

plotter.add_mesh(
    pv.Sphere(radius=120.0, center=path_points[-1]),
    color="crimson"
)

plotter.add_axes()
plotter.set_background("white")
plotter.camera_position = "iso"


# =====================================================
# Animation
# =====================================================

plotter.show(
    interactive_update=True
)

frame = 0
camera_offset = None

while True:

    if frame >= len(time_data):
        frame = 0

    if frame % 50 == 0:
        print(
            "frame:", frame,
            "world position:",
            f"{x_world[frame]:.3f}",
            f"{y_world[frame]:.3f}",
            "zu:",
            f"{zu[frame, 0]:.4f}",
            f"{zu[frame, 1]:.4f}",
            f"{zu[frame, 2]:.4f}",
            f"{zu[frame, 3]:.4f}",
        )

    # =================================================
    # Update chassis
    # =================================================

    car.SetPosition(
        x_path[frame] * scale,
        y_path[frame] * scale,
        z_body[frame] * scale
    )

    car.SetOrientation(
        np.degrees(roll[frame])*Attitude_gain,
        -np.degrees(pitch[frame]*Attitude_gain),
        np.degrees(yaw[frame])
    )

    # =================================================
    # Update wheels
    # =================================================
    if frame % 4 == 0:
        for name, idx in wheel_id.items():

            actor = wheels[name]

            base_x, base_y, base_z = wheel_base_position[name]
            wheel_x_body, wheel_y_body = rotate_xy(base_x, base_y, yaw[frame])

            actor.SetPosition(
                (x_path[frame] + wheel_x_body) * scale,
                (y_path[frame] + wheel_y_body) * scale,
                (z_body[frame] + base_z + zu[frame, idx]) * scale
            )

            actor.SetOrientation(
                IA_deg[frame, idx],
                0.0,
                steer_deg[frame, idx] + np.degrees(yaw[frame])
            )

            wheel_position = np.array([
                (x_path[frame] + wheel_x_body) * scale,
                (y_path[frame] + wheel_y_body) * scale,
                (z_body[frame] + base_z + zu[frame, idx]) * scale
            ])

            Fx_body, Fy_body = tire_force_to_body(
                Fx_data[frame, idx],
                Fy_data[frame, idx],
                steer_deg[frame, idx]
            )

            Fx_body_world, Fy_body_world = rotate_xy(
                Fx_body,
                Fy_body,
                yaw[frame]
            )

            Fx_component, Fy_component, Fz_component = tire_force_components_to_world(
                Fx_data[frame, idx],
                Fy_data[frame, idx],
                Fz_data[frame, idx],
                steer_deg[frame, idx],
                yaw[frame]
            )

            F_total = np.array([
                Fx_body_world,
                Fy_body_world,
                Fz_data[frame, idx]
            ])

            update_force_arrow(
                force_arrows[name]["Fx"],
                wheel_position,
                Fx_component * force_arrow_scale
            )
            update_force_arrow(
                force_arrows[name]["Fy"],
                wheel_position,
                Fy_component * force_arrow_scale
            )
            update_force_arrow(
                force_arrows[name]["Fz"],
                wheel_position,
                Fz_component * force_arrow_scale
            )
            update_force_arrow(
                force_arrows[name]["F"],
                wheel_position,
                F_total * force_arrow_scale
            )

    # =================================================
    # Follow vehicle with camera
    # =================================================

        vehicle_position = np.array([
            x_path[frame] * scale,
            y_path[frame] * scale,
            z_body[frame] * scale
        ])

        if camera_offset is None:
            camera_offset = (
                np.array(plotter.camera.position)
                - vehicle_position
            ) * camera_distance_scale

        plotter.camera.focal_point = vehicle_position
        plotter.camera.position = vehicle_position + camera_offset


        plotter.update()

    frame += 1
