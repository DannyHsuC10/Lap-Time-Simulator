import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# 參數設定=====================================

DEFAULT_FRONTWHEEL_TRACK = 1.25 # m
DEFAULT_REARWHEEL_TRACK = 1.25 # m

DEFAULT_FRONTWHEEL_AXLE = 1.53/2 # 重心與前軸距離 (m)
DEFAULT_REARWHEEL_AXLE = -1.53/2 # 重心與後軸距離 (m)
# ==================================================
# Read Excel
# ==================================================
def read_mass_file(file_path):
    """
    讀取質量分布檔案

    Parameters
    ----------
    file_path : str or Path

    Returns
    -------
    df : DataFrame
    total_mass : float
    """

    df = pd.read_excel(file_path, skiprows=[1])
    df.columns = df.columns.str.strip()

    # convert numeric columns to numeric dtype
    for col in ["weight LP03", "dx", "dy", "dz", "dy/y"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')

    total_mass = df["weight LP03"].sum()

    return df, total_mass

# ==================================================
# First Moment
# ==================================================
def calc_first_moment(
    sprung_mass_df,
    unsprung_front_df=None,
    unsprung_rear_df=None,
    frontwheel_track=DEFAULT_FRONTWHEEL_TRACK,
    rearwheel_track=DEFAULT_REARWHEEL_TRACK,
    frontwheel_axle=DEFAULT_FRONTWHEEL_AXLE,
    rearwheel_axle=DEFAULT_REARWHEEL_AXLE,
):
    """
    計算一次慣性矩(質量矩)

    Parameters
    ----------
    sprung_mass_df : DataFrame
        Sprung mass data
    unsprung_front_df : DataFrame, optional
        Front unsprung mass data
    unsprung_rear_df : DataFrame, optional
        Rear unsprung mass data

    Returns
    -------
    result : dict
    """

    mx_parts = [sprung_mass_df["weight LP03"] * sprung_mass_df["dx"]]
    my_parts = [sprung_mass_df["weight LP03"] * sprung_mass_df["dy"]]
    mz_parts = [sprung_mass_df["weight LP03"] * sprung_mass_df["dz"]]

    if unsprung_front_df is not None:
        mx_parts.append(unsprung_front_df["weight LP03"] * frontwheel_axle)
        my_parts.append(unsprung_front_df["weight LP03"] * unsprung_front_df["dy/y"] * frontwheel_track)
        mz_parts.append(unsprung_front_df["weight LP03"] * unsprung_front_df["dz"])

    if unsprung_rear_df is not None:
        mx_parts.append(unsprung_rear_df["weight LP03"] * rearwheel_axle)
        my_parts.append(unsprung_rear_df["weight LP03"] * unsprung_rear_df["dy/y"] * rearwheel_track)
        mz_parts.append(unsprung_rear_df["weight LP03"] * unsprung_rear_df["dz"])

    mx = pd.concat(mx_parts, ignore_index=True)
    my = pd.concat(my_parts, ignore_index=True)
    mz = pd.concat(mz_parts, ignore_index=True)

    result = {
        "Mx": mx,
        "My": my,
        "Mz": mz}

    return result

# ==================================================
# Second Moment
# ==================================================
def normalize_unsprung_dy(unsprung_mass_df, track_width):
    df = unsprung_mass_df.copy()
    df["dy"] = df["dy/y"] * track_width
    return df


def combine_mass_data(
    sprung_mass_df,
    unsprung_front_df=None,
    unsprung_rear_df=None,
    frontwheel_track=DEFAULT_FRONTWHEEL_TRACK,
    rearwheel_track=DEFAULT_REARWHEEL_TRACK,
):
    frames = [sprung_mass_df.copy()]
    if unsprung_front_df is not None:
        frames.append(normalize_unsprung_dy(unsprung_front_df, frontwheel_track))
    if unsprung_rear_df is not None:
        frames.append(normalize_unsprung_dy(unsprung_rear_df, rearwheel_track))
    return pd.concat(frames, ignore_index=True)


def calc_second_moment(
    sprung_mass_df,
    unsprung_front_df=None,
    unsprung_rear_df=None,
    frontwheel_track=DEFAULT_FRONTWHEEL_TRACK,
    rearwheel_track=DEFAULT_REARWHEEL_TRACK,
):
    """
    計算慣量矩

    Returns
    -------
    result : dict
    """


    df = combine_mass_data(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
    )

    Ixx = (df["weight LP03"] * (df["dy"]**2 + df["dz"]**2))
    Iyy = (df["weight LP03"] * (df["dx"]**2 + df["dz"]**2))
    Izz = (df["weight LP03"] * (df["dx"]**2 + df["dy"]**2))

    Ixx_all = Ixx.sum()
    Iyy_all = Iyy.sum()
    Izz_all = Izz.sum()

    result = {
        "Ixx_all": Ixx_all,
        "Iyy_all": Iyy_all,
        "Izz_all": Izz_all,

        "Ixx": Ixx,
        "Iyy": Iyy,
        "Izz": Izz,
    }

    return result

# ==================================================
# 3D View
# ==================================================
def plot_3d_distribution(
    sprung_mass_df,
    unsprung_front_df=None,
    unsprung_rear_df=None,
    size=20,
    frontwheel_track=DEFAULT_FRONTWHEEL_TRACK,
    rearwheel_track=DEFAULT_REARWHEEL_TRACK,
):

    save_dir = Path(__file__).parent
    df = combine_mass_data(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
    )
    total_mass = df["weight LP03"].sum()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    scatter = ax.scatter(
        df["dx"],
        df["dy"],
        df["dz"],
        s=df["weight LP03"] * size,
        c=df["weight LP03"],
        cmap='viridis'
    )

    for _, row in df.iterrows():
        ax.text(
            row["dx"],
            row["dy"],
            row["dz"],
            row["name"],
            fontsize=7
        )

    ax.scatter(
        0,
        0,
        0,
        marker='x',
        s=200,
        label='CG'
    )

    ax.set_title(f"Mass Distribution 3D ({total_mass :.2f}kg)")
    ax.set_xlabel("dx (m)")
    ax.set_ylabel("dy (m)")
    ax.set_zlabel("dz (m)")

    ax.legend()

    cbar = plt.colorbar(scatter)
    cbar.set_label("Mass (kg)")

    plt.tight_layout()

    save_path = save_dir/"Mass_Distribution_3D.png"
    plt.savefig(save_path, dpi=300)

    plt.show()

# ==================================================
# Top View second_moment (XY)
# ==================================================
def plot_top_view(
    sprung_mass_df,
    unsprung_front_df=None,
    unsprung_rear_df=None,
    size=80,
    frontwheel_track=DEFAULT_FRONTWHEEL_TRACK,
    rearwheel_track=DEFAULT_REARWHEEL_TRACK,
):# 轉動慣輛評估

    save_dir = Path(__file__).parent
    df = combine_mass_data(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
    )
    second = calc_second_moment(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
    )

    fig, ax = plt.subplots(figsize=(10, 8))

    scatter = ax.scatter(
        df["dx"],
        df["dy"],
        s=abs(second["Izz"] * size),
        c=abs(second["Izz"]),
        cmap='viridis'
    )

    for _, row in df.iterrows():
        ax.text(
            row["dx"],
            row["dy"],
            row["name"],
            fontsize=7
        )

    ax.scatter(
        0,
        0,
        marker='x',
        s=200,
        label='CG'
    )

    ax.set_title(f"second_moment Distribution Top View (XY)\n I(xx,yy,zz) = {second['Ixx_all']:.1f},{second['Iyy_all']:.1f},{second['Izz_all']:.1f}")
    ax.set_xlabel("dx (m)")
    ax.set_ylabel("dy (m)")

    ax.grid(True)
    ax.axis('equal')

    cbar = plt.colorbar(scatter)
    cbar.set_label("second_moment (kg·m²)")

    ax.legend()

    plt.tight_layout()

    save_path = save_dir/"second_moment_Distribution_XY.png"
    plt.savefig(save_path, dpi=300)

    plt.show()

# ==================================================
# Side View first_moment (XZ)
# ==================================================
def plot_side_view(
    sprung_mass_df,
    unsprung_front_df=None,
    unsprung_rear_df=None,
    size=120,
    frontwheel_track=DEFAULT_FRONTWHEEL_TRACK,
    rearwheel_track=DEFAULT_REARWHEEL_TRACK,
    frontwheel_axle=DEFAULT_FRONTWHEEL_AXLE,
    rearwheel_axle=DEFAULT_REARWHEEL_AXLE,
):# 用於重心評估

    save_dir = Path(__file__).parent
    df = combine_mass_data(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
    )
    first = calc_first_moment(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
        frontwheel_axle=frontwheel_axle,
        rearwheel_axle=rearwheel_axle,
    )

    fig, ax = plt.subplots(figsize=(10, 8))

    scatter = ax.scatter(
        df["dx"],
        df["dz"],
        s=abs(first["Mz"] * size),
        c=abs(first["Mz"]),
        cmap='viridis')

    for _, row in df.iterrows():
        ax.text(
            row["dx"],
            row["dz"],
            row["name"],
            fontsize=7)

    ax.scatter(
        0,
        0,
        marker='x',
        s=200,
        label='CG')

    ax.set_title("first_moment Distribution Side View (XZ)")
    ax.set_xlabel("mdx (m)")
    ax.set_ylabel("mdz (m)")

    ax.grid(True)
    ax.axis('equal')

    cbar = plt.colorbar(scatter)
    cbar.set_label("first_moment (Nm)")

    ax.legend()

    plt.tight_layout()

    save_path = save_dir/"first_moment_Distribution_XZ.png"
    plt.savefig(save_path, dpi=300)

    plt.show()

# ==================================================
# Public API
# ==================================================

def load_vehicle_mass_data(data_dir=None):
    current_dir = Path(__file__).parent if data_dir is None else Path(data_dir)

    sprung_mass_df, sprung_mass = read_mass_file(current_dir / "Center_of_gravity_LP03_sprung.xlsx")
    unsprung_front_df, unsprung_front_mass = read_mass_file(current_dir / "Center_of_gravity_LP03_unsprungfront.xlsx")
    unsprung_rear_df, unsprung_rear_mass = read_mass_file(current_dir / "Center_of_gravity_LP03_unsprungrear.xlsx")

    return {
        "sprung_mass_df": sprung_mass_df,
        "unsprung_front_df": unsprung_front_df,
        "unsprung_rear_df": unsprung_rear_df,
        "sprung_mass": sprung_mass,
        "unsprung_front_mass": unsprung_front_mass,
        "unsprung_rear_mass": unsprung_rear_mass,
        "total_mass": sprung_mass + unsprung_front_mass + unsprung_rear_mass,
    }


def calculate_vehicle_inertia(
    frontwheel_track=DEFAULT_FRONTWHEEL_TRACK,
    rearwheel_track=DEFAULT_REARWHEEL_TRACK,
    frontwheel_axle=DEFAULT_FRONTWHEEL_AXLE,
    rearwheel_axle=DEFAULT_REARWHEEL_AXLE,
    data_dir=None,
):
    mass_data = load_vehicle_mass_data(data_dir=data_dir)
    sprung_mass_df = mass_data["sprung_mass_df"]
    unsprung_front_df = mass_data["unsprung_front_df"]
    unsprung_rear_df = mass_data["unsprung_rear_df"]

    first = calc_first_moment(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
        frontwheel_axle=frontwheel_axle,
        rearwheel_axle=rearwheel_axle,
    )
    second = calc_second_moment(
        sprung_mass_df,
        unsprung_front_df,
        unsprung_rear_df,
        frontwheel_track=frontwheel_track,
        rearwheel_track=rearwheel_track,
    )

    return {
        **mass_data,
        "first_moment": first,
        "second_moment": second,
        "Ixx": second["Ixx_all"],
        "Iyy": second["Iyy_all"],
        "Izz": second["Izz_all"],
    }


def main():
    mass_data = load_vehicle_mass_data()
    sprung_mass_df = mass_data["sprung_mass_df"]
    unsprung_front_df = mass_data["unsprung_front_df"]
    unsprung_rear_df = mass_data["unsprung_rear_df"]

    print("sprung_mass", mass_data["sprung_mass"])
    print("unsprung_front_mass", mass_data["unsprung_front_mass"])
    print("unsprung_rear_mass", mass_data["unsprung_rear_mass"])
    print("total_mass", mass_data["total_mass"])

    result = calculate_vehicle_inertia()
    second = result["second_moment"]
    print("I(xx,yy,zz)", second["Ixx_all"], second["Iyy_all"], second["Izz_all"])
    plot_3d_distribution(sprung_mass_df, unsprung_front_df, unsprung_rear_df)
    plot_top_view(sprung_mass_df, unsprung_front_df, unsprung_rear_df)
    plot_side_view(sprung_mass_df, unsprung_front_df, unsprung_rear_df)


if __name__ == "__main__":
    main()
