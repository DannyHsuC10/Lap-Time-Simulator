import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import PchipInterpolator
import pandas as pd

from scipy.interpolate import CubicSpline
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent

class CircularTrack:


    def __init__(
        self,
        radius=30,
        num_points=1000,
        track_width=3.0
    ):

        self.radius = radius
        self.track_width = track_width


        theta = np.linspace(
            -np.pi/2,
            3*np.pi/2,
            num_points
        )


        cx = 0
        cy = radius


        self.x = cx + radius*np.cos(theta)

        self.y = cy + radius*np.sin(theta)



        # ======================
        # Heading
        # ======================

        dx = np.gradient(self.x)
        dy = np.gradient(self.y)


        self.psi = np.arctan2(
            dy,
            dx
        )


        # ======================
        # Arc length
        # ======================

        ds = np.sqrt(
            dx**2 + dy**2
        )

        self.s = np.cumsum(ds)



        # ======================
        # Track Boundary
        # ======================

        # tangent vector normalize

        tangent_norm = np.sqrt(
            dx**2 + dy**2
        )

        tx = dx / tangent_norm
        ty = dy / tangent_norm


        # 左側法向量
        nx = -ty
        ny = tx



        offset = track_width / 2



        # inner boundary
        self.inner_x = (
            self.x 
            + nx * offset
        )

        self.inner_y = (
            self.y 
            + ny * offset
        )


        # outer boundary
        self.outer_x = (
            self.x 
            - nx * offset
        )

        self.outer_y = (
            self.y 
            - ny * offset
        )



    def nearest_point(self,x,y):

        dist = (
            (self.x-x)**2
            +
            (self.y-y)**2
        )

        idx=np.argmin(dist)

        return idx



    def plot(self):
        """
        Plot track geometry
        """

        plt.figure(figsize=(8, 8))

        # Track boundaries
        plt.plot(
            self.inner_x,
            self.inner_y,
            'k',
            linewidth=2,
            label="Inner Boundary"
        )

        plt.plot(
            self.outer_x,
            self.outer_y,
            'k',
            linewidth=2,
            label="Outer Boundary"
        )

        # Center line
        plt.plot(
            self.x,
            self.y,
            '--b',
            linewidth=1.5,
            label="Center Line"
        )

        # Start point
        plt.scatter(
            self.x[0],
            self.y[0],
            color='red',
            s=80,
            zorder=5,
            label="Start"
        )

        # Direction arrow
        plt.arrow(
            self.x[0],
            self.y[0],
            self.x[5] - self.x[0],
            self.y[5] - self.y[0],
            head_width=0.6,
            head_length=1.0,
            color='red'
        )

        plt.axis("equal")
        plt.grid(True)
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.title("Circular Track")
        plt.legend()

        plt.show()


class Track:


    def __init__(
        self,
        csv_file=None,
        x=None,
        y=None,
        track_width=3.0,
        resample_distance=0.1,
        closed=True
    ):

        self.track_width = track_width
        self.closed = closed


        # ==========================
        # Load data
        # ==========================

        if csv_file is not None:

            self.load_csv(csv_file)

        elif x is not None and y is not None:

            self.x = np.asarray(x)
            self.y = np.asarray(y)

        else:

            raise ValueError(
                "Need csv_file or x,y data"
            )


        # ==========================
        # Resample
        # ==========================

        if resample_distance is not None:

            self.resample(
                resample_distance
            )


        # ==========================
        # Track properties
        # ==========================

        self.compute_heading()

        self.compute_arc_length()

        self.compute_curvature()

        self.compute_boundary()


        x0, y0, psi0 = self.compute_start_pose()

        print("起始點 : ",x0,y0)
        print("起始方向 : ",np.rad2deg(psi0))


    # =====================================================
    # CSV Loader
    # =====================================================

    def load_csv(
        self,
        filename
    ):

        data = pd.read_csv(
            filename
        )


        if (
            "X" not in data.columns
            or
            "Y" not in data.columns
        ):

            raise ValueError(
                "CSV must contain x,y columns"
            )


        self.x = data["X"].to_numpy()

        self.y = data["Y"].to_numpy()



    # =====================================================
    # Resampling
    # =====================================================

    def resample(
        self,
        ds_target=0.1
    ):


        # original arc length

        dx = np.gradient(self.x)
        dy = np.gradient(self.y)

        ds = np.sqrt(
            dx**2 + dy**2
        )


        s = np.concatenate(
            (
                [0],
                np.cumsum(ds[:-1])
            )
        )


        total_length = s[-1]


        s_new = np.arange(
            0,
            total_length,
            ds_target
        )


        # spline interpolation

        cs_x = PchipInterpolator(
            s,
            self.x
        )

        cs_y = PchipInterpolator(
            s,
            self.y
        )


        self.x = cs_x(
            s_new
        )

        self.y = cs_y(
            s_new
        )



    # =====================================================
    # Heading
    # =====================================================

    def compute_heading(
        self
    ):

        dx = np.gradient(
            self.x
        )

        dy = np.gradient(
            self.y
        )


        self.psi = np.arctan2(
            dy,
            dx
        )



    # =====================================================
    # Arc Length
    # =====================================================

    def compute_arc_length(
        self
    ):

        dx = np.gradient(
            self.x
        )

        dy = np.gradient(
            self.y
        )


        ds = np.sqrt(
            dx**2 + dy**2
        )


        self.s = np.concatenate(
            (
                [0],
                np.cumsum(ds[:-1])
            )
        )


        self.length = self.s[-1]



    # =====================================================
    # Curvature
    # =====================================================

    def compute_curvature(
        self
    ):


        dx = np.gradient(
            self.x
        )

        dy = np.gradient(
            self.y
        )


        ddx = np.gradient(
            dx
        )

        ddy = np.gradient(
            dy
        )


        denominator = (
            dx**2 + dy**2
        )**1.5


        self.curvature = (
            dx * ddy
            -
            dy * ddx
        ) / denominator



    # =====================================================
    # Boundary
    # =====================================================

    def compute_boundary(
        self
    ):


        dx = np.gradient(
            self.x
        )

        dy = np.gradient(
            self.y
        )


        norm = np.sqrt(
            dx**2 + dy**2
        )


        tx = dx / norm

        ty = dy / norm


        # left normal

        nx = -ty

        ny = tx



        offset = (
            self.track_width / 2
        )


        self.inner_x = (
            self.x
            +
            nx * offset
        )

        self.inner_y = (
            self.y
            +
            ny * offset
        )



        self.outer_x = (
            self.x
            -
            nx * offset
        )

        self.outer_y = (
            self.y
            -
            ny * offset
        )



    # =====================================================
    # Find nearest point
    # =====================================================

    def nearest_point(
        self,
        x,
        y
    ):


        dist = (
            (self.x-x)**2
            +
            (self.y-y)**2
        )


        idx = np.argmin(
            dist
        )


        return idx



    # =====================================================
    # Lookahead point
    # =====================================================

    def lookahead_point(
        self,
        x,
        y,
        distance=5.0
    ):


        idx = self.nearest_point(
            x,
            y
        )


        target_s = (
            self.s[idx]
            +
            distance
        )


        if self.closed:

            target_s %= self.length


        else:

            target_s = min(
                target_s,
                self.length
            )


        target_idx = np.argmin(
            np.abs(
                self.s-target_s
            )
        )


        return target_idx



    # =====================================================
    # Projection
    # =====================================================

    def project(
        self,
        x,
        y
    ):

        idx = self.nearest_point(
            x,
            y
        )


        dx = (
            x-self.x[idx]
        )

        dy = (
            y-self.y[idx]
        )


        e_y = (
            -np.sin(self.psi[idx])*dx
            +
            np.cos(self.psi[idx])*dy
        )


        return idx, e_y


    def compute_start_pose(self):
        """
        Calculate start position and heading
        """

        self.start_x = self.x[0]
        self.start_y = self.y[0]

        self.start_heading = self.psi[0]


        return self.start_x,self.start_y,self.start_heading


    # =====================================================
    # Plot
    # =====================================================

    def plot(
        self
    ):


        plt.figure(
            figsize=(8,8)
        )


        plt.plot(
            self.inner_x,
            self.inner_y,
            "k",
            linewidth=2,
            label="Inner"
        )


        plt.plot(
            self.outer_x,
            self.outer_y,
            "k",
            linewidth=2,
            label="Outer"
        )


        plt.plot(
            self.x,
            self.y,
            "--b",
            linewidth=1.5,
            label="Ideal Racing Line"
        )


        plt.scatter(
            self.x[0],
            self.y[0],
            color="red",
            s=80,
            label="Start"
        )


        plt.arrow(
            self.x[0],
            self.y[0],
            self.x[1]-self.x[0],
            self.y[1]-self.y[0],
            head_width=0.5,
            color="red"
        )


        plt.axis(
            "equal"
        )

        plt.grid(
            True
        )

        plt.xlabel(
            "X [m]"
        )

        plt.ylabel(
            "Y [m]"
        )

        plt.title(
            "Ideal Racing Line Track"
        )

        plt.legend()

        plt.show()



if __name__ == "__main__":


    track = Track(
        csv_file=PACKAGE_DIR / "fsae_A_2025_xy.csv",
        track_width=3.0,
        resample_distance=1
    )


    print(
        "Track length:",
        track.length,
        "m"
    )


    track.plot()


    track = CircularTrack(
        radius=18.25/2,
        track_width=3
    )

    track.plot()