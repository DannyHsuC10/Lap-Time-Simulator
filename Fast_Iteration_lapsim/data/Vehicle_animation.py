import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


class Vehicle2DVisualizer:

    def __init__(
        self,
        result,
        length=2.5,
        width=1.2,
        fps=30,
        margin=5
    ):

        self.result = result

        # ===========================
        # World Frame
        # ===========================
        self.time = result["t"]

        self.pos = result["s_world"]

        self.x = self.pos[:, 0]
        self.y = self.pos[:, 1]

        # yaw (Body attitude)
        self.yaw = result["theta"][:, 3]

        self.length = length
        self.width = width

        self.fps = fps
        self.margin = margin

        # figure
        self.fig, self.ax = plt.subplots(figsize=(8, 8))

        self.vehicle = None
        self.arrow = None
        self.path_line = None


    def setup(self):

        self.ax.set_aspect("equal")

        self.ax.set_xlim(
            np.min(self.x) - self.margin,
            np.max(self.x) + self.margin
        )

        self.ax.set_ylim(
            np.min(self.y) - self.margin,
            np.max(self.y) + self.margin
        )

        self.ax.set_xlabel("World X (m)")
        self.ax.set_ylabel("World Y (m)")
        self.ax.grid(True)

        # trajectory
        self.path_line, = self.ax.plot(
            [],
            [],
            linewidth=2,
            label="Trajectory"
        )

        # vehicle body
        self.vehicle = Rectangle(
            (-self.length/2,
             -self.width/2),
            self.length,
            self.width,
            fill=False,
            linewidth=2
        )

        self.ax.add_patch(self.vehicle)

        # heading arrow
        self.arrow = self.ax.arrow(
            0,
            0,
            1,
            0,
            width=0.05,
            head_width=0.25,
            length_includes_head=True
        )


    def update(self, frame):

        x = self.x[frame]
        y = self.y[frame]
        yaw = self.yaw[frame]

        # -------------------
        # trajectory
        # -------------------

        self.path_line.set_data(
            self.x[:frame+1],
            self.y[:frame+1]
        )

        # -------------------
        # vehicle body
        # -------------------

        transform = (
            Affine2D()
            .rotate(yaw)
            .translate(x, y)
            + self.ax.transData
        )

        self.vehicle.set_transform(transform)

        # -------------------
        # heading
        # -------------------

        if self.arrow is not None:
            self.arrow.remove()

        arrow_length = self.length * 0.6

        self.arrow = self.ax.arrow(
            x,
            y,
            arrow_length * np.cos(yaw),
            arrow_length * np.sin(yaw),
            width=0.05,
            head_width=0.25,
            length_includes_head=True
        )

        self.ax.set_title(
            f"Time = {self.time[frame]:.2f} s"
        )

        return (
            self.vehicle,
            self.path_line,
            self.arrow
        )


    def get_animation_frames(self):

        total_time = self.time[-1] - self.time[0]

        target_frames = max(
            1,
            int(total_time * self.fps)
        )

        step = max(
            1,
            len(self.time) // target_frames
        )

        return range(
            0,
            len(self.time),
            step
        )


    def show(self):

        self.setup()

        ani = FuncAnimation(
            self.fig,
            self.update,
            frames=self.get_animation_frames(),
            interval=1000/self.fps,
            blit=False,
            repeat=True
        )

        plt.show()

        return ani