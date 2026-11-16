"""Rotatable GGV surface rendered in a Tk canvas."""

from math import cos, radians, sin

import numpy as np

from .charts import GRID, PlotCanvas, speed_color


LAP_POINT_COLOR = "#f054a5"


def draw_operating_points(canvas, screen):
    # Keep one marker per small screen cell so dense laps can still rotate smoothly.
    _, indices = np.unique(np.floor(screen / 3), axis=0, return_index=True)
    for x, y in screen[indices]:
        canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=LAP_POINT_COLOR, outline="")


class GGVSurface(PlotCanvas):
    def __init__(self, master):
        super().__init__(master)
        self.slices = []
        self.points = np.empty((0, 3))
        self.drag_start = None
        self.bind("<ButtonPress-1>", self.start_rotation)
        self.bind("<B1-Motion>", self.rotate)
        self.bind("<Double-Button-1>", self.reset_view)
        self.reset_view()

    def reset_view(self, _event=None):
        self.azimuth = 29.0
        self.elevation = 22.0
        self.drag_start = None
        self.draw()

    def start_rotation(self, event):
        self.drag_start = (event.x, event.y)

    def rotate(self, event):
        if not self.slices or self.drag_start is None:
            return
        x, y = self.drag_start
        self.drag_start = (event.x, event.y)
        self.azimuth = (self.azimuth + (event.x - x) * 0.55) % 360
        self.elevation = float(np.clip(self.elevation + (event.y - y) * 0.45, -75, 75))
        self.draw()

    def draw_content(self, w, h):
        if not self.slices:
            self.label(w / 2, h / 2, "按「計算 GGV」查看車輛加速度上限", size=14, color="#e2e8f0")
            return

        ay_extent = max(0.1, max(float(np.abs(part.ay_g).max()) for part in self.slices))
        ax_extent = max(0.1, max(float(max(part.drive_g.max(), -part.brake_g.min())) for part in self.slices))
        vmax = self.slices[-1].speed_kmh
        if self.points.size:
            ay_extent = max(ay_extent, np.abs(self.points[:, 0]).max())
            ax_extent = max(ax_extent, np.abs(self.points[:, 1]).max())
            vmax = max(vmax, self.points[:, 2].max())
        ay_extent *= 1.06
        ax_extent *= 1.06
        azimuth, elevation = radians(self.azimuth), radians(self.elevation)
        ca, sa = cos(azimuth), sin(azimuth)
        ce, se = cos(elevation), sin(elevation)
        rotation = np.array([[ca, -sa, 0.0], [-sa * se, -ca * se, ce], [sa * ce, ca * ce, se]])
        corners = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        projected_corners = corners @ rotation.T
        spread = np.ptp(projected_corners, axis=0)
        scale = min((w - 88) / spread[0], (h - 110) / spread[1])

        def project(points):
            view = np.asarray(points) @ rotation.T
            screen = view[..., :2] * [scale, -scale] + [w / 2, h / 2 + 4]
            return screen, view[..., 2]

        def line(start, end, **style):
            points, _ = project([start, end])
            self.create_line(*points.ravel(), **style)

        for value in np.linspace(-1, 1, 5):
            line((value, -1, -1), (value, 1, -1), fill=GRID)
            line((-1, value, -1), (1, value, -1), fill=GRID)

        speeds = np.array([part.speed_kmh / vmax for part in self.slices])
        ay = np.array([part.ay_g for part in self.slices]) / ay_extent
        z = np.broadcast_to((2 * speeds - 1)[:, None], ay.shape)
        faces = []
        for branch, outline in (("drive_g", "#1d3552"), ("brake_g", "#422b48")):
            ax = np.array([getattr(part, branch) for part in self.slices]) / ax_extent
            screen, depth = project(np.stack((ay, ax, z), axis=-1))
            for i, fraction in enumerate((speeds[:-1] + speeds[1:]) / 2):
                color = speed_color(float(fraction))
                for j in range(screen.shape[1] - 1):
                    rows, columns = (i, i, i + 1, i + 1), (j, j + 1, j + 1, j)
                    faces.append((depth[rows, columns].mean(), screen[rows, columns].ravel(), color, outline))
        for _, polygon, color, outline in sorted(faces, key=lambda face: face[0]):
            self.create_polygon(*polygon, fill=color, outline=outline, width=1, tags="ggv-face")

        if self.points.size:
            normalized = self.points / [ay_extent, ax_extent, vmax / 2] - [0, 0, 1]
            screen, _ = project(normalized)
            draw_operating_points(self, screen)

        # Place the axes on the edge nearest the viewer for the current angle.
        x_edge = 1 if sa >= 0 else -1
        y_edge = 1 if ca >= 0 else -1
        x_values = np.linspace(-1, 1, 5)
        z_values = np.linspace(-1, 1, 5)
        for start, end, label in (
            ((-1, y_edge, -1), (1, y_edge, -1), "ay [g]"),
            ((x_edge, -1, -1), (x_edge, 1, -1), "ax [g]"),
            ((x_edge, y_edge, -1), (x_edge, y_edge, 1), "車速 [km/h]"),
        ):
            line(start, end, fill="#a9b9cf", width=1)
            point, _ = project(end)
            self.label(point[0], point[1] - (16 if label.startswith("車速") else 14), label, bold=True, color="#f1f5f9")
        for value, z_value in zip(x_values, z_values):
            x_tick, _ = project((value, y_edge, -1))
            y_tick, _ = project((x_edge, value, -1))
            z_tick, _ = project((x_edge, y_edge, z_value))
            if value != x_edge:
                self.label(x_tick[0], x_tick[1] + 11, f"{value * ay_extent:.1f}", size=9)
            if value != y_edge:
                self.label(y_tick[0], y_tick[1] + 11, f"{value * ax_extent:.1f}", size=9)
            self.label(z_tick[0] + 8, z_tick[1], f"{(z_value + 1) * vmax / 2:g}", size=9, anchor="w")

        self.label(18, 18, "GGV 車輛加速度極限", anchor="w", bold=True, color="#e2e8f0")
        bar_left, bar_width = (w - 144) / 2, 144
        for i in range(24):
            color = speed_color(i / 23)
            self.create_rectangle(bar_left + i * 6, h - 26, bar_left + (i + 1) * 6, h - 20, fill=color, outline="")
        self.label(bar_left - 6, h - 23, "0", anchor="e", size=9)
        self.label(bar_left + bar_width + 6, h - 23, f"{vmax:g} km/h", anchor="w", size=9)
