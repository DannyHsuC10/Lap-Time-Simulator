"""Responsive Tk plots for the speed map and distance histories."""

import tkinter as tk

import numpy as np

from .planner import GRAVITY


BG = "#0f172a"
GRID = "#27364b"
TEXT = "#a9b9cf"
COLORS = ("#38bdf8", "#fbbf24", "#a78bfa", "#34d399")
SPEED_STOPS = np.array([(96, 99, 246), (34, 211, 238), (52, 211, 153), (250, 204, 21), (251, 113, 90)])
FONT = "Microsoft JhengHei UI"


def speed_color(fraction):
    index = min(int(fraction * (len(SPEED_STOPS) - 1)), len(SPEED_STOPS) - 2)
    mix = fraction * (len(SPEED_STOPS) - 1) - index
    rgb = np.rint(SPEED_STOPS[index] * (1 - mix) + SPEED_STOPS[index + 1] * mix)
    return "#{:02x}{:02x}{:02x}".format(*map(int, rgb))


SPEED_COLORS = tuple(speed_color(index / 63) for index in range(64))


class PlotCanvas(tk.Canvas):
    def __init__(self, master):
        super().__init__(master, bg=BG, highlightthickness=0, width=640, height=440)
        self.bind("<Configure>", self.draw)
        self.bind("<Map>", self.draw)

    def label(self, x, y, value, *, color=TEXT, size=10, anchor="center", bold=False):
        self.create_text(
            x, y, text=value, fill=color, anchor=anchor,
            font=(FONT, size, "bold" if bold else "normal"),
        )

    def draw(self, _event=None):
        self.delete("all")
        w, h = self.winfo_width(), self.winfo_height()
        if w < 250 or h < 170:
            return
        self.draw_content(w, h)

    def draw_lines(self, w, h, title, unit, xlabel, x_values, series):
        left, right, top, bottom = 68, w - 28, 79, h - 55
        start, end = float(x_values[0]), float(x_values[-1])
        span = end - start
        low = min(float(values.min()) for _, values in series)
        high = max(float(values.max()) for _, values in series)
        padding = 0.08 * (high - low) if high > low else max(0.05 * abs(high), 0.5)
        low, high = low - padding, high + padding

        self.label(23, 23, f"{title}  [{unit}]", size=14, bold=True, anchor="w", color="#e2e8f0")
        for index in range(6):
            f = index / 5
            x, y = left + f * (right - left), bottom - f * (bottom - top)
            self.create_line(x, top, x, bottom, fill=GRID)
            self.create_line(left, y, right, y, fill=GRID)
            self.label(x, bottom + 17, f"{start + f * span:.4g}")
            self.label(left - 9, y, f"{low + f * (high - low):.4g}", anchor="e")
        if low < 0 < high:
            zero = bottom - (0 - low) * (bottom - top) / (high - low)
            self.create_line(left, zero, right, zero, fill="#64748b")
        self.label((left + right) / 2, h - 15, xlabel)

        xs = left + (x_values - start) * (right - left) / span
        for index, ((name, values), color) in enumerate(zip(series, COLORS)):
            legend_x = left + index * min(98, (right - left) / len(series))
            self.create_line(legend_x, 52, legend_x + 16, 52, width=3, fill=color)
            self.label(legend_x + 23, 52, name, anchor="w", color=color)
            ys = bottom - (values - low) * (bottom - top) / (high - low)
            self.create_line(*np.column_stack((xs, ys)).ravel().tolist(), fill=color, width=2)
        return left, right, top, bottom


class LineChart(PlotCanvas):
    def __init__(self, master):
        super().__init__(master)
        self.lines = None

    def set_lines(self, *lines):
        self.lines = lines
        if self.winfo_ismapped():
            self.draw()

    def draw_content(self, w, h):
        if self.lines:
            self.draw_lines(w, h, *self.lines)
        else:
            self.label(w / 2, h / 2, "按「更新曲線」繪圖", color="#e2e8f0", size=15)


class SpeedMap(PlotCanvas):
    def __init__(self, master, *, seek=None):
        super().__init__(master)
        self.result = None
        self.seek = seek
        self.sample = 0
        self.map_points = None
        if seek is not None:
            self.bind("<Button-1>", self.on_click)
            self.bind("<B1-Motion>", self.on_click)

    def set_result(self, result):
        self.result = result
        self.map_points = None
        if self.winfo_ismapped():
            self.draw()

    def draw_content(self, w, h):
        if self.result is None:
            self.label(w / 2, h / 2, "完成圈速計算後顯示賽道彩虹圖", color="#e2e8f0", size=14)
            return
        self.draw_map(w, h)

    def draw_map(self, w, h):
        track, plan = self.result.track, self.result.plan
        all_x = np.concatenate((track.x, track.inner_x, track.outer_x))
        all_y = np.concatenate((track.y, track.inner_y, track.outer_y))
        xmin, xmax = float(all_x.min()), float(all_x.max())
        ymin, ymax = float(all_y.min()), float(all_y.max())
        left, right, top, bottom = 34, w - 34, 59, h - 71
        scale = min((right - left) / max(xmax - xmin, 1), (bottom - top) / max(ymax - ymin, 1))
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        px, py = (left + right) / 2, (top + bottom) / 2

        def points(x, y):
            return np.column_stack((px + (x - cx) * scale, py - (y - cy) * scale))

        self.label(23, 23, "賽道速度分布", size=14, bold=True, anchor="w", color="#e2e8f0")
        self.label(w - 23, 23, "●  起點", anchor="e", color="#e2e8f0")
        for x, y in ((track.inner_x, track.inner_y), (track.outer_x, track.outer_y)):
            self.create_line(*points(x, y).ravel().tolist(), fill="#526480", width=1)
        line = points(track.x, track.y)
        speeds = plan.speed * 3.6
        low, high = float(speeds.min()), float(speeds.max())
        fractions = (speeds[:-1] - low) / (high - low) if high > low else np.full(len(line) - 1, 0.5)
        bins = np.clip(np.rint(fractions * 63).astype(int), 0, 63)
        breaks = np.flatnonzero(np.diff(bins)) + 1
        starts = np.r_[0, breaks]
        ends = np.r_[breaks, len(line) - 1]
        for start, end in zip(starts, ends):
            self.create_line(
                *line[start:end + 1].ravel().tolist(),
                fill=SPEED_COLORS[bins[start]], width=3, capstyle="round",
            )
        x, y = line[0]
        self.create_oval(x - 4, y - 4, x + 4, y + 4, fill="#f8fafc", outline=BG, width=1)

        bar_width = min(300, w - 130)
        bar_left = (w - bar_width) / 2
        bar_y = h - 51
        for i, color in enumerate(SPEED_COLORS):
            a = bar_left + i * bar_width / 64
            b = bar_left + (i + 1) * bar_width / 64
            if high == low:
                color = SPEED_COLORS[32]
            self.create_rectangle(a, bar_y, b, bar_y + 8, fill=color, outline=color)
        self.label(bar_left, bar_y + 23, f"{low:.1f}")
        self.label(w - bar_left, bar_y + 23, f"{high:.1f}")
        self.label(w / 2, bar_y + 23, "km/h")
        self.map_points = line
        self.draw_cursor()

    def draw_cursor(self):
        self.delete("sample")
        if self.seek is None or self.map_points is None:
            return
        x, y = self.map_points[self.sample]
        self.create_oval(x - 6, y - 6, x + 6, y + 6, fill="#f8fafc", outline=BG, width=2, tags="sample")

    def on_click(self, event):
        if self.map_points is None:
            return
        distance_sq = (self.map_points[:, 0] - event.x) ** 2 + (self.map_points[:, 1] - event.y) ** 2
        self.seek(int(np.argmin(distance_sq)))


class ResultChart(PlotCanvas):
    def __init__(self, master, kind):
        super().__init__(master)
        self.kind = kind
        self.result = None

    def set_result(self, result):
        self.result = result
        if self.winfo_ismapped():
            self.draw()

    def draw_content(self, w, h):
        if self.result is None:
            self.label(w / 2, h / 2 - 12, "設定參數後按「計算」", color="#e2e8f0", size=17)
            self.label(w / 2, h / 2 + 22, "使用內附 FSAE 賽道與模型資料即可開始")
            return
        title, unit, series = self.history()
        self.draw_lines(w, h, title, unit, "距離  [m]", self.result.track.s, series)

    def history(self):
        plan = self.result.plan
        if self.kind == "speed":
            return "速度", "km/h", (
                ("規劃速度", plan.speed * 3.6),
                ("側向上限", plan.lateral_speed_limit * 3.6),
            )
        if self.kind == "accel":
            return "加速度", "g", (
                ("縱向", plan.acceleration / GRAVITY),
                ("橫向", self.result.lateral_acceleration / GRAVITY),
            )
        if self.kind == "loads":
            return "四輪垂直輪荷", "N", tuple(zip(("FL", "FR", "RL", "RR"), plan.wheel_loads.T))
        if self.kind == "attitude":
            return "車身姿態", "deg", (
                ("側傾 Roll", np.degrees(plan.theta[:, 0])),
                ("俯仰 Pitch", np.degrees(plan.theta[:, 2])),
            )
        return "空氣力", "N", (("下壓力", plan.downforce), ("阻力", plan.drag))

