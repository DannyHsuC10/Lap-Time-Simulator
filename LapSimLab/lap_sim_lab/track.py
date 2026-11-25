"""Sample the LINE/ARC section CSV using the geometry of new/lapsim."""

import csv
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from .localization import localize


MAX_SAMPLES = 25_000
REQUIRED_COLUMNS = {"section", "Type", "Start_X", "Start_Y", "End_X", "End_Y", "length"}


def smooth_curvature(distance: np.ndarray, curvature: np.ndarray, sigma_m: float) -> np.ndarray:
    """Smooth the curvature signal in metres while preserving the route coordinates."""
    grid = np.linspace(0.0, distance[-1], len(distance))
    sigma = sigma_m / (grid[1] - grid[0])
    if sigma < 0.125:
        return curvature.copy()
    radius = min(math.ceil(4 * sigma), len(grid) - 1)
    offsets = np.arange(-radius, radius + 1)
    weights = np.exp(-0.5 * (offsets / sigma) ** 2)
    uniform = np.interp(grid, distance, curvature)
    filtered = np.convolve(np.pad(uniform, radius, mode="edge"), weights / weights.sum(), mode="valid")
    return np.interp(distance, grid, filtered)


@dataclass(frozen=True)
class SectionGeometry:
    x: np.ndarray
    y: np.ndarray
    inner_x: np.ndarray
    inner_y: np.ndarray
    outer_x: np.ndarray
    outer_y: np.ndarray
    curvature: float
    length: float


def sample_straight(length: float, width: float, step: float) -> SectionGeometry:
    count = example_sample_count(length, step, min_intervals=1)
    x = np.linspace(0.0, length, count)
    y = np.zeros(count)
    return SectionGeometry(x, y, x, y + width / 2.0, x, y - width / 2.0, 0.0, length)


def sample_circle(radius: float, width: float, step: float) -> SectionGeometry:
    length = math.tau * radius
    count = example_sample_count(length, step, min_intervals=4)
    angle = np.linspace(0.0, math.tau, count)
    dx, dy = np.cos(angle), np.sin(angle)
    dx[-1], dy[-1] = dx[0], dy[0]
    inner = max(radius - width / 2.0, 0.0)
    outer = radius + width / 2.0
    return SectionGeometry(
        radius * dx, radius * dy, inner * dx, inner * dy, outer * dx, outer * dy, 1.0 / radius, length,
    )


def example_sample_count(length: float, step: float, *, min_intervals: int) -> int:
    intervals = length / step
    if intervals > MAX_SAMPLES - 1:
        raise ValueError(localize("取樣點超過 {maximum:,}，請增大取樣間距", maximum=MAX_SAMPLES))
    return max(math.ceil(intervals), min_intervals) + 1


def read_sections(path: Path, width: float, step: float, *, check_deadline=None) -> list[SectionGeometry]:
    sections = []
    sample_count = 1
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if not REQUIRED_COLUMNS.issubset(reader.fieldnames or ()):
            raise ValueError(localize("{filename} 缺少 LINE／ARC 分段賽道的必要欄位。", filename=path.name))
        for row in reader:
            if check_deadline is not None:
                check_deadline()
            try:
                length = number(row, "length")
                if length <= 0:
                    raise ValueError(localize("分段長度須大於零"))
                intervals = length / step
                if intervals > MAX_SAMPLES or sample_count + math.ceil(intervals) > MAX_SAMPLES:
                    raise ValueError(localize("取樣點超過 {maximum:,}，請增大取樣間距", maximum=MAX_SAMPLES))
                count = max(math.ceil(intervals), 1) + 1
                sections.append(section_geometry(row, count, width, length))
                sample_count += count - 1
            except (KeyError, ValueError) as exc:
                raise ValueError(localize(
                    "{filename} 第 {line_number} 列：{exc}",
                    filename=path.name, line_number=reader.line_num, exc=exc,
                )) from exc
    if not sections:
        raise ValueError(localize("{filename} 沒有賽道分段。", filename=path.name))
    return sections


def number(row, key):
    try:
        value = float(row.get(key, ""))
    except (ValueError, TypeError) as exc:
        raise ValueError(localize("{key} 須為數字", key=key)) from exc
    if not math.isfinite(value):
        raise ValueError(localize("{key} 須為有限值", key=key))
    return value


def section_geometry(row, count, width, length):
    kind = (row.get("Type") or "").strip().upper()
    start_x, start_y = number(row, "Start_X"), number(row, "Start_Y")
    end_x, end_y = number(row, "End_X"), number(row, "End_Y")
    if kind == "LINE":
        if (start_x, start_y) == (end_x, end_y):
            raise ValueError(localize("直線起點與終點須不同"))
        x = np.linspace(start_x, end_x, count)
        y = np.linspace(start_y, end_y, count)
        curvature = 0.0
    elif kind == "ARC":
        center_x, center_y = number(row, "Center_X"), number(row, "Center_Y")
        radius = number(row, "Radius")
        direction = (row.get("Direction") or "").strip().lower()
        if radius <= 0 or direction not in ("clockwise", "counterclockwise"):
            raise ValueError(localize("圓弧須有正半徑及 Clockwise 或 Counterclockwise 方向"))
        start = np.deg2rad(number(row, "Start_Angle (deg)"))
        end = np.deg2rad(number(row, "End_Angle (deg)"))
        sign = -1 if direction == "clockwise" else 1
        delta = sign * ((sign * (end - start)) % (2.0 * np.pi))
        if delta == 0:
            raise ValueError(localize("圓弧角度跨度須大於零，整圓可拆成多段"))
        angle = start + np.linspace(0.0, delta, count)
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        curvature = sign / radius
    else:
        raise ValueError(localize("Type 須為 LINE 或 ARC"))

    dx, dy = np.gradient(x), np.gradient(y)
    norm = np.maximum(np.hypot(dx, dy), 1e-12)
    offset_x = -(dy / norm) * (width / 2.0)
    offset_y = (dx / norm) * (width / 2.0)
    return SectionGeometry(
        x, y, x + offset_x, y + offset_y, x - offset_x, y - offset_y, curvature, length
    )
