from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd


@dataclass
class TrackSection:
    section_id: int
    section_type: str
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    center_x: float | None
    center_y: float | None
    radius: float
    start_angle_deg: float | None
    end_angle_deg: float | None
    length: float
    direction_name: str
    signed_radius: float
    turn_direction: str
    max_corner_speed: float
    exit_target_speed: float


class SectionGeometry:
    """
    Track-like geometry for one CSV section.

    It exposes x, y, psi, s, boundaries, nearest_point, and project so existing
    path-following drivers can use a section the same way they use Track.
    """

    def __init__(
        self,
        section,
        x,
        y,
        psi,
        track_width=3.0,
    ):
        self.section = section
        self.radius = section.radius
        self.signed_radius = section.signed_radius
        self.length = section.length
        self.track_width = track_width
        self.closed = False

        self.x = np.asarray(x, dtype=float)
        self.y = np.asarray(y, dtype=float)
        self.psi = np.unwrap(np.asarray(psi, dtype=float))
        self.heading = self.psi

        ds = np.hypot(
            np.diff(self.x, prepend=self.x[0]),
            np.diff(self.y, prepend=self.y[0]),
        )
        self.s = np.cumsum(ds)
        self.s -= self.s[0]

        if len(self.s) > 1 and self.s[-1] > 1e-9:
            self.s *= self.length / self.s[-1]
        else:
            self.s = np.linspace(0.0, self.length, len(self.x))

        self._compute_boundary()

    def _compute_boundary(self):
        dx = np.gradient(self.x)
        dy = np.gradient(self.y)
        norm = np.maximum(np.hypot(dx, dy), 1e-12)

        tx = dx / norm
        ty = dy / norm

        nx = -ty
        ny = tx

        offset = self.track_width / 2.0
        self.inner_x = self.x + nx * offset
        self.inner_y = self.y + ny * offset
        self.outer_x = self.x - nx * offset
        self.outer_y = self.y - ny * offset

    def nearest_point(self, x, y):
        dist = (self.x - x) ** 2 + (self.y - y) ** 2
        return int(np.argmin(dist))

    def project(self, x, y):
        idx = self.nearest_point(x, y)
        dx = x - self.x[idx]
        dy = y - self.y[idx]
        psi = self.psi[idx]

        e_y = -np.sin(psi) * dx + np.cos(psi) * dy
        return idx, e_y

    @property
    def start_pose(self):
        return self.x[0], self.y[0], self.psi[0]

    @property
    def end_pose(self):
        return self.x[-1], self.y[-1], self.psi[-1]


class SegmentTrack:
    """
    Loader for fsae_A_2025_track.csv section data.
    """

    def __init__(
        self,
        csv_file="fsae_A_2025_track.csv",
        track_width=3.0,
        ds=0.2,
        mux=1.5,
        muy=1.7,
        max_straight_speed=30.0,
    ):
        self.csv_file = Path(csv_file)
        self.track_width = float(track_width)
        self.ds = float(ds)
        self.mux = float(mux)
        self.muy = float(muy)
        self.max_straight_speed = float(max_straight_speed)
        self.g = 9.81

        self.dataframe = pd.read_csv(self.csv_file)
        self.dataframe = self.dataframe.dropna(
            subset=["section", "Type"]
        ).copy()
        self.sections = self._build_sections()
        self.section_geometries = [
            self.geometry_for_section(section)
            for section in self.sections
        ]

    def _build_sections(self):
        sections = []

        for row in self.dataframe.to_dict("records"):
            section_type = str(row["Type"]).upper()
            direction_name = (
                ""
                if pd.isna(row.get("Direction"))
                else str(row.get("Direction"))
            )
            turn_direction = self._turn_direction(section_type, direction_name)
            radius = self._finite_or_inf(row.get("Radius"))
            signed_radius = self._signed_radius(radius, turn_direction)
            max_corner_speed = self._max_speed(radius)

            sections.append(
                TrackSection(
                    section_id=int(row["section"]),
                    section_type=section_type,
                    start_x=float(row["Start_X"]),
                    start_y=float(row["Start_Y"]),
                    end_x=float(row["End_X"]),
                    end_y=float(row["End_Y"]),
                    center_x=self._optional_float(row.get("Center_X")),
                    center_y=self._optional_float(row.get("Center_Y")),
                    radius=radius,
                    start_angle_deg=self._optional_float(row.get("Start_Angle (deg)")),
                    end_angle_deg=self._optional_float(row.get("End_Angle (deg)")),
                    length=float(row["length"]),
                    direction_name=direction_name,
                    signed_radius=signed_radius,
                    turn_direction=turn_direction,
                    max_corner_speed=max_corner_speed,
                    exit_target_speed=max_corner_speed,
                )
            )

        self._assign_exit_targets(sections)
        return sections

    def _assign_exit_targets(self, sections):
        for index, section in enumerate(sections):
            next_section = sections[(index + 1) % len(sections)]
            section.exit_target_speed = min(
                section.max_corner_speed,
                next_section.max_corner_speed,
            )

    @staticmethod
    def _optional_float(value):
        if value is None or pd.isna(value):
            return None

        return float(value)

    @staticmethod
    def _finite_or_inf(value):
        if value is None or pd.isna(value):
            return np.inf

        return float(value)

    @staticmethod
    def _turn_direction(section_type, direction_name):
        if section_type == "LINE":
            return "straight"

        if direction_name == "Counterclockwise":
            return "left"

        if direction_name == "Clockwise":
            return "right"

        return "unknown"

    @staticmethod
    def _signed_radius(radius, turn_direction):
        if not np.isfinite(radius):
            return np.inf

        if turn_direction == "right":
            return -abs(radius)

        return abs(radius)

    def _max_speed(self, radius):
        if not np.isfinite(radius):
            return self.max_straight_speed

        return float(np.sqrt(self.muy * self.g * abs(radius)))

    def geometry_for_section(self, section):
        n = max(int(np.ceil(section.length / self.ds)) + 1, 2)

        if section.section_type == "LINE":
            x = np.linspace(section.start_x, section.end_x, n)
            y = np.linspace(section.start_y, section.end_y, n)
            heading = np.arctan2(
                section.end_y - section.start_y,
                section.end_x - section.start_x,
            )
            psi = np.full(n, heading)
            return SectionGeometry(section, x, y, psi, self.track_width)

        if (
            section.center_x is None
            or section.center_y is None
            or section.start_angle_deg is None
            or section.end_angle_deg is None
        ):
            raise ValueError(
                f"ARC section {section.section_id} needs center and angle data"
            )

        start = np.deg2rad(section.start_angle_deg)
        end = np.deg2rad(section.end_angle_deg)
        delta = self._arc_delta(start, end, section.direction_name)
        angle = start + np.linspace(0.0, delta, n)

        x = section.center_x + section.radius * np.cos(angle)
        y = section.center_y + section.radius * np.sin(angle)

        tangent_offset = np.pi / 2.0 if delta >= 0.0 else -np.pi / 2.0
        psi = angle + tangent_offset

        return SectionGeometry(section, x, y, psi, self.track_width)

    def origin_geometry_for_section(self, section):
        """
        Build the same section shape with start pose at x=0, y=0, heading=0.
        """
        n = max(int(np.ceil(section.length / self.ds)) + 1, 2)

        if section.section_type == "LINE":
            x = np.linspace(0.0, section.length, n)
            y = np.zeros(n)
            psi = np.zeros(n)
            return SectionGeometry(section, x, y, psi, self.track_width)

        s = np.linspace(0.0, section.length, n)
        radius = abs(section.radius)
        theta = s / radius

        if section.turn_direction == "right":
            x = radius * np.sin(theta)
            y = -radius * (1.0 - np.cos(theta))
            psi = -theta
        else:
            x = radius * np.sin(theta)
            y = radius * (1.0 - np.cos(theta))
            psi = theta

        return SectionGeometry(section, x, y, psi, self.track_width)

    @staticmethod
    def _arc_delta(start, end, direction_name):
        if direction_name == "Counterclockwise":
            delta = (end - start) % (2.0 * np.pi)
        elif direction_name == "Clockwise":
            delta = -((start - end) % (2.0 * np.pi))
        else:
            delta = end - start

        return delta

    def summary(self):
        return pd.DataFrame(
            {
                "section": [section.section_id for section in self.sections],
                "type": [section.section_type for section in self.sections],
                "turn": [section.turn_direction for section in self.sections],
                "radius_m": [section.radius for section in self.sections],
                "length_m": [section.length for section in self.sections],
                "max_speed_mps": [
                    section.max_corner_speed
                    for section in self.sections
                ],
                "exit_target_mps": [
                    section.exit_target_speed
                    for section in self.sections
                ],
            }
        )
