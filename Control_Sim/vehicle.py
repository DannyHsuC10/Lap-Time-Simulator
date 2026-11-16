"""Compatibility imports for the standalone Control_Sim demo."""

from pathlib import Path
import sys

# Keep one vehicle model while allowing LapSimLab to run as a standalone folder.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "LapSimLab"))
from lap_sim_lab.control_vehicle import (  # noqa: E402
    VehicleParameters,
    VehicleState,
    draw_vehicle,
    handle_input,
    update_vehicle,
)

__all__ = ["VehicleParameters", "VehicleState", "draw_vehicle", "handle_input", "update_vehicle"]
