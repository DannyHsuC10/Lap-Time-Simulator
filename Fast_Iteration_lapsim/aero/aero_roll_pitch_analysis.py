"""Plot total downforce against roll angle using the local aero model."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from Aero import ModalAeroModel


def main():
    # Keep the operating point fixed so the plot shows only roll sensitivity.
    speed = 16.7  # m/s
    heave = -0.01  # m
    pitch = 0.0  # m, model pitch coordinate
    roll_deg = np.linspace(-3.0, 3.0, 121)

    aero = ModalAeroModel()

    downforce = np.array(
        [
            aero.calculate_from_modal(
                speed,
                qh=heave,
                qp=pitch,
                qr=np.deg2rad(angle),
            )["F_down"]
            for angle in roll_deg
        ]
    )

    pitch_deg = np.linspace(-3.0, 3.0, 121)
    pitch_downforce = np.array(
        [
            aero.calculate_from_modal(
                speed,
                qh=heave,
                qp=np.deg2rad(angle),
                qr=0.0,
            )["F_down"]
            for angle in pitch_deg
        ]
    )

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(roll_deg, downforce, color="tab:blue", linewidth=2)
    ax.set_xlabel("Roll angle (deg)")
    ax.set_ylabel("Total downforce (N)")
    ax.set_title("Downforce vs Roll Angle")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    output_path = Path(__file__).resolve().parent / "roll_downforce_2d.png"
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"Saved plot to: {output_path}")

    pitch_fig, pitch_ax = plt.subplots(figsize=(8, 5))
    pitch_ax.plot(pitch_deg, pitch_downforce, color="tab:orange", linewidth=2)
    pitch_ax.set_xlabel("Pitch angle (deg)")
    pitch_ax.set_ylabel("Total downforce (N)")
    pitch_ax.set_title("Downforce vs Pitch Angle")
    pitch_ax.grid(True, alpha=0.3)
    pitch_fig.tight_layout()

    pitch_output_path = Path(__file__).resolve().parent / "pitch_downforce_2d.png"
    pitch_fig.savefig(pitch_output_path, dpi=150, bbox_inches="tight")
    print(f"Saved plot to: {pitch_output_path}")
    plt.show()


if __name__ == "__main__":
    main()
