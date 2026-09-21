from __future__ import annotations

from time import perf_counter

from lap_simulation_runner import run_lap_simulation


def main() -> None:
    start_time = perf_counter()
    result = run_lap_simulation()
    elapsed_time = perf_counter() - start_time

    print("Lap simulation benchmark complete")
    print(f"Runtime: {elapsed_time:.3f} s")
    print(f"Lap time: {result.lap_time_s:.3f} s")


if __name__ == "__main__":
    main()
