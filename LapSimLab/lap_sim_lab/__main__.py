import sys


def main():
    try:
        from .gui import LapSimLab
    except ModuleNotFoundError as exc:
        if exc.name == "numpy":
            print("Install NumPy with: python -m pip install -r requirements.txt", file=sys.stderr)
            return 1
        raise
    LapSimLab().mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
