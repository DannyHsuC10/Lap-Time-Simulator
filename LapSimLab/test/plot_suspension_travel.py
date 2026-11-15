"""Read a complete LapSimLab CSV and plot four-wheel displacement to SVG."""

import argparse
import csv
from math import ceil, floor, isfinite, log10
from pathlib import Path
from xml.sax.saxutils import escape


WHEELS = ("FL", "FR", "RL", "RR")
WHEEL_TITLES = ("左前 FL", "右前 FR", "左後 RL", "右後 RR")
SERIES = (
    ("zs", "簧上位移", "#1d64b4", ""),
    ("zu", "簧下位移", "#008578", ""),
    ("zs_minus_zu", "相對位移（簧上 − 簧下）", "#c44920", ' stroke-dasharray="6 3"'),
)


def read_csv(path):
    columns = {
        name: [] for name in (
            "time_s", *(f"{key}_{wheel}_mm" for wheel in WHEELS for key, *_ in SERIES)
        )
    }
    with path.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = columns.keys() - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"缺少欄位 {', '.join(sorted(missing))}。請使用 LapSimLab 的完整 CSV。")
        for row in reader:
            for key, values in columns.items():
                try:
                    value = float(row[key])
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"第 {reader.line_num} 列的 {key} 須為數字。") from exc
                if not isfinite(value):
                    raise ValueError(f"第 {reader.line_num} 列的 {key} 須為有限值。")
                values.append(value)

    time = columns.pop("time_s")
    if len(time) < 2 or time[-1] <= time[0] or any(a > b for a, b in zip(time, time[1:])):
        raise ValueError("time_s 須由小到大排列，並至少有兩個不同的時間點。")
    return time, columns


def displacement_ticks(columns):
    minimum = min(0.0, *(min(values) for values in columns.values()))
    maximum = max(0.0, *(max(values) for values in columns.values()))
    if minimum == maximum:
        return [-1, 0, 1]
    target = (maximum - minimum) / 6
    decade = 10 ** floor(log10(target))
    step = decade * next(multiplier for multiplier in (1, 2, 2.5, 5, 10) if multiplier * decade >= target)
    return [i * step for i in range(floor(minimum / step), ceil(maximum / step) + 1)]


def write_plot(path, source_name, time, columns):
    ticks = displacement_ticks(columns)
    drawing = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1360" height="858" viewBox="0 0 1360 858">',
        '<rect width="1360" height="858" fill="#f1f5f9"/>',
        '<g font-family="Microsoft JhengHei, Noto Sans CJK TC, sans-serif" fill="#17283c">',
    ]

    def label(x, y, text, size=14, anchor="start", weight="normal"):
        drawing.append(
            f'<text x="{x:g}" y="{y:g}" font-size="{size}" text-anchor="{anchor}" '
            f'font-weight="{weight}">{escape(text)}</text>'
        )

    def line(x1, y1, x2, y2, color, width=1, style=""):
        drawing.append(
            f'<path d="M{x1:g} {y1:g}L{x2:g} {y2:g}" fill="none" '
            f'stroke="{color}" stroke-width="{width}"{style}/>'
        )

    label(32, 43, "四輪位移隨時間變化", 29, weight="bold")
    label(32, 73, f"{source_name}  ·  {len(time):,} 個取樣點  ·  {time[0]:.3f}–{time[-1]:.3f} s", 15)
    for x, (_, name, color, style) in zip((32, 168, 304), SERIES):
        line(x, 98, x + 28, 98, color, 3, style)
        label(x + 36, 103, name)
    label(1328, 103, "四圖共用位移刻度  ·  相對位移負值為壓縮", anchor="end")

    for index, (wheel, title) in enumerate(zip(WHEELS, WHEEL_TITLES)):
        panel_x = 32 + (index % 2) * 657
        panel_y = 125 + (index // 2) * 361
        drawing.append(
            f'<rect x="{panel_x}" y="{panel_y}" width="639" height="341" '
            'rx="11" fill="#ffffff" stroke="#dce5ef"/>'
        )
        label(panel_x + 20, panel_y + 30, title, 19, weight="bold")
        label(panel_x + 618, panel_y + 30, "位移 [mm]", anchor="end")
        left, right = panel_x + 66, panel_x + 613
        top, bottom = panel_y + 52, panel_y + 285

        def x_at(value):
            return left + (value - time[0]) / (time[-1] - time[0]) * (right - left)

        def y_at(value):
            return bottom - (value - ticks[0]) / (ticks[-1] - ticks[0]) * (bottom - top)

        for tick in ticks:
            y = y_at(tick)
            line(left, y, right, y, "#8b9db1" if tick == 0 else "#e4ebf3")
            label(left - 9, y + 5, f"{tick:g}", 13, "end")
        for i in range(6):
            value = time[0] + (time[-1] - time[0]) * i / 5
            x = x_at(value)
            line(x, top, x, bottom, "#e4ebf3")
            label(x, bottom + 22, f"{value:.4g}", 13, "middle")
        label((left + right) / 2, bottom + 43, "時間 [s]", 14, "middle")

        x_positions = [x_at(value) for value in time]
        for key, _, color, style in SERIES:
            points = " ".join(
                f"{x:.2f},{y_at(value):.2f}"
                for x, value in zip(x_positions, columns[f"{key}_{wheel}_mm"])
            )
            drawing.append(
                f'<polyline points="{points}" fill="none" stroke="{color}" '
                f'stroke-width="1.6" stroke-linejoin="round"{style}/>'
            )
    drawing.extend(("</g>", "</svg>"))
    path.write_text("\n".join(drawing), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="繪製四輪簧上、簧下與相對位移的時間曲線。")
    parser.add_argument("csv", type=Path, nargs="?", default=Path(__file__).with_name("lap_result.csv"))
    parser.add_argument("-o", "--output", type=Path, help="指定輸出的 SVG 路徑。")
    args = parser.parse_args()
    output = args.output or args.csv.with_name(f"{args.csv.stem}_suspension.svg")
    if output.suffix.lower() != ".svg":
        parser.error("輸出檔名須以 .svg 結尾。")
    try:
        time, columns = read_csv(args.csv)
        write_plot(output, args.csv.name, time, columns)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"{exc}\n")
    print(output.resolve())


if __name__ == "__main__":
    main()
