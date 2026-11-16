from dataclasses import dataclass
import colorsys
import csv
from pathlib import Path
import pickle
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np


VIEWER_DIR = Path(__file__).resolve().parent
PROJECT_DIR = VIEWER_DIR.parent
TARGET_PATH_DS = 0.05

if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))


@dataclass
class SectionSimulationResult:
    section: object
    geometry: object
    result: dict
    commands: dict
    completed: bool
    final_progress: float
    final_travel_distance: float
    elapsed_time: float


class LapDataUnpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module == "__main__" and name == "SectionSimulationResult":
            return SectionSimulationResult
        return super().find_class(module, name)


def load_pickle(path):
    with Path(path).open("rb") as file:
        return LapDataUnpickler(file).load()


def display_path(path):
    path = Path(path)
    try:
        return str(path.relative_to(VIEWER_DIR))
    except ValueError:
        return str(path)


def clean_row(row):
    return {key.lstrip("\ufeff"): value for key, value in row.items()}


def to_float(value, default=None):
    if value is None or value == "":
        return default
    return float(value)


def arc_delta(start, end, direction):
    if direction == "Counterclockwise":
        return (end - start) % (2.0 * np.pi)
    if direction == "Clockwise":
        return -((start - end) % (2.0 * np.pi))
    return end - start


def section_target_points(row, ds=TARGET_PATH_DS):
    section_type = row["Type"].upper()
    length = to_float(row["length"], 0.0)
    n = max(int(np.ceil(length / ds)) + 1, 2)

    start_x = to_float(row["Start_X"])
    start_y = to_float(row["Start_Y"])
    end_x = to_float(row["End_X"])
    end_y = to_float(row["End_Y"])

    if section_type == "LINE":
        return np.linspace(start_x, end_x, n), np.linspace(start_y, end_y, n)

    center_x = to_float(row["Center_X"])
    center_y = to_float(row["Center_Y"])
    radius = to_float(row["Radius"])
    start = np.deg2rad(to_float(row["Start_Angle (deg)"]))
    end = np.deg2rad(to_float(row["End_Angle (deg)"]))
    delta = arc_delta(start, end, row.get("Direction", ""))
    angle = start + np.linspace(0.0, delta, n)

    return center_x + radius * np.cos(angle), center_y + radius * np.sin(angle)


def load_sections_from_csv(csv_path):
    sections = []

    with csv_path.open("r", newline="", encoding="utf-8-sig") as file:
        for row in csv.DictReader(file):
            cleaned = clean_row(row)
            if cleaned.get("section") and cleaned.get("Type"):
                sections.append(cleaned)

    return csv_path, sections


def load_target_sections():
    csv_paths = sorted(VIEWER_DIR.glob("*.csv"))
    if not csv_paths:
        return None, []

    return load_sections_from_csv(csv_paths[0])


def get_package_data(data):
    if isinstance(data, dict) and isinstance(data.get("data"), dict):
        return data["data"]

    if isinstance(data, dict) and isinstance(data.get("result"), dict):
        return data["result"]

    return None


def get_package_metadata(data):
    if isinstance(data, dict) and isinstance(data.get("metadata"), dict):
        return data["metadata"]

    return {}


def format_loaded_status(path, plot_key_count, metadata):
    parts = [f"Loaded {path}", f"{plot_key_count} plottable keys"]

    samples = metadata.get("samples")
    if samples is not None:
        parts.append(f"{samples} samples")

    lap_time = metadata.get("lap_time_s")
    if lap_time is not None:
        parts.append(f"lap time {float(lap_time):.3f} s")

    return " | ".join(parts)


def safe_csv_filename(name):
    stem = Path(name).stem or "lap_data_export"
    safe_chars = []

    for char in stem:
        if char.isalnum() or char in ("-", "_"):
            safe_chars.append(char)
        else:
            safe_chars.append("_")

    return f"{''.join(safe_chars)}_selected.csv"


def get_saved_section_count(data, fallback_path=None):
    if isinstance(data, dict):
        section_results = data.get("section_results")
        if section_results:
            return len(section_results)

        latest = data.get("latest_section_result")
        section = getattr(latest, "section", None)
        section_id = getattr(section, "section_id", None)
        if section_id is not None:
            return int(section_id)

    if fallback_path is not None:
        try:
            return int(Path(fallback_path).parent.name)
        except ValueError:
            return 0

    return 0


def build_target_path(sections, section_count):
    if not sections:
        return None

    if section_count is None or section_count <= 0:
        section_count = len(sections)

    xs = []
    ys = []

    for row in sections[:section_count]:
        x, y = section_target_points(row)
        if xs:
            x = x[1:]
            y = y[1:]
        xs.extend(x)
        ys.extend(y)

    if not xs:
        return None

    return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)


def as_numeric_array(value):
    try:
        array = np.asarray(value)
    except Exception:
        return None

    if array.size == 0:
        return None

    if array.dtype == object:
        converted = []
        for item in array:
            try:
                converted.append(float(item))
            except Exception:
                return None
        array = np.asarray(converted, dtype=float)

    if not np.issubdtype(array.dtype, np.number):
        return None

    return array


def rainbow_color(value, value_min, value_max):
    if not np.isfinite(value):
        return "#888888"

    if value_max <= value_min:
        fraction = 0.5
    else:
        fraction = (float(value) - value_min) / (value_max - value_min)

    fraction = float(np.clip(fraction, 0.0, 1.0))
    hue = (2.0 / 3.0) * (1.0 - fraction)
    red, green, blue = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
    return f"#{int(red * 255):02x}{int(green * 255):02x}{int(blue * 255):02x}"


def finite_min_max(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]

    if finite.size == 0:
        return None

    return float(np.min(finite)), float(np.max(finite))


def make_series_keys(prefix, series_dict, time_array):
    keys = {}
    if not isinstance(series_dict, dict) or time_array is None:
        return keys

    time_length = len(time_array)

    for key, value in sorted(series_dict.items()):
        array = as_numeric_array(value)
        if array is None or len(array) != time_length:
            continue

        if array.ndim == 1:
            keys[f"{prefix}.{key}"] = (time_array, array, f"{prefix}.{key}")
            continue

        flattened = array.reshape((array.shape[0], -1))
        for column_index in range(flattened.shape[1]):
            label = f"{prefix}.{key}[{column_index}]"
            keys[label] = (time_array, flattened[:, column_index], label)

    return keys


def get_time_array(result):
    if not isinstance(result, dict):
        return None

    for key in ("t", "time"):
        if key in result:
            time_array = as_numeric_array(result[key])
            if time_array is not None and time_array.ndim == 1:
                return time_array

    return None


def collect_plot_keys(data):
    keys = {}

    package_data = get_package_data(data)
    package_time = get_time_array(package_data)
    keys.update(make_series_keys("data", package_data, package_time))

    full_result = data.get("result") if isinstance(data, dict) else None
    full_time = get_time_array(full_result)
    keys.update(make_series_keys("result", full_result, full_time))

    latest = data.get("latest_section_result") if isinstance(data, dict) else None
    latest_result = getattr(latest, "result", None)
    latest_time = get_time_array(latest_result)
    keys.update(make_series_keys("latest_section_result.result", latest_result, latest_time))
    keys.update(
        make_series_keys(
            "latest_section_result.commands",
            getattr(latest, "commands", None),
            latest_time,
        )
    )

    return keys


def get_xy_series(data):
    if not isinstance(data, dict):
        return None

    package_data = get_package_data(data)
    if isinstance(package_data, dict) and "s_world" in package_data:
        s_world = as_numeric_array(package_data["s_world"])
        if s_world is not None and s_world.ndim >= 2 and s_world.shape[1] >= 2:
            return s_world[:, 0], s_world[:, 1], get_time_array(package_data), "data.s_world XY"

    result = data.get("result")
    if isinstance(result, dict) and "s_world" in result:
        s_world = as_numeric_array(result["s_world"])
        if s_world is not None and s_world.ndim >= 2 and s_world.shape[1] >= 2:
            return s_world[:, 0], s_world[:, 1], get_time_array(result), "result.s_world XY"

    latest = data.get("latest_section_result")
    latest_result = getattr(latest, "result", None)
    if isinstance(latest_result, dict) and "s_world" in latest_result:
        s_world = as_numeric_array(latest_result["s_world"])
        if s_world is not None and s_world.ndim >= 2 and s_world.shape[1] >= 2:
            return (
                s_world[:, 0],
                s_world[:, 1],
                get_time_array(latest_result),
                "latest_section_result.result.s_world XY",
            )

    return None


class LapDataViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Lap Section Data Viewer")
        self.geometry("1180x760")
        self.minsize(920, 560)

        self.data = None
        self.plot_keys = {}
        self.pkl_paths = []
        self.xy_series = None
        self.target_csv_path, self.target_sections = load_target_sections()
        self.target_xy = None

        self._build_ui()
        self._refresh_files()

    def _build_ui(self):
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)

        top_bar = ttk.Frame(self, padding=(10, 8))
        top_bar.grid(row=0, column=0, columnspan=2, sticky="ew")
        top_bar.columnconfigure(1, weight=1)

        ttk.Label(top_bar, text="Pickle").grid(row=0, column=0, padx=(0, 8))
        self.file_var = tk.StringVar()
        self.file_combo = ttk.Combobox(
            top_bar,
            textvariable=self.file_var,
            state="readonly",
            width=80,
        )
        self.file_combo.grid(row=0, column=1, sticky="ew")
        self.file_combo.bind("<<ComboboxSelected>>", self._on_file_selected)

        ttk.Button(top_bar, text="Browse", command=self._browse_file).grid(
            row=0,
            column=2,
            padx=(8, 0),
        )
        ttk.Button(top_bar, text="Refresh", command=self._refresh_files).grid(
            row=0,
            column=3,
            padx=(8, 0),
        )
        ttk.Button(top_bar, text="XY", command=self._plot_xy).grid(
            row=0,
            column=4,
            padx=(8, 0),
        )
        ttk.Button(top_bar, text="To CSV", command=self._open_csv_export_dialog).grid(
            row=0,
            column=5,
            padx=(8, 0),
        )

        side = ttk.Frame(self, padding=(10, 0, 6, 10))
        side.grid(row=1, column=0, sticky="nsew")
        side.rowconfigure(2, weight=1)

        ttk.Label(side, text="Keys").grid(row=0, column=0, sticky="w")
        self.search_var = tk.StringVar()
        search_entry = ttk.Entry(side, textvariable=self.search_var, width=34)
        search_entry.grid(row=1, column=0, sticky="ew", pady=(6, 6))
        self.search_var.trace_add("write", lambda *_: self._populate_key_list())

        list_frame = ttk.Frame(side)
        list_frame.grid(row=2, column=0, sticky="nsew")
        list_frame.rowconfigure(0, weight=1)
        list_frame.columnconfigure(0, weight=1)

        self.key_list = tk.Listbox(list_frame, width=42, exportselection=False)
        self.key_list.grid(row=0, column=0, sticky="nsew")
        self.key_list.bind("<<ListboxSelect>>", self._on_key_selected)

        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.key_list.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.key_list.configure(yscrollcommand=scrollbar.set)

        plot_frame = ttk.Frame(self, padding=(6, 0, 10, 10))
        plot_frame.grid(row=1, column=1, sticky="nsew")
        plot_frame.rowconfigure(0, weight=1)
        plot_frame.columnconfigure(0, weight=1)

        self.plot_canvas = tk.Canvas(plot_frame, background="#ffffff", highlightthickness=1)
        self.plot_canvas.grid(row=0, column=0, sticky="nsew")
        self.plot_canvas.bind("<Configure>", lambda _event: self._redraw_current_plot())
        self.plot_canvas.bind("<Button-1>", self._on_canvas_click)
        self.current_plot = None
        self.plot_transform = None

        self.status_var = tk.StringVar(value=f"Folder: {VIEWER_DIR}")
        ttk.Label(self, textvariable=self.status_var, anchor="w", padding=(10, 4)).grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="ew",
        )

    def _refresh_files(self):
        self.pkl_paths = sorted(
            VIEWER_DIR.rglob("*.pkl"),
            key=lambda path: (path.name != "track_speed_map_lap_data.pkl", str(path)),
        )
        values = [str(path.relative_to(VIEWER_DIR)) for path in self.pkl_paths]
        self.file_combo["values"] = values

        if self.pkl_paths and not self.file_var.get():
            self.file_combo.current(0)
            self._load_selected_file(self.pkl_paths[0])
        elif not self.pkl_paths:
            self.status_var.set(f"No .pkl files found in {VIEWER_DIR}")

    def _browse_file(self):
        path = filedialog.askopenfilename(
            initialdir=VIEWER_DIR,
            title="Select pickle data",
            filetypes=(("Pickle files", "*.pkl"), ("All files", "*.*")),
        )
        if path:
            self._load_selected_file(Path(path))

    def _on_file_selected(self, _event=None):
        index = self.file_combo.current()
        if index >= 0:
            self._load_selected_file(self.pkl_paths[index])

    def _load_selected_file(self, path):
        try:
            self.data = load_pickle(path)
        except Exception as exc:
            messagebox.showerror("Load failed", f"Could not load:\n{path}\n\n{exc}")
            return

        self.plot_keys = collect_plot_keys(self.data)
        self.xy_series = get_xy_series(self.data)
        self.target_csv_path, self.target_sections = self._load_target_sections_for_data(path)
        section_count = get_saved_section_count(self.data, path)
        self.target_xy = build_target_path(self.target_sections, section_count)
        self.file_var.set(display_path(path))
        self._populate_key_list()
        self.status_var.set(
            format_loaded_status(path, len(self.plot_keys), get_package_metadata(self.data))
        )

        if self.plot_keys:
            self.key_list.selection_clear(0, tk.END)
            self.key_list.selection_set(0)
            self.key_list.activate(0)
            self._plot_key(self.key_list.get(0))
        else:
            self._clear_plot("No plottable numeric keys found")

    def _load_target_sections_for_data(self, data_path):
        metadata = get_package_metadata(self.data)
        track_csv = metadata.get("track_csv")

        if track_csv:
            csv_path = Path(track_csv)
            if not csv_path.is_absolute():
                csv_path = data_path.parent / csv_path

            if csv_path.exists():
                try:
                    return load_sections_from_csv(csv_path)
                except Exception as exc:
                    self.status_var.set(f"Could not load track CSV {csv_path}: {exc}")

        return self.target_csv_path, self.target_sections

    def _populate_key_list(self):
        self.key_list.delete(0, tk.END)
        query = self.search_var.get().strip().lower()
        for key in self.plot_keys:
            if not query or query in key.lower():
                self.key_list.insert(tk.END, key)

    def _on_key_selected(self, _event=None):
        selection = self.key_list.curselection()
        if selection:
            self._plot_key(self.key_list.get(selection[0]))

    def _selected_key(self):
        selection = self.key_list.curselection()
        if not selection:
            return None
        return self.key_list.get(selection[0])

    def _open_csv_export_dialog(self):
        if not self.plot_keys:
            messagebox.showinfo("CSV export unavailable", "No plottable numeric keys are loaded.")
            return

        dialog = tk.Toplevel(self)
        dialog.title("Export CSV")
        dialog.geometry("520x620")
        dialog.minsize(420, 420)
        dialog.transient(self)

        dialog.columnconfigure(0, weight=1)
        dialog.rowconfigure(2, weight=1)

        selected_key = self._selected_key()
        variables = {
            key: tk.BooleanVar(value=(key == selected_key))
            for key in self.plot_keys
        }

        file_frame = ttk.Frame(dialog, padding=(10, 10, 10, 4))
        file_frame.grid(row=0, column=0, sticky="ew")
        file_frame.columnconfigure(1, weight=1)

        ttk.Label(file_frame, text="CSV file").grid(row=0, column=0, sticky="w", padx=(0, 8))
        output_var = tk.StringVar(value=safe_csv_filename(self.file_var.get()))
        ttk.Entry(file_frame, textvariable=output_var).grid(row=0, column=1, sticky="ew")

        tools = ttk.Frame(dialog, padding=(10, 4))
        tools.grid(row=1, column=0, sticky="ew")
        tools.columnconfigure(0, weight=1)

        filter_var = tk.StringVar()
        filter_entry = ttk.Entry(tools, textvariable=filter_var)
        filter_entry.grid(row=0, column=0, sticky="ew", padx=(0, 8))

        list_canvas = tk.Canvas(dialog, highlightthickness=1, background="#ffffff")
        list_canvas.grid(row=2, column=0, sticky="nsew", padx=10, pady=(4, 8))
        list_frame = ttk.Frame(list_canvas)
        list_window = list_canvas.create_window((0, 0), window=list_frame, anchor="nw")
        scrollbar = ttk.Scrollbar(dialog, orient="vertical", command=list_canvas.yview)
        scrollbar.grid(row=2, column=1, sticky="ns", pady=(4, 8))
        list_canvas.configure(yscrollcommand=scrollbar.set)

        def resize_list_window(event):
            list_canvas.itemconfigure(list_window, width=event.width)

        def update_scroll_region(_event=None):
            list_canvas.configure(scrollregion=list_canvas.bbox("all"))

        list_canvas.bind("<Configure>", resize_list_window)
        list_frame.bind("<Configure>", update_scroll_region)

        def populate_checks(*_args):
            for child in list_frame.winfo_children():
                child.destroy()

            query = filter_var.get().strip().lower()
            row = 0
            for key in self.plot_keys:
                if query and query not in key.lower():
                    continue
                ttk.Checkbutton(
                    list_frame,
                    text=key,
                    variable=variables[key],
                ).grid(row=row, column=0, sticky="w", padx=8, pady=2)
                row += 1

            update_scroll_region()

        def set_all(value):
            query = filter_var.get().strip().lower()
            for key, variable in variables.items():
                if not query or query in key.lower():
                    variable.set(value)

        def save_csv():
            selected = [key for key, variable in variables.items() if variable.get()]
            if not selected:
                messagebox.showinfo("No fields selected", "Select at least one field to export.")
                return

            filename = output_var.get().strip()
            if not filename:
                messagebox.showinfo("Missing filename", "Enter a CSV filename.")
                return
            if not filename.lower().endswith(".csv"):
                filename += ".csv"

            output_path = VIEWER_DIR / Path(filename).name
            max_length = max(len(self.plot_keys[key][1]) for key in selected)

            try:
                with output_path.open("w", newline="", encoding="utf-8-sig") as file:
                    writer = csv.writer(file)
                    writer.writerow(["index", *selected])
                    for row_index in range(max_length):
                        row = [row_index]
                        for key in selected:
                            values = self.plot_keys[key][1]
                            if row_index < len(values):
                                row.append(values[row_index])
                            else:
                                row.append("")
                        writer.writerow(row)
            except Exception as exc:
                messagebox.showerror("Save failed", f"Could not save CSV:\n{output_path}\n\n{exc}")
                return

            relative_path = output_path.relative_to(VIEWER_DIR)
            self.status_var.set(f"Saved CSV: {relative_path} ({len(selected)} fields)")
            messagebox.showinfo("CSV saved", f"Saved {relative_path}")
            dialog.destroy()

        filter_var.trace_add("write", populate_checks)
        populate_checks()

        button_frame = ttk.Frame(dialog, padding=(10, 0, 10, 10))
        button_frame.grid(row=3, column=0, columnspan=2, sticky="ew")
        ttk.Button(button_frame, text="Select visible", command=lambda: set_all(True)).pack(
            side="left"
        )
        ttk.Button(button_frame, text="Clear visible", command=lambda: set_all(False)).pack(
            side="left",
            padx=(8, 0),
        )
        ttk.Button(button_frame, text="Save", command=save_csv).pack(side="right")
        ttk.Button(button_frame, text="Cancel", command=dialog.destroy).pack(
            side="right",
            padx=(0, 8),
        )

    def _plot_key(self, key):
        if key not in self.plot_keys:
            return

        time_array, value_array, label = self.plot_keys[key]
        self.current_plot = {
            "mode": "time",
            "x": time_array,
            "y": value_array,
            "label": label,
            "x_label": "Time (s)",
            "y_label": label,
            "equal_aspect": False,
        }
        self._redraw_current_plot()
        self.status_var.set(f"Showing {label}: {len(value_array)} samples")

    def _plot_xy(self):
        if self.xy_series is None:
            messagebox.showinfo("XY unavailable", "This pickle does not contain s_world XY data.")
            return

        x_array, y_array, time_array, label = self.xy_series
        selected_key = self._selected_key()
        color_values = None
        color_label = None

        if selected_key in self.plot_keys:
            _, value_array, value_label = self.plot_keys[selected_key]
            value_array = np.asarray(value_array, dtype=float)
            if len(value_array) == len(x_array):
                color_values = value_array
                color_label = value_label

        self.current_plot = {
            "mode": "xy",
            "x": x_array,
            "y": y_array,
            "time": time_array,
            "color_values": color_values,
            "color_label": color_label,
            "label": label,
            "x_label": "World X (m)",
            "y_label": "World Y (m)",
            "equal_aspect": True,
            "extra_paths": [
                {
                    "x": self.target_xy[0],
                    "y": self.target_xy[1],
                    "label": "Ideal path",
                    "color": "#d07a00",
                    "width": 2,
                }
            ]
            if self.target_xy is not None
            else [],
            "selected_index": None,
        }
        self._redraw_current_plot()
        if color_label:
            self.status_var.set(
                f"Showing world XY rainbow map by {color_label}: {len(x_array)} samples"
            )
        else:
            self.status_var.set(
                f"Showing world XY path: {len(x_array)} samples. Select a same-length signal for rainbow color."
            )

    def _clear_plot(self, message):
        self.current_plot = None
        self.plot_canvas.delete("all")
        self.plot_canvas.create_text(
            self.plot_canvas.winfo_width() / 2,
            self.plot_canvas.winfo_height() / 2,
            text=message,
            fill="#444444",
            font=("Segoe UI", 14),
        )

    def _redraw_current_plot(self):
        self.plot_canvas.delete("all")

        if self.current_plot is None:
            self.plot_canvas.create_text(
                self.plot_canvas.winfo_width() / 2,
                self.plot_canvas.winfo_height() / 2,
                text="Select a pickle file and a key",
                fill="#444444",
                font=("Segoe UI", 14),
            )
            return

        x_array = self.current_plot["x"]
        y_array = self.current_plot["y"]
        label = self.current_plot["label"]
        x_label = self.current_plot["x_label"]
        selected_index = self.current_plot.get("selected_index")
        extra_paths = self.current_plot.get("extra_paths", [])
        color_values = self.current_plot.get("color_values")
        color_label = self.current_plot.get("color_label")
        width = max(self.plot_canvas.winfo_width(), 2)
        height = max(self.plot_canvas.winfo_height(), 2)
        left = 72
        right = 88 if color_values is not None else 24
        top = 48
        bottom = 54
        plot_width = max(width - left - right, 1)
        plot_height = max(height - top - bottom, 1)

        finite_mask = np.isfinite(x_array) & np.isfinite(y_array)
        x = np.asarray(x_array[finite_mask], dtype=float)
        y = np.asarray(y_array[finite_mask], dtype=float)

        if len(x) == 0:
            self.plot_canvas.create_text(
                width / 2,
                height / 2,
                text="No finite values to plot",
                fill="#444444",
                font=("Segoe UI", 14),
            )
            return

        x_min = float(np.min(x))
        x_max = float(np.max(x))
        y_min = float(np.min(y))
        y_max = float(np.max(y))

        for path in extra_paths:
            path_x = np.asarray(path["x"], dtype=float)
            path_y = np.asarray(path["y"], dtype=float)
            path_mask = np.isfinite(path_x) & np.isfinite(path_y)
            if np.any(path_mask):
                x_min = min(x_min, float(np.min(path_x[path_mask])))
                x_max = max(x_max, float(np.max(path_x[path_mask])))
                y_min = min(y_min, float(np.min(path_y[path_mask])))
                y_max = max(y_max, float(np.max(path_y[path_mask])))

        if x_min == x_max:
            x_min -= 0.5
            x_max += 0.5
        if y_min == y_max:
            y_min -= 0.5
            y_max += 0.5

        if self.current_plot["equal_aspect"]:
            x_span = x_max - x_min
            y_span = y_max - y_min
            data_aspect = x_span / y_span
            canvas_aspect = plot_width / plot_height

            if data_aspect > canvas_aspect:
                target_y_span = x_span / canvas_aspect
                y_center = (y_min + y_max) / 2
                y_min = y_center - target_y_span / 2
                y_max = y_center + target_y_span / 2
            else:
                target_x_span = y_span * canvas_aspect
                x_center = (x_min + x_max) / 2
                x_min = x_center - target_x_span / 2
                x_max = x_center + target_x_span / 2

        self.plot_transform = {
            "left": left,
            "right": left + plot_width,
            "top": top,
            "bottom": top + plot_height,
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
        }

        def to_canvas_x(value):
            return left + (value - x_min) / (x_max - x_min) * plot_width

        def to_canvas_y(value):
            return top + (y_max - value) / (y_max - y_min) * plot_height

        self.plot_canvas.create_text(
            width / 2,
            20,
            text=label,
            fill="#111111",
            font=("Segoe UI", 13, "bold"),
        )

        axis_color = "#555555"
        grid_color = "#dddddd"
        for step in range(6):
            fraction = step / 5
            gx = left + fraction * plot_width
            gy = top + fraction * plot_height
            self.plot_canvas.create_line(gx, top, gx, top + plot_height, fill=grid_color)
            self.plot_canvas.create_line(left, gy, left + plot_width, gy, fill=grid_color)

            x_tick = x_min + fraction * (x_max - x_min)
            y_tick = y_max - fraction * (y_max - y_min)
            self.plot_canvas.create_text(gx, height - 28, text=f"{x_tick:.2f}", fill=axis_color, font=("Segoe UI", 9))
            self.plot_canvas.create_text(42, gy, text=f"{y_tick:.2g}", fill=axis_color, font=("Segoe UI", 9))

        self.plot_canvas.create_line(left, top, left, top + plot_height, fill=axis_color, width=2)
        self.plot_canvas.create_line(left, top + plot_height, left + plot_width, top + plot_height, fill=axis_color, width=2)
        self.plot_canvas.create_text(width / 2, height - 10, text=x_label, fill="#111111", font=("Segoe UI", 10))

        for path in extra_paths:
            path_x = np.asarray(path["x"], dtype=float)
            path_y = np.asarray(path["y"], dtype=float)
            path_mask = np.isfinite(path_x) & np.isfinite(path_y)
            target_points = []
            for x_value, y_value in zip(path_x[path_mask], path_y[path_mask]):
                target_points.extend((to_canvas_x(x_value), to_canvas_y(y_value)))

            if len(target_points) >= 4:
                self.plot_canvas.create_line(
                    *target_points,
                    fill=path.get("color", "#d07a00"),
                    width=path.get("width", 2),
                    smooth=False,
                )

        points = []
        for x_value, y_value in zip(x, y):
            points.extend((to_canvas_x(x_value), to_canvas_y(y_value)))

        color_limits = finite_min_max(color_values) if color_values is not None else None
        if (
            color_values is not None
            and color_limits is not None
            and len(color_values) == len(x_array)
            and len(x_array) >= 2
        ):
            color_min, color_max = color_limits
            self._draw_colored_xy_path(
                x_array,
                y_array,
                np.asarray(color_values, dtype=float),
                color_min,
                color_max,
                to_canvas_x,
                to_canvas_y,
            )
            self._draw_colorbar(
                left + plot_width + 24,
                top,
                height=plot_height,
                value_min=color_min,
                value_max=color_max,
                label=color_label,
            )
        elif len(points) >= 4:
            self.plot_canvas.create_line(*points, fill="#0f6b8f", width=2, smooth=False)
        elif points:
            self.plot_canvas.create_oval(points[0] - 3, points[1] - 3, points[0] + 3, points[1] + 3, fill="#0f6b8f", outline="")

        if len(points) >= 2:
            self.plot_canvas.create_oval(points[0] - 4, points[1] - 4, points[0] + 4, points[1] + 4, fill="#1f9d55", outline="")
            self.plot_canvas.create_oval(points[-2] - 4, points[-1] - 4, points[-2] + 4, points[-1] + 4, fill="#d64545", outline="")

        if selected_index is not None and self.current_plot.get("mode") == "xy":
            self._draw_xy_point_label(selected_index, to_canvas_x, to_canvas_y)

    def _draw_colored_xy_path(
        self,
        x_array,
        y_array,
        color_values,
        color_min,
        color_max,
        to_canvas_x,
        to_canvas_y,
    ):
        for index in range(len(x_array) - 1):
            segment_values = (
                x_array[index],
                y_array[index],
                color_values[index],
                x_array[index + 1],
                y_array[index + 1],
                color_values[index + 1],
            )
            if not np.all(np.isfinite(segment_values)):
                continue

            segment_color = rainbow_color(
                0.5 * (color_values[index] + color_values[index + 1]),
                color_min,
                color_max,
            )
            self.plot_canvas.create_line(
                to_canvas_x(x_array[index]),
                to_canvas_y(y_array[index]),
                to_canvas_x(x_array[index + 1]),
                to_canvas_y(y_array[index + 1]),
                fill=segment_color,
                width=3,
                smooth=False,
            )

    def _draw_colorbar(self, x, y, height, value_min, value_max, label):
        bar_width = 14
        steps = max(int(height), 1)

        for step in range(steps):
            fraction = 1.0 - step / max(steps - 1, 1)
            value = value_min + fraction * (value_max - value_min)
            color = rainbow_color(value, value_min, value_max)
            self.plot_canvas.create_line(
                x,
                y + step,
                x + bar_width,
                y + step,
                fill=color,
            )

        self.plot_canvas.create_rectangle(
            x,
            y,
            x + bar_width,
            y + height,
            outline="#555555",
        )
        self.plot_canvas.create_text(
            x + bar_width + 6,
            y,
            text=f"{value_max:.3g}",
            fill="#555555",
            font=("Segoe UI", 9),
            anchor="w",
        )
        self.plot_canvas.create_text(
            x + bar_width + 6,
            y + height,
            text=f"{value_min:.3g}",
            fill="#555555",
            font=("Segoe UI", 9),
            anchor="w",
        )
        if label:
            short_label = label if len(label) <= 22 else f"...{label[-19:]}"
            self.plot_canvas.create_text(
                x + bar_width / 2,
                y + height + 18,
                text=short_label,
                fill="#111111",
                font=("Segoe UI", 9),
            )

    def _on_canvas_click(self, event):
        if not self.current_plot or self.current_plot.get("mode") != "xy":
            return
        if not self.plot_transform:
            return

        transform = self.plot_transform
        if not (
            transform["left"] <= event.x <= transform["right"]
            and transform["top"] <= event.y <= transform["bottom"]
        ):
            return

        x_array = np.asarray(self.current_plot["x"], dtype=float)
        y_array = np.asarray(self.current_plot["y"], dtype=float)
        finite_mask = np.isfinite(x_array) & np.isfinite(y_array)
        finite_indices = np.flatnonzero(finite_mask)

        if len(finite_indices) == 0:
            return

        x = x_array[finite_indices]
        y = y_array[finite_indices]
        screen_x = transform["left"] + (x - transform["x_min"]) / (transform["x_max"] - transform["x_min"]) * (transform["right"] - transform["left"])
        screen_y = transform["top"] + (transform["y_max"] - y) / (transform["y_max"] - transform["y_min"]) * (transform["bottom"] - transform["top"])
        nearest_local_index = int(np.argmin((screen_x - event.x) ** 2 + (screen_y - event.y) ** 2))
        selected_index = int(finite_indices[nearest_local_index])

        self.current_plot["selected_index"] = selected_index
        self._redraw_current_plot()

    def _draw_xy_point_label(self, index, to_canvas_x, to_canvas_y):
        x_array = self.current_plot["x"]
        y_array = self.current_plot["y"]
        time_array = self.current_plot.get("time")
        color_values = self.current_plot.get("color_values")
        color_label = self.current_plot.get("color_label")

        if index < 0 or index >= len(x_array):
            return

        point_x = float(x_array[index])
        point_y = float(y_array[index])
        if not np.isfinite(point_x) or not np.isfinite(point_y):
            return

        canvas_x = to_canvas_x(point_x)
        canvas_y = to_canvas_y(point_y)
        label = f"index={index}\nx={point_x:.3f} m\ny={point_y:.3f} m"

        if time_array is not None and len(time_array) == len(x_array):
            point_time = float(time_array[index])
            if np.isfinite(point_time):
                label += f"\nt={point_time:.3f} s"

        if color_values is not None and color_label and len(color_values) == len(x_array):
            signal_value = float(color_values[index])
            if np.isfinite(signal_value):
                short_label = color_label if len(color_label) <= 18 else f"...{color_label[-15:]}"
                label += f"\n{short_label}={signal_value:.4g}"

        width = max(self.plot_canvas.winfo_width(), 2)
        height = max(self.plot_canvas.winfo_height(), 2)
        box_width = 190 if color_values is not None else 146
        box_height = 96 if color_values is not None else 76
        text_x = min(max(canvas_x + 80, box_width / 2 + 8), width - box_width / 2 - 8)
        text_y = min(max(canvas_y - 42, box_height / 2 + 8), height - box_height / 2 - 8)

        self.plot_canvas.create_oval(
            canvas_x - 5,
            canvas_y - 5,
            canvas_x + 5,
            canvas_y + 5,
            fill="#f2b705",
            outline="#8a6100",
            width=1,
        )
        self.plot_canvas.create_line(canvas_x, canvas_y, text_x - 52, text_y + 26, fill="#8a6100")
        self.plot_canvas.create_rectangle(
            text_x - box_width / 2,
            text_y - box_height / 2,
            text_x + box_width / 2,
            text_y + box_height / 2,
            fill="#ffffff",
            outline="#8a6100",
        )
        self.plot_canvas.create_text(
            text_x,
            text_y,
            text=label,
            fill="#111111",
            font=("Segoe UI", 9),
            justify="left",
        )


if __name__ == "__main__":
    app = LapDataViewer()
    app.mainloop()
