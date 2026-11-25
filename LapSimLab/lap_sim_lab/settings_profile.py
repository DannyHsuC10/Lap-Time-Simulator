"""Read and write the workbench's complete, versioned input snapshot."""

from dataclasses import fields
import json
import math
from pathlib import Path
from tempfile import NamedTemporaryFile

from .settings import INPUT_CHOICES, Settings


SCHEMA_VERSION = 1
RETIRED_SETTINGS = {"aero_cl0", "aero_cd0", "rolling_model", "closed_lap"}


def save_profile(path: Path, snapshot: dict):
    defaults = Settings()
    folder = path.absolute().parent
    values = {}
    for key, value in snapshot.items():
        if key in RETIRED_SETTINGS:
            continue
        default = getattr(defaults, key)
        if isinstance(value, Path):
            value = value.absolute()
            value = str(value.relative_to(folder)) if value.is_relative_to(folder) else str(value)
        elif isinstance(value, str) and type(default) in (int, float):
            value = _number_or_draft(value, default)
        values[key] = value

    write_settings_json(path, {"schema_version": SCHEMA_VERSION, "settings": values})


def write_settings_json(path: Path, values: dict):
    payload = json.dumps(values, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    folder = path.absolute().parent
    temporary = None
    try:
        with NamedTemporaryFile(mode="w", encoding="utf-8", dir=folder, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _number_or_draft(value: str, default: int | float):
    try:
        number = int(value) if type(default) is int else float(value)
    except ValueError:
        return value
    return value if isinstance(number, float) and not math.isfinite(number) else number


def load_profile(path: Path) -> dict:
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except ValueError as error:
        raise ValueError("設定檔須為有效的 JSON。") from error
    if not isinstance(data, dict) or not isinstance(data.get("settings"), dict):
        raise ValueError("請選擇 LapSimLab 儲存的設定 JSON。")
    version = data.get("schema_version")
    if type(version) is not int or version != SCHEMA_VERSION:
        raise ValueError("不支援此設定檔版本。")

    defaults = Settings()
    names = {field.name for field in fields(defaults)}
    folder = path.absolute().parent
    values = {}
    for key, value in data["settings"].items():
        if key in RETIRED_SETTINGS:
            continue
        if key not in names:
            raise ValueError(f"無法辨識設定欄位「{key}」。")
        default = getattr(defaults, key)
        if isinstance(default, Path):
            if not isinstance(value, str) or not value.strip() or "\0" in value:
                raise ValueError(f"「{key}」須為資料檔路徑。")
            value = folder / Path(value)
        elif key in INPUT_CHOICES:
            label, choices = INPUT_CHOICES[key]
            if not isinstance(value, str) or value not in choices:
                raise ValueError(f"無法辨識「{label}」的選項。")
        elif isinstance(default, bool):
            if type(value) is not bool:
                raise ValueError(f"「{key}」須為 true 或 false。")
        elif type(value) not in (int, float, str):
            raise ValueError(f"「{key}」須為數值或輸入文字。")
        values[key] = value
    return values
