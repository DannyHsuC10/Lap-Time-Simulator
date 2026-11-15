"""Choose simpler vehicle models after a lap calculation times out."""

from .settings import MODEL_CHOICES, Settings


def lower_model_choices(settings: Settings) -> dict[str, str | bool]:
    changes = {}
    for key in ("tire_model", "load_model", "motor_model", "brake_model",
                "aero_model", "rolling_model"):
        if key == "aero_model" and not settings.aero_enabled:
            continue
        if key == "rolling_model" and settings.rolling_model == "table" and settings.tire_model != "table":
            # The unavailable table already contributes zero rolling resistance.
            continue
        choices = tuple(MODEL_CHOICES[key][1])
        index = choices.index(getattr(settings, key))
        if index + 1 < len(choices):
            changes[key] = choices[index + 1]

    if settings.curvature_model == "smooth":
        changes["curvature_model"] = "raw"
    if settings.yaw_correction:
        changes["yaw_correction"] = False
    if changes.get("load_model") == "point_mass":
        if settings.tire_model != "ellipse":
            changes["tire_model"] = "ellipse"
        if settings.ellipse_source != "fixed":
            changes["ellipse_source"] = "fixed"
    return changes


def describe_model_changes(settings: Settings, changes: dict[str, str | bool]) -> str:
    lines = []
    for key, new in changes.items():
        if key == "yaw_correction":
            lines.append("偏航慣量修正　啟用 >> 關閉")
        else:
            label, choices = MODEL_CHOICES[key]
            lines.append(f"{label}　{choices[getattr(settings, key)]} >> {choices[new]}")
    return "\n".join(lines)


def timeout_suggestions(settings: Settings) -> str:
    suggestions = []
    if settings.solver == "lap":
        if settings.passes > 1:
            fewer = (settings.passes + 1) // 2
            suggestions.append(f"最大掃描次數從 {settings.passes} 輪降到 {fewer} 輪")
        else:
            suggestions.append("最大掃描次數已是 1 輪")
    suggestions.append(f"取樣間距從 {settings.step_m:g} m 加大到 {2 * settings.step_m:g} m，以減少取樣點")
    return "目前已無可再降低的模型。建議在「賽道」調整：" + "；".join(suggestions) + "。"
