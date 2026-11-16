"""Section-aware MF6.1 TIR I/O; preserves template comments and vendor data.

``[UNITS] MASS`` is a unit symbol, whereas ``[INERTIA] MASS`` is a physical
mass. No unit conversion or estimation of missing tire properties is done.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numbers
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Mapping


class TirError(ValueError):
    """Invalid or ambiguous TIR content/update."""


SI_UNITS = dict(LENGTH="meter", FORCE="newton", ANGLE="radians", MASS="kg", TIME="second")
_META = {"MDI_HEADER", "UNITS"}
_SECTION = re.compile(r"^\s*\[([A-Za-z0-9_]+)\]\s*(?:\$.*)?$")
_ASSIGN = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*)$")
_FIELDS = {
    "MODEL": "FITTYP TYRESIDE LONGVL VXLOW ROAD_INCREMENT ROAD_DIRECTION USE_MODE",
    "DIMENSION": "UNLOADED_RADIUS WIDTH RIM_RADIUS RIM_WIDTH ASPECT_RATIO",
    "OPERATING_CONDITIONS": "INFLPRES NOMPRES",
    "INERTIA": "MASS IXX IYY BELT_MASS BELT_IXX BELT_IYY GRAVITY",
    "VERTICAL": (
        "FNOMIN VERTICAL_STIFFNESS VERTICAL_DAMPING MC_CONTOUR_A MC_CONTOUR_B "
        "BREFF DREFF FREFF Q_RE0 Q_V1 Q_V2 Q_FZ2 Q_FCX Q_FCY Q_FCY2 Q_CAM "
        "Q_CAM1 Q_CAM2 Q_CAM3 Q_FYS1 Q_FYS2 Q_FYS3 PFZ1 BOTTOM_OFFST BOTTOM_STIFF"
    ),
    "STRUCTURAL": (
        "LONGITUDINAL_STIFFNESS LATERAL_STIFFNESS YAW_STIFFNESS FREQ_LONG "
        "FREQ_LAT FREQ_YAW FREQ_WINDUP DAMP_LONG DAMP_LAT DAMP_YAW DAMP_WINDUP "
        "DAMP_RESIDUAL DAMP_VLOW Q_BVX Q_BVT PCFX1 PCFX2 PCFX3 PCFY1 PCFY2 PCFY3 PCMZ1"
    ),
    "CONTACT_PATCH": (
        "Q_RA1 Q_RA2 Q_RB1 Q_RB2 ELLIPS_SHIFT ELLIPS_LENGTH ELLIPS_HEIGHT "
        "ELLIPS_ORDER ELLIPS_MAX_STEP ELLIPS_NWIDTH ELLIPS_NLENGTH ENV_C1 ENV_C2"
    ),
    "INFLATION_PRESSURE_RANGE": "PRESMIN PRESMAX",
    "VERTICAL_FORCE_RANGE": "FZMIN FZMAX",
    "LONG_SLIP_RANGE": "KPUMIN KPUMAX",
    "SLIP_ANGLE_RANGE": "ALPMIN ALPMAX",
    "INCLINATION_ANGLE_RANGE": "CAMMIN CAMMAX",
    "SCALING_COEFFICIENTS": (
        "LFZO LCX LMUX LEX LKX LHX LVX LCY LMUY LEY LKY LKYC LKZC LHY LVY "
        "LTR LRES LXAL LYKA LVYKA LS LMX LVMX LMY LMP"
    ),
    "LONGITUDINAL_COEFFICIENTS": (
        "PCX1 PDX1 PDX2 PDX3 PEX1 PEX2 PEX3 PEX4 PKX1 PKX2 PKX3 PHX1 PHX2 "
        "PVX1 PVX2 PPX1 PPX2 PPX3 PPX4 RBX1 RBX2 RBX3 RCX1 REX1 REX2 RHX1"
    ),
    "LATERAL_COEFFICIENTS": (
        "PCY1 PDY1 PDY2 PDY3 PEY1 PEY2 PEY3 PEY4 PEY5 PKY1 PKY2 PKY3 PKY4 PKY5 "
        "PKY6 PKY7 PHY1 PHY2 PVY1 PVY2 PVY3 PVY4 PPY1 PPY2 PPY3 PPY4 PPY5 "
        "RBY1 RBY2 RBY3 RBY4 RCY1 REY1 REY2 RHY1 RHY2 RVY1 RVY2 RVY3 RVY4 RVY5 RVY6"
    ),
    "OVERTURNING_COEFFICIENTS": "QSX1 QSX2 QSX3 QSX4 QSX5 QSX6 QSX7 QSX8 QSX9 QSX10 QSX11 QSX12 QSX13 QSX14 PPMX1",
    "ROLLING_COEFFICIENTS": "QSY1 QSY2 QSY3 QSY4 QSY5 QSY6 QSY7 QSY8",
    "ALIGNING_COEFFICIENTS": (
        "QBZ1 QBZ2 QBZ3 QBZ4 QBZ5 QBZ9 QBZ10 QCZ1 QDZ1 QDZ2 QDZ3 QDZ4 QDZ6 "
        "QDZ7 QDZ8 QDZ9 QDZ10 QDZ11 QEZ1 QEZ2 QEZ3 QEZ4 QEZ5 QHZ1 QHZ2 QHZ3 "
        "QHZ4 PPZ1 PPZ2 SSZ1 SSZ2 SSZ3 SSZ4"
    ),
    "TURNSLIP_COEFFICIENTS": (
        "PDXP1 PDXP2 PDXP3 PKYP1 PDYP1 PDYP2 PDYP3 PDYP4 PHYP1 PHYP2 "
        "PHYP3 PHYP4 PECP1 PECP2 QDTP1 QCRP1 QCRP2 QBRP1 QDRP1"
    ),
}
_FIELD_SECTION = {key: section for section, names in _FIELDS.items() for key in names.split()}
_COEFFICIENT_SECTIONS = {
    "SCALING_COEFFICIENTS", "LONGITUDINAL_COEFFICIENTS", "LATERAL_COEFFICIENTS",
    "OVERTURNING_COEFFICIENTS", "ROLLING_COEFFICIENTS", "ALIGNING_COEFFICIENTS", "TURNSLIP_COEFFICIENTS",
}


def _comment_parts(text: str) -> tuple[str, str]:
    quoted, i = False, 0
    while i < len(text):
        if text[i] == "'":
            if quoted and i + 1 < len(text) and text[i + 1] == "'":
                i += 2
                continue
            quoted = not quoted
        elif text[i] == "$" and not quoted:
            return text[:i], text[i:]
        i += 1
    return text, ""


def _parse_value(raw: str) -> Any:
    raw = raw.strip()
    if not raw:
        return None
    if raw.startswith("'"):
        if not raw.endswith("'") or len(raw) < 2:
            raise TirError(f"Unterminated quoted value: {raw!r}")
        return raw[1:-1].replace("''", "'")
    try:
        return float(raw.replace("D", "E").replace("d", "e"))
    except ValueError:
        return raw


@dataclass
class TirDocument:
    lines: list[str]
    sections: dict[str, dict[str, Any]]
    locations: dict[tuple[str, str], int]
    headers: dict[str, int]

    @classmethod
    def from_text(cls, text: str) -> "TirDocument":
        lines, sections, locations, headers = text.splitlines(keepends=True), {}, {}, {}
        section = None
        for index, line in enumerate(lines):
            stripped = line.rstrip("\r\n")
            header = _SECTION.match(stripped)
            if header:
                section = header.group(1).upper()
                if section in sections:
                    raise TirError(f"Duplicate section [{section}] at line {index + 1}")
                sections[section], headers[section] = {}, index
                continue
            content, _ = _comment_parts(stripped)
            match = _ASSIGN.match(content)
            if not match:
                continue
            if section is None:
                raise TirError(f"Parameter outside a section at line {index + 1}")
            key = match.group(1).upper()
            if key in sections[section]:
                raise TirError(f"Duplicate [{section}] {key} at line {index + 1}")
            sections[section][key] = _parse_value(match.group(2))
            locations[(section, key)] = index
        return cls(lines, sections, locations, headers)

    def model_parameters(self) -> dict[str, Any]:
        """Flatten model fields without conflating metadata and physical values."""
        params = {}
        for section, values in self.sections.items():
            if section in _META:
                continue
            for key, value in values.items():
                if key in params:
                    raise TirError(f"Ambiguous model parameter {key}; use section-qualified values")
                if value is not None:
                    params[key] = value
        return params


def read_tir(path: str | Path) -> TirDocument:
    with Path(path).open("r", encoding="utf-8-sig", newline="") as handle:
        return TirDocument.from_text(handle.read())


def validate_tir(source: str | Path | TirDocument) -> list[str]:
    """Raise on malformed MF6.1/SI data; return optional-property warnings.

    This checks the file contract, not CarSim's DLL version, selected dynamics
    mode or whether coefficients describe a measured tire.
    """
    doc = source if isinstance(source, TirDocument) else read_tir(source)
    sections, header = doc.sections, doc.sections.get("MDI_HEADER", {})
    if str(header.get("FILE_TYPE", "")).lower() != "tir":
        raise TirError("[MDI_HEADER] FILE_TYPE must be 'tir'")
    if header.get("FILE_VERSION") != 3:
        raise TirError("[MDI_HEADER] FILE_VERSION must be 3 (3.0 is equivalent)")
    if str(header.get("FILE_FORMAT", "")).upper() != "ASCII":
        raise TirError("[MDI_HEADER] FILE_FORMAT must be 'ASCII'")
    for key, expected in SI_UNITS.items():
        value = sections.get("UNITS", {}).get(key)
        if not isinstance(value, str) or value.lower() != expected:
            raise TirError(f"[UNITS] {key} must be '{expected}', got {value!r}; no implicit unit conversion")
    if sections.get("MODEL", {}).get("FITTYP") != 61:
        raise TirError("[MODEL] FITTYP must be 61; changing a version tag does not convert coefficients")
    for section, values in sections.items():
        for key, value in values.items():
            if isinstance(value, numbers.Real) and not math.isfinite(value):
                raise TirError(f"[{section}] {key} must be finite")
            if section in _COEFFICIENT_SECTIONS and value is not None and not isinstance(value, numbers.Real):
                raise TirError(f"[{section}] {key} must be numeric")
    for section, keys in {
        "DIMENSION": ("UNLOADED_RADIUS", "WIDTH", "RIM_RADIUS"), "VERTICAL": ("FNOMIN",),
    }.items():
        for key in keys:
            value = sections.get(section, {}).get(key)
            if not isinstance(value, numbers.Real) or value <= 0:
                raise TirError(f"[{section}] {key} must be a finite positive number")
    for key in (
        "INFLPRES", "NOMPRES", "LONGVL", "VXLOW", "RIM_WIDTH", "MASS", "IXX", "IYY",
        "BELT_MASS", "BELT_IXX", "BELT_IYY", "VERTICAL_STIFFNESS",
    ):
        section = _FIELD_SECTION[key]
        value = sections.get(section, {}).get(key)
        if value is not None and (not isinstance(value, numbers.Real) or value <= 0):
            raise TirError(f"[{section}] {key} must be positive when supplied")
    lfzo = sections.get("SCALING_COEFFICIENTS", {}).get("LFZO")
    if lfzo is not None and lfzo <= 0:
        raise TirError("[SCALING_COEFFICIENTS] LFZO must be positive")
    for section, lo, hi in (
        ("INFLATION_PRESSURE_RANGE", "PRESMIN", "PRESMAX"), ("VERTICAL_FORCE_RANGE", "FZMIN", "FZMAX"),
        ("LONG_SLIP_RANGE", "KPUMIN", "KPUMAX"), ("SLIP_ANGLE_RANGE", "ALPMIN", "ALPMAX"),
        ("INCLINATION_ANGLE_RANGE", "CAMMIN", "CAMMAX"),
    ):
        lower, upper = (sections.get(section, {}).get(key) for key in (lo, hi))
        if lower is not None and upper is not None:
            if not isinstance(lower, numbers.Real) or not isinstance(upper, numbers.Real) or lower >= upper:
                raise TirError(f"[{section}] requires numeric {lo} < {hi}")
    warnings = []
    for section, key in (
        ("OPERATING_CONDITIONS", "INFLPRES"), ("OPERATING_CONDITIONS", "NOMPRES"),
        ("VERTICAL", "VERTICAL_STIFFNESS"), ("VERTICAL", "VERTICAL_DAMPING"),
    ):
        if sections.get(section, {}).get(key) is None:
            warnings.append(f"[{section}] {key} is absent; external MF-Tyre may estimate it. Supply measured data for dynamics validation.")
    for section in ("STRUCTURAL", "CONTACT_PATCH"):
        if not any(value is not None for value in sections.get(section, {}).values()):
            warnings.append(f"[{section}] has no data; verify the selected CarSim dynamics/contact mode.")
    return warnings


def load_tir(path: str | Path, *, validate: bool = True) -> dict[str, Any]:
    doc = read_tir(path)
    if validate:
        validate_tir(doc)
    return doc.model_parameters()


def _infer_section(key: str) -> str | None:
    # Explicit schema catches typos such as PDY33 instead of inventing a new
    # coefficient. Vendor extensions can still use a section-qualified key.
    return _FIELD_SECTION.get(key)


def _format_value(value: Any) -> str:
    if value is None or isinstance(value, bool):
        raise TirError("TIR updates require a nonempty string or finite number")
    if isinstance(value, numbers.Real):
        if not math.isfinite(value):
            raise TirError("TIR updates cannot contain NaN or Infinity")
        return format(float(value), ".17g")
    if isinstance(value, str) and value.strip():
        if "\n" in value or "\r" in value:
            raise TirError("TIR strings cannot contain newlines")
        return "'" + value.replace("'", "''") + "'"
    raise TirError(f"Unsupported TIR value {value!r}")


def write_tir(template_path: str | Path, output_path: str | Path,
              params: Mapping[str | tuple[str, str], Any], *, validate: bool = True) -> Path:
    """Atomically update a template with 17-significant-digit numeric precision.

    Flat keys address model fields only (MASS means INERTIA/MASS). Explicit
    (section, key) pairs may update metadata or add vendor fields. Untouched
    lines and comments are preserved. Serialization is checked before saving.
    """
    doc, updates = read_tir(template_path), {}
    for raw_key, value in params.items():
        if isinstance(raw_key, tuple) and len(raw_key) == 2:
            section, key = (str(part).upper() for part in raw_key)
        elif isinstance(raw_key, str):
            key = raw_key.upper()
            matches = [section for section in doc.sections if section not in _META and key in doc.sections[section]]
            if len(matches) > 1:
                raise TirError(f"Ambiguous update {key}; specify (section, key)")
            section = matches[0] if matches else _infer_section(key)
            if section is None:
                raise TirError(f"Unknown model update {key}; specify its (section, key)")
        else:
            raise TirError(f"Invalid parameter key {raw_key!r}")
        if any(not re.fullmatch(r"[A-Z_][A-Z0-9_]*", part) for part in (section, key)):
            raise TirError(f"Invalid section/parameter name {(section, key)!r}")
        _format_value(value)
        if (section, key) in updates:
            raise TirError(f"Duplicate update for [{section}] {key}")
        updates[(section, key)] = value

    newline = "\r\n" if any(line.endswith("\r\n") for line in doc.lines) else "\n"
    lines, additions = list(doc.lines), {}
    for (section, key), value in updates.items():
        formatted, index = _format_value(value), doc.locations.get((section, key))
        if index is None:
            additions.setdefault(section, []).append(f"{key:<29}= {formatted}{newline}")
        else:
            original = lines[index].rstrip("\r\n")
            _, comment = _comment_parts(original)
            lines[index] = original.split("=", 1)[0] + "= " + formatted + (("  " + comment) if comment else "") + newline
    insertions, headers = {}, sorted(doc.headers.values())
    for section, extra in additions.items():
        if section in doc.headers:
            end = next((i for i in headers if i > doc.headers[section]), len(lines))
            insertions.setdefault(end, []).extend(extra)
        else:
            insertions.setdefault(len(lines), []).extend([f"{newline}[{section}]{newline}", *extra])
    result = []
    for index in range(len(lines) + 1):
        if index in insertions:
            if result and not result[-1].endswith(("\n", "\r")):
                result[-1] += newline
            result.extend(insertions[index])
        if index < len(lines):
            result.append(lines[index])
    text = "".join(result)
    roundtrip = TirDocument.from_text(text)
    if validate:
        validate_tir(roundtrip)
    for (section, key), value in updates.items():
        actual = roundtrip.sections[section][key]
        if actual != value:
            raise TirError(f"Value changed during serialization: [{section}] {key}: {value!r} -> {actual!r}")
    output, temp_path = Path(output_path), None
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="", dir=output.parent, suffix=".tmp", delete=False) as handle:
            temp_path = Path(handle.name)
            handle.write(text)
        os.replace(temp_path, output)
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()
    return output
