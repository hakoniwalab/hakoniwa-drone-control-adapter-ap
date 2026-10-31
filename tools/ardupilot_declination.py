"""Read ArduPilot's generated magnetic table and evaluate its bilinear model.

The table remains in the GPL ArduPilot submodule; this module does not copy it.  Keeping the
runtime and Control Link preparation on this one reader prevents their magnetic models drifting.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

TABLES = Path(__file__).resolve().parents[1] / "thirdparty/ardupilot/libraries/AP_Declination/tables.cpp"


def _table(name: str, path: Path = TABLES) -> list[list[float]]:
    text = path.read_text(encoding="utf-8")
    match = re.search(rf"{name}_table\[.*?\]\[.*?\]\s*=\s*\{{(.*?)\n\}};", text, re.S)
    if not match:
        raise ValueError(f"cannot find {name}_table in {path}")
    rows = []
    for row in re.findall(r"\{([^{}]+)\}", match.group(1)):
        rows.append([float(value.rstrip("f")) for value in row.split(",") if value.strip()])
    if len(rows) != 19 or any(len(row) != 37 for row in rows):
        raise ValueError(f"unexpected {name} table dimensions in {path}")
    return rows


def magnetic_field(latitude_deg: float, longitude_deg: float, path: Path = TABLES) -> dict[str, float]:
    """Match AP_Declination's 10-degree grid and bilinear interpolation."""
    min_lat = max(-90.0, min(80.0, math.floor(latitude_deg / 10.0) * 10.0))
    min_lon = max(-180.0, min(170.0, math.floor(longitude_deg / 10.0) * 10.0))
    li, oi = int((90.0 + min_lat) / 10.0), int((180.0 + min_lon) / 10.0)
    fy, fx = (latitude_deg - min_lat) / 10.0, (longitude_deg - min_lon) / 10.0

    def interpolate(table: list[list[float]]) -> float:
        low = table[li][oi] + fx * (table[li][oi + 1] - table[li][oi])
        high = table[li + 1][oi] + fx * (table[li + 1][oi + 1] - table[li + 1][oi])
        return low + fy * (high - low)

    return {
        "declination_deg": interpolate(_table("declination", path)),
        "inclination_deg": interpolate(_table("inclination", path)),
        "intensity_gauss": interpolate(_table("intensity", path)),
    }


# [SEN-MAG-295] How far a configured simulation.location.magneticField may be from the world magnetic model before
# the setup records a warning (the configured value is still the one the simulation uses).
INTENSITY_TOLERANCE = 0.01   # 1 %
ANGLE_TOLERANCE_DEG = 0.5


def magnetic_field_warnings(configured: dict, model: dict[str, float]) -> list[str]:
    """Compare magneticField (intensity_nT, declination_deg, inclination_deg) with the model at the place."""
    warnings = []
    if "intensity_nT" in configured:
        model_nt = model["intensity_gauss"] * 100000.0
        ratio = float(configured["intensity_nT"]) / model_nt - 1.0
        if abs(ratio) > INTENSITY_TOLERANCE:
            warnings.append(f"magneticField.intensity_nT {float(configured['intensity_nT']):g} differs from the "
                            f"world magnetic model {model_nt:g} nT by {100.0 * ratio:+.2f}% (more than 1%)")
    for key in ("declination_deg", "inclination_deg"):
        if key in configured:
            difference = float(configured[key]) - model[key]
            if abs(difference) > ANGLE_TOLERANCE_DEG:
                warnings.append(f"magneticField.{key} {float(configured[key]):g} differs from the world magnetic "
                                f"model {model[key]:g} deg by {difference:+.3f} deg (more than 0.5 deg)")
    return warnings
