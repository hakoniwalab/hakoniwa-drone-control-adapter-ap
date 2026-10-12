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
