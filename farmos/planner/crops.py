"""Crop profiles — agronomic constants + which calendar day-types each crop needs."""
from __future__ import annotations

import json
import os
from dataclasses import dataclass

_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

with open(os.path.join(_DATA, "crops.json")) as _f:
    _CROPS = json.load(_f)


@dataclass
class Crop:
    name: str
    nokku: str                 # required Nokku Naal type: keezh | sama | mel
    biodynamic: str            # required biodynamic day: root | leaf | flower | fruit
    row_gap_m: float
    seed_gap_m: float
    seeds_per_spot: int
    sow_months: list[int]
    soil_temp_c: list[int]
    commodity: str
    note: str = ""


def get_crop(name: str) -> Crop:
    key = name.strip().lower()
    if key not in _CROPS:
        raise KeyError(f"unknown crop '{name}'. Known: {', '.join(sorted(_CROPS))}")
    return Crop(name=key, **_CROPS[key])


def known_crops() -> list[str]:
    return sorted(_CROPS)
