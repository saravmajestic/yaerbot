"""Almanac — reconciles the two real calendars (Tamil panchangam Nokku Naal + the
biodynamic root/leaf/flower/fruit calendar) for a given date.

All data is loaded from the cached JSON in data/ (pulled from drikpanchang and the
Biodynamic Association of India 2026 calendar — see each file's _source).
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import date, timedelta

_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def _load(name: str) -> dict:
    with open(os.path.join(_DATA, name)) as f:
        return json.load(f)


_PANCH = _load("panchangam_cache.json")
_NOKKU = _load("nakshatra_nokku.json")
_BIO = _load("biodynamic_cache.json")

_NAK_BY_DATE = _PANCH["nakshatra_by_date"]
_NOKKU_BY_NAK = _NOKKU["nokku_by_nakshatra"]
_TAMIL = _NOKKU["tamil_names"]
_BIO_MONTHS = _BIO["months"]


@dataclass
class DayDetail:
    date: str                    # YYYY-MM-DD
    nakshatra: str | None
    nakshatra_tamil: str | None
    nokku: str | None            # keezh | sama | mel
    biodynamic: str | None       # root | leaf | flower | fruit
    is_avoid: bool               # கரி நாள்
    in_range: bool               # both caches cover this date


def nakshatra_for(d: str) -> str | None:
    return _NAK_BY_DATE.get(d)


def nokku_for(d: str) -> str | None:
    nak = nakshatra_for(d)
    return _NOKKU_BY_NAK.get(nak) if nak else None


def biodynamic_for(d: str) -> str | None:
    y, m, day = (int(x) for x in d.split("-"))
    month = _BIO_MONTHS.get(str(m))
    if not month:
        return None
    for t in ("root", "leaf", "flower", "fruit"):
        if day in month.get(t, []):
            return t
    return None


def is_avoid(d: str) -> bool:
    y, m, day = (int(x) for x in d.split("-"))
    month = _BIO_MONTHS.get(str(m))
    return bool(month and day in month.get("avoid", []))


def day_detail(d: str) -> DayDetail:
    nak = nakshatra_for(d)
    bio = biodynamic_for(d)
    return DayDetail(
        date=d,
        nakshatra=nak,
        nakshatra_tamil=_TAMIL.get(nak) if nak else None,
        nokku=nokku_for(d),
        biodynamic=bio,
        is_avoid=is_avoid(d),
        in_range=nak is not None and bio is not None,
    )


def find_dual_favorable(nokku_needed: str, biodynamic_needed: str,
                        after: str, horizon_days: int = 40) -> list[DayDetail]:
    """Dates on/after `after` that are favourable in BOTH systems and not a கரி நாள்."""
    start = date.fromisoformat(after)
    out: list[DayDetail] = []
    for i in range(horizon_days + 1):
        d = (start + timedelta(days=i)).isoformat()
        det = day_detail(d)
        if not det.in_range:
            continue
        if det.is_avoid:
            continue
        if det.nokku == nokku_needed and det.biodynamic == biodynamic_needed:
            out.append(det)
    return out


def survey_window(after: str, horizon_days: int = 40) -> list[DayDetail]:
    """Every in-range day on/after `after` within the horizon, with both systems classified."""
    start = date.fromisoformat(after)
    out: list[DayDetail] = []
    for i in range(horizon_days + 1):
        det = day_detail((start + timedelta(days=i)).isoformat())
        if det.in_range:
            out.append(det)
    return out


def coverage() -> tuple[str, str]:
    """First and last date the panchangam cache covers."""
    ks = sorted(_NAK_BY_DATE)
    return (ks[0], ks[-1]) if ks else ("", "")
