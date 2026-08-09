"""Tool registry for the LLM — the functions the model calls to consult BOTH calendars.

The tools return real data from the reconciliation layer (planner.survey gives the full
picture: both-systems recommendations + single-system alternatives + avoid days). The LLM
presents/explains; it must only use dates the tools return.
"""
from __future__ import annotations

import json

from . import almanac, planner
from .crops import get_crop, known_crops


def get_crop_profile(crop: str) -> dict:
    c = get_crop(crop)
    return {"crop": c.name, "nokku": c.nokku, "biodynamic": c.biodynamic,
            "row_gap_m": c.row_gap_m, "seed_gap_m": c.seed_gap_m,
            "seeds_per_spot": c.seeds_per_spot, "sow_months": c.sow_months,
            "commodity": c.commodity, "note": c.note}


def check_day(date: str) -> dict:
    d = almanac.day_detail(date)
    return {"date": d.date, "nakshatra": d.nakshatra, "nakshatra_tamil": d.nakshatra_tamil,
            "nokku": d.nokku, "biodynamic": d.biodynamic, "avoid": d.is_avoid,
            "in_range": d.in_range}


def survey_sowing_window(crop: str, after: str | None = None, horizon_days: int = 40) -> dict:
    return planner.survey(crop, after, horizon_days)


DISPATCH = {
    "get_crop_profile": get_crop_profile,
    "check_day": check_day,
    "survey_sowing_window": survey_sowing_window,
}

TOOL_SPECS = [
    {"type": "function", "function": {
        "name": "survey_sowing_window",
        "description": ("List every sowing day for a crop over a date window, each classified by "
                        "BOTH the Tamil panchangam (Nokku Naal, from the day's nakshatra) and the "
                        "biodynamic calendar (root/leaf/flower/fruit). Returns dates where both "
                        "systems agree (recommended), single-system alternatives "
                        "(panchangam_only, biodynamic_only), and kari-naal avoid days."),
        "parameters": {"type": "object", "properties": {
            "crop": {"type": "string", "description": "crop name, e.g. groundnut"},
            "after": {"type": "string", "description": "start date, YYYY-MM-DD"},
            "horizon_days": {"type": "integer", "description": "how many days to scan (default 40)"},
        }, "required": ["crop"]}}},
    {"type": "function", "function": {
        "name": "get_crop_profile",
        "description": ("Get a crop's required Nokku Naal type, biodynamic day-type, row/seed "
                        "spacing, sowing-season months and market commodity name."),
        "parameters": {"type": "object", "properties": {
            "crop": {"type": "string", "description": "crop name, e.g. groundnut"}},
            "required": ["crop"]}}},
    {"type": "function", "function": {
        "name": "check_day",
        "description": ("For a single date, return its nakshatra, its panchangam Nokku type, its "
                        "biodynamic day-type, and whether it is a kari-naal avoid day."),
        "parameters": {"type": "object", "properties": {
            "date": {"type": "string", "description": "date, YYYY-MM-DD"}},
            "required": ["date"]}}},
]

KNOWN_CROPS = known_crops()


def call_tool(name: str, arguments) -> dict:
    """Dispatch a tool call. Arguments may be a dict or a JSON string. Errors are returned,
    not raised, so the model can recover."""
    if isinstance(arguments, str):
        try:
            arguments = json.loads(arguments or "{}")
        except json.JSONDecodeError:
            return {"error": f"could not parse arguments: {arguments!r}"}
    fn = DISPATCH.get(name)
    if fn is None:
        return {"error": f"unknown tool '{name}'. Available: {list(DISPATCH)}"}
    try:
        return fn(**arguments)
    except KeyError as e:
        return {"error": str(e), "known_crops": KNOWN_CROPS}
    except TypeError as e:
        return {"error": f"bad arguments for {name}: {e}"}
