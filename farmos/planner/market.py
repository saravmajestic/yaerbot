"""Market prices for the planner.

⚠️  CURRENTLY MOCK DATA. `MockPriceSource` serves the generated series in
data/prices_mock.json — plausible but NOT real. It exists so the planner has a price
signal to reason over during development, and WILL BE REPLACED by `AgmarknetPriceSource`
(real current price from the data.gov.in Agmarknet API + real historical series). The
swap is one line — see `_SOURCE` at the bottom. Prices are ₹/quintal, modal.
"""
from __future__ import annotations

import json
import os
from typing import Protocol

_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

with open(os.path.join(_DATA, "prices_mock.json")) as _f:
    _MOCK = json.load(_f)


class PriceSource(Protocol):
    def current(self, crop: str) -> dict: ...
    def history_monthly(self, crop: str) -> dict: ...  # {"YYYY-MM": price}


class MockPriceSource:
    """⚠️ MOCK. Reads data/prices_mock.json. Replace with AgmarknetPriceSource."""

    mock = True

    def current(self, crop: str) -> dict:
        return dict(_MOCK["crops"][crop.lower()]["current"], unit=_MOCK["unit"], mock=True)

    def history_monthly(self, crop: str) -> dict:
        return _MOCK["crops"][crop.lower()]["history_monthly"]


# ─────────────────────────────────────────────────────────────────────────────
# REAL PATH (stub — not active yet). Enable by setting DATAGOV_API_KEY + market,
# then set `_SOURCE = AgmarknetPriceSource(...)` below.
#
# class AgmarknetPriceSource:
#     """Real modal prices from the data.gov.in Agmarknet resource.
#     Current:  GET https://api.data.gov.in/resource/<mandi-resource-id>
#                 ?api-key=$DATAGOV_API_KEY&format=json
#                 &filters[commodity]=Groundnut&filters[state]=Tamil Nadu&filters[market]=<mandi>
#               -> records with min/max/modal_price + arrival_date.
#     History:  the API is recent-daily only, so either query an arrival_date range for a
#               short trend OR keep a rolling on-device cache that accumulates daily prices.
#     Reliability: fetch live, cache the last good response to JSON, fall back to cache
#               offline/on error (so a recorded demo never breaks on a failed call)."""
#     def __init__(self, api_key, state, market): ...
#     def current(self, crop): ...        # HTTP + cache-fallback
#     def history_monthly(self, crop): ...
# ─────────────────────────────────────────────────────────────────────────────

_SOURCE: PriceSource = MockPriceSource()   # ← swap to AgmarknetPriceSource(...) when live


def get_current_price(crop: str) -> dict:
    return _SOURCE.current(crop)


def get_price_history(crop: str) -> dict:
    return _SOURCE.history_monthly(crop)


def price_summary(crop: str) -> dict:
    """Current price + 3-year context (min/max/avg, YoY, recent trend). MOCK until the API is wired."""
    hist = get_price_history(crop)
    cur = get_current_price(crop)
    months = sorted(hist)
    vals = [hist[m] for m in months]
    cur_month = cur["month"]
    cur_price = cur["modal_inr_per_quintal"]

    # year-on-year: same month last year
    y, m = cur_month.split("-")
    prev_year_key = f"{int(y) - 1}-{m}"
    yoy = None
    if prev_year_key in hist:
        yoy = round((cur_price - hist[prev_year_key]) / hist[prev_year_key] * 100, 1)

    recent6 = vals[-6:]
    trend = "rising" if recent6[-1] > recent6[0] else "falling" if recent6[-1] < recent6[0] else "flat"

    return {
        "crop": crop.lower(),
        "unit": _MOCK["unit"],
        "mock": True,
        "current": {"month": cur_month, "price": cur_price},
        "history_span": [months[0], months[-1]],
        "min": min(vals), "max": max(vals), "avg": int(round(sum(vals) / len(vals))),
        "yoy_change_pct": yoy,
        "recent_trend": trend,
        "recent_12_months": {m: hist[m] for m in months[-12:]},
    }
