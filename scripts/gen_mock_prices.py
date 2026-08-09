"""Generate MOCK mandi-price history -> farmos/planner/data/prices_mock.json.

⚠️  MOCK DATA — NOT REAL PRICES. This exists only so the planner has a plausible
price series to reason over during development. It will be REPLACED by real data from
the data.gov.in Agmarknet API (current daily modal price) + real historical series.
See farmos/planner/market.py (AgmarknetPriceSource stub) for the real path.

Deterministic (seeded): base price × yearly trend × monthly seasonality × small noise.
Monthly modal price in ₹/quintal, Jan 2023 → Aug 2026, for groundnut / corn / sesame.
"""
import json
import math
import os
import random

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "farmos", "planner", "data", "prices_mock.json")

# Plausible Tamil-Nadu-ish modal prices (₹/quintal) and harvest-driven seasonality.
CROPS = {
    #             base   yearly_trend  harvest_month (seasonal low ~here)
    "groundnut": (6500,  0.06,         11),   # kharif harvest ~Oct-Nov -> lower then
    "corn":      (2100,  0.05,         10),
    "sesame":    (11000, 0.07,         9),
}
START = (2023, 1)
END = (2026, 8)


def _months(start, end):
    y, m = start
    while (y, m) <= end:
        yield y, m
        m += 1
        if m > 12:
            y, m = y + 1, 1


def gen():
    out = {
        "_mock": True,
        "_warning": "MOCK PRICES — NOT REAL. Replace with data.gov.in Agmarknet API + real history.",
        "unit": "INR/quintal (modal)",
        "generated_by": "scripts/gen_mock_prices.py (deterministic, seed=42)",
        "crops": {},
    }
    rng = random.Random(42)
    for crop, (base, trend, harvest_m) in CROPS.items():
        hist = {}
        for y, m in _months(START, END):
            years_elapsed = (y - START[0]) + (m - 1) / 12.0
            trend_factor = (1 + trend) ** years_elapsed
            # seasonal: lowest at harvest month, ~±8%
            season = 1 + 0.08 * math.cos(2 * math.pi * ((m - harvest_m) / 12.0))
            noise = 1 + rng.uniform(-0.03, 0.03)
            price = base * trend_factor * season * noise
            hist[f"{y:04d}-{m:02d}"] = int(round(price / 10.0) * 10)  # round to ₹10
        keys = sorted(hist)
        out["crops"][crop] = {
            "commodity": crop.title(),
            "history_monthly": hist,
            "current": {"month": keys[-1], "modal_inr_per_quintal": hist[keys[-1]]},
        }
    with open(OUT, "w") as f:
        json.dump(out, f, indent=2)
    print(f"wrote {OUT}: {len(out['crops'])} crops, "
          f"{len(next(iter(out['crops'].values()))['history_monthly'])} months each")


if __name__ == "__main__":
    gen()
