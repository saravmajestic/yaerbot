"""Rule-based planner (Act 1) — the authoritative, deterministic recommendation.

It reconciles the two real calendars: it recommends the earliest date on/after the
requested start that is favourable in BOTH the Tamil panchangam (the crop's required
Nokku Naal) AND the biodynamic calendar (the crop's day-type), and is not a கரி நாள்
(avoid) day. The concrete outputs (date, spacing) come from data + rules — never invented.

The LLM layer (llm.py / tools.py) drives these same tools and turns this into a
natural-language conversation, but the numbers a farmer acts on come from here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date

from . import almanac
from .crops import get_crop

_NOKKU_LABEL = {"keezh": "Keezh Nokku (downward — root/tuber sowing)",
                "sama": "Sama Nokku (level)",
                "mel": "Mel Nokku (upward — above-ground)"}


@dataclass
class Recommendation:
    crop: str
    recommended_date: str | None
    both_agree: bool
    nakshatra: str | None
    nakshatra_tamil: str | None
    nokku: str | None
    biodynamic: str | None
    alternatives: list[str]
    row_gap_m: float
    seed_gap_m: float
    seeds_per_spot: int
    rationale: str
    searched_from: str
    searched_days: int

    def to_seed_plan(self, *, plot_w_m: float, plot_l_m: float, speed_mps: float = 0.10):
        """Hand off to Act 2 — build a SeedPlan carrying the recommended date + rationale."""
        from ..config import SeedPlan
        return SeedPlan(
            crop=self.crop,
            plot_w_m=plot_w_m, plot_l_m=plot_l_m,
            row_gap_m=self.row_gap_m, seed_gap_m=self.seed_gap_m,
            seeds_per_spot=self.seeds_per_spot, speed_mps=speed_mps,
            recommended_date=self.recommended_date or "",
            rationale=self.rationale,
        )


def recommend(crop_name: str, after: str | None = None, horizon_days: int = 40) -> Recommendation:
    crop = get_crop(crop_name)
    after = after or date.today().isoformat()

    dual = almanac.find_dual_favorable(crop.nokku, crop.biodynamic, after, horizon_days)
    best = dual[0] if dual else None
    alts = [d.date for d in dual[1:4]]

    if best is not None:
        in_season = int(best.date.split("-")[1]) in crop.sow_months
        rationale = (
            f"{crop.name.title()} is a {_NOKKU_LABEL[crop.nokku]} crop and a biodynamic "
            f"{crop.biodynamic} crop. The earliest date favourable in BOTH systems on/after "
            f"{after} is {best.date}: nakshatra {best.nakshatra} ({best.nakshatra_tamil}) → "
            f"{crop.nokku.title()} Nokku, and a biodynamic {best.biodynamic} day, and it is "
            f"not a கரி நாள் (avoid) day. "
            + ("It also falls within the crop's sowing season. " if in_season
               else "NOTE: this is outside the usual sowing months for this crop. ")
            + (f"Other dual-favourable dates in the window: {', '.join(alts)}." if alts else "")
        )
    else:
        rationale = (
            f"No date on/after {after} within {horizon_days} days is favourable in both the "
            f"panchangam ({crop.nokku} Nokku) and the biodynamic ({crop.biodynamic}) calendars "
            f"while avoiding கரி நாள் days. Widen the window or re-pull the calendars."
        )

    return Recommendation(
        crop=crop.name,
        recommended_date=best.date if best else None,
        both_agree=best is not None,
        nakshatra=best.nakshatra if best else None,
        nakshatra_tamil=best.nakshatra_tamil if best else None,
        nokku=best.nokku if best else None,
        biodynamic=best.biodynamic if best else None,
        alternatives=alts,
        row_gap_m=crop.row_gap_m,
        seed_gap_m=crop.seed_gap_m,
        seeds_per_spot=crop.seeds_per_spot,
        rationale=rationale,
        searched_from=after,
        searched_days=horizon_days,
    )
