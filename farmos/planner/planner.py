"""Rule-based planner (Act 1) — reconciles the two real calendars.

`survey()` returns the FULL picture for a crop over a window — the both-systems-agree
recommendation PLUS single-system alternatives and the கரி நாள் avoid days — so the LLM
layer can present the recommendation and let the farmer choose an alternative.

`recommend()` is the deterministic pick (earliest both-agree, non-avoid day) built on
top of the survey, and `Recommendation.to_seed_plan()` hands off to Act 2. The concrete
dates/spacing come from data + rules — never invented.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import date

from . import almanac
from .crops import get_crop

_NOKKU_LABEL = {"keezh": "Keezh Nokku (downward — root/tuber sowing)",
                "sama": "Sama Nokku (level)",
                "mel": "Mel Nokku (upward — above-ground)"}


def _brief(det: almanac.DayDetail) -> dict:
    return {"date": det.date, "nakshatra": det.nakshatra, "nakshatra_tamil": det.nakshatra_tamil,
            "nokku": det.nokku, "biodynamic": det.biodynamic, "avoid": det.is_avoid}


def survey(crop_name: str, after: str | None = None, horizon_days: int = 40) -> dict:
    """All data for the LLM: both-agree recommendations + single-system alternatives + avoid days."""
    crop = get_crop(crop_name)
    after = after or date.today().isoformat()
    days = almanac.survey_window(after, horizon_days)

    recommended, panchangam_only, biodynamic_only, avoid = [], [], [], []
    for d in days:
        if d.is_avoid:
            avoid.append(d.date)
            continue
        keezh_ok = d.nokku == crop.nokku
        bio_ok = d.biodynamic == crop.biodynamic
        if keezh_ok and bio_ok:
            recommended.append(_brief(d))
        elif keezh_ok:
            panchangam_only.append(_brief(d))
        elif bio_ok:
            biodynamic_only.append(_brief(d))

    return {
        "crop": crop.name,
        "needs": {"nokku": crop.nokku, "biodynamic": crop.biodynamic},
        "window": {"from": after, "days": horizon_days,
                   "covered": [days[0].date, days[-1].date] if days else []},
        "spacing": {"row_gap_m": crop.row_gap_m, "seed_gap_m": crop.seed_gap_m,
                    "seeds_per_spot": crop.seeds_per_spot},
        "sow_season_months": crop.sow_months,
        "recommended_both_systems": recommended,   # panchangam AND biodynamic agree, not avoid
        "panchangam_only": panchangam_only,         # Keezh Nokku, not avoid, but not a biodynamic root day
        "biodynamic_only": biodynamic_only,         # biodynamic root, not avoid, but not Keezh Nokku
        "avoid_days_kari_naal": avoid,
        "_note": "Prefer recommended_both_systems. Offer panchangam_only / biodynamic_only as "
                 "alternatives if the farmer must sow sooner or favours one tradition.",
    }


@dataclass
class Recommendation:
    crop: str
    recommended_date: str | None
    both_agree: bool
    nakshatra: str | None
    nakshatra_tamil: str | None
    nokku: str | None
    biodynamic: str | None
    alternatives_both: list[str]
    alternatives_panchangam_only: list[str]
    alternatives_biodynamic_only: list[str]
    avoid_days: list[str]
    row_gap_m: float
    seed_gap_m: float
    seeds_per_spot: int
    rationale: str
    survey: dict

    def to_seed_plan(self, *, plot_w_m: float, plot_l_m: float, speed_mps: float = 0.10):
        from ..config import SeedPlan
        return SeedPlan(
            crop=self.crop, plot_w_m=plot_w_m, plot_l_m=plot_l_m,
            row_gap_m=self.row_gap_m, seed_gap_m=self.seed_gap_m,
            seeds_per_spot=self.seeds_per_spot, speed_mps=speed_mps,
            recommended_date=self.recommended_date or "", rationale=self.rationale,
        )


def recommend(crop_name: str, after: str | None = None, horizon_days: int = 40) -> Recommendation:
    crop = get_crop(crop_name)
    after = after or date.today().isoformat()
    s = survey(crop_name, after, horizon_days)

    rec = s["recommended_both_systems"]
    best = rec[0] if rec else None
    if best is not None:
        in_season = int(best["date"].split("-")[1]) in crop.sow_months
        rationale = (
            f"{crop.name.title()} is a {_NOKKU_LABEL[crop.nokku]} crop and a biodynamic "
            f"{crop.biodynamic} crop. Earliest date favourable in BOTH systems on/after "
            f"{after} is {best['date']}: nakshatra {best['nakshatra']} ({best['nakshatra_tamil']}) "
            f"→ {crop.nokku.title()} Nokku, and a biodynamic {best['biodynamic']} day, and not a "
            f"கரி நாள் (avoid) day. "
            + ("Within the sowing season. " if in_season else "NOTE: outside usual sowing months. ")
        )
    else:
        rationale = (f"No both-systems date on/after {after} within {horizon_days} days. "
                     f"See panchangam_only / biodynamic_only alternatives.")

    return Recommendation(
        crop=crop.name,
        recommended_date=best["date"] if best else None,
        both_agree=best is not None,
        nakshatra=best["nakshatra"] if best else None,
        nakshatra_tamil=best["nakshatra_tamil"] if best else None,
        nokku=best["nokku"] if best else None,
        biodynamic=best["biodynamic"] if best else None,
        alternatives_both=[r["date"] for r in rec[1:4]],
        alternatives_panchangam_only=[r["date"] for r in s["panchangam_only"][:4]],
        alternatives_biodynamic_only=[r["date"] for r in s["biodynamic_only"][:4]],
        avoid_days=s["avoid_days_kari_naal"],
        row_gap_m=crop.row_gap_m, seed_gap_m=crop.seed_gap_m, seeds_per_spot=crop.seeds_per_spot,
        rationale=rationale, survey=s,
    )
