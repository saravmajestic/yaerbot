"""farmos.planner — Act 1: reconcile the Tamil panchangam (Nokku Naal) and the
biodynamic calendar to recommend a seeding date + spacing (a SeedPlan for Act 2).

Data (all real, cached in data/): panchangam_cache.json, nakshatra_nokku.json,
biodynamic_cache.json, crops.json.
"""
from .almanac import day_detail, find_dual_favorable, nokku_for, biodynamic_for, is_avoid
from .crops import Crop, get_crop, known_crops
from .planner import Recommendation, recommend

__all__ = [
    "recommend", "Recommendation",
    "get_crop", "Crop", "known_crops",
    "day_detail", "find_dual_favorable", "nokku_for", "biodynamic_for", "is_avoid",
]
