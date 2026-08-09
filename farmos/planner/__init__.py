"""farmos.planner — Act 1: reconcile the Tamil panchangam (Nokku Naal) and the
biodynamic calendar to recommend a seeding date + spacing (a SeedPlan for Act 2).

Data (all real, cached in data/): panchangam_cache.json, nakshatra_nokku.json,
biodynamic_cache.json, crops.json.
"""
from .almanac import day_detail, find_dual_favorable, nokku_for, biodynamic_for, is_avoid
from .crops import Crop, get_crop, known_crops
from .market import get_current_price, get_price_history, price_summary
from .weather import weather_summary, get_forecast
from .planner import Recommendation, recommend, survey

__all__ = [
    "recommend", "Recommendation", "survey",
    "get_crop", "Crop", "known_crops",
    "price_summary", "get_current_price", "get_price_history",
    "weather_summary", "get_forecast",
    "day_detail", "find_dual_favorable", "nokku_for", "biodynamic_for", "is_avoid",
]
