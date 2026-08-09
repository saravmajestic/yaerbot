"""Weather forecast — REAL, live from Open-Meteo (free, no API key), with cache fallback.

Pattern: fetch live, cache the last good response to data/weather_cache.json, and fall back
to that cache if the call fails (so a recorded demo never breaks on a dropped request).

Caveat: Open-Meteo's daily forecast horizon is ~16 days. If the sowing date is further out,
weather_summary() flags `recommended_in_horizon: false` — use climatological normals for the
exact date (a follow-up). The near-term outlook here is still useful for land prep / rain.
"""
from __future__ import annotations

import json
import os
import ssl
import time
import urllib.parse
import urllib.request

# Use certifi's CA bundle if present (macOS python.org builds lack system certs);
# on the robot's Debian Python this falls back to the system trust store.
try:
    import certifi
    _SSL_CTX = ssl.create_default_context(cafile=certifi.where())
except Exception:  # noqa: BLE001
    _SSL_CTX = ssl.create_default_context()

_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
_CACHE = os.path.join(_DATA, "weather_cache.json")

# Preset farm locations (avoids a geocode round-trip); geocode() handles anything else.
LOCATIONS = {
    "salem": {"name": "Salem, Tamil Nadu", "lat": 11.6643, "lon": 78.1460, "tz": "Asia/Kolkata"},
}

_FORECAST_URL = "https://api.open-meteo.com/v1/forecast"
_GEOCODE_URL = "https://geocoding-api.open-meteo.com/v1/search"


def _get_json(url: str, params: dict, timeout: int = 15) -> dict:
    full = url + "?" + urllib.parse.urlencode(params)
    with urllib.request.urlopen(full, timeout=timeout, context=_SSL_CTX) as r:
        return json.load(r)


def geocode(name: str) -> dict:
    data = _get_json(_GEOCODE_URL, {"name": name, "count": 1, "country": "IN", "language": "en"})
    res = (data.get("results") or [None])[0]
    if not res:
        raise ValueError(f"could not geocode '{name}'")
    return {"name": f"{res['name']}, {res.get('admin1', '')}".strip(", "),
            "lat": res["latitude"], "lon": res["longitude"], "tz": res.get("timezone", "auto")}


def resolve_location(location: str) -> dict:
    return LOCATIONS.get(location.lower()) or geocode(location)


def get_forecast(location: str = "salem", days: int = 16) -> dict:
    """Live Open-Meteo daily forecast; on failure, fall back to the cached last-good response."""
    loc = resolve_location(location)
    try:
        data = _get_json(_FORECAST_URL, {
            "latitude": loc["lat"], "longitude": loc["lon"], "timezone": loc["tz"],
            "forecast_days": days,
            "daily": "temperature_2m_max,temperature_2m_min,precipitation_sum,precipitation_probability_max",
        })
        d = data["daily"]
        out = {
            "location": loc["name"], "lat": loc["lat"], "lon": loc["lon"],
            "source": "open-meteo (live)", "fetched": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "days": [{"date": d["time"][i], "tmax": d["temperature_2m_max"][i],
                      "tmin": d["temperature_2m_min"][i], "rain_mm": d["precipitation_sum"][i],
                      "rain_prob": d["precipitation_probability_max"][i]}
                     for i in range(len(d["time"]))],
        }
        with open(_CACHE, "w") as f:            # cache last-good for offline fallback
            json.dump(out, f, indent=2)
        return out
    except Exception as e:                       # noqa: BLE001 — network/parse: use cache
        if os.path.exists(_CACHE):
            with open(_CACHE) as f:
                cached = json.load(f)
            cached["source"] = f"cache-fallback (live failed: {e})"
            return cached
        raise


def weather_summary(location: str = "salem", days: int = 16,
                    recommended_date: str | None = None) -> dict:
    fc = get_forecast(location, days)
    ds = fc["days"]
    dates = [x["date"] for x in ds]
    n = max(len(ds), 1)
    in_horizon = recommended_date in dates if recommended_date else None
    return {
        "location": fc["location"], "source": fc["source"], "horizon_days": len(ds),
        "covers": [dates[0], dates[-1]] if dates else [],
        "avg_tmax_c": round(sum(x["tmax"] for x in ds) / n, 1),
        "avg_tmin_c": round(sum(x["tmin"] for x in ds) / n, 1),
        "total_rain_mm": round(sum(x["rain_mm"] for x in ds), 1),
        "rainy_days": sum(1 for x in ds if (x["rain_mm"] or 0) >= 1.0),
        "recommended_date": recommended_date,
        "recommended_in_horizon": in_horizon,
        "note": (None if in_horizon or recommended_date is None else
                 "Sowing date is beyond the 16-day forecast horizon; this is the near-term "
                 "outlook (useful for land prep). Use climatological normals for the exact date."),
        "days": ds,
    }
