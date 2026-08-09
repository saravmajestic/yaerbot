"""Sowing Advisor — a small web app wrapping the Act 1 planner + LLM presentation.

Runnable standalone:  python webchat/server.py   (then open http://localhost:8765)
Designed to later drop into the operator console as a 'Plan' tab.

Needs a local Ollama server for the LLM narration (falls back to a plain summary if the
model call fails, so the UI still works offline). The recommendation/alternatives/avoid
days/price/weather all come from the deterministic planner — the LLM only phrases them.
"""
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.planner import survey, recommend, price_summary, weather_summary, known_crops
from farmos.planner.llm import present, OllamaClient

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.environ.get("PLANNER_MODEL", "gemma2:2b")
DEFAULT_AFTER = os.environ.get("PLANNER_AFTER", "2026-08-10")
LOCATION = os.environ.get("PLANNER_LOCATION", "salem")


def parse_crop(message: str) -> str | None:
    m = (message or "").lower()
    return next((c for c in known_crops() if c in m), None)


def build_plan(crop: str, after: str = DEFAULT_AFTER, model: str = DEFAULT_MODEL) -> dict:
    data = survey(crop, after=after)
    rec = recommend(crop, after=after)
    price = price_summary(crop)
    try:
        weather = weather_summary(LOCATION, recommended_date=rec.recommended_date)
    except Exception:                       # offline: skip weather
        weather = None

    question = f"When should I sow {crop}, and what are my options?"
    try:
        text = present(OllamaClient(model), question, data, price=price, weather=weather)
    except Exception as e:                  # LLM/Ollama down: deterministic fallback text
        text = (f"Recommended sowing date for {crop}: {rec.recommended_date} "
                f"(both the panchangam and biodynamic calendar agree). "
                f"[LLM narration unavailable: {e}]")

    return {
        "crop": crop, "recommended_date": rec.recommended_date, "both_agree": rec.both_agree,
        "nakshatra": rec.nakshatra, "nakshatra_tamil": rec.nakshatra_tamil,
        "needs": data["needs"], "spacing": data["spacing"],
        "alternatives": {"panchangam": [r["date"] for r in data["panchangam_only"]],
                         "biodynamic": [r["date"] for r in data["biodynamic_only"]]},
        "avoid_days": data["avoid_days_kari_naal"],
        "price": price, "weather": weather, "model": model, "text": text,
    }


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        payload = body if isinstance(body, (bytes, bytearray)) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif self.path == "/api/crops":
            self._send(200, {"crops": known_crops()})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/api/plan":
            return self._send(404, {"error": "not found"})
        length = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            return self._send(400, {"error": "bad JSON"})
        crop = (body.get("crop") or parse_crop(body.get("message", "")) or "").lower()
        if crop not in known_crops():
            return self._send(200, {"error": "unknown_crop", "known_crops": known_crops()})
        try:
            self._send(200, build_plan(crop, model=body.get("model", DEFAULT_MODEL)))
        except Exception as e:                # noqa: BLE001
            self._send(500, {"error": str(e)})

    def log_message(self, *_):                # quiet
        pass


def main(port: int = 8765) -> None:
    print(f"Sowing Advisor on http://localhost:{port}  (model={DEFAULT_MODEL}, after={DEFAULT_AFTER})")
    ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 8765)
