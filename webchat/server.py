"""Sowing Advisor — web app wrapping the Act 1 planner + LLM presentation.

Runnable standalone:  python webchat/server.py   (open http://localhost:8765)
Set PLANNER_HOST=0.0.0.0 to expose on the LAN. Designed to drop into the operator console.

Endpoints:
  GET  /                -> the chat UI
  GET  /api/crops       -> known crops
  POST /api/plan        -> deterministic card data ONLY (fast, <1s, no LLM)
  POST /api/narrate     -> streams the LLM prose token-by-token (text/plain chunks)

Split so the UI renders the recommendation card instantly and streams the friendly prose
after — important because the on-device LLM is slow (~30-90s) on this board.
"""
import json
import os
import sys
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.planner import survey, recommend, price_summary, weather_summary, known_crops
from farmos.planner.llm import present_messages, PRESENT_OPTIONS

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.environ.get("PLANNER_MODEL", "qwen2.5:1.5b")
DEFAULT_AFTER = os.environ.get("PLANNER_AFTER", "2026-08-10")
LOCATION = os.environ.get("PLANNER_LOCATION", "salem")
OLLAMA = os.environ.get("OLLAMA_HOST", "http://localhost:11434").rstrip("/")


def parse_crop(message: str) -> str | None:
    m = (message or "").lower()
    return next((c for c in known_crops() if c in m), None)


def _context(crop: str):
    data = survey(crop, after=DEFAULT_AFTER)
    price = price_summary(crop)
    try:
        weather = weather_summary(LOCATION, recommended_date=recommend(crop, after=DEFAULT_AFTER).recommended_date)
    except Exception:
        weather = None
    return data, price, weather


def build_data(crop: str) -> dict:
    """Deterministic card data — no LLM, fast."""
    data, price, weather = _context(crop)
    rec = recommend(crop, after=DEFAULT_AFTER)
    return {
        "crop": crop, "recommended_date": rec.recommended_date, "both_agree": rec.both_agree,
        "nakshatra": rec.nakshatra, "nakshatra_tamil": rec.nakshatra_tamil,
        "needs": data["needs"], "spacing": data["spacing"],
        "alternatives": {"panchangam": [r["date"] for r in data["panchangam_only"]],
                         "biodynamic": [r["date"] for r in data["biodynamic_only"]]},
        "avoid_days": data["avoid_days_kari_naal"], "price": price, "weather": weather,
    }


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json"):
        payload = body if isinstance(body, (bytes, bytearray)) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def _body(self) -> dict:
        n = int(self.headers.get("Content-Length", 0))
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return {}

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif self.path == "/api/crops":
            self._send(200, {"crops": known_crops()})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path == "/api/plan":
            return self._plan()
        if self.path == "/api/narrate":
            return self._narrate()
        self._send(404, {"error": "not found"})

    def _crop_from(self, body):
        crop = (body.get("crop") or parse_crop(body.get("message", "")) or "").lower()
        return crop if crop in known_crops() else None

    def _plan(self):
        crop = self._crop_from(self._body())
        if not crop:
            return self._send(200, {"error": "unknown_crop", "known_crops": known_crops()})
        try:
            self._send(200, build_data(crop))
        except Exception as e:                       # noqa: BLE001
            self._send(500, {"error": str(e)})

    def _narrate(self):
        body = self._body()
        crop = self._crop_from(body)
        if not crop:
            return self._send(400, {"error": "unknown_crop"})
        model = body.get("model", DEFAULT_MODEL)
        try:
            data, price, weather = _context(crop)
            messages = present_messages(data, price, weather)
            req = urllib.request.Request(
                OLLAMA + "/api/chat",
                data=json.dumps({"model": model, "messages": messages, "stream": True,
                                 "options": {**PRESENT_OPTIONS, "num_predict": 150}}).encode(),
                headers={"Content-Type": "application/json"})
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            with urllib.request.urlopen(req, timeout=300) as resp:
                for line in resp:                    # Ollama streams NDJSON, one obj per line
                    line = line.strip()
                    if not line:
                        continue
                    obj = json.loads(line)
                    tok = obj.get("message", {}).get("content", "")
                    if tok:
                        self.wfile.write(tok.encode("utf-8"))
                        self.wfile.flush()
                    if obj.get("done"):
                        break
        except Exception as e:                       # noqa: BLE001
            try:
                self.wfile.write(f"\n[narration unavailable: {e}]".encode())
            except Exception:
                pass

    def log_message(self, *_):
        pass


def main(port: int = 8765) -> None:
    host = os.environ.get("PLANNER_HOST", "127.0.0.1")   # 0.0.0.0 to expose on the LAN
    print(f"Sowing Advisor on http://{host}:{port}  (model={DEFAULT_MODEL})")
    ThreadingHTTPServer((host, port), Handler).serve_forever()


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 8765)
