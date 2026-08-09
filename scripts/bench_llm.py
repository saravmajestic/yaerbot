"""Benchmark the on-device planner presentation across models: speed AND answer quality.

Runs the real present() prompt (compact groundnut analysis) through /api/chat with
CPU-friendly options (small num_ctx, 4 threads, capped output) and prints tokens/sec +
the actual answer for each model, so we can judge speed and quality together.

Run on the board:  python3 scripts/bench_llm.py gemma2:2b qwen2.5:1.5b qwen2.5:0.5b
"""
import json
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.planner import survey, price_summary, weather_summary

HOST = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
OPTS = {"num_ctx": 1024, "num_thread": 4, "num_predict": 140, "temperature": 0.3}

# LEAN prompt — prompt-eval dominates on the A53, so keep it tiny.
SHORT_SYS = ("You are a farm sowing advisor. Use ONLY the data; never invent dates. Reply in "
             "2-3 short sentences: the recommended date (both panchangam + biodynamic agree), "
             "that earlier alternatives exist, and one line on price/weather.")


def build_messages() -> list[dict]:
    data = survey("groundnut", after="2026-08-10")
    price = price_summary("groundnut")
    try:
        w = weather_summary("salem", recommended_date="2026-09-03")
    except Exception:
        w = None
    rec = (data["recommended_both_systems"] or [{}])[0]
    d = {
        "crop": "groundnut",
        "recommended_date": rec.get("date"),
        "earlier_alternatives": len(data["panchangam_only"]) + len(data["biodynamic_only"]),
        "price": f"Rs{price['current']['price']}/qtl {price['recent_trend']}",
        "weather": (f"{w['avg_tmax_c']}/{w['avg_tmin_c']}C {w['total_rain_mm']}mm" if w else "n/a"),
    }
    user = f"Data: {json.dumps(d, ensure_ascii=False)}\nAnswer in 2-3 sentences."
    return [{"role": "system", "content": SHORT_SYS}, {"role": "user", "content": user}]


def run(model: str, messages: list[dict]) -> None:
    body = json.dumps({"model": model, "messages": messages, "tools": [],
                       "stream": False, "options": OPTS}).encode()
    req = urllib.request.Request(HOST + "/api/chat", data=body,
                                 headers={"Content-Type": "application/json"})
    t0 = time.time()
    d = json.load(urllib.request.urlopen(req, timeout=300))
    wall = time.time() - t0
    ec, ed = d.get("eval_count", 0), d.get("eval_duration", 1) / 1e9
    pc, pd = d.get("prompt_eval_count", 0), d.get("prompt_eval_duration", 1) / 1e9
    ld = d.get("load_duration", 0) / 1e9
    print(f"\n===== {model} =====")
    print(f"load {ld:.1f}s | prompt {pc}tok/{pd:.1f}s | gen {ec}tok/{ed:.1f}s = "
          f"{ec/ed:.1f} tok/s | WALL {wall:.1f}s")
    print("ANSWER:", d["message"]["content"].strip())


if __name__ == "__main__":
    msgs = build_messages()
    for m in (sys.argv[1:] or ["gemma2:2b"]):
        try:
            run(m, msgs)
        except Exception as e:  # noqa: BLE001
            print(f"\n===== {m} ===== FAILED: {e}")
