"""Live LLM planner presentation — reconciliation computes ALL the data deterministically,
the LLM presents it (recommendation + alternatives) to the farmer.

This 'data-in -> prose-out' path is reliable with small local models: the dates come from
our real calendars; the model only phrases them (it does not pick dates or drive tools).

Requires a local Ollama server + the model pulled.
Run:  python examples/planner_chat_demo.py [model]     # e.g. qwen2.5:1.5b or gemma2:2b
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from farmos.planner import survey, recommend
from farmos.planner.llm import present, OllamaClient

MODEL = sys.argv[1] if len(sys.argv) > 1 else "qwen2.5:1.5b"
QUESTION = "I want to plant groundnut this season — when should I sow it, and what are my options?"
AFTER = "2026-08-10"


def main() -> None:
    # ── deterministic reconciliation (authoritative) ──
    data = survey("groundnut", after=AFTER)
    rec = recommend("groundnut", after=AFTER)
    print("── deterministic reconciliation (ground truth) ──")
    print(f"recommended (both agree): {rec.recommended_date}")
    print(f"panchangam-only alts: {[r['date'] for r in data['panchangam_only']][:5]}")
    print(f"biodynamic-only alts: {[r['date'] for r in data['biodynamic_only']][:5]}")
    print(f"avoid days: {data['avoid_days_kari_naal']}\n")

    # ── LLM presents that data to the farmer ──
    print(f"── {MODEL} presenting to the farmer ──")
    print(present(OllamaClient(MODEL), QUESTION, data))


if __name__ == "__main__":
    main()
