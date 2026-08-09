"""LLM layer — lets a small local model (Qwen/Gemma via Ollama) drive the calendar tools
and present the recommendation + alternatives to the farmer.

- OllamaClient: dependency-free (stdlib urllib) client for a local Ollama server; works on
  the Mac now and on the UNO Q later (same /api/chat + tools contract).
- StubLLMClient: scripted client so the tool-loop is testable offline with no model.
- converse(): the tool-calling loop — model asks for tools, we run them, feed results back,
  until the model gives a final answer.

Design guardrail (matches planner.py): the tools return the authoritative data; the model
only narrates and must use dates the tools returned — it never invents dates or prices.
"""
from __future__ import annotations

import json
import urllib.request

from .tools import TOOL_SPECS, call_tool

DEFAULT_SYSTEM = (
    "You are a sowing advisor for a Tamil Nadu farmer. When asked when to sow a crop, use the "
    "tools to consult BOTH calendars: the Tamil panchangam (Nokku Naal, from the nakshatra) and "
    "the biodynamic calendar (root/leaf/flower/fruit days). Recommend the earliest date that is "
    "favourable in BOTH systems and is NOT a kari-naal (avoid) day. Then also present the "
    "single-system alternatives (panchangam_only and biodynamic_only) so the farmer can choose, "
    "and mention the avoid days to skip. Only use dates returned by the tools — never invent a "
    "date. Keep the explanation short and clear."
)


class OllamaClient:
    def __init__(self, model: str = "qwen2.5:1.5b",
                 host: str = "http://localhost:11434", timeout: int = 180):
        self.model, self.host, self.timeout = model, host.rstrip("/"), timeout

    def chat(self, messages: list[dict], tools: list[dict], options: dict | None = None) -> dict:
        payload = {"model": self.model, "messages": messages, "tools": tools, "stream": False}
        if options:
            payload["options"] = options   # e.g. {"num_predict": 110} to cap output (slow CPUs)
        body = json.dumps(payload).encode()
        req = urllib.request.Request(self.host + "/api/chat", data=body,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r:
            return json.load(r)["message"]


class StubLLMClient:
    """Deterministic scripted model: calls survey_sowing_window, then answers. For offline tests."""

    def __init__(self, crop: str = "groundnut", after: str = "2026-08-10"):
        self.crop, self.after, self._round = crop, after, 0

    def chat(self, messages: list[dict], tools: list[dict]) -> dict:
        self._round += 1
        if self._round == 1:
            return {"role": "assistant", "content": "",
                    "tool_calls": [{"function": {"name": "survey_sowing_window",
                                                 "arguments": {"crop": self.crop, "after": self.after}}}]}
        tool_msg = next((m for m in reversed(messages) if m.get("role") == "tool"), None)
        data = json.loads(tool_msg["content"]) if tool_msg else {}
        rec = data.get("recommended_both_systems") or []
        pick = rec[0]["date"] if rec else "no both-systems date found"
        return {"role": "assistant",
                "content": f"Recommended sowing date (both systems agree): {pick}."}


PRESENT_SYSTEM = (
    "You are a sowing advisor for a Tamil Nadu farmer. You are GIVEN a compact analysis — do not "
    "compute or invent anything, use only the values given. The UI already shows every date in a "
    "table, so DO NOT list them all. Reply in 2-3 short, friendly sentences: name the recommended "
    "date and why (both the panchangam Nokku Naal and the biodynamic day agree), mention that "
    "earlier alternatives exist if they can't wait, and one short line on price/weather if given. "
    "Never invent a date."
)


def _compact(survey_data: dict, price: dict | None, weather: dict | None) -> dict:
    """Shrink the full survey to just what the model needs — a big prompt is very slow on the
    UNO Q CPU (a full-JSON prompt timed out at >3 min)."""
    rec = survey_data.get("recommended_both_systems") or []
    best = rec[0] if rec else None
    needs = survey_data.get("needs", {})
    c = {
        "crop": survey_data.get("crop"),
        "recommended_date": best["date"] if best else None,
        "why": (f"{best['nakshatra']} nakshatra ({needs.get('nokku')} Nokku) and a biodynamic "
                f"{needs.get('biodynamic')} day, not a kari-naal avoid day") if best else "none in window",
        "earlier_panchangam_only_count": len(survey_data.get("panchangam_only", [])),
        "earlier_biodynamic_only_count": len(survey_data.get("biodynamic_only", [])),
    }
    if price:
        c["price"] = f"{price['current']['price']} {price['unit']} ({price['recent_trend']}, YoY {price['yoy_change_pct']}%) [indicative]"
    if weather:
        c["weather"] = f"near-term {weather['avg_tmax_c']}/{weather['avg_tmin_c']}C, {weather['total_rain_mm']}mm/{weather['rainy_days']} rainy days"
    return c


def present(llm, question: str, survey_data: dict, price: dict | None = None,
            weather: dict | None = None, num_predict: int = 130) -> str:
    """Data-in -> prose-out: a COMPACT summary to the model, capped output (CPU-friendly).

    All dates/numbers come from the passed-in data; the model only phrases them briefly (the UI
    shows the full detail). This keeps small local models fast and grounded."""
    user = (f"Farmer's question: {question}\n\nData (use only these):\n"
            f"{json.dumps(_compact(survey_data, price, weather), ensure_ascii=False)}\n\n"
            f"Answer in 2-3 short sentences.")
    msg = llm.chat([{"role": "system", "content": PRESENT_SYSTEM},
                    {"role": "user", "content": user}],
                   [], options={"num_predict": num_predict, "temperature": 0.3})
    return msg.get("content", "")


def converse(llm, user_message: str, *, system: str = DEFAULT_SYSTEM,
             tools: list[dict] = TOOL_SPECS, max_rounds: int = 6, trace: list | None = None):
    """Run the tool-calling loop. Returns (final_text, full_message_list)."""
    messages = [{"role": "system", "content": system},
                {"role": "user", "content": user_message}]
    for _ in range(max_rounds):
        msg = llm.chat(messages, tools)
        messages.append(msg)
        tool_calls = msg.get("tool_calls") or []
        if not tool_calls:
            return msg.get("content", ""), messages
        for tc in tool_calls:
            fn = tc.get("function", {})
            name = fn.get("name", "")
            result = call_tool(name, fn.get("arguments", {}))
            if trace is not None:
                trace.append({"tool": name, "args": fn.get("arguments", {}), "result": result})
            messages.append({"role": "tool", "name": name, "content": json.dumps(result)})
    return messages[-1].get("content", ""), messages
