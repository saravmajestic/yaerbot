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

    def chat(self, messages: list[dict], tools: list[dict]) -> dict:
        body = json.dumps({"model": self.model, "messages": messages,
                           "tools": tools, "stream": False}).encode()
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
    "You are a sowing advisor for a Tamil Nadu farmer. You are GIVEN a completed analysis of "
    "sowing days (JSON) — do not compute or invent anything. Present it to the farmer in simple "
    "language: (1) the RECOMMENDED date(s) where BOTH the panchangam (Nokku Naal) and the "
    "biodynamic calendar agree and it is not a kari-naal avoid day; (2) ALTERNATIVES — "
    "panchangam-only and biodynamic-only dates — if they must sow sooner or prefer one tradition; "
    "(3) the avoid days to skip. Use ONLY the exact dates present in the data. If a list is empty, "
    "say so. Keep it short and friendly."
)


def present(llm, question: str, survey_data: dict) -> str:
    """Data-in -> prose-out: hand the deterministic survey to the model to phrase for the farmer.

    The dates all come from `survey_data`; the model only presents them. This is the reliable
    path for small local models (they narrate given facts rather than driving tools + picking)."""
    user = (f"Farmer's question: {question}\n\n"
            f"Analysis data (use only these dates):\n{json.dumps(survey_data, ensure_ascii=False)}\n\n"
            f"Now present the recommendation and alternatives to the farmer.")
    msg = llm.chat([{"role": "system", "content": PRESENT_SYSTEM},
                    {"role": "user", "content": user}], [])
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
