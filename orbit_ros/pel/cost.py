"""ORBIT-PEL money accounting (v0.14 milestone M6): prices from a user-maintained file, cost per call from the recorded usage, and a
hard spending cap enforced before every model call.

No price is built into the code. `configs/pricing.json` holds one entry per (provider, model id); provider is the client's name as written in api_usage.csv (google-gemini, groq, cerebras, openrouter, openai) with the currency, the price per
million tokens, the page the price was read from and the date it was read. A model without an entry has NO price: its cost is
reported as unknown, and `--max-cost` refuses to start (a cap that cannot be computed is not a cap).

Cost of one call, from the provider's own usage fields (as logged in api_usage.csv):
    (input_tokens - cached_tokens) * input + cached_tokens * cached_input + (output_tokens + reasoning_tokens) * output
all prices per 1,000,000 tokens. Gemini reports prompt_token_count INCLUDING cached tokens and candidates_token_count EXCLUDING thinking
tokens (thoughts_token_count), which are billed as output; this is the formula above. Prices with tiers (for example a higher price
above a prompt-length threshold) are applied with `tier_threshold_tokens` / `*_above_threshold` when given. Free-tier usage is not
modelled: the computed figure is what the same usage costs on the paid tier, and every report says so.

Validity (v0.14.1, review L7): an entry may carry `valid_until` (an ISO date, YYYY-MM-DD, the last day the provider's page says the
price applies). After that date `--max-cost` refuses to start with that entry (`check_valid`), because a cap computed from a stale
price is not a cap. Without `valid_until` nothing is checked; the file's comment says to re-read the page before a paid run.
"""
from __future__ import annotations
import datetime, json, os

PRICE_KEYS = ("input", "output")
OPTIONAL_KEYS = ("cached_input", "tier_threshold_tokens", "input_above_threshold", "output_above_threshold", "cached_input_above_threshold")

class PricingError(ValueError): pass
class CostCapReached(RuntimeError): pass

def load_pricing(path: str) -> dict:
    """-> {"currency", "entries": {(provider, model): entry}}. Rejects entries without a source URL and a retrieval date."""
    with open(path, encoding="utf-8") as f: d = json.load(f)
    cur = d.get("currency")
    if not isinstance(cur, str) or not cur: raise PricingError("pricing file needs a currency, e.g. \"USD\"")
    out = {}
    for i, e in enumerate(d.get("models") or []):
        where = f"models[{i}]"
        for k in ("provider", "model", "source_url", "retrieved"):
            if not isinstance(e.get(k), str) or not e[k]: raise PricingError(f"{where}: {k} is required")
        for k in PRICE_KEYS:
            if not isinstance(e.get(k), (int, float)) or isinstance(e.get(k), bool) or e[k] < 0: raise PricingError(f"{where}: {k} must be a number >= 0 (price per 1M tokens)")
        for k in OPTIONAL_KEYS:
            if k in e and (not isinstance(e[k], (int, float)) or isinstance(e[k], bool) or e[k] < 0): raise PricingError(f"{where}: {k} must be a number >= 0")
        if "valid_until" in e: _date(e["valid_until"], f"{where}: valid_until")
        out[(e["provider"], e["model"])] = dict(e)
    return {"currency": cur, "entries": out}

def _date(v, where) -> datetime.date:
    if not isinstance(v, str): raise PricingError(f"{where} must be a date YYYY-MM-DD")
    try: return datetime.date.fromisoformat(v)
    except ValueError: raise PricingError(f"{where} must be a date YYYY-MM-DD")

def check_valid(price: dict, today: datetime.date | None = None) -> None:
    """Raise PricingError when the entry's valid_until is in the past (UTC date). No valid_until: nothing to check."""
    if not price or "valid_until" not in price: return
    today = today or datetime.datetime.now(datetime.timezone.utc).date()
    until = _date(price["valid_until"], "valid_until")
    if today > until:
        raise PricingError(f"the price for ({price.get('provider')}, {price.get('model')}) was valid until {price['valid_until']}; "
                           f"re-read {price.get('source_url')} and update configs/pricing.json")

def price_for(pricing: dict | None, provider: str, model: str) -> dict | None:
    if not pricing: return None
    return pricing["entries"].get((provider, model))

def _n(v) -> int:
    try: return max(0, int(v))
    except (TypeError, ValueError): return 0

def call_cost(usage: dict, price: dict) -> float | None:
    """Cost of one call in the pricing file's currency, or None when the provider reported no token counts."""
    inp, out = usage.get("input_tokens"), usage.get("output_tokens")
    if not isinstance(inp, int) or not isinstance(out, int) or isinstance(inp, bool) or isinstance(out, bool): return None
    cached, reasoning = _n(usage.get("cached_tokens")), _n(usage.get("reasoning_tokens"))
    above = "tier_threshold_tokens" in price and inp > price["tier_threshold_tokens"]
    p_in = price.get("input_above_threshold", price["input"]) if above else price["input"]
    p_out = price.get("output_above_threshold", price["output"]) if above else price["output"]
    p_cached = price.get("cached_input_above_threshold" if above else "cached_input", p_in)
    cached = min(cached, inp)
    return ((inp - cached) * p_in + cached * p_cached + (out + reasoning) * p_out) / 1_000_000

class CostCappedClient:
    """Wraps a model client (GeminiClient, OpenAICompatClient, ScriptedClient ...). Before every call it adds up the cost of the calls
    already made through it; once that reaches `cap`, the call is refused with CostCapReached (no request is sent). Overshoot is at most
    the cost of the one call that crossed the cap. Calls whose usage carries no token counts make the total unknown: the client then
    refuses further calls too (fail closed)."""
    def __init__(self, client, price: dict, cap: float):
        if price is None: raise PricingError("no price for this model: --max-cost needs an entry in the pricing file")
        if not isinstance(cap, (int, float)) or cap <= 0: raise PricingError("--max-cost must be > 0")
        self._client, self.price, self.cap, self.refused = client, price, float(cap), 0
    def __getattr__(self, k): return getattr(self._client, k)
    @property
    def usage_events(self): return self._client.usage_events
    def spent(self) -> float | None:
        total = 0.0
        for u in self._client.usage_events:
            c = call_cost(u, self.price)
            if c is None: return None
            total += c
        return total
    def complete(self, prompt, tag="", images=None):
        s = self.spent()
        if s is None: self.refused += 1; raise CostCapReached("cost unknown: a previous call reported no token usage")
        if s >= self.cap: self.refused += 1; raise CostCapReached(f"spending cap reached: {s:.6f} >= {self.cap:.6f}")
        return self._client.complete(prompt, tag, images=images) if images else self._client.complete(prompt, tag)

def default_pricing_path() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "configs", "pricing.json")
