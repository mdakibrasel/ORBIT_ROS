"""Provider I/O for the meta-tool (section 4.5). Uses the existing provider config (orbit_ros.llm, env_config keys).

Mode order per provider: response_format=json_schema -> json_object -> plain text. The mode a provider rejected is remembered so we do
not keep retrying it. Retries 429/5xx (and transient 403, e.g. Cloudflare) with backoff honouring retry-after. Records tokens + latency.
The returned text is ALWAYS re-validated locally by the caller, because some hosted models ignore the schema. The key is never logged."""
from __future__ import annotations
import json, os, time, urllib.request, urllib.error

from .plan import extract_json

# provider base urls / key env names (gemini uses its OpenAI-compatible endpoint, as in orbit_chat)
def _provider_cfg():
    from orbit_ros.llm import BASE_URLS, KEY_VARS
    base = dict(BASE_URLS, gemini="https://generativelanguage.googleapis.com/v1beta/openai")
    keyv = dict(KEY_VARS, gemini="GEMINI_API_KEY")
    return base, keyv

_REJECTED_MODE = {}      # (provider, model) -> set of modes the host rejected this process


class LlmClient:
    def __init__(self, provider="groq", model=None, temperature=0.0, max_tokens=4096, timeout=90, max_retries=5):
        from orbit_ros.env_config import load_env
        load_env()
        base, keyv = _provider_cfg()
        if provider not in base: raise ValueError(f"unknown provider {provider}")
        self.provider = provider
        self.key = os.getenv(keyv[provider], "")
        self.model = model or os.getenv(provider.upper() + "_MODEL", "")
        self.url = (os.getenv(provider.upper() + "_BASE_URL") or base[provider]).rstrip("/") + "/chat/completions"
        self.temperature, self.max_tokens, self.timeout, self.max_retries = temperature, max_tokens, timeout, max_retries
        self.calls = 0
        self.tokens = {}
        self.latency_s = 0.0
        self.retries = 0

    def available(self):
        return bool(self.key) and bool(self.model)

    def _modes(self, schema):
        key = (self.provider, self.model)
        rejected = _REJECTED_MODE.get(key, set())
        order = []
        if schema is not None and "json_schema" not in rejected: order.append("json_schema")
        if "json_object" not in rejected: order.append("json_object")
        order.append("text")
        return order

    def _body(self, messages, tools, schema, mode):
        b = {"model": self.model, "messages": messages, "temperature": self.temperature, "max_tokens": self.max_tokens}
        if tools: b["tools"] = tools
        if mode == "json_schema" and schema is not None:
            b["response_format"] = {"type": "json_schema", "json_schema": {"name": "orbit_plan", "schema": schema, "strict": True}}
        elif mode == "json_object":
            b["response_format"] = {"type": "json_object"}
        return b

    def _post(self, body):
        data = json.dumps(body).encode()
        for attempt in range(self.max_retries + 1):
            req = urllib.request.Request(self.url, data=data, method="POST",
                                        headers={"Authorization": "Bearer " + self.key, "Content-Type": "application/json",
                                                 "User-Agent": "orbit-metatool/1"})
            t = time.time()
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as r:
                    out = json.loads(r.read().decode())
            except urllib.error.HTTPError as e:
                if e.code in (403, 429, 500, 502, 503, 504) and attempt < self.max_retries:
                    ra = e.headers.get("retry-after") if hasattr(e, "headers") else None
                    self.retries += 1
                    time.sleep(float(ra) if ra and str(ra).replace(".", "", 1).isdigit() else min(30, 2 ** attempt)); continue
                raise
            self.calls += 1; self.latency_s += time.time() - t
            for k in ("prompt_tokens", "completion_tokens", "total_tokens"):
                if k in (out.get("usage") or {}): self.tokens[k] = self.tokens.get(k, 0) + out["usage"][k]
            return out

    def plan(self, messages, schema, tools=None):
        """Request a plan; try modes in order; return (plan_dict_or_None, raw_text, mode_used). Caller re-validates locally."""
        key = (self.provider, self.model)
        last_text = ""
        for mode in self._modes(schema):
            try:
                out = self._post(self._body(messages, tools, schema, mode))
            except urllib.error.HTTPError as e:
                # a 400/422 usually means the host rejected the response_format mode: remember and fall through
                if e.code in (400, 404, 422):
                    _REJECTED_MODE.setdefault(key, set()).add(mode); continue
                raise
            msg = out["choices"][0]["message"]
            last_text = msg.get("content") or ""
            d = extract_json(last_text)
            if d is not None:
                return d, last_text, mode
            # the host accepted the mode but returned no JSON: remember to try a looser mode
            _REJECTED_MODE.setdefault(key, set()).add(mode)
        return None, last_text, "text"

    def chat(self, messages, tools=None):
        """A plain tool-calling turn (used by repair / the stepwise path). Returns the assistant message dict."""
        out = self._post(self._body(messages, tools, None, "text"))
        return out["choices"][0]["message"]

    def metrics(self):
        return {"llm_calls": self.calls, "llm_tokens": dict(self.tokens), "llm_latency_s": round(self.latency_s, 2),
                "llm_retries": self.retries, "provider": self.provider, "model": self.model}
