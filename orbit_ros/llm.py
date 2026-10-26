"""LLM clients. Gemini needs `pip install google-genai` and GEMINI_API_KEY; the OpenAI-compatible client (Groq etc.) uses the standard library.
Keys are read from the environment / .env and never printed or logged.

Common interface: client.complete(prompt, tag, images=None) -> reply text; client.usage_events is a list of per-call dicts
(tag, input_tokens, output_tokens, cached_tokens, latency_s, finish_reason, attempts). images: list of PNG bytes for vision models (v0.12).
Prompts put the static part (conventions, tools, sensor documentation) first, so providers with automatic prefix caching can reuse it;
cached_tokens is logged when the provider reports it."""
import base64, json, os, time, urllib.request, urllib.error

def _env_float(name, default):
    try: return float(os.getenv(name) or default)
    except ValueError: return default

class _Pacer:
    """Minimum spacing between requests (LLM_MIN_INTERVAL seconds) so a free-tier rate limit is not hit in bursts."""
    def __init__(self, min_interval): self.min_interval, self.last = min_interval, 0.0
    def wait(self):
        if self.min_interval > 0:
            d = self.last + self.min_interval - time.monotonic()
            if d > 0: time.sleep(d)
        self.last = time.monotonic()

class GeminiClient:
    provider = "google-gemini"
    def __init__(self, model_id="gemini-2.5-flash", temperature=0.0, max_output_tokens=None, max_retries=None, thinking_budget=None):
        from google import genai
        from google.genai import types
        from .env_config import load_env; load_env()          # reads .env from the project root; real environment variables win
        key = os.getenv("GEMINI_API_KEY")
        if not key: raise RuntimeError("Set GEMINI_API_KEY in the environment or in .env")
        self.client, self.types = genai.Client(api_key=key), types
        self.model_id, self.temperature = model_id, temperature
        self.max_tokens = int(max_output_tokens or _env_float("LLM_MAX_OUTPUT_TOKENS", 8192))
        self.max_retries = int(_env_float("LLM_MAX_RETRIES", 6)) if max_retries is None else max_retries
        tb = os.getenv("GEMINI_THINKING_BUDGET") if thinking_budget is None else thinking_budget   # thinking models spend output tokens on thoughts
        self.thinking_budget = None if tb in (None, "") else int(tb)
        self.pacer = _Pacer(_env_float("LLM_MIN_INTERVAL", 0))
        self.usage_events = []
    def settings(self): return {"max_output_tokens": self.max_tokens, "thinking_budget": self.thinking_budget, "max_retries": self.max_retries}
    def complete(self, prompt: str, tag: str = "", images=None) -> str:
        kw = dict(temperature=self.temperature, max_output_tokens=self.max_tokens, response_mime_type="application/json")
        if self.thinking_budget is not None: kw["thinking_config"] = self.types.ThinkingConfig(thinking_budget=self.thinking_budget)
        t0, last = time.perf_counter(), None
        for attempt in range(self.max_retries + 1):
            self.pacer.wait()
            try:
                contents = [prompt] + [self.types.Part.from_bytes(data=im, mime_type="image/png") for im in (images or [])] if images else prompt
                r = self.client.models.generate_content(model=self.model_id, contents=contents, config=self.types.GenerateContentConfig(**kw)); break
            except Exception as e:                       # google-genai raises errors.APIError with .code; retry rate limits and server errors
                last, code = e, getattr(e, "code", None)
                if code in (429, 500, 502, 503, 504) and attempt < self.max_retries: time.sleep(min(60, 2 ** attempt)); continue
                raise RuntimeError(f"gemini error {code}: {str(e)[:300]}") from None
        else: raise RuntimeError(f"gemini failed after retries: {last}")
        u = getattr(r, "usage_metadata", None)
        fr = None
        try: fr = str(r.candidates[0].finish_reason)
        except Exception: pass
        self.usage_events.append({"tag": tag, "input_tokens": getattr(u, "prompt_token_count", None),
            "output_tokens": getattr(u, "candidates_token_count", None), "reasoning_tokens": getattr(u, "thoughts_token_count", None),
            "cached_tokens": getattr(u, "cached_content_token_count", None), "images": len(images or []),
            "latency_s": round(time.perf_counter() - t0, 4), "finish_reason": fr, "attempts": attempt + 1})
        if not getattr(r, "text", None): raise RuntimeError(f"empty model response (finish_reason={fr})")
        return r.text
    def list_models(self):
        return sorted(m.name.removeprefix("models/") for m in self.client.models.list())

class FakeClient:
    """Returns canned replies (for unit tests of parsing/plumbing)."""
    provider, model_id = "fake", "fake"
    def __init__(self, replies): self.replies, self.usage_events = list(replies), []
    def complete(self, prompt, tag="", images=None): self.usage_events.append({"tag": tag, "input_tokens": len(prompt), "output_tokens": 0, "latency_s": 0}); return self.replies.pop(0)

class ScriptedClient:
    """Test double: reply = fn(prompt, tag). Lets tests drive every architecture through the real prompt/parse path without a network."""
    provider, model_id = "scripted", "scripted"
    def __init__(self, fn): self.fn, self.usage_events, self.prompts, self.images = fn, [], [], []
    def complete(self, prompt, tag="", images=None):
        self.prompts.append((tag, prompt)); self.images.append(list(images or []))
        self.usage_events.append({"tag": tag, "input_tokens": len(prompt) // 4, "output_tokens": 10, "latency_s": 0.0})
        return self.fn(prompt, tag)


# ----------------------------------------------------------------------------------------------------------------------------------
# OpenAI-compatible chat-completions client: Groq (the paper's GPT-OSS-20B/120B and Kimi-K2 runs used the Groq API), Cerebras, OpenRouter, OpenAI.
# Standard library only. The HTTP layer is tested against a local fake server (tests/test_llm_openai_compat.py); it has NOT been run against
# the live services by the author. `python preflight_llm.py --planner groq` is the first live test.
BASE_URLS = {"groq": "https://api.groq.com/openai/v1", "cerebras": "https://api.cerebras.ai/v1",
             "openrouter": "https://openrouter.ai/api/v1", "openai": "https://api.openai.com/v1"}
KEY_VARS = {"groq": "GROQ_API_KEY", "cerebras": "CEREBRAS_API_KEY", "openrouter": "OPENROUTER_API_KEY", "openai": "OPENAI_API_KEY"}

class OpenAICompatClient:
    """complete(prompt, tag) -> text, with the same interface and `usage_events` as GeminiClient."""
    def __init__(self, provider="groq", model_id="openai/gpt-oss-120b", temperature=0.0, max_output_tokens=None,
                 base_url=None, api_key=None, timeout=None, max_retries=None, json_mode=True, reasoning_effort=None, min_interval=None):
        from .env_config import load_env; load_env()
        timeout = _env_float("LLM_REQUEST_TIMEOUT", 60) if timeout is None else timeout             # .env: LLM_REQUEST_TIMEOUT
        max_retries = int(_env_float("LLM_MAX_RETRIES", 6)) if max_retries is None else max_retries  # .env: LLM_MAX_RETRIES
        if provider not in BASE_URLS: raise ValueError(f"unknown provider {provider}; choose from {sorted(BASE_URLS)}")
        self.provider, self.model_id, self.temperature = provider, model_id, temperature
        # Reasoning models (gpt-oss) spend completion tokens on reasoning before the answer: keep this generous.
        self.max_tokens = int(max_output_tokens or _env_float("LLM_MAX_OUTPUT_TOKENS", 8192))
        self.base_url = (base_url or os.getenv(provider.upper() + "_BASE_URL") or BASE_URLS[provider]).rstrip("/")
        self.key = api_key if api_key is not None else os.getenv(KEY_VARS[provider], "")
        if not self.key: raise RuntimeError(f"Set {KEY_VARS[provider]} in the environment or in .env")
        self.timeout, self.max_retries, self.json_mode = timeout, max_retries, json_mode
        self.reasoning_effort = reasoning_effort if reasoning_effort is not None else (os.getenv("LLM_REASONING_EFFORT") or None)
        self.pacer = _Pacer(_env_float("LLM_MIN_INTERVAL", 0) if min_interval is None else min_interval)
        self.usage_events = []
    def settings(self): return {"base_url": self.base_url, "max_output_tokens": self.max_tokens, "json_mode": self.json_mode,
                                "reasoning_effort": self.reasoning_effort, "max_retries": self.max_retries, "timeout_s": self.timeout,
                                "min_interval_s": self.pacer.min_interval}

    def _request(self, path, body=None):
        req = urllib.request.Request(self.base_url + path, data=None if body is None else json.dumps(body).encode(), method="GET" if body is None else "POST",
                                     headers={"Authorization": "Bearer " + self.key, "Content-Type": "application/json", "User-Agent": "orbit-ros/0.10"})
        with urllib.request.urlopen(req, timeout=self.timeout) as r: return json.loads(r.read().decode())

    @staticmethod
    def _retry_delay(e, attempt):
        ra = None
        try: ra = float(e.headers.get("retry-after")) if e.headers.get("retry-after") else None
        except (TypeError, ValueError): ra = None
        return min(60.0, ra if ra is not None else 2.0 ** attempt)

    def complete(self, prompt: str, tag: str = "", images=None) -> str:
        content = prompt if not images else [{"type": "text", "text": prompt}] + [
            {"type": "image_url", "image_url": {"url": "data:image/png;base64," + base64.b64encode(im).decode()}} for im in images]
        body = {"model": self.model_id, "temperature": self.temperature, "max_tokens": self.max_tokens,
                "messages": [{"role": "system", "content": "Reply with a single JSON object only. No markdown, no commentary."},
                             {"role": "user", "content": content}]}
        if self.json_mode: body["response_format"] = {"type": "json_object"}
        if self.reasoning_effort: body["reasoning_effort"] = self.reasoning_effort
        t0, last, attempt = time.perf_counter(), None, 0
        for attempt in range(self.max_retries + 1):
            self.pacer.wait()
            try:
                data = self._request("/chat/completions", body); break
            except urllib.error.HTTPError as e:
                last = e; detail = e.read().decode(errors="replace")[:300]
                if e.code == 400 and "response_format" in body and ("response_format" in detail or "json" in detail.lower()):
                    body.pop("response_format"); self.json_mode = False; continue        # model without JSON mode: plain text from now on
                if e.code == 400 and "json_validate_failed" in detail and attempt < self.max_retries:
                    continue                                                              # Groq: the model produced invalid JSON in JSON mode
                if e.code in (408, 409, 429, 500, 502, 503, 504) and attempt < self.max_retries:
                    time.sleep(self._retry_delay(e, attempt)); continue
                raise RuntimeError(f"{self.provider} HTTP {e.code}: {detail}") from None
            except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
                last = e
                if attempt < self.max_retries: time.sleep(min(60, 2 ** attempt)); continue
                raise RuntimeError(f"{self.provider} request failed: {e}") from None
        else: raise RuntimeError(f"{self.provider} failed after retries: {last}")
        u = data.get("usage") or {}
        choice = (data.get("choices") or [{}])[0]
        details = u.get("completion_tokens_details") or {}
        self.usage_events.append({"tag": tag, "input_tokens": u.get("prompt_tokens"), "output_tokens": u.get("completion_tokens"),
                                  "reasoning_tokens": details.get("reasoning_tokens"),
                                  "cached_tokens": (u.get("prompt_tokens_details") or {}).get("cached_tokens"), "images": len(images or []), "latency_s": round(time.perf_counter() - t0, 4),
                                  "finish_reason": choice.get("finish_reason"), "attempts": attempt + 1})
        text = (choice.get("message") or {}).get("content")
        if not text: raise RuntimeError(f"empty model response (finish_reason={choice.get('finish_reason')}); raise LLM_MAX_OUTPUT_TOKENS or lower --reasoning-effort")
        return text

    def list_models(self):
        return sorted(m.get("id", "") for m in self._request("/models").get("data", []))

def make_client(provider: str, model_id: str, temperature: float = 0.0, reasoning_effort=None, max_output_tokens=None):
    if provider == "gemini": return GeminiClient(model_id, temperature, max_output_tokens)
    return OpenAICompatClient(provider, model_id, temperature, max_output_tokens=max_output_tokens, reasoning_effort=reasoning_effort)
