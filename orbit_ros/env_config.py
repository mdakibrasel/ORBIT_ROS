"""Dependency-free `.env` loader and provider status. Keys are never printed, logged or written anywhere.

* The file is read from an explicit path (default: the project root, i.e. next to `run_v08.py`), not from the working directory.
* Variables that already exist in the environment win (same rule as python-dotenv's default), so a real environment variable
  or a CI secret always overrides the file.
* `provider_status` reports only configured / missing and the model name.
"""
from __future__ import annotations
import os, re

PROVIDERS = {"gemini": ("GEMINI_API_KEY", "GEMINI_MODEL"), "groq": ("GROQ_API_KEY", "GROQ_MODEL"), "cerebras": ("CEREBRAS_API_KEY", "CEREBRAS_MODEL"),
             "openrouter": ("OPENROUTER_API_KEY", "OPENROUTER_MODEL"), "openai": ("OPENAI_API_KEY", "OPENAI_MODEL"),
             "anthropic": ("ANTHROPIC_API_KEY", "ANTHROPIC_MODEL")}
_LINE = re.compile(r"^\s*(?:export\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$")

def project_root() -> str:
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def parse_env(text: str) -> dict:
    out = {}
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"): continue
        m = _LINE.match(raw)
        if not m: continue
        key, val = m.group(1), m.group(2)
        if val and val[0] in "\"'" and val.count(val[0]) >= 2: val = val[1:val.index(val[0], 1)]       # quoted value: keep everything inside
        else: val = re.split(r"(?:^|\s+)#", val, maxsplit=1)[0].strip()                        # unquoted: strip trailing comment (an empty value + comment is empty)
        out[key] = val
    return out

def load_env(path: str | None = None, override: bool = False, environ=None) -> list:
    """Load KEY=VALUE pairs into the environment. Returns the NAMES that were set (never values). Missing file -> []."""
    environ = os.environ if environ is None else environ
    path = path or os.path.join(project_root(), ".env")
    if not os.path.isfile(path): return []
    with open(path, encoding="utf-8-sig") as f: pairs = parse_env(f.read())
    set_now = []
    for k, v in pairs.items():
        if v == "": continue                                   # an empty placeholder never overrides anything
        if override or k not in environ: environ[k] = v; set_now.append(k)
    return set_now

def provider_status(environ=None) -> list:
    environ = os.environ if environ is None else environ
    order = [p.strip() for p in environ.get("LLM_PROVIDER_ORDER", "gemini").split(",") if p.strip()]
    rows = []
    for name in order + [p for p in PROVIDERS if p not in order]:
        if name not in PROVIDERS: rows.append({"provider": name, "configured": False, "model": "", "in_order": name in order, "note": "unknown provider name"}); continue
        key_var, model_var = PROVIDERS[name]
        rows.append({"provider": name, "configured": bool(environ.get(key_var, "").strip()), "model": environ.get(model_var, ""), "in_order": name in order, "note": ""})
    return rows

def warnings(environ=None) -> list:
    environ = os.environ if environ is None else environ
    w = []
    if environ.get("GOOGLE_API_KEY") and environ.get("GEMINI_API_KEY"):
        w.append("GOOGLE_API_KEY and GEMINI_API_KEY are both set: the google-genai SDK uses GOOGLE_API_KEY. Remove one.")
    if environ.get("OPENAI_BASE_URL"): w.append("OPENAI_BASE_URL is set: OpenAI-compatible clients will use it.")
    return w

def format_status(environ=None) -> str:
    lines = ["Provider        key        model"]
    for r in provider_status(environ):
        lines.append(f"{r['provider']:<15} {'set' if r['configured'] else 'missing':<10} {r['model'] or '-'}{'' if r['in_order'] else '   (not in LLM_PROVIDER_ORDER)'}{'   ' + r['note'] if r['note'] else ''}")
    lines += ["WARNING: " + x for x in warnings(environ)]
    return "\n".join(lines)
