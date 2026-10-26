"""Pre-flight: prove the configured LLM provider is genuinely reachable with a real API call (no robot, minimal tokens).
Prints model id, latency, and provider-reported token usage if present. Never prints the key."""
import os, sys, time, json, urllib.request, urllib.error
sys.path.insert(0, ".")
from orbit_ros.env_config import load_env
from orbit_ros.llm import BASE_URLS, KEY_VARS
load_env()
BASE_URLS = dict(BASE_URLS, gemini="https://generativelanguage.googleapis.com/v1beta/openai")
KEY_VARS = dict(KEY_VARS, gemini="GEMINI_API_KEY")
prov = sys.argv[1] if len(sys.argv) > 1 else "groq"
key = os.getenv(KEY_VARS[prov], ""); model = os.getenv(prov.upper() + "_MODEL", "")
if not key: print(f"BLOCKER: no API key for {prov}"); sys.exit(3)
url = BASE_URLS[prov].rstrip("/") + "/chat/completions"
body = json.dumps({"model": model, "messages": [{"role": "user", "content": "Reply with the single word READY."}],
                   "temperature": 0, "max_tokens": 16}).encode()
req = urllib.request.Request(url, data=body, method="POST",
                            headers={"Authorization": "Bearer " + key, "Content-Type": "application/json", "User-Agent": "p9/1"})
t0 = time.time()
try:
    with urllib.request.urlopen(req, timeout=60) as r:
        data = json.loads(r.read().decode())
    dt = time.time() - t0
    msg = data["choices"][0]["message"].get("content", "")
    usage = data.get("usage", {})
    print(f"PROVIDER={prov} MODEL={model} LATENCY_S={dt:.2f}")
    print(f"REPLY={msg!r}")
    print(f"USAGE={json.dumps(usage)}")
    print("LLM_CHECK: OK")
except urllib.error.HTTPError as e:
    print(f"LLM_CHECK: HTTP {e.code} {e.read().decode(errors='replace')[:300]}"); sys.exit(4)
except Exception as e:
    print(f"LLM_CHECK: FAILED {type(e).__name__}: {e}"); sys.exit(5)
