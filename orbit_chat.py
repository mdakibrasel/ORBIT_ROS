r"""ORBIT chat: test the MCP server in natural language with YOUR API key, without Claude Desktop or VS Code (v0.16.1).

It starts orbit_mcp_server.py exactly as an AI client does (MCP over stdio), gives the server's tools to a language model, and runs
every tool call the model asks for through the real MCP server. Keys come from .env (copy .env.example to .env and paste one key).

    .venv\Scripts\python.exe orbit_chat.py --provider gemini                     # NAO in live Webots via the bridge
    .venv\Scripts\python.exe orbit_chat.py --provider groq --once "go to the kitchen and stand in front of the fridge"
    .\run.ps1 -Mode server                                                       # open the house and start its bridge first

Providers: gemini, groq, cerebras, openrouter, openai (all through their OpenAI-compatible chat endpoint, standard library only).
Model: --model, else <PROVIDER>_MODEL from .env. Type "quit" to leave. Keys are never printed or logged.
"""
from __future__ import annotations
import argparse, json, os, sys, time, urllib.error, urllib.request
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
from orbit_ros.env_config import load_env
from orbit_ros.llm import BASE_URLS, KEY_VARS

BASE_URLS = dict(BASE_URLS, gemini="https://generativelanguage.googleapis.com/v1beta/openai")
KEY_VARS = dict(KEY_VARS, gemini="GEMINI_API_KEY")
DEFAULT_SERVER_ARGS = ["--robot", "nao", "--sim", "webots"]

class McpConnection:
    """The MCP client side: initialize, tools/list, tools/call against orbit_mcp_server.py over stdio."""
    def __init__(self, server_args):
        from orbit_mcp_server import _Client
        self.c = _Client([sys.executable, os.path.join(ROOT, "orbit_mcp_server.py")] + list(server_args))
        init = self.c.request("initialize", {"protocolVersion": "2025-06-18", "capabilities": {},
                                             "clientInfo": {"name": "orbit-chat", "version": "1"}})["result"]
        self.server_info, self.instructions = init["serverInfo"], init.get("instructions", "")
        self.c.notify("notifications/initialized")
        self.tools = self.c.request("tools/list")["result"]["tools"]
    def call(self, name, arguments):
        r = self.c.request("tools/call", {"name": name, "arguments": arguments})
        if "error" in r: return json.dumps({"status": "error", "message": r["error"].get("message", "")})
        return "\n".join(c.get("text", "") for c in r["result"]["content"])
    def close(self): self.c.close()

class ChatModel:
    """OpenAI-compatible chat completions with tool calling."""
    def __init__(self, provider, model, base_url=None, api_key=None, timeout=120, max_retries=4):
        if provider not in BASE_URLS: raise ValueError(f"unknown provider {provider}; choose from {sorted(BASE_URLS)}")
        self.key = api_key or os.getenv(KEY_VARS[provider], "")
        if not self.key: raise RuntimeError(f"Set {KEY_VARS[provider]} in .env (copy .env.example to .env first)")
        self.model = model or os.getenv(provider.upper() + "_MODEL", "")
        if not self.model: raise RuntimeError(f"Set {provider.upper()}_MODEL in .env or pass --model")
        self.provider, self.timeout, self.max_retries = provider, timeout, max_retries
        self.url = (base_url or os.getenv(provider.upper() + "_BASE_URL") or BASE_URLS[provider]).rstrip("/") + "/chat/completions"
    def ask(self, messages, tools):
        body = json.dumps({"model": self.model, "messages": messages, "tools": tools, "temperature": 0}).encode()
        for attempt in range(self.max_retries + 1):
            req = urllib.request.Request(self.url, data=body, method="POST", headers={"Authorization": "Bearer " + self.key,
                                         "Content-Type": "application/json", "User-Agent": "orbit-chat/0.16"})
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as r: return json.loads(r.read().decode())["choices"][0]["message"]
            except urllib.error.HTTPError as e:
                detail = e.read().decode(errors="replace")[:400]
                if e.code in (429, 500, 502, 503, 504) and attempt < self.max_retries: time.sleep(min(30, 2 ** attempt)); continue
                raise RuntimeError(f"{self.provider} HTTP {e.code}: {detail}") from None
            except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
                if attempt < self.max_retries: time.sleep(min(30, 2 ** attempt)); continue
                raise RuntimeError(f"{self.provider} request failed: {e}") from None

def openai_tools(mcp_tools):
    return [{"type": "function", "function": {"name": t["name"], "description": t.get("description", ""),
             "parameters": t.get("inputSchema") or {"type": "object", "properties": {}}}} for t in mcp_tools]

def parse_view_command(text):
    """Parse a '/view ...' chat command into camera() MCP args, or return None if `text` is not a view command.

    Forms:  /view                    -> overview
            /view overview|follow|closeup|target|human|demo|reset
            /view zoom in | /view zoom out | /view zoom <positive number>
            /view target X Y          -> frame the robot and point (X, Y)
            /view <mode> zoom in|out|<n>   -> a framing mode plus a dolly
    Returns (args_dict, error_message). On a parse error, args_dict is None and error_message explains it.
    """
    s = text.strip()
    if not (s == "/view" or s.lower().startswith("/view ") or s.lower().startswith("/camera")):
        return None
    parts = s.split()[1:]                       # drop the leading /view (or /camera)
    MODES = ("overview", "follow", "closeup", "target", "human", "demo", "reset")
    args = {"mode": "overview"}
    if not parts:
        return args, None
    i = 0
    first = parts[0].lower()
    if first == "zoom":
        # zoom-only command; keep the current framing (mode 'zoom' is a no-mode dolly)
        args["mode"] = "zoom"
        if len(parts) < 2:
            return None, "usage: /view zoom in | out | <positive number>"
        args["zoom"] = _parse_zoom_amount(parts[1])
        if args["zoom"] is None:
            return None, f"invalid zoom amount {parts[1]!r}: use in, out or a positive number"
        return args, None
    if first in MODES:
        args["mode"] = first; i = 1
    else:
        return None, f"unknown view mode {parts[0]!r}: use one of {', '.join(MODES)} or 'zoom'"
    if args["mode"] == "target":
        if len(parts) < i + 2:
            return None, "usage: /view target X Y"
        try:
            args["target"] = [float(parts[i]), float(parts[i + 1])]
        except ValueError:
            return None, f"target needs two numbers X Y, got {parts[i:i+2]}"
        i += 2
    # optional trailing 'zoom in|out|<n>'
    if i < len(parts):
        if parts[i].lower() != "zoom":
            return None, f"unexpected {parts[i]!r}; after a mode only 'zoom in|out|<n>' is allowed"
        if len(parts) < i + 2:
            return None, "usage: ... zoom in | out | <positive number>"
        z = _parse_zoom_amount(parts[i + 1])
        if z is None:
            return None, f"invalid zoom amount {parts[i+1]!r}: use in, out or a positive number"
        args["zoom"] = z
    return args, None


def _parse_zoom_amount(tok):
    t = tok.strip().lower()
    if t in ("in", "out"):
        return t
    try:
        v = float(t)
    except ValueError:
        return None
    return v if v > 0 else None


def view_turn(mcp, text, out=print):
    """Handle a '/view ...' command by calling the camera MCP tool directly (NOT through the LLM, so it is never mistaken for a
    robot task). Returns a short human-readable line. Camera is display only: no robot motion, no LLM, no memory change."""
    args, err = parse_view_command(text)
    if args is None:
        return f"view: {err}"
    raw = mcp.call("camera", args)
    try:
        r = json.loads(raw)
    except (ValueError, TypeError):
        return f"view: {raw}"
    if r.get("status") == "error":
        return f"view error: {r.get('detail', 'camera failed')}"
    if not r.get("available", False):
        return f"view: camera not available ({r.get('detail', 'no Webots viewpoint')})"
    bits = [f"mode={r.get('mode') or args['mode']}"]
    if r.get("zoom") is not None:
        zr = r.get("zoom_result") or {}
        bits.append(f"zoom={r['zoom']} (dist {zr.get('distance_before')}->{zr.get('distance_after')})")
    if r.get("builtin_follow") is not None:
        bits.append(f"builtin_follow={r['builtin_follow']}")
    return "view: " + ", ".join(bits)


def run_turn(model, mcp, messages, tools, text, max_steps=25, out=print, memory=False):
    """One user message: let the model call MCP tools until it answers in words. Returns the answer.
    memory=True: the NAO memory brief (recall_memory) is put in front of the request, labelled as advice; live sensors still decide."""
    if memory:
        try:
            brief = json.loads(mcp.call("recall_memory", {})).get("brief")
            if brief: text = f"{brief}\n\nUser request: {text}"; out("  [memory] brief added")
        except (ValueError, AttributeError, KeyError): pass
    messages.append({"role": "user", "content": text})
    for _ in range(max_steps):
        msg = model.ask(messages, tools)
        calls = msg.get("tool_calls") or []
        messages.append({"role": "assistant", "content": msg.get("content") or "", **({"tool_calls": calls} if calls else {})})
        if not calls: return msg.get("content") or ""
        for tc in calls:
            name = tc["function"]["name"]
            try: args = json.loads(tc["function"].get("arguments") or "{}")
            except json.JSONDecodeError: args = {}
            result = mcp.call(name, args)
            try: st = json.loads(result).get("status", "ok")
            except (ValueError, AttributeError): st = "ok"
            out(f"  [MCP] {name} {json.dumps(args)} -> {st}")
            messages.append({"role": "tool", "tool_call_id": tc.get("id", name), "content": result})
    return "(stopped: too many tool calls in one turn)"

def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    server_args = DEFAULT_SERVER_ARGS
    if "--" in argv: i = argv.index("--"); argv, server_args = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--provider", default="gemini", choices=sorted(BASE_URLS))
    ap.add_argument("--model", default=None)
    ap.add_argument("--base-url", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--once", default=None, help="send one message, print the answer, exit")
    ap.add_argument("--mode", default="stepwise", choices=("stepwise", "plan"),
                    help="stepwise: the model calls tools turn by turn (current behaviour); plan: the server-side meta-tool loop "
                         "(schema-constrained plan -> validate -> execute with guard + verify + repair) via plan_and_execute")
    a = ap.parse_args(argv)
    load_env()
    try: model = ChatModel(a.provider, a.model, a.base_url)
    except (RuntimeError, ValueError) as e: print(e, file=sys.stderr); return 2
    mcp = McpConnection(server_args)
    print(f"MCP connected: {mcp.server_info['name']} {mcp.server_info['version']}, {len(mcp.tools)} tools "
          f"({' '.join(server_args)}). Model: {a.provider} {model.model}. Mode: {a.mode}")
    tools = openai_tools(mcp.tools)
    use_memory = any(t["name"] == "recall_memory" for t in mcp.tools)
    messages = [{"role": "system", "content": mcp.instructions or "You control a robot through the given tools."}]

    def handle(text):
        if text.strip().lower() == "/view" or text.strip().lower().startswith(("/view ", "/camera")):
            return view_turn(mcp, text)
        if a.mode == "plan":
            return plan_turn(mcp, text)
        return run_turn(model, mcp, messages, tools, text, memory=use_memory)

    try:
        if a.once: print(handle(a.once)); return 0
        while True:
            try: text = input("\nyou> ").strip()
            except EOFError: break
            if text.lower() in ("quit", "exit", "q"): break
            if text: print("robot> " + handle(text))
    except RuntimeError as e: print(e, file=sys.stderr); return 1
    finally: mcp.close()
    return 0


def plan_turn(mcp, text):
    """Server-side meta-tool loop: call plan_and_execute; print the plan + per-step verdicts; return a short answer."""
    raw = mcp.call("plan_and_execute", {"instruction": text})
    try: payload = json.loads(raw)
    except (ValueError, TypeError): return raw
    res = payload.get("result") or {}
    plan = res.get("plan") or {}
    if not plan.get("feasible", True):
        print(f"  [plan] REFUSED: {plan.get('refusal_reason')}")
        return plan.get("refusal_reason") or "refused"
    for s in plan.get("steps", []):
        print(f"  [plan] {s.get('status','?'):7} {s.get('id')}: {s.get('tool')}({json.dumps(s.get('args', {}))})")
    m = res.get("metrics") or {}
    print(f"  [plan] status={res.get('status')} success={res.get('success')} "
          f"tool_calls={m.get('tool_calls')} repairs={m.get('repairs')} llm_calls={m.get('llm_calls')}")
    said = " ".join(res.get("said") or [])
    return said or f"{res.get('status')} (success={res.get('success')})"

if __name__ == "__main__": sys.exit(main())
