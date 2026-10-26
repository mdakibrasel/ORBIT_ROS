"""First live test of a model provider, BEFORE any pilot. Costs a handful of calls. Never prints the key.

  python preflight_llm.py --planner groq   --model openai/gpt-oss-120b
  python preflight_llm.py --planner groq   --model openai/gpt-oss-20b
  python preflight_llm.py --planner groq   --model moonshotai/kimi-k2-instruct-0905     (check the id with --list-models)
  python preflight_llm.py --planner gemini --model gemini-2.5-flash
  python preflight_llm.py --planner groq --list-models

Steps: (1) the model id is offered by the provider, (2) one JSON round trip (parse, tokens, latency, finish_reason),
(3) one easy benchmark episode (motion level 1) through the real prompt path with S1 and C4. Exit code 0 only if (2) and (3) work.
Episode success in step 3 is NOT a result: it only shows the plumbing works."""
import argparse, json, os, sys, time
from orbit_ros.env_config import load_env

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--planner", required=True, choices=["gemini", "groq", "cerebras", "openrouter", "openai"])
    p.add_argument("--model", default=None); p.add_argument("--reasoning-effort", default=None)
    p.add_argument("--list-models", action="store_true"); p.add_argument("--skip-episode", action="store_true")
    a = p.parse_args()
    load_env()
    from orbit_ros.llm import make_client
    model = a.model or os.getenv(a.planner.upper() + "_MODEL") or {"gemini": "gemini-2.5-flash", "groq": "openai/gpt-oss-120b"}.get(a.planner)
    try: client = make_client(a.planner, model or "none", 0.0, a.reasoning_effort)
    except Exception as e: print(f"[preflight] FAIL creating client: {e}"); return 2
    print(f"[preflight] provider={a.planner} model={model} settings={json.dumps(getattr(client, 'settings', lambda: {})())}")
    try:
        ids = client.list_models()
        if a.list_models: print("\n".join(ids)); return 0
        print(f"[1] model list: {len(ids)} models; requested id {'FOUND' if model in ids else 'NOT in the list (check the spelling / --list-models)'}")
    except Exception as e:
        print(f"[1] model list unavailable ({str(e)[:200]}); continuing")
        if a.list_models: return 2
    try:
        t0 = time.perf_counter()
        txt = client.complete('Return the JSON object {"ok": true, "sum": X} where X is 2+3.', "preflight")
        u = client.usage_events[-1]
        from orbit_ros.backends2d import _extract_json
        obj = _extract_json(txt)
        good = isinstance(obj, dict) and obj.get("sum") == 5
        print(f"[2] JSON round trip: {'OK' if good else 'UNEXPECTED'} reply={txt[:120]!r} latency={time.perf_counter() - t0:.2f}s "
              f"in={u.get('input_tokens')} out={u.get('output_tokens')} reasoning={u.get('reasoning_tokens')} finish={u.get('finish_reason')} attempts={u.get('attempts')}")
        if not good: return 1
    except Exception as e: print(f"[2] FAIL: {str(e)[:300]}"); return 1
    if a.skip_episode: return 0
    from orbit_ros.tasks2d import make_task
    from orbit_ros.agent2d import run_episode, PRESETS
    from orbit_ros.backends2d import LLMBackend
    ok = True
    for name in ("S1", "C4"):
        world, task = make_task("motion", 0, 90, 1, False, 1); cfg = PRESETS[name]; events = []
        try:
            row = run_episode(world, task, cfg, LLMBackend(client, cfg.grounded, cfg.memory), events)
            print(f"[3] {name}: '{task.phases[0].text}' success={row['success']} model_calls={row['llm_calls']} parse_errors={row['parse_errors']} "
                  f"invalid_calls={row['invalid_calls']} repeated={row['repeated_calls']} gate_rejections={row['gate_rejections']} time={row['elapsed_s']}s")
            for e in events[:12]: print("      ", e["event"], e["details"][:140])
        except Exception as e: ok = False; print(f"[3] {name}: FAIL {str(e)[:300]}")
    print("[preflight] done: plumbing works, now run the --limit 20 pilot" if ok else "[preflight] episode step failed: send me this output")
    return 0 if ok else 1

if __name__ == "__main__": sys.exit(main())
