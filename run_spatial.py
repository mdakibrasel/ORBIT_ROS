"""Spatial benchmark runner: instruction families x architectures (S0-S5 ladder, paper arms C1-C4).

Oracle (no key, harness validation only):
  python run_spatial.py --planner oracle --suite all --seeds 3 --out results_spatial_oracle_v010
Real model, pilot first (writes as it goes; re-run the same command with --resume after an interruption):
  python run_spatial.py --planner groq --model openai/gpt-oss-120b --suite all --seeds 1 --limit 20 --configs C1 C2 C3 C4 --events --out results_groq120b_pilot
Cost estimate without any API call (counts the oracle's model calls for the same grid):
  python run_spatial.py --planner groq --suite all --seeds 1 --limit 20 --configs C1 C2 C3 C4 --estimate
"""
import argparse, csv, dataclasses, hashlib, itertools, json, os, platform, sys, time
from datetime import datetime, timezone
from orbit_ros.tasks2d import make_task, movable_ids, FAMILIES, EXTENDED_FAMILIES, HARD_FAMILIES, ALL_FAMILIES
from orbit_ros.agent2d import run_episode, PRESETS, S_LADDER, C_LADDER, WINDOW, config_by_name
from orbit_ros.backends2d import OracleBackend, LLMBackend, CONVENTIONS, TOOLS
from orbit_ros.phrasing import rephrase, STYLES
from orbit_ros.lessons import LessonStore, template_lesson
from orbit_ros.world2d import Realism
from orbit_ros.scenario_sets import SCENARIOS, instances as scenario_instances, scenarios_of, realism_for
from orbit_ros.ensemble import SelfConsistent, Mixed, TokenBudget, Deadline
from orbit_ros.manifest import run_manifest

VERSION = "0.16.2-orbit-nao-house"
YAWS = (0, 90, 180, 270)
LEVELS = {"motion": (1, 2, 3), "clean_side": (1, 2, 3), "sequence": (1, 2, 3), "relative": (1, 2, 3),
          "ambiguous": (1, 2, 3, 4), "impossible": (1, 2), "change": (1, 2), "prohibit": (1, 2), "conditional": (1, 2), "report": (1, 2)}
NS = {"motion": (1, 3), "goto": (1, 3, 5), "clean_left": (1, 3, 5), "remember_return": (2,),
      "clean_side": (1, 3, 5), "sequence": (3, 5), "relative": (3, 5),
      "ambiguous": (3, 5), "change": (3, 5), "prohibit": (3, 5), "conditional": (3, 5), "report": (3, 5), "impossible": (3, 5)}
MIRRORED = ("goto", "remember_return", "sequence", "relative", "ambiguous", "change", "conditional", "report")

HELDOUT_SEED0 = 1000

def unwrap(backend, cls):
    """Find a wrapper of type `cls` in a chain of backend wrappers (TokenBudget / Deadline / SelfConsistent ... -> inner)."""
    b = backend
    while b is not None:
        if isinstance(b, cls): return b
        b = b.__dict__.get("inner")
    return None

def grid(families, seeds, extended_motion=False):
    for fam in families:
        levels = LEVELS.get(fam, (1,))
        if fam == "motion" and extended_motion: levels = (1, 2, 3, 4)
        mirs = (False, True) if fam in MIRRORED else (False,)
        for yaw, n, m, lv, s in itertools.product(YAWS, NS[fam], mirs, levels, range(seeds)):
            yield fam, s, yaw, n, m, lv

def _spread(items, k):
    """Deterministic spread of k items across the list (not a prefix-biased slice)."""
    if not k or k >= len(items): return list(items)
    if k == 1: return [items[0]]
    return [items[round(i * (len(items) - 1) / (k - 1))] for i in range(k)]

def select_instances(families, seeds, suite, limit, per_family=0):
    instances = list(grid(families, seeds, extended_motion=(suite != "core")))
    if suite == "extended": instances = [i for i in instances if i[0] != "motion" or i[5] == 4]   # extended suite adds only motion level 4
    if suite == "hard": instances = [i for i in instances if i[0] in HARD_FAMILIES]
    if per_family:          # protocol design: exactly N trials per family (raise --seeds if a family has fewer than N instances)
        out = []
        for fam in dict.fromkeys(i[0] for i in instances):
            fi = [i for i in instances if i[0] == fam]
            if len(fi) < per_family: raise SystemExit(f"--per-family {per_family}: family {fam} has only {len(fi)} instances at --seeds {seeds}; raise --seeds")
            out += _spread(fi, per_family)
        instances = out
    return _spread(instances, limit) if limit else instances

def write_csv(path, rows):
    if not rows: return
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); w.writerows(rows)

def read_jsonl(path):
    if not os.path.isfile(path): return []
    with open(path, encoding="utf-8") as f: return [json.loads(l) for l in f if l.strip()]

def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--planner", choices=["oracle", "gemini", "groq", "cerebras", "openrouter", "openai"], default="oracle")
    p.add_argument("--model", default=None, help="model id; default = <PROVIDER>_MODEL from .env, else a built-in default")
    p.add_argument("--suite", choices=["core", "extended", "hard", "all"], default="core",
                   help="core = original 4 families (reproduces old results); extended = clean_side, sequence, relative + motion level 4; "
                        "hard = ambiguous, change, prohibit, conditional, report; all = everything")
    p.add_argument("--families", nargs="*", default=None, help="explicit family list (overrides --suite)")
    p.add_argument("--scenario", nargs="*", choices=list(SCENARIOS), default=None,
                   help="v0.12 scenario sets A (basic navigation), B (orientations), C (multiple objects), D (long-horizon); overrides --suite/--families")
    p.add_argument("--configs", nargs="*", default=list(S_LADDER) + ["S6"] + list(C_LADDER), help="S0..S6, C1..C5, PEL, PEL-<registry|guard|verify|replan>, S4v<k>, <config>+L (lesson memory)")
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--repeats", type=int, default=1, help="run every (instance, config) this many times (run-to-run variance of a real model)")
    p.add_argument("--limit", type=int, default=0, help="max number of task instances (smoke/pilot runs)")
    p.add_argument("--per-family", type=int, default=0, help="exactly N instances per family, spread over the grid (protocol: 20)")
    p.add_argument("--phrasing", choices=STYLES, default="canonical", help="instruction wording: canonical | paraphrase | bn (Bengali)")
    p.add_argument("--slip", type=float, default=0.05); p.add_argument("--half-confusion", type=float, default=0.30)
    p.add_argument("--deviation", type=float, default=0.10, help="oracle only: C3/C4 per-step plan-deviation probability (assumed)")
    p.add_argument("--max-actions", type=int, default=150, help="per-instruction step budget (episode tool-call cap = 2x). Pre-register it; lower = cheaper pilots")
    p.add_argument("--ask-p", type=float, default=0.8, help="oracle only: probability of asking when an instruction is ambiguous (assumed)")
    p.add_argument("--false-ask", type=float, default=0.0, help="oracle only: probability of an unnecessary question per instruction (assumed)")
    p.add_argument("--legacy-oracle", action="store_true", help="oracle never abstains (reproduces v0.10 and older oracle numbers)")
    p.add_argument("--no-ask", action="store_true", help="LLM only: do not offer the ask/abstain tools in the prompt")
    p.add_argument("--nav-tools", action="store_true", help="offer the goto_cell / cover_region macro tools (coverage planner as a tool)")
    p.add_argument("--hierarchical", action="store_true", help="LLM only: ask for plans as room -> regions -> tool calls")
    p.add_argument("--lesson-k", type=int, default=3, help="lessons retrieved per episode for S6 / <config>+L")
    p.add_argument("--window", type=int, default=WINDOW, help="history window shown to step-wise agents (S0, C1-C4)")
    p.add_argument("--no-sensor-doc", action="store_true", help="ablation: hide the camera documentation from grounded configs (LLM only)")
    p.add_argument("--no-interp-view", action="store_true", help="ablation: show raw camera values instead of the interpreted view (LLM only)")
    p.add_argument("--room-scale", type=int, default=1, help="realism: room size multiplier for clean_left/clean_side/goto (2 = 12x8 / 17x13)")
    p.add_argument("--odom-noise", type=float, default=0.0, help="realism: P(move over/under-runs by 1 m); the reported pose does not see it")
    p.add_argument("--det-drop", type=float, default=0.0, help="realism: P(a visible object is missed in one describe_view)")
    p.add_argument("--img-noise", type=float, default=0.0, help="realism: std-dev of Gaussian noise on image_x (relative on apparent_size)")
    p.add_argument("--stale-view", type=float, default=0.0, help="realism: P(describe_view returns the previous frame) = camera latency")
    p.add_argument("--moving-every", type=int, default=0, help="v0.12 realism: every N robot calls, objects the instruction does not mention step to a free neighbouring cell")
    p.add_argument("--localizer", choices=["none", "tool", "auto"], default="none", help="v0.12: map-based localiser: none | tool (the model may call localize) | auto (runtime localises after every move)")
    p.add_argument("--visual-reloc", action="store_true", help="v0.12: grounded configs (S2+) correct odometry drift from re-observed landmarks")
    p.add_argument("--self-consistency", type=int, default=1, help="v0.12: sample every request K times and vote (use --temperature > 0 with a real model)")
    p.add_argument("--step-planner", choices=["gemini", "groq", "cerebras", "openrouter", "openai"], default=None,
                   help="v0.12 model mixing: per-step calls (S0, C1-C4) go to this provider; plans/repairs stay with --planner")
    p.add_argument("--step-model", default=None, help="model id for --step-planner")
    p.add_argument("--token-budget", type=int, default=0, help="v0.12: stop asking the model once an episode used this many tokens (0 = no limit)")
    p.add_argument("--vision", action="store_true", help="v0.12: the model also gets the rendered camera image (PNG); needs a vision model")
    p.add_argument("--horizon", type=int, default=0, help="v0.13: plan configs execute at most K actions, then observe and re-plan (0 = open loop)")
    p.add_argument("--safety-gate", action="store_true", help="v0.13: simulator-side gate stops the robot before an obstacle or wall (independent of the model)")
    p.add_argument("--fail-closed", action="store_true", help="v0.13: a camera frame that stays stale after 2 retries stops the robot (outcome observation_failure)")
    p.add_argument("--sim", choices=["world2d", "fake", "webots", "ros2"], default="world2d",
                   help="v0.14: simulator backend (fake = pure-Python FakeSim; webots / ros2 need the real simulator)")
    p.add_argument("--motion", choices=["grid", "continuous"], default="grid", help="v0.14: grid (teleport) or continuous (wheel physics) motion in --sim")
    p.add_argument("--ros2-config", default=None, help="v0.14: ROS 2 topic config for --sim ros2 (configs/ros2_jazzy_tb3.json)")
    p.add_argument("--pel-prompt", choices=["legacy", "v1"], default="legacy",
                   help="v0.14: PEL configs ask for plans in the S1-S5 format (legacy) or in orbit.plan/v1 (v1); LLM planners only")
    p.add_argument("--max-cost", type=float, default=0, help="v0.14: stop before spending more than this (currency of --pricing); needs a price for the model")
    p.add_argument("--pricing", default=None, help="v0.14: pricing file (default configs/pricing.json)")
    p.add_argument("--pel-state-dir", default=None, help="v0.14: PEL configs write their orbit.exec_state/v1 record per instruction to this folder")
    p.add_argument("--episode-timeout", type=float, default=0, help="v0.13: wall-clock seconds per episode for model decisions (0 = none; outcome timeout)")
    p.add_argument("--heldout", action="store_true", help="v0.13: use held-out seeds 1000+ (never use them for prompt tuning)")
    p.add_argument("--config", default=None, help="v0.13: JSON file of option values (keys = option names with _ instead of -); command-line options win")
    p.add_argument("--temperature", type=float, default=0.0)
    p.add_argument("--reasoning-effort", default=None, help="gpt-oss on Groq: low|medium|high (default: provider default)")
    p.add_argument("--max-output-tokens", type=int, default=None)
    p.add_argument("--max-llm-calls", type=int, default=0, help="stop cleanly once this many model calls were made (budget guard)")
    p.add_argument("--estimate", action="store_true", help="no API call: run the oracle on the same grid and print the number of model calls")
    p.add_argument("--resume", action="store_true", help="skip (instance, config, repeat) already in <out>/trial_results.jsonl")
    p.add_argument("--out", default="results_spatial"); p.add_argument("--events", action="store_true")
    pre, _ = p.parse_known_args(argv)
    if pre.config:
        with open(pre.config, encoding="utf-8") as f: conf = {k: v for k, v in json.load(f).items() if not k.startswith("_")}   # "_..." = comment
        unknown = [k for k in conf if k not in vars(pre)]
        if unknown: p.error(f"unknown keys in {pre.config}: {unknown}")
        p.set_defaults(**conf)
    a = p.parse_args(argv)
    cfgs = [config_by_name(n) for n in a.configs]
    if a.families is None:
        a.families = {"core": list(FAMILIES), "extended": list(EXTENDED_FAMILIES) + ["motion"], "hard": list(HARD_FAMILIES),
                      "all": list(ALL_FAMILIES)}[a.suite]
    if a.scenario:
        instances = scenario_instances(a.scenario, a.seeds); a.families = list(dict.fromkeys(i[0] for i in instances))
        instances = _spread(instances, a.limit) if a.limit else instances
    else: instances = select_instances(a.families, a.seeds, a.suite, a.limit, a.per_family)
    seed0 = HELDOUT_SEED0 if a.heldout else 0          # held-out seeds: same grid, unseen worlds (never used for prompt tuning)
    instances = [(f, s_ + seed0, y, n, m, lv) for f, s_, y, n, m, lv in instances]
    live = a.planner != "oracle" and not a.estimate
    client = None
    if live:
        from orbit_ros.llm import make_client
        from orbit_ros.env_config import load_env; load_env()
        a.model = a.model or os.getenv(a.planner.upper() + "_MODEL") or {"gemini": "gemini-2.5-flash", "groq": "openai/gpt-oss-120b"}.get(a.planner)
        if not a.model: p.error(f"no model for {a.planner}: pass --model or set {a.planner.upper()}_MODEL")
        client = make_client(a.planner, a.model, a.temperature, a.reasoning_effort, a.max_output_tokens)
        if a.max_cost:                                  # v0.14: hard money cap, checked before every model call
            from orbit_ros.pel.cost import load_pricing, price_for, CostCappedClient, default_pricing_path, check_valid, PricingError
            pr = load_pricing(a.pricing or default_pricing_path()); price = price_for(pr, client.provider, a.model)
            if price is None: p.error(f"--max-cost: no price for ({client.provider}, {a.model}) in {a.pricing or default_pricing_path()}; add it from the provider's pricing page")
            try: check_valid(price)                         # v0.14.1: a price past its valid_until is refused
            except PricingError as e: p.error(f"--max-cost: {e}")
            client = CostCappedClient(client, price, a.max_cost); a.cost_currency = pr["currency"]
    elif a.max_cost and not a.estimate: p.error("--max-cost needs a real --planner (the oracle has no cost)")
    step_client = None
    if a.step_planner and live:
        from orbit_ros.llm import make_client
        a.step_model = a.step_model or os.getenv(a.step_planner.upper() + "_MODEL")
        if not a.step_model: p.error(f"no model for {a.step_planner}: pass --step-model")
        step_client = make_client(a.step_planner, a.step_model, a.temperature, a.reasoning_effort, a.max_output_tokens)
    elif a.step_planner and not a.estimate: p.error("--step-planner needs a real --planner (model mixing is not simulated by the oracle)")
    if a.step_planner and a.max_cost: p.error("--max-cost does not cover --step-planner yet; run without model mixing")
    if not a.estimate: os.makedirs(a.out, exist_ok=True)
    jt, je, ju = (os.path.join(a.out, n) for n in ("trial_results.jsonl", "event_log.jsonl", "api_usage.jsonl"))
    if not a.resume and not a.estimate:
        for f in [jt, je, ju] + [os.path.join(a.out, x) for x in os.listdir(a.out) if x.startswith("lessons_") and x.endswith(".jsonl")]:
            if os.path.exists(f): os.remove(f)
    stores = {}
    realism = Realism(a.odom_noise, a.det_drop, a.img_noise, a.stale_view, moving_every=a.moving_every)
    done = {(r["task_id"], r["config"], int(r.get("rep", 0))) for r in read_jsonl(jt) if not r.get("error")} if a.resume else set()
    if a.resume:            # drop errored rows so they are re-run
        keep = [r for r in read_jsonl(jt) if not r.get("error")]
        with open(jt, "w", encoding="utf-8") as f: f.writelines(json.dumps(r) + "\n" for r in keep)
    est = {}; total_calls = 0; stopped = None; t_start = time.time()
    for i, (fam, s, yaw, n, m, lv) in enumerate(instances):
        for cfg in cfgs:
            for rep in range(a.repeats):
                world, task = make_task(fam, s, yaw, n, m, lv, a.room_scale)          # identical world for every config (paired)
                if a.sim != "world2d":                                   # v0.14: the same world inside a simulator backend
                    from orbit_ros.sim_select import in_simulator
                    world = in_simulator(world, a.sim, a.motion, ros2_config=a.ros2_config)
                world.safety_gate = a.safety_gate
                rz = dataclasses.replace(realism, **realism_for(fam, s - seed0, yaw, n, m, lv, a.scenario, a.seeds)) if a.scenario else realism
                if rz.active():
                    world.set_realism(dataclasses.replace(rz, seed=f"{task.task_id}|{rep}")); world.movable = movable_ids(world, task)
                rephrase(task, a.phrasing)
                if (task.task_id, cfg.name, rep) in done: continue
                if a.max_llm_calls and total_calls >= a.max_llm_calls: stopped = f"--max-llm-calls {a.max_llm_calls} reached"; break
                if a.max_cost and client is not None and (client.spent() is None or client.spent() >= a.max_cost):
                    stopped = f"--max-cost {a.max_cost} reached (spent {client.spent()})"; break
                if client is None: backend = OracleBackend(a.slip, a.half_confusion, a.deviation, a.ask_p, a.false_ask, abstain=not a.legacy_oracle)
                else:
                    mk = lambda cl: LLMBackend(cl, cfg.grounded, cfg.memory, not a.no_sensor_doc, not a.no_interp_view,
                                               extra_tools=(' | {"tool":"report","args":{"answer":str}} (states an answer to a question)' if fam == "report" else ""),
                                               interactive=not a.no_ask, nav_tools=a.nav_tools, hierarchical=a.hierarchical,
                                               localize_tool=a.localizer == "tool", vision=a.vision)
                    backend = mk(client) if step_client is None else Mixed(mk(client), mk(step_client))
                if client is not None and cfg.exec_mode == "pel" and a.pel_prompt == "v1":
                    from orbit_ros.pel.prompts import PelV1Backend; backend = PelV1Backend(backend)
                if a.self_consistency > 1: backend = SelfConsistent(backend, a.self_consistency)
                if a.token_budget and client is not None: backend = TokenBudget(backend, a.token_budget)
                if a.episode_timeout: backend = Deadline(backend, a.episode_timeout)
                events = []                                              # always collected (lessons need the trace); written only with --events
                usage_before = len(backend.usage_events) if client is not None else 0
                lessons = None
                if cfg.lessons:
                    store = stores.setdefault(cfg.name, LessonStore(None if a.estimate else os.path.join(a.out, "lessons_" + cfg.name.replace("+", "_") + ".jsonl")))
                    lessons = store.retrieve(task, a.lesson_k)
                try:
                    row = run_episode(world, task, cfg, backend, events, window=a.window, max_actions=a.max_actions, lessons=lessons, nav_tools=a.nav_tools,
                                      localizer=a.localizer, visual_reloc=a.visual_reloc, horizon=a.horizon, fail_closed=a.fail_closed,
                                      **({"pel_state_dir": a.pel_state_dir, "pel_rep": rep} if a.pel_state_dir else {}))
                except Exception as e:                                  # keep failures, never drop trials
                    row = {"task_id": task.task_id, "family": fam, "config": cfg.name, **task.factors,
                           "success": 0, "error": repr(e)[:300], "llm_calls": getattr(backend, "calls", 0), "elapsed_s": ""}
                row["rep"] = rep; row["phrasing"] = a.phrasing
                if a.scenario: row["scenarios"] = " ".join(scenarios_of(fam, s - seed0, yaw, n, m, lv, a.scenario, a.seeds))
                tb, dl, sc = (unwrap(backend, k) for k in (TokenBudget, Deadline, SelfConsistent))
                if tb is not None: row["token_budget_exhausted"] = int(tb.exhausted)
                if dl is not None: row["timed_out"] = int(dl.timed_out)
                if row.get("error") and "CostCapReached" in str(row["error"]):        # v0.14: refused by the spending cap, not a task failure
                    row["outcome_code"] = "cost_cap_reached"; stopped = f"--max-cost {a.max_cost} reached during an episode"
                elif row.get("error"): row["outcome_code"] = "infra_error"
                elif not row.get("success") and dl is not None and dl.timed_out: row["outcome_code"] = "timeout"
                elif not row.get("success") and tb is not None and tb.exhausted: row["outcome_code"] = "token_budget_exhausted"
                if sc is not None:
                    row["sc_votes"] = len(sc.votes); row["sc_mean_agreement"] = round(sum(v / k for v, k in sc.votes) / len(sc.votes), 3) if sc.votes else ""
                if cfg.lessons and not row.get("success") and not row.get("error"):      # turn the failure into a lesson for later episodes
                    from failure_taxonomy import classify
                    cls, why = classify(row)
                    if client is None: text, effect = template_lesson(cls)
                    else:
                        trace = [json.loads(e["details"]) | {"event": e["event"]} for e in events if e["event"] in ("action", "ask", "abstain", "gate_rejected")]
                        text, effect = backend.write_lesson(task, row, trace), ""; row["lesson_calls"] = 1
                    if text:
                        les = store.add({"task_id": task.task_id, "rep": rep, "family": fam, "level": lv, "yaw": yaw,
                                         "instruction": " ".join(p.text for p in task.phases), "failure_class": cls, "text": text, "effect": effect})
                        row["lesson_written"] = les["id"]
                total_calls += int(row.get("llm_calls") or 0) + int(row.get("lesson_calls") or 0)
                if a.estimate:
                    e = est.setdefault(cfg.name, [0, 0]); e[0] += 1; e[1] += int(row.get("llm_calls") or 0); continue
                with open(jt, "a", encoding="utf-8") as f: f.write(json.dumps(row, default=str) + "\n")
                if events and a.events:
                    with open(je, "a", encoding="utf-8") as f: f.writelines(json.dumps({**ev, "rep": rep}) + "\n" for ev in events)
                if client is not None:
                    with open(ju, "a", encoding="utf-8") as f:
                        for call_index, u in enumerate(backend.usage_events[usage_before:], start=1):
                            inp, out = u.get("input_tokens"), u.get("output_tokens")
                            cl, mid = (step_client, a.step_model) if u.get("role") == "stepper" else (client, a.model)
                            f.write(json.dumps({"task_id": task.task_id, "family": fam, "config": cfg.name, "rep": rep,
                                "provider": getattr(cl, "provider", "unknown"), "model_id": mid, "role": u.get("role", "planner"),
                                "cached_tokens": "" if u.get("cached_tokens") is None else u["cached_tokens"], "images": u.get("images", 0),
                                "call_index_in_episode": call_index, "tag": u.get("tag", ""),
                                "input_tokens": "" if inp is None else inp, "output_tokens": "" if out is None else out,
                                "reasoning_tokens": "" if u.get("reasoning_tokens") is None else u["reasoning_tokens"],
                                "total_tokens": (inp + out) if isinstance(inp, int) and isinstance(out, int) else "",
                                "latency_s": u.get("latency_s", ""), "finish_reason": u.get("finish_reason") or "", "attempts": u.get("attempts", 1),
                                "usage_metadata_available": int(isinstance(inp, int) and isinstance(out, int))}) + "\n")
            if stopped: break
        if stopped: break
        print(f"\rinstance {i+1}/{len(instances)}  model calls so far {total_calls}", end="", file=sys.stderr)
    print(file=sys.stderr)
    if a.estimate:
        print(f"Estimate for {len(instances)} instances x {a.repeats} repeat(s), counted with the ORACLE (a real model usually needs more calls: retries, replans):")
        for k, (eps, calls) in est.items(): print(f"  {k:<5} {eps:>5} episodes  {calls:>7} model calls  ({calls / max(1, eps):.1f} per episode)")
        print(f"  total {sum(v[1] for v in est.values())} model calls")
        return
    rows = read_jsonl(jt)
    write_csv(os.path.join(a.out, "trial_results.csv"), rows)
    if a.events: write_csv(os.path.join(a.out, "event_log.csv"), read_jsonl(je))
    if client is not None: write_csv(os.path.join(a.out, "api_usage.csv"), read_jsonl(ju))
    prompt_hash = hashlib.sha256((CONVENTIONS + TOOLS + open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "orbit_ros", "backends2d.py"), encoding="utf-8").read()).encode()).hexdigest()[:16]
    with open(os.path.join(a.out, "run_metadata.json"), "w") as f:
        json.dump({"orbit_ros_version": VERSION, "utc": datetime.now(timezone.utc).isoformat(), "python": sys.version, "platform": platform.platform(),
                   "planner": a.planner, "model": a.model if client else "OracleBackend-v1", "temperature": a.temperature,
                   "client_settings": client.settings() if client is not None and hasattr(client, "settings") else None,
                   "slip_p": None if client else a.slip, "half_confusion_p": None if client else a.half_confusion,
                   "deviation_p": None if client else a.deviation, "ask_p": None if client else a.ask_p, "false_ask_p": None if client else a.false_ask,
                   "legacy_oracle": a.legacy_oracle if not client else None, "ask_abstain_tools_offered": not a.no_ask, "nav_tools": a.nav_tools, "hierarchical": a.hierarchical,
                   "room_scale": a.room_scale, "realism": realism.as_dict(), "localizer": a.localizer, "visual_reloc": a.visual_reloc,
                   "self_consistency": a.self_consistency, "step_planner": a.step_planner, "step_model": a.step_model if step_client else None,
                   "token_budget": a.token_budget, "vision": a.vision, "scenarios": a.scenario,
                   "horizon": a.horizon, "safety_gate": a.safety_gate, "fail_closed": a.fail_closed, "episode_timeout_s": a.episode_timeout,
                   "heldout_seeds": a.heldout, "seed_offset": seed0, "config_file": a.config, **run_manifest(),
                   **({"sim": a.sim, "motion": a.motion} if a.sim != "world2d" else {}),
                   **({"pel_prompt": a.pel_prompt} if a.pel_prompt != "legacy" else {}), **({"pel_state_dir": a.pel_state_dir} if a.pel_state_dir else {}),
                   **({"max_cost": a.max_cost, "cost_currency": getattr(a, "cost_currency", None), "pricing_file": a.pricing or "configs/pricing.json",
                       "cost_spent_estimate": client.spent() if client is not None else None} if a.max_cost else {}),
                   "seeds_per_cell": a.seeds, "repeats": a.repeats, "instances": len(instances), "limit": a.limit, "per_family": a.per_family, "suite": a.suite,
                   "configs": a.configs, "families": a.families, "phrasing": a.phrasing, "window": a.window, "max_actions": a.max_actions,
                   "ablation": {"sensor_doc": not a.no_sensor_doc, "interp_view": not a.no_interp_view},
                   "episodes_written": len(rows), "model_calls_this_invocation": total_calls, "stopped_early": stopped,
                   "wall_clock_s_this_invocation": round(time.time() - t_start, 1), "backend_source_hash": prompt_hash,
                   "NOTE": "oracle = synthetic error model, harness validation only" if not client else "real LLM run"}, f, indent=2)
    print(f"wrote {len(rows)} episodes to {a.out}/" + (f"  (STOPPED EARLY: {stopped}; re-run with --resume to continue)" if stopped else ""))
if __name__ == "__main__": main()
