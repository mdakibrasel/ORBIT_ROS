"""Configurable agent architectures (ablation ladder S0-S5, paper arms C1-C4) and the episode runner."""
from __future__ import annotations
import json, random, re, time
from collections import Counter, deque
from dataclasses import dataclass
from .cognition import SensorInterpreter, Memory, verify_plan, AGENT_TOOLS, dry_run as verify_dry_run
from .tasks2d import check_success
from .embodiment_runtime import EmbodimentAdapter, hypothesis_of
from .navtools import MACROS, expand as expand_macro

@dataclass(frozen=True)
class AgentConfig:
    name: str
    reactive: bool      # True: one LLM decision per step, no stored plan (ReAct-like)
    grounded: bool      # sensor documentation + interpreter (camera semantics, frame conventions)
    memory: bool        # persistent landmark / cleaned-cell memory + `remember` tool
    verify: bool        # deterministic dry-run contract check before execution
    repair: bool        # runtime monitor: failure / expect-mismatch -> repair from current state
    desc: str = ""
    # Execution mode. "" derives it from `reactive` (S0 -> "reactive", S1-S5 -> "plan").
    #   reactive    : one tool call per model call, no plan object (paper C1)
    #   transient   : the model writes a whole plan at every step, only its first action runs, the plan is discarded (paper C2)
    #   scratchpad  : plan written once per instruction and kept with a cursor; the model picks every call and may deviate (paper C3)
    #   constrained : as scratchpad, but a runtime gate admits only the next planned step (or `replan`) (paper C4, MetaTool)
    #   plan        : the stored plan is executed by the runtime without a per-step model call (S1-S5)
    #   pel         : v0.14 ORBIT-PEL: the stored plan is executed step by step by the guarded execution manager (orbit_ros/pel/manager.py)
    mode: str = ""
    lessons: bool = False     # v0.11 lesson memory: lessons from earlier failures are retrieved and shown (S6, or any config + "+L")
    verify_retries: int = 0   # S4 study: plan-time re-planning attempts after a verifier rejection when repair is off (0 = abort, original S4)
    feedback: bool = False    # v0.13 (C5): a failed call or a pose that differs from the plan's prediction triggers an automatic re-plan
    pel_off: str = ""         # v0.14 PEL ablation: the one PEL component switched off ("registry" | "guard" | "verify" | "replan"; "" = none)

    @property
    def exec_mode(self) -> str: return self.mode or ("reactive" if self.reactive else "plan")
    @property
    def stepwise(self) -> bool: return self.exec_mode != "plan"
    @property
    def persistent(self) -> bool: return self.exec_mode not in ("reactive", "transient")

PRESETS = {c.name: c for c in [
    AgentConfig("S0", True,  False, False, False, False, "reactive ReAct-like, last-6-step window"),
    AgentConfig("S1", False, False, False, False, False, "persistent plan (MetaTool-style), no safeguards"),
    AgentConfig("S2", False, True,  False, False, False, "S1 + sensor/frame grounding"),
    AgentConfig("S3", False, True,  True,  False, False, "S2 + persistent memory"),
    AgentConfig("S4", False, True,  True,  True,  False, "S3 + dry-run plan verification"),
    AgentConfig("S5", False, True,  True,  True,  True,  "S4 + runtime-triggered repair"),
    # Paper arms (Mahmud et al., Table IV). Same tools, same ungrounded camera and no landmark memory as S0/S1, so C1-C4 differ ONLY in
    # how the plan is kept and whether it constrains tool calls. Every C arm asks the model once per executed tool call.
    AgentConfig("C1", True,  False, False, False, False, "paper C1: prompt only, no plan, no persistent state", "reactive"),
    AgentConfig("C2", False, False, False, False, False, "paper C2: plan regenerated at every step, never stored", "transient"),
    AgentConfig("C3", False, False, False, False, False, "paper C3: persistent scratchpad plan, no execution constraint", "scratchpad"),
    AgentConfig("C4", False, False, False, False, False, "paper C4: persistent scratchpad + plan-constrained execution", "constrained"),
    # v0.13: C4 + execution feedback. The gate still admits only the next planned step, but a failed call or an unexpected pose
    # (compared with the dry-run prediction of the plan prefix) makes the runtime ask for a new plan at once.
    AgentConfig("C5", False, False, False, False, False, "C4 + execution-feedback re-planning (failed call or unexpected pose)", "constrained", feedback=True),
    # v0.11: S1 + lesson memory. Same ungrounded camera and no landmark memory as S1: can lessons written after failures replace engineered grounding?
    AgentConfig("S6", False, False, False, False, False, "S1 + lesson memory (lessons from earlier failures)", lessons=True),
    # v0.14 ORBIT-PEL. Same ungrounded camera and no landmark memory as the C arms; the planner is asked for a whole plan (plan, repair,
    # assess calls only) and every step goes through schema -> registry -> safety guard -> robot -> fresh observation -> verification.
    # PEL-<component> switches that one component off (ablation). Not in the default config list of any runner.
    AgentConfig("PEL", False, False, False, False, False, "ORBIT-PEL: validated, guarded, verified step execution with bounded re-planning", "pel"),
    AgentConfig("PEL-registry", False, False, False, False, False, "PEL without the tool registry (no typed validation)", "pel", pel_off="registry"),
    AgentConfig("PEL-guard", False, False, False, False, False, "PEL without the safety guard", "pel", pel_off="guard"),
    AgentConfig("PEL-verify", False, False, False, False, False, "PEL without outcome verification", "pel", pel_off="verify"),
    AgentConfig("PEL-replan", False, False, False, False, False, "PEL without re-planning (fails at the first error)", "pel", pel_off="replan"),
]}
S_LADDER = ("S0", "S1", "S2", "S3", "S4", "S5")
C_LADDER = ("C1", "C2", "C3", "C4", "C5")
PEL_LADDER = ("PEL", "PEL-registry", "PEL-guard", "PEL-verify", "PEL-replan")

def config_by_name(name: str) -> AgentConfig:
    """PRESETS plus parametric study variants: S4v<k> = S4 with k verifier-driven re-planning attempts (no runtime repair)."""
    if name in PRESETS: return PRESETS[name]
    if name.endswith("+L"):                     # any configuration + lesson memory, e.g. C4+L, S5+L
        import dataclasses
        base = config_by_name(name[:-2]); return dataclasses.replace(base, name=name, lessons=True, desc=base.desc + " + lesson memory")
    m = re.fullmatch(r"S4v(\d+)", name)
    if m:
        k = int(m.group(1)); base = PRESETS["S4"]
        return AgentConfig(name, False, True, True, True, False, f"S4 + up to {k} verifier-driven re-plans (no runtime repair)", verify_retries=k)
    raise KeyError(f"unknown config {name}; choose from {list(PRESETS)} or S4v<k>")

WINDOW, MAX_ACTIONS, MAX_REPLANS, MAX_REPAIRS, MAX_VERIFY = 6, 150, 6, 5, 3
MAX_ASKS = 2          # clarification questions per instruction (v0.11)
NO_CLARIFICATION = "Nothing to clarify: the instruction means exactly what it says."
DONE = {"tool": "done", "args": {}}

OUTCOME_CODES = ("success", "refused", "observation_failure", "step_budget_exhausted", "unsafe_failure", "invalid_call_failure", "task_failure",
                 "token_budget_exhausted", "timeout", "infra_error", "cost_cap_reached")   # the last four are set by the runner (docs/OUTCOME_CODES.md)

def outcome_code(row) -> str:
    """One code per episode, first match wins: success, refused (abstained), observation_failure (fail-closed stop),
    step_budget_exhausted (tool-call cap reached), unsafe_failure (collision or forbidden cell), invalid_call_failure (>= 1 invalid call),
    task_failure (anything else)."""
    if row.get("success"): return "success"
    if row.get("abstained"): return "refused"
    if row.get("observation_failure"): return "observation_failure"
    if (row.get("env_tool_calls") or 0) >= (row.get("tool_call_cap") or 1e9): return "step_budget_exhausted"
    if (row.get("safety_violations") or 0) > 0: return "unsafe_failure"
    if (row.get("invalid_calls") or 0) > 0: return "invalid_call_failure"
    return "task_failure"

def normalize_action(a):
    """Model output -> {"tool": str, "args": dict}. Anything malformed becomes a call to a non-existent tool (counted as invalid)."""
    if not isinstance(a, dict) or not isinstance(a.get("tool"), str): return {"tool": "__malformed__", "args": {"raw": str(a)[:120]}}
    args = a.get("args")
    if args is None: args = {}
    if not isinstance(args, dict): return {"tool": "__malformed__", "args": {"raw": str(a)[:120]}}
    return {**a, "tool": a["tool"], "args": args}

def normalize_plan(p):
    if isinstance(p, list): p = {"actions": p}
    if not isinstance(p, dict): return {"actions": [], "malformed": True}
    if isinstance(p.get("regions"), list) and not p.get("actions"):      # hierarchical plan (--hierarchical): room -> regions -> steps
        flat = []
        for k, reg in enumerate(p["regions"]):
            if not isinstance(reg, dict): continue
            name = str(reg.get("name", f"region {k}"))[:60]
            steps = reg.get("actions") if isinstance(reg.get("actions"), list) else reg.get("steps") if isinstance(reg.get("steps"), list) else []
            flat += [{**normalize_action(a), "_region": name} for a in steps]
        return {**p, "actions": flat}
    acts = p.get("actions")
    if not isinstance(acts, list): acts = []
    return {**p, "actions": [normalize_action(a) for a in acts]}

def _same_call(a, b) -> bool:
    return a is not None and b is not None and a.get("tool") == b.get("tool") and \
        json.dumps(a.get("args") or {}, sort_keys=True) == json.dumps(b.get("args") or {}, sort_keys=True)

def run_episode(world, task, cfg: AgentConfig, backend, events=None, embodiment=None, window: int = WINDOW, max_actions: int = MAX_ACTIONS,
                lessons=None, nav_tools: bool = False, localizer: str = "none", visual_reloc: bool = False,
                horizon: int = 0, fail_closed: bool = False, pel_state_dir: str | None = None, pel_rep: int = 0):
    """embodiment=None keeps the original behaviour (agent is told the camera truth, actuators are nominal).
    embodiment=EmbodimentStrategy: the agent only *believes* an embodiment (identified or assumed); commands go
    through an adapter built from that belief, the interpreter uses the believed camera convention.
    window: length of the recent-action history shown to step-wise agents (S0, C1-C4); ablation knob.
    nav_tools: the goto_cell / cover_region macro tools are available (navtools.py); otherwise they are unknown tools.
    lessons: retrieved lesson dicts (lessons.LessonStore.retrieve) shown to the planner; None/[] = no lessons.
    localizer: "none" | "tool" (the model may call `localize`) | "auto" (the runtime localises before every observation after a move).
    visual_reloc: grounded configs (S2+) correct their odometry from landmarks seen again at a shifted cell (>= 2 agreeing landmarks,
    or one landmark shifted the same way in two consecutive observations).
    horizon: plan modes (S1-S6) execute at most this many actions of a plan, then observe and re-plan (0 = the whole plan, open loop).
    fail_closed: a stale camera frame is re-requested twice; if it is still stale the robot is stopped and the episode ends
    (outcome_code observation_failure).
    pel_state_dir: PEL configs only: write the orbit.exec_state/v1 record of every instruction there (atomic, after every step).
    pel_rep: repeat index, part of the state file name (v0.14.1) so repeats of the same episode do not overwrite each other.
    max_actions: per-instruction step budget of step-wise agents; the episode-wide tool-call cap is twice this (default 150 / 300)."""
    t0 = time.perf_counter()
    truth0 = hypothesis_of(world)
    backend.begin_episode(f"{task.task_id}|{cfg.name}")
    calls0, usage0 = backend.calls, len(backend.usage_events)
    parse0 = getattr(backend, "parse_errors", 0)
    rng = random.Random(f"interp|{task.task_id}|{cfg.name}")
    adapter = None
    if embodiment is not None:
        embodiment.start(world); adapter = EmbodimentAdapter(embodiment.belief)
    interp = SensorInterpreter(world.fov, embodiment.belief.mirrored_camera if embodiment else world.mirrored, cfg.grounded, rng=rng)
    st = {"reid": False, "last_call": None, "rep0": 0, "exec0": 0, "budget": None, "phase": None, "asks_phase": 0, "abstain": False}
    clar = []             # clarification dialogue of this episode: [{"question", "answer"}]
    def adopt_belief():
        adapter.belief = embodiment.belief; interp.mirrored = embodiment.belief.mirrored_camera
    mem, own_cleaned, recent_clean = Memory(), set(), deque(maxlen=6)
    history = deque(maxlen=max(0, window)); blocked_known = set()
    c = dict(repairs=0, verify_fail=0, replans=0, aborted=0, executed=0, repeated=0, proposals=0, gate_rejections=0, deviations=0,
             asks=0, asks_needed=0, unnecessary_asks=0, ask_needed_phases=0, horizon_replans=0, feedback_replans=0, stale_retries=0)
    def ev(name, **d):
        if events is not None: events.append({"task_id": task.task_id, "config": cfg.name, "event": name, "details": json.dumps(d, default=str)})

    world.localizer = localizer != "none"
    st.update(moved=False, anchors={}, pending=None)

    def relocalise(raw, pose, it):
        """Visual re-localisation: a landmark first seen at cell A and now seen at cell B means the pose estimate is off by A - B."""
        counts = Counter(o["label"] for o in it)
        uniq = [o for o in it if counts[o["label"]] == 1]
        votes = Counter((st["anchors"][o["label"]][0] - o["world_xy"][0], st["anchors"][o["label"]][1] - o["world_xy"][1])
                        for o in uniq if o["label"] in st["anchors"])
        for o in uniq: st["anchors"].setdefault(o["label"], tuple(o["world_xy"]))
        if not votes: st["pending"] = None; return pose, it
        (d, n), zero = votes.most_common(1)[0], votes.get((0, 0), 0)
        ok = d != (0, 0) and n > zero and (n >= 2 or (len(votes) == 1 and st["pending"] == d))
        st["pending"] = d if d != (0, 0) and not ok else None
        if not ok: return pose, it
        world.correct_odometry(*d); ev("visual_relocalisation", shift=d, landmarks=n)
        pose = world.execute("get_pose", {})["data"]; return pose, interp.interpret(raw, pose)

    def observe(phase_start, fresh):
        if localizer == "auto" and st["moved"]: world.execute("localize", {}); st["moved"] = False; ev("auto_localise")
        pose = world.execute("get_pose", {})["data"]; raw = world.execute("describe_view", {})["data"]
        if fail_closed and getattr(world, "last_view_stale", False):
            for _ in range(2):
                c["stale_retries"] += 1; raw = world.execute("describe_view", {})["data"]
                if not world.last_view_stale: break
            else:
                if not st.get("obs_fail"):
                    st["obs_fail"] = st["abstain"] = True; world.execute("stop_robot", {}); ev("observation_failure", reason="camera frame stale 3 times")
        if st.get("obs_fail"):                     # the robot is stopped: its tools refuse, so the context carries the last estimate and no view
            pose = pose or world.pose(); raw = raw if raw is not None else []
        it = interp.interpret(raw, pose)
        if visual_reloc and cfg.grounded: pose, it = relocalise(raw, pose, it)
        if cfg.memory: mem.seen |= {tuple(o["world_xy"]) for o in it}
        obst = {tuple(o["world_xy"]) for o in it} | blocked_known | (mem.seen if cfg.memory else set())
        if cfg.memory: known = set(mem.cleaned)
        elif not cfg.persistent: known = set(recent_clean)       # S0/C1/C2 keep no state beyond the recent window
        else: known = set(own_cleaned)
        return {"pose": pose, "raw": raw, "interp": it, "obstacles_known": sorted(obst), "landmarks": dict(mem.landmarks), "cleaned_known": known,
                "start_pose": world.start_pose, "phase_start_pose": phase_start, "history": list(history) if cfg.stepwise else [],
                "use_memory": cfg.memory, "reactive": not cfg.persistent, "grounded": cfg.grounded,
                "sensor_spec": (world.sensor_spec(embodiment.belief.mirrored_camera if embodiment else None) if cfg.grounded else None),
                "embodiment_card": embodiment.card() if embodiment else None, "fresh_phase": fresh,
                "reported": list(world.reports[st["rep0"]:]), "clarifications": list(clar),
                "nav_tools": nav_tools, "localizer": localizer, "pose_stale": localizer == "tool" and st["moved"], "lessons": [l["text"] for l in (lessons or [])], "lesson_effects": {l.get("effect") for l in (lessons or []) if l.get("effect")}}

    def cut():
        """Mid-course change: the current instruction is interrupted after `interrupt_after` executed actions."""
        if st["budget"] is not None and c["executed"] - st["exec0"] >= st["budget"]:
            if not st.get("cut_logged"): st["cut_logged"] = True; ev("instruction_interrupted", after_actions=st["budget"])
            return True
        return False

    def do(a, ctx):
        a = normalize_action(a)
        tool, args = a.get("tool"), a.get("args") or {}
        c["executed"] += 1
        # Repeated call (paper metric, our operational rule): same tool and arguments as the previous agent call, issued from the same
        # robot state (pose, stop flag, number of cleaned cells), i.e. a call that cannot yield anything the previous one did not.
        key = (tool, json.dumps(args, sort_keys=True, default=str), world.x, world.y, world.yaw, world.stopped, len(world.cleaned), len(mem.landmarks))
        if key == st["last_call"]: c["repeated"] += 1; ev("repeated_call", tool=tool, args=args)
        st["last_call"] = key
        if tool in ("ask", "abstain"):
            world.tool_calls += 1; ph = st["phase"]
            if tool == "abstain":
                world.abstained = st["abstain"] = True; r = {"ok": True, "message": "abstained: the task is not carried out"}
                ev("abstain", reason=args.get("reason", ""))
            elif st["asks_phase"] >= MAX_ASKS:
                r = {"ok": False, "message": "no more questions allowed; act on what you know"}
            else:
                st["asks_phase"] += 1; c["asks"] += 1
                needed = ph is not None and ph.params.get("clarify_answer") is not None
                c["asks_needed" if needed else "unnecessary_asks"] += 1
                ans = ph.params["clarify_answer"] if needed else NO_CLARIFICATION
                clar.append({"question": str(args.get("question", ""))[:300], "answer": ans}); r = {"ok": True, "message": "answer: " + ans}
                ev("ask", question=args.get("question", ""), answer=ans, needed=needed)
            history.append({"action": a, "ok": r["ok"], "msg": r["message"]})
            return r
        if tool == "remember":
            if not cfg.memory: world.invalid_calls += 1; world.tool_calls += 1; r = {"ok": False, "message": "no memory tool"}
            else:
                world.tool_calls += 1
                ok = mem.remember(args.get("label"), ctx["interp"]); r = {"ok": ok, "message": "remembered" if ok else "label not visible"}
            history.append({"action": a, "ok": r["ok"], "msg": r["message"]}); ev("action", tool=tool, args=args, ok=r["ok"], msg=r["message"])
            return r
        if tool in MACROS and nav_tools:
            known_clean = set(mem.cleaned) if cfg.memory else (set(recent_clean) if not cfg.persistent else set(own_cleaned))
            prims, err = expand_macro(tool, args, world.pose(), world.start_pose, ctx["obstacles_known"], known_clean)
            if err: world.invalid_calls += 1; world.tool_calls += 1; r = {"ok": False, "message": err}
            else:
                r = {"ok": True, "message": "", "data": None}
                for p_ in prims:
                    r = prim(p_["tool"], p_["args"])
                    if not r["ok"]: break
                r = {"ok": r["ok"], "message": f"{tool}: {len(prims)} robot actions, done" if r["ok"] else f"{tool} stopped: {r['message']}",
                     "data": world.pose()}
            history.append({"action": a, "ok": r["ok"], "msg": r["message"]}); ev("action", tool=tool, args=args, ok=r["ok"], msg=r["message"])
            return r
        if tool not in AGENT_TOOLS:
            world.invalid_calls += 1; world.tool_calls += 1; r = {"ok": False, "message": f"unknown tool {tool}"}
            history.append({"action": a, "ok": False, "msg": r["message"]}); ev("action", tool=tool, args=args, ok=False, msg=r["message"])
            return r
        if a.get("_region") and a["_region"] != st.get("region"): st["region"] = a["_region"]; ev("region_start", name=a["_region"])
        r = prim(tool, args)
        history.append({"action": a, "ok": r["ok"], "msg": r["message"]})
        ev("action", tool=tool, args=args, ok=r["ok"], msg=r["message"])
        return r

    def prim(tool, args):
        """One primitive robot call (through the embodiment adapter when present) + cleaned/blocked bookkeeping."""
        if adapter is None:
            pre = world.pose(); r = world.execute(tool, args)
            if localizer == "auto" and tool == "move_forward" and r["ok"]:     # closed-loop execution: localise after every move
                from .world2d import heading as _h
                dx, dy = _h(pre["yaw_deg"]); n = args.get("distance", 0)
                post = world.execute("localize", {})["data"]; st["moved"] = False
                if (post["x"], post["y"]) != (pre["x"] + dx * n, pre["y"] + dy * n):
                    ev("odometry_drift_detected", expected=[pre["x"] + dx * n, pre["y"] + dy * n], localised=[post["x"], post["y"]])
                    r = {"ok": False, "message": f"localiser: ended at ({post['x']},{post['y']}), expected ({pre['x'] + dx * n},{pre['y'] + dy * n})", "data": post}
                return r
        else:
            for hw_tool, hw_args in adapter.translate(tool, args):
                pre = world.pose(); r = world.execute(hw_tool, hw_args)
                if embodiment.observe_motion(world, hw_tool, hw_args, pre, r): adopt_belief(); st["reid"] = True; ev("embodiment_reidentified", card=embodiment.card())
                if not r["ok"]: break
        if tool == "move_forward": st["moved"] = True
        if tool == "localize" and r["ok"]: st["moved"] = False
        if tool == "clean" and r["ok"]:
            _p = world.pose(); cell = (_p["x"], _p["y"])           # the agent's belief (odometry), not the truth
            own_cleaned.add(cell); recent_clean.append(cell); mem.cleaned.add(cell)
        if tool == "move_forward" and not r["ok"] and str(r["message"]).startswith(("blocked", "safety stop")) and r["data"]:
            from .world2d import heading as _h
            dx, dy = _h(r["data"]["yaw_deg"]); blocked_known.add((r["data"]["x"] + dx, r["data"]["y"] + dy))
        return r

    def believed(ctx): return [tuple(o) for o in ctx["obstacles_known"]]

    def verified(ph, plan, ctx):
        for attempt in range(max(MAX_VERIFY, cfg.verify_retries + 1)):
            errs = verify_plan(plan, ctx["pose"], believed(ctx), world.start_pose) if cfg.verify else []
            if not errs: return plan
            c["verify_fail"] += 1; ev("verify_failed", errors=errs)
            # S4 is verification-only: it must not silently gain S5's repair capability. S4v<k> may re-plan k times from the verifier's errors.
            if not cfg.repair and attempt >= cfg.verify_retries:
                c["aborted"] += 1; ev("aborted_unverified", repair_enabled=False); return None
            plan = normalize_plan(backend.repair(ph, ctx, "verification errors: " + "; ".join(errs[:3])))
            c["repairs"] += 1
        c["aborted"] += 1; ev("aborted_unverified"); return None

    def stepwise_phase(ph, phase_start):
        """C1-C4 and S0: the model is asked once per executed tool call."""
        mode, plan, cursor, replans = cfg.exec_mode, None, 0, 0
        if mode in ("scratchpad", "constrained"):
            ctx0 = observe(phase_start, True); plan = normalize_plan(backend.plan(ph, ctx0)); ev("scratchpad_written", n_actions=len(plan["actions"]))
            plan_pose = ctx0["pose"]; fb = 0
        for _ in range(max_actions):
            if world.tool_calls > 2 * max_actions: break
            ctx = observe(phase_start, False)
            if st["abstain"]: break                    # fail-closed stop during this observation
            if mode == "reactive":
                a = normalize_action(backend.next_action(ph, ctx))
            elif mode == "transient":
                acts = normalize_plan(backend.plan_step(ph, ctx))["actions"]
                a = acts[0] if acts else DONE
            else:
                acts = plan["actions"]; planned = acts[cursor] if cursor < len(acts) else None
                a = normalize_action(backend.scratchpad_step(ph, ctx, acts, cursor, mode == "constrained"))
                c["proposals"] += 1
                if a["tool"] == "replan" or (mode == "constrained" and planned is None and a["tool"] != "done"):
                    if a["tool"] != "replan": c["gate_rejections"] += 1; ev("gate_rejected", proposed=a, planned=None, note="plan finished: only done/replan admissible")
                    if replans >= MAX_REPLANS: break
                    replans += 1; c["replans"] += 1; ev("replan", reason=(a.get("args") or {}).get("reason", ""))
                    ctxr = observe(phase_start, False); plan = normalize_plan(backend.plan(ph, ctxr)); cursor = 0; plan_pose = ctxr["pose"]
                    continue
                if planned is not None and not _same_call(a, planned):
                    if mode == "constrained":
                        c["gate_rejections"] += 1; ev("gate_rejected", proposed=a, planned=planned); a = planned
                    else:
                        c["deviations"] += 1; ev("plan_deviation", proposed=a, planned=planned, cursor=cursor)
                if planned is not None and _same_call(a, planned): cursor += 1
            if a["tool"] == "done" or cut(): break
            r = do(a, ctx)
            if st["abstain"]: break
            if cfg.feedback and mode == "constrained" and fb < MAX_REPAIRS:
                why = None
                if not r.get("ok", True): why = f"{a['tool']} failed: {r.get('message')}"
                elif a["tool"] in ("move_forward", "turn"):
                    pred, _, _ = verify_dry_run(plan["actions"][:cursor], plan_pose, [tuple(o) for o in ctx["obstacles_known"]], world.start_pose)
                    now = world.pose()
                    if (pred["x"], pred["y"], pred["yaw_deg"]) != (now["x"], now["y"], now["yaw_deg"]):
                        why = f"pose ({now['x']},{now['y']},{now['yaw_deg']}) differs from the plan's prediction ({pred['x']},{pred['y']},{pred['yaw_deg']})"
                if why:
                    fb += 1; c["feedback_replans"] += 1; ev("feedback_replan", reason=why)
                    ctxr = observe(phase_start, False); plan = normalize_plan(backend.repair(ph, ctxr, why)); cursor = 0; plan_pose = ctxr["pose"]

    pel = None
    if cfg.exec_mode == "pel":
        from .pel.manager import ExecutionManager, PelFlags
        from .pel.registry import RegistryContext
        def pel_observe(phase_start, fresh):
            ctx = observe(phase_start, fresh); ctx["view_stale"] = bool(getattr(world, "last_view_stale", False)); return ctx
        pel = ExecutionManager(PelFlags.without(cfg.pel_off) if cfg.pel_off else PelFlags(), backend, pel_observe, do, ev,
                               budget_ok=lambda: world.tool_calls < 2 * max_actions and not cut() and not st["abstain"],
                               reg_ctx=lambda: RegistryContext(world.stopped, bool(world.localizer), cfg.memory, nav_tools, MAX_ASKS - st["asks_phase"]),
                               stopped=lambda: world.stopped, start_pose=world.start_pose, episode=task.task_id, config=cfg.name,
                               state_dir=pel_state_dir, rep=pel_rep)

    for i_ph, ph in enumerate(task.phases):
        start = world.pose(); phase_start = (start["x"], start["y"], start["yaw_deg"]); world.phase_log.append(phase_start)
        if st["abstain"]: break                       # an abstention ends the episode
        st.update(rep0=len(world.reports), exec0=c["executed"], budget=ph.params.get("interrupt_after"), cut_logged=False, phase=ph, asks_phase=0)
        if ph.params.get("clarify_answer") is not None: c["ask_needed_phases"] += 1
        if pel is not None:
            pel.run_phase(ph, phase_start, i_ph); continue
        if cfg.stepwise:
            stepwise_phase(ph, phase_start); continue
        ctx = observe(phase_start, True)
        plan = normalize_plan(backend.plan(ph, ctx))
        if cfg.verify: plan = verified(ph, plan, ctx)
        replans = repairs = 0
        while plan is not None and world.tool_calls < 2 * max_actions and not cut() and not st["abstain"]:
            failure = None; truncated = False
            for i_a, a in enumerate(plan.get("actions", [])):
                if cut() or st["abstain"]: break
                if horizon and i_a >= horizon: truncated = True; break      # closed loop: observe and re-plan after `horizon` actions
                r = do(a, ctx)
                if a.get("tool") == "ask" and r["ok"]:          # an answer makes the rest of a pre-committed plan stale: re-plan with it
                    if cfg.repair: failure = "clarification received: " + r["message"]
                    break
                if not r["ok"] and cfg.repair: failure = f"{a} failed: {r['message']}"; break
                if st["reid"]:
                    st["reid"] = False
                    if cfg.repair: failure = "embodiment surprise: actuator behaviour no longer matches the card; re-identified"; break
            if cut() or st["abstain"]: break
            ctx = observe(phase_start, False)
            e = plan.get("expect") or {}
            if not isinstance(e, dict): e = {}
            if failure is None and not truncated and cfg.repair and any(k in e and e[k] != ctx["pose"][k] for k in ("x", "y")):
                failure = f"final pose ({ctx['pose']['x']},{ctx['pose']['y']}) != expected ({e.get('x')},{e.get('y')})"
            if failure and embodiment is not None and embodiment.on_failure(world):
                adopt_belief(); ev("embodiment_identified_after_failure", card=embodiment.card())
            if failure:
                if repairs >= MAX_REPAIRS: break
                repairs += 1; c["repairs"] += 1; ev("repair", failure=failure)
                plan = normalize_plan(backend.repair(ph, ctx, failure))
            elif truncated:
                c["horizon_replans"] += 1; ev("horizon_replan", horizon=horizon)
                plan = normalize_plan(backend.plan(ph, ctx))
            else:
                if backend.assess(ph, ctx) or replans >= MAX_REPLANS: break
                replans += 1; c["replans"] += 1; ev("replan_after_exhaustion")
                plan = normalize_plan(backend.plan(ph, ctx))
            if plan is not None and cfg.verify: plan = verified(ph, plan, ctx)
    res = check_success(world, task)
    usage = backend.usage_events[usage0:]
    tin = [u.get("input_tokens") for u in usage]; tout = [u.get("output_tokens") for u in usage]
    lat = [u.get("latency_s") for u in usage if isinstance(u.get("latency_s"), (int, float))]
    elapsed = time.perf_counter() - t0
    n_agent = c["executed"]
    row = {"task_id": task.task_id, "family": task.family, "config": cfg.name, **task.factors,
           "success": int(res["success"]), "interpretation": res.get("interpretation", ""), "violations": res.get("violations", ""),
           "answer": res.get("answer", ""), "coverage": res.get("coverage", ""), "wrong_side": res.get("wrong_side", ""), "progress": res.get("progress", ""),
           "llm_calls": backend.calls - calls0, "env_tool_calls": world.tool_calls, "tool_call_cap": 2 * max_actions, "actions_executed": n_agent,
           "invalid_calls": world.invalid_calls, "invalid_call_pct": round(100 * world.invalid_calls / n_agent, 2) if n_agent else 0.0,
           "repeated_calls": c["repeated"], "repeated_call_pct": round(100 * c["repeated"] / n_agent, 2) if n_agent else 0.0,
           "proposals": c["proposals"], "gate_rejections": c["gate_rejections"], "plan_deviations": c["deviations"],
           "parse_errors": getattr(backend, "parse_errors", 0) - parse0,
           "lessons_used": len(lessons or []), "lesson_ids": " ".join(str(l["id"]) for l in (lessons or [])),
           "asks": c["asks"], "ask_needed_phases": c["ask_needed_phases"], "asks_when_needed": c["asks_needed"], "unnecessary_asks": c["unnecessary_asks"],
           "abstained": int(world.abstained), "wall_bumps": world.wall_bumps, "object_hits": world.object_hits,
           "safety_violations": world.wall_bumps + world.object_hits + (res.get("violations") or 0),
           "safe_success": int(res["success"] and world.wall_bumps + world.object_hits + (res.get("violations") or 0) == 0),
           "collisions": world.collisions, "repairs": c["repairs"],
           "verify_failures": c["verify_fail"], "replans": c["replans"], "aborted": c["aborted"],
           "input_tokens": sum(t for t in tin if t is not None) if any(t is not None for t in tin) else "",
           "output_tokens": sum(t for t in tout if t is not None) if any(t is not None for t in tout) else "",
           "elapsed_s": round(elapsed, 4), "llm_latency_s": round(sum(lat), 4) if lat else "",
           "harness_s": round(elapsed - sum(lat), 4),
           "final_x": world.x, "final_y": world.y, "final_yaw": world.yaw}
    row.update({"horizon_replans": c["horizon_replans"], "feedback_replans": c["feedback_replans"], "safety_rejections": world.safety_rejections, "backtracks": world.backtracks,
                "stale_retries": c["stale_retries"], "observation_failure": int(bool(st.get("obs_fail")))})
    row["outcome_code"] = outcome_code(row)
    if pel is not None:
        row.update(pel.c); row["pel_terminal"] = pel.state.terminal["code"] if pel.state is not None and pel.state.terminal else ""
    if localizer != "none" or visual_reloc:
        row.update({"localize_calls": world.localize_calls, "visual_corrections": world.visual_corrections})
    if world.realism.active():
        row.update({"odom_errors": world.odom_errors, "stale_views": world.stale_views, "dropped_detections": world.dropped_detections,
                    "object_moves": world.object_moves, "injected_failures": world.injected_failures, "final_pose_error_m": abs(world.est_x - world.x) + abs(world.est_y - world.y) if world.realism.odom_p else 0})
    if embodiment is not None:
        truth1 = hypothesis_of(world)
        row.update({"embodiment_method": embodiment.name, "true_embodiment": truth0.key, "final_true_embodiment": truth1.key,
                    "belief_key": embodiment.belief.key,
                    "belief_correct": int((embodiment.belief.mirrored_camera, embodiment.belief.left_right_swapped, round(embodiment.belief.move_gain, 3))
                                          == (truth1.mirrored_camera, truth1.left_right_swapped, round(truth1.move_gain, 3))),
                    "id_probes": embodiment.probes, "id_bay_actions": embodiment.bay_actions, "id_bay_travel": embodiment.bay_travel,
                    "reidentifications": embodiment.reids, "surprise_triggers": embodiment.triggers})
    return row
