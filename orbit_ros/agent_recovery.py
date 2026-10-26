"""Episode runner for v0.8.0: v0.6/0.7 runner + runtime execution monitor + failure recovery (instead of abstention).

Copy-and-modify of `agent_robust.run_episode_robust` (which stays byte-identical).  With `rcfg=None` (monitor and recovery off) it reproduces
`voi.run_episode_v07` episode for episode (tests/test_recovery.py checks this).  Changes are marked `# V08`.  See recovery.py for the ladder.
"""
from __future__ import annotations
import json, random, time
from collections import deque
from .cognition import SensorInterpreter, Memory, verify_plan, AGENT_TOOLS
from .tasks2d import check_success
from .embodiment_runtime import EmbodimentAdapter, hypothesis_of
from .agent2d import WINDOW, MAX_ACTIONS, MAX_REPLANS, MAX_REPAIRS, MAX_VERIFY
from .execution_monitor import ExecutionMonitor                                 # V08
from .recovery import RecoveryConfig, StepGuard                                  # V08

V08_KEYS = ("monitor_trips", "dev_trips", "blocked_trips", "rec_entries", "rec_reprobes", "rec_reprobe_resolved", "rec_guarded_phases", "guarded_steps",
            "guarded_creeps", "guarded_unsafe", "rec_entered_phases", "rec_resolved_phases", "rec_collisions", "rec_actions", "rec_llm_calls", "rec_env_calls")

def _run_v08(world, task, cfg, backend, strategy, rcfg, events=None):
    t0 = time.perf_counter()
    truth0 = hypothesis_of(world)
    backend.begin_episode(f"{task.task_id}|{cfg.name}")
    calls0, usage0 = backend.calls, len(backend.usage_events)
    rng = random.Random(f"interp|{task.task_id}|{cfg.name}")
    strategy.start(world); adapter = EmbodimentAdapter(strategy.belief)
    interp = SensorInterpreter(world.fov, strategy.belief.mirrored_camera, cfg.grounded, rng=rng)
    mem, own_cleaned, recent_clean = Memory(), set(), deque(maxlen=6)
    history = deque(maxlen=WINDOW); blocked_known = set()
    c = dict(repairs=0, verify_fail=0, replans=0, aborted=0, executed=0, probe_replans=0, robust_rejects=0)   # NEW: last two
    first_exec = {"n_plausible": None, "confidence": None}                                                        # NEW
    c.update({k: 0 for k in V08_KEYS})                                                                           # V08
    mon = ExecutionMonitor(enabled=rcfg.monitor or rcfg.enabled)                                                 # V08 (recovery implies monitoring)
    ph_state = {"guarded": False, "entered": False, "entries": 0, "calls0": 0, "env0": 0, "extra_used": False}                       # V08
    def ev(name, **d):
        if events is not None: events.append({"task_id": task.task_id, "config": cfg.name, "event": name, "details": json.dumps(d, default=str)})

    def adopt_belief():
        b = strategy.belief
        if b.mirrored_camera != interp.mirrored: mem.seen.clear()       # NEW: stale landmark cells from the wrong convention
        adapter.belief = strategy.cmd_belief; interp.mirrored = b.mirrored_camera

    def observe(phase_start, fresh):
        pose = world.execute("get_pose", {})["data"]; raw = world.execute("describe_view", {})["data"]
        it = interp.interpret(raw, pose)
        if cfg.memory: mem.seen |= {tuple(o["world_xy"]) for o in it}
        obst = {tuple(o["world_xy"]) for o in it} | blocked_known | (mem.seen if cfg.memory else set())
        if cfg.memory: known = set(mem.cleaned)
        elif cfg.reactive: known = set(recent_clean)
        else: known = set(own_cleaned)
        return {"pose": pose, "raw": raw, "interp": it, "obstacles_known": sorted(obst), "landmarks": dict(mem.landmarks), "cleaned_known": known,
                "start_pose": world.start_pose, "phase_start_pose": phase_start, "history": list(history) if cfg.reactive else [],
                "use_memory": cfg.memory, "reactive": cfg.reactive, "grounded": cfg.grounded,
                "sensor_spec": (world.sensor_spec(strategy.belief.mirrored_camera) if cfg.grounded else None),
                "embodiment_card": strategy.card(), "fresh_phase": fresh}

    def do(a, ctx):
        tool, args = a.get("tool"), a.get("args") or {}
        c["executed"] += 1
        if tool == "remember":
            if not cfg.memory: world.invalid_calls += 1; world.tool_calls += 1; return {"ok": False, "message": "no memory tool"}
            world.tool_calls += 1
            ok = mem.remember(args.get("label"), ctx["interp"]); return {"ok": ok, "message": "remembered" if ok else "label not visible"}
        if tool not in AGENT_TOOLS: world.invalid_calls += 1; world.tool_calls += 1; return {"ok": False, "message": f"unknown tool {tool}"}
        adapter.belief = strategy.cmd_belief                                                                    # NEW: robust translation
        cb = adapter.belief                                                                                      # V08: belief the commands were translated for
        for hw_tool, hw_args in ([(tool, args)] if a.get("hw") else adapter.translate(tool, args)):             # NEW: raw hw sub-moves
            pre = world.pose(); r = world.execute(hw_tool, hw_args); mon.after_hw(pre, hw_tool, hw_args, world.pose(), r, cb)   # V08
            if strategy.observe_command(pre, hw_tool, hw_args, world.pose(), r): adopt_belief(); ev("belief_changed", card=strategy.card())   # NEW
            if not r["ok"] or mon.tripped: break                                                                # V08: stop a translated move once contradicted
        if tool == "clean" and r["ok"]:
            cell = (world.x, world.y); own_cleaned.add(cell); recent_clean.append(cell); mem.cleaned.add(cell)
        if tool == "move_forward" and not r["ok"] and str(r["message"]).startswith("blocked") and r["data"]:
            from .world2d import heading as _h
            dx, dy = _h(r["data"]["yaw_deg"]); blocked_known.add((r["data"]["x"] + dx, r["data"]["y"] + dy))
        history.append({"action": a, "ok": r["ok"], "msg": r["message"]})
        ev("action", tool=tool, args=args, ok=r["ok"], msg=r["message"])
        return r

    def believed(ctx): return [tuple(o) for o in ctx["obstacles_known"]]

    # ---- NEW: plan-as-probe -------------------------------------------------------------------------
    def probe_loop(ph, plan, ctx, phase_start, force):
        """Execute safe, informative prefixes of the plan, update the belief, replan.  Returns (plan, ctx)."""
        while plan is not None and strategy.wants_probe():
            probe = strategy.plan_probe(plan, ctx, blocked_known, world.fov, force=force)
            if probe is None: break
            ev("task_probe", probe=probe["name"], gain_bits=probe.get("expected_information_gain_bits"), plausible=len(strategy.plausible()))
            strategy.suspend_cmd_updates = True; mon.suspended = True                                            # V08
            try:
                for a in probe["actions"]:
                    r = do(a, ctx)
                    if not r["ok"]: break
            finally: strategy.suspend_cmd_updates = False; mon.suspended = False                                 # V08
            strategy.finish_probe(probe, world); adopt_belief()
            ctx = observe(phase_start, False)
            plan = backend.plan(ph, ctx); c["probe_replans"] += 1
            ev("task_probe_result", card=strategy.card())
        return plan, ctx

    def check(ph, plan, ctx):
        if strategy.verifier == "robust":                                                                      # NEW
            v = strategy.check(plan, ctx, ph, blocked_known, world.fov)
            if not v.safe: c["robust_rejects"] += 1
            return v.errors
        return verify_plan(plan, ctx["pose"], believed(ctx))                                                    # baseline verifier

    def abstain(why):                                                                                              # V08
        c["aborted"] += 1; ev("aborted_unverified", reason=why); return None

    def recover(ph, plan, ctx, phase_start, why):                                                                  # V08
        """Instead of abstaining: rung 1 extra safe probes, rung 2 guarded stepwise execution, else abstain."""
        if not rcfg.enabled or ph_state["entries"] >= rcfg.max_entries_per_phase: return abstain(why), ctx
        if not ph_state["entered"]: ph_state.update(entered=True, calls0=backend.calls, env0=world.tool_calls); c["rec_entered_phases"] += 1
        ph_state["entries"] += 1; c["rec_entries"] += 1; ev("recovery_enter", why=why, card=strategy.card())
        if rcfg.extra_probes > 0 and not ph_state["extra_used"] and strategy.verifier == "robust":
            ph_state["extra_used"] = True; strategy.max_task_probes += rcfg.extra_probes; n0 = strategy.task_probes
            plan, ctx = probe_loop(ph, plan, ctx, phase_start, force=True)
            if plan is None: return abstain(why), ctx
            if strategy.task_probes > n0:
                c["rec_reprobes"] += strategy.task_probes - n0
                if not check(ph, plan, ctx): c["rec_reprobe_resolved"] += 1; ev("recovery_reprobe_ok"); return plan, ctx
        if rcfg.guarded:
            strategy._cmd_override = None; ph_state["guarded"] = True; c["rec_guarded_phases"] += 1; ev("recovery_guarded"); return plan, ctx
        return abstain(why), ctx

    def verified(ph, plan, ctx, phase_start):
        for attempt in range(MAX_VERIFY):
            errs = check(ph, plan, ctx) if cfg.verify else []
            if not errs: return plan, ctx
            c["verify_fail"] += 1; ev("verify_failed", errors=errs)
            if strategy.verifier == "robust" and strategy.wants_probe():                                        # NEW: learn instead of guess
                n0 = strategy.task_probes
                plan, ctx = probe_loop(ph, plan, ctx, phase_start, force=True)
                if plan is None: break
                if strategy.task_probes > n0: continue
            if not cfg.repair:
                return recover(ph, plan, ctx, phase_start, "no_repair")                                    # V08
            plan = backend.repair(ph, ctx, "verification errors: " + "; ".join(errs[:3]))
            c["repairs"] += 1
        return recover(ph, plan, ctx, phase_start, "repair_exhausted")                                       # V08

    for ph in task.phases:
        ph_state.update(guarded=False, entered=False, entries=0); ab0 = c["aborted"]                      # V08
        start = world.pose(); phase_start = (start["x"], start["y"], start["yaw_deg"])
        if cfg.reactive:
            for _ in range(MAX_ACTIONS):
                if world.tool_calls > 2 * MAX_ACTIONS: break
                ctx = observe(phase_start, False)
                a = backend.next_action(ph, ctx)
                if a.get("tool") == "done": break
                do(a, ctx)
            continue
        ctx = observe(phase_start, True)
        plan = backend.plan(ph, ctx)
        plan, ctx = probe_loop(ph, plan, ctx, phase_start, force=False)                                         # NEW
        if cfg.verify: plan, ctx = verified(ph, plan, ctx, phase_start)
        replans = repairs = 0
        while plan is not None and world.tool_calls < 2 * MAX_ACTIONS:
            if first_exec["n_plausible"] is None: first_exec.update(n_plausible=len(strategy.plausible()), confidence=round(strategy.confidence(), 3))
            failure = None; unsafe = False; mon.reset()                                                          # V08
            guard = StepGuard(strategy, ctx, blocked_known, world.fov, rcfg.tau_action) if ph_state["guarded"] else None
            col0, ex0 = world.collisions, c["executed"]
            for a in plan.get("actions", []):
                if guard is not None and a.get("tool") == "move_forward" and not a.get("hw"):                       # V08: guarded step
                    a2, _risk, mode = guard.fit(world.pose(), a); c["guarded_steps"] += 1
                    if a2 is None: failure = "guard: no safe step"; unsafe = True; c["guarded_unsafe"] += 1; break
                    if mode == "creep":
                        c["guarded_creeps"] += 1; do(a2, ctx); failure = "guard: move shortened"; ev("guard_creep", asked=a, did=a2); break
                    a = a2
                r = do(a, ctx)
                if not r["ok"] and cfg.repair: failure = f"{a} failed: {r['message']}"; break
                if mon.tripped and cfg.repair:                                                                      # V08: runtime monitor
                    failure = f"monitor {mon.tripped}: {mon.events[-1].detail}"; c["monitor_trips"] += 1; c[f"{'dev' if mon.tripped == 'deviation' else 'blocked'}_trips"] += 1; ev("monitor_trip", kind=mon.tripped); break
            if guard is not None: c["rec_collisions"] += world.collisions - col0; c["rec_actions"] += c["executed"] - ex0
            ctx = observe(phase_start, False)
            if unsafe: c["aborted"] += 1; ev("aborted_unsafe_step"); break                                          # V08: rung 3
            e = plan.get("expect") or {}
            if failure is None and cfg.repair and any(k in e and e[k] != ctx["pose"][k] for k in ("x", "y")):
                failure = f"final pose ({ctx['pose']['x']},{ctx['pose']['y']}) != expected ({e.get('x')},{e.get('y')})"
            if failure:
                if repairs >= (rcfg.max_guarded_repairs if ph_state["guarded"] else MAX_REPAIRS): break            # V08
                repairs += 1; c["repairs"] += 1; ev("repair", failure=failure)
                plan = backend.repair(ph, ctx, failure)
            else:
                if backend.assess(ph, ctx) or replans >= MAX_REPLANS: break
                replans += 1; c["replans"] += 1; ev("replan_after_exhaustion")
                plan = backend.plan(ph, ctx)
            if plan is not None and cfg.verify and not ph_state["guarded"]: plan, ctx = verified(ph, plan, ctx, phase_start)   # V08
        if ph_state["entered"]:                                                                                  # V08: close recovery accounting
            c["rec_llm_calls"] += backend.calls - ph_state["calls0"]; c["rec_env_calls"] += world.tool_calls - ph_state["env0"]
            if c["aborted"] == ab0: c["rec_resolved_phases"] += 1
    res = check_success(world, task)
    usage = backend.usage_events[usage0:]
    tin = [u.get("input_tokens") for u in usage]; tout = [u.get("output_tokens") for u in usage]
    truth1, b = hypothesis_of(world), strategy.belief
    row = {"task_id": task.task_id, "family": task.family, "config": cfg.name, **task.factors,
           "success": int(res["success"]), "coverage": res.get("coverage", ""), "wrong_side": res.get("wrong_side", ""),
           "llm_calls": backend.calls - calls0, "env_tool_calls": world.tool_calls, "actions_executed": c["executed"],
           "invalid_calls": world.invalid_calls, "collisions": world.collisions, "repairs": c["repairs"],
           "verify_failures": c["verify_fail"], "replans": c["replans"], "aborted": c["aborted"],
           "input_tokens": sum(t for t in tin if t is not None) if any(t is not None for t in tin) else "",
           "output_tokens": sum(t for t in tout if t is not None) if any(t is not None for t in tout) else "",
           "elapsed_s": round(time.perf_counter() - t0, 4),
           "final_x": world.x, "final_y": world.y, "final_yaw": world.yaw,
           "embodiment_method": strategy.name, "true_embodiment": truth0.key, "final_true_embodiment": truth1.key, "belief_key": b.key,
           "belief_correct": int((b.mirrored_camera, b.left_right_swapped, round(b.move_gain, 3))
                                 == (truth1.mirrored_camera, truth1.left_right_swapped, round(truth1.move_gain, 3))),
           "id_probes": strategy.probes, "id_bay_actions": strategy.bay_actions, "id_bay_travel": strategy.bay_travel,
           "reidentifications": 0, "surprise_triggers": 0,
           # NEW columns
           "task_probes": strategy.task_probes, "task_probe_travel": strategy.task_probe_travel, "task_probe_failures": strategy.task_probe_failures,
           "probe_replans": c["probe_replans"], "robust_rejects": c["robust_rejects"], "final_confidence": round(strategy.confidence(), 3),
           "plausible_at_first_exec": "" if first_exec["n_plausible"] is None else first_exec["n_plausible"],
           "executed_any": int(c["executed"] > 0), **{k: c[k] for k in V08_KEYS}}                                  # V08
    if hasattr(strategy, "diag_row"): row.update(strategy.diag_row())
    return row

def run_episode_v08(world, task, cfg, backend, strategy, rcfg: RecoveryConfig | None = None, events=None):
    """Public entry point.  Exposes the current phase to VoiStrategy exactly like run_episode_v07."""
    from .voi import _PhaseTap
    tk = _PhaseTap(task, strategy) if hasattr(strategy, "current_phase") else task
    return _run_v08(world, tk, cfg, backend, strategy, rcfg or RecoveryConfig(), events)
