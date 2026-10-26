"""Policy backends for the spatial benchmark.

OracleBackend: perfect instruction parsing + geometric solver, with *mechanism-based* noise knobs. It validates
the harness only; its numbers are NOT evidence about any LLM. LLMBackend builds prompts for a real model.
"""
from __future__ import annotations
import json, random
from collections import deque
from .world2d import heading
from .tasks2d import simulate_ops, left_region, side_region, Phase, COUNT_QUESTIONS
from .navtools import TOOL_TEXT as NAV_TOOL_TEXT

def _bfs(start, goals, blocked, w, h):
    q, prev = deque([start]), {start: None}
    while q:
        c = q.popleft()
        if c in goals:
            path = []
            while c is not None: path.append(c); c = prev[c]
            return path[::-1]
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            n = (c[0] + dx, c[1] + dy)
            if 0 <= n[0] < w and 0 <= n[1] < h and n not in blocked and n not in prev:
                prev[n] = c; q.append(n)
    return None

def _act(tool, **args): return {"tool": tool, "args": args}

def _path_actions(x, y, yaw, path, clean_cells=frozenset(), clean_start=False):
    acts, run = [], 0
    def flush():
        nonlocal run
        if run: acts.append(_act("move_forward", distance=run)); run = 0
    if clean_start: acts.append(_act("clean"))
    for a, b in zip(path, path[1:]):
        want = {(1, 0): 0, (0, 1): 90, (-1, 0): 180, (0, -1): 270}[(b[0] - a[0], b[1] - a[1])]
        d = (want - yaw) % 360
        if d:
            flush()
            acts.append(_act("turn", direction="right" if d == 270 else "left", degrees=180 if d == 180 else 90)); yaw = want
        run += 1
        if b in clean_cells: flush(); acts.append(_act("clean"))
    flush()
    return acts, yaw

class OracleBackend:
    provider, model_id = "oracle-sim", "OracleBackend-v1"
    def __init__(self, slip_p=0.05, half_confusion_p=0.30, deviation_p=0.10, ask_p=0.8, false_ask_p=0.0, abstain=False):
        # abstain=True (run_spatial.py from v0.11): give up on an unsafe motion command or after looking all around without seeing the object.
        # abstain=False (default, all older runners) reproduces the pre-v0.11 oracle exactly: it searches until the step budget runs out.
        # ask_p: probability of asking when the instruction really is ambiguous; false_ask_p: probability of an unnecessary question at the
        # start of an instruction. Both ASSUMED (v0.11); a separate random stream keeps all earlier results unchanged.
        # deviation_p: C3/C4 only. Probability that a per-step proposal ignores the stored plan and re-derives the next action from the
        # current observation (as a non-persistent agent would). An ASSUMED value: no data on real models exists yet (sweep it).
        self.slip_p, self.half_p, self.dev_p, self.ask_p, self.false_ask_p = slip_p, half_confusion_p, deviation_p, ask_p, false_ask_p
        self.abstain = abstain
        self.calls = 0; self.usage_events = []; self.rng = random.Random(0); self._ref = {}; self.ask_rng = random.Random(0)
    def begin_episode(self, key): self.rng = random.Random(key); self._ref = {}; self.ask_rng = random.Random(key + "|ask")

    def _slip(self, acts, ctx=None):
        p = self.slip_p / 2 if ctx and "arith" in ctx.get("lesson_effects", ()) else self.slip_p     # lesson "re-check distances" (assumed effect)
        for a in acts:
            if a["tool"] == "move_forward" and self.rng.random() < p:
                a["args"]["distance"] = max(1, a["args"]["distance"] + self.rng.choice([-1, 1]))
        return acts

    def _target(self, ph, ctx):
        sel = ph.params.get("select")
        if sel and ctx.get("clarifications") and ph.params.get("clarified_select"): sel = ph.params["clarified_select"]
        if sel:       # relative reference ("closest", "furthest to your left", ...), resolved from the interpreted view
            if not ctx["reactive"] and ph.text + sel in self._ref: return self._ref[ph.text + sel], False   # a persistent plan keeps its referent; a reactive loop re-resolves every step
            vis = ctx["interp"]
            if not vis: return None, False
            if sel.startswith("box_"): vis = [o for o in vis if o["label"] == ph.params.get("label", "box")]
            if not vis: return None, False
            key = {"nearest": lambda o: o["distance_m"], "farthest": lambda o: -o["distance_m"],
                   "leftmost": lambda o: -o["bearing_deg"], "rightmost": lambda o: o["bearing_deg"],
                   "box_left_robot": lambda o: -o["bearing_deg"], "box_left_door": lambda o: o["bearing_deg"]}[sel]
            t = tuple(min(vis, key=key)["world_xy"])
            if not ctx["reactive"]: self._ref[ph.text + sel] = t
            return t, False
        label = ph.params["label"]
        if ph.params.get("use_memory_target") and ctx["use_memory"] and label in ctx["landmarks"]:
            return tuple(ctx["landmarks"][label]), True
        vis = [o for o in ctx["interp"] if o["label"] == label]
        if not vis: return None, False
        return tuple(min(vis, key=lambda o: o["distance_m"])["world_xy"]), False

    def _core(self, ph, ctx):
        p = ctx["pose"]; w, h = p["room_w"], p["room_h"]; me = (p["x"], p["y"]); yaw = p["yaw_deg"]
        blocked = {tuple(c) for c in ctx["obstacles_known"]}
        # v0.11 clarification: decide once per instruction whether to ask (separate random stream)
        key = ph.text + "|ask"
        if key not in self._ref:
            needed = ph.params.get("clarify_answer") is not None
            self._ref[key] = self.ask_rng.random() < (self.ask_p if needed else self.false_ask_p)
        if self._ref[key] and not any(ph.text in str(c.get("question", "")) for c in ctx.get("clarifications", [])):
            return {"actions": [_act("ask", question=f"What exactly do you mean by: {ph.text}")]}, False
        if ph.kind == "remember":
            return ({"actions": [_act("remember", label=ph.params["label"])] if ctx["use_memory"] else []}), True
        if ph.kind == "motion":
            ops = ph.params["ops"]; ps = ctx["phase_start_pose"]
            gp, route = simulate_ops(ps[:2], ps[2], ops)
            if self.abstain and any(not (0 <= x < w and 0 <= y < h) for x, y in route):      # v0.11 safety: refuse a command that drives into the wall
                return {"actions": [_act("abstain", reason="the commanded motion leaves the room")]}, False
            if (me, yaw) == ((gp[0], gp[1]), gp[2]): return {"actions": []}, True
            k = max((k for k in range(len(ops) + 1)
                     if simulate_ops(ps[:2], ps[2], ops[:k])[0] == (me[0], me[1], yaw)), default=None)
            if k is not None:          # pose matches the state after k completed ops: continue with the rest
                acts = []
                for op, v in ops[k:]:
                    if op == "forward": acts.append(_act("move_forward", distance=v))
                    elif v == "around": acts.append(_act("turn", direction="left", degrees=180))
                    else: acts.append(_act("turn", direction=v, degrees=90))
            else:                      # off the nominal route (e.g. after a slip): navigate to the goal pose
                path = _bfs(me, {(gp[0], gp[1])}, blocked, w, h) or [me]
                acts, y2 = _path_actions(*me, yaw, path)
                d = (gp[2] - y2) % 360
                if d: acts.append(_act("turn", direction="right" if d == 270 else "left", degrees=180 if d == 180 else 90))
            return {"actions": self._slip(acts, ctx), "expect": {"x": gp[0], "y": gp[1], "yaw_deg": gp[2]}}, False
        if ph.kind == "return_start":
            t = tuple(ctx["start_pose"][:2])
            if me == t: return {"actions": []}, True
            path = _bfs(me, {t}, blocked - {t}, w, h)
            if not path: return {"actions": [_act("turn", direction="left", degrees=90)]}, False
            if ctx.get("nav_tools"): return {"actions": [_act("goto_cell", x=t[0], y=t[1])], "expect": {"x": t[0], "y": t[1]}}, False
            acts, _ = _path_actions(*me, yaw, path)
            return {"actions": self._slip(acts, ctx), "expect": {"x": t[0], "y": t[1]}}, False
        if ph.kind == "goto":
            t, _ = self._target(ph, ctx)
            if t is None:              # v0.11: give up (abstain) after looking in all four directions without seeing the object
                if ctx["reactive"]:
                    hist = [(hst.get("action") or {}).get("tool") for hst in ctx["history"][-3:]]
                    searched = len(hist) == 3 and all(x == "turn" for x in hist)
                else:
                    sk = ph.text + "|search"; cell, seen = self._ref.get(sk, (None, set()))
                    if cell != me: seen = set()
                    seen.add(yaw); self._ref[sk] = (me, seen); searched = len(seen) >= 4      # looked all around from this cell
                if self.abstain and searched and not ph.params.get("select"):
                    return {"actions": [_act("abstain", reason=f"no {ph.params.get('label')} anywhere in view")]}, False
                return {"actions": [_act("turn", direction="left", degrees=90)]}, False
            if abs(t[0] - me[0]) + abs(t[1] - me[1]) == 1: return {"actions": []}, True
            goals = {(t[0] + dx, t[1] + dy) for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1))
                     if 0 <= t[0] + dx < w and 0 <= t[1] + dy < h} - (blocked - {t}) - {t}
            path = _bfs(me, goals, blocked, w, h)
            if not path: return {"actions": [_act("turn", direction="left", degrees=90)]}, False
            if ctx.get("nav_tools"):   # v0.11 macro tool: the runtime computes the path, no arithmetic slips
                return {"actions": [_act("goto_cell", x=path[-1][0], y=path[-1][1])], "expect": {"x": path[-1][0], "y": path[-1][1]}}, False
            acts, _ = _path_actions(*me, yaw, path)
            return {"actions": self._slip(acts, ctx), "expect": {"x": path[-1][0], "y": path[-1][1]}}, False
        if ph.kind == "conditional":
            # persistent agents evaluate the condition once, on the view at the start of the instruction; a reactive loop re-evaluates it
            key = ph.text + "|branch"
            if ctx["reactive"] or key not in self._ref:
                self._ref[key] = any(o["label"] == ph.params["if_label"] for o in ctx["interp"])
            lab = ph.params["then_label"] if self._ref[key] else ph.params["else_label"]
            return self._core(Phase(ph.text + "|" + lab, "goto", {"label": lab}), ctx)
        if ph.kind == "report":
            pred = COUNT_QUESTIONS[ph.params["level"]][1]
            cur = {(o["label"], tuple(o["world_xy"])) for o in ctx["interp"]}
            if ctx["reported"]: return {"actions": []}, True
            if ctx["reactive"]:      # no state: only the current view and the turns still in the history window
                obs, n_yaws = cur, 1 + sum(1 for hst in ctx["history"] if (hst.get("action") or {}).get("tool") == "turn")
            else:                    # latest view per heading; objects are merged across headings by their (believed) world cell
                stt = self._ref.setdefault(ph.text + "|report", {})
                stt[yaw] = cur; obs, n_yaws = set().union(*stt.values()), len(stt)
            if n_yaws < 4: return {"actions": [_act("turn", direction="left", degrees=90)]}, False
            return {"actions": [_act("report", answer=str(sum(1 for lab, _ in obs if pred(lab))))]}, False
        if ph.kind in ("clean_left", "clean_region"):
            side = ph.params.get("side", "left")
            region = {(x, y) for x in range(w) for y in range(h)} if side == "all" else side_region(w, h, ctx["start_pose"], side)
            half_p = 0.0 if "frame" in ctx.get("lesson_effects", ()) else self.half_p                    # lesson "use the start heading" (assumed effect)
            confused = side != "all" and not ctx["grounded"] and ctx["start_pose"][2] != 0 and self.rng.random() < half_p
            if confused: region = {(x, y) for x in range(w) for y in range(h)} - region      # frame confusion
            if ph.params.get("avoid_label"):          # cells next to the avoided object, as localised from the camera (persistent agents keep it)
                key = ph.text + "|avoid"
                seen = [tuple(o["world_xy"]) for o in ctx["interp"] if o["label"] == ph.params["avoid_label"]]
                if ctx["reactive"] or key not in self._ref or (seen and not self._ref[key]): self._ref[key] = seen
                forb = {(c[0] + dx, c[1] + dy) for c in self._ref[key] for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1))}
                region = region - forb
                if ph.params.get("avoid_mode") == "enter": blocked = blocked | (forb - {me})
            todo = [c for c in region if c not in blocked and c not in ctx["cleaned_known"]]
            reach = _bfs(me, set(todo), blocked, w, h)
            if not todo or not reach: return {"actions": []}, True
            if ctx.get("nav_tools"):   # v0.11 coverage tool: the model only chooses the half (and what to avoid)
                said = {"left": "right", "right": "left", "front": "back", "back": "front"}.get(side, side) if confused else side
                avoid = sorted(forb) if ph.params.get("avoid_label") else []
                return {"actions": [_act("cover_region", side=said, avoid=[list(c) for c in avoid])]}, False
            if ctx["reactive"]:      # one decision at a time: nearest unclean known cell
                path = reach
                acts, _ = _path_actions(*me, yaw, path, clean_cells={path[-1]}, clean_start=(len(path) == 1))
                return {"actions": self._slip(acts[:1], ctx) if acts else []}, False
            rows = sorted(todo, key=lambda c: (c[1], c[0] if c[1] % 2 == 0 else -c[0]))
            if abs(rows[-1][0] - me[0]) + abs(rows[-1][1] - me[1]) < abs(rows[0][0] - me[0]) + abs(rows[0][1] - me[1]): rows = rows[::-1]
            cur, cy, acts, cells, last = me, yaw, [], set(rows), me
            if me in cells: acts.append(_act("clean")); cells.discard(me)
            for c in rows:
                if c == me: continue
                path = _bfs(cur, {c}, blocked, w, h)
                if not path: continue
                a, cy = _path_actions(*cur, cy, path, clean_cells={c}); acts += a; cur = c
            return {"actions": self._slip(acts, ctx), "expect": {"x": cur[0], "y": cur[1], "cleaned_min": len(rows)}}, False
        raise ValueError(ph.kind)

    # v0.12 localiser tool (assumed oracle policy): after any motion, localise before the next decision.
    LOCALIZE = {"actions": [{"tool": "localize", "args": {}}]}
    def plan(self, ph, ctx):
        self.calls += 1
        if ctx.get("pose_stale"): return json.loads(json.dumps(self.LOCALIZE))
        return self._core(ph, dict(ctx, fresh_phase=ctx.get("fresh_phase", False)))[0]
    def repair(self, ph, ctx, failure):
        self.calls += 1
        if ctx.get("pose_stale"): return json.loads(json.dumps(self.LOCALIZE))
        return self._core(ph, dict(ctx, fresh_phase=False))[0]
    def next_action(self, ph, ctx):
        self.calls += 1
        if ctx.get("pose_stale"): return _act("localize")
        plan, done = self._core(ph, dict(ctx, fresh_phase=False))
        if done or not plan["actions"]: return _act("done")
        return plan["actions"][0]
    def plan_step(self, ph, ctx):
        """C2: a fresh plan at every step from the current observation, nothing kept (so the referent and the half-room are re-derived)."""
        self.calls += 1
        if ctx.get("pose_stale"): return json.loads(json.dumps(self.LOCALIZE))
        plan, done = self._core(ph, dict(ctx, fresh_phase=False, reactive=True))
        return {"actions": []} if done else plan
    def scratchpad_step(self, ph, ctx, actions, cursor, constrained):
        """C3/C4: propose the next call given the stored plan. With probability deviation_p the proposal is re-derived from the observation."""
        self.calls += 1
        if self.rng.random() < self.dev_p:
            plan, done = self._core(ph, dict(ctx, fresh_phase=False, reactive=True))
            return _act("done") if done or not plan["actions"] else plan["actions"][0]
        if cursor < len(actions): return json.loads(json.dumps(actions[cursor]))
        return _act("done") if self._core(ph, dict(ctx, fresh_phase=False))[1] else _act("replan", reason="plan finished, instruction not complete")
    def assess(self, ph, ctx):
        self.calls += 1
        if ph.kind == "goto":
            t, _ = self._target(ph, ctx)
            return t is not None and abs(t[0] - ctx["pose"]["x"]) + abs(t[1] - ctx["pose"]["y"]) == 1
        return self._core(ph, dict(ctx, fresh_phase=False))[1]

# ---------------------------------------------------------------- real LLM backend
CONVENTIONS = """You control a wheeled robot in a rectangular room (grid, 1 cell = 1 m, x right, y up on the map).
yaw 0 = facing +x, yaw 90 = facing +y; turning LEFT adds +90 deg, RIGHT subtracts 90. Objects occupy one cell; you cannot enter an occupied cell
or leave the room. move_forward moves along the current heading."""
TOOLS = """Tools (JSON): {"tool":"move_forward","args":{"distance":int 0..12}} | {"tool":"turn","args":{"direction":"left|right","degrees":90|180|270}}
| {"tool":"clean","args":{}} (cleans current cell) | {"tool":"describe_view","args":{}} | {"tool":"get_pose","args":{}}"""

LOCALIZE_TOOL = (' | {"tool":"localize","args":{}} (map-based localiser: corrects your pose estimate; wheel odometry can drift, '
                 'so use it after driving when an exact position matters)')
INTERACTIVE_TOOLS = (' | {"tool":"ask","args":{"question":str}} (ask the person who gave the instruction; only if it is genuinely ambiguous; '
                     'at most 2 per instruction) | {"tool":"abstain","args":{"reason":str}} (refuse because the instruction is impossible or unsafe; ends the task)')

class LLMParseError(ValueError): pass

def _extract_json(text: str):
    """Model reply -> JSON value. Accepts a bare object, a fenced block, or an object embedded in prose (first {...} that parses)."""
    t = text.strip()
    if t.startswith("```"): t = t.strip("`").strip(); t = t[4:].strip() if t.lower().startswith("json") else t
    try: return json.loads(t)
    except json.JSONDecodeError: pass
    dec = json.JSONDecoder()
    for k, ch in enumerate(t):
        if ch in "{[":
            try: return dec.raw_decode(t[k:])[0]
            except json.JSONDecodeError: continue
    raise LLMParseError(f"no JSON in reply: {t[:120]!r}")

class LLMBackend:
    """Prompts for a real model. Ablation switches (all on by default for grounded configs):
    sensor_doc: show the camera documentation; interp_view: show the interpreted view (bearing/distance/world cell)."""
    def __init__(self, client, grounded: bool, use_memory: bool, sensor_doc: bool = True, interp_view: bool = True, extra_tools: str = "",
                 interactive: bool = True, nav_tools: bool = False, hierarchical: bool = False, localize_tool: bool = False,
                 vision: bool = False):
        self.client, self.grounded, self.use_memory, self.interactive = client, grounded, use_memory, interactive
        self.nav_tools, self.hierarchical, self.localize_tool, self.vision = nav_tools, hierarchical, localize_tool, vision
        self._images = None
        if hierarchical: self.PLAN_FORMAT = self.PLAN_FORMAT_H
        self.sensor_doc, self.interp_view, self.extra_tools = sensor_doc, interp_view, extra_tools
        self.calls = 0; self.parse_errors = 0
    @property
    def provider(self): return getattr(self.client, "provider", "llm")
    @property
    def model_id(self): return getattr(self.client, "model_id", "llm")
    @property
    def usage_events(self): return getattr(self.client, "usage_events", [])
    def begin_episode(self, key): pass

    def _context(self, ph, ctx):
        s = [CONVENTIONS, TOOLS + (' | {"tool":"remember","args":{"label":str}} (stores nearest visible object of that label in memory)' if self.use_memory else "") + self.extra_tools
             + (INTERACTIVE_TOOLS if self.interactive else "") + (NAV_TOOL_TEXT if self.nav_tools else "") + (LOCALIZE_TOOL if self.localize_tool else "")]
        self._images = None
        if self.vision:            # v0.12: the camera image replaces the detector output (labels, image_x, sizes) and the interpreted view
            from .vision import render_view
            self._images = [render_view(ctx["raw"])]
            s.append("CAMERA IMAGE attached (front camera, horizontal FOV 90 deg, range 7 m; nearer objects look bigger and stand lower in the "
                     "image). No object list is given: recognise the objects (colour and type) in the image yourself.")
            if self.grounded and self.sensor_doc: s.append("SENSOR DOCUMENTATION: " + ctx["sensor_spec"])
        elif self.grounded and (self.sensor_doc or self.interp_view):
            if self.sensor_doc: s.append("SENSOR DOCUMENTATION: " + ctx["sensor_spec"])
            if self.interp_view: s.append("Interpreted view (computed from the documentation): " + json.dumps(ctx["interp"]))
            else: s.append("Raw camera output: " + json.dumps(ctx["raw"]))
        else:
            s.append("Raw camera output (no documentation available): " + json.dumps(ctx["raw"]))
        if ctx.get("embodiment_card"):
            s.append("EMBODIMENT CARD (from calibration probes): " + json.dumps(ctx["embodiment_card"]) +
                     " The runtime already compensates actuator differences (turn polarity, move gain); plan in nominal terms.")
        s.append("Pose: " + json.dumps(ctx["pose"]) + "  Pose at start of this instruction: " + json.dumps(ctx["phase_start_pose"]))
        s.append("Pose at the very start of the episode (x,y,yaw): " + json.dumps(ctx["start_pose"]))
        s.append("Obstacle cells known so far: " + json.dumps(sorted(ctx["obstacles_known"])))
        if self.use_memory: s.append("Memory: " + json.dumps({"landmarks": ctx["landmarks"], "cleaned_cells": sorted(ctx["cleaned_known"])}))
        else: s.append("Cells cleaned (as far as you can tell): " + json.dumps(sorted(ctx["cleaned_known"])))
        if ctx["history"]: s.append("Recent actions and results: " + json.dumps(ctx["history"]))
        if ctx.get("lessons"): s.append("Lessons you wrote after earlier failed attempts on similar tasks: " + json.dumps(ctx["lessons"], ensure_ascii=False))
        if ctx.get("clarifications"): s.append("Clarifications from the person who gave the instruction: " + json.dumps(ctx["clarifications"]))
        s.append(f"Current instruction: {ph.text}")
        return "\n".join(s)

    def _ask(self, prompt, tag):
        """One model call; on unparseable output ONE corrective retry (counted as a second call). Raises LLMParseError if both fail."""
        self.calls += 1
        kw = {"images": self._images} if self._images else {}
        text = self.client.complete(prompt, tag, **kw)
        try: return _extract_json(text)
        except LLMParseError:
            self.parse_errors += 1; self.calls += 1
            text = self.client.complete(prompt + "\nYour previous reply was not valid JSON. Reply with ONE JSON object only.", tag + "_retry", **kw)
            try: return _extract_json(text)
            except LLMParseError: self.parse_errors += 1; raise

    PLAN_FORMAT_H = ('\nReturn ONLY JSON {"regions":[{"name":str,"actions":[...]}, ...], "expect":{"x":int,"y":int,"yaw_deg":int(optional),'
                     '"cleaned_min":int(optional)}}. Plan hierarchically: split the room into named regions (e.g. "left-front quarter"), visit them '
                     'in a sensible order and give the tool calls for each region; expect is the final pose. Do not add markdown.')
    PLAN_FORMAT = ('\nReturn ONLY JSON {"actions":[...], "expect":{"x":int,"y":int,"yaw_deg":int(optional),"cleaned_min":int(optional)}} '
                   "where expect is the final pose and (for cleaning) number of cells you intend to clean. Plan the whole instruction; do not add markdown.")
    def _plan_or_empty(self, prompt, tag):
        try: return self._ask(prompt, tag)
        except LLMParseError: return {"actions": [], "parse_error": True}
    def _step_or_invalid(self, prompt, tag):
        try: return self._ask(prompt, tag)
        except LLMParseError: return {"tool": "__unparseable__", "args": {}}

    def plan(self, ph, ctx):
        return self._plan_or_empty(self._context(ph, ctx) + self.PLAN_FORMAT, "plan")
    def repair(self, ph, ctx, failure):
        p = self._context(ph, ctx) + f"\nThe previous plan failed: {failure}\nReturn ONLY JSON like before, a new plan for the REMAINING work starting from the current pose." + self.PLAN_FORMAT
        return self._plan_or_empty(p, "repair")
    def next_action(self, ph, ctx):
        p = self._context(ph, ctx) + '\nChoose exactly ONE next action. Return ONLY JSON {"tool":..., "args":{...}} or {"tool":"done","args":{}} if the instruction is complete.'
        return self._step_or_invalid(p, "step")
    def plan_step(self, ph, ctx):
        """C2 (non-persistent planner): a full plan for the remaining work at every step; the runtime executes only its first action."""
        p = (self._context(ph, ctx) + "\nNo plan is stored between steps. Write the plan for ALL remaining work of the current instruction, starting from "
             "the current pose. Only its FIRST action will be executed; then you will be asked again with a new observation. "
             'If the instruction is already complete, return {"actions":[]}.' + self.PLAN_FORMAT)
        return self._plan_or_empty(p, "plan_step")
    def scratchpad_step(self, ph, ctx, actions, cursor, constrained):
        """C3/C4: the stored plan (scratchpad) is shown with a cursor; the model issues the next tool call."""
        pad = "\n".join(f"  {'->' if k == cursor else '  '} {k}: {json.dumps(a)}" for k, a in enumerate(actions)) or "  (empty)"
        state = f"Steps 0..{cursor - 1} are done; the next planned step is {cursor}." if cursor < len(actions) else "All planned steps are done."
        if constrained:
            rule = ("EXECUTION CONSTRAINT: the runtime admits ONLY the next planned step, or {\"tool\":\"replan\",\"args\":{\"reason\":str}} if the plan is wrong "
                    "or incomplete, or {\"tool\":\"done\",\"args\":{}} once all steps are done and the instruction is complete. Any other call is rejected "
                    "and the planned step is executed instead.")
        else:
            rule = ("You may follow the plan or issue a different tool call if the observation shows the plan is wrong. "
                    "{\"tool\":\"replan\",\"args\":{\"reason\":str}} rewrites the plan; {\"tool\":\"done\",\"args\":{}} ends the instruction.")
        p = (self._context(ph, ctx) + f"\nSCRATCHPAD (persistent plan for this instruction):\n{pad}\n{state}\n{rule}\n"
             'Return ONLY JSON for ONE tool call: {"tool":..., "args":{...}}.')
        return self._step_or_invalid(p, "scratch_step")
    def write_lesson(self, task, row, trace):
        """One call after a failed episode: the model turns its own failure into a short, general lesson."""
        self._images = None
        p = ("You control a wheeled robot in a grid room. You just FAILED this task.\nInstructions: " + json.dumps([ph.text for ph in task.phases], ensure_ascii=False) +
             f"\nOutcome: final pose x={row.get('final_x')} y={row.get('final_y')} yaw={row.get('final_yaw')}, collisions={row.get('collisions')}, "
             f"invalid calls={row.get('invalid_calls')}, repeated calls={row.get('repeated_calls')}, coverage={row.get('coverage')}, answer={row.get('answer')}.\n"
             "Last actions: " + json.dumps(trace[-15:]) + "\nWrite ONE general lesson (at most 30 words) that would help on similar tasks next time. "
             'Do not mention this task\'s coordinates. Return ONLY JSON {"lesson": str}.')
        try: r = self._ask(p, "lesson")
        except LLMParseError: return ""
        return str(r.get("lesson", ""))[:300] if isinstance(r, dict) else ""
    def assess(self, ph, ctx):
        p = self._context(ph, ctx) + '\nIs the current instruction fully completed? Return ONLY JSON {"done": true|false}.'
        try: r = self._ask(p, "assess")
        except LLMParseError: return False
        return bool(r.get("done")) if isinstance(r, dict) else False
