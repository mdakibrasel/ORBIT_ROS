"""Hypothesis-robust plan verification and task-embedded probing ("plan-as-probe").   (v0.6.0 extension)

This module adds two things on top of the v0.5.1 embodiment code WITHOUT changing any v0.5.1 behaviour:

1. Robust verification.  The v0.4/0.5 verifier dry-runs a plan under ONE believed embodiment (the MAP hypothesis).
   `check_plan` dry-runs the same plan under EVERY still-plausible hypothesis (weight >= min_weight) and reports which
   hypotheses make it fail (collision / out-of-bounds / invalid call / wrong declared final pose / wrong goal cell).
   A plan is accepted only if it passes under all of them.  If none passes, the caller must not execute it.

2. Plan-as-probe.  Instead of a separate calibration bay, the agent looks at the first actions of its OWN task plan.
   Each safe prefix (<= `probe_len` move/turn commands, no clean/remember) is a candidate probe: the predicted outcome under
   each hypothesis is (final pose, image-side of each already-seen landmark).  The landmark model is rebuilt per hypothesis
   from the first camera view with that hypothesis' camera convention, so a mirrored camera is detectable from the
   task-world itself.  A prefix is "safe" only if it is collision-free under every plausible hypothesis.  The prefix with the
   highest expected information gain is executed, the belief is updated from the observed outcome, and the agent replans.

Everything here is a simulator-level mechanism.  Nothing is claimed about LLM performance; see docs/ROBUST_PROBE_EXTENSION.md.
"""
from __future__ import annotations
import random
from dataclasses import dataclass, field
from typing import Any
from .world2d import World2D, Obj
from .cognition import SensorInterpreter
from .embodiment import Hypothesis, make_probe
from .embodiment_runtime import (NOMINAL, MAX_PROBE_TRAVEL, PROBES, _new_identifier, _deviation, _execute, build_probe,
                                 hypothesis_of, predict_pose, EmbodimentAdapter, IdResult, travel_of)

NON_MOTION = ("describe_view", "get_pose", "done")
IDENT_MODES = ("none", "bay", "probe", "hybrid")
VERIFIERS = ("old", "robust")
ARMS = tuple(f"{i}_{v}" for i in IDENT_MODES for v in VERIFIERS)

# ------------------------------------------------------------------ simulation under one hypothesis
def _believed_world(h: Hypothesis, pose, raw, common_obstacles, fov) -> tuple[World2D, list[dict]]:
    """World as it would be IF `h` were true: landmark cells come from the raw view read with h's camera convention."""
    interp = SensorInterpreter(fov, h.mirrored_camera, True).interpret(raw, pose)
    objs, seen = [], set()
    for i, o in enumerate(interp):
        c = tuple(o["world_xy"])
        if c not in seen: seen.add(c); objs.append(Obj(f"v{i}", o["label"], *c))
    for j, c in enumerate(sorted(common_obstacles)):
        if tuple(c) not in seen: seen.add(tuple(c)); objs.append(Obj(f"k{j}", "blocked", *c))
    w = World2D(pose["room_w"], pose["room_h"], (pose["x"], pose["y"]), pose["yaw_deg"], objs, mirrored_camera=h.mirrored_camera, fov_deg=fov)
    w.swapped, w.move_gain, w.turn_gain = h.left_right_swapped, h.move_gain, h.turn_gain
    return w, interp

def _run_commands(sim: World2D, actions, cmd_belief: Hypothesis) -> list[str]:
    """Execute nominal-semantics `actions` through an adapter built for cmd_belief on `sim`. Returns error strings."""
    ad, errors = EmbodimentAdapter(cmd_belief), []
    for i, a in enumerate(actions):
        tool, args = a.get("tool"), a.get("args") or {}
        if tool in NON_MOTION or tool == "remember": continue
        if tool not in ("move_forward", "turn", "clean", "stop_robot"): errors.append(f"action[{i}]: unknown tool {tool}"); continue
        for hw_tool, hw_args in ([(tool, args)] if a.get("hw") else ad.translate(tool, args)):
            r = sim.execute(hw_tool, hw_args)
            if not r["ok"]: errors.append(f"action[{i}] {tool}{args}: {r['message']}"); break
    return errors

def view_label(pose: dict, raw_view: list[dict], known_labels, noise_p=0.0, rng: random.Random | None = None) -> str:
    """What a probe can observe: pose + image side (L/R/C) of every landmark whose label was seen in the first view."""
    toks = []
    for o in raw_view:
        if o["label"] not in known_labels: continue
        side = "L" if o["image_x"] < 0 else ("R" if o["image_x"] > 0 else "C")
        if noise_p and rng is not None and side != "C" and rng.random() < noise_p: side = "R" if side == "L" else "L"
        toks.append(f'{o["label"]}:{side}')
    return f'{pose["x"]},{pose["y"]},{pose["yaw_deg"]}|{";".join(sorted(toks))}'

# ------------------------------------------------------------------ robust verification
@dataclass
class RobustVerdict:
    safe: bool
    checked: int                                   # hypotheses examined
    failing: list = field(default_factory=list)    # [(hypothesis key, weight, [errors])]
    failing_mass: float = 0.0
    errors: list = field(default_factory=list)     # flat strings for the agent loop / logs

def _goal_cell(ph, ctx, interp):
    if getattr(ph, "kind", None) != "goto": return None
    label = ph.params["label"]
    if ph.params.get("use_memory_target") and ctx.get("use_memory") and label in ctx.get("landmarks", {}): return tuple(ctx["landmarks"][label])
    vis = [o for o in interp if o["label"] == label]
    return tuple(min(vis, key=lambda o: o["distance_m"])["world_xy"]) if vis else None

def check_plan(plan, ctx, ph, hyps_w, cmd_belief: Hypothesis, common_obstacles=(), fov=90.0, min_weight=0.01) -> RobustVerdict:
    """Dry-run `plan` under every hypothesis with weight >= min_weight. `hyps_w` = [(Hypothesis, weight)]."""
    pose, raw = ctx["pose"], ctx["raw"]
    actions, e = plan.get("actions", []), plan.get("expect") or {}
    verdict = RobustVerdict(True, 0)
    for h, w in hyps_w:
        if w < min_weight: continue
        verdict.checked += 1
        sim, interp = _believed_world(h, pose, raw, common_obstacles, fov)
        errs = _run_commands(sim, actions, cmd_belief)
        final = sim.pose()
        for k in ("x", "y", "yaw_deg"):
            if k in e and e[k] != final[k]: errs.append(f"expect.{k}={e[k]} but dry-run gives {final[k]}")
        if "cleaned_min" in e and len(sim.cleaned) < e["cleaned_min"]: errs.append(f"expect cleaned>={e['cleaned_min']} but dry-run cleans {len(sim.cleaned)}")
        tgt = _goal_cell(ph, ctx, interp)
        if tgt is not None and abs(tgt[0] - final["x"]) + abs(tgt[1] - final["y"]) != 1:
            errs.append(f"goal cell {tgt} is not adjacent to final pose ({final['x']},{final['y']})")
        if errs:
            verdict.failing.append((h.key, w, errs)); verdict.failing_mass += w
            verdict.errors.append(f"[{h.key} w={w:.2f}] {errs[0]}")
    verdict.safe = not verdict.failing
    return verdict

# ------------------------------------------------------------------ strategy
class RobustStrategy:
    """Identification mode x verifier.  ident in IDENT_MODES, verifier in VERIFIERS.

    none    uniform prior, no evidence at all (isolates the verifier: robust verification must abstain or find a plan safe for all)
    bay     calibration bay, information-gain probes (= v0.5.1 `infogain`)
    probe   no bay; plan-as-probe during the task
    hybrid  a cheap bay (look + turn90, <= 0.5 m of travel) followed by plan-as-probe for what is left (the move gain)
    """
    def __init__(self, ident: str, verifier: str, eps=0.0, noise_p=0.0, seed=0, stop=0.95, max_task_probes=3, probe_len=3,
                 min_weight=0.01, cost_weight=0.0, hybrid_names=("look", "turn90"), hybrid_probes=2):
        if ident not in IDENT_MODES: raise ValueError(f"unknown ident mode {ident}")
        if verifier not in VERIFIERS: raise ValueError(f"unknown verifier {verifier}")
        self.ident_mode, self.verifier = ident, verifier
        self.name = f"{ident}_{verifier}"
        self.eps, self.noise_p, self.stop, self.cost_weight = eps, noise_p, stop, cost_weight
        self.max_task_probes, self.probe_len, self.min_weight = max_task_probes, probe_len, min_weight
        self.hybrid_names, self.hybrid_probes = hybrid_names, hybrid_probes
        self.rng = random.Random(f"{ident}|{seed}")      # shared across verifiers so old-vs-robust is a paired comparison
        self.ident = _new_identifier(10 ** 6, eps, cost_weight, stop)
        self.probes = self.bay_actions = 0; self.bay_travel = 0.0           # bay phase (same names as EmbodimentStrategy)
        self.task_probes = 0; self.task_probe_travel = 0.0; self.task_probe_failures = 0
        self.cmd_updates = 0; self.reids = self.triggers = 0; self.suspend_cmd_updates = False
        self.known_labels: set[str] = set()
        self.status = "none"; self._cmd_override: Hypothesis | None = None

    # -- belief
    @property
    def task_probe_enabled(self): return self.ident_mode in ("probe", "hybrid")
    @property
    def belief(self) -> Hypothesis:
        return min(self.ident.hypotheses, key=lambda h: (-self.ident.weights[h.key], _deviation(h)))
    def confidence(self): return self.ident.weights[self.belief.key]
    def plausible(self): return [(h, self.ident.weights[h.key]) for h in self.ident.hypotheses if self.ident.weights[h.key] >= self.min_weight]
    def wants_probe(self): return self.task_probe_enabled and self.task_probes < self.max_task_probes

    # -- identification before the task
    def start(self, world):
        truth = hypothesis_of(world)
        if self.ident_mode in ("bay", "hybrid"):
            names, budget = (PROBES, 4) if self.ident_mode == "bay" else (self.hybrid_names, self.hybrid_probes)
            res = IdResult(NOMINAL, 0.0); self.ident.max_probes = budget
            for _ in range(2 * budget):
                p = self.ident.choose_probe([build_probe(n, self.ident.hypotheses) for n in names])
                if p is None: break
                _execute(truth, p["name"], res, self.ident, self.rng, self.noise_p)
            self.probes, self.bay_actions, self.bay_travel = self.ident.probes_used, res.bay_actions, res.bay_travel
        self.ident.max_probes = 10 ** 6          # task-phase probing is bounded by max_task_probes instead
        self.status = "identified" if self.confidence() >= self.stop else "ambiguous"

    # -- verification
    @property
    def cmd_belief(self) -> Hypothesis:
        """Embodiment used to translate commands: the MAP belief, unless robust verification found a better translation."""
        return self._cmd_override or self.belief

    def check(self, plan, ctx, ph, common_obstacles, fov) -> RobustVerdict:
        """Robust verification.  Tries the MAP translation first, then the translation of every other plausible hypothesis;
        the first translation that is safe under ALL plausible hypotheses wins (and is remembered for execution)."""
        plaus = self.plausible(); self._cmd_override = None
        cands, seen = [self.belief] + [h for h, _ in sorted(plaus, key=lambda hw: -hw[1])], set()
        best = None
        for cmd in cands:
            sig = (cmd.left_right_swapped, cmd.move_gain)
            if sig in seen: continue
            seen.add(sig)
            v = check_plan(plan, ctx, ph, plaus, cmd, common_obstacles, fov, self.min_weight)
            if v.safe: self._cmd_override = cmd; return v
            if best is None or v.failing_mass < best.failing_mass - 1e-12: best = v
        return best

    # -- plan-as-probe
    def plan_probe(self, plan, ctx, common_obstacles, fov, force=False) -> dict | None:
        """Best safe, informative prefix of `plan`, or None.  `force` ignores the confidence stop (used when verification failed)."""
        if not self.wants_probe() or not plan: return None
        self._cmd_override = None                    # probes are translated with the MAP belief they were predicted under
        if not self.known_labels: self.known_labels = {o["label"] for o in ctx["raw"]}
        cmd, plaus = self.belief, self.plausible()
        prefix, cands, moved = [], [], False
        for a in plan.get("actions", []):
            if a.get("tool") in NON_MOTION: continue
            if a.get("tool") in ("remember", "stop_robot") or len(prefix) >= self.probe_len: break
            if a.get("tool") == "clean":
                if moved: break                      # a clean after (uncertain) motion could hit the wrong cell
                prefix.append(a); continue           # a clean at the CURRENT cell is exactly what the plan wants
            prefix.append(a); moved = True; cands.append(self._build(list(prefix), ctx, common_obstacles, fov, cmd, plaus))
            if a.get("tool") == "move_forward" and isinstance(a.get("args", {}).get("distance"), int):
                # guarded sub-moves: raw hardware commands c, a few metres either side of the intended n, replacing this move.
                # They separate move gains that the intended move alone cannot (0.25 vs 0.5 for n=1..2).
                n = a["args"]["distance"]
                for c_ in range(1, min(12, n + 3) + 1):
                    if c_ == n: continue
                    sub = list(prefix[:-1]) + [{"tool": "move_forward", "args": {"distance": c_}, "hw": True}]
                    cands.append(self._build(sub, ctx, common_obstacles, fov, cmd, plaus))
        old_stop = self.ident.stop_confidence
        if force: self.ident.stop_confidence = 1.01
        try: chosen = self.ident.choose_probe(cands)
        finally: self.ident.stop_confidence = old_stop
        return chosen

    def _build(self, prefix, ctx, common_obstacles, fov, cmd, plaus) -> dict:
        outcomes, safe = {}, travel_of(prefix) <= MAX_PROBE_TRAVEL
        plaus_keys = {h.key for h, _ in plaus}
        for h in self.ident.hypotheses:
            sim, _ = _believed_world(h, ctx["pose"], ctx["raw"], common_obstacles, fov)
            errs = _run_commands(sim, prefix, cmd)
            outcomes[h.key] = view_label(sim.pose(), sim.describe_view(), self.known_labels)
            if errs and h.key in plaus_keys: safe = False
        p = make_probe("plan_prefix:" + "+".join(a["tool"] for a in prefix), safe, outcomes)
        p["cost"] = travel_of(prefix) + 0.5 * sum(a["tool"] == "turn" for a in prefix)
        p["actions"] = prefix
        return p

    def finish_probe(self, probe: dict, world) -> None:
        """Read the real outcome after the runner executed probe['actions'], and update the belief."""
        pose = world.execute("get_pose", {})["data"]; raw = world.execute("describe_view", {})["data"]
        label = view_label(pose, raw, self.known_labels, self.noise_p, self.rng)
        self.task_probes += 1; self.task_probe_travel += travel_of(probe["actions"]); self._cmd_override = None
        n_before = len(self.ident.history)
        self.ident.update(probe, label)
        if self.ident.history[n_before:] and self.ident.history[-1].get("surprise"): self.task_probe_failures += 1

    # -- per-command evidence while executing the (already verified) plan
    def observe_command(self, pre_pose, hw_tool, hw_args, post_pose, result) -> bool:
        """Pose-only evidence from every turn / move.  Returns True if the MAP belief changed."""
        if not self.task_probe_enabled or self.suspend_cmd_updates or hw_tool not in ("turn", "move_forward") or not result["ok"]: return False
        if (hw_tool == "turn" and hw_args.get("degrees") == 180) or (hw_tool == "move_forward" and hw_args.get("distance") == 0): return False
        before = self.belief
        outcomes = {h.key: ",".join(map(str, predict_pose(pre_pose, hw_tool, hw_args, h))) for h in self.ident.hypotheses}
        self.ident.update(make_probe("cmd", True, outcomes), f'{post_pose["x"]},{post_pose["y"]},{post_pose["yaw_deg"]}')
        self.cmd_updates += 1
        return self.belief != before

    def on_failure(self, world) -> bool: return False
    def observe_motion(self, *a, **k) -> bool: return False

    def card(self) -> dict[str, Any]:
        b = self.belief
        return {"camera_mirrored": b.mirrored_camera, "left_right_swapped": b.left_right_swapped, "move_gain": round(b.move_gain, 3),
                "turn_gain": round(b.turn_gain, 3), "confidence": round(self.confidence(), 3),
                "status": "identified" if self.confidence() >= self.stop else "ambiguous",
                "probes_used": self.probes + self.task_probes, "n_plausible": len(self.plausible())}
