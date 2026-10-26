"""Embodiment identification runtime: calibration bay, probe library, command adapter, strategies.

The robot's hidden embodiment is a Hypothesis (mirrored camera x left/right swap x move gain x turn gain).
Strategies decide how the agent comes to *believe* one:

  nominal          no identification (assumes the factory-default robot)
  docs_sensor      documentation is right about the camera, silent about actuators
  fixed_probe      EnvProbe-style analogue: fixed probe order, fixed budget, no information-gain selection
  random_probe     random probe order, stops when confident
  post_failure     CoRAL-style analogue: runs nominal; identifies only after the first failure signal
  kinematic_cal    classical-calibration analogue: continuous gain / polarity from odometry; no camera-convention model
  infogain         ours: information-gain probe choice, stops at confidence >= 0.95, one safe probe at a time
  infogain_monitor ours + surprise monitor that re-identifies when actuation surprises persist
  oracle           ground truth (upper bound)

All of these are simplified in-simulator analogues, NOT reproductions of the published systems.
Probes run in a CalibrationBay (an empty, fenced room with a fixture of known position) built with the same
hidden embodiment as the real robot, so probing never disturbs the task world. Every probe is dry-run under
every still-plausible hypothesis first; if any hypothesis predicts a collision / out-of-bounds, it is not safe.
"""
from __future__ import annotations
import random
from dataclasses import dataclass, field
from typing import Any
from .world2d import World2D, Obj, heading
from .embodiment import Hypothesis, EmbodimentIdentifier, SurpriseMonitor, make_probe

MOVE_GAINS = (0.25, 0.5, 1.0)   # under-travel only: over-travel needs sub-cell resolution the grid world lacks
TURN_GAINS = (1.0,)             # yaw is quantised to 90 deg, so fractional turn gain is unobservable here
NOMINAL = Hypothesis(False, False, 1.0, 1.0)
MAX_PROBE_TRAVEL = 16           # safety amplitude cap (metres of commanded travel)
BAY_W, BAY_H, BAY_START, BAY_MARKER = 21, 9, (2, 4), (5, 5)

def hypothesis_space() -> list[Hypothesis]:
    return [Hypothesis(m, s, g, t) for m in (False, True) for s in (False, True) for g in MOVE_GAINS for t in TURN_GAINS]

def hypothesis_of(world) -> Hypothesis:
    return Hypothesis(bool(world.mirrored), bool(world.swapped), float(world.move_gain), float(world.turn_gain))

def apply_embodiment(world, h: Hypothesis):
    world.mirrored, world.swapped, world.move_gain, world.turn_gain = h.mirrored_camera, h.left_right_swapped, h.move_gain, h.turn_gain
    return world

BAY_HOOK = None   # optional callable(World2D) -> World2D; the Webots runner uses it to run the REAL fixture in the simulator

def make_real_bay(h: Hypothesis) -> World2D:
    """The bay the actual robot drives in (identification runs). Dry runs under hypotheses use make_bay directly and stay pure Python."""
    bay = make_bay(h)
    return BAY_HOOK(bay) if BAY_HOOK is not None else bay

def make_bay(h: Hypothesis) -> World2D:
    bay = World2D(BAY_W, BAY_H, BAY_START, 0, [Obj("m", "calibration marker", *BAY_MARKER)])
    return apply_embodiment(bay, h)

def _act(tool, **a): return {"tool": tool, "args": a}

# name -> (actions, cost). cost = commanded metres + 0.5 per turn.
PROBES = {
    "look":   ([_act("describe_view")], 0.0),
    "turn90": ([_act("turn", direction="left", degrees=90)], 0.5),
    "fwd8":   ([_act("move_forward", distance=8)], 8.0),
    "tour":   ([_act("describe_view"), _act("move_forward", distance=8), _act("turn", direction="left", degrees=90),
                _act("move_forward", distance=4)], 12.5),
}
ATOMIC = ("look", "turn90", "fwd8")

def run_probe(bay: World2D, actions, noise_p: float = 0.0, rng: random.Random | None = None) -> str:
    """Execute a probe in a bay; the label is what the robot can observe: final pose + image-side of the fixture."""
    views = []
    for a in actions:
        r = bay.execute(a["tool"], a["args"])
        if a["tool"] == "describe_view":
            sides = []
            for o in r["data"]:
                side = "L" if o["image_x"] < 0 else ("R" if o["image_x"] > 0 else "C")
                if noise_p and rng is not None and rng.random() < noise_p and side != "C": side = "R" if side == "L" else "L"
                sides.append(f'{o["label"][:3]}:{side}')
            views.append(",".join(sides) or "-")
    p = bay.pose()
    return f"{p['x']},{p['y']},{p['yaw_deg']}|{';'.join(views)}"

def travel_of(actions) -> float:
    return float(sum(a["args"].get("distance", 0) for a in actions if a["tool"] == "move_forward"))

def build_probe(name: str, hyps) -> dict[str, Any]:
    """Probe contract with per-hypothesis predicted labels and a computed (not asserted) safety flag."""
    actions, cost = PROBES[name]
    outcomes, safe = {}, travel_of(actions) <= MAX_PROBE_TRAVEL
    for h in hyps:
        w = make_bay(h)
        outcomes[h.key] = run_probe(w, actions)
        if w.collisions or w.invalid_calls: safe = False        # dry-run verifier: no hypothesis may predict a failure
    p = make_probe(name, safe, outcomes)
    p["cost"], p["actions"] = cost, actions
    return p

def _deviation(h: Hypothesis) -> float:
    return h.mirrored_camera + h.left_right_swapped + abs(h.move_gain - 1) + abs(h.turn_gain - 1)

@dataclass
class IdResult:
    belief: Hypothesis
    confidence: float
    probes: int = 0
    bay_actions: int = 0
    bay_travel: float = 0.0
    status: str = "none"
    log: list = field(default_factory=list)

    def card(self) -> dict[str, Any]:
        """Embodiment Card: small, readable, token-cheap, shareable between robots."""
        b = self.belief
        return {"camera_mirrored": b.mirrored_camera, "left_right_swapped": b.left_right_swapped,
                "move_gain": round(b.move_gain, 3), "turn_gain": round(b.turn_gain, 3),
                "confidence": round(self.confidence, 3), "status": self.status, "probes_used": self.probes}

def _new_identifier(max_probes, eps, cost_weight, stop):
    return EmbodimentIdentifier(mirror=(False, True), swapped=(False, True), move_gains=MOVE_GAINS, turn_gains=TURN_GAINS,
                                max_probes=max_probes, epsilon=eps, cost_weight=cost_weight, stop_confidence=stop)

def _finish(ident, res: IdResult) -> IdResult:
    card = ident.card()
    ranked = sorted(ident.hypotheses, key=lambda h: (-ident.weights[h.key], _deviation(h)))
    top = ranked[0]
    res.belief, res.confidence, res.probes, res.status = top, ident.weights[top.key], ident.probes_used, card.status
    return res

def _execute(true_h, name, res, ident, rng, noise_p):
    probe = build_probe(name, ident.hypotheses)
    if not probe["safe"]: raise RuntimeError(f"probe {name} is not safe under the current hypothesis set")
    label = run_probe(make_real_bay(true_h), probe["actions"], noise_p, rng)
    ident.update(probe, label)
    res.bay_actions += len(probe["actions"]); res.bay_travel += travel_of(probe["actions"])
    res.log.append({"probe": name, "label": label})

def identify_infogain(true_h, eps=0.0, noise_p=0.0, max_probes=4, cost_weight=0.0, stop=0.95, rng=None, names=None):
    rng = rng or random.Random(0)
    ident, res = _new_identifier(max_probes, eps, cost_weight, stop), IdResult(NOMINAL, 0.0)
    for _ in range(2 * max_probes):
        probe = ident.choose_probe([build_probe(n, ident.hypotheses) for n in (names or PROBES)])
        if probe is None: break
        _execute(true_h, probe["name"], res, ident, rng, noise_p)
    return _finish(ident, res)

def identify_fixed(true_h, eps=0.0, noise_p=0.0, rng=None, order=ATOMIC):
    """EnvProbe-style analogue: fixed hand-chosen probe order, always spends the whole budget."""
    rng = rng or random.Random(0)
    ident, res = _new_identifier(len(order), eps, 0.0, 0.95), IdResult(NOMINAL, 0.0)
    for n in order: _execute(true_h, n, res, ident, rng, noise_p)
    return _finish(ident, res)

def identify_random(true_h, eps=0.0, noise_p=0.0, rng=None, max_probes=4, stop=0.95):
    rng = rng or random.Random(0)
    ident, res = _new_identifier(max_probes, eps, 0.0, stop), IdResult(NOMINAL, 0.0)
    pool = list(PROBES)
    while ident.probes_used < max_probes and ident.confidence() < stop:
        _execute(true_h, rng.choice(pool), res, ident, rng, noise_p)
    return _finish(ident, res)

def identify_kinematic(true_h, **_):
    """Classical-calibration analogue: continuous gain from odometry ratio, polarity from yaw sign.
    No model of camera conventions, so it can never discover a mirrored camera."""
    res = IdResult(NOMINAL, 1.0, probes=2, status="estimated")
    b1 = make_real_bay(true_h); b1.execute("move_forward", {"distance": 8}); gain = (b1.x - BAY_START[0]) / 8
    b2 = make_real_bay(true_h); b2.execute("turn", {"direction": "left", "degrees": 90})
    res.bay_actions, res.bay_travel = 2, 8.0
    res.belief = Hypothesis(False, b2.yaw == 270, gain, 1.0)
    return res

IDENTIFIERS = {"infogain": identify_infogain, "fixed_probe": identify_fixed, "random_probe": identify_random,
               "kinematic_cal": identify_kinematic}

# ------------------------------------------------------------------ command adapter
def _chunks(c, size=12):
    while c > 0: yield min(c, size); c -= size

def _realised(c, g): return sum(int(k * g + 0.5) for k in _chunks(c))

class EmbodimentAdapter:
    """Translates the agent's nominal commands into hardware commands under the believed embodiment."""
    def __init__(self, belief: Hypothesis): self.belief = belief

    def translate(self, tool, args) -> list[tuple[str, dict]]:
        b = self.belief
        if tool == "turn" and b.left_right_swapped and args.get("direction") in ("left", "right"):
            return [("turn", dict(args, direction="right" if args["direction"] == "left" else "left"))]
        if tool == "move_forward" and b.move_gain != 1.0:
            n = args.get("distance")
            if not isinstance(n, int) or isinstance(n, bool) or n < 0: return [(tool, args)]
            c = min(range(0, int(n / b.move_gain) + 6), key=lambda c: (abs(_realised(c, b.move_gain) - n), c))
            return [("move_forward", {"distance": k}) for k in _chunks(c)] or [("move_forward", {"distance": 0})]
        return [(tool, args)]

def predict_pose(pose, tool, args, belief: Hypothesis):
    """Pose the believed hardware should reach after one *hardware* command (walls/obstacles ignored)."""
    x, y, yaw = pose["x"], pose["y"], pose["yaw_deg"]
    if tool == "turn":
        d = args["direction"]
        signed = args["degrees"] if d == "left" else -args["degrees"]
        if belief.left_right_swapped: signed = -signed
        return (x, y, (yaw + signed) % 360)
    if tool == "move_forward":
        dx, dy = heading(yaw); n = int(args["distance"] * belief.move_gain + 0.5)
        return (x + dx * n, y + dy * n, yaw)
    return None

# ------------------------------------------------------------------ strategy
class EmbodimentStrategy:
    def __init__(self, name: str, eps=0.0, noise_p=0.0, monitor_threshold: int | None = None, seed: int = 0, cost_weight=0.0):
        if name not in STRATEGY_NAMES: raise ValueError(f"unknown strategy {name}")
        self.name, self.eps, self.noise_p, self.cost_weight = name, eps, noise_p, cost_weight
        self.rng = random.Random(f"{name}|{seed}")
        if name == "infogain_monitor" and monitor_threshold is None: monitor_threshold = 2
        self.monitor_threshold = monitor_threshold
        self.monitors: dict[str, SurpriseMonitor] = {}      # one per actuation channel (turn / move)
        self.monitor = bool(monitor_threshold)
        self.belief, self.result = NOMINAL, IdResult(NOMINAL, 0.0)
        self.probes = self.bay_actions = 0; self.bay_travel = 0.0
        self.reids = self.triggers = 0
        self.identified_once = False

    def _identify(self, world):
        truth = hypothesis_of(world)
        kw = dict(eps=self.eps, noise_p=self.noise_p, rng=self.rng)
        fn = {"infogain": identify_infogain, "infogain_monitor": identify_infogain, "post_failure": identify_infogain,
              "fixed_probe": identify_fixed, "random_probe": identify_random, "kinematic_cal": identify_kinematic}[self.name]
        if fn is identify_infogain: kw["cost_weight"] = self.cost_weight
        res = fn(truth, **kw)
        self.result, self.belief = res, res.belief
        self.probes += res.probes; self.bay_actions += res.bay_actions; self.bay_travel += res.bay_travel
        self.identified_once = True
        for m in self.monitors.values(): m.consecutive = 0

    def start(self, world):
        truth = hypothesis_of(world)
        if self.name == "nominal" or self.name == "post_failure": self.belief = NOMINAL
        elif self.name == "docs_sensor": self.belief = Hypothesis(truth.mirrored_camera, False, 1.0, 1.0)
        elif self.name == "oracle": self.belief = truth
        else: self._identify(world)

    def on_failure(self, world) -> bool:
        """CoRAL-style: identify once, after the first failure signal. Returns True if the belief changed."""
        if self.name == "post_failure" and not self.identified_once:
            self.reids += 1; self._identify(world); return True
        return False

    def observe_motion(self, world, hw_tool, hw_args, pre_pose, result) -> bool:
        """Surprise monitor on actuation. Returns True when persistent surprise triggered re-identification."""
        if not self.monitor or hw_tool not in ("turn", "move_forward") or not result["ok"]: return False
        # A 180-degree turn looks identical under either polarity, a zero-length move under any gain: no information.
        if (hw_tool == "turn" and hw_args.get("degrees") == 180) or (hw_tool == "move_forward" and hw_args.get("distance") == 0): return False
        pred = predict_pose(pre_pose, hw_tool, hw_args, self.belief)
        mon = self.monitors.setdefault(hw_tool, SurpriseMonitor(self.monitor_threshold))
        p = world.pose(); ev = mon.observe(pred, (p["x"], p["y"], p["yaw_deg"]), hw_tool)
        if ev["trigger"]:
            self.triggers += 1; self.reids += 1; self._identify(world); return True
        return False

    def card(self) -> dict[str, Any]:
        c = self.result.card() if self.result.probes or self.identified_once else {
            "camera_mirrored": self.belief.mirrored_camera, "left_right_swapped": self.belief.left_right_swapped,
            "move_gain": round(self.belief.move_gain, 3), "turn_gain": round(self.belief.turn_gain, 3),
            "confidence": None, "status": "assumed", "probes_used": 0}
        return c

STRATEGY_NAMES = ("nominal", "docs_sensor", "fixed_probe", "random_probe", "post_failure", "kinematic_cal",
                  "infogain", "infogain_monitor", "oracle")
