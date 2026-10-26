"""Agent-side modules: sensor interpreter, landmark memory, dry-run plan verifier."""
from __future__ import annotations
import math, random
from .world2d import World2D, Obj, ENV_TOOLS, heading

AGENT_TOOLS = ENV_TOOLS | {"remember", "ask", "abstain"}    # ask / abstain: agent-level (v0.11), never sent to the robot

class SensorInterpreter:
    """Turns raw camera values into egocentric semantics (bearing, distance, world cell).
    grounded=True uses the true camera calibration (FOV, mirroring, size->distance).
    grounded=False mimics an agent that assumes a standard, unmirrored camera and has no metric scale
    (relative distance noise sigma_rel)."""
    def __init__(self, fov=90.0, mirrored=False, grounded=True, sigma_rel=0.25, rng=None):
        self.fov, self.mirrored, self.grounded = fov, mirrored, grounded
        self.sigma, self.rng = sigma_rel, rng or random.Random(0)

    def interpret(self, raw, pose):
        out = []
        for r in raw:
            ix = r["image_x"]
            if self.grounded and self.mirrored: ix = -ix
            bearing = -math.degrees(math.atan(ix * math.tan(math.radians(self.fov / 2))))
            dist = 1.0 / max(r["apparent_size"], 1e-6)
            if not self.grounded: dist *= max(0.2, 1 + self.rng.gauss(0, self.sigma))
            yaw = math.radians(pose["yaw_deg"] + bearing)
            wx, wy = pose["x"] + dist * math.cos(yaw), pose["y"] + dist * math.sin(yaw)
            out.append({"label": r["label"], "bearing_deg": round(bearing, 1), "distance_m": round(dist, 2),
                        "world_xy": [round(wx), round(wy)]})
        return out

class Memory:
    """Persistent landmark store (world frame) + record of cleaned cells."""
    def __init__(self): self.landmarks: dict[str, list[int]] = {}; self.cleaned: set = set(); self.seen: set = set()
    def remember(self, label, interp):
        m = [o for o in interp if o["label"] == label]
        if not m: return False
        self.landmarks[label] = min(m, key=lambda o: o["distance_m"])["world_xy"]; return True
    def to_dict(self): return {"landmarks": self.landmarks, "n_cleaned": len(self.cleaned)}

def dry_run(actions, pose, believed_obstacles, start_pose=None):
    """Execute a plan in a believed copy of the world. Returns (final_pose, cleaned_cells, errors).
    Macro tools (goto_cell, cover_region) are expanded exactly as the runtime would expand them at that point of the plan."""
    from .navtools import MACROS, expand
    sim = World2D(pose["room_w"], pose["room_h"], (pose["x"], pose["y"]), pose["yaw_deg"],
                  [Obj(f"b{i}", "believed", *c) for i, c in enumerate(believed_obstacles)])
    errors = []; start_pose = start_pose or (pose["x"], pose["y"], pose["yaw_deg"])
    for i, a in enumerate(actions):
        if a.get("tool") in MACROS:
            prims, err = expand(a["tool"], a.get("args") or {}, sim.pose(), start_pose, believed_obstacles, sim.cleaned)
            if err: errors.append(f"action[{i}] {a['tool']}: {err}"); continue
            for p in prims:
                r = sim.execute(p["tool"], p["args"])
                if not r["ok"]: errors.append(f"action[{i}] {a['tool']} -> {p['tool']}{p['args']}: {r['message']}"); break
            continue
        if a["tool"] in ("describe_view", "get_pose", "remember", "done", "ask", "abstain", "report", "localize"): continue
        if a["tool"] not in AGENT_TOOLS: errors.append(f"action[{i}]: unknown tool {a['tool']}"); continue
        r = sim.execute(a["tool"], a.get("args", {}))
        if not r["ok"]: errors.append(f"action[{i}] {a['tool']}{a.get('args')}: {r['message']}")
    return sim.pose(), sim.cleaned, errors

def verify_plan(plan, pose, believed_obstacles, start_pose=None):
    """Deterministic contract: tools/args valid, no predicted collision/out-of-bounds, and the dry-run final
    pose equals the plan's own declared `expect` (x, y, optional yaw_deg; optional cleaned_min)."""
    final, cleaned, errors = dry_run(plan["actions"], pose, believed_obstacles, start_pose)
    e = plan.get("expect") or {}
    for k, key in (("x", "x"), ("y", "y"), ("yaw_deg", "yaw_deg")):
        if k in e and e[k] != final[key]: errors.append(f"expect.{k}={e[k]} but dry-run gives {final[key]}")
    if "cleaned_min" in e and len(cleaned) < e["cleaned_min"]:
        errors.append(f"expect cleaned>={e['cleaned_min']} but dry-run cleans {len(cleaned)}")
    return errors
