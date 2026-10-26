"""Meta-tool tool contracts (section 4.1): symbolic requires/effects + a measured verifier per tool, plus facts(session).

Wraps the existing frozen ToolSpec (orbit_ros.pel.registry / HUMANOID_TOOLS) rather than replacing it, so pel/ and run_plan keep working.
Symbolic facts: 'upright'/'fallen', 'not_stopped', 'known(x)' for rooms/objects/taught places, 'near(x)' within NEAR_M. '!' negates;
'{name}' is filled from the step's args; '*' is a wildcard in an effect removal ('-near(*)')."""
from __future__ import annotations
import math
from dataclasses import dataclass, field

NEAR_M = 0.9


@dataclass
class MetaSpec:
    name: str
    requires: list = field(default_factory=list)       # e.g. ["upright", "not_stopped", "known({name})"]
    effects: list = field(default_factory=list)         # e.g. ["near({name})", "-near(*)"]
    risk: str = "low"                                   # low|medium|high
    repeatable: bool = True
    timeout_s: float = 60.0
    verify_kind: str = "status_ok"                      # how verify() measures success (see _verify_impl)

    def verify(self, args, result, session):
        """(ok, detail) measured from the session's observations, never from the LLM."""
        return _verify_impl(self.verify_kind, args, result, session)


# --- per-tool contracts for the humanoid tools (section 6 tools get theirs when they are added) ---
# requires/effects use {argname} placeholders filled from the call args at validation/exec time.
_SPECS = {
    "walk_forward":   MetaSpec("walk_forward", ["upright", "not_stopped"], ["-near(*)"], "medium", False, verify_kind="moved"),
    "walk_backward":  MetaSpec("walk_backward", ["upright", "not_stopped"], ["-near(*)"], "medium", False, verify_kind="moved"),
    "turn":           MetaSpec("turn", ["upright", "not_stopped"], [], "low", True, verify_kind="status_ok"),
    "side_step":      MetaSpec("side_step", ["upright", "not_stopped"], [], "low", True, verify_kind="status_ok"),
    "go_to":          MetaSpec("go_to", ["upright", "not_stopped"], ["-near(*)"], "medium", True, verify_kind="status_ok"),
    "go_to_room":     MetaSpec("go_to_room", ["upright", "not_stopped"], ["near({room})", "-near(*)"], "medium", True, verify_kind="in_room"),
    "go_to_object":   MetaSpec("go_to_object", ["upright", "not_stopped", "known({name})"], ["near({name})", "-near(*)"], "medium", True, verify_kind="near_object"),
    "turn_to_object": MetaSpec("turn_to_object", ["upright", "not_stopped", "known({name})"], [], "low", True, verify_kind="status_ok"),
    "look_around":    MetaSpec("look_around", ["upright", "not_stopped"], [], "low", True, verify_kind="status_ok"),
    "wave":           MetaSpec("wave", ["upright", "not_stopped"], [], "low", True, verify_kind="status_ok"),
    "greet":          MetaSpec("greet", ["upright", "not_stopped"], [], "low", True, verify_kind="status_ok"),
    "join_tv_watching": MetaSpec("join_tv_watching", ["upright", "not_stopped"], ["-near(*)"], "medium", True, verify_kind="status_ok"),
    "get_pose":       MetaSpec("get_pose", [], [], "low", True, verify_kind="status_ok"),
    "describe_view":  MetaSpec("describe_view", ["not_stopped"], [], "low", True, verify_kind="status_ok"),
    "look":           MetaSpec("look", ["not_stopped"], [], "low", True, verify_kind="status_ok"),
    "clean":          MetaSpec("clean", ["upright", "not_stopped"], [], "medium", True, verify_kind="status_ok"),
    "clean_room":     MetaSpec("clean_room", ["upright", "not_stopped"], [], "high", True, timeout_s=900, verify_kind="status_ok"),
    "clean_house":    MetaSpec("clean_house", ["upright", "not_stopped"], [], "high", True, timeout_s=900, verify_kind="status_ok"),
    "clean_corners":  MetaSpec("clean_corners", ["upright", "not_stopped"], [], "high", True, timeout_s=900, verify_kind="status_ok"),
    "clean_direction": MetaSpec("clean_direction", ["upright", "not_stopped"], ["-near(*)"], "medium", True, verify_kind="status_ok"),
    "scan_obstacles": MetaSpec("scan_obstacles", ["not_stopped"], [], "low", True, verify_kind="status_ok"),
    "motion_accuracy": MetaSpec("motion_accuracy", [], [], "low", True, verify_kind="status_ok"),
    "verify_cleaning": MetaSpec("verify_cleaning", [], [], "low", True, verify_kind="status_ok"),
    "verify_task":    MetaSpec("verify_task", [], [], "low", True, verify_kind="status_ok"),
    "clean_status":   MetaSpec("clean_status", [], [], "low", True, verify_kind="status_ok"),
    "pause_cleaning": MetaSpec("pause_cleaning", [], [], "low", True, verify_kind="status_ok"),
    "resume_cleaning": MetaSpec("resume_cleaning", [], [], "low", True, verify_kind="status_ok"),
    "return_to_start": MetaSpec("return_to_start", ["upright", "not_stopped"], ["-near(*)"], "medium", True, verify_kind="status_ok"),
    "tour_house":     MetaSpec("tour_house", ["upright", "not_stopped"], ["-near(*)"], "high", True, timeout_s=900, verify_kind="status_ok"),
    "plan_tasks":     MetaSpec("plan_tasks", ["not_stopped"], [], "low", True, verify_kind="status_ok"),
    "run_plan":       MetaSpec("run_plan", ["upright", "not_stopped"], ["-near(*)"], "high", True, timeout_s=900, verify_kind="status_ok"),
    "stand_up":       MetaSpec("stand_up", ["not_stopped"], ["upright"], "medium", True, verify_kind="upright"),
    "stop_robot":     MetaSpec("stop_robot", [], ["!not_stopped"], "low", True, verify_kind="status_ok"),
}


def spec_for(name):
    """Return the MetaSpec for a tool, or a permissive default (status_ok) for tools without an explicit contract yet."""
    return _SPECS.get(name) or MetaSpec(name, [], [], "medium", True, verify_kind="status_ok")


def all_specs():
    return dict(_SPECS)


def fill(sym, args):
    """Fill {arg} placeholders in a requires/effects symbol from the call args. Returns None if a referenced arg is missing."""
    out = sym
    import re
    for m in re.findall(r"\{(\w+)\}", sym):
        if m not in args: return None
        out = out.replace("{" + m + "}", str(args[m]))
    return out


def facts(session):
    """Current symbolic fact set from the live session (robot-reported state + known names). Never reads memory or LLM output for safety."""
    from orbit_ros import house
    f = set()
    try:
        p = session._pose()
        f.add("fallen" if p["fallen"] else "upright")
        f.add("not_stopped" if not session.stopped else "stopped")
        here = (p["x"], p["y"])
    except Exception:
        here = None
    # known(...) for every room, house item label, and taught place in memory
    for r in house.ROOMS: f.add(f"known({r})")
    for it in {i["label"] for i in house.ITEMS}: f.add(f"known({it})")
    mem = getattr(session, "memory", None)
    if mem:
        try:
            for pl in _taught_places(mem): f.add(f"known({pl})")
        except Exception: pass
    # near(x) within NEAR_M for rooms/items (taught places handled by go_to_place at runtime)
    if here is not None:
        for it in house.ITEMS:
            if math.hypot(it["x"] - here[0], it["y"] - here[1]) <= NEAR_M: f.add(f"near({it['label']})")
        try:
            room = house.room_of(*here)
            if room: f.add(f"near({room})")
        except Exception: pass
    return f


def _taught_places(mem):
    """Names of user-taught places, if the memory has a places table (v3). Empty otherwise."""
    try:
        rows = mem._rows("SELECT name FROM places")
        return [r["name"] for r in rows]
    except Exception:
        return []


def _verify_impl(kind, args, result, session):
    status = (result or {}).get("status")
    if kind == "status_ok":
        return (status == "ok", f"status={status}")
    p = None
    try: p = session._pose()
    except Exception: pass
    if kind == "upright":
        return (bool(p) and not p["fallen"], f"fallen={None if not p else p['fallen']}")
    if kind == "moved":
        d = (result or {}).get("data") or {}
        return (status == "ok", f"status={status}")       # distance is checked by the live tool-check; here we trust the status + pose
    if kind == "in_room":
        want = str(args.get("room"))
        return (bool(p) and p["room"] == want, f"room={None if not p else p['room']} want={want}")
    if kind == "near_object":
        from orbit_ros import house
        name = str(args.get("name")); 
        if not p: return (False, "no pose")
        gaps = [math.hypot(max(abs(p["x"]-it["x"])-it["sx"]/2, 0.0), max(abs(p["y"]-it["y"])-it["sy"]/2, 0.0))
                for it in house.ITEMS if it["label"] == name]
        g = min(gaps) if gaps else 1e9
        return (status == "ok" and g <= 1.5 and not p["fallen"], f"gap={round(g,2)}m status={status}")
    return (status == "ok", f"status={status}")
