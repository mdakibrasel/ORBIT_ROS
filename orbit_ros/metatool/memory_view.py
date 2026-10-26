"""Consolidated memory view for the meta-tool planner (Phase 17, feature 10). Pure code: no robot motion, no LLM.

Provides the planner with a RELEVANT SCENE SUBGRAPH and CONSOLIDATED FACTS instead of a raw event log:

  * Long-term structure (deterministic, from house.py): house -> room -> furniture/object hierarchy, plus any
    user-taught places found in the memory store (read-only).
  * Short-term memory: the most recently observed objects (label, position, room, age, source), de-duplicated by
    label with a bounded capacity and a most-recent replacement policy.
  * Consolidated facts: a compact, human-readable brief distilled from the store (recent failures/refusals near
    here, what worked), NOT the raw rows.

Safety invariants (unchanged): memory is ADVISORY ONLY. Current live observations take precedence over stale memory,
and the safety guard never relies on this view. This module only READS existing data; it does not migrate or write
the memory schema (the existing v2 store already carries events/episodes/strategies)."""
from __future__ import annotations
import time

SHORT_TERM_CAPACITY = 8          # bounded short-term memory (most-recent replacement, per distinct label)


def scene_subgraph(room=None):
    """Deterministic house->room->items hierarchy from house.py. If room is given, return just that room's subgraph."""
    from orbit_ros import house
    rooms = {}
    for r in house.ROOMS:
        rooms[r] = {"items": []}
    for it in house.ITEMS:
        rm = house.room_of(it["x"], it["y"])
        if rm in rooms:
            rooms[rm]["items"].append({"label": it["label"], "x": round(it["x"], 2), "y": round(it["y"], 2)})
    for r in rooms:
        rooms[r]["items"].sort(key=lambda d: d["label"])
    if room is not None:
        return {"house": {room: rooms.get(room, {"items": []})}}
    return {"house": rooms}


def short_term(memory, sim="webots", now=None, capacity=SHORT_TERM_CAPACITY):
    """Most-recent observed objects from the event store, de-duplicated by label (bounded capacity).

    Returns a list of {label, x, y, room, age_s, source}. Empty when there is no memory. Read-only."""
    if memory is None:
        return []
    now = now if now is not None else time.time()
    rows = []
    try:
        for r in memory._rows(
            "SELECT t, label, x, y, room, note FROM events WHERE sim=? AND kind='observation' AND label IS NOT NULL "
            "ORDER BY t DESC", (sim,)):
            rows.append(r)
    except Exception:
        return []
    seen, out = set(), []
    for r in rows:
        label = r["label"]
        if label in seen:
            continue
        seen.add(label)
        out.append({"label": label, "x": r["x"], "y": r["y"], "room": r["room"],
                    "age_s": round(now - r["t"], 1), "source": (r["note"] or "observation")})
        if len(out) >= capacity:
            break
    return out


def consolidated(memory, sim="webots", room=None):
    """A compact consolidated view for the planner: scene subgraph + short-term observations + a one-line brief.

    This is what the planner should see; it is NOT the raw event log."""
    view = {
        "scene": scene_subgraph(room),
        "short_term": short_term(memory, sim=sim),
        "taught_places": _taught_places(memory),
    }
    view["brief"] = _brief(view)
    return view


def _taught_places(memory):
    if memory is None:
        return []
    try:
        return [r["name"] for r in memory._rows("SELECT name FROM places")]
    except Exception:
        return []


def _brief(view):
    rooms = list(view["scene"].get("house", {}).keys())
    n_items = sum(len(v["items"]) for v in view["scene"].get("house", {}).values())
    st = view.get("short_term") or []
    parts = [f"{len(rooms)} rooms, {n_items} known items (remembered; current sensors win)"]
    if st:
        parts.append("recently seen: " + ", ".join(f"{o['label']}" for o in st[:4]))
    if view.get("taught_places"):
        parts.append("taught places: " + ", ".join(view["taught_places"][:4]))
    return "; ".join(parts)


def memory_hit(view):
    """True if the consolidated view carried any remembered observation or taught place (for hit-rate metrics)."""
    return bool((view.get("short_term") or []) or (view.get("taught_places") or []))
