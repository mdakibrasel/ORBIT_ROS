"""Navigation macro tools (v0.11, offered with `run_spatial.py --nav-tools`).

goto_cell {"x": int, "y": int}                         drive to a map cell along a shortest path through cells not known to be blocked
cover_region {"side": "left|right|front|back|all",      clean every reachable free cell of that half of the room (halves are relative to the
              "avoid": [[x, y], ...] (optional)}         heading at the START of the episode), in a boustrophedon (row by row) order

The runtime expands a macro into primitive calls (turn / move_forward / clean) from the CURRENT pose and the obstacles known at that moment,
so a macro is one decision of the model but many robot actions. This is the "coverage planner as a tool" arm of the comparison with
LLM-written sweeps; the model still has to choose the right half (frame of reference) and the cells to avoid."""
from collections import deque
from .tasks2d import side_region

MACROS = {"goto_cell", "cover_region"}
SIDES = ("left", "right", "front", "back", "all")
TOOL_TEXT = (' | {"tool":"goto_cell","args":{"x":int,"y":int}} (drives to that map cell on a shortest known-free path)'
             ' | {"tool":"cover_region","args":{"side":"left|right|front|back|all","avoid":[[x,y],...]}} (cleans every reachable cell of that half of '
             'the room, halves relative to your heading at the START of the episode; avoid = cells never to enter)')

def bfs(start, goals, blocked, w, h):
    q, prev = deque([start]), {start: None}
    while q:
        c = q.popleft()
        if c in goals:
            path = []
            while c is not None: path.append(c); c = prev[c]
            return path[::-1]
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            n = (c[0] + dx, c[1] + dy)
            if 0 <= n[0] < w and 0 <= n[1] < h and n not in blocked and n not in prev: prev[n] = c; q.append(n)
    return None

def path_actions(yaw, path, clean_cells=frozenset()):
    acts, run = [], 0
    def flush():
        nonlocal run
        if run: acts.append({"tool": "move_forward", "args": {"distance": run}}); run = 0
    for a, b in zip(path, path[1:]):
        want = {(1, 0): 0, (0, 1): 90, (-1, 0): 180, (0, -1): 270}[(b[0] - a[0], b[1] - a[1])]
        d = (want - yaw) % 360
        if d:
            flush(); acts.append({"tool": "turn", "args": {"direction": "right" if d == 270 else "left", "degrees": 180 if d == 180 else 90}}); yaw = want
        run += 1
        if b in clean_cells: flush(); acts.append({"tool": "clean", "args": {}})
    flush()
    return acts, yaw

def _cells(v):
    try: return {(int(c[0]), int(c[1])) for c in (v or [])}
    except (TypeError, ValueError, IndexError): return None

def expand(tool, args, pose, start_pose, obstacles, cleaned=frozenset()):
    """-> (primitive actions, error message or None)."""
    w, h, me, yaw = pose["room_w"], pose["room_h"], (pose["x"], pose["y"]), pose["yaw_deg"]
    blocked = {tuple(c) for c in obstacles}
    if tool == "goto_cell":
        try: t = (int(args["x"]), int(args["y"]))
        except (KeyError, TypeError, ValueError): return [], "goto_cell needs integer x and y"
        if not (0 <= t[0] < w and 0 <= t[1] < h): return [], f"cell {t} is outside the room"
        if t in blocked: return [], f"cell {t} is known to be blocked"
        path = bfs(me, {t}, blocked, w, h)
        return (path_actions(yaw, path)[0], None) if path else ([], f"no known free path to {t}")
    if tool == "cover_region":
        side, avoid = args.get("side"), _cells(args.get("avoid"))
        if side not in SIDES: return [], "cover_region needs side left|right|front|back|all"
        if avoid is None: return [], "avoid must be a list of [x, y] cells"
        region = {(x, y) for x in range(w) for y in range(h)} if side == "all" else side_region(w, h, tuple(start_pose), side)
        blocked = blocked | (avoid - {me})
        todo = [c for c in region if c not in blocked and c not in cleaned]
        rows = sorted(todo, key=lambda c: (c[1], c[0] if c[1] % 2 == 0 else -c[0]))
        if rows and abs(rows[-1][0] - me[0]) + abs(rows[-1][1] - me[1]) < abs(rows[0][0] - me[0]) + abs(rows[0][1] - me[1]): rows = rows[::-1]
        acts, cur, cy = [], me, yaw
        if me in set(rows): acts.append({"tool": "clean", "args": {}})
        for c in rows:
            if c == me: continue
            path = bfs(cur, {c}, blocked, w, h)
            if not path: continue
            a, cy = path_actions(cy, path, clean_cells={c}); acts += a; cur = c
        return acts, None
    return [], f"unknown macro {tool}"
