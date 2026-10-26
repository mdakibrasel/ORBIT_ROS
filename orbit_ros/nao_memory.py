"""NAO memory (v0.16.4): a small SQLite store of what the robot saw and did, so the LLM can look at it before it plans.

Kept: actions with their outcome (ok / failed / vetoed / rejected), positions, items seen, obstacles found by a bump or the sonar.
The memory is ADVICE ONLY. It never feeds the safety guard, the planner or the obstacle list: the guard and the planner use the live pose,
the live sonar and the map, as before. Everything recalled is marked as remembered and with its age; the model must trust
get_pose / describe_view / scan_obstacles over it.

    m = NaoMemory("results_mcp/nao_memory.db")      # or ":memory:" in tests
    m.add("action", sim="webots", tool="go_to_room", args={"room": "kitchen"}, status="ok", room="kitchen", x=4.1, y=5.6)
    m.recall(sim="webots", room="kitchen", x=4.0, y=5.5)
"""
from __future__ import annotations
import json, math, os, sqlite3, threading, time

KINDS = ("action", "observation", "obstacle", "position")
SCHEMA_VERSION = 2                                 # v1 = events only; v2 adds episodes (episodic) + strategies (semantic) + meta
SCHEMA = """CREATE TABLE IF NOT EXISTS events(
    id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL NOT NULL, sim TEXT NOT NULL, kind TEXT NOT NULL, tool TEXT, args TEXT, status TEXT,
    room TEXT, x REAL, y REAL, yaw REAL, label TEXT, note TEXT);
CREATE INDEX IF NOT EXISTS ix_events_kind ON events(sim, kind, t)"""
# v2 episodic memory: one row per completed/failed TASK (not per primitive). Semantic memory: distilled, reusable strategies.
SCHEMA_V2 = """CREATE TABLE IF NOT EXISTS meta(key TEXT PRIMARY KEY, value TEXT);
CREATE TABLE IF NOT EXISTS episodes(
    id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL NOT NULL, sim TEXT NOT NULL, experiment TEXT, scenario TEXT,
    instruction TEXT, goal TEXT, start_room TEXT, start_x REAL, start_y REAL, start_yaw REAL,
    planned TEXT, executed TEXT, outcome TEXT, success INTEGER, final_room TEXT, final_x REAL, final_y REAL, final_yaw REAL,
    replans INTEGER, failed_actions INTEGER, recovery TEXT, feedback TEXT, provenance TEXT, dedup_key TEXT);
CREATE INDEX IF NOT EXISTS ix_episodes ON episodes(sim, goal, t);
CREATE TABLE IF NOT EXISTS strategies(
    id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL NOT NULL, sim TEXT NOT NULL, goal TEXT NOT NULL, tool TEXT, args TEXT,
    verified INTEGER NOT NULL DEFAULT 0, successes INTEGER NOT NULL DEFAULT 0, failures INTEGER NOT NULL DEFAULT 0,
    confidence REAL, note TEXT, provenance TEXT, UNIQUE(sim, goal, tool, args))"""
MAX_ROWS = 5000                                   # the oldest rows are dropped beyond this: the memory stays small and quick

def age_text(seconds: float) -> str:
    s = max(0, int(seconds))
    return f"{s}s ago" if s < 90 else f"{s // 60} min ago" if s < 5400 else f"{s // 3600} h ago" if s < 172800 else f"{s // 86400} d ago"

def _f(v):
    return None if v is None else float(v)

class NaoMemory:
    def __init__(self, path=":memory:", max_records=MAX_ROWS, retrieval_enabled=True, write_after_task=True):
        self.path = path
        self.max_records = int(max_records) if max_records else MAX_ROWS
        self.retrieval_enabled = bool(retrieval_enabled)
        self.write_after_task = bool(write_after_task)
        if path != ":memory:": os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        # A busy timeout + WAL make interrupted writes safe to recover (the journal is replayed on the next open).
        self.db = sqlite3.connect(path, check_same_thread=False, timeout=30)
        self.db.row_factory = sqlite3.Row
        self.lock = threading.Lock()
        with self.lock:
            try:
                if path != ":memory:": self.db.execute("PRAGMA journal_mode=WAL"); self.db.execute("PRAGMA synchronous=NORMAL")
            except sqlite3.Error: pass
            for stmt in SCHEMA.split(";\n"): self.db.execute(stmt)
            for stmt in SCHEMA_V2.split(";\n"): self.db.execute(stmt)           # migration: v1 dbs gain episodes/strategies/meta in place
            self.db.execute("INSERT OR REPLACE INTO meta(key, value) VALUES ('schema_version', ?)", (str(SCHEMA_VERSION),))
            self.db.commit()

    def schema_version(self) -> int:
        with self.lock:
            try:
                row = self.db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()
                return int(row[0]) if row else 1
            except sqlite3.Error: return 1

    def add(self, kind, sim="fake", tool=None, args=None, status=None, room=None, x=None, y=None, yaw=None, label=None, note=None, t=None):
        if kind not in KINDS: raise ValueError(f"kind must be one of {KINDS}")
        with self.lock:
            self.db.execute("INSERT INTO events(t, sim, kind, tool, args, status, room, x, y, yaw, label, note) VALUES (?,?,?,?,?,?,?,?,?,?,?,?)",
                            (t if t is not None else time.time(), sim, kind, tool, json.dumps(args, sort_keys=True) if args else None, status, room,
                             None if x is None else float(x), None if y is None else float(y), None if yaw is None else float(yaw), label, note))
            self.db.execute("DELETE FROM events WHERE id <= (SELECT MAX(id) FROM events) - ?", (self.max_records,))
            self.db.commit()

    def count(self, kind=None) -> int:
        with self.lock:
            q, a = ("SELECT COUNT(*) FROM events WHERE kind=?", (kind,)) if kind else ("SELECT COUNT(*) FROM events", ())
            return self.db.execute(q, a).fetchone()[0]

    def clear(self, episodic=True, semantic=True):
        """Explicit reset. By default clears events and (optionally) episodic + semantic memory. Caller confirms intent."""
        with self.lock:
            self.db.execute("DELETE FROM events")
            if episodic: self.db.execute("DELETE FROM episodes")
            if semantic: self.db.execute("DELETE FROM strategies")
            self.db.commit()

    def close(self):
        with self.lock: self.db.close()

    def _rows(self, sql, args=()):
        with self.lock: return [dict(r) for r in self.db.execute(sql, args).fetchall()]

    def recall(self, sim="fake", room=None, x=None, y=None, near_m=2.5, limit=5, now=None) -> dict:
        """The most relevant remembered things for the current place: recent actions and failures in this room or near this point, obstacles
        near it, items last seen, and a few recent positions. Newest first, each with its age."""
        now = time.time() if now is None else now
        limit = max(1, min(int(limit), 20))
        has_xy = x is not None and y is not None
        def near(r): return has_xy and r["x"] is not None and math.hypot(r["x"] - x, r["y"] - y) <= near_m
        def relevant(r): return (room is not None and r["room"] == room) or near(r)
        def view(r):
            d = {"age": age_text(now - r["t"]), "room": r["room"]}
            if r["x"] is not None: d["at"] = [round(r["x"], 2), round(r["y"], 2)]
            if has_xy and r["x"] is not None: d["distance_m"] = round(math.hypot(r["x"] - x, r["y"] - y), 1)
            for k in ("tool", "status", "label", "note"):
                if r[k]: d[k] = r[k]
            if r["args"]: d["args"] = json.loads(r["args"])
            return d
        def pick(kind, extra="", cap=400):
            rows = self._rows(f"SELECT * FROM events WHERE sim=? AND kind=? {extra} ORDER BY t DESC, id DESC LIMIT ?", (sim, kind, cap))
            return rows
        actions = pick("action")
        failed = [r for r in actions if r["status"] in ("failed", "vetoed", "error") and relevant(r)][:limit]
        recent = [r for r in actions if relevant(r) and r["status"] == "ok"][:limit]
        obstacles = [r for r in pick("obstacle") if relevant(r)][:limit]
        seen, items = set(), []
        for r in pick("observation"):                                      # the latest sighting of each item
            if r["label"] in seen: continue
            seen.add(r["label"])
            if relevant(r) or room is None: items.append(r)
        positions = [r for r in pick("position")][:3]
        return {
            "note": "REMEMBERED, possibly out of date. Current sensors (get_pose, describe_view, scan_obstacles) and the safety guard always win over this.",
            "room": room, "counts": {k: self.count(k) for k in KINDS},
            "recent_failures": [view(r) for r in failed], "recent_successes": [view(r) for r in recent],
            "obstacles": [view(r) for r in obstacles], "items_seen": [view(r) for r in items[:limit]],
            "recent_positions": [view(r) for r in positions]}

    # ---------------------------------------------------------------------------------------------------- episodic (per task)
    @staticmethod
    def normalize_goal(instruction: str, rooms=None, objects=None) -> str:
        """A coarse, deterministic goal key from a free-text instruction: used for task-similarity retrieval (not an LLM call).
        Maps synonyms of go/clean/greet + a room/object token to a stable key, e.g. 'go the kitchen' and 'walk to kitchen' -> 'go:kitchen'.
        `rooms`/`objects` are optional vocabularies (the session passes the house's); nao_memory stays a leaf and imports nothing from the robot."""
        s = (instruction or "").lower()
        verb = ("clean" if "clean" in s else "greet" if ("greet" in s or "wave" in s) else
                "watch_tv" if ("television" in s or " tv" in s or "watch" in s) else
                "stop" if "stop" in s else "go")
        rooms = list(rooms) if rooms else ["kitchen", "bathroom", "bedroom", "living_room"]
        tok = ""
        for r in rooms:
            if r.replace("_", " ") in s or r in s: tok = r; break
        if not tok and objects:
            for it in sorted(objects, key=len, reverse=True):
                if str(it).lower() in s: tok = it; break
        return f"{verb}:{tok}" if tok else verb

    @staticmethod
    def _dedup_key(sim, goal, start_room):
        return f"{sim}|{goal}|{start_room}"

    def add_episode(self, instruction, goal=None, sim="fake", experiment=None, scenario=None, start=None, planned=None,
                    executed=None, outcome=None, success=None, final=None, replans=0, failed_actions=0, recovery=None,
                    feedback=None, provenance=None, dedup=True, t=None) -> int:
        """Store one TASK episode (episodic memory). Returns the row id. If dedup and an episode with the same (sim, goal, start_room)
        exists, the newest row is kept and the prior one updated in place rather than piling duplicates. provenance records WHERE this
        came from (e.g. experiment/scenario/trial) so a later plan can cite it. Never store secrets here."""
        goal = goal or self.normalize_goal(instruction)
        start = start or {}; final = final or {}
        dk = self._dedup_key(sim, goal, start.get("room"))
        row = (t if t is not None else time.time(), sim, experiment, scenario, instruction, goal,
               start.get("room"), _f(start.get("x")), _f(start.get("y")), _f(start.get("yaw")),
               json.dumps(planned, default=str) if planned is not None else None,
               json.dumps(executed, default=str) if executed is not None else None,
               outcome, None if success is None else int(bool(success)),
               final.get("room"), _f(final.get("x")), _f(final.get("y")), _f(final.get("yaw")),
               int(replans or 0), int(failed_actions or 0),
               json.dumps(recovery, default=str) if recovery is not None else None,
               feedback, json.dumps(provenance, default=str) if provenance is not None else None, dk)
        with self.lock:
            if dedup:
                prev = self.db.execute("SELECT id FROM episodes WHERE dedup_key=? ORDER BY t DESC LIMIT 1", (dk,)).fetchone()
                if prev:
                    cols = ("t,sim,experiment,scenario,instruction,goal,start_room,start_x,start_y,start_yaw,planned,executed,"
                            "outcome,success,final_room,final_x,final_y,final_yaw,replans,failed_actions,recovery,feedback,provenance,dedup_key")
                    sets = ",".join(f"{c}=?" for c in cols.split(","))
                    self.db.execute(f"UPDATE episodes SET {sets} WHERE id=?", row + (prev[0],)); self.db.commit(); return prev[0]
            cur = self.db.execute(
                "INSERT INTO episodes(t,sim,experiment,scenario,instruction,goal,start_room,start_x,start_y,start_yaw,planned,executed,"
                "outcome,success,final_room,final_x,final_y,final_yaw,replans,failed_actions,recovery,feedback,provenance,dedup_key) "
                "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", row)
            self.db.execute("DELETE FROM episodes WHERE id <= (SELECT MAX(id) FROM episodes) - ?", (self.max_records,))
            self.db.commit(); return cur.lastrowid

    def recall_episodes(self, instruction=None, goal=None, sim="fake", start_room=None, limit=3, now=None) -> list:
        """Retrieve past task episodes most relevant to the current instruction/goal, newest first. Deterministic similarity:
        exact goal match > same verb > same room/object token. Each returned episode carries its age, outcome and provenance."""
        if not self.retrieval_enabled: return []
        goal = goal or (self.normalize_goal(instruction) if instruction else None)
        now = time.time() if now is None else now
        limit = max(1, min(int(limit), 20))
        rows = self._rows("SELECT * FROM episodes WHERE sim=? ORDER BY t DESC LIMIT 500", (sim,))
        verb = goal.split(":")[0] if goal else None
        tok = goal.split(":")[1] if goal and ":" in goal else None
        def score(r):
            g = r.get("goal") or ""
            s = 0
            if goal and g == goal: s += 100
            if verb and g.split(":")[0] == verb: s += 20
            if tok and (":" + tok) in g: s += 15
            if start_room and r.get("start_room") == start_room: s += 5
            return s
        scored = sorted(((score(r), r) for r in rows), key=lambda sr: (-sr[0], -(sr[1]["t"] or 0)))
        out = []
        for sc, r in scored:
            if sc <= 0: continue
            out.append({"age": age_text(now - (r["t"] or now)), "instruction": r["instruction"], "goal": r["goal"],
                        "outcome": r["outcome"], "success": None if r["success"] is None else bool(r["success"]),
                        "start_room": r["start_room"], "final_room": r["final_room"], "replans": r["replans"],
                        "failed_actions": r["failed_actions"], "feedback": r["feedback"],
                        "executed": json.loads(r["executed"]) if r["executed"] else None,
                        "provenance": json.loads(r["provenance"]) if r["provenance"] else None, "relevance": sc})
            if len(out) >= limit: break
        return out

    def count_episodes(self) -> int:
        with self.lock: return self.db.execute("SELECT COUNT(*) FROM episodes").fetchone()[0]

    # ---------------------------------------------------------------------------------------------------- semantic (strategies)
    def learn_strategy(self, goal, tool=None, args=None, success=True, sim="fake", note=None, provenance=None):
        """Distil a reusable strategy ('for goal G, tool T worked'). Upserts success/failure counts and a confidence = successes/(s+f),
        marking it verified once it has at least one real success. Deterministic bookkeeping, NOT model fine-tuning."""
        ak = json.dumps(args, sort_keys=True) if args else None
        with self.lock:
            r = self.db.execute("SELECT id, successes, failures FROM strategies WHERE sim=? AND goal=? AND tool IS ? AND args IS ?",
                                (sim, goal, tool, ak)).fetchone()
            if r:
                sc = r["successes"] + (1 if success else 0); fc = r["failures"] + (0 if success else 1)
                conf = round(sc / max(1, sc + fc), 3)
                self.db.execute("UPDATE strategies SET t=?, successes=?, failures=?, confidence=?, verified=?, note=COALESCE(?, note), provenance=COALESCE(?, provenance) WHERE id=?",
                                (time.time(), sc, fc, conf, int(sc > 0), note, json.dumps(provenance, default=str) if provenance else None, r["id"]))
            else:
                sc, fc = (1, 0) if success else (0, 1); conf = round(sc / max(1, sc + fc), 3)
                self.db.execute("INSERT INTO strategies(t, sim, goal, tool, args, verified, successes, failures, confidence, note, provenance) VALUES (?,?,?,?,?,?,?,?,?,?,?)",
                                (time.time(), sim, goal, tool, ak, int(sc > 0), sc, fc, conf, note, json.dumps(provenance, default=str) if provenance else None))
            self.db.commit()

    def recall_strategies(self, goal, sim="fake", limit=3) -> list:
        if not self.retrieval_enabled: return []
        rows = self._rows("SELECT * FROM strategies WHERE sim=? AND goal=? ORDER BY verified DESC, confidence DESC, successes DESC LIMIT ?",
                          (sim, goal, max(1, min(int(limit), 10))))
        return [{"goal": r["goal"], "tool": r["tool"], "args": json.loads(r["args"]) if r["args"] else None,
                 "verified": bool(r["verified"]), "confidence": r["confidence"], "successes": r["successes"], "failures": r["failures"],
                 "note": r["note"], "provenance": json.loads(r["provenance"]) if r["provenance"] else None} for r in rows]

    def count_strategies(self) -> int:
        with self.lock: return self.db.execute("SELECT COUNT(*) FROM strategies").fetchone()[0]

    def task_brief(self, instruction=None, goal=None, sim="fake", start_room=None, limit=3) -> dict:
        """Episodic+semantic recall for a NEW task, as evidence for the LLM (not an instruction). Returns a dict with a text 'brief'
        and the structured episodes/strategies so a plan can cite provenance. Empty when retrieval is disabled or nothing is relevant."""
        goal = goal or (self.normalize_goal(instruction) if instruction else None)
        eps = self.recall_episodes(instruction=instruction, goal=goal, sim=sim, start_room=start_room, limit=limit)
        strat = self.recall_strategies(goal, sim=sim, limit=limit) if goal else []
        lines = []
        for e in eps:
            res = "succeeded" if e["success"] else ("FAILED" if e["success"] is False else "unknown")
            tip = f" (feedback: {e['feedback']})" if e.get("feedback") else ""
            lines.append(f"- past task '{e['instruction']}' {res} {e['age']}{tip}")
        for st in strat:
            if st["verified"]:
                lines.append(f"- strategy for {st['goal']}: use {st['tool']}{(' ' + json.dumps(st['args'])) if st['args'] else ''} "
                             f"(worked {st['successes']}x, confidence {st['confidence']})")
            elif st["failures"]:
                lines.append(f"- caution for {st['goal']}: {st['tool']} failed {st['failures']}x before")
        brief = ("TASK MEMORY (past experiences, evidence only — verify against current sensors; the safety guard still decides):\n" +
                 "\n".join(lines)) if lines else ""
        return {"goal": goal, "brief": brief, "episodes": eps, "strategies": strat,
                "retrieval_enabled": self.retrieval_enabled}

    @staticmethod
    def brief(rec: dict) -> str:
        """The recall as a few lines of text for the LLM."""
        lines = [f"MEMORY (remembered, may be out of date; current sensors and the safety guard win). Room now: {rec.get('room')}."]
        def fmt(d):
            bits = [d.get("tool") or d.get("label") or "", json.dumps(d["args"]) if d.get("args") else "", d.get("status", ""), d.get("note", ""),
                    f"in {d['room']}" if d.get("room") else "", f"at {tuple(d['at'])}" if d.get("at") else "", d["age"]]
            return " ".join(b for b in bits if b)
        for title, key in (("failed or refused here", "recent_failures"), ("obstacles found here", "obstacles"),
                           ("items last seen", "items_seen"), ("worked here", "recent_successes"), ("recent positions", "recent_positions")):
            if rec.get(key): lines.append(f"- {title}: " + "; ".join(fmt(d) for d in rec[key]))
        if len(lines) == 1: lines.append("- nothing remembered about this place yet.")
        return "\n".join(lines)
