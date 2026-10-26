"""Meta-tool trace (section 4.9): JSON-lines events + per-run metrics. No robot, no LLM."""
from __future__ import annotations
import json, os, time

EVENTS = ("start", "llm", "validate", "plan", "rejected", "guard", "tool", "verify", "repair", "reflex", "done")


class Trace:
    def __init__(self, path=None):
        self.path = path
        self.records = []
        self._f = None
        if path:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            self._f = open(path, "w", encoding="utf-8")
        self._t0 = time.time()
        self.counts = {"tool": 0, "tool_ok": 0, "repeated": 0, "invalid": 0, "blocked": 0, "llm": 0,
                       "repairs": 0, "validations": 0, "falls": 0, "verified": 0}

    def event(self, kind, **kw):
        rec = {"t": round(time.time() - self._t0, 3), "event": kind, **kw}
        self.records.append(rec)
        if self._f: self._f.write(json.dumps(rec, default=str) + "\n"); self._f.flush()
        if kind == "tool":
            self.counts["tool"] += 1
            if kw.get("status") == "ok": self.counts["tool_ok"] += 1
            if kw.get("status") in ("rejected", "error"): self.counts["invalid"] += 1
            if kw.get("repeated"): self.counts["repeated"] += 1
        elif kind == "guard" and kw.get("blocked"): self.counts["blocked"] += 1
        elif kind == "llm": self.counts["llm"] += 1
        elif kind == "repair": self.counts["repairs"] += 1
        elif kind == "validate": self.counts["validations"] += 1
        elif kind == "reflex" and kw.get("kind") == "stand_up": self.counts["falls"] += 1
        elif kind == "verify" and kw.get("ok"): self.counts["verified"] += 1
        return rec

    def metrics(self, extra=None):
        c = self.counts
        m = {"tool_calls": c["tool"], "tool_ok": c["tool_ok"],
             "repeated_pct": round(100 * c["repeated"] / c["tool"], 1) if c["tool"] else 0.0,
             "invalid_pct": round(100 * c["invalid"] / c["tool"], 1) if c["tool"] else 0.0,
             "blocked_calls": c["blocked"], "llm_calls": c["llm"], "repairs": c["repairs"],
             "validation_rounds": c["validations"], "falls": c["falls"], "verified_steps": c["verified"],
             "wall_s": round(time.time() - self._t0, 2)}
        if extra: m.update(extra)
        return m

    def close(self):
        if self._f: self._f.close(); self._f = None
