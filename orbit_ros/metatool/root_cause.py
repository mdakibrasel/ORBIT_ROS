"""Root-cause attribution for the meta-tool (Phase 17, feature 7). Pure code: no robot, no LLM.

Classifies a failure into a fixed taxonomy and, given the per-step trace, identifies the EARLIEST step that
contributed to the failure so repair can restart from there (not from the last step), while preserving completed
physical actions as facts. This replaces the single-string heuristic in executor._attribute_cause with a structured
label; the executor keeps its existing splice logic (completed steps are frozen, never repeated).

Taxonomy (fixed):
  planning:   impossible_action | ignored_constraint | wrong_target | missing_step | inefficient_plan
  action:     malformed_action | invalid_parameter
  perception: missed_detection | incorrect_detection
  memory:     stale_memory | incorrect_memory
  execution:  blockage | fall | drift | timeout
  system:     connection | provider

A label is only asserted when trace/observation evidence supports it; otherwise 'unknown' is returned."""
from __future__ import annotations
from dataclasses import dataclass, field

TAXONOMY = {
    "planning": ("impossible_action", "ignored_constraint", "wrong_target", "missing_step", "inefficient_plan"),
    "action": ("malformed_action", "invalid_parameter"),
    "perception": ("missed_detection", "incorrect_detection"),
    "memory": ("stale_memory", "incorrect_memory"),
    "execution": ("blockage", "fall", "drift", "timeout"),
    "system": ("connection", "provider"),
}


@dataclass
class RootCause:
    category: str              # planning | action | perception | memory | execution | system | unknown
    label: str                 # a value from TAXONOMY[category], or "unknown"
    earliest_step: str = ""    # id of the earliest step that contributed, if identifiable
    evidence: str = ""         # the trace/observation text that supports the label
    supported: bool = True     # False when we could not find evidence (label == unknown)

    def as_dict(self):
        return {"category": self.category, "label": self.label, "earliest_step": self.earliest_step,
                "evidence": self.evidence, "supported": self.supported}


def classify_result(result):
    """Classify a single tool result dict into (category, label, evidence). Deterministic keyword rules."""
    r = result or {}
    st = str(r.get("status", "")).lower()
    msg = str(r.get("message", "")).lower()
    blob = st + " " + msg

    if any(k in blob for k in ("connection", "disconnected", "controller", "socket", "timed out connecting")):
        return "system", "connection", blob.strip()
    if any(k in blob for k in ("provider", "http 4", "http 5", "rate limit", "429", "api key", "no provider")):
        return "system", "provider", blob.strip()
    if "fell" in blob or "fallen" in blob or "fall" in blob:
        return "execution", "fall", blob.strip()
    if st == "vetoed" or "guard" in blob or "no-go" in blob or "too close" in blob or "collision" in blob or "blocked" in blob:
        return "execution", "blockage", blob.strip()
    if "no progress" in blob or "stuck" in blob:
        return "execution", "timeout", blob.strip()
    if "drift" in blob or "heading" in blob:
        return "execution", "drift", blob.strip()
    if "timeout" in blob or "timed out" in blob:
        return "execution", "timeout", blob.strip()
    if "not found" in blob or "not visible" in blob or "cannot see" in blob:
        return "perception", "missed_detection", blob.strip()
    if "no safe path" in blob or "unreachable" in blob or "outside the house" in blob:
        return "planning", "impossible_action", blob.strip()
    if "precondition" in blob or "stopped" in blob or "upright" in blob:
        return "planning", "missing_step", blob.strip()
    if "invalid" in blob or "range" in blob or "enum" in blob or "argument" in blob:
        return "action", "invalid_parameter", blob.strip()
    if "malformed" in blob or "ref " in blob or "reference" in blob:
        return "action", "malformed_action", blob.strip()
    if "stale" in blob or "moved" in blob or "out of date" in blob:
        return "memory", "stale_memory", blob.strip()
    return "unknown", "unknown", blob.strip()


def attribute(plan_steps, failed_step_id=None):
    """Given the plan's steps (each with .id, .status, .result), return a RootCause pointing at the EARLIEST
    contributing step. Completed steps are preserved as facts by the caller; this only locates the restart point.

    plan_steps: iterable of objects with .id, .status, .result (dict) in execution order."""
    steps = list(plan_steps or [])
    # earliest step that failed / errored / was vetoed carries the root cause
    for s in steps:
        status = getattr(s, "status", "")
        res = getattr(s, "result", {}) or {}
        rstatus = str(res.get("status", "")).lower()
        if status == "failed" or rstatus in ("error", "rejected", "blocked", "vetoed"):
            cat, label, ev = classify_result(res)
            return RootCause(category=cat, label=label, earliest_step=s.id, evidence=ev,
                             supported=(label != "unknown"))
    # fall back to the named failed step if given
    if failed_step_id:
        for s in steps:
            if s.id == failed_step_id:
                cat, label, ev = classify_result(getattr(s, "result", {}) or {})
                return RootCause(cat, label, s.id, ev, supported=(label != "unknown"))
    return RootCause("unknown", "unknown", "", "", supported=False)
