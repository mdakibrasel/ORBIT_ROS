"""Consistency voting over normalized goals/constraints for the meta-tool (Phase 17, feature 5). Pure code.

Given K candidate plans (each already schema-valid), group them by a STABLE normalized key built from the goal
verb, resolved target names and constraints, then return the plan from the largest group. The winning plan is still
subject to the full validation, safety-gate, contract and verification pipeline downstream; voting only chooses which
candidate to carry forward. This does NOT implement unsupported self-critique.

normalize_key(goal_struct) -> a hashable tuple (verb, sorted targets, sorted constraints, sorted negations).
vote(plans) -> (winning_plan, tally) where tally maps key -> count."""
from __future__ import annotations
from collections import Counter


def _norm_list(xs):
    return tuple(sorted(str(x).strip().lower().replace(" ", "_") for x in (xs or []) if str(x).strip()))


def normalize_key(goal_struct):
    gs = goal_struct or {}
    verb = str(gs.get("verb", "")).strip().lower()
    return (verb, _norm_list(gs.get("targets")), _norm_list(gs.get("constraints")), _norm_list(gs.get("negations")))


def plan_key(plan):
    """Normalized key for a Plan (or dict). Refusals key on ('<refused>',) so they group together."""
    gs = getattr(plan, "goal_struct", None)
    if gs is None and isinstance(plan, dict):
        gs = plan.get("goal_struct")
    feasible = getattr(plan, "feasible", None)
    if feasible is None and isinstance(plan, dict):
        feasible = plan.get("feasible", True)
    if not feasible:
        return ("<refused>",)
    return normalize_key(gs)


def vote(plans):
    """Return (winning_plan, tally). Ties broken by the FIRST candidate in the winning group (stable)."""
    plans = [p for p in (plans or []) if p is not None]
    if not plans:
        return None, {}
    keys = [plan_key(p) for p in plans]
    tally = Counter(keys)
    # winning key: highest count, tie-break by earliest first appearance
    best_key, best_count, best_first = None, -1, len(plans)
    first_seen = {}
    for i, k in enumerate(keys):
        first_seen.setdefault(k, i)
    for k, c in tally.items():
        if c > best_count or (c == best_count and first_seen[k] < best_first):
            best_key, best_count, best_first = k, c, first_seen[k]
    winner = next(p for p, k in zip(plans, keys) if k == best_key)
    return winner, {str(k): c for k, c in tally.items()}


def agreement(plans):
    """Fraction of candidates that agree with the winning key (1.0 = unanimous). 0.0 for no plans."""
    plans = [p for p in (plans or []) if p is not None]
    if not plans:
        return 0.0
    _, tally = vote(plans)
    top = max(tally.values()) if tally else 0
    return round(top / len(plans), 3)
