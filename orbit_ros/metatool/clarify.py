"""Calibrated clarification for the meta-tool (Phase 17, feature 4). Pure code: no robot, no LLM here.

This module implements the DECISION LOGIC and CALIBRATION MACHINERY for clarification. It does NOT ship a
calibration dataset: a trustworthy threshold requires at least ~50 real English instructions with user-confirmed
goal labels, and labelling needs a real person. Until such a dataset exists, `load_calibration` returns None and the
caller must fall back to the existing deterministic hard triggers (ask when a target is unresolved/ambiguous). That
dependency is recorded honestly rather than faked.

Pipeline (when a calibration set exists):
  1. Generate K candidate goals (the caller samples the provider; this module does not call the LLM).
  2. Normalize each candidate's goal_struct into a stable key (verb + sorted resolved targets + sorted constraints).
  3. Compute each option's empirical candidate frequency.
  4. Build a prediction set using a documented split-conformal-style threshold from the calibration scores and a
     configurable target error rate alpha.
  5. If exactly one supported option remains -> proceed (subject to all other validation/safety checks).
     If multiple remain -> ask the user to choose among them. If none remain -> ask an open question.

The empirical coverage estimate assumes the calibration instructions are exchangeable with deployment instructions
and that enough candidates are sampled per instruction; it is an empirical estimate, NOT a formal guarantee, and this
limitation is reported with every decision."""
from __future__ import annotations
import json, math, os
from collections import Counter
from dataclasses import dataclass, field

from .voting import normalize_key

CALIBRATION_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                                "configs", "clarify_calibration.json")


@dataclass
class ClarifyDecision:
    action: str                     # proceed | choose | ask_open
    options: list = field(default_factory=list)      # candidate goal keys offered (for 'choose')
    frequencies: dict = field(default_factory=dict)  # key -> empirical frequency
    coverage: float = 0.0           # empirical coverage estimate (0..1), 0 when uncalibrated
    calibrated: bool = False        # False -> fell back to deterministic behavior (no dataset)
    note: str = ""

    def as_dict(self):
        return {"action": self.action, "options": [list(o) if isinstance(o, tuple) else o for o in self.options],
                "frequencies": {str(k): v for k, v in self.frequencies.items()},
                "coverage": self.coverage, "calibrated": self.calibrated, "note": self.note}


def candidate_frequencies(candidate_goal_structs):
    """Map normalized goal key -> empirical frequency across the K sampled candidates."""
    keys = [normalize_key(gs) for gs in (candidate_goal_structs or [])]
    n = len(keys) or 1
    c = Counter(keys)
    return {k: round(v / n, 4) for k, v in c.items()}


def load_calibration(path=CALIBRATION_PATH):
    """Load the calibration record, or None if it does not exist (dependency BLOCKED on real labels)."""
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def threshold_from_calibration(calibration, alpha=0.1):
    """Split-conformal-style threshold: the (alpha) empirical quantile of the calibration 'true-option frequency'
    scores. An option whose frequency >= threshold is kept in the prediction set. Returns None if uncalibrated."""
    if not calibration:
        return None
    scores = sorted(float(s) for s in (calibration.get("true_option_frequencies") or []))
    if not scores:
        return None
    # keep options at least as frequent as the alpha-quantile of correct-option frequencies
    idx = max(0, min(len(scores) - 1, int(math.floor(alpha * (len(scores) + 1))) - 1))
    return scores[idx]


def decide(candidate_goal_structs, calibration=None, alpha=0.1):
    """Return a ClarifyDecision. If no calibration, action='ask_open' with calibrated=False so the caller uses its
    existing deterministic hard triggers instead of a fabricated threshold."""
    freqs = candidate_frequencies(candidate_goal_structs)
    if not freqs:
        return ClarifyDecision("ask_open", note="no candidates to evaluate", calibrated=False)
    thr = threshold_from_calibration(calibration, alpha)
    if thr is None:
        return ClarifyDecision("ask_open", frequencies=freqs, calibrated=False,
                               note="no calibration dataset (needs >=50 user-labelled instructions); "
                                    "caller should use deterministic hard triggers")
    kept = [k for k, f in freqs.items() if f >= thr]
    coverage = round(sum(f for k, f in freqs.items() if k in kept), 4)
    base_note = (f"prediction set at alpha={alpha}, threshold={thr}; empirical coverage estimate only "
                 f"(assumes exchangeability + enough samples; not a formal guarantee)")
    if len(kept) == 1:
        return ClarifyDecision("proceed", options=kept, frequencies=freqs, coverage=coverage,
                               calibrated=True, note=base_note)
    if len(kept) > 1:
        return ClarifyDecision("choose", options=kept, frequencies=freqs, coverage=coverage,
                               calibrated=True, note=base_note)
    return ClarifyDecision("ask_open", frequencies=freqs, coverage=coverage, calibrated=True,
                           note="no option cleared the threshold; " + base_note)
