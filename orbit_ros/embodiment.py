"""Interpretable embodiment identification and safe active calibration primitives.

This module is an experimental, simulator-independent component. It does not yet
control World2D/Webots directly; callers provide predicted outcomes for candidate
probes and observed outcomes after execution.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from itertools import product
from math import log2
from typing import Iterable, Mapping, Any

@dataclass(frozen=True, order=True)
class Hypothesis:
    mirrored_camera: bool
    left_right_swapped: bool
    move_gain: float
    turn_gain: float

    @property
    def key(self) -> str:
        return f"mirror={int(self.mirrored_camera)};swap={int(self.left_right_swapped)};mg={self.move_gain:g};tg={self.turn_gain:g}"

    def to_dict(self):
        return asdict(self)

@dataclass
class EmbodimentCard:
    """Compact, auditable summary of the currently plausible sensor/actuator model."""
    hypotheses: list[dict[str, Any]]
    selected: dict[str, Any] | None
    confidence: float
    probes_used: int
    status: str
    schema_version: str = "1.0"

    def to_dict(self):
        return asdict(self)

class SafeProbeError(ValueError):
    pass

class EmbodimentIdentifier:
    """Finite-hypothesis Bayesian-style update with entropy-reducing probe choice.

    A probe is represented as {name, safe, outcomes: {hypothesis.key: predicted_label}}.
    The selected probe maximizes expected entropy reduction under current weights.
    """
    def __init__(self, mirror=(False, True), swapped=(False, True), move_gains=(0.8, 1.0, 1.2), turn_gains=(0.8, 1.0, 1.2), max_probes=4,
                 epsilon=0.0, cost_weight=0.0, stop_confidence=0.95):
        self.hypotheses = [Hypothesis(*v) for v in product(mirror, swapped, move_gains, turn_gains)]
        self.weights = {h.key: 1.0 / len(self.hypotheses) for h in self.hypotheses}
        self.max_probes = max_probes
        # epsilon: probability that a probe observation is misread. 0 -> hard elimination (original behaviour);
        # >0 -> soft likelihood update (match: 1-eps, mismatch: eps), so one noisy reading cannot be decisive.
        self.epsilon, self.cost_weight, self.stop_confidence = epsilon, cost_weight, stop_confidence
        self.probes_used = 0
        self.history: list[dict[str, Any]] = []

    @staticmethod
    def _entropy(weights: Mapping[str, float]) -> float:
        return -sum(p * log2(p) for p in weights.values() if p > 0)

    def confidence(self) -> float:
        return max(self.weights.values(), default=0.0)

    def choose_probe(self, probes: Iterable[Mapping[str, Any]]) -> dict[str, Any] | None:
        if self.probes_used >= self.max_probes or self.confidence() >= self.stop_confidence:
            return None
        base = self._entropy(self.weights)
        best, best_gain, best_util = None, -1.0, float("-inf")
        for probe in probes:
            if not probe.get("safe", False):
                continue
            outcomes = probe.get("outcomes", {})
            groups: dict[str, float] = {}
            for key, weight in self.weights.items():
                label = str(outcomes.get(key, "unknown"))
                groups[label] = groups.get(label, 0.0) + weight
            expected = 0.0
            for label, mass in groups.items():
                if mass <= 0: continue
                posterior = {key: (w / mass if str(outcomes.get(key, "unknown")) == label else 0.0) for key, w in self.weights.items()}
                expected += mass * self._entropy(posterior)
            gain = base - expected
            utility = gain - self.cost_weight * float(probe.get("cost", 0.0))   # information per unit of probing cost
            if gain > 1e-12 and utility > best_util:
                best, best_gain, best_util = dict(probe), gain, utility
        if best is None:
            return None
        best["expected_information_gain_bits"] = round(best_gain, 6)
        return best

    def update(self, probe: Mapping[str, Any], observed_outcome: str) -> None:
        if not probe.get("safe", False):
            raise SafeProbeError("Refusing to update from a probe not marked safe")
        outcomes = probe.get("outcomes", {})
        if self.epsilon > 0:
            like = {k: (1 - self.epsilon if str(outcomes.get(k, "unknown")) == str(observed_outcome) else self.epsilon) for k in self.weights}
            post = {k: self.weights[k] * like[k] for k in self.weights}
            z = sum(post.values())
            self.weights = {k: v / z for k, v in post.items()}
            self.probes_used += 1
            self.history.append({"probe": probe.get("name"), "observed": observed_outcome, "surprise": False, "soft": True})
            return
        keep = [h.key for h in self.hypotheses if str(outcomes.get(h.key, "unknown")) == str(observed_outcome)]
        if not keep:
            # Model surprise: preserve evidence rather than silently choosing a hypothesis.
            self.history.append({"probe": probe.get("name"), "observed": observed_outcome, "surprise": True})
            return
        total = sum(self.weights[k] for k in keep)
        self.weights = {k: (self.weights[k] / total if k in keep and total else 0.0) for k in self.weights}
        self.probes_used += 1
        self.history.append({"probe": probe.get("name"), "observed": observed_outcome, "surprise": False, "remaining_hypotheses": len(keep)})

    def card(self) -> EmbodimentCard:
        ranked = sorted(((self.weights[h.key], h) for h in self.hypotheses), key=lambda x: x[0], reverse=True)
        top_weight, top = ranked[0]
        selected = top.to_dict() if top_weight >= self.stop_confidence else None
        plausible = [h.to_dict() for weight, h in ranked if weight > (1e-9 if self.epsilon else 0)]
        status = "identified" if selected else ("budget_exhausted" if self.probes_used >= self.max_probes else "ambiguous")
        return EmbodimentCard(plausible, selected, round(top_weight, 6), self.probes_used, status)

class SurpriseMonitor:
    """Detects predicted/observed mismatches; no automatic causal diagnosis is claimed."""
    def __init__(self, consecutive_threshold: int = 1):
        if consecutive_threshold < 1: raise ValueError("threshold must be >= 1")
        self.threshold = consecutive_threshold
        self.consecutive = 0
        self.total_surprises = 0
        self.events: list[dict[str, Any]] = []

    def observe(self, predicted: Any, observed: Any, context: str = "") -> dict[str, Any]:
        surprise = predicted != observed
        self.consecutive = self.consecutive + 1 if surprise else 0
        if surprise: self.total_surprises += 1
        event = {"predicted": predicted, "observed": observed, "context": context,
                 "surprise": surprise, "trigger": self.consecutive >= self.threshold,
                 "consecutive_surprises": self.consecutive}
        self.events.append(event)
        return event


def make_probe(name: str, safe: bool, outcome_by_hypothesis: Mapping[str, str],
               preconditions: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Construct a probe contract. Caller must independently validate its safety."""
    return {"name": name, "safe": bool(safe), "outcomes": dict(outcome_by_hypothesis),
            "preconditions": dict(preconditions or {}), "requires_dry_run": True}
