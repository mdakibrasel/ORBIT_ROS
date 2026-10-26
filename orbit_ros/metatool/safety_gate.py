"""Deterministic safety gate + Task Safety Contract for the meta-tool (Phase 17, feature 3).

Pure code: no robot motion, no LLM. The final AUTHORIZE / DEFER / REJECT decision is made by deterministic rules
here, NOT by trusting the LLM's safety judgment. This sits in front of execution; it never bypasses the existing
pipeline (HumanoidSession.robot_call -> arg-check -> preconditions -> HumanoidGuard -> motion). The contract it
produces is advisory-to-execution for step guards and abort conditions; the HumanoidGuard remains the hard safety
authority and the LLM can never override an abort.

Decisions:
  AUTHORIZE - sufficient information, no identified policy/safety violation.
  DEFER     - information missing or ambiguous; ask the user (zero motion until resolved).
  REJECT    - the task is unsafe or outside supported capabilities (zero motion).

Person clearance: 1.0 m preferred, 0.6 m critical. These are only ENFORCED when the sensing system can reliably
measure person distance. This project's live NAO has no person-distance sensing wired into the metatool, so when a
task implicates a person the gate DEFERS (or an unsafe one REJECTs) rather than assuming the area is clear."""
from __future__ import annotations
from dataclasses import dataclass, field

PERSON_CLEARANCE_PREFERRED_M = 1.0
PERSON_CLEARANCE_CRITICAL_M = 0.6

# Capability policy: words that imply a capability this robot does not support -> REJECT (outside capabilities).
UNSUPPORTED_CAPABILITY_TERMS = (
    "fly", "jump", "climb stairs", "climb the stairs", "go upstairs", "upstairs", "downstairs",
    "drive the car", "cook", "pour", "lift the", "carry the", "open the door", "pick up", "pick the",
    "grab the", "fetch the", "outside the house", "outdoors", "go outside", "leave the house",
)

# Harmful / policy-violating intents -> REJECT.
HARMFUL_TERMS = (
    "hit the", "kick the", "push the person", "attack", "hurt", "harm", "break the", "smash",
    "knock over", "destroy", "damage", "step on the", "run into",
)

# Terms implying a person is involved -> require reliable person sensing or DEFER.
PERSON_TERMS = ("person", "someone", "somebody", "the man", "the woman", "the child", "the baby",
                "the guest", "the visitor", "me", "him", "her", "human", "people")

# Ambiguity: a target that cannot be resolved -> DEFER (ask).
def _norm(x):
    return str(x).strip().lower()


@dataclass
class Contract:
    """A Task Safety Contract compiled for an AUTHORIZED task. Monitored during execution; never overrides the guard."""
    invariants: list = field(default_factory=list)       # human-readable invariants that must hold throughout
    step_guards: list = field(default_factory=list)      # per-concern live checks to run before motion
    abort_conditions: list = field(default_factory=list) # conditions that must stop the task immediately
    person_sensing_available: bool = False               # True only if distance to a person can be measured live

    def as_dict(self):
        return {"invariants": list(self.invariants), "step_guards": list(self.step_guards),
                "abort_conditions": list(self.abort_conditions),
                "person_sensing_available": self.person_sensing_available}


@dataclass
class Decision:
    decision: str                 # AUTHORIZE | DEFER | REJECT
    reason: str = ""
    contract: Contract | None = None
    question: str = ""            # for DEFER: what to ask the user

    def as_dict(self):
        d = {"decision": self.decision, "reason": self.reason, "question": self.question}
        d["contract"] = self.contract.as_dict() if self.contract else None
        return d


def _text_of(instruction, goal_struct, plan):
    parts = [str(instruction or "")]
    gs = goal_struct or {}
    parts += [str(gs.get("verb", ""))]
    parts += [str(t) for t in (gs.get("targets") or [])]
    parts += [str(c) for c in (gs.get("constraints") or [])]
    if plan is not None:
        parts.append(str(getattr(plan, "goal", "")))
    return " ".join(parts).lower()


def _contains(text, terms):
    for t in terms:
        if t in text:
            return t
    return ""


def evaluate(instruction, goal_struct=None, plan=None, facts=None, known_names=None,
             person_positions=None, person_sensing=False, capabilities=None):
    """Deterministic gate decision. Returns a Decision.

    person_positions: a list of measured (x, y) person positions, or None if not observed.
    person_sensing:   True ONLY if the system can reliably measure distance to a person. If a person is implicated
                      but this is False, we DEFER (or REJECT if the intent is harmful) rather than assume clear.
    capabilities:     iterable of supported capability tags (optional; informational)."""
    text = _text_of(instruction, goal_struct, plan)
    facts = set(facts or ())
    known = {_norm(n) for n in (known_names or [])}

    # 1) harmful intent -> REJECT (highest priority; never conditioned on sensing)
    h = _contains(text, HARMFUL_TERMS)
    if h:
        return Decision("REJECT", f"harmful/unsafe intent detected ({h!r})")

    # 2) unsupported capability -> REJECT
    u = _contains(text, UNSUPPORTED_CAPABILITY_TERMS)
    if u:
        return Decision("REJECT", f"outside supported capabilities ({u!r})")

    # 3) person implicated: require reliable sensing, else DEFER (do not assume the area is clear)
    person_word = _contains(text, PERSON_TERMS)
    if person_word and not person_sensing:
        return Decision("DEFER",
                        f"task implicates a person ({person_word!r}) but person-distance sensing is not available",
                        question="A person may be involved and I cannot reliably measure distance to people. "
                                 "Please confirm the area is clear or rephrase the task.")

    # 4) ambiguous/unresolved target -> DEFER
    targets = [t for t in ((goal_struct or {}).get("targets") or []) if _norm(t)]
    if known:
        unresolved = [t for t in targets if _norm(t) not in known]
        if unresolved and not plan_addresses(plan):
            return Decision("DEFER", f"unresolved target(s): {unresolved}",
                            question=f"I could not resolve {unresolved}. Which known room or object do you mean?")

    # 5) otherwise AUTHORIZE with a compiled contract
    return Decision("AUTHORIZE", "sufficient information; no identified policy or safety violation",
                    contract=compile_contract(person_sensing=person_sensing, person_positions=person_positions))


def plan_addresses(plan):
    """A feasible plan with at least one step is taken to have resolved its targets to concrete tool args."""
    if plan is None:
        return False
    return bool(getattr(plan, "feasible", False)) and bool(getattr(plan, "steps", None))


def compile_contract(person_sensing=False, person_positions=None):
    """Compile the Task Safety Contract. Invariants/step-guards/abort-conditions are explicit and monitored.

    The HumanoidGuard remains the hard authority for walls/furniture/no-go zones; the contract adds task-level
    invariants on top and names the abort conditions the monitor must honor."""
    invariants = [
        "remain within the approved house boundary",
        "never enter a declared no-go zone",
        "maintain an upright posture (recover a fall before continuing)",
    ]
    step_guards = [
        "before each motion: robot not stopped and not fallen (live pose)",
        "before each motion: the HumanoidGuard admits the motion (walls/furniture/no-go)",
    ]
    abort_conditions = [
        "stop-latch activated",
        "fall detected (and not recoverable within the fall-recovery budget)",
        "a verified critical safety-boundary violation",
    ]
    if person_sensing:
        invariants.append(f"keep >= {PERSON_CLEARANCE_PREFERRED_M} m from any detected person")
        step_guards.append(f"before each motion: nearest detected person >= {PERSON_CLEARANCE_CRITICAL_M} m "
                           f"(critical) and plan keeps >= {PERSON_CLEARANCE_PREFERRED_M} m (preferred)")
        abort_conditions.append(f"a person observed within {PERSON_CLEARANCE_CRITICAL_M} m")
    return Contract(invariants=invariants, step_guards=step_guards, abort_conditions=abort_conditions,
                    person_sensing_available=bool(person_sensing))
