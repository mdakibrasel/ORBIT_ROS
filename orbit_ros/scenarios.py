from dataclasses import dataclass

@dataclass(frozen=True)
class Scenario:
    task_level: str
    horizon: int
    dependency_depth: int
    disturbance_rate: float
    ambiguity: float

# T1-T6 independently increase different dimensions where practical.
SCENARIOS = {
    "T1": Scenario("T1", 2, 1, 0.00, 0.00),
    "T2": Scenario("T2", 4, 2, 0.00, 0.10),
    "T3": Scenario("T3", 6, 3, 0.05, 0.10),
    "T4": Scenario("T4", 8, 3, 0.15, 0.10),  # environment changes
    "T5": Scenario("T5", 8, 5, 0.05, 0.25),  # deeper dependencies / ambiguity
    "T6": Scenario("T6", 12, 6, 0.20, 0.30), # long + dynamic + ambiguous
}
