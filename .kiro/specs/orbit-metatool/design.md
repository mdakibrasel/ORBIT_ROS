# ORBIT meta-tool — Design

## Overview

ORBIT becomes a **plan-constrained, verified meta-tool** layered between an LLM and the existing NAO tool surface. The LLM never drives
the robot step-by-step in the meta-tool path; it emits a whole plan (schema-constrained), which is statically validated, dry-run on the
house map, optionally chosen among K candidates / reused from a cache, then executed deterministically with per-step precondition checks,
the unchanged safety guard, and per-step verification from the true session state, with bounded repair on failure.

```
instruction
  -> LLM: K candidate plans (json_schema constrained)  +  plan cache (goal_key)
  -> validator (static, no motion)
  -> map dry-run (A* + guard logic on house.py, no motion)
  -> pick best plan (score)        -> clarify (ask_user) if ambiguous
  -> executor:  per step [ resolve ${refs} -> precondition -> loop guard -> robot_call(validate+guard+motion) -> verify ]
  -> repair (remaining work only, re-validated) | skip | abort
  -> memory + trace + evidence pack
```

Everything below `robot_call` is unchanged: argument validation → preconditions → `HumanoidGuard` → motion. The meta-tool only decides
WHICH validated plan runs and whether to ask first; it cannot bypass the guard, the stop-latch or the call budget.

## Architecture layering (enforced)

- `orbit_ros/metatool/` is a new package. It imports `orbit_ros.humanoid` (session), `orbit_ros.house` (map, for dry-run), `orbit_ros.llm`
  (provider), `orbit_ros.nao_memory` (memory). `nao_memory` stays a leaf (imports nothing from the robot layer) — enforced by
  `test_pel_schema.py`.
- The MCP server (`orbit_ros/mcp_server.py`) gains `validate_plan`, `execute_plan`, `plan_and_execute` tools that call the meta-tool.
- The live harness `tools/live/` is test/eval only; core code never imports it.

## Components

| Module | Responsibility |
|---|---|
| `metatool/spec.py` | `MetaSpec` wrapping `ToolSpec` with `requires/effects/verify/risk/repeatable/timeout_s`; `facts(session)` |
| `metatool/plan.py` | `Plan`/`Step` data model, `goal_struct`, `topo_order`, `pseudo_code`, `extract_json` |
| `metatool/schema.py` | `plan_json_schema(registry, strict)` → anyOf-per-tool grammar |
| `metatool/validator.py` | `validate(plan, facts, done_ids)` → `Issue[]` with the full code set + forward-sim of requires/effects |
| `metatool/llm_io.py` | provider I/O (json_schema→json_object→text fallback), retries, token/latency accounting, local re-validation |
| `metatool/prompts.py` | system/user/refine/repair prompt builders from the registry |
| `metatool/guard.py` | loop/budget guard (fingerprint = tool+args+state digest; A-B-A-B; budgets) |
| `metatool/executor.py` | `MetaToolAgent.run(instruction)` — the deterministic contract |
| `metatool/trace.py` | JSONL events + per-run `metrics()` |
| `metatool/dryrun.py` | map dry-run (reachability, predicted path/motions/time) |
| `metatool/candidates.py` | K candidates, validate+dryrun+score, pick best |
| `metatool/plan_cache.py` | verified-plan reuse (SQLite `plan_cache`, part of memory v3) |
| `metatool/clarify.py` | deterministic ambiguity checks → `ask_user` |

## Key decisions

- **Schema-constrained decoding, then always re-validate locally** — hosted models may ignore the schema, so the local validator is the
  authority (same philosophy as the existing registry re-check).
- **No LLM between steps** — determinism and reproducibility; the LLM is consulted only for planning and repair.
- **Verification from the session's true observations**, never from the LLM's claim (mirrors the existing `_verify`/`_step_met`).
- **Dry-run is a map calculation, not a simulator** — labelled `"dryrun":"map"`; it reuses `house.plan_path` and the guard's geometry so a
  prediction uses the same logic the live guard will.
- **Honesty first (Phase 1)** — the kinematic fallback and supervisor reset become debug-only and mark a trial invalid, so measured
  success reflects the real gait.

## Memory v3

In-place migration from v2 (`schema_version=3`): add `aliases`, `places`, `object_state`, and `plan_cache` tables; structured goals from
`goal_struct`; confidence decay; optional embedding hook (off). Invariants unchanged.

## Testing strategy

- **Code checks** (`-m "not live"`, no robot/LLM): schema variant count == tool count; every validator code on hand-written plan JSON;
  `${ref}` resolution; loop guard on a recorded sequence; SQLite memory (episodic/semantic/aliases/places/object_state/plan_cache);
  goal normalization incl. Bengali/negation; docs consistency; the no-fake grep.
- **Live tests** (`@pytest.mark.live`, Webots + real provider): each writes an evidence pack and is judged from the true pose/state.
- The harness enforces one Supervisor per process and refuses a second extern controller.

## Environmental constraints & scope (explicit)

- One Supervisor per process → a fresh process per live trial.
- Live nav trials ~200–550 s → the full Phase-6 matrix (≥10 × ~20 tasks × ~13 cells × 2 models) is thousands of trials = days of
  wall-clock. This design is executed **incrementally**; partial completion is reported honestly per the DoD, never faked.
- Providers: Groq reachable; Gemini times out from this machine. The "small+large model" axis needs both reachable; where only one is
  reachable the report records a single model and states the gap.
