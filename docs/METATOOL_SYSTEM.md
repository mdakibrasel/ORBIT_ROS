# ORBIT MetaTool System

A plan-constrained, verified layer between an LLM and the NAO tools (`orbit_ros/metatool/`). The LLM emits a whole plan
(schema-constrained); it is statically validated and self-refined BEFORE any motion, then executed deterministically with per-step
preconditions, the unchanged `HumanoidGuard`, and per-step verification from the true session state, with bounded repair on failure.
Nothing bypasses `HumanoidSession.robot_call` (argument validation → preconditions → guard → motion), the stop-latch or the call budget;
the guard never reads memory or LLM output.

## Modules (all standard-library + numpy)
| Module | Role |
|---|---|
| `plan.py` | `Plan`/`Step` model, `goal_struct`, `topo_order`, cycle detection, `${id.field}` ancestor-only refs, `extract_json` |
| `spec.py` | `MetaSpec` (requires/effects/verify/risk/repeatable/timeout) per tool; `facts(session)` |
| `schema.py` | `plan_json_schema(registry, strict)` → anyOf-per-tool grammar for constrained decoding |
| `validator.py` | static `validate(plan, registry, facts, done_ids)` → issues (EMPTY, TOO_LONG, DUP_ID, BAD_DEP, CYCLE, BAD_ONFAIL, UNKNOWN_TOOL, ARG_*, BAD_REF, PRECONDITION via forward-sim, REPEAT) |
| `llm_io.py` | provider I/O: json_schema → json_object → text fallback (remembers rejections), 403/429/5xx backoff, token/latency; local re-validation always |
| `prompts.py` | system/user/refine/repair prompts generated from the registry |
| `guard.py` | loop guard (fingerprint = tool+args+state-digest; A-B-A-B) + budgets (60 tool / 12 LLM / 3 repair / 900 s) |
| `executor.py` | `MetaToolAgent.run(instruction)`: plan→validate→refine→refuse-or-execute (topo order, no LLM between steps)→verify→skip/abort/repair; fall reflex = real stand_up |
| `trace.py` | JSONL events (start/llm/validate/plan/rejected/guard/tool/verify/repair/reflex/done) + per-run metrics |

## Outcomes
`done` · `refused` (feasible:false, 0 motions) · `failed` · `aborted` · `invalid_plan`. Success = every non-skipped step done AND verified
from the true pose — a valid plan is not proof of success.

## MCP interface (verified against the live `tools/list`)
`validate_plan(plan)` (never moves), `execute_plan(plan)`, `plan_and_execute(instruction)` — part of the 48-tool NAO surface.
`orbit_chat.py --mode plan` drives `plan_and_execute`; `--mode stepwise` keeps the turn-by-turn behaviour.

## Known-target rule (2026-10-25)
A plan target must be a **known room** (living_room, kitchen, bedroom, bathroom) or a **known house item** — i.e. one of the
`known(...)` facts computed by `spec.facts()` from `orbit_ros/house.py` (rooms + item labels). There is no runtime place-teaching or
object-discovery tool in this build (`nao_memory.py` has no `places` table). The system prompt (`prompts.py`) and the validator hint
(`validator.py` `_pre_hint`) therefore instruct the model: if the request names an unknown place/object, reply `feasible:false` with a
`refusal_reason` rather than inventing a discovery tool. Live-verified: "go to the spaceship launchpad" → refused, 0 m moved; "go to the
kitchen" → done, true pose in kitchen. (Earlier text wrongly referenced non-existent `remember_place`/`find_object` tools.)

NOT yet implemented (honest): `plan_status`, `cancel_plan`, `resume_plan`, `interrupt`, `approve`, `deny`, `answer_clarification`,
`list_skills`, `explain_last_run`, `get_metrics`, and the advanced modules (dryrun, candidates, plan_cache, clarify, constraints,
scheduler, interrupts, skills, routing, tool_stats, explain, approval, certificate).

## Live evidence
- All 10 core live-acceptance scenarios PASS (Phase 14, `docs/PHASE14_METATOOL_REPORT.md`): evidence under
  `results_live/phase14_metatool_20261025/` (dependent-order, loop-guard, stop-latch, no-repeat, memory-context, blocked→abort) and
  `results_live/m10_20261025/` (invalid-tool reject, navigation, hazardous refusal, chat `--mode plan`).
- Full loop: `results_live/m10_20261025/go_kitchen/` (real Groq plan → validate → `go_to_room` → verified, end room=kitchen; trace
  `start→llm→validate→plan→tool→verify→done`, 1 LLM call, 2001 tokens, 0 fallbacks).
- Refusal before motion: `results_live/m10_20261025/walk_through_wall/` (feasible:false, 0 m moved).
- Static reject before motion: `results_live/m10_20261025/validate_badtool/` (UNKNOWN_TOOL, 0 m moved).
- Code checks: `tests/test_metatool_core.py`, `tests/test_metatool_runtime.py`.
- Advanced features (dryrun/candidates/cache/clarify/…): dependency-ordered follow-up in `docs/METATOOL_ADVANCED_PLAN.md`, each with its own later live acceptance.
