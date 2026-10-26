# ORBIT-PEL plan schema `orbit.plan/v1` (milestone M1)

Code: `orbit_ros/pel/schema.py`. Tests: `tests/test_pel_schema.py`, `tests/test_pel_v0141_fixes.py`. Read by the execution manager
(`orbit_ros/pel/manager.py`) in every PEL configuration. See "What the manager enforces" below for which fields are checked (v0.14.1).

## Plan object

| field | type | required | meaning |
|---|---|---|---|
| `schema` | `"orbit.plan/v1"` | yes (for a v1 plan) | without it the reply is read as a legacy plan |
| `plan_id` | string | no | e.g. `<task_id>#<config>` |
| `revision` | int >= 0 | no (0) | increases with every re-plan |
| `parent_revision` | int < revision | no | the revision this one replaces |
| `revision_reason` | string | no | why the plan was replaced (failure message, stale plan, clarification) |
| `goal` | string | no | the planner's restatement of the instruction |
| `status` | `ok` / `needs_clarification` / `unsupported` | no (`ok`) | the last two must have no steps |
| `clarification` | string | with `needs_clarification` | the question to ask the user |
| `unsupported_reason` | string | with `unsupported` | why the task cannot be done |
| `constraints` | object | no | `no_go_cells` [[x, y], ...], `max_steps` int, `stay_in_room` bool |
| `steps` | list of steps | yes | may be empty only when the instruction is already complete (warning `empty_plan`) |
| `completion` | object | no | `pose` {x, y, yaw_deg}, `cleaned_min` int, `reported` bool, `visited` [[x, y], ...] |
| `made_for` | object | no | observation-derived context the plan was written for (used by M3 to detect stale plans) |

## Step object

| field | type | required | meaning |
|---|---|---|---|
| `id` | int >= 1, unique | yes | |
| `tool` | string | yes | must be a tool the runtime knows (`default_known_tools()`); argument ranges are checked by the registry in M2 |
| `args` | object | no (`{}`) | |
| `pre` | object | no | `robot_stopped` bool, `pose` {x, y, yaw_deg}, `sees` label, `localised` bool |
| `expect` | object | no | `pose` {x, y, yaw_deg}, `sees` label, `not_sees` label, `cleaned_cell` bool, `reported` bool |
| `on_failure` | `replan` / `retry` / `ask` / `stop` | no (`replan`) | `stop` is honoured (safe stop); `replan`, `retry` and `ask` all lead to a bounded re-plan (see below) |
| `region` | string | no | region name of a hierarchical plan |
| `note` | string | no | free text for the log |

`yaw_deg` values must be multiples of 90. Unknown fields are kept out of the plan and reported as warnings (`unknown_field`).

## Issue codes

`schema_error`, `missing_field`, `bad_type`, `bad_value`, `duplicate_id`, `unknown_tool`, `malformed_action` (errors) and
`unknown_field`, `empty_plan` (warnings). Each issue names its location, e.g. `steps[2].expect.pose.x`.

## Legacy compatibility

Replies without `schema` are read exactly as `agent2d.normalize_plan` reads them today: a list of actions, `{"actions", "expect"}`,
or hierarchical `{"regions": [...]}`. The legacy `expect` becomes `completion`. `to_legacy(parse_plan(x)[0]) == normalize_plan(x)`
holds for every input; this was checked on 10,885 plans produced by the oracle across all 13 families, 3 seeds, 4 headings and 9
plan-writing configs (`tools/pel_roundtrip_check.py --seeds 3`), and on 500 random malformed inputs in the tests.

`to_legacy` on a native v1 plan gives `{"actions": [...], "expect": {...}}` from the steps and `completion`; per-step `pre` and
`expect` have no legacy form and are not executed by the existing runner.

## Ground truth

The schema has no field that is filled from `Task.expected`. `constraints.no_go_cells` and `completion` are the planner's own reading
of the instruction; the M4 guard will enforce what the planner declared, and the evaluator will still judge the episode against the
hidden ground truth.

## What the manager enforces (v0.14.1)

v0.14.0 parsed several of these fields without acting on them (code review finding M3). In v0.14.1:

| field | enforced? | how |
|---|---|---|
| plan-level errors (any `error` issue above, and registry errors on plan calls) | yes | sent back to the planner up to 3 times; a plan still invalid after that (or with re-planning off, or out of budget) is **not executed**, terminal `safe_stop` |
| `constraints.no_go_cells` | yes | the guard vetoes every move into them; macros are expanded and verified around them; kept across revisions and across a resume |
| `constraints.max_steps` | yes (plan level) | a plan with more non-`done` steps than its own `max_steps` is a `bad_value` error |
| `constraints.stay_in_room` | always on | `false` is ignored (warning): the guard always keeps the robot inside the room |
| step `pre.robot_stopped`, `pre.pose`, `pre.sees`, `pre.localised` | yes (verify component) | checked on the current observation before the step; unmet means nothing is sent and the step fails (`precondition_failed`); `sees` on a stale camera frame counts as unmet |
| step `on_failure: stop` | yes | any failure of that step (precondition, rejection, veto, duplicate, failed call, verification mismatch, unhandled failure found on resume) ends the instruction, terminal `safe_stop` |
| step `on_failure: retry` / `ask` | no, treated as `replan` | a retry happens only when the manager can prove it safe (a retry-safe tool, or a fault that left the pose unchanged), whatever the plan says |
| step `expect.pose`, `expect.sees`, `expect.not_sees` | yes (verify) | as in v0.14.0; object expectations are skipped on a stale camera frame |
| step `expect.cleaned_cell`, `expect.reported` | yes (verify) | the believed cleaned-cell count / the report list must (not) grow during the step |
| `completion.pose` | yes (verify) | as in v0.14.0 |
| `completion.cleaned_min` | yes (verify) | believed cleaned cells since the instruction started must be at least this many (a legacy plan's `expect.cleaned_min` is read the same way) |
| `completion.reported` | yes (verify) | at least one `report` in this instruction |
| `completion.visited` | yes (verify) | every listed cell must lie on the robot's reported path (consecutive reported poses joined along the axis; a macro that turns contributes only its end cells) |

"verify" means the check belongs to the verification component and is off in the `PEL-verify` ablation. All checks use what the robot
reports, never the evaluator's ground truth.
