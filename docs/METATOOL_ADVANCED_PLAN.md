# MetaTool Advanced Features — Dependency-Ordered Follow-up Plan

The **core** plan-constrained MetaTool (contracts, plan model, schema, validator, provider I/O, deterministic executor, trace, MCP + chat)
is implemented and live-verified (`docs/PHASE14_METATOOL_REPORT.md`, all 10 section-5 scenarios PASS). The 13 advanced modules are
deliberately NOT in the core phase. Each below is independently configurable (its own switch) and has its own live acceptance check in a
later phase; none may bypass validation, the guard, the stop-latch, the call budget or verification.

Legend: deps = what must exist first.

## Tier 1 — pure, no new live risk (do first)
1. `dryrun.py` — map A* reachability + pre-motion guard check from the live pose. **deps:** `house.plan_path`, `guard`, `spec`. Live
   acceptance: a target behind a no-go zone is rejected before motion (0 m). Enables earlier/cheaper rejection than execution.
2. `tool_stats.py` — reliability stats from verified live executions (trace-derived). **deps:** `trace`. Live: stats match the trace.
3. `explain.py` — evidence-grounded explanation with step ids traceable to execution records. **deps:** `trace`. Live: ids resolve.
4. `certificate.py` — SHA-256 execution certificate + verification over an evidence pack. **deps:** evidence pipeline. Live: tamper detected.

## Tier 2 — plan selection / reuse (depend on Tier 1 dryrun)
5. `candidates.py` — K candidate plans, validate + dry-run + score, pick best. **deps:** `dryrun`, `llm_io`. Live: K=3 long-horizon trace shows candidates/scores/choice.
6. `plan_cache.py` — cache only live-verified plans; re-validate + dry-run a cache hit against current facts. **deps:** `candidates`, memory v3 `plan_cache` table. Live: 2nd identical instruction = cache hit, 0 planning LLM calls, still verified.
7. `clarify.py` — `ask_user` on target ambiguity / unknown target / candidate disagreement. **deps:** `candidates`, a `clarify`/`ask_user` MCP tool. Live: "go to the chair" → clarification → answer → reached.

## Tier 3 — control flow & lifecycle (depend on the executor)
8. `constraints.py` — room/object avoidance, deadlines, motion limits, destination constraints as validator + executor checks. **deps:** `validator`, `executor`.
9. `interrupts.py` — safe cancel/replace/resume; MCP `cancel_plan`/`resume_plan`/`interrupt`/`plan_status`. **deps:** `executor` run-state. Live: cancel mid-plan stops safely; resume continues.
10. `scheduler.py` — simulation-time scheduling of validated plans within the one controller lifecycle. **deps:** `interrupts`. Live: a sim-time-scheduled step runs only when its preconditions still hold.
11. `approval.py` — human approval gate for high-risk/low-reliability steps; MCP `approve`/`deny`. **deps:** `tool_stats`, `interrupts`. Live: a high-risk step waits for approval.

## Tier 4 — higher-level (depend on cache + stats + routing)
12. `skills.py` — promote repeatedly-verified plans into parameterised skills that expand into individually-validated steps; MCP `list_skills`. **deps:** `plan_cache`, `validator`.
13. `routing.py` — small/large model cascade when both providers are genuinely configured and reachable. **deps:** `llm_io`. BLOCKED today: only Groq is reachable from this machine (Gemini times out), so the cascade cannot be live-verified until a second provider works.

## Extended MCP interface (added alongside the modules above)
`plan_status`, `cancel_plan`, `resume_plan`, `interrupt(text,priority)`, `approve(step_id)`, `deny(step_id)`, `answer_clarification(text)`,
`list_skills`, `explain_last_run(level)`, `get_metrics`. Each wires to its module; `get_metrics`/`explain_last_run` are the cheapest
(trace-derived, no new live risk) and are good first additions.

## Rule for every advanced feature
Define a tool contract → validate args/preconditions → safety-check → execute through the correct path → return
ACCEPTED/RUNNING/COMPLETED/FAILED/TIMED_OUT/NOT_SUPPORTED → record evidence. No feature is marked done without its own live acceptance pack.
