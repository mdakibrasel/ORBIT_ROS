"""ORBIT meta-tool: a plan-constrained, verified layer between an LLM and the NAO tools.

The LLM emits a whole plan (schema-constrained); it is statically validated and (Phase 2b) dry-run on the house map, then executed
deterministically with per-step preconditions, the unchanged safety guard, and per-step verification from the true session state, with
bounded repair on failure. Nothing here bypasses HumanoidSession.robot_call (argument validation -> preconditions -> HumanoidGuard ->
motion), the stop-latch or the call budget; the guard never reads memory or LLM output.

Standard library + numpy only (must run inside Webots' Python)."""
