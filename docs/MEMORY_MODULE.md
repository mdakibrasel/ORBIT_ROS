# ORBIT NAO — Memory Module (v0.16.6)

The robot memory lives in `orbit_ros/nao_memory.py` (`NaoMemory`, SQLite). Phase 10 **extended the existing v1 event store**
(it was not replaced) with episodic task memory, semantic strategy memory, schema versioning, configuration, and task-similarity
retrieval. Memory is **advice/evidence only**: it never feeds the safety guard, the planner, or the obstacle list; live sensors and the
`HumanoidGuard` always win.

## Layers

| Layer | Where | What it holds | Lifetime |
|---|---|---|---|
| **Working memory** | `HumanoidSession` instance fields (`self.last_task`, `self._dbg`, `self.replans`, `self.calls`, live `_pose()`) | current goal/subgoal, recent actions + outcomes, live pose, replanning state | the current process/session only — **not** persisted |
| **Event memory (v1)** | `events` table | per-primitive actions/observations/obstacles/positions with outcome, room, xy | persistent (SQLite), capped at `max_records` |
| **Episodic memory (v2)** | `episodes` table | one row per completed/failed TASK: instruction, normalized goal, start/final pose, planned vs executed, outcome, success, replans, failed-action count, recovery, feedback, provenance, timestamp, experiment/scenario | persistent |
| **Semantic memory (v2)** | `strategies` table | distilled reusable strategies: `(goal, tool, args)` with success/failure counts, confidence = successes/(s+f), `verified` once ≥1 real success, note, provenance | persistent |

Schema version is stored in a `meta` table (`schema_version=2`). Opening a v1 db **migrates in place** (adds `meta`/`episodes`/`strategies`;
existing `events` untouched). WAL + a 30 s busy timeout make interrupted writes recoverable (the journal replays on next open).

## Public API (additions)

```python
m = NaoMemory(path, max_records=5000, retrieval_enabled=True, write_after_task=True)
m.schema_version()                                   # -> 2
m.add_episode(instruction, goal=None, sim=, experiment=, scenario=, start={room,x,y,yaw}, planned=, executed=,
              outcome=, success=, final={...}, replans=, failed_actions=, feedback=, provenance=, dedup=True) -> episode_id
m.recall_episodes(instruction=|goal=, sim=, start_room=, limit=3) -> [ {goal, success, feedback, provenance, relevance, age, ...} ]
m.learn_strategy(goal, tool=, args=, success=, sim=, note=, provenance=)   # upserts success/failure counts + confidence
m.recall_strategies(goal, sim=, limit=3) -> [ {tool, args, verified, confidence, successes, failures, ...} ]
m.task_brief(instruction=|goal=, sim=, start_room=, limit=3) -> {goal, brief (text for the LLM), episodes, strategies, retrieval_enabled}
m.clear(episodic=True, semantic=True)                # explicit reset; caller confirms intent
NaoMemory.normalize_goal(instruction, rooms=, objects=) -> "verb:token"   # deterministic goal key (NOT an LLM call)
```

Session-level (in `HumanoidSession`, `orbit_ros/humanoid.py`):
```python
s.task_memory(instruction=, limit=3)                 # retrieval for a NEW task (episodic+semantic), as the MCP tool `task_memory`
s.record_episode(instruction, result, experiment=, scenario=, feedback=, provenance=)   # write-back after a verified outcome
s.recall_memory(limit=5)                             # unchanged v1 here-and-now recall (MCP tool `recall_memory`)
```

Two read-only MCP tools expose retrieval to the LLM: `recall_memory` (here-and-now events) and **`task_memory`** (episodic+semantic for a
new instruction). Total advertised NAO tools: **45**.

## Retrieval (deterministic, not an LLM call, not fine-tuning)

`normalize_goal` maps an instruction to a coarse key like `go:kitchen` / `clean:bathroom` using the house's room/object vocabulary
(passed in by the session so `nao_memory` imports nothing from the robot layer — a leaf module, enforced by
`tests/test_pel_schema.py`). `recall_episodes` scores candidates: exact goal (+100) > same verb (+20) > same room/object token (+15) >
same start room (+5); ties broken by recency. This is transparent similarity, **explicitly not** semantic-vector search or model learning.

## Integration with the LLM (what makes it real learning-in-context)

Pipeline: instruction → live pose → **`task_memory(instruction)` retrieval** → the returned `brief` is prepended to the user turn of the
**real LLM request** → LLM plans (chooses MCP tools) → argument validation + safety guard → real robot action → observe true pose →
**`record_episode(...)` write-back** (+ `learn_strategy`) → next task can retrieve it.

Guarantees (verified by tests + live trials):
- Current observations override stale memory (Experiment E: a stale strategy's unsafe target was refused by the guard; the robot never entered the zone).
- Memory cannot bypass the guard, stop-latch, or argument validation (the guard never reads memory; `test_memory_never_changes_what_the_guard_sees`).
- Failed strategies are retrieved as cautions, not blindly repeated (Experiment B).
- Retrieval failure never fabricates context: `recall_episodes` returns `[]` and `task_brief` returns an empty `brief`; a broken db leaves
  the robot fully operational (`test_a_broken_memory_does_not_stop_the_robot`).
- Provenance: every episode/strategy stores where it came from (experiment/scenario/trial), so a plan's influencing memory is traceable.
- No secrets stored: only instructions, goals, rooms, tools, poses, outcomes — never API keys or credentials.

## Configuration

CLI (in `orbit_mcp_server.py`), surfaced in the startup log line `[orbit-mcp] memory: ...` and in experiment metadata:

| Flag | Effect | Default |
|---|---|---|
| `--no-memory` | memory fully off (no store, no recall) | memory on |
| `--memory-db PATH` | SQLite file (default `results_mcp/nao_memory.db`) | default path |
| `--no-memory-retrieval` | keep writing, but do not retrieve (`retrieval_enabled=False`) | retrieval on |
| `--no-memory-write` | keep retrieving, but do not write after a task (`write_after_task=False`) | write on |
| `--memory-max-records N` | cap rows per table (oldest dropped beyond) | 5000 |

Changing memory settings never alters previous benchmark records (separate db paths; historical files hashed in the Phase 10 registry).

## Reset / backup / migration

- Reset: `NaoMemory.clear(episodic=, semantic=)` (explicit, caller-confirmed). The default live db is never auto-cleared.
- Backup: copy the `.db` file (WAL-checkpointed on close). Phase 10 keeps a scoped experimental db under its results dir.
- Migration: opening an older db upgrades the schema in place; `schema_version()` reports the version.



## Phase 17 — consolidated memory view (read-only)

`orbit_ros/metatool/memory_view.py` gives the planner a **relevant scene subgraph and consolidated facts** instead of
a raw event log, without any schema migration (the v2 store already carries events/episodes/strategies/meta):

- `scene_subgraph(room=None)` — deterministic house→room→items hierarchy from `orbit_ros/house.py`.
- `short_term(memory, sim)` — most-recent observed objects, de-duplicated by label, bounded capacity (default 8),
  most-recent replacement.
- `consolidated(memory, sim, room)` — `{scene, short_term, taught_places, brief}` for the planner prompt.
- `memory_hit(view)` — for a memory-hit-rate metric.

Invariants are unchanged: memory is advisory only; live observations and the `HumanoidGuard` always win; this module
only READS and never migrates or writes the schema. Status: CODE-CHECK ONLY (tests in
`tests/test_phase17_metatool.py::TestMemoryView`); live hit-rate/stale-incident metrics are BLOCKED (Webots down).
See `docs/PHASE17_RESEARCH_UPGRADES_REPORT.md` feature 10.
