---
inclusion: always
---

# ORBIT global rules (section 1)

* Every robot motion goes through `HumanoidSession.robot_call` → argument validation → preconditions → `HumanoidGuard` → motion.
  Nothing new may bypass the guard, the stop-latch or the call budget. The guard must never read memory or LLM output.
* No silent simulator cheating. No teleport, supervisor pose reset or physics reset during a task, except for `load_scene` (scene start).
  If an explicit debug flag allows a fallback, it is counted, and the trial is marked `invalid`.
* LIVE ONLY — no fake anything. This project is presented live to a professor, so:
  - no `FakeNao`, no `--sim fake`, no kinematic stand-in, no World2D/oracle simulator, no mock or scripted LLM, no replayed or
    pre-recorded LLM answers and no hand-made result numbers anywhere in the running system, the demos, the benchmarks or the reports;
  - the robot is always the NAO in the live Webots house, and LLM calls always go to a real provider;
  - every test that involves the robot or an LLM is a LIVE test against Webots (marked `@pytest.mark.live`) and produces evidence
    (section 2);
  - only pure code checks that touch neither a robot nor an LLM may run without Webots: JSON parsing, the schema generator, the plan
    validator on hand-written plan JSON, the SQLite memory, the docs consistency. These are labelled "code checks" and are never
    presented as robot results;
  - a task or tool is "done" only when its live check (section 2) has passed and its evidence pack exists.
* Never delete or modify existing measured results (section 9 says where they go). Never re-label old numbers.
* Core code uses only the standard library + numpy (it must run inside Webots' Python). Small, readable modules; no researcher or
  paper names anywhere in code, comments or docs.
* Before editing a file, keep a copy in `_work_backup/<timestamp>/` (git-ignored, excluded from releases).
* After each phase, run:
  `.\.venv\Scripts\python.exe -m pytest -q -m "not live"` (code checks), then, with Webots running,
  `.\.venv\Scripts\python.exe -m pytest -q -m live` and
  `.\.venv\Scripts\python.exe orbit_mcp_server.py --selftest --robot nao --sim webots`, then
  the phase's live checks. Report the files changed, the tests (passed/failed/skipped), the live checks with the evidence paths,
  and anything that was not verified live.
* If a live check fails, fix the cause. Do not weaken the check, and do not mark the task done.
