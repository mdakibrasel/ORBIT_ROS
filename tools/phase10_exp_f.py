r"""Phase 10 Experiment F: memory retrieval relevance, latency, duplicate rate, and robustness to corrupted/unavailable memory.
Deterministic (no LLM, no Webots) measurement over a controlled, labelled memory set. Reported separately from live LLM trials.

    .venv\Scripts\python.exe tools\phase10_exp_f.py --out <dir>
"""
from __future__ import annotations
import argparse, json, os, sqlite3, sys, tempfile, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros.nao_memory import NaoMemory

ROOMS = ["kitchen", "bathroom", "bedroom", "living_room"]

def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    res = {"generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}

    # --- Build a labelled set: for each of 4 goals, one relevant episode; plus 6 unrelated episodes. ---
    d = tempfile.mkdtemp(); db = os.path.join(d, "f.db"); m = NaoMemory(db)
    for r in ROOMS:
        m.add_episode(f"go to the {r}", sim="webots", success=True, start={"room": "living_room"},
                      executed=[{"tool": "go_to_room", "status": "ok"}], provenance={"label": f"go:{r}"})
    for i in range(6):
        m.add_episode(f"clean room variant {i}", sim="webots", success=bool(i % 2), start={"room": "kitchen"}, goal=f"clean:room{i}")

    # --- Retrieval relevance: query each goal, check the top hit is the matching goal. precision@1 ---
    hits = 0; lat = []
    for r in ROOMS:
        t = time.perf_counter()
        got = m.recall_episodes(instruction=f"please walk to the {r}", sim="webots", start_room="living_room", limit=3)
        lat.append((time.perf_counter() - t) * 1000)
        top_goal = got[0]["goal"] if got else None
        hits += int(top_goal == f"go:{r}")
    res["retrieval_precision_at_1"] = {"hits": hits, "queries": len(ROOMS), "precision": round(hits / len(ROOMS), 3)}
    res["retrieval_latency_ms"] = {"n": len(lat), "mean": round(sum(lat) / len(lat), 2), "max": round(max(lat), 2)}

    # --- Write latency ---
    wl = []
    for i in range(20):
        t = time.perf_counter(); m.add_episode(f"timing probe {i}", sim="webots", goal=f"probe{i}", start={"room": "kitchen"}); wl.append((time.perf_counter() - t) * 1000)
    res["write_latency_ms"] = {"n": len(wl), "mean": round(sum(wl) / len(wl), 2), "max": round(max(wl), 2)}

    # --- Duplicate rate: insert the SAME (sim, goal, start_room) episode 5x; dedup should keep 1 ---
    before = m.count_episodes()
    for _ in range(5): m.add_episode("go to the kitchen", sim="webots", success=True, start={"room": "living_room"})
    added = m.count_episodes() - before
    res["duplicate_control"] = {"inserts": 5, "net_new_rows": added, "dedup_ok": added <= 1}

    # --- Strategy conflict / confidence: 3 successes + 1 failure for one goal ---
    for _ in range(3): m.learn_strategy("go:kitchen", tool="go_to_room", args={"room": "kitchen"}, success=True, sim="webots")
    m.learn_strategy("go:kitchen", tool="go_to_room", args={"room": "kitchen"}, success=False, sim="webots")
    st = m.recall_strategies("go:kitchen", sim="webots")[0]
    res["strategy_confidence"] = {"successes": st["successes"], "failures": st["failures"], "confidence": st["confidence"]}
    m.close()

    # --- Robustness: corrupted db file ---
    cdb = os.path.join(d, "corrupt.db")
    with open(cdb, "wb") as f: f.write(b"this is not a sqlite database at all" * 10)
    corrupt_handled = None
    try:
        NaoMemory(cdb); corrupt_handled = "opened_without_raising"     # sqlite may still open then fail on use
    except sqlite3.DatabaseError as e:
        corrupt_handled = f"raised DatabaseError (caught): {type(e).__name__}"
    except Exception as e:
        corrupt_handled = f"raised {type(e).__name__}"
    res["corrupted_db"] = {"result": corrupt_handled,
                           "note": "The session wraps memory writes in try/except and runs without memory if it fails "
                                   "(tested live by test_a_broken_memory_does_not_stop_the_robot)."}

    # --- Unavailable memory: session with memory=False retrieves/writes nothing ---
    from orbit_ros.humanoid import HumanoidSession, FakeNao
    s = HumanoidSession(sim="fake", log_dir=False, backend=FakeNao(), memory=False); s.load_scene("living_room")
    res["memory_disabled"] = {"task_memory_enabled": s.task_memory(instruction="go to the kitchen")["enabled"],
                              "record_episode_enabled": s.record_episode("x", {"status": "ok"})["enabled"]}

    with open(os.path.join(a.out, "expF_metrics.json"), "w", encoding="utf-8") as f: json.dump(res, f, indent=2)
    print(json.dumps(res, indent=2)); print("SAVED expF ->", a.out)
    return 0

if __name__ == "__main__": sys.exit(main())
