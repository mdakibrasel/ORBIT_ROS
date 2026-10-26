"""v0.11: clarification (`ask`), refusal (`abstain`), safety metrics, lesson memory, navigation tools, realism knobs."""
import json, os, unittest
from orbit_ros.tasks2d import make_task, check_success, CLARIFY_DOOR
from orbit_ros.agent2d import run_episode, PRESETS, NO_CLARIFICATION, MAX_ASKS
from orbit_ros.backends2d import OracleBackend, LLMBackend
from orbit_ros.llm import ScriptedClient

def replies(*xs):
    it = iter(xs); return ScriptedClient(lambda p, tag: next(it))

class AskTests(unittest.TestCase):
    def test_ask_returns_the_intent_and_is_counted_as_needed(self):
        w, t = make_task("ambiguous", 0, 0, 3, False, 4)
        cl = replies('{"tool":"ask","args":{"question":"whose left?"}}', '{"tool":"done","args":{}}')
        row = run_episode(w, t, PRESETS["C1"], LLMBackend(cl, False, False))
        self.assertEqual((row["asks"], row["asks_when_needed"], row["unnecessary_asks"], row["ask_needed_phases"]), (1, 1, 0, 1))
        self.assertIn(CLARIFY_DOOR, cl.prompts[-1][1])                      # the answer reaches the next prompt
    def test_unnecessary_question_and_limit(self):
        w, t = make_task("goto", 0, 0, 1, False, 1)
        q = '{"tool":"ask","args":{"question":"sure?"}}'
        cl = replies(q, q, q, '{"tool":"done","args":{}}')
        row = run_episode(w, t, PRESETS["S0"], LLMBackend(cl, False, False))
        self.assertEqual((row["asks"], row["unnecessary_asks"]), (MAX_ASKS, MAX_ASKS)); self.assertIn(NO_CLARIFICATION, cl.prompts[-1][1])
    def test_level4_needs_the_door_frame_and_the_oracle_follows_the_answer(self):
        w, t = make_task("ambiguous", 0, 90, 3, False, 4); self.assertEqual(t.expected["target_id"], "o1")
        asked = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0, ask_p=1.0))
        w, t = make_task("ambiguous", 0, 90, 3, False, 4)
        silent = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0, ask_p=0.0))
        self.assertEqual((asked["success"], silent["success"]), (1, 0)); self.assertEqual(asked["interpretation"], "door_frame")
    def test_plan_mode_replans_after_an_answer(self):
        w, t = make_task("ambiguous", 0, 0, 3, False, 4)
        row = run_episode(w, t, PRESETS["S1"], OracleBackend(0, 0, 0, ask_p=1.0)); self.assertEqual(row["success"], 1)
    def test_prompt_offers_the_tools_only_when_interactive(self):
        w, t = make_task("goto", 0, 0, 1, False, 1); cl = replies('{"tool":"done","args":{}}')
        run_episode(w, t, PRESETS["S0"], LLMBackend(cl, False, False)); self.assertIn('"tool":"abstain"', cl.prompts[0][1])
        w, t = make_task("goto", 0, 0, 1, False, 1); cl = replies('{"tool":"done","args":{}}')
        run_episode(w, t, PRESETS["S0"], LLMBackend(cl, False, False, interactive=False)); self.assertNotIn('"tool":"ask"', cl.prompts[0][1])

class SafetyTests(unittest.TestCase):
    def test_impossible_tasks_need_an_abstention_without_collisions(self):
        for lv in (1, 2):
            w, t = make_task("impossible", 0, 270, 3, False, lv)
            self.assertFalse(check_success(w, t)["success"]); w.abstained = True; self.assertTrue(check_success(w, t)["success"])
        w, t = make_task("impossible", 0, 0, 3, False, 2)
        cl = replies(json.dumps({"tool": "move_forward", "args": {"distance": t.phases[0].params["ops"][0][1]}}), '{"tool":"done","args":{}}')
        row = run_episode(w, t, PRESETS["S0"], LLMBackend(cl, False, False))
        self.assertEqual((row["success"], row["wall_bumps"], row["safe_success"]), (0, 1, 0))
    def test_oracle_abstains_on_impossible_and_legacy_oracle_does_not(self):
        for lv in (1, 2):
            w, t = make_task("impossible", 0, 0, 3, False, lv); r = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0, abstain=True))
            self.assertEqual((r["success"], r["abstained"]), (1, 1))
        w, t = make_task("impossible", 0, 0, 3, False, 1); self.assertEqual(run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0, 0))["abstained"], 0)
    def test_abstain_ends_the_episode(self):
        w, t = make_task("sequence", 0, 0, 3, False, 1)
        cl = replies('{"tool":"abstain","args":{"reason":"x"}}')
        row = run_episode(w, t, PRESETS["C1"], LLMBackend(cl, False, False)); self.assertEqual((row["abstained"], row["llm_calls"]), (1, 1))
    def test_wall_and_object_hits_are_split(self):
        from orbit_ros.world2d import World2D, Obj
        w = World2D(5, 5, (2, 2), 0, [Obj("a", "box", 4, 2)])
        w.execute("move_forward", {"distance": 3}); w.execute("turn", {"direction": "left", "degrees": 90}); w.execute("move_forward", {"distance": 5})
        self.assertEqual((w.object_hits, w.wall_bumps, w.collisions), (1, 1, 2))


# ------------------------------------------------------------------------------------------------ lessons, nav tools, hierarchy, realism
import sys, tempfile, contextlib, io
from orbit_ros.agent2d import config_by_name, normalize_plan
from orbit_ros.lessons import LessonStore, template_lesson
from orbit_ros import navtools
from orbit_ros.world2d import World2D, Obj, Realism
from orbit_ros.tasks2d import side_region

class LessonTests(unittest.TestCase):
    def lesson(self, tid, fam="clean_left", text="use the start heading", effect="frame", level=1, yaw=0):
        return {"task_id": tid, "family": fam, "level": level, "yaw": yaw, "instruction": "Clean the left side of the room.", "text": text, "effect": effect}
    def test_retrieval_skips_the_same_instance_dedupes_and_ranks_family_first(self):
        _, t = make_task("clean_left", 0, 0, 1, False, 1)
        st = LessonStore(); st.add(self.lesson(t.task_id)); st.add(self.lesson("x1")); st.add(self.lesson("x2"))
        st.add(self.lesson("x3", fam="goto", text="other", effect="")); st.add(self.lesson("x4", text="second"))
        got = st.retrieve(t, k=3)
        self.assertNotIn(t.task_id, [l["task_id"] for l in got])
        self.assertEqual(len({l["text"] for l in got}), len(got))
        self.assertEqual(got[0]["family"], "clean_left")
    def test_store_persists_and_reloads(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "l.jsonl"); LessonStore(p).add(self.lesson("a")); self.assertEqual(len(LessonStore(p).items), 1)
    def test_templates_and_configs(self):
        self.assertEqual(template_lesson("left_right_confusion")[1], "frame"); self.assertEqual(template_lesson("??")[1], "")
        self.assertTrue(PRESETS["S6"].lessons and config_by_name("C4+L").lessons and not PRESETS["S5"].lessons)
    def test_frame_lesson_removes_the_oracle_confusion(self):
        res = []
        for les in (None, [{"id": 0, "text": "start heading", "effect": "frame"}]):
            w, t = make_task("clean_left", 0, 90, 1, False, 1)       # confusion needs an ungrounded config and a start heading != 0
            res.append(run_episode(w, t, PRESETS["S1"], OracleBackend(0.0, 1.0), lessons=les)["success"])
        self.assertEqual(res, [0, 1])

class NavToolTests(unittest.TestCase):
    pose = {"x": 0, "y": 0, "yaw_deg": 0, "room_w": 4, "room_h": 3}
    def test_goto_cell_goes_around_a_known_obstacle(self):
        acts, err = navtools.expand("goto_cell", {"x": 2, "y": 0}, self.pose, (0, 0, 0), [(1, 0)])
        self.assertIsNone(err)
        w = World2D(4, 3, (0, 0), 0, [Obj("o0", "box", 1, 0)])
        for a in acts: self.assertTrue(w.execute(a["tool"], a["args"])["ok"])
        self.assertEqual((w.x, w.y), (2, 0))
    def test_bad_arguments_are_errors(self):
        self.assertTrue(navtools.expand("goto_cell", {"x": 9, "y": 0}, self.pose, (0, 0, 0), [])[1])
        self.assertTrue(navtools.expand("goto_cell", {"x": "a"}, self.pose, (0, 0, 0), [])[1])
        self.assertTrue(navtools.expand("cover_region", {"side": "up"}, self.pose, (0, 0, 0), [])[1])
        self.assertTrue(navtools.expand("cover_region", {"side": "left", "avoid": 5}, self.pose, (0, 0, 0), [])[1])
    def test_cover_region_cleans_the_whole_half_and_respects_avoid(self):
        w = World2D(6, 4, (3, 2), 90, [])
        acts, err = navtools.expand("cover_region", {"side": "left", "avoid": [[0, 3]]}, w.pose(), w.start_pose, [])
        for a in acts: w.execute(a["tool"], a["args"])
        region = side_region(6, 4, w.start_pose, "left")
        self.assertEqual(w.cleaned, region - {(0, 3)}); self.assertNotIn((0, 3), w.visited)
    def test_oracle_with_nav_tools_solves_clean_left_in_few_decisions(self):
        w, t = make_task("clean_left", 1, 90, 3, False, 1)
        row = run_episode(w, t, PRESETS["S2"], OracleBackend(0.0, 0.0), nav_tools=True)
        w, t = make_task("clean_left", 1, 90, 3, False, 1)
        base = run_episode(w, t, PRESETS["S2"], OracleBackend(0.0, 0.0))
        self.assertEqual((row["success"], base["success"]), (1, 1)); self.assertLessEqual(row["actions_executed"], 5); self.assertGreater(base["actions_executed"], 20)   # one macro = one decision

class HierarchyTests(unittest.TestCase):
    def test_regions_are_flattened_with_their_name(self):
        p = normalize_plan({"regions": [{"name": "left strip", "actions": [{"tool": "clean"}, {"tool": "turn", "args": {"direction": "left", "degrees": 90}}]},
                                        {"name": "back", "steps": [{"tool": "move_forward", "args": {"distance": 1}}]}, "junk"]})
        self.assertEqual([a["tool"] for a in p["actions"]], ["clean", "turn", "move_forward"])
        self.assertEqual([a["_region"] for a in p["actions"]], ["left strip", "left strip", "back"])
    def test_flat_plan_unchanged(self):
        self.assertEqual(normalize_plan([{"tool": "clean", "args": {}}])["actions"][0]["tool"], "clean")

class RealismTests(unittest.TestCase):
    def test_defaults_are_the_nominal_world(self):
        w, t = make_task("goto", 0, 0, 3, False, 1)
        self.assertFalse(w.realism.active()); self.assertEqual(t.task_id.count("|"), 5); self.assertEqual(t.factors["room_scale"], 1)
        row = run_episode(w, t, PRESETS["S5"], OracleBackend(0.05, 0.3)); self.assertNotIn("odom_errors", row)
    def test_odometry_error_is_invisible_to_the_reported_pose(self):
        w = World2D(9, 3, (0, 1), 0, []); w.set_realism(Realism(odom_p=1.0, seed="t"))
        w.execute("move_forward", {"distance": 3})
        self.assertEqual(w.pose()["x"], 3); self.assertIn(w.x, (2, 4)); self.assertEqual(w.odom_errors, 1)
    def test_blocked_motion_resyncs_the_step_count(self):
        w = World2D(3, 1, (0, 0), 0, []); w.set_realism(Realism(odom_p=1.0, seed="t"))
        r = w.execute("move_forward", {"distance": 5}); self.assertFalse(r["ok"])
    def test_detection_drop_noise_and_stale_view(self):
        objs = [Obj("o0", "red box", 3, 0), Obj("o1", "blue ball", 3, 1)]
        w = World2D(5, 3, (0, 0), 0, objs); w.set_realism(Realism(det_drop=1.0, seed="a"))
        self.assertEqual(w.describe_view(), []); self.assertEqual(w.dropped_detections, 2)
        w = World2D(5, 3, (0, 0), 0, objs); clean = w.describe_view(); w.set_realism(Realism(img_noise=0.2, seed="a"))
        noisy = w.describe_view(); self.assertEqual([v["label"] for v in noisy], [v["label"] for v in clean]); self.assertNotEqual(noisy, clean)
        w = World2D(5, 3, (0, 0), 0, objs); w.set_realism(Realism(stale_p=1.0, seed="a"))
        first = w.describe_view(); w.execute("turn", {"direction": "left", "degrees": 90})
        self.assertEqual(w.describe_view(), first); self.assertEqual(w.stale_views, 1)
    def test_realism_is_deterministic_per_seed(self):
        rows = []
        for _ in range(2):
            w, t = make_task("goto", 2, 90, 3, False, 1); w.set_realism(Realism(0.2, 0.2, 0.1, 0.2, seed=t.task_id))
            r = run_episode(w, t, PRESETS["S2"], OracleBackend(0.05, 0.3)); rows.append({k: v for k, v in r.items() if not k.endswith("_s")})
        self.assertEqual(rows[0], rows[1]); self.assertIn("final_pose_error_m", rows[0])
    def test_room_scale_and_task_id_round_trip(self):
        from failure_taxonomy import parse_tid
        w, t = make_task("clean_left", 0, 0, 3, False, 1, 2)
        self.assertEqual((w.w, w.h), (12, 8)); self.assertTrue(t.task_id.endswith("|r2"))
        self.assertEqual(parse_tid(t.task_id), ("clean_left", 0, 0, 3, False, 1, 2))
        w, t = make_task("goto", 0, 0, 3, False, 1, 2); self.assertEqual((w.w, w.h), (17, 13))
        w, t = make_task("motion", 0, 0, 1, False, 1, 2); self.assertEqual((w.w, w.h), (9, 7)); self.assertNotIn("|r", t.task_id)
        self.assertEqual(parse_tid(t.task_id)[-1], 1)

class CompareRunsTests(unittest.TestCase):
    def test_paired_report_lists_the_differing_setting(self):
        import run_spatial, compare_runs
        with tempfile.TemporaryDirectory() as d:
            a, b = os.path.join(d, "a"), os.path.join(d, "b")
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                run_spatial.main(["--families", "clean_left", "--seeds", "1", "--configs", "S2", "--out", a])
                run_spatial.main(["--families", "clean_left", "--seeds", "1", "--configs", "S2", "--nav-tools", "--out", b])
                old = sys.argv; sys.argv = ["compare_runs.py", a, b, "--out", os.path.join(d, "c.md")]
                try: compare_runs.main()
                finally: sys.argv = old
            txt = open(os.path.join(d, "c.md"), encoding="utf-8").read()
            self.assertIn("`nav_tools`: False -> True", txt); self.assertIn("SYNTHETIC", txt); self.assertIn("| S2 | ALL | 12 |", txt)

if __name__ == "__main__": unittest.main()
