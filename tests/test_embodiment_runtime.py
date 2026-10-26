import random
import unittest
from orbit_ros.world2d import World2D, Obj
from orbit_ros.embodiment import EmbodimentIdentifier
from orbit_ros.embodiment_runtime import (
    hypothesis_space, apply_embodiment, hypothesis_of, make_bay, build_probe, run_probe, PROBES, NOMINAL,
    identify_infogain, identify_fixed, identify_random, identify_kinematic, EmbodimentAdapter, EmbodimentStrategy,
    STRATEGY_NAMES, _realised, MOVE_GAINS)
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.world2d import Obj as _Obj

def key3(h): return (h.mirrored_camera, h.left_right_swapped, round(h.move_gain, 3))

class WorldEmbodimentTests(unittest.TestCase):
    def test_defaults_are_nominal(self):
        w = World2D(9, 7, (4, 3), 0, [])
        self.assertEqual(hypothesis_of(w), NOMINAL)
    def test_swap_reverses_turns(self):
        w = World2D(9, 7, (4, 3), 0, [], swapped=True)
        w.execute("turn", {"direction": "left", "degrees": 90}); self.assertEqual(w.yaw, 270)
        w.execute("turn", {"direction": "right", "degrees": 90}); self.assertEqual(w.yaw, 0)
    def test_move_gain_scales_travel_but_reports_commanded(self):
        w = World2D(20, 5, (1, 2), 0, [], move_gain=0.5)
        r = w.execute("move_forward", {"distance": 8}); self.assertEqual(w.x, 5); self.assertEqual(r["message"], "moved 8 m")
    def test_drift_schedule_applies_once(self):
        w = World2D(9, 7, (4, 3), 0, []); w.drift_schedule = [(2, {"swapped": True})]
        w.execute("get_pose", {}); self.assertFalse(w.swapped)
        w.execute("get_pose", {}); self.assertTrue(w.swapped); self.assertEqual(w.drift_schedule, [])

class ProbeTests(unittest.TestCase):
    def test_tour_distinguishes_all_hypotheses(self):
        hs = hypothesis_space(); self.assertEqual(len(hs), 12)
        labels = {run_probe(make_bay(h), PROBES["tour"][0]) for h in hs}
        self.assertEqual(len(labels), 12)
    def test_builtin_probes_are_safe_under_every_hypothesis(self):
        for n in PROBES: self.assertTrue(build_probe(n, hypothesis_space())["safe"], n)
    def test_out_of_bounds_probe_is_marked_unsafe_and_never_chosen(self):
        PROBES["bad"] = ([{"tool": "move_forward", "args": {"distance": 12}}] * 3, 36.0)    # 36 m: leaves the 21 m bay
        try:
            self.assertFalse(build_probe("bad", hypothesis_space())["safe"])
            ident = EmbodimentIdentifier((False, True), (False, True), MOVE_GAINS, (1.0,))
            chosen = ident.choose_probe([build_probe("bad", ident.hypotheses)])
            self.assertIsNone(chosen)
        finally: del PROBES["bad"]
    def test_cost_weight_changes_choice(self):
        hs = hypothesis_space()
        free = EmbodimentIdentifier((False, True), (False, True), MOVE_GAINS, (1.0,))
        self.assertEqual(free.choose_probe([build_probe(n, hs) for n in PROBES])["name"], "tour")
        thrifty = EmbodimentIdentifier((False, True), (False, True), MOVE_GAINS, (1.0,), cost_weight=0.3)
        self.assertNotEqual(thrifty.choose_probe([build_probe(n, hs) for n in PROBES])["name"], "tour")

class IdentificationTests(unittest.TestCase):
    def test_infogain_identifies_every_hypothesis_with_one_probe(self):
        for h in hypothesis_space():
            r = identify_infogain(h)
            self.assertEqual(key3(r.belief), key3(h)); self.assertEqual(r.probes, 1); self.assertEqual(r.status, "identified")
    def test_fixed_probe_always_spends_full_budget_and_is_correct(self):
        for h in hypothesis_space():
            r = identify_fixed(h); self.assertEqual(r.probes, 3); self.assertEqual(key3(r.belief), key3(h))
    def test_random_probe_correct_when_it_stops(self):
        for i, h in enumerate(hypothesis_space()):
            r = identify_random(h, rng=random.Random(i)); self.assertEqual(key3(r.belief), key3(h))
    def test_kinematic_cal_cannot_see_mirror_but_gets_actuators(self):
        for h in hypothesis_space():
            r = identify_kinematic(h)
            self.assertFalse(r.belief.mirrored_camera)
            self.assertEqual((r.belief.left_right_swapped, r.belief.move_gain), (h.left_right_swapped, h.move_gain))
    def test_noisy_observation_needs_more_than_one_probe_with_soft_update(self):
        rng = random.Random(1); counts = []
        for _ in range(30):
            r = identify_infogain(hypothesis_space()[5], eps=0.1, noise_p=0.1, rng=rng, max_probes=6); counts.append(r.probes)
        self.assertGreaterEqual(min(counts), 2)
    def test_card_is_small_json(self):
        c = identify_infogain(hypothesis_space()[7]).card()
        self.assertEqual(set(c), {"camera_mirrored", "left_right_swapped", "move_gain", "turn_gain", "confidence", "status", "probes_used"})

class AdapterTests(unittest.TestCase):
    def test_adapter_realises_every_distance_exactly_for_every_gain(self):
        for g in MOVE_GAINS:
            for n in range(0, 9):
                self.assertEqual(_realised(min(range(0, int(n / g) + 6), key=lambda c: (abs(_realised(c, g) - n), c)), g), n, (g, n))
    def test_adapter_moves_world_by_commanded_distance(self):
        for h in hypothesis_space():
            w = apply_embodiment(World2D(20, 9, (1, 4), 0, []), h); ad = EmbodimentAdapter(h)
            for t, a in ad.translate("move_forward", {"distance": 5}): w.execute(t, a)
            self.assertEqual(w.x, 6, h.key)
            for t, a in ad.translate("turn", {"direction": "left", "degrees": 90}): w.execute(t, a)
            self.assertEqual(w.yaw, 90, h.key)

class EpisodeTests(unittest.TestCase):
    def _rate(self, strategy_name, families=("motion", "goto", "remember_return"), seeds=2):
        ok = n = 0
        for h in hypothesis_space():
            for fam in families:
                for yaw in (0, 90, 180, 270):
                    for s in range(seeds):
                        w, t = make_task(fam, s, yaw, 3 if fam != "remember_return" else 2, False, 2)
                        apply_embodiment(w, h)
                        row = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0), embodiment=EmbodimentStrategy(strategy_name, seed=s))
                        ok += row["success"]; n += 1
        return ok / n
    def test_oracle_and_infogain_match_and_beat_nominal(self):
        o, i, nom = self._rate("oracle"), self._rate("infogain"), self._rate("nominal")
        self.assertGreater(o, 0.95); self.assertAlmostEqual(i, o, places=6); self.assertLess(nom, o - 0.2)
    def test_all_strategy_names_run(self):
        for name in STRATEGY_NAMES:
            w, t = make_task("goto", 0, 90, 3, True); apply_embodiment(w, hypothesis_space()[9])
            row = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0), embodiment=EmbodimentStrategy(name))
            self.assertIn("belief_key", row)
    def test_legacy_path_unchanged_without_embodiment(self):
        w, t = make_task("goto", 0, 90, 3, True)
        self.assertNotIn("belief_key", run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0)))
    def test_surprise_monitor_reidentifies_after_actuator_drift(self):
        w, t = make_task("motion", 0, 0, 1, False, 3)
        w.drift_schedule = [(3, {"swapped": True})]
        strat = EmbodimentStrategy("infogain_monitor", monitor_threshold=1, seed=0)
        row = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0), embodiment=strat)
        self.assertGreaterEqual(row["surprise_triggers"], 1); self.assertEqual(row["belief_correct"], 1)
    def test_no_false_trigger_without_drift(self):
        for h in hypothesis_space():
            w, t = make_task("motion", 1, 90, 1, False, 3); apply_embodiment(w, h)
            strat = EmbodimentStrategy("infogain_monitor"); row = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0), embodiment=strat)
            self.assertEqual(row["surprise_triggers"], 0, h.key)
    def test_post_failure_only_identifies_after_a_failure(self):
        w, t = make_task("motion", 0, 0, 1, False, 1)            # nominal robot: nothing fails -> no probing
        strat = EmbodimentStrategy("post_failure"); row = run_episode(w, t, PRESETS["S5"], OracleBackend(0, 0), embodiment=strat)
        self.assertEqual(row["id_probes"], 0)

if __name__ == "__main__": unittest.main()
