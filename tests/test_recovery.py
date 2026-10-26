import itertools, unittest
from orbit_ros.world2d import World2D, Obj
from orbit_ros.embodiment import Hypothesis
from orbit_ros.embodiment_runtime import hypothesis_space, apply_embodiment
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import PRESETS, MAX_ACTIONS
from orbit_ros.backends2d import OracleBackend
from orbit_ros.robust import RobustStrategy
from orbit_ros.voi import VoiStrategy, run_episode_v07
from orbit_ros.execution_monitor import ExecutionMonitor
from orbit_ros.recovery import RecoveryConfig, StepGuard
from orbit_ros.agent_recovery import run_episode_v08

def pose(x, y, yaw=0): return {"x": x, "y": y, "yaw_deg": yaw}
def mv(n): return {"tool": "move_forward", "args": {"distance": n}}

class MonitorTests(unittest.TestCase):
    NOM = Hypothesis(False, False, 1.0, 1.0)
    def test_ok_when_prediction_matches(self):
        m = ExecutionMonitor(); ev = m.after_hw(pose(2, 4), "move_forward", {"distance": 3}, pose(5, 4), {"ok": True, "message": "moved"}, self.NOM)
        self.assertEqual(ev.kind, "ok"); self.assertIsNone(m.tripped)

    def test_deviation_when_gain_differs(self):
        m = ExecutionMonitor(); ev = m.after_hw(pose(2, 4), "move_forward", {"distance": 4}, pose(3, 4), {"ok": True, "message": "moved"}, self.NOM)
        self.assertEqual(ev.kind, "deviation"); self.assertEqual(m.tripped, "deviation"); self.assertEqual(m.n_deviation, 1)

    def test_blocked_and_invalid_trip(self):
        m = ExecutionMonitor(); m.after_hw(pose(2, 4), "move_forward", {"distance": 3}, pose(3, 4), {"ok": False, "message": "blocked after 1 m"}, self.NOM)
        self.assertEqual(m.tripped, "blocked"); m.reset(); self.assertIsNone(m.tripped)
        m.after_hw(pose(2, 4), "turn", {"direction": "up", "degrees": 90}, pose(2, 4), {"ok": False, "message": "turn needs ..."}, self.NOM); self.assertEqual(m.tripped, "invalid")

    def test_suspended_and_disabled_record_nothing(self):
        for m in (ExecutionMonitor(enabled=False), ExecutionMonitor()):
            if m.enabled: m.suspended = True
            self.assertIsNone(m.after_hw(pose(2, 4), "move_forward", {"distance": 4}, pose(3, 4), {"ok": True, "message": "moved"}, self.NOM))
            self.assertEqual((m.events, m.tripped), ([], None))

    def test_turn_polarity_deviation(self):
        m = ExecutionMonitor(); ev = m.after_hw(pose(2, 4, 0), "turn", {"direction": "left", "degrees": 90}, pose(2, 4, 270), {"ok": True, "message": "turned"}, self.NOM)
        self.assertEqual(ev.kind, "deviation")

class StepGuardTests(unittest.TestCase):
    def guard(self, box, tau=0.0):
        world = apply_embodiment(World2D(14, 9, (2, 4), 0, [Obj("o0", "red box", *box)]), Hypothesis(False, False, 1.0, 1.0))
        st = RobustStrategy("none", "robust"); st.start(world)            # uniform prior over all 12 hypotheses
        ctx = {"pose": world.pose(), "raw": world.describe_view()}
        return world, StepGuard(st, ctx, set(), world.fov, tau)

    def test_clear_path_is_full_step(self):
        world, g = self.guard((9, 4)); a, risk, mode = g.fit(world.pose(), mv(2))
        self.assertEqual((mode, a["args"]["distance"]), ("full", 2)); self.assertEqual(risk, 0.0)

    def test_move_is_shortened_before_the_obstacle(self):
        world, g = self.guard((6, 4)); a, risk, mode = g.fit(world.pose(), mv(5))
        self.assertEqual(mode, "creep"); self.assertEqual(a["args"]["distance"], 3)      # (5,4) is the last free cell on the way to (6,4)

    def test_no_safe_step_when_blocked_ahead(self):
        world, g = self.guard((3, 4)); a, risk, mode = g.fit(world.pose(), mv(3))
        self.assertEqual(mode, "unsafe"); self.assertIsNone(a); self.assertGreater(risk, 0.0)

    def test_turns_and_hw_moves_pass_through(self):
        world, g = self.guard((3, 4)); t = {"tool": "turn", "args": {"direction": "left", "degrees": 90}}
        self.assertEqual(g.fit(world.pose(), t), (t, 0.0, "full"))

    def test_larger_tau_accepts_more(self):
        world, g0 = self.guard((6, 4), 0.0); _, g1 = self.guard((6, 4), 1.0)
        self.assertEqual(g1.fit(world.pose(), mv(5))[2], "full"); self.assertEqual(g0.fit(world.pose(), mv(5))[2], "creep")

def episode(runner, fam, yaw, seed, h, noise, voi=False, rcfg=None, drift=None):
    world, task = make_task(fam, seed, yaw, 2 if fam == "remember_return" else 3, False, 2 if fam == "motion" else 1); apply_embodiment(world, h)
    if drift: world.drift_schedule.append(drift)
    st = VoiStrategy("hybrid", voi=voi, eps=noise, noise_p=noise, seed=seed)
    be = OracleBackend(0, 0)
    return runner(world, task, PRESETS["S5"], be, st) if rcfg is None else run_episode_v08(world, task, PRESETS["S5"], be, st, rcfg)

class RunnerTests(unittest.TestCase):
    def test_defaults_are_off(self):
        r = RecoveryConfig(); self.assertFalse(r.enabled); self.assertFalse(r.monitor)

    def test_v08_with_monitor_and_recovery_off_reproduces_v07(self):
        for fam, yaw, h in itertools.product(("goto", "clean_left", "remember_return", "motion"), (0, 90), hypothesis_space()):
            for noise in (0.0, 0.1):
                a = episode(run_episode_v07, fam, yaw, 1, h, noise); b = episode(None, fam, yaw, 1, h, noise, rcfg=RecoveryConfig())
                for k in ("success", "collisions", "aborted", "task_probes", "env_tool_calls", "llm_calls", "final_x", "final_y", "actions_executed", "repairs"):
                    self.assertEqual(a[k], b[k], (fam, yaw, h.key, noise, k))

    def test_recovery_reduces_abstention_and_completes_more_tasks(self):
        cells = list(itertools.product(("goto", "clean_left", "remember_return", "motion"), (0, 90, 180, 270), hypothesis_space()))
        off = [episode(None, f, y, 100, h, 0.1, rcfg=RecoveryConfig()) for f, y, h in cells]
        on = [episode(None, f, y, 100, h, 0.1, rcfg=RecoveryConfig(enabled=True)) for f, y, h in cells]
        ab = lambda R: sum(1 for r in R if r["aborted"]); su = lambda R: sum(r["success"] for r in R)
        self.assertLess(ab(on), ab(off)); self.assertGreater(su(on), su(off))
        self.assertGreater(sum(r["rec_entries"] for r in on), 0); self.assertEqual(sum(r["rec_entries"] for r in off), 0)

    def test_recovery_counters_are_consistent_and_bounded(self):
        cells = itertools.product(("goto", "clean_left", "remember_return", "motion"), (0, 180), hypothesis_space())
        for f, y, h in cells:
            r = episode(None, f, y, 101, h, 0.1, rcfg=RecoveryConfig(enabled=True))
            self.assertLessEqual(r["rec_resolved_phases"], r["rec_entered_phases"]); self.assertLessEqual(r["rec_entered_phases"], r["rec_entries"])
            self.assertLessEqual(r["rec_collisions"], r["collisions"]); self.assertLessEqual(r["guarded_creeps"], r["guarded_steps"])
            self.assertLess(r["env_tool_calls"], 2 * MAX_ACTIONS + 200)

    def test_monitor_trips_on_mid_episode_drift(self):
        h = Hypothesis(False, False, 1.0, 1.0); trips = 0
        for f, y in itertools.product(("goto", "clean_left", "motion"), (0, 90, 180, 270)):
            r = episode(None, f, y, 100, h, 0.0, rcfg=RecoveryConfig(monitor=True), drift=(12, {"move_gain": 0.5})); trips += r["monitor_trips"]
        self.assertGreater(trips, 0)

    def test_monitor_is_silent_without_surprise(self):
        h = Hypothesis(False, False, 1.0, 1.0)
        for f, y in itertools.product(("goto", "motion"), (0, 90)):
            self.assertEqual(episode(None, f, y, 100, h, 0.0, rcfg=RecoveryConfig(monitor=True))["monitor_trips"], 0)

if __name__ == "__main__": unittest.main()
