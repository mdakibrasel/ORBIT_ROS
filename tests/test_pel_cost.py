"""ORBIT-PEL M6: prices from the pricing file only, cost per call, the spending cap, the cost report and the optional v1 prompt.
No network: the model is a scripted test double."""
import csv, json, os, tempfile, unittest
from orbit_ros.pel.cost import load_pricing, price_for, call_cost, CostCappedClient, CostCapReached, PricingError, default_pricing_path
from orbit_ros.pel.prompts import PelV1Backend, PEL_V1_FORMAT
from orbit_ros.llm import ScriptedClient
from orbit_ros.backends2d import LLMBackend
from orbit_ros.agent2d import run_episode, PRESETS
from orbit_ros.world2d import World2D
from orbit_ros.tasks2d import Task, Phase
from tools.cost_report import report, incremental, saving_pct

PRICE = {"provider": "p", "model": "m", "input": 1.0, "output": 10.0, "cached_input": 0.1, "source_url": "https://example.org/pricing", "retrieved": "2026-10-18"}

def write_pricing(entries, currency="USD"):
    d = tempfile.mkdtemp(); p = os.path.join(d, "pricing.json")
    with open(p, "w") as f: json.dump({"currency": currency, "models": entries}, f)
    return p

class PricingTests(unittest.TestCase):
    def test_shipped_pricing_file_is_valid_and_sourced(self):
        pr = load_pricing(default_pricing_path())
        for (prov, model), e in pr["entries"].items():
            self.assertTrue(e["source_url"].startswith("https://")); self.assertRegex(e["retrieved"], r"^\d{4}-\d{2}-\d{2}$")
    def test_entries_without_source_or_price_are_rejected(self):
        for bad in ({**PRICE, "source_url": ""}, {k: v for k, v in PRICE.items() if k != "retrieved"}, {**PRICE, "input": -1}, {**PRICE, "output": "3"}):
            with self.assertRaises(PricingError): load_pricing(write_pricing([bad]))
        with self.assertRaises(PricingError): load_pricing(write_pricing([PRICE], currency=""))
    def test_unknown_model_has_no_price(self):
        pr = load_pricing(write_pricing([PRICE]))
        self.assertIsNotNone(price_for(pr, "p", "m")); self.assertIsNone(price_for(pr, "p", "other")); self.assertIsNone(price_for(None, "p", "m"))

class CallCostTests(unittest.TestCase):
    def test_formula(self):
        u = {"input_tokens": 1_000_000, "cached_tokens": 400_000, "output_tokens": 100_000, "reasoning_tokens": 50_000}
        self.assertAlmostEqual(call_cost(u, PRICE), 0.6 * 1.0 + 0.4 * 0.1 + 0.15 * 10.0)
        self.assertAlmostEqual(call_cost({"input_tokens": 10, "output_tokens": 0}, PRICE), 10 / 1e6)
        self.assertIsNone(call_cost({"input_tokens": None, "output_tokens": 5}, PRICE))
        self.assertAlmostEqual(call_cost({"input_tokens": 100, "output_tokens": 0, "cached_tokens": 500}, PRICE), 100 * 0.1 / 1e6)   # cached <= input
    def test_tiers(self):
        p = {**PRICE, "tier_threshold_tokens": 200_000, "input_above_threshold": 2.0, "output_above_threshold": 20.0}
        self.assertAlmostEqual(call_cost({"input_tokens": 300_000, "output_tokens": 0}, p), 0.6)
        self.assertAlmostEqual(call_cost({"input_tokens": 100_000, "output_tokens": 0}, p), 0.1)

class CapTests(unittest.TestCase):
    def client(self, out_tokens=100_000):
        c = ScriptedClient(lambda p, t: "{}"); real = c.complete
        def complete(prompt, tag="", images=None):
            r = real(prompt, tag); c.usage_events[-1].update(input_tokens=0, output_tokens=out_tokens); return r
        c.complete = complete; return c
    def test_cap_refuses_before_sending(self):
        inner = self.client(); cc = CostCappedClient(inner, PRICE, 2.5)     # each call costs 1.0
        for _ in range(3): cc.complete("x")
        with self.assertRaises(CostCapReached): cc.complete("x")
        self.assertEqual(len(inner.prompts), 3); self.assertEqual(cc.refused, 1); self.assertAlmostEqual(cc.spent(), 3.0)
    def test_unknown_usage_fails_closed(self):
        inner = ScriptedClient(lambda p, t: "{}"); cc = CostCappedClient(inner, PRICE, 100.0)
        inner.usage_events.append({"tag": "x", "input_tokens": None, "output_tokens": None})
        with self.assertRaises(CostCapReached): cc.complete("x")
    def test_no_price_no_cap(self):
        with self.assertRaises(PricingError): CostCappedClient(ScriptedClient(lambda p, t: "{}"), None, 1.0)
        with self.assertRaises(PricingError): CostCappedClient(ScriptedClient(lambda p, t: "{}"), PRICE, 0)

class ReportTests(unittest.TestCase):
    def run_dir(self, trials, usage):
        d = tempfile.mkdtemp()
        for name, rows in (("trial_results.csv", trials), ("api_usage.csv", usage)):
            if not rows: continue
            with open(os.path.join(d, name), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=sorted({k for r in rows for k in r})); w.writeheader(); w.writerows(rows)
        return d
    def test_zero_successes_give_undefined_cost_per_success(self):
        trials = [{"config": "A", "success": 0, "llm_calls": 1, "outcome_code": "task_failure"},
                  {"config": "A", "success": 0, "llm_calls": 1, "outcome_code": "infra_error", "error": "HTTP 500"}]
        usage = [{"config": "A", "provider": "p", "model_id": "m", "input_tokens": 1_000_000, "output_tokens": 0, "cached_tokens": "", "reasoning_tokens": "", "latency_s": 1, "attempts": 2}] * 2
        text, s = report(self.run_dir(trials, usage), write_pricing([PRICE]))
        a = s["configs"]["A"]
        self.assertEqual((a["attempted"], a["infra_errors"], a["successes"], a["provider_retries"]), (1, 1, 0, 2))
        self.assertIsNone(a["cost_per_success"]); self.assertIn("undefined (0 successes)", text)
        self.assertAlmostEqual(a["cost_total"], 2.0); self.assertAlmostEqual(a["cost_per_attempted"], 1.0)
    def test_unpriced_model_is_unknown_not_zero(self):
        trials = [{"config": "A", "success": 1, "llm_calls": 1}]
        usage = [{"config": "A", "provider": "p", "model_id": "unpriced", "input_tokens": 10, "output_tokens": 10, "latency_s": 1, "attempts": 1}]
        text, s = report(self.run_dir(trials, usage), write_pricing([PRICE]))
        self.assertIsNone(s["configs"]["A"]["cost_total"]); self.assertIn("unknown (n/a), not zero", text)
    def test_oracle_runs_are_labelled_synthetic(self):
        text, s = report(self.run_dir([{"config": "A", "success": 1, "llm_calls": 3}], []), write_pricing([PRICE]))
        self.assertFalse(s["real"]); self.assertIn("SYNTHETIC", text); self.assertIsNone(s["configs"]["A"]["cost_total"])
    def test_incremental_cost_only_for_a_gain(self):
        base = {"success_rate": 0.5, "cost_per_attempted": 1.0}
        self.assertEqual(incremental(base, {"success_rate": 0.4, "cost_per_attempted": 2.0})[1], "undefined (no success gain)")
        dpp, inc = incremental(base, {"success_rate": 0.6, "cost_per_attempted": 2.0}); self.assertAlmostEqual(dpp, 10.0); self.assertAlmostEqual(inc, 0.1)
        self.assertEqual(incremental(base, {"success_rate": 0.6, "cost_per_attempted": None})[1], "cost unknown")

class SavingTests(unittest.TestCase):
    def test_saving_percent(self):
        self.assertAlmostEqual(saving_pct({"c": 2.0}, {"c": 0.5}, "c"), 75.0)
        self.assertAlmostEqual(saving_pct({"c": 1.0}, {"c": 1.5}, "c"), -50.0)
        self.assertIsNone(saving_pct({"c": None}, {"c": 1.0}, "c")); self.assertIsNone(saving_pct({"c": 0.0}, {"c": 1.0}, "c"))

class PromptTests(unittest.TestCase):
    def test_v1_prompt_drives_the_manager(self):
        seen = []
        def fn(p, tag):
            seen.append((tag, p))
            if tag.startswith("assess"): return '{"done": true}'
            return json.dumps({"schema": "orbit.plan/v1", "status": "ok", "steps": [
                {"id": 1, "tool": "move_forward", "args": {"distance": 1}, "expect": {"pose": {"x": 5, "y": 3}}},
                {"id": 2, "tool": "turn", "args": {"direction": "right", "degrees": 90}}], "completion": {"pose": {"x": 5, "y": 3, "yaw_deg": 270}}})
        w = World2D(9, 7, (4, 3), 0, [])
        t = Task("unit|m", "motion", [Phase("Move forward 1 meter, then turn right.", "motion", {})], {}, {"final_pose": (5, 3, 270)})
        row = run_episode(w, t, PRESETS["PEL"], PelV1Backend(LLMBackend(ScriptedClient(fn), False, False)))
        self.assertEqual((row["success"], row["pel_plan_errors"], row["pel_verify_failures"]), (1, 0, 0))
        self.assertIn('"schema":"orbit.plan/v1"', seen[0][1]); self.assertIn(PEL_V1_FORMAT, seen[0][1])
    def test_existing_prompts_are_untouched(self):
        from orbit_ros import backends2d
        with open(backends2d.__file__, encoding="utf-8") as f: self.assertNotIn("orbit.plan/v1", f.read())

if __name__ == "__main__": unittest.main()
