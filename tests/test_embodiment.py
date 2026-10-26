import unittest
from orbit_ros.embodiment import EmbodimentIdentifier, Hypothesis, make_probe, SafeProbeError, SurpriseMonitor

class EmbodimentTests(unittest.TestCase):
    def test_hypothesis_space_is_explicit_and_interpretable(self):
        ident = EmbodimentIdentifier(mirror=(False, True), swapped=(False, True), move_gains=(1.0,), turn_gains=(1.0,))
        self.assertEqual(len(ident.hypotheses), 4)
        self.assertIn("mirror=1", ident.hypotheses[-1].key)

    def test_information_gain_prefers_discriminating_safe_probe(self):
        ident = EmbodimentIdentifier(mirror=(False, True), swapped=(False, True), move_gains=(1.0,), turn_gains=(1.0,))
        hs = ident.hypotheses
        weak = make_probe("weak", True, {h.key: "same" for h in hs})
        strong = make_probe("mirror_check", True, {h.key: str(h.mirrored_camera) for h in hs})
        unsafe = make_probe("unsafe", False, {h.key: h.key for h in hs})
        chosen = ident.choose_probe([weak, strong, unsafe])
        self.assertEqual(chosen["name"], "mirror_check")
        self.assertAlmostEqual(chosen["expected_information_gain_bits"], 1.0)

    def test_unsafe_probe_cannot_update_belief(self):
        ident = EmbodimentIdentifier(mirror=(False, True), swapped=(False,), move_gains=(1.0,), turn_gains=(1.0,))
        with self.assertRaises(SafeProbeError): ident.update(make_probe("bad", False, {}), "x")

    def test_update_reduces_hypothesis_set_and_card_is_ambiguous_when_needed(self):
        ident = EmbodimentIdentifier(mirror=(False, True), swapped=(False, True), move_gains=(1.0,), turn_gains=(1.0,))
        p = make_probe("mirror", True, {h.key: str(h.mirrored_camera) for h in ident.hypotheses})
        ident.update(p, "True")
        card = ident.card()
        self.assertEqual(card.probes_used, 1)
        self.assertEqual(len(card.hypotheses), 2)
        self.assertIsNone(card.selected)

    def test_surprise_monitor_flags_mismatch_and_resets_on_match(self):
        mon = SurpriseMonitor(consecutive_threshold=2)
        self.assertFalse(mon.observe(1, 1)["trigger"])
        self.assertFalse(mon.observe(1, 2)["trigger"])
        self.assertTrue(mon.observe(1, 3)["trigger"])
        self.assertFalse(mon.observe(2, 2)["trigger"])

if __name__ == "__main__": unittest.main()
