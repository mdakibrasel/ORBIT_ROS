"""v0.9.0 seed discipline: held-out blocks are disjoint from each other, from dev/calibration seeds and from seeds already used."""
import json, os, unittest, itertools

REG = os.path.join(os.path.dirname(__file__), "..", "configs", "v0_9_seed_registry.json")

class TestSeedRegistry(unittest.TestCase):
    def setUp(self):
        with open(REG) as f: self.r = json.load(f)

    def test_blocks_have_no_duplicates_inside(self):
        for name, seeds in list(self.r["heldout"].items()) + list(self.r["used"].items()) + [("dev", self.r["dev"]), ("calibration", self.r["calibration"])]:
            self.assertEqual(len(seeds), len(set(seeds)), name)

    def test_heldout_blocks_are_pairwise_disjoint(self):
        for (a, sa), (b, sb) in itertools.combinations(self.r["heldout"].items(), 2):
            self.assertFalse(set(sa) & set(sb), f"{a} overlaps {b}")

    def test_heldout_disjoint_from_dev_calibration_and_used(self):
        other = set(self.r["dev"]) | set(self.r["calibration"])
        for s in self.r["used"].values(): other |= set(s)
        for name, seeds in self.r["heldout"].items():
            self.assertFalse(set(seeds) & other, f"{name} overlaps dev/calibration/used seeds")

    def test_dev_and_calibration_are_disjoint(self):
        self.assertFalse(set(self.r["dev"]) & set(self.r["calibration"]))

    def test_registry_matches_v081_config(self):
        with open(os.path.join(os.path.dirname(REG), "v0_8_1.json")) as f: c = json.load(f)
        self.assertEqual(c["dev_seeds"], self.r["dev"])
        self.assertEqual(c["test_seeds"], self.r["used"]["v081_confirmatory"])

if __name__ == "__main__":
    unittest.main()
