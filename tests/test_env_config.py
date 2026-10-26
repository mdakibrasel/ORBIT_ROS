import os, tempfile, unittest, io, contextlib
from orbit_ros.env_config import parse_env, load_env, provider_status, format_status, warnings

class TestEnv(unittest.TestCase):
    def test_parse(self):
        d = parse_env('A=1\n# c\nB="x # y" # z\nexport C = v # tail\nD=\n  \nE=\'q\'')
        self.assertEqual(d, {"A": "1", "B": "x # y", "C": "v", "D": "", "E": "q"})

    def test_example_file_has_empty_keys_and_loads_nothing_secret(self):
        root = os.path.join(os.path.dirname(__file__), "..")
        with open(os.path.join(root, ".env.example")) as f: d = parse_env(f.read())
        for k, v in d.items():
            if k.endswith("_API_KEY"): self.assertEqual(v, "", k)
        self.assertEqual(load_env(os.path.join(root, ".env.example"), environ={}), [k for k, v in d.items() if v])

    def test_environment_wins_and_empty_placeholders_do_not_override(self):
        with tempfile.TemporaryDirectory() as t:
            p = os.path.join(t, ".env")
            with open(p, "w") as f: f.write("GEMINI_API_KEY=file-key\nGROQ_API_KEY=\nX=1\n")
            env = {"GEMINI_API_KEY": "real-env-key"}
            names = load_env(p, environ=env)
            self.assertEqual(env["GEMINI_API_KEY"], "real-env-key"); self.assertNotIn("GROQ_API_KEY", env); self.assertEqual(names, ["X"])
            self.assertEqual(load_env(p, override=True, environ=env), ["GEMINI_API_KEY", "X"])
            self.assertEqual(load_env(os.path.join(t, "missing.env"), environ=env), [])

    def test_status_never_contains_key_values(self):
        env = {"LLM_PROVIDER_ORDER": "gemini,groq", "GEMINI_API_KEY": "SECRET-ABC", "GEMINI_MODEL": "m1", "GROQ_API_KEY": ""}
        text = format_status(env)
        self.assertNotIn("SECRET-ABC", text); self.assertIn("gemini", text)
        st = {r["provider"]: r for r in provider_status(env)}
        self.assertTrue(st["gemini"]["configured"]); self.assertFalse(st["groq"]["configured"])

    def test_google_key_warning(self):
        self.assertTrue(warnings({"GOOGLE_API_KEY": "a", "GEMINI_API_KEY": "b"}))
        self.assertFalse(warnings({"GEMINI_API_KEY": "b"}))

    def test_gitignore_protects_env(self):
        root = os.path.join(os.path.dirname(__file__), "..")
        with open(os.path.join(root, ".gitignore")) as f: g = f.read().splitlines()
        self.assertIn(".env", g); self.assertIn("!.env.example", g)

if __name__ == "__main__":
    unittest.main()
