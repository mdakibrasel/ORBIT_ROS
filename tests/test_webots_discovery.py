"""v0.14.2: `--sim webots` finds Webots by itself (sim_select.ensure_webots_controller). No Webots is installed here: a stand-in Webots
folder with a stub `controller` module checks that the module is found and loaded; nothing here shows that the real Webots works."""
import json, os, sys, tempfile, unittest
from unittest import mock
from orbit_ros import sim_select

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class WebotsDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.path0, self.mods0 = list(sys.path), dict(sys.modules)
        sys.modules.pop("controller", None)
    def tearDown(self):
        sys.path[:] = self.path0
        for k in set(sys.modules) - set(self.mods0): sys.modules.pop(k, None)

    def fake_webots(self):
        home = tempfile.mkdtemp(); py = os.path.join(home, "lib", "controller", "python"); os.makedirs(py)
        with open(os.path.join(py, "controller.py"), "w") as f: f.write("class Supervisor:\n    STUB = True\n")
        return home

    def test_finds_webots_from_webots_home(self):
        home = self.fake_webots()
        with mock.patch.dict(os.environ, {"WEBOTS_HOME": home}), mock.patch("orbit_ros.env_config.load_env", lambda *a, **k: []):
            self.assertEqual(sim_select.ensure_webots_controller(), home)
            import controller
            self.assertTrue(controller.Supervisor.STUB)

    def test_nothing_to_do_inside_webots(self):
        sys.modules["controller"] = type(sys)("controller")
        self.assertIsNone(sim_select.ensure_webots_controller())

    def test_clear_error_without_webots(self):
        with mock.patch("orbit_ros.webots_env.find_webots_home", lambda env=None: None), \
             mock.patch("orbit_ros.env_config.load_env", lambda *a, **k: []):
            with self.assertRaises(RuntimeError) as cm: sim_select.ensure_webots_controller()
        self.assertIn("WEBOTS_HOME", str(cm.exception))

    def test_finds_webots_installed_anywhere(self):
        """v0.16.2: without WEBOTS_HOME, Webots is found from the list of installed programs, the PATH, or a folder on another drive."""
        from orbit_ros import webots_env
        none = dict(defaults=lambda env: [], registry=lambda: [], which=lambda name: None, drives=lambda env: [])   # the real Webots of this PC is not looked at
        self.assertIsNone(webots_env.find_webots_home({}, **none))
        home = self.fake_webots()
        self.assertEqual(webots_env.find_webots_home({}, **dict(none, registry=lambda: [home])), home)
        self.assertEqual(webots_env.find_webots_home({}, **dict(none, drives=lambda env: ["/nowhere", home])), home)
        exe = os.path.join(home, "msys64", "mingw64", "bin", "webots.exe"); os.makedirs(os.path.dirname(exe)); open(exe, "w").close()
        self.assertEqual(webots_env.find_webots_home({}, **dict(none, which=lambda name: exe)), home)
        self.assertEqual(webots_env.find_webots_home({"WEBOTS_HOME": home}, **none), home)

    def test_powershell_scripts_use_the_shared_finder(self):
        for f in ("setup.ps1", "start_webots.ps1"):
            with open(os.path.join(ROOT, f), encoding="utf-8") as fh: self.assertIn("Find-WebotsExe $PSScriptRoot", fh.read(), f)
        with open(os.path.join(ROOT, "tools", "find_webots.ps1"), encoding="utf-8") as fh: src = fh.read()
        for needle in ("Uninstall", "Get-Command", "Get-PSDrive", "WEBOTS_HOME"): self.assertIn(needle, src)

    def test_vscode_has_no_turtlebot_webots_entries(self):
        with open(os.path.join(ROOT, ".vscode", "launch.json"), encoding="utf-8") as f: confs = json.load(f)["configurations"]
        for c in confs: self.assertNotIn("run_extern.py", c.get("program", ""), c["name"])
        for c in confs:
            if "program" in c: self.assertTrue(os.path.isfile(c["program"].replace("${workspaceFolder}", ROOT)), c["name"])

if __name__ == "__main__": unittest.main()
