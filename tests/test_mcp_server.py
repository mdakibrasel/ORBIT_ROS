"""v0.15 MCP server: natural-language control of the ORBIT robot from an AI client (orbit_ros/mcp_bridge.py, orbit_ros/mcp_server.py,
orbit_mcp_server.py). Everything here runs without Webots: World2D and FakeSim (grid and continuous)."""
import asyncio, io, json, os, random, subprocess, sys, unittest
from unittest import mock
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros.mcp_bridge import OrbitMcpSession, PRIMITIVES, scene_list
from orbit_ros.mcp_server import McpServer, PROTOCOL_VERSIONS, robot_tool_def
from orbit_ros.pel.registry import REGISTRY, validate_call
from orbit_ros.remote import RemoteSession
from orbit_ros.world2d import Obj

SERVER = os.path.join(ROOT, "orbit_mcp_server.py")

def session(**k):
    k.setdefault("log_dir", False); return OrbitMcpSession(**k)

class SpyWorld:
    """Wraps world.execute: every call that reaches the robot must be admissible for the registry and the guard AT THAT MOMENT."""
    def __init__(self, test, s):
        self.test, self.s, self.sent = test, s, []
        self.orig = s.world.execute
        s.world.execute = self
    def __call__(self, tool, args):
        s, call = self.s, {"tool": tool, "args": args}
        errs = [i for i in validate_call(call, s._ctx()) if i.severity == "error"]
        self.test.assertEqual(errs, [], f"inadmissible call reached the robot: {call}")
        self.test.assertIsNone(s.guard.check(call, s.world.pose(), s.believed_obstacles(), s.no_go, s.world.start_pose, s.cleaned_known),
                               f"guard-vetoed call reached the robot: {call}")
        self.sent.append(call)
        return self.orig(tool, args)

class TestSession(unittest.TestCase):
    def test_offered_tools_are_registry_robot_tools(self):
        s = session()
        for t in s.robot_tools(): self.assertIn(t, REGISTRY)
        for agent_tool in ("ask", "abstain", "remember", "done", "replan"): self.assertNotIn(agent_tool, s.robot_tools())
        self.assertNotIn("localize", s.robot_tools()); self.assertIn("localize", session(localizer=True).robot_tools())
        self.assertNotIn("goto_cell", session(nav_tools=False).robot_tools())

    def test_registry_rejection_does_not_move(self):
        s = session(); s.load_scene("demo"); p0, n0 = s.world.pose(), s.world.tool_calls
        for args in ({"distance": -1}, {"distance": 13}, {"distance": 2.5}, {"distance": True}, {}, {"distance": "2"}):
            r = s.robot_call("move_forward", args)
            self.assertEqual(r["status"], "rejected", args)
        self.assertEqual(s.robot_call("turn", {"direction": "up", "degrees": 90})["status"], "rejected")
        self.assertEqual(s.robot_call("turn", {"direction": "left", "degrees": 45})["status"], "rejected")
        self.assertEqual((s.world.pose(), s.world.tool_calls), (p0, n0))

    def test_guard_veto_does_not_move(self):
        s = session(); s.load_scene("demo"); n0 = s.world.tool_calls
        r = s.robot_call("move_forward", {"distance": 5})                     # (4,3) facing +x in a 9 m room: would leave at x=9
        self.assertEqual(r["status"], "vetoed"); self.assertIn("leave the room", r["message"])
        self.assertEqual(s.world.tool_calls, n0); self.assertEqual(s.status()["last_veto"], r["message"].split(": ", 1)[1])

    def test_seen_object_is_guarded(self):
        s = session(); s.load_scene("empty"); s.world.objects.append(Obj("x", "red box", 6, 3))
        self.assertEqual(s.robot_call("describe_view", {})["status"], "ok")
        self.assertIn((6, 3), s.believed_obstacles())
        self.assertEqual(s.robot_call("move_forward", {"distance": 2})["status"], "vetoed")
        self.assertEqual(s.robot_call("move_forward", {"distance": 1})["status"], "ok")

    def test_unseen_object_blocks_then_is_remembered(self):
        s = session(); s.load_scene("empty"); s.world.objects.append(Obj("x", "red box", 6, 3))
        r = s.robot_call("move_forward", {"distance": 3})                     # never looked: the simulator blocks it
        self.assertEqual(r["status"], "failed"); self.assertIn((6, 3), s.blocked_known)
        self.assertEqual(s.robot_call("move_forward", {"distance": 1})["status"], "vetoed")

    def test_no_go_cells(self):
        s = session(); s.load_scene("empty")
        s.set_no_go([[6, 3]])
        self.assertEqual(s.robot_call("move_forward", {"distance": 3})["status"], "vetoed")
        self.assertEqual(s.robot_call("goto_cell", {"x": 6, "y": 3})["status"], "vetoed")
        r = s.robot_call("goto_cell", {"x": 8, "y": 3}); self.assertEqual(r["status"], "ok")
        self.assertNotIn((6, 3), s.world.visited)
        with self.assertRaises(ValueError): s.set_no_go([["a"]])
        s.load_scene("empty"); self.assertEqual(s.no_go, set())

    def test_macro_primitives_are_rechecked(self):
        s = session(); s.load_scene("empty")
        seen = []
        orig = s.guard.check
        def check(call, *a, **k):
            seen.append(call["tool"])
            if call["tool"] == "move_forward" and seen.count("move_forward") >= 3: return "test veto"
            return orig(call, *a, **k)
        s.guard.check = check
        r = s.robot_call("goto_cell", {"x": 1, "y": 0})
        self.assertEqual(r["status"], "vetoed"); self.assertIn("test veto", r["message"])
        self.assertGreaterEqual(seen.count("move_forward"), 3)           # the macro itself and its primitives were checked one by one

    def test_stop_latch_and_restart(self):
        s = session(); s.load_scene("demo")
        self.assertEqual(s.robot_call("stop_robot", {})["status"], "ok")
        for t, a in (("move_forward", {"distance": 1}), ("turn", {"direction": "left", "degrees": 90}), ("goto_cell", {"x": 1, "y": 1}), ("clean", {})):
            self.assertEqual(s.robot_call(t, a)["status"], "rejected", t)
        s.load_scene("demo"); self.assertEqual(s.robot_call("move_forward", {"distance": 1})["status"], "ok")

    def test_call_budget(self):
        s = session(max_calls=3); s.load_scene("demo")
        st = [s.robot_call("get_pose", {})["status"] for _ in range(5)]
        self.assertEqual(st, ["ok", "ok", "ok", "rejected", "rejected"])
        s.load_scene("demo"); self.assertEqual(s.robot_call("get_pose", {})["status"], "ok")

    def test_unknown_and_agent_tools_rejected(self):
        s = session(); s.load_scene("demo")
        for t in ("fly", "ask", "abstain", "remember", "done"):
            self.assertEqual(s.robot_call(t, {})["status"], "rejected", t)

    def test_no_bypass_random_calls(self):
        """Property test: 600 random (often malformed) calls; the spy asserts every call reaching the robot was admissible."""
        rng = random.Random(15)
        vals = [-3, -1, 0, 1, 2, 3, 5, 8, 12, 13, 2.5, "2", None, True, [1, 2], 90, 180, 270, 45, "left", "right", "up"]
        for sim, motion in (("world2d", "grid"), ("fake", "grid"), ("fake", "continuous")):
            s = session(sim=sim, motion=motion, max_calls=10_000); s.load_scene("demo")
            spy = SpyWorld(self, s)
            for i in range(200):
                if i % 70 == 69: s.load_scene("demo"); spy = SpyWorld(self, s)
                tool = rng.choice(s.robot_tools() + ["fly"])
                keys = list(REGISTRY[tool].args) if tool in REGISTRY else ["x"]
                args = {k: rng.choice(vals) for k in keys if rng.random() < 0.9}
                if rng.random() < 0.1: args["extra"] = 1
                if tool == "stop_robot" and rng.random() < 0.8: continue
                r = s.robot_call(tool, args)
                self.assertIn(r["status"], ("ok", "rejected", "vetoed", "failed", "error"))
            self.assertGreater(len(spy.sent), 0)
            self.assertEqual(s.world.collisions, 0, f"{sim}/{motion}: guarded robot hit something")

    def test_task_scene_and_check(self):
        s = session(); r = s.load_scene("task:goto", seed=1, yaw=90)
        self.assertTrue(r["instruction"]); self.assertIn("goto", r["task_id"])
        c = s.check_task(); self.assertIn("success", c["result"])
        self.assertIsNone(session().check_task()["task"])
        with self.assertRaises(ValueError): s.load_scene("task:nope")
        with self.assertRaises(ValueError): s.load_scene("mars")
        names = [x["name"] for x in scene_list()]
        self.assertIn("demo", names); self.assertIn("task:goto", names)

    def test_map_status_capabilities(self):
        s = session(); s.robot_call("describe_view", {})
        m = s.believed_map()["map"].splitlines()
        self.assertEqual(len(m), 8); self.assertIn(">", m[6 - 3])          # robot at y=3 facing +x
        st = s.status(); self.assertEqual(st["calls"]["ok"], 1); self.assertFalse(st["stopped"])
        cap = s.capabilities(); self.assertEqual(cap["robot_tools"]["move_forward"]["args"]["distance"]["max"], 12)

    def test_audit_log(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            s = OrbitMcpSession(log_dir=d); s.robot_call("move_forward", {"distance": 9}); s.robot_call("get_pose", {})
            recs = [json.loads(l) for l in open(s.log_path, encoding="utf-8")]
            self.assertEqual([r["event"] for r in recs], ["load_scene", "call", "call"])
            self.assertEqual(recs[1]["status"], "vetoed")

    def test_constructor_checks(self):
        with self.assertRaises(ValueError): OrbitMcpSession(sim="gazebo", log_dir=False)
        with self.assertRaises(ValueError): OrbitMcpSession(sim="world2d", motion="continuous", log_dir=False)

class TestProtocol(unittest.TestCase):
    def rpc(self, srv, method, params=None, mid=1):
        return srv.handle({"jsonrpc": "2.0", "id": mid, "method": method, "params": params or {}})

    def test_initialize_negotiation(self):
        srv = McpServer(session(), "t")
        for v in PROTOCOL_VERSIONS: self.assertEqual(self.rpc(srv, "initialize", {"protocolVersion": v})["result"]["protocolVersion"], v)
        r = self.rpc(srv, "initialize", {"protocolVersion": "1999-01-01"})["result"]
        self.assertEqual(r["protocolVersion"], PROTOCOL_VERSIONS[0]); self.assertIn("tools", r["capabilities"]); self.assertTrue(r["instructions"])

    def test_tools_list_schemas(self):
        srv = McpServer(session(), "t")
        tools = {t["name"]: t for t in self.rpc(srv, "tools/list")["result"]["tools"]}
        self.assertEqual(tools["move_forward"]["inputSchema"]["properties"]["distance"], {"description": REGISTRY["move_forward"].args["distance"].doc,
                                                                                         "type": "integer", "minimum": 0, "maximum": 12})
        self.assertEqual(tools["turn"]["inputSchema"]["properties"]["degrees"]["enum"], [90, 180, 270])
        self.assertTrue(tools["get_pose"]["annotations"]["readOnlyHint"])
        for t in tools.values(): self.assertEqual(t["inputSchema"]["type"], "object")
        for t in PRIMITIVES:
            if t != "localize": self.assertIn(t, tools)

    def test_robot_tools_only_through_robot_call(self):
        s = session(); srv = McpServer(s, "t"); hits = []
        orig = s.robot_call
        s.robot_call = lambda t, a: hits.append(t) or orig(t, a)
        for t in s.robot_tools():
            srv.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": t, "arguments": {}}})
        self.assertEqual(sorted(hits), sorted(s.robot_tools()))

    def test_tools_call_results_and_errors(self):
        srv = McpServer(session(), "t")
        r = self.rpc(srv, "tools/call", {"name": "move_forward", "arguments": {"distance": 2}})["result"]
        self.assertFalse(r["isError"]); self.assertEqual(json.loads(r["content"][0]["text"])["status"], "ok")
        r = self.rpc(srv, "tools/call", {"name": "move_forward", "arguments": {"distance": 99}})["result"]
        self.assertTrue(r["isError"]); self.assertEqual(json.loads(r["content"][0]["text"])["status"], "rejected")
        r = self.rpc(srv, "tools/call", {"name": "load_scene", "arguments": {"scene": "mars"}})["result"]
        self.assertTrue(r["isError"])
        self.assertEqual(self.rpc(srv, "tools/call", {"name": "fly", "arguments": {}})["error"]["code"], -32602)
        self.assertEqual(self.rpc(srv, "tools/call", {"name": "get_pose", "arguments": [1]})["error"]["code"], -32602)
        self.assertEqual(self.rpc(srv, "resources/list")["error"]["code"], -32601)
        self.assertEqual(self.rpc(srv, "ping")["result"], {})
        self.assertIsNone(srv.handle({"jsonrpc": "2.0", "method": "notifications/initialized"}))
        self.assertEqual(srv.handle({"id": 3})["error"]["code"], -32600)

    def test_serve_stream(self):
        srv = McpServer(session(), "t")
        inp = io.BytesIO(b'{"jsonrpc":"2.0","id":1,"method":"ping"}\n\nnot json\n'
                         b'[{"jsonrpc":"2.0","id":2,"method":"ping"},{"jsonrpc":"2.0","method":"notifications/x"}]\n')
        out = io.BytesIO(); srv.serve(inp, out)
        lines = [json.loads(l) for l in out.getvalue().decode().splitlines()]
        self.assertEqual(lines[0]["result"], {}); self.assertEqual(lines[1]["error"]["code"], -32700); self.assertEqual(lines[2][0]["id"], 2)

    def test_webots_missing_is_a_tool_error(self):
        """--sim webots without Webots: the AI client gets a clear tool error and the server keeps running (no crash, no fallback)."""
        from orbit_ros.webots_env import find_webots_home
        try: import controller  # noqa: F401
        except ImportError: pass
        else: self.skipTest("Webots controller module is importable here")
        if find_webots_home(): self.skipTest("a Webots installation exists on this machine")
        srv = McpServer(session(sim="webots"), "t")
        r = self.rpc(srv, "tools/call", {"name": "get_pose", "arguments": {}})["result"]
        self.assertTrue(r["isError"]); self.assertIn("Webots not found", r["content"][0]["text"])
        self.assertEqual(self.rpc(srv, "ping", mid=2)["result"], {})

    def test_tool_def_from_registry(self):
        d = robot_tool_def("cover_region")
        self.assertEqual(d["inputSchema"]["required"], ["side"])
        self.assertEqual(d["inputSchema"]["properties"]["avoid"]["type"], "array")

class TestServerProcess(unittest.TestCase):
    """The real entry point as a subprocess, exactly as an AI client starts it."""
    def test_selftest_world2d_and_fake(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            for extra in (["--robot", "turtlebot"], ["--robot", "turtlebot", "--sim", "fake", "--motion", "continuous"]):
                p = subprocess.run([sys.executable, SERVER, "--selftest", "--log-dir", d] + extra, capture_output=True, text=True, timeout=120, cwd=ROOT)
                self.assertEqual(p.returncode, 0, p.stdout + p.stderr); self.assertIn("SELF-TEST PASSED", p.stdout)

    def test_stdout_is_protocol_only(self):
        """Prints from the server side (startup banner, simulator messages) must go to stderr, never into the protocol stream."""
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            msgs = [{"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18", "capabilities": {}}},
                    {"jsonrpc": "2.0", "method": "notifications/initialized"},
                    {"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "goto_cell", "arguments": {"x": 1, "y": 1}}},
                    {"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": "get_map", "arguments": {}}}]
            code = "import builtins,runpy,sys; sys.argv=[%r,'--sim','fake','--motion','continuous','--log-dir',%r]; " \
                   "print('noise on stdout before serving'); runpy.run_path(%r, run_name='__main__')" % (SERVER, d, SERVER)
            # the noise above is printed BEFORE serve() claims stdout, so it must be the only non-JSON line
            p = subprocess.run([sys.executable, "-c", code], input="\n".join(map(json.dumps, msgs)) + "\n", capture_output=True, text=True, timeout=120, cwd=ROOT)
            lines = p.stdout.splitlines()
            self.assertEqual(lines[0], "noise on stdout before serving")
            replies = [json.loads(l) for l in lines[1:]]
            self.assertEqual([r["id"] for r in replies], [1, 2, 3])
            self.assertIn("[orbit-mcp]", p.stderr)

    def test_official_sdk_client(self):
        """Interoperability with the official MCP Python SDK client (skipped when the `mcp` package is not installed)."""
        try:
            from mcp import ClientSession
            from mcp.client.stdio import stdio_client, StdioServerParameters
        except ImportError:
            self.skipTest("mcp package not installed (pip install mcp) - optional interoperability check")
        import tempfile
        async def run(d):
            p = StdioServerParameters(command=sys.executable, args=[SERVER, "--robot", "turtlebot", "--log-dir", d], cwd=ROOT)
            async with stdio_client(p) as streams:
                async with ClientSession(streams[0], streams[1]) as s:
                    await s.initialize()
                    names = [t.name for t in (await s.list_tools()).tools]
                    ok = await s.call_tool("move_forward", {"distance": 2})
                    bad = await s.call_tool("move_forward", {"distance": 9})
                    return names, ok, bad
        with tempfile.TemporaryDirectory() as d:
            names, ok, bad = asyncio.run(run(d))
        self.assertIn("goto_cell", names)
        self.assertFalse(getattr(ok, "is_error", getattr(ok, "isError", None)))
        self.assertTrue(getattr(bad, "is_error", getattr(bad, "isError", None)))
        self.assertEqual(json.loads(bad.content[0].text)["status"], "vetoed")

class TestClientConfigs(unittest.TestCase):
    def test_health_check_does_not_import_humanoid(self):
        code = ("import sys; sys.argv=['orbit_mcp_server.py','--health-check','--robot','nao','--sim','webots']; "
                "import orbit_mcp_server; orbit_mcp_server.main(); print('humanoid_loaded=' + str('orbit_ros.humanoid' in sys.modules))")
        p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=30, cwd=ROOT)
        self.assertIn("humanoid_loaded=False", p.stdout)

    def test_make_session_connects_to_webots_directly_without_bridge_or_1240(self):
        # v0.16.5: NAO + Webots builds a direct in-process HumanoidSession (extern controller on 1235). No bridge, no port 1240.
        import orbit_mcp_server
        args = type("Args", (), {"robot": "nao", "sim": "webots", "bridge_only": False, "direct": False,
                                 "max_calls": 10, "log_dir": False, "scene": "living_room", "no_memory": True, "memory_db": None})()
        with mock.patch("orbit_ros.humanoid.HumanoidSession", return_value="direct-session") as hs, \
             mock.patch("orbit_ros.nao_memory.NaoMemory"):
            self.assertEqual(orbit_mcp_server.make_session(args), "direct-session")
        hs.assert_called_once()
        self.assertEqual(hs.call_args.kwargs.get("sim"), "webots")

    def test_bridge_only_is_accepted_as_a_deprecated_alias(self):
        # Old MCP configs still pass --bridge-only; it must not require a bridge and must still build the direct session.
        import orbit_mcp_server
        args = type("Args", (), {"robot": "nao", "sim": "webots", "bridge_only": True, "direct": False,
                                 "max_calls": 10, "log_dir": False, "scene": "living_room", "no_memory": True, "memory_db": None})()
        with mock.patch("orbit_ros.humanoid.HumanoidSession", return_value="direct-session") as hs, \
             mock.patch("orbit_ros.nao_memory.NaoMemory"):
            self.assertEqual(orbit_mcp_server.make_session(args), "direct-session")
        hs.assert_called_once()

    def test_print_config(self):
        p = subprocess.run([sys.executable, SERVER, "--print-config"], capture_output=True, text=True, timeout=60, cwd=ROOT)
        cfg = json.loads(p.stdout)["mcpServers"]["orbit-nao-webots"]
        self.assertEqual(cfg["command"], sys.executable)
        # NAO + Webots: --keep-alive is part of the correct client config so stopping the AI client does not drop the
        # single extern controller (Webots would otherwise fall back to "Waiting for connection").
        self.assertEqual(cfg["args"], [SERVER, "--robot", "nao", "--sim", "webots", "--keep-alive"])
        self.assertNotIn("--bridge-only", cfg["args"]); self.assertNotIn("--direct", cfg["args"])

    def test_print_config_rejects_fake_nao_mcp(self):
        p = subprocess.run([sys.executable, SERVER, "--robot", "nao", "--sim", "fake", "--print-config"],
                           capture_output=True, text=True, timeout=30, cwd=ROOT)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("NAO MCP client configurations require live Webots", p.stderr)

    def test_bridge_only_rejects_non_webots_nao_configuration(self):
        p = subprocess.run([sys.executable, SERVER, "--robot", "nao", "--sim", "fake", "--bridge-only"],
                           capture_output=True, text=True, timeout=30, cwd=ROOT)
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("--bridge-only requires --robot nao --sim webots", p.stderr)

    def test_config_files(self):
        vs = json.load(open(os.path.join(ROOT, ".vscode", "mcp.json"), encoding="utf-8"))
        self.assertEqual(set(vs["servers"]), {"orbit-nao-webots"})
        srv = vs["servers"]["orbit-nao-webots"]; self.assertEqual(srv["type"], "stdio")
        self.assertEqual(srv["cwd"], "${workspaceFolder}")
        self.assertEqual(srv["command"], "powershell.exe")
        self.assertIn("${workspaceFolder}/tools/start_orbit_mcp.ps1", srv["args"])
        self.assertNotIn("--bridge-only", srv["args"]); self.assertNotIn("--direct", srv["args"])
        cd = json.load(open(os.path.join(ROOT, "configs", "claude_desktop_config.example.json"), encoding="utf-8"))
        self.assertTrue(cd["mcpServers"]["orbit-nao-webots"]["args"][0].endswith("orbit_mcp_server.py"))
        self.assertIn("--keep-alive", cd["mcpServers"]["orbit-nao-webots"]["args"])
        self.assertNotIn("--bridge-only", cd["mcpServers"]["orbit-nao-webots"]["args"])
        launch = json.load(open(os.path.join(ROOT, ".vscode", "launch.json"), encoding="utf-8"))["configurations"]
        mcp = [c for c in launch if "orbit_mcp_server.py" in c.get("program", "")]
        self.assertEqual(len(mcp), 1)
        self.assertEqual(mcp[0]["cwd"], "${workspaceFolder}")
        self.assertEqual(mcp[0]["python"], "${workspaceFolder}/.venv/Scripts/python.exe")
        self.assertEqual(mcp[0]["preLaunchTask"], "Start Webots (NAO house)")
        self.assertIn("--selftest", mcp[0]["args"])
        self.assertIn("--sim", mcp[0]["args"]); self.assertIn("webots", mcp[0]["args"])
        self.assertNotIn("--bridge-only", mcp[0]["args"])
        tasks = json.load(open(os.path.join(ROOT, ".vscode", "tasks.json"), encoding="utf-8"))["tasks"]
        self.assertIn("Start Webots (NAO house)", {t["label"] for t in tasks})

    def test_no_config_or_script_references_port_1240(self):
        # Mandatory: port 1240 must not appear in any client config, startup script or VS Code task.
        for rel in (os.path.join(".vscode", "mcp.json"), os.path.join(".vscode", "launch.json"),
                    os.path.join(".vscode", "tasks.json"), os.path.join("configs", "claude_desktop_config.example.json"),
                    os.path.join("configs", "gemini_settings.example.json"), "run.ps1"):
            with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
                self.assertNotIn("1240", f.read(), f"{rel} still references retired port 1240")

class TestRemoteSession(unittest.TestCase):
    def test_reconnects_only_on_a_later_call_after_bridge_disconnect(self):
        class File:
            def __init__(self, response=None, error=None): self.response, self.error = response, error
            def write(self, data): pass
            def readline(self):
                if self.error: raise self.error
                return self.response

        class Socket:
            def __init__(self, stream): self.stream, self.closed = stream, False
            def makefile(self, *args, **kwargs): return self.stream
            def close(self): self.closed = True

        first = Socket(File(error=ConnectionResetError("bridge restarted")))
        second = Socket(File(response=b'{"ok":"connected"}\n'))
        remote = object.__new__(RemoteSession)
        remote.host, remote.port, remote.timeout = "127.0.0.1", 9, 1   # mocked socket; any port (never the retired 1240)
        remote.sock, remote.f = first, first.stream
        with mock.patch("orbit_ros.remote.socket.create_connection", return_value=second) as connect:
            with self.assertRaisesRegex(RuntimeError, "request was not retried"):
                remote._call("robot_call", "walk_forward", {"steps": 1})
            self.assertTrue(first.closed)
            self.assertEqual(remote._call("status"), "connected")
        connect.assert_called_once_with(("127.0.0.1", 9), timeout=1)
        remote.close()

if __name__ == "__main__": unittest.main()
