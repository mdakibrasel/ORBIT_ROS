r"""ORBIT ROS MCP server (v0.15, NAO house v0.16): control the ORBIT TurtleBot3 or a NAO humanoid in natural language from an AI client.

The AI client (Claude Desktop, VS Code agent mode, Gemini CLI, ...) starts this script itself; you only put its path in the client's
configuration (README Part G). Then you type, for example, "drive to the blue box but keep away from the red chair".

    python orbit_mcp_server.py                               # World2D (no simulator needed)
    python orbit_mcp_server.py --sim fake --motion continuous # pure-Python stand-in simulator with wheels and lidar
    python orbit_mcp_server.py --robot nao --sim webots --keep-alive  # live Webots: direct extern controller on 127.0.0.1:1235 (no bridge, no port 1240)
    python orbit_mcp_server.py --selftest [--sim ...]         # check it works WITHOUT an AI client (starts the server and talks MCP to it)
    .venv\Scripts\python.exe orbit_mcp_server.py --print-config   # the block to paste into claude_desktop_config.json
    python orbit_mcp_server.py --robot nao                   # NAO in Webots (default): direct extern controller on 127.0.0.1:1235
    python orbit_mcp_server.py --robot nao --sim webots      # NAO in Webots: open webots/worlds/orbit_house.wbt first (start_webots.ps1 -Humanoid)

stdout carries the MCP protocol and nothing else: everything else (Webots messages, warnings) goes to stderr, and every robot call is
written to results_mcp/session_*.jsonl. Every robot call passes the PEL tool registry and safety guard (orbit_ros/mcp_bridge.py).
"""
from __future__ import annotations
import argparse, json, os, sys, time
ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)

def _version():
    try: return open(os.path.join(ROOT, "VERSION"), encoding="utf-8").read().strip()
    except OSError: return "0"

def build_parser():
    from orbit_ros.sim_select import SIMS
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--robot", choices=("turtlebot", "nao"), default="nao", help="nao (default, PEL registry tools) or turtlebot "
                    "(humanoid in the furnished house, v0.16)")
    ap.add_argument("--sim", choices=SIMS, default=None, help="world2d (default for turtlebot), webots (default for nao), "
                    "fake (offline testing), ros2 (turtlebot only, untested)")
    ap.add_argument("--motion", choices=("grid", "continuous"), default="grid")
    ap.add_argument("--perception", choices=("camera", "truth"), default="camera")
    ap.add_argument("--scene", default=None, help="scene loaded at the first robot call: turtlebot demo | empty | task:<family>; "
                    "nao living_room | kitchen | bedroom | bathroom")
    ap.add_argument("--no-nav-tools", action="store_true", help="do not offer goto_cell / cover_region")
    ap.add_argument("--localizer", action="store_true", help="offer the localize tool")
    ap.add_argument("--max-calls", type=int, default=500, help="robot calls allowed per scene (runaway protection)")
    ap.add_argument("--log-dir", default=None, help="audit log folder (default results_mcp/)")
    ap.add_argument("--ros2-config", default=None)
    ap.add_argument("--no-memory", action="store_true", help="NAO: do not store or recall past observations, positions, obstacles and actions")
    ap.add_argument("--memory-db", default=None, help="NAO: SQLite memory file (default results_mcp/nao_memory.db)")
    ap.add_argument("--no-memory-retrieval", action="store_true", help="NAO: keep memory but do not RETRIEVE past episodes/strategies (writes only)")
    ap.add_argument("--no-memory-write", action="store_true", help="NAO: keep memory retrieval but do not WRITE new episodes/strategies after a task")
    ap.add_argument("--memory-max-records", type=int, default=5000, help="NAO: cap on rows per memory table (oldest dropped beyond this)")
    ap.add_argument("--direct", action="store_true", help="NAO + Webots: deprecated no-op; the server always connects to Webots directly on 127.0.0.1:1235")
    ap.add_argument("--bridge-only", action="store_true", help="NAO + Webots: deprecated alias kept for old MCP configs; now connects directly on 127.0.0.1:1235 (no bridge, no port 1240)")
    ap.add_argument("--keep-alive", action="store_true", help="NAO + Webots: stay connected (Webots keeps running) after the client or self-test ends")
    ap.add_argument("--selftest", action="store_true", help="start the server as the AI client would and run a short scripted session")
    ap.add_argument("--print-config", action="store_true", help="print the Claude Desktop / Gemini CLI configuration block for THIS folder and "
                    "THIS Python (run it with the .venv Python), then exit")
    ap.add_argument("--health-check", action="store_true", help="NAO + Webots: quick, non-connecting readiness probe. Reports whether the "
                    "controller port 1235 has a listener (Webots up) WITHOUT taking the controller slot; exit 0 = ready, 2 = not ready.")
    ap.add_argument("--verify-robot", action="store_true", help="with --health-check, take the controller slot briefly and read the NAO pose; "
                    "use only before starting the MCP server")
    return ap

def print_config(a, argv):
    """The mcpServers block with this Python and this folder, correctly escaped, ready to paste into the client's JSON file."""
    extra = [x for x in argv if x != "--print-config"]
    if a.robot == "nao" and a.sim == "fake":
        raise SystemExit("NAO MCP client configurations require live Webots; use --sim webots --bridge-only")
    key = ("orbit-nao" if a.robot == "nao" else "orbit-ros") + ("-webots" if a.sim == "webots" else "")
    if a.robot == "nao" and a.sim == "webots":
        if "--robot" not in extra: extra += ["--robot", "nao"]
        if "--sim" not in extra: extra += ["--sim", "webots"]
        # v0.16.5: the server connects to Webots directly on 127.0.0.1:1235 over stdio; no --bridge-only, no port 1240.
        extra = [x for x in extra if x not in ("--bridge-only", "--direct")]
        # The AI client starts/stops this process; without --keep-alive, every stop drops the single 'orbit_nao' extern
        # controller and Webots falls back to "Waiting for connection". --keep-alive holds the controller across client
        # disconnects, so the house stays connected until Webots closes or the user quits.
        if "--keep-alive" not in extra: extra += ["--keep-alive"]
    block = {"mcpServers": {key:
                            {"command": sys.executable, "args": [os.path.abspath(__file__)] + extra}}}
    print(json.dumps(block, indent=2))
    if "venv" not in sys.executable.lower():
        print("\nNote: this is not a .venv Python. Run it as  .\\.venv\\Scripts\\python.exe orbit_mcp_server.py --print-config", file=sys.stderr)
    return 0

def resolve(a):
    """Fill the robot-dependent defaults of --sim and --scene; refuse combinations that do not exist."""
    if a.robot == "nao":
        a.sim = a.sim or "webots"; a.scene = a.scene or "living_room"
        if a.sim not in ("fake", "webots"): 
            raise SystemExit(f"--robot nao runs with --sim fake or --sim webots, not {a.sim}")
        from orbit_ros import house
        if a.scene not in house.ROOMS: raise SystemExit(f"--robot nao: --scene must be one of {', '.join(house.ROOMS)}")
    else:
        a.sim = a.sim or "world2d"; a.scene = a.scene or "demo"
    if a.bridge_only and (a.robot != "nao" or a.sim != "webots"):
        raise SystemExit("--bridge-only requires --robot nao --sim webots")
    # --bridge-only and --direct are both deprecated and now mean the same thing (direct 1235), so they may coexist.
    return a

def make_session(a):
    if a.robot == "nao":
        if a.sim == "webots":
            # Transport (v0.16.5): the MCP server is itself the single Webots extern controller and connects in-process
            # over the Webots controller endpoint tcp://127.0.0.1:1235. The MCP client (Claude/VS Code/Gemini/chat) keeps this
            # one process alive over stdio for the whole session, so the controller stays connected -- no separate TCP relay
            # process and, deliberately, NO port 1240 anywhere. `--keep-alive` keeps Webots connected after the client leaves.
            if getattr(a, "bridge_only", False):
                print("[orbit-mcp] note: --bridge-only is deprecated; the server now connects to Webots directly on "
                      "127.0.0.1:1235 (no bridge, no port 1240).", file=sys.stderr)
            print("[orbit-mcp] connecting to Webots directly on 127.0.0.1:1235 (extern controller 'orbit_nao')", file=sys.stderr)
        from orbit_ros.humanoid import HumanoidSession
        from orbit_ros.nao_memory import NaoMemory
        mem = False
        if not a.no_memory:
            if a.memory_db:
                mem = NaoMemory(a.memory_db, max_records=a.memory_max_records,
                                retrieval_enabled=not a.no_memory_retrieval, write_after_task=not a.no_memory_write)
            else:
                mem = None   # None = session builds the default results_mcp/nao_memory.db; apply retrieval/write config to it below
        s = HumanoidSession(sim=a.sim, max_calls=a.max_calls, log_dir=a.log_dir, default_scene=a.scene, memory=mem)
        if mem is None and s.memory is not None:                 # the session built the default db: push the config flags onto it
            s.memory.retrieval_enabled = not a.no_memory_retrieval
            s.memory.write_after_task = not a.no_memory_write
            s.memory.max_records = a.memory_max_records
        mdesc = "off" if a.no_memory else (f"on (db={getattr(s.memory,'path','?')}, retrieval={getattr(s.memory,'retrieval_enabled',None)}, "
                 f"write_after_task={getattr(s.memory,'write_after_task',None)}, max_records={getattr(s.memory,'max_records',None)})" if s.memory else "off")
        print(f"[orbit-mcp] memory: {mdesc}", file=sys.stderr)
        return s
    from orbit_ros.mcp_bridge import OrbitMcpSession
    s = OrbitMcpSession(sim=a.sim, motion=a.motion, perception=a.perception, nav_tools=not a.no_nav_tools, localizer=a.localizer,
                        max_calls=a.max_calls, log_dir=a.log_dir, ros2_config=a.ros2_config, default_scene=a.scene)
    return s

def serve(a):
    # Flush any output emitted before the server claims stdout. Otherwise buffered
    # text can be redirected to stderr by the fd swap below on Windows/Python.
    sys.stdout.flush()
    # The protocol owns the real stdout. fd 1 is pointed at stderr so that nothing else (print, Webots' C library) can corrupt it.
    proto_out = os.fdopen(os.dup(1), "wb", buffering=0)
    os.dup2(2, 1); sys.stdout = sys.stderr
    from orbit_ros.mcp_server import McpServer, HumanoidMcpServer
    s = make_session(a)
    print(f"[orbit-mcp] ORBIT ROS {_version()} MCP server on stdio: robot={a.robot} sim={a.sim} motion={a.motion}; audit log {s.log_path}", file=sys.stderr)
    (HumanoidMcpServer if a.robot == "nao" else McpServer)(s, _version()).serve(sys.stdin.buffer, proto_out)
    if a.keep_alive and a.robot == "nao" and a.sim == "webots":
        _keep_webots_alive(s)

def _keep_webots_alive(s):
    """Hold the Webots extern controller after the MCP client (or self-test) disconnects.

    Webots drops an extern controller the instant its Python process exits, so without this the house would fall back to
    'Waiting for connection' every time VS Code / Claude stops the server or a self-test finishes. We keep this one process
    (the single 'orbit_nao' controller) alive and keep stepping the simulation until Webots itself ends or the user presses
    Ctrl+C. This only steps the world; it issues no motion and never bypasses the guard, the stop-latch or the call budget.
    """
    # Build the backend explicitly (do NOT use hasattr, which would hide a connection error and silently skip keep-alive).
    try:
        nao = s.nao
    except Exception as e:                              # Webots not reachable: there is nothing to hold open, so report and return.
        print(f"[orbit-mcp] keep-alive: no live Webots controller to hold ({type(e).__name__}: {e}); exiting.", file=sys.stderr)
        return
    print("[orbit-mcp] client closed; keeping the Webots controller connected on 127.0.0.1:1235 "
          "(Ctrl+C or close Webots to quit)", file=sys.stderr)
    while True:
        try:
            nao.step()
        except KeyboardInterrupt:
            print("[orbit-mcp] keep-alive: Ctrl+C -> releasing the Webots controller.", file=sys.stderr)
            return
        except RuntimeError as e:                       # WebotsNao.step raises this when sup.step() == -1 or Webots reloads.
            print(f"[orbit-mcp] DISCONNECTED: keep-alive lost Webots ({e})", file=sys.stderr)
            if getattr(s, "reconnect_webots", lambda: False)():
                nao = s.nao
                continue
            print("[orbit-mcp] ERROR: keep-alive could not reattach orbit_nao; stopping.", file=sys.stderr)
            return
        except (OSError, ConnectionError) as e:
            print(f"[orbit-mcp] RECONNECTING: transient Webots socket error ({type(e).__name__}: {e})", file=sys.stderr)
            time.sleep(1.0)

# ---------------------------------------------------------------------------------------------------------------- self-test
SCRIPT = [
    ("load_scene", {"scene": "demo"}),
    ("get_pose", {}),
    ("describe_view", {}),
    ("move_forward", {"distance": 2}),
    ("move_forward", {"distance": 9}),                 # vetoed by the safety guard: would leave the room
    ("move_forward", {"distance": -1}),                # rejected by the registry: distance must be 0..12
    ("set_no_go_cells", {"cells": [[4, 5]]}),
    ("goto_cell", {"x": 4, "y": 6}),                   # path through the no-go cell is not taken
    ("get_map", {}),
    ("stop_robot", {}),
    ("move_forward", {"distance": 1}),                 # rejected: the robot is stopped
    ("get_status", {}),
]

EXPECT = {4: "vetoed", 5: "rejected", 10: "rejected"}

NAO_SCRIPT = [
    ("load_scene", {"scene": "living_room"}),
    ("house_info", {}),
    ("describe_view", {}),
    ("get_pose", {}),
    ("greet", {}),
    ("join_tv_watching", {}),
    ("load_scene", {"scene": "living_room"}),
    ("walk_forward", {"steps": 9}),                   # rejected by the argument check: steps must be 1..6
    ("set_no_go_zone", {"x": 4.2, "y": 3.1, "radius": 0.3}),
    ("walk_forward", {"steps": 1}),                   # vetoed before motion: the zone is ahead of the start pose
    ("go_to", {"x": 20, "y": 3}),                     # rejected: outside the house
    ("stop_robot", {}),
    ("wave", {}),                                     # rejected: the robot is stopped
    ("get_status", {}),
]
NAO_EXPECT = {7: "rejected", 9: "vetoed", 10: "rejected", 12: "rejected"}

class _Client:
    def __init__(self, argv):
        import subprocess                                    # deferred: only the self-test spawns a child; the serve path never imports it
        self.p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=sys.stderr, cwd=ROOT)
        self.n = 0
    def request(self, method, params=None):
        self.n += 1
        self.p.stdin.write((json.dumps({"jsonrpc": "2.0", "id": self.n, "method": method, "params": params or {}}) + "\n").encode()); self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line: raise RuntimeError("the server closed stdout (see the error above)")
        msg = json.loads(line)
        if msg.get("id") != self.n: raise RuntimeError(f"unexpected reply {msg}")
        return msg
    def notify(self, method):
        self.p.stdin.write((json.dumps({"jsonrpc": "2.0", "method": method}) + "\n").encode()); self.p.stdin.flush()
    def close(self, wait=True):
        self.p.stdin.close()
        if wait: self.p.wait(timeout=30)

def selftest(a, argv):
    """Run the self-test; report a server that died early as a clear failure, not a traceback."""
    holder = {}
    try:
        return _selftest(a, argv, holder)
    except (RuntimeError, OSError, ValueError) as e:
        print(f"\nSELF-TEST FAILED: {e}", file=sys.stderr)
        if a.robot == "nao" and a.sim == "webots":
            print("The NAO server needs the live Webots house. Run `.\\run.ps1 -Mode server` (keep its window open) "
                  "and stop any other orbit-nao-webots / benchmark process, then try again.", file=sys.stderr)
        c = holder.get("client")
        if c is not None and c.p.poll() is None:   # reap a server that is still running
            try: c.p.stdin.close()
            except OSError: pass
            try: c.p.wait(timeout=5)
            except Exception: c.p.kill()
        return 1

def _selftest(a, argv, holder):
    child = [sys.executable, os.path.abspath(__file__)] + [x for x in argv if x != "--selftest"]
    print(f"Starting the server the way an AI client does:\n  {' '.join(child)}\n")
    c, ok = _Client(child), True
    holder["client"] = c
    t0 = time.time()
    init = c.request("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "orbit-selftest", "version": "1"}})
    info = init["result"]["serverInfo"]
    print(f"initialize: {info['name']} {info['version']}, protocol {init['result']['protocolVersion']}")
    c.notify("notifications/initialized")
    tools = [t["name"] for t in c.request("tools/list")["result"]["tools"]]
    print(f"tools/list: {len(tools)} tools: {', '.join(tools)}\n")
    script, expect = (NAO_SCRIPT, NAO_EXPECT) if a.robot == "nao" else (SCRIPT, EXPECT)
    for i, (name, args) in enumerate(script):
        r = c.request("tools/call", {"name": name, "arguments": args})["result"]
        payload = json.loads(r["content"][0]["text"])
        st = payload.get("status", "error" if r["isError"] else "ok") if isinstance(payload, dict) else "ok"
        want = expect.get(i, "ok")
        mark = "OK " if st == want else "BAD"; ok &= st == want
        msg = payload.get("message", "") if isinstance(payload, dict) else ""
        if len(msg) > 160: msg = msg[:157] + "..."
        print(f"{mark} {name} {json.dumps(args)} -> {st}{': ' + msg if msg else ''}")
        if name == "get_map": print("\n".join("      " + l for l in payload["map"].splitlines()))
    if a.robot == "nao":
        r = c.request("tools/call", {"name": "load_scene", "arguments": {"scene": "living_room"}})["result"]
        payload = json.loads(r["content"][0]["text"])
        st = payload.get("status", "error" if r["isError"] else "ok") if isinstance(payload, dict) else "ok"
        ok &= st == "ok"
        print(f"{'OK ' if st == 'ok' else 'BAD'} cleanup load_scene {{\"scene\": \"living_room\"}} -> {st}")
    c.close(wait=not a.keep_alive)
    what = f"robot {a.robot}, simulator {a.sim}" + ("" if a.robot == "nao" else f", motion {a.motion}")
    print(f"\n{'SELF-TEST PASSED' if ok else 'SELF-TEST FAILED'} ({time.time() - t0:.1f} s, {what})")
    return 0 if ok else 1

def health_check(a):
    """Fast, non-connecting readiness probe for NAO + Webots.

    It does NOT build the extern controller (that would take the single controller slot away from the running server/AI client);
    it only checks whether Webots is listening on the controller port 1235. Prints one line and returns an exit code:
    0 = ready (port open), 2 = not ready. For --sim fake there is no port, so it reports the in-process backend as ready.
    """
    import socket
    if a.robot == "nao" and a.sim == "webots":
        host, port = "127.0.0.1", 1235
        try:
            with socket.create_connection((host, port), timeout=1.0):
                url = "tcp://127.0.0.1:1235/orbit_nao"
                if getattr(a, "verify_robot", False):
                    from orbit_ros.humanoid import HumanoidSession
                    session = HumanoidSession(sim="webots", log_dir=False, memory=False, default_scene=a.scene)
                    pose = session.nao.pose()
                    print(f"[health] READY: Webots and NAO controller responded at {url}; pose=({pose[0]:.3f}, {pose[1]:.3f}, "
                          f"{pose[2]:.1f} deg, fallen={bool(pose[3])})")
                    return 0
                print(f"[health] READY: Webots controller port {port} is listening ({url}); the MCP server can own the "
                      f"'orbit_nao' extern controller.")
                return 0
        except OSError:
            print(f"[health] NOT READY: nothing is listening on {host}:{port}. Start Webots with .\\start_webots.ps1 "
                  f"(the house must be open before the MCP server connects).", file=sys.stderr)
            return 2
    print(f"[health] READY: robot={a.robot} sim={a.sim} uses an in-process backend (no controller port).")
    return 0

def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    a = build_parser().parse_args(argv)
    # A readiness probe must stay cheap and non-invasive. In particular, do not import the
    # house/tool registry just to validate the default NAO scene.
    if a.health_check:
        if a.robot == "nao":
            a.sim = a.sim or "webots"
            a.scene = a.scene or "living_room"
        else:
            a.sim = a.sim or "world2d"
            a.scene = a.scene or "demo"
        return health_check(a)
    a = resolve(a)
    if a.print_config: return print_config(a, argv)
    if a.health_check: return health_check(a)
    if a.selftest: return selftest(a, argv)
    serve(a); return 0

if __name__ == "__main__": sys.exit(main())
