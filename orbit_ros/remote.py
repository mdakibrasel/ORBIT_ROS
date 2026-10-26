"""Legacy JSON-line session protocol (RETIRED as a live transport in v0.16.5).

Historically orbit_bridge.py held the single Webots extern controller and re-shared it over TCP port 1240 so short-lived clients
could reuse one connection. As of v0.16.5 the MCP server connects to Webots DIRECTLY on the controller endpoint 127.0.0.1:1235 and
stays alive for the whole client session (standard MCP stdio lifetime), so no separate relay and no port 1240 are used anywhere.

This module is kept only for the serialization protocol it defines and the unit tests that exercise it on an ephemeral port;
PORT is 0 (disabled) so nothing binds or dials the old 1240. Request {"m": method, "a": [args], "k": {kwargs}};
reply {"ok": result} or {"err": message, "type": exception class}."""
import json, socket

HOST, PORT = "127.0.0.1", 0          # 0 = disabled: the retired bridge no longer uses a fixed port (was 1240 before v0.16.5)
METHODS = ("robot_tools", "robot_call", "load_scene", "house_info", "status", "believed_map", "set_no_go_zone", "capabilities",
           "clean_status", "metrics", "export_csv", "get_attr", "recall_memory", "debug_view", "search_github")
ERRORS = {"ValueError": ValueError, "TypeError": TypeError, "KeyError": KeyError}

def bridge_up(host=HOST, port=PORT, timeout=0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout): return True
    except OSError: return False

class RemoteSession:
    """Same calls as HumanoidSession, executed by the bridge process that holds Webots."""
    def __init__(self, host=HOST, port=PORT, timeout=3600):      # a multi-target plan in Webots can run for tens of minutes
        self.host, self.port, self.timeout = host, port, timeout
        self.sock = self.f = None
        self._connect()
        self.sim = "webots (bridge)"
        self.log_path = self.get_attr("log_path")
        self.robot_name = self.get_attr("robot_name")

    def _connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.f = self.sock.makefile("rwb", buffering=0)

    def _call(self, m, *a, **k):
        if self.sock is None: self._connect()
        try:
            self.f.write((json.dumps({"m": m, "a": list(a), "k": k}) + "\n").encode())
            line = self.f.readline()
            if not line: raise ConnectionError("the Webots bridge closed the connection")
        except (OSError, ValueError) as e:
            self.close()
            raise RuntimeError("the Webots bridge connection was lost; this request was not retried, and the next call will reconnect") from e
        r = json.loads(line)
        if "err" in r: raise ERRORS.get(r.get("type"), RuntimeError)(r["err"])
        return r["ok"]

    def get_attr(self, name): return self._call("get_attr", name)
    def close(self):
        if self.sock is not None:
            try: self.sock.close()
            except OSError: pass
            self.sock = self.f = None

    def __getattr__(self, name):
        if name in METHODS: return lambda *a, **k: self._call(name, *a, **k)
        raise AttributeError(name)
