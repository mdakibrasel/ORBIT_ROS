"""RETIRED in v0.16.5: the Webots relay that used TCP port 1240.

Webots drops an extern controller when its Python process ends. This file used to be a never-ending relay that held the single
'orbit_nao' controller and re-shared it over port 1240 so short-lived clients could reuse it. As of v0.16.5 the MCP server connects
to Webots DIRECTLY on the controller endpoint tcp://127.0.0.1:1235 and lives for the whole client session, so the relay (and port 1240)
is gone. This module no longer starts a server; its serve_client/execute helpers remain only for the serialization unit tests.
Use instead:
    .venv\\Scripts\\python.exe orbit_mcp_server.py --robot nao --sim webots --keep-alive
"""
import json, os, sys, threading

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
from orbit_ros.remote import HOST, PORT, METHODS

def serve_client(conn, jobs):
    f = conn.makefile("rwb", buffering=0)
    try:
        for line in f:
            req, done, box = json.loads(line), threading.Event(), {}
            jobs.put((req, box, done)); done.wait()
            f.write((json.dumps(box["r"], default=str) + "\n").encode())
    except (OSError, ValueError): pass
    finally: conn.close()

def execute(session, req):
    m = req["m"]
    try:
        if m == "get_attr": return {"ok": getattr(session, req["a"][0])}
        if m not in METHODS: return {"err": f"method {m!r} is not offered", "type": "ValueError"}
        return {"ok": getattr(session, m)(*req.get("a", []), **req.get("k", {}))}
    except Exception as e: return {"err": str(e), "type": type(e).__name__}

def main(argv=None):
    """Retired: refuse to start a relay so no project process can bind the old port 1240."""
    sys.stderr.write(
        "orbit_bridge.py is RETIRED (v0.16.5). The MCP server now talks to Webots directly on tcp://127.0.0.1:1235 and "
        "does not use any relay or port 1240.\nStart the live NAO server instead:\n"
        "    .venv\\Scripts\\python.exe orbit_mcp_server.py --robot nao --sim webots --keep-alive\n")
    return 1

if __name__ == "__main__": sys.exit(main())
