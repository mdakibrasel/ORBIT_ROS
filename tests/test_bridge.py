import os, queue, socket, sys, threading, unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import io, json
from unittest import mock
import orbit_bridge
from orbit_ros.humanoid import HumanoidSession
from orbit_ros.remote import RemoteSession, METHODS


class _FakeResp(io.BytesIO):
    def __enter__(self): return self
    def __exit__(self, *a): self.close(); return False


class TestBridge(unittest.TestCase):
    def _serve(self, session):
        jobs = queue.Queue()
        srv = socket.socket(); srv.bind(("127.0.0.1", 0)); srv.listen(2); port = srv.getsockname()[1]
        def accept():
            while True:
                try: conn, _ = srv.accept()
                except OSError: return
                threading.Thread(target=orbit_bridge.serve_client, args=(conn, jobs), daemon=True).start()
        def work():
            while True:
                req, box, done = jobs.get(); box["r"] = orbit_bridge.execute(session, req); done.set()
        for f in (accept, work): threading.Thread(target=f, daemon=True).start()
        return srv, port

    def test_remote_session_matches_local(self):
        session = HumanoidSession(sim="fake", log_dir=False)
        srv, port = self._serve(session)
        try:
            r = RemoteSession(port=port)
            self.assertEqual(r.robot_tools(), session.robot_tools())
            self.assertEqual(r.load_scene("kitchen")["scene"], "kitchen")
            self.assertEqual(r.robot_call("get_pose")["status"], "ok")
            with self.assertRaises(ValueError): r.load_scene("garage")
            r.close()
            r2 = RemoteSession(port=port)                       # a second client sees the same robot
            self.assertEqual(r2.status()["pose"]["room"], "kitchen"); r2.close()
        finally: srv.close()

    def test_search_github_is_offered_over_the_bridge(self):
        # Regression: a tool that the MCP server calls on the session must be in METHODS, or it fails only in live Webots.
        self.assertIn("search_github", METHODS)
        session = HumanoidSession(sim="fake", log_dir=False, memory=False)
        srv, port = self._serve(session)
        body = {"total_count": 1, "items": [{"full_name": "a/b", "description": "d", "html_url": "u",
                                             "stargazers_count": 3, "forks_count": 1, "language": "Python", "updated_at": "z"}]}
        def _open(req, timeout=None): return _FakeResp(json.dumps(body).encode())
        try:
            r = RemoteSession(port=port)
            with mock.patch("urllib.request.urlopen", _open):
                out = r.search_github("nao", limit=2, sort="stars")     # travels over TCP to the bridge and back
            self.assertEqual(out["status"], "ok")
            self.assertEqual(out["repositories"][0]["full_name"], "a/b")
            r.close()
        finally: srv.close()


if __name__ == "__main__": unittest.main()
