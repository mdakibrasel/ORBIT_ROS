"""Tests for the search_github session tool (orbit_ros/humanoid.py, exposed in orbit_ros/mcp_server.py).

No live network: urllib.request.urlopen is replaced by a fake. The tool is read-only, must never move the
robot and must never raise, returning {"status": "error", ...} on any failure instead.
"""
import io, json, tempfile, unittest, urllib.error
from unittest import mock

from orbit_ros.humanoid import HumanoidSession, FakeNao
from orbit_ros.mcp_server import HumanoidMcpServer

SAMPLE = {
    "total_count": 2,
    "items": [
        {"full_name": "nao/robot", "description": "a NAO humanoid", "html_url": "https://github.com/nao/robot",
         "stargazers_count": 123, "forks_count": 4, "language": "Python", "updated_at": "2026-01-02T00:00:00Z"},
        {"full_name": "foo/bar", "description": None, "html_url": "https://github.com/foo/bar",
         "stargazers_count": 7, "forks_count": 0, "language": None, "updated_at": "2026-01-01T00:00:00Z"},
    ],
}


def session(log_dir=False):
    s = HumanoidSession(sim="fake", log_dir=log_dir, memory=False, backend=FakeNao())
    s.load_scene("living_room")
    return s


class _FakeResp(io.BytesIO):
    """A minimal context-manager stand-in for urllib's response object."""
    def __enter__(self): return self
    def __exit__(self, *a): self.close(); return False


def fake_urlopen(body, capture=None):
    def _open(req, timeout=None):
        if capture is not None:
            capture["url"] = req.full_url
            capture["headers"] = dict(req.headers)
        return _FakeResp(json.dumps(body).encode("utf-8"))
    return _open


class TestSearchGithub(unittest.TestCase):
    def test_ok_parses_repositories(self):
        cap = {}
        with mock.patch("urllib.request.urlopen", fake_urlopen(SAMPLE, cap)):
            out = session().search_github("nao humanoid", limit=5, sort="stars")
        self.assertEqual(out["status"], "ok")
        self.assertEqual(out["count"], 2)
        self.assertEqual(out["total_count"], 2)
        self.assertEqual(out["repositories"][0]["full_name"], "nao/robot")
        self.assertEqual(out["repositories"][0]["stars"], 123)
        self.assertIsNone(out["repositories"][1]["description"])
        # query terms and the stars sort must have been sent to GitHub's search endpoint
        self.assertIn("api.github.com/search/repositories", cap["url"])
        self.assertIn("sort=stars", cap["url"])
        self.assertIn("nao", cap["url"])

    def test_best_sort_sends_no_sort_param(self):
        cap = {}
        with mock.patch("urllib.request.urlopen", fake_urlopen(SAMPLE, cap)):
            session().search_github("python", sort="best")
        self.assertNotIn("sort=", cap["url"])

    def test_limit_is_honoured(self):
        with mock.patch("urllib.request.urlopen", fake_urlopen(SAMPLE)):
            out = session().search_github("x", limit=1)
        self.assertEqual(out["count"], 1)
        self.assertEqual(len(out["repositories"]), 1)

    def test_empty_query_is_rejected_without_network(self):
        with mock.patch("urllib.request.urlopen", side_effect=AssertionError("must not call the network")):
            out = session().search_github("   ")
        self.assertEqual(out["status"], "error")
        self.assertIn("query", out["message"])

    def test_bad_limit_and_sort_rejected(self):
        s = session()
        self.assertEqual(s.search_github("x", limit=0)["status"], "error")
        self.assertEqual(s.search_github("x", limit=99)["status"], "error")
        self.assertEqual(s.search_github("x", sort="nonsense")["status"], "error")

    def test_http_error_is_returned_not_raised(self):
        err = urllib.error.HTTPError("u", 403, "rate limited", {}, io.BytesIO(b'{"message": "API rate limit exceeded"}'))
        with mock.patch("urllib.request.urlopen", side_effect=err):
            out = session().search_github("x")
        err.close()
        self.assertEqual(out["status"], "error")
        self.assertIn("403", out["message"])

    def test_network_down_is_returned_not_raised(self):
        with mock.patch("urllib.request.urlopen", side_effect=urllib.error.URLError("no route to host")):
            out = session().search_github("x")
        self.assertEqual(out["status"], "error")
        self.assertIn("GitHub", out["message"])

    def test_does_not_move_the_robot(self):
        s = session()
        before = s._pose()
        with mock.patch("urllib.request.urlopen", fake_urlopen(SAMPLE)):
            s.search_github("x")
        self.assertEqual(s._pose(), before)

    def test_logs_a_search_event(self):
        with tempfile.TemporaryDirectory() as d:
            s = session(d)
            with mock.patch("urllib.request.urlopen", fake_urlopen(SAMPLE)):
                s.search_github("nao", limit=3, sort="forks")
            with open(s.log_path, encoding="utf-8") as f:
                events = [json.loads(l) for l in f]
        gh = [e for e in events if e.get("event") == "search_github"]
        self.assertTrue(gh)
        self.assertEqual(gh[-1]["query"], "nao")
        self.assertEqual(gh[-1]["sort"], "forks")

    def test_mcp_tool_listed_and_callable(self):
        srv = HumanoidMcpServer(session(), "t")
        defs = {t["name"]: t for t in srv.tool_defs()}
        self.assertIn("search_github", defs)
        self.assertTrue(defs["search_github"]["annotations"]["readOnlyHint"])
        self.assertTrue(defs["search_github"]["annotations"]["openWorldHint"])
        self.assertEqual(defs["search_github"]["inputSchema"]["required"], ["query"])
        with mock.patch("urllib.request.urlopen", fake_urlopen(SAMPLE)):
            reply = srv.handle({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                                "params": {"name": "search_github", "arguments": {"query": "nao", "limit": 2}}})
        self.assertFalse(reply["result"]["isError"])
        out = json.loads(reply["result"]["content"][0]["text"])
        self.assertEqual(out["status"], "ok")
        self.assertEqual(out["count"], 2)

    def test_mcp_tool_reports_error_as_iserror(self):
        srv = HumanoidMcpServer(session(), "t")
        with mock.patch("urllib.request.urlopen", side_effect=urllib.error.URLError("down")):
            reply = srv.handle({"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                                "params": {"name": "search_github", "arguments": {"query": "nao"}}})
        self.assertTrue(reply["result"]["isError"])
        out = json.loads(reply["result"]["content"][0]["text"])
        self.assertEqual(out["status"], "error")


if __name__ == "__main__":
    unittest.main()
