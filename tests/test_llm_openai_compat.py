"""OpenAI-compatible client (Groq etc.) against a local fake server: request shape, JSON-mode fallback, retry, usage accounting."""
import json, threading, unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from orbit_ros.llm import OpenAICompatClient, make_client
from orbit_ros.backends2d import LLMBackend
from orbit_ros.tasks2d import make_task
from orbit_ros.agent2d import run_episode, PRESETS

class Fake(BaseHTTPRequestHandler):
    script = []          # list of (status, body dict); popped per request
    seen = []
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0)); body = json.loads(self.rfile.read(n))
        Fake.seen.append({"path": self.path, "auth": self.headers.get("Authorization"), "body": body})
        status, payload = Fake.script.pop(0)
        self.send_response(status); self.send_header("Content-Type", "application/json")
        if status == 429: self.send_header("retry-after", "0")
        self.end_headers(); self.wfile.write(json.dumps(payload).encode())
    def log_message(self, *a): pass

def ok(text, pt=11, ct=7): return (200, {"choices": [{"message": {"content": text}}], "usage": {"prompt_tokens": pt, "completion_tokens": ct}})

class ClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = HTTPServer(("127.0.0.1", 0), Fake); cls.url = f"http://127.0.0.1:{cls.srv.server_port}/v1"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
    @classmethod
    def tearDownClass(cls): cls.srv.shutdown(); cls.srv.server_close()
    def setUp(self): Fake.script, Fake.seen = [], []
    def client(self, **kw): return OpenAICompatClient("groq", "openai/gpt-oss-120b", base_url=self.url, api_key="TEST-KEY", **kw)

    def test_request_shape_and_usage(self):
        Fake.script = [ok('{"actions": []}')]
        c = self.client(); self.assertEqual(c.complete("hello", "plan"), '{"actions": []}')
        r = Fake.seen[0]; self.assertEqual(r["path"], "/v1/chat/completions"); self.assertEqual(r["auth"], "Bearer TEST-KEY")
        self.assertEqual(r["body"]["model"], "openai/gpt-oss-120b"); self.assertEqual(r["body"]["messages"][-1]["content"], "hello")
        self.assertEqual(r["body"]["response_format"], {"type": "json_object"}); self.assertEqual(r["body"]["temperature"], 0.0)
        self.assertEqual((c.usage_events[0]["input_tokens"], c.usage_events[0]["output_tokens"], c.usage_events[0]["tag"]), (11, 7, "plan"))
    def test_falls_back_to_plain_text_when_json_mode_is_rejected(self):
        Fake.script = [(400, {"error": "response_format unsupported"}), ok("{}")]
        self.assertEqual(self.client().complete("x"), "{}"); self.assertNotIn("response_format", Fake.seen[1]["body"])
    def test_retries_rate_limit_then_succeeds(self):
        Fake.script = [(429, {"error": "slow down"}), ok("{}")]
        self.assertEqual(self.client().complete("x"), "{}"); self.assertEqual(len(Fake.seen), 2)
    def test_hard_error_is_raised_and_empty_reply_is_an_error(self):
        Fake.script = [(401, {"error": "bad key"})]
        with self.assertRaises(RuntimeError): self.client().complete("x")
        Fake.script = [(200, {"choices": [{"message": {"content": ""}}]})]
        with self.assertRaises(RuntimeError): self.client().complete("x")
    def test_missing_key_is_reported_without_leaking_anything(self):
        import os
        old = os.environ.pop("GROQ_API_KEY", None)
        try:
            with self.assertRaises(RuntimeError) as cm: OpenAICompatClient("groq", "m", base_url=self.url, api_key="")
            self.assertIn("GROQ_API_KEY", str(cm.exception))
        finally:
            if old is not None: os.environ["GROQ_API_KEY"] = old
    def test_unknown_provider(self):
        with self.assertRaises(ValueError): OpenAICompatClient("nope", "m", api_key="k")
    def test_drives_a_real_episode_through_the_llm_backend(self):
        """An LLM that answers a goto instruction with a correct plan completes it end to end (plumbing, not a model result)."""
        world, task = make_task("motion", 3, 0, 1, False, 1)
        ops = task.phases[0].params["ops"]
        acts = [{"tool": "move_forward", "args": {"distance": ops[0][1]}}, {"tool": "turn", "args": {"direction": "right", "degrees": 90}}]
        Fake.script = [ok(json.dumps({"actions": acts})), ok(json.dumps({"done": True}))]
        be = LLMBackend(self.client(), grounded=False, use_memory=False)
        row = run_episode(world, task, PRESETS["S1"], be); self.assertEqual(row["success"], 1); self.assertEqual(row["llm_calls"], 2)
        self.assertEqual(row["input_tokens"], 22)

if __name__ == "__main__": unittest.main()
