"""orbit_chat.py against a local fake model server: the model's tool calls go through the REAL MCP server over stdio."""
import io, json, threading, unittest
from contextlib import redirect_stdout
from http.server import BaseHTTPRequestHandler, HTTPServer
import orbit_chat

def tool_call(name, args, i):
    return {"role": "assistant", "content": None, "tool_calls": [{"id": f"c{i}", "type": "function",
            "function": {"name": name, "arguments": json.dumps(args)}}]}

class Fake(BaseHTTPRequestHandler):
    script, seen = [], []
    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        Fake.seen.append({"auth": self.headers.get("Authorization"), "body": body})
        msg = Fake.script.pop(0)
        self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
        self.wfile.write(json.dumps({"choices": [{"message": msg}]}).encode())
    def log_message(self, *a): pass

class ChatTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv = HTTPServer(("127.0.0.1", 0), Fake); cls.url = f"http://127.0.0.1:{cls.srv.server_port}/v1"
        threading.Thread(target=cls.srv.serve_forever, daemon=True).start()
    @classmethod
    def tearDownClass(cls): cls.srv.shutdown(); cls.srv.server_close()

    def test_model_drives_the_real_mcp_server(self):
        Fake.script = [tool_call("load_scene", {"scene": "living_room"}, 1), tool_call("go_to_room", {"room": "kitchen"}, 2),
                       tool_call("walk_forward", {"steps": 9}, 3), {"role": "assistant", "content": "I am in the kitchen."}]
        Fake.seen = []
        model = orbit_chat.ChatModel("groq", "test-model", base_url=self.url, api_key="TEST-KEY", max_retries=0)
        test_server_args = ["--robot", "nao", "--sim", "fake", "--no-memory"]
        mcp = orbit_chat.McpConnection(test_server_args)
        lines = []
        try:
            tools = orbit_chat.openai_tools(mcp.tools)
            messages = [{"role": "system", "content": mcp.instructions}]
            answer = orbit_chat.run_turn(model, mcp, messages, tools, "go to the kitchen", out=lines.append)
        finally: mcp.close()
        self.assertEqual(answer, "I am in the kitchen.")
        # The tool count is authoritative from the registry + session tools, not a magic number:
        # every advertised MCP tool must be exposed to the model one-to-one.
        from orbit_ros.humanoid import HUMANOID_TOOLS
        from orbit_ros.mcp_server import H_SESSION_TOOLS
        expected = len(HUMANOID_TOOLS) + len(H_SESSION_TOOLS)
        self.assertEqual(len(tools), expected)
        self.assertEqual(len(tools), len(mcp.tools))   # no tool dropped when mapped to the OpenAI schema
        self.assertIn("go_to_object", {t["function"]["name"] for t in tools})
        self.assertIn("look_around", {t["function"]["name"] for t in tools})
        self.assertIn("tour_house", {t["function"]["name"] for t in tools})
        self.assertIn("clean_room", {t["function"]["name"] for t in tools})
        self.assertIn("greet", {t["function"]["name"] for t in tools})
        self.assertNotIn("greet_resident", {t["function"]["name"] for t in tools})
        self.assertIn("join_tv_watching", {t["function"]["name"] for t in tools})
        self.assertEqual([l.split(" -> ")[1] for l in lines], ["ok", "ok", "rejected"])   # steps must be 1..6
        self.assertEqual(Fake.seen[0]["auth"], "Bearer TEST-KEY")
        tool_msgs = [m for m in Fake.seen[-1]["body"]["messages"] if m["role"] == "tool"]
        self.assertIn("kitchen", tool_msgs[1]["content"])

    def test_missing_key_is_reported(self):
        import os
        old = os.environ.pop("GROQ_API_KEY", None)
        try:
            with self.assertRaises(RuntimeError) as e: orbit_chat.ChatModel("groq", "m", api_key="")
            self.assertIn("GROQ_API_KEY", str(e.exception))
        finally:
            if old is not None: os.environ["GROQ_API_KEY"] = old

if __name__ == "__main__": unittest.main()
