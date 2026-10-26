# ORBIT-ROS: start here (Windows, VS Code, Webots)

1. Open this folder in VS Code (`File > Open Folder`). Install the recommended extensions when VS Code asks.
2. In the VS Code terminal run **once**:
   `powershell -ExecutionPolicy Bypass -File .\setup.ps1`
   It creates `.venv`, installs the packages, runs the test suite (**623 pass, 2 skip** as of 2026-10-25), creates your private
   **`.env`**, and prepares Webots.
3. **API keys:** open `.env` (`code .env`) and paste your keys after the `=` signs, for example `GEMINI_API_KEY=...`. Save. Never share this file;
   `.gitignore` keeps it out of Git. Check it with `python check_env.py` (shows set/missing, never the key).
   The Webots and Oracle runs need **no key**; keys are only for live-LLM runs.
4. **Open the NAO house (persistent controller):**
   `powershell -ExecutionPolicy Bypass -File .\run.ps1 -Mode server`
   It opens `webots\worlds\orbit_house.wbt` and waits. The MCP server itself is the single Webots NAO extern controller and connects
   **directly** on `tcp://127.0.0.1:1235` — there is **no** relay/bridge process (`orbit_bridge.py` is retired as of v0.16.5) and no port 1240.
   Keep Webots open, then start `orbit-nao-webots` from `.vscode\mcp.json` (README section 4).
5. **Talk to and test the live NAO (MCP):** with Webots open, start `orbit-nao-webots` from `.vscode\mcp.json`. To verify the live
   connection and robot without an AI app, run:
   `.\.venv\Scripts\python.exe orbit_mcp_server.py --selftest --robot nao --sim webots`
   It must end with **`SELF-TEST PASSED`** (verified live 2026-10-25: 49 tools, safety guard + stop-latch exercised).
   The retired TurtleBot Webots controller is not used.

### Verify / troubleshoot (verified 2026-10-25)
* **Start Webots:** `.\start_webots.ps1` (finds `webots.exe` via `WEBOTS_HOME`/PATH/default folders; refuses a 2nd instance; opens the
  house on controller port **1235**). If "webots.exe not found", set `WEBOTS_HOME=<your Webots folder>` in `.env`.
* **Consecutive natural-navigation / Home-Base / no-teleport live check:** with Webots open,
  `.\.venv\Scripts\python.exe tools/live/consecutive_live.py` (one persistent connection drives go-to-room sequences,
  `return_to_start`, and a `run_plan ... finish:start`; writes `results_live/consecutive_<ts>/trace.jsonl`). Verified: connection
  persisted across 7 commands, pose preserved between commands, Home/Base returned only when requested, 0 teleports/resets.
* **Code checks (no Webots):** `.\.venv\Scripts\python.exe -m pytest -m "not live" -q` → 623 passed, 2 skipped.
* **One extern controller at a time:** run either the MCP server/chat OR the standalone benchmark, not both (both own the NAO on 1235).

**Real models (v0.10):** after step 3, run `python preflight_llm.py --planner groq --model openai/gpt-oss-120b` and send me the output.
Then follow `docs/REAL_MODEL_RUNBOOK.md` (pilot with `--limit 20`, then the main run). VS Code: F5 entries 8-10.
