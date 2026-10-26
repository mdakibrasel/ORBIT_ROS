# ORBIT ROS v0.16.5: a NAO humanoid that cleans a furnished house, driven by natural language over MCP

A NAO humanoid in a furnished 10 x 8 m house (`webots/worlds/orbit_house.wbt`, Webots R2025a). You give commands in plain words
through an AI app (MCP), for example *"go to the kitchen, the washroom and the bedroom and clean"*. Every motion is checked by a safety
guard before the robot moves. The project includes a real-world-style benchmark (section 6). The Webots TurtleBot3 world was removed:
the only Webots world is the NAO house.

```
.\setup.ps1              # one-time install and self-test
.\start_webots.ps1       # opens the NAO house (refuses a second Webots instance)
```
Then start `orbit-nao-webots` in VS Code (`.vscode\mcp.json`) and talk to the NAO. The VS Code MCP server launches
`orbit_mcp_server.py --robot nao --sim webots --keep-alive`: that one process **is** the single Webots extern controller and connects
directly on `tcp://127.0.0.1:1235`. `--keep-alive` holds the controller connected even when the AI client stops or a self-test ends
(see "Keeping the NAO controller connected" below). The VS Code MCP server and NAO self-test use the live Webots house; fake simulation
is reserved for internal unit tests.

> **v0.16.5 transport (no relay, no port 1240):** the old TCP relay `orbit_bridge.py` on port 1240 is **retired**. The MCP server now
> connects to Webots directly on `tcp://127.0.0.1:1235` and lives for the whole client session. Wherever older text below says "bridge"
> or "keep the bridge window open", it means this single direct controller process; there is no separate relay process.

**v0.16.3 foundation:** guarded whole-house cleaning with pause/resume and return-to-start,
a custom `.motion` player (Webots' `Motion.isOver()` ended walking motions after 0.26 s here), automatic recovery from falls,
`start_webots.ps1` opening the NAO house by default, and `tools/nao_benchmark.py`. Missing Webots assets are copied from the R2025a
sources when the local installation is incomplete.

**The house:** 2.4 m walls with windows, a ceiling light in every room, parquet, tile and carpet floors. Living room: sofa, armchair,
coffee table with a TV remote, television, bookshelf, floor lamp and plants. NAO can use `join_tv_watching` to take a clear spot beside the sofa and face the screen. Kitchen: fridge, counter with sink, stove, dining table and chairs, fruit bowl,
clock. Bedroom: bed, wardrobe, desk with laptop and chair. Bathroom: bathtub with a rubber duck, toilet, washbasin, washing machine.

Only one result file is shipped, `results_spatial_oracle/trial_results.csv`: a unit test compares against it
(regenerate it only on purpose with `python tools/make_oracle_baseline.py`).

**Tests (code checks, no Webots):** `python -m pytest -q -m "not live"` ran **624 passed, 1 skipped** here in about 6m42s on
2026-10-26 (`.venv` Python 3.14). Webots, Claude Desktop and live API keys are not part of these code checks; live Webots behaviour is
covered by `@pytest.mark.live` tests and the NAO benchmark, which must be run on your computer with Webots open.

## 1. Install (Windows, one time)

1. Unzip `ORBIT_ROS_v0.16.5_RUN.zip` to a short path, for example `C:\orbit_ros`. You get the folder `C:\orbit_ros\orbit_ros_v0165_run`.
   Do not unzip it on top of an older version, and do not copy files between version folders.
2. Open that folder in VS Code: **File, Open Folder**. Install the recommended extensions when asked.
3. Open the terminal (**Terminal, New Terminal**) and run once:
   ```
   powershell -ExecutionPolicy Bypass -File .\setup.ps1
   ```
   It creates `.venv`, installs the packages, runs the unit tests (`python -m unittest discover -s tests`), creates your private
   `.env`, and tells Webots which Python to use. It must end without red text. If tests fail, stop and send the whole output.
4. Press `Ctrl+Shift+P`, type `Python: Select Interpreter`, choose the one with `.venv`. From now on F5 uses it.

`.venv` belongs to this folder only. A new folder always needs step 3 again. If your Python is 3.14 and `pip` fails, install Python
3.13 and run `py -3.13 -m venv .venv` before step 3.

## 2. Run from VS Code (F5)

Press **F5** and choose an NAO entry:

| F5 entry | needs Webots? | what it does |
|---|---|---|
| NAO: MCP self-test (live Webots, direct on 1235) | yes | starts the server like an AI client and runs a scripted session; ends with `SELF-TEST PASSED` |
| NAO: benchmark (live Webots) | yes | runs the real Webots benchmark (connects directly on 1235) |
| NAO: live Webots chat with Gemini / Groq | yes | natural-language control, using a key in `.env` |
| Project: unit tests | no | runs `pytest tests` |

Only one extern controller can own the NAO at a time. For a Webots F5 test, **first** start Webots with `.\start_webots.ps1`, wait
until the house appears, and make sure no other NAO controller is connected (stop `orbit-nao-webots` in VS Code, and any `.\run.ps1
-Mode hold` window). These F5 entries then attach directly to the NAO on `tcp://127.0.0.1:1235`.

These commands were run from this folder here (no Webots):

| command | result here |
|---|---|
| `python conformance.py --sim fake --motion grid` | 11 lines `PASS` |
| `python orbit_mcp_server.py --selftest --sim fake` | `SELF-TEST PASSED` |
| `python orbit_mcp_server.py --selftest --robot nao --sim fake` | `SELF-TEST PASSED` |
| `python tools/make_house_world.py --check` | `orbit_house.wbt is up to date` |
| `python tools/sim_agreement.py --sim fake --motion grid --configs C4 PEL --seeds 1 --limit 3 --out r_sa` | writes `r_sa/sim_agreement_fake_grid.md` |
| `python run_spatial.py --planner oracle --suite core --seeds 1 --configs C4 PEL --out r_try` | `wrote 136 episodes` |

## 4. If Webots shows errors

* `.\run.ps1 -Mode server` opens the only project world, `webots/worlds/orbit_house.wbt`. Then start `orbit-nao-webots`; the MCP
  server itself connects as the single `<extern>` NAO controller over `tcp://127.0.0.1:1235` (no relay process, no port 1240). The
  retired TurtleBot Webots world/controller are not included.
* "'orbit_nao' extern controller: disconnected" means the MCP server process that held the controller exited (the AI client stopped
  it, or Webots restarted). Webots drops an extern controller the instant its Python process ends. Fixes: (a) the VS Code / Claude
  config launches the server with `--keep-alive`, so it holds the controller even when the AI client disconnects; (b) for a controller
  that is independent of any AI client, run `.\run.ps1 -Mode hold` and leave that window open. Only one extern controller can own the
  robot at a time, so run exactly one of: the AI-client server, `.\run.ps1 -Mode hold`, or the standalone Webots benchmark.
  Keep one Webots open; `start_webots.ps1` refuses to start a second instance (it checks for a running `webots`/`webots-bin` process
  and for another listener on controller port 1235).
* The Webots house starts on controller port 1235 so it does not collide with local model servers commonly using port 1234.
  `orbit_ros/humanoid.py` connects the NAO to `tcp://127.0.0.1:1235/orbit_nao`. 1235 is the only TCP port the robot-control path uses.
  If Webots reports
  `WARNING: Unsupported file type '/api/v1/models'`, an HTTP model-discovery request was sent to the Webots controller port; restart
  the house with `.\start_webots.ps1` after closing the old Webots process so the port change takes effect.
* "No module named numpy": the Python that runs the MCP server is not the project's. Use `<this folder>\.venv\Scripts\python.exe` (run `setup.ps1` first).
* `Webots not found`: the scripts look in the default folders, the Windows list of installed programs, the PATH and `Webots` /
  `Program Files\Webots` on every drive. If Webots is somewhere else, put `WEBOTS_HOME=<your Webots folder>` into `.env`.
* The NAO Webots MCP cannot connect: make sure Webots is open (`.\start_webots.ps1`) and showing the house, and that no other NAO
  controller already owns the robot (stop a second `orbit-nao-webots` or a `.\run.ps1 -Mode hold` window). The server connects directly
  on `tcp://127.0.0.1:1235`; there is no separate bridge process to start.
* The picture still looks soft or blurry: in Webots open **Tools, Preferences, OpenGL**, set the texture quality to the highest value
  and make sure anti-aliasing is not disabled, then restart Webots. **File, Reload World** brings the 3D view back after moving it.
* The 2D picture you may have seen earlier is not Webots: World2D (and the map `get_map` prints in the AI app) is a text grid used for
  logic and tests. The 3D robot is only in the Webots window.
* Red underlines under `from controller import ...` in VS Code are only the editor not finding Webots' library; they do not stop
  anything. `.vscode/settings.json` points the editor to the two usual Webots folders.
* If something still fails, send the full terminal text and the Webots console text.

---

## 4. Control the NAO in natural language (MCP)

Talk to the NAO in plain words, such as *"go to the kitchen"* or *"clean the bedroom"*. The AI app sends robot commands to ORBIT
through the **Model Context Protocol (MCP)**. ORBIT checks every command with its tool registry and safety guard before the NAO moves.
Webots is the only supported physical simulator in this project; the old TurtleBot Webots world and controller are retired.

```
you -> Copilot/Claude -> MCP -> orbit_mcp_server.py -> registry + safety guard -> NAO in orbit_house.wbt
```

Start with section 1. For a live connection check, start Webots with `.\start_webots.ps1`, then run the VS Code launch entry
**"NAO: MCP self-test (live Webots, direct on 1235)"**; it must end with `SELF-TEST PASSED`.

### Keeping the NAO controller connected (v0.16.5)

Webots gives up an `<extern>` controller the moment the controller's Python process exits, and the house falls back to
"Waiting for connection". The MCP server **is** that controller, so without care it would connect and disconnect every time the AI
client starts, stops, or reloads it. Two mechanisms keep the connection continuous; run exactly one controller at a time:

* **AI-client server with `--keep-alive` (default config).** `.vscode\mcp.json`, the Claude example config, and `--print-config`
  all launch `orbit_mcp_server.py --robot nao --sim webots --keep-alive`. When the AI client disconnects (chat closed, config reload,
  idle shutdown) the server does **not** exit: it keeps stepping the simulation and holds the controller on `tcp://127.0.0.1:1235`
  until Webots closes or you press Ctrl+C. A finished self-test no longer drops the controller either.
* **Standalone persistent hold:** `.\run.ps1 -Mode hold` opens Webots (if needed) and runs the same `--keep-alive` server in that
  window, holding the controller until Ctrl+C — useful for a demo or a manual session with no AI client attached. Do not also start
  `orbit-nao-webots` while this runs, because only one extern controller may own the robot.

Reconnect handling: if the keep-alive loop sees Webots end (`sup.step()` returns -1) or you press Ctrl+C, the server releases the
controller cleanly and exits; a transient error pauses one second and retries rather than crashing. Keep-alive only steps the world —
it issues no motion and never bypasses the safety guard, the stop-latch or the per-scene call budget.

Duplicate-process prevention: `start_webots.ps1` refuses to open a second Webots if a `webots`/`webots-bin` process is already running
or if controller port 1235 already has a listener. "Only one extern controller can own the robot" is enforced by Webots itself; the
docs and `run.ps1` messages steer you to run just one of the three controllers (AI-client server, `-Mode hold`, or benchmark).

### Startup and health check (v0.16.5)

`run.ps1` no longer blocks on a fixed `Start-Sleep -Seconds 8` after launching Webots. It now **polls the controller port 1235 with a
fast TCP probe** (≈40 ms per probe, every 150 ms, up to 60 s) and continues the instant Webots is ready — not sooner, not later. This
is faster on a warm machine and, more importantly, correct on a cold start: on this machine a fresh Webots took **~13 s** to open port
1235, so the old fixed 8 s sleep under-waited and the server tried to connect before the house was ready. `run.ps1` also skips the
launch entirely when Webots is already running (no duplicate instance).

Reliable readiness check (non-connecting, safe to run while the server/AI client owns the controller — it never takes the controller
slot):

```powershell
.\.venv\Scripts\python.exe orbit_mcp_server.py --robot nao --sim webots --health-check
# [health] READY: Webots controller port 1235 is listening (...)   -> exit code 0
# [health] NOT READY: nothing is listening on 127.0.0.1:1235 ...    -> exit code 2
```

Verified live here on 2026-10-26 (Webots R2025a, `.venv` Python 3.14): Webots opened port 1235 in ~13.2 s; the scripted live self-test
ran 15 MCP commands and ended `SELF-TEST PASSED (245.2 s)`; after the client disconnected, the `--keep-alive` server logged
`client closed; keeping the Webots controller connected on 127.0.0.1:1235` and port 1235 stayed listening across repeated checks until
the server was stopped. Startup Python work is small and unchanged in behaviour (the self-test's `subprocess` import is deferred off the
serve path; `numpy` is still imported lazily only when Webots actually connects).

### Claude Desktop

1. Run `.\.venv\Scripts\python.exe orbit_mcp_server.py --robot nao --sim webots --print-config` (it prints the block, already
   including `--keep-alive`).
2. Add the printed `orbit-nao-webots` entry to `%APPDATA%\Claude\claude_desktop_config.json`.
3. Start Webots with `.\start_webots.ps1`, then restart Claude Desktop. The server holds the controller connected via `--keep-alive`.

### VS Code Copilot

The checked-in `.vscode\mcp.json` contains only `orbit-nao-webots` (the live house world) and includes `--keep-alive`. Start Webots
with `.\start_webots.ps1` first. Then start `orbit-nao-webots` in VS Code and use Agent mode. The MCP server itself owns the single
Webots extern controller and connects directly on `tcp://127.0.0.1:1235` (no relay, no port 1240). Run only one extern controller at a
time (the MCP server, `.\run.ps1 -Mode hold`, or the standalone Webots benchmark).


---

## 5. NAO humanoid in the house (v0.16)

You need section 1 done first, and for the AI app part, section 4.

### Step 5.1. Check it without Webots (1 minute)

```
python orbit_mcp_server.py --selftest --robot nao --sim fake
```
Or F5 entry **"29. NAO in the house: self-test WITHOUT Webots"**. The NAO walks from the living room to the kitchen, to the fridge,
then through the living room into the bedroom to the bed, in a simple stand-in simulator. You see a text map of the house (`|` walls,
`#` furniture, `X` a no-go zone, an arrow for the robot) and lines such as `vetoed: safety guard: would come too close to the bed`:
the guard working on purpose. The last line must be **`SELF-TEST PASSED`**.

Cleaning commands are also available in the stand-in:

```
python orbit_mcp_server.py --robot nao --sim fake --selftest
```

Through an MCP client, use `clean` with `{"mode":"vacuum"}` or `{"mode":"mop"}`, then use
`clean_status`. `clean_house` visits every room through the guarded planner, `pause_cleaning` and
`resume_cleaning` control the cleaning state, and `return_to_start` returns to the scene entry point.

### Step 5.2. See the house in Webots

```
.\run.ps1 -Mode server
```
Webots opens `webots/worlds/orbit_house.wbt` in real time, looking into the house from above (there is no ceiling, so you see
into every room; drag with the mouse to look around, wheel to zoom). `-Mode server` only opens the house; your AI client then starts
`orbit-nao-webots`, which becomes the single extern controller and (thanks to `--keep-alive`) stays connected. To hold the controller
without any AI client, use `.\run.ps1 -Mode hold` instead and leave that window open. For a standalone self-test, stop every other NAO
controller first, then run the F5 entry **"NAO: MCP self-test (live Webots, direct on 1235)"**; you should see the NAO walk
to the kitchen and the fridge.

The NAO walks by driving its joint motors from Webots' own `.motion` files (v0.16.3; Webots' `Motion` player stopped after 0.26 s in this
world, so the turns did not turn). `clean` selects the visible floor attachment (`mode: "vacuum"` or `"mop"`) for a pass where the NAO stands. `clean_room`,
`clean_house`, `clean_corners`, and `clean_direction` accept the same mode; room and house sweeps travel guarded paths with the
selected attachment, and the mode remains selected until another mode is requested or a new scene is loaded. A fall during a walk
is recovered automatically (up to 3 times per walk). `clean_status` reports the mode and progress. Coverage is based on the simulated
brush path, not an external dirt sensor.

* The NAO's walking motions come with Webots (`projects\robots\softbank\nao\motions`). They are found in the same Webots folder as above; if you
  still get "NAO motion files not found", put `WEBOTS_HOME=<your Webots folder>` into `.env`.
* If Webots prints an error about a furniture PROTO (for example `Television.proto` not found), your Webots version does not have that
  object: only that piece of furniture is missing from the picture; the robot still knows it is there and walks around it.
* The robot falls over: ask the AI app to call `stand_up` (or say "stand up").

### Step 5.3. Talk to the NAO from an AI app

* **VS Code (Copilot agent mode):** run `.\run.ps1 -Mode server`, then start the sole configured NAO server, `orbit-nao-webots`, in
  `.vscode\mcp.json`.
* **Claude Desktop:** run `.\.venv\Scripts\python.exe orbit_mcp_server.py --robot nao --sim webots --print-config` and add the
  printed `orbit-nao-webots` block to `claude_desktop_config.json` as in section 4; start Webots with `.\start_webots.ps1`, then
  quit and restart Claude Desktop.

Things to type:
* `Where are you and what do you see?`
* `Go to the kitchen.`  /  `Walk to the fridge.`  /  `Go to the bathroom and wave.`
* `Tour the whole house and return to where you started.`
* `Deep-clean the bathroom.` / `Clean only the kitchen.`
* `Greet me.` / `Wave hello.`
* `Show me the map of the house.`
* `Stay away from the television, then go to the bed.`
* `Clean the whole house in vacuum mode, report progress, and return to where you started.`
* `Pause cleaning.` / `Resume cleaning.` / `Switch to mop mode.`
* `Stop!`  (the robot then refuses every motion until a room is loaded again)

### What the NAO server does and does not allow

* Tools: `go_to_room`, `go_to_object` and `go_to` plan a path around walls and furniture and walk it one motion at a time; `turn_to_object` faces a named item in place without walking. The small
  motions are `walk_forward` (1 to 6 walking cycles), `walk_backward`, `turn` (40, 60 or 180 degrees), `side_step`, plus `wave`/`greet`,
  `tour_house` (visits each room and optionally returns to the start),
  `join_tv_watching` (take a guarded spot beside the sofa and face the television),
  `describe_view`, `look` (camera picture, Webots only), `get_pose`, `stand_up`, `stop_robot`, `house_info`, `get_map`,
  `set_no_go_zone`, `load_scene` (start in a room).
  Cleaning tools: `clean` (`vacuum` or `mop`), `clean_room` (safely navigate to and sweep one room), `clean_house`, `clean_corners` (every corner of the house or one room; corners blocked by
  furniture are listed), `clean_direction` (`forward`, `backward`, `left`, `right` with the brush down), `clean_status`, `pause_cleaning`,
  `resume_cleaning`, and `return_to_start`. Cleaning movement uses the same safety guard as navigation.
* Every motion passes the argument check (wrong values are refused, never changed) and the safety guard: it never walks closer than
  0.3 m to a wall, furniture, something it bumped into or a no-go zone you declared. A refused motion does not move the robot.
* At most 500 robot commands per room start (`--max-calls`). Every command is written to `results_mcp\nao_session_*.jsonl`.
* The house plan is in `orbit_ros/house.py`. To change the house, edit that file and run `python tools/make_house_world.py`; the
  Webots world and the robot's map then change together.

### Step 5.4. Test the MCP server with your own API key (no Claude Desktop or VS Code needed)

`orbit_chat.py` is a small AI app in the terminal. It starts the MCP server exactly as Claude Desktop does, gives its tools to the
model of your API key, and every robot command the model sends goes through the server and its safety guard.

1. Copy the key file once (setup.ps1 already did this if you ran it): `Copy-Item .env.example .env`
2. Open `.env` in VS Code and paste ONE key after the `=`, for example `GEMINI_API_KEY=...` (free key: https://aistudio.google.com/apikey)
   or `GROQ_API_KEY=...` (https://console.groq.com/keys). Check the model line below it (`GEMINI_MODEL=`, `GROQ_MODEL=`). Save.
   Never share `.env`; it is never put in a zip.
3. Run it (or F5 entry **31** for Gemini, **32** for Groq):
```
.\.venv\Scripts\python.exe orbit_chat.py --provider gemini
```
   First start Webots with `.\start_webots.ps1`. The first line must include `--robot nao --sim webots` and list the NAO tools
   (49 here). Then type `Go to the kitchen and stand in front of the fridge.`
   Each robot command is shown as `[MCP] go_to_object {"name": "fridge"} -> ok`. Type `quit` to leave.
4. Both chat launch entries use the live Webots house (direct controller on 1235); there is no fake NAO MCP/chat default.

Other providers: `--provider cerebras | openrouter | openai` with their key and model in `.env`. A wrong or missing key gives a clear
message such as `Set GEMINI_API_KEY in .env`. **Not run against a live provider by us** (no key here); it was tested with a local
stand-in model that drives the real MCP server.

### Step 5.5. NAO memory (v0.16.4)

The NAO remembers what it saw and did, in a small SQLite file (`results_mcp\nao_memory.db`, standard library only, at most 5000 rows).
`orbit_ros/nao_memory.py` holds the store; `orbit_ros/humanoid.py` writes to it; `orbit_chat.py` and the AI app read it.

| Remembered | Written when |
|---|---|
| actions with `ok`, `failed`, `vetoed` or `error` (tool, arguments, room, position, reason) | every motion, macro, wave, stand_up or cleaning call |
| items seen (label, position, room) | `describe_view` |
| obstacles (bump or sonar echo) | the robot bumps into or the sonar detects something the map does not show |
| positions | start of a scene, and after each successful `go_to`, `go_to_room`, `go_to_object`, `clean_house`, ... |

* **Tool `recall_memory`** (read-only, `limit` 1..20) returns what is relevant to where the robot is now: failures and refusals in this
  room or within 2.5 m, obstacles there, items last seen, what worked, recent positions, each with its age, plus a short `brief`.
* **`orbit_chat.py`** calls `recall_memory` before each of your messages and puts the brief in front of the request, so the model sees it
  before it plans. In an AI app (Claude, Copilot) the server instructions tell the model to call `recall_memory` before planning.
* **Current sensors always win.** The memory is advice only: it never feeds the safety guard, the path planner or the obstacle list.
  Every motion still passes the argument check, the guard and the sonar check. Everything recalled is labelled as remembered and old.
  A memory error is swallowed and never stops the robot. Memory of the stand-in simulator and of Webots is kept apart.
* Off: `--no-memory`. Other file: `--memory-db C:\path\mem.db`. Start fresh: delete `results_mcp\nao_memory.db`.

```powershell
cd C:\Users\mdaki\Desktop\LLM_Robot_Project\ORBIT_ROS_v0.16.2_RUN
# 1. tests (memory store, session integration, MCP tool, chat brief)
.\.venv\Scripts\python.exe -m unittest tests.test_nao_memory
# 2. try it without an API key: the robot acts, then you read what it remembers
.\.venv\Scripts\python.exe -c "from orbit_ros.humanoid import HumanoidSession as H; s=H(sim='fake'); s.robot_call('go_to_room',{'room':'kitchen'}); print(s.recall_memory()['brief'])"
# 3. chat with the memory brief added to every request (needs a key in .env); live Webots bridge required
.\.venv\Scripts\python.exe orbit_chat.py --provider groq
# 4. look inside the file
.\.venv\Scripts\python.exe -c "import sqlite3; [print(r) for r in sqlite3.connect('results_mcp/nao_memory.db').execute('select kind, tool, status, room, x, y, label from events order by id desc limit 10')]"
```

Not done: the memory does not forget by age (only by the 5000-row limit), does not merge repeated sightings of one obstacle, and is not
used by the planner itself; a remembered obstacle that has since moved is only a hint for the model. It has not been run in live Webots.

### Step 5.6. Visual debugging (v0.16.4)

The `debug_view` tool (also `HumanoidSession.debug_view()`) shows the current target, planned route, robot position, obstacle locations and
task status. It only reads session state and never moves the robot.

* **Map:** the existing `get_map` text map with the route `*`, target `T`, unmapped obstacles found by bump or sonar `?`, and the robot arrow.
* **Data:** `task` (last motion tool, args, status), `target`, `route`, `next_waypoint`, `pose`, `obstacles` (bumped, sonar with expiry, no-go zones), `replans`.
* **Log:** `results_mcp\nao_session_*.jsonl` gets `route`, `replan` and one `debug` record after every motion, macro, effector or safety call.
* **Webots:** the same six summary lines appear as a label in the top-left of the 3D view (Supervisor `setLabel`; display only, it adds no object).

```powershell
.\.venv\Scripts\python.exe -c "from orbit_ros.humanoid import HumanoidSession; s=HumanoidSession(sim='fake'); s.robot_call('go_to_room',{'room':'kitchen'}); v=s.debug_view(); print(*v['summary'],sep='\n'); print(v['map'])"
Select-String '"event": "debug"' (Get-ChildItem results_mcp\nao_session_*.jsonl | Sort-Object LastWriteTime | Select-Object -Last 1) | Select-Object -Last 3
```

Not done: the route is not drawn as 3D markers in Webots (text label and map only), and the label was not checked in a live Webots window.

### Step 5.7. Several targets in one job (v0.16.4)

`run_plan` takes a list of small steps and runs them in the order that needs the least walking; `plan_tasks` shows the same plan without
moving. The LLM breaks a sentence like "clean the kitchen, wipe beside the sofa, wave in the bedroom, then come back" into steps:

```json
{"steps": [{"room": "kitchen", "do": "clean"}, {"object": "sofa", "do": "clean"}, {"room": "bedroom", "do": "wave"}], "finish": "start"}
```

* **Step:** exactly one target (`room`, `object`, or `x` and `y`) and `do` = `visit` (default), `clean` (sweep a room or make a pass with the currently selected attachment at an object or point) or `wave`.
* **Order:** `optimize` (default) is exact for up to 8 steps (nearest neighbour plus 2-opt above that), using real A* walking lengths around walls and furniture;
  `given` keeps the person's order ("first ... then ..."). `finish: start` ends back at the start and counts that walk. The plan reports the walking length and
  how much ordering saved (`saved_m`).
* **Clean-up of the request:** repeated steps are merged, a visit to a room that is also swept is dropped, and a step with no safe route (no-go zone,
  blocked furniture) is reported as left out and never forced.
* **Completion condition per step:** visit = in the room within 0.5 m of its entry point, or within 0.5 m of the spot beside the object or point; wave = and the
  wave played; clean object/point = and the brush pass done; clean room = brush coverage of the room at least the pattern's limit (34% quick, 45% full).
  Each is checked from the pose and the coverage grid, not from the tool's own status. `verify_task` repeats the check.
* **Failures:** a failed step is reported and the plan goes on with the next one, unless the robot is stopped or has fallen. The reply says which steps were not completed.
* **Safety:** every motion still goes through the same guard, sonar check and replanning. `debug_view` and the Webots label show `plan: 2/4 done, now: ...`.

```powershell
.\.venv\Scripts\python.exe -c "from orbit_ros.humanoid import HumanoidSession; import json; s=HumanoidSession(sim='fake'); print(json.dumps(s.robot_call('plan_tasks',{'steps':[{'room':'bedroom'},{'room':'kitchen'},{'object':'desk'}]})['data'],indent=1))"
.\.venv\Scripts\python.exe -m unittest tests.test_nao_plan -v
```

Not done: the order is only by walking length (no priorities or deadlines), the sweep of a room is not interleaved with other targets, and a step is not retried
from a different side after a failure.

### Change the house or make your own world

* Easiest: edit `orbit_ros/house.py` (rooms, walls, windows, lights, objects with position, rotation and size), then run
  `python tools/make_house_world.py`. **This overwrites `orbit_house.wbt`**, including anything you saved from the Webots editor, so
  copy the file first if you want to keep it. The Webots world and the robot's map change together, so the guard and path planner know
  every new object. `python -m unittest tests.test_humanoid` checks the NAO can still reach every room and object.
* Objects you add by hand in the Webots editor (Add node, `+`) appear in the picture, but the robot does not know them: it finds them
  only by bumping into them. Put them into `house.py` too.
* Webots' own furnished apartment, to look at ideas: `File > Open Sample World > environments > indoor > complete_apartment.wbt`.

---

## 6. NAO benchmark (real-life-style, `tools/nao_benchmark.py`)

Scores the NAO on household jobs. Success is judged from the **simulator's true pose** (Webots supervisor), never from what the robot
reports about itself.

```
.\start_webots.ps1                                             # then, in another terminal:
python tools/nao_benchmark.py --sim webots --only greet tour_house_return clean_room_kitchen
python tools/nao_benchmark.py --sim webots --repeat 3          # whole suite, 3 runs each (hours: the NAO walks in real time)
```
Before running the standalone benchmark, stop every other NAO controller (close any `.\run.ps1 -Mode hold` window and stop
`orbit-nao-webots` in the AI client); the benchmark connects directly to Webots on `tcp://127.0.0.1:1235`. Only one extern controller
can own the robot at a time.

| Group | Tasks | Passes when |
|---|---|---|
| Navigation | `room_kitchen`, `room_bathroom`, `room_bedroom`, `tour_house_return` | true pose visits every room and returns to start |
| Object search | `object_fridge`, `bed`, `toilet`, `sofa`, `desk`, `wardrobe` | ends within 1.5 m of the object, standing |
| Household jobs | `clean_room_kitchen`, `mission_kitchen_bathroom_bedroom_clean`, `clean_each_room_by_hand`, `mission_then_return_to_start`, `long_route_all_rooms`, `clean_house` | selected/all rooms are swept and required coverage is verified |
| Interaction | `greet` | hand-wave motion completes while the NAO remains standing |
| Safety | `fall_recovery`, `no_go_zone_respected`, `blocked_doorway_refused`, `refuse_outside_house`, `guard_stops_wall_walk` | recovers from a fall, never enters a no-go zone, refuses unsafe or impossible goals, no true collisions |

Metrics follow Habitat, ObjectNav, BEHAVIOR and ALFRED: success rate (SR), SPL = success x shortest / max(walked, shortest),
goal-conditioned success, true collisions, falls recovered, motions, metres walked, and simulated and wall-clock time. Output goes to
`results_nao_benchmark/<timestamp>/` as `results.csv`, `summary.json` and `report.txt`. Benchmark examples use live Webots, not a stand-in simulator.
