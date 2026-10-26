"""ORBIT robot session for the MCP server (v0.15): natural-language control of the ORBIT TurtleBot3 from an AI client.

    s = OrbitMcpSession(sim="world2d")            # or "fake", "webots" (extern controller), "ros2"
    s.load_scene("demo")
    s.robot_call("move_forward", {"distance": 2})  # -> {"status": "ok" | "rejected" | "vetoed" | "failed" | "error", ...}

The AI client (Claude Desktop, VS Code, Gemini CLI, ...) turns the person's words into tool calls; this module executes them. There is
exactly ONE path from an MCP tool call to the robot, `robot_call`, and it is the PEL path without the planner:

    budget check -> registry.validate_call (name, typed arguments, preconditions) -> SafetyGuard.check (room bounds, believed obstacles,
    declared no-go cells, per-call distance limit) -> world.execute (World2D, or the existing Webots/FakeSim/ROS 2 adapter) -> belief update

A macro (goto_cell, cover_region) is expanded by navtools.expand and every primitive it produces goes through the same registry and guard
checks again before it is sent, so a macro cannot reach the robot by a side door. Nothing here can switch the registry or the guard off.

What the guard knows: the robot's REPORTED pose and the session's BELIEVED map (objects the camera reported, cells where a move was
blocked, no-go cells the person declared). It never reads the simulator's true occupancy; the simulator-side stop (World2D safety gate,
Webots contacts) is a separate layer.

Agent tools of the benchmark (ask, abstain, remember, done, replan) are not exposed: the AI client is the agent and talks to the person
directly. Scene loading, status and task checking are session tools: they never move the robot.

Every call is appended to a JSON Lines audit log (call, registry issues, guard verdict, result).
"""
from __future__ import annotations
import json, os, time
from .pel.registry import REGISTRY, RegistryContext, validate_call
from .pel.guard import SafetyGuard
from .navtools import MACROS, expand
from .world2d import World2D, Obj, heading
from .cognition import SensorInterpreter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PRIMITIVES = ("move_forward", "turn", "clean", "describe_view", "get_pose", "localize", "report", "stop_robot")
DEFAULT_MAX_CALLS = 500

DEMO_OBJECTS = (("blue box", 7, 5), ("red chair", 2, 1), ("green box", 6, 1), ("red shelf", 1, 5), ("blue shelf", 7, 2))

def _demo():
    return World2D(9, 7, (4, 3), 0, [Obj(f"o{i}", lab, x, y) for i, (lab, x, y) in enumerate(DEMO_OBJECTS)]), None

def _empty():
    return World2D(9, 7, (4, 3), 0, []), None

SCENES = {
    "demo": ("9 x 7 m room, robot at (4, 3) facing +x, five objects (boxes, a chair, shelves)", _demo),
    "empty": ("9 x 7 m room, robot at (4, 3) facing +x, no objects", _empty),
}

def _task_families():
    from .tasks2d import ALL_FAMILIES
    return ALL_FAMILIES

def scene_list() -> list:
    out = [{"name": n, "description": d} for n, (d, _) in SCENES.items()]
    out += [{"name": f"task:{f}", "description": f"benchmark task family '{f}' (tasks2d.make_task); comes with the person's instruction and "
             "check_task scores it"} for f in _task_families()]
    return out

class OrbitMcpSession:
    def __init__(self, sim="world2d", motion: str = "grid", perception: str = "camera", nav_tools: bool = True, localizer: bool = False,
                 max_calls: int = DEFAULT_MAX_CALLS, log_dir: str | None = None, ros2_config: str | None = None, guard: SafetyGuard | None = None,
                 default_scene: str = "demo"):
        from .sim_select import SIMS
        if isinstance(sim, str) and sim not in SIMS: raise ValueError(f"unknown simulator {sim}; choose from {SIMS}")
        if motion not in ("grid", "continuous"): raise ValueError("motion must be grid or continuous")
        if sim == "world2d" and motion != "grid": raise ValueError("world2d is a grid world: use --motion grid (continuous needs fake, webots or ros2)")
        self.sim, self.motion, self.perception, self.ros2_config = sim, motion, perception, ros2_config
        self.nav_tools, self.localizer, self.max_calls = nav_tools, localizer, max_calls
        self.guard = guard or SafetyGuard()
        self.default_scene = default_scene                          # loaded at the first robot call if no load_scene came before
        self.world = self.task = None
        self.scene = None
        self.log_path = None
        if log_dir is not False:
            d = log_dir or os.path.join(ROOT, "results_mcp")
            try:
                os.makedirs(d, exist_ok=True)
                self.log_path = os.path.join(d, time.strftime("session_%Y%m%dT%H%M%SZ", time.gmtime()) + f"_{os.getpid()}.jsonl")
            except OSError:
                self.log_path = None
        self._reset_belief()

    # ------------------------------------------------------------------------------------------------------------ tools offered
    def robot_tools(self) -> list:
        """Registry tools this session offers to the AI client (primitives, plus macros / localize when switched on)."""
        out = [t for t in PRIMITIVES if t != "localize" or self.localizer]
        if self.nav_tools: out += sorted(MACROS)
        return out

    # ------------------------------------------------------------------------------------------------------------ scene
    def _reset_belief(self):
        self.seen, self.blocked_known, self.no_go, self.cleaned_known = set(), set(), set(), set()
        self.calls = {"total": 0, "ok": 0, "rejected": 0, "vetoed": 0, "failed": 0, "error": 0}
        self.last_view, self.last_veto = [], None

    def load_scene(self, scene: str = "demo", seed: int = 0, yaw: int = 0, n_objects: int = 1, level: int = 1) -> dict:
        """(Re)builds the room in the chosen simulator and puts the robot at the scene's start. Also the only way out of stop_robot."""
        task = None
        if scene in SCENES:
            world, _ = SCENES[scene][1]()
        elif scene.startswith("task:"):
            from .tasks2d import make_task
            fam = scene.split(":", 1)[1]
            if fam not in _task_families(): raise ValueError(f"unknown task family {fam!r}; see list_scenes")
            if yaw not in (0, 90, 180, 270): raise ValueError("yaw must be 0, 90, 180 or 270")
            world, task = make_task(fam, int(seed), int(yaw), n_objects=int(n_objects), level=int(level))
        else:
            raise ValueError(f"unknown scene {scene!r}; see list_scenes")
        world.localizer = bool(self.localizer)
        from .sim_select import in_simulator
        self.world = in_simulator(world, self.sim, self.motion, self.perception, self.ros2_config)
        self.task, self.scene = task, scene
        self._reset_belief()
        out = {"scene": scene, "simulator": self.sim_name(), "motion": self.motion, "room_w": self.world.w, "room_h": self.world.h,
               "pose": self.world.pose()}
        if task is not None:
            out["task_id"] = task.task_id
            out["instruction"] = [p.text for p in task.phases]
        self._log({"event": "load_scene", **out})
        return out

    def sim_name(self) -> str: return self.sim if isinstance(self.sim, str) else type(self.sim).__name__

    def _ensure(self):
        if self.world is None: self.load_scene(self.default_scene)

    # ------------------------------------------------------------------------------------------------------------ belief
    def believed_obstacles(self) -> set:
        return set(self.seen) | set(self.blocked_known)

    def _ctx(self):
        # robot-reported facts only; asks_left=0 and memory=False because ask / remember are not offered
        return RegistryContext(stopped=bool(self.world.stopped), has_localiser=bool(self.world.localizer), memory=False,
                               nav_tools=self.nav_tools, asks_left=0)

    # ------------------------------------------------------------------------------------------------------------ the one path
    def robot_call(self, tool: str, args: dict | None = None) -> dict:
        """Every MCP robot tool lands here. Returns {"status", "message", "data", ...}; never raises for a refused call."""
        self._ensure()
        args = {} if args is None else args
        call = {"tool": tool, "args": args}
        self.calls["total"] += 1
        if tool not in self.robot_tools():
            return self._finish(call, {"status": "rejected", "message": f"{tool!r} is not offered by this server (offered: {', '.join(self.robot_tools())})"})
        if self.calls["total"] > self.max_calls:
            return self._finish(call, {"status": "rejected", "message": f"call budget of {self.max_calls} robot calls for this scene is used up; "
                                       "load_scene starts a new one"})
        res = self._guarded(call)
        if res["status"] == "ok" and tool in MACROS:
            res = self._run_macro(call, res)
        return self._finish(call, res)

    def _check(self, call):
        """Registry, then guard. -> refusal dict or None (admissible)."""
        issues = validate_call(call, self._ctx())
        errs = [i for i in issues if i.severity == "error"]
        if errs:
            return {"status": "rejected", "message": "; ".join(i.message for i in errs), "issues": [str(i) for i in issues]}
        why = self.guard.check(call, self.world.pose(), self.believed_obstacles(), self.no_go, self.world.start_pose, self.cleaned_known)
        if why:
            self.last_veto = why
            return {"status": "vetoed", "message": f"safety guard: {why}"}
        warn = [i.message for i in issues]
        return {"status": "ok", "warnings": warn} if warn else {"status": "ok"}

    def _guarded(self, call):
        ref = self._check(call)
        if ref["status"] != "ok" or call["tool"] in MACROS: return ref
        return self._execute(call, ref)

    def _execute(self, call, pre):
        tool, args = call["tool"], call["args"]
        try:
            r = self.world.execute(tool, args)
        except Exception as e:                                      # adapter failure (Webots closed, ROS 2 timeout): report, keep serving
            return {"status": "error", "message": f"{type(e).__name__}: {e}"}
        data = r.get("data")
        if tool == "describe_view" and r["ok"]:
            data = self._interpret(data)
        if tool == "clean" and r["ok"]:
            p = self.world.pose(); self.cleaned_known.add((p["x"], p["y"]))
        if tool == "move_forward" and not r["ok"] and str(r["message"]).startswith(("blocked", "safety stop")) and r.get("data"):
            dx, dy = heading(r["data"]["yaw_deg"]); self.blocked_known.add((r["data"]["x"] + dx, r["data"]["y"] + dy))
        out = {"status": "ok" if r["ok"] else "failed", "message": r["message"], "data": data}
        if pre.get("warnings"): out["warnings"] = pre["warnings"]
        return out

    def _interpret(self, raw):
        """Camera detections -> bearing, distance and estimated map cell (the same interpreter the benchmark agents use)."""
        pose = self.world.pose()
        it = SensorInterpreter(self.world.fov, self.world.mirrored, grounded=True).interpret(raw or [], pose)
        me = (pose["x"], pose["y"])
        self.seen |= {tuple(o["world_xy"]) for o in it if tuple(o["world_xy"]) != me}
        self.last_view = it
        return [{**r, **{k: o[k] for k in ("bearing_deg", "distance_m", "world_xy")}} for r, o in zip(raw or [], it)]

    def _run_macro(self, call, checked):
        pose = self.world.pose()
        prims, err = expand(call["tool"], call["args"], pose, self.world.start_pose, sorted(self.believed_obstacles() | self.no_go), self.cleaned_known)
        if err: return {"status": "rejected", "message": err}
        steps = []
        for p in prims:
            r = self._guarded(p)                                    # every primitive is checked again before it is sent
            steps.append({"call": p, "status": r["status"], "message": r["message"]})
            if r["status"] != "ok":
                return {"status": r["status"], "message": f"{call['tool']} stopped at step {len(steps)} of {len(prims)}: {r['message']}", "steps": steps,
                        "data": self.world.pose()}
        return {"status": "ok", "message": f"{call['tool']}: {len(prims)} robot actions, done", "steps": steps, "data": self.world.pose()}

    def _finish(self, call, res):
        res.setdefault("data", None)
        self.calls[res["status"]] = self.calls.get(res["status"], 0) + 1
        self._log({"event": "call", "call": call, **{k: v for k, v in res.items() if k != "steps"}, "n_steps": len(res.get("steps", []))})
        return res

    # ------------------------------------------------------------------------------------------------------------ session tools
    def set_no_go(self, cells) -> dict:
        """Adds cells the robot must never enter (the person says "stay away from ..."). Cleared by load_scene; cannot be removed otherwise."""
        self._ensure()
        try: add = {(int(c[0]), int(c[1])) for c in cells}
        except (TypeError, ValueError, IndexError, KeyError): raise ValueError("cells must be a list of [x, y] pairs")
        self.no_go |= add
        self._log({"event": "set_no_go", "added": sorted(map(list, add))})
        return {"no_go_cells": sorted(map(list, self.no_go))}

    def status(self) -> dict:
        self._ensure()
        w = self.world
        return {"scene": self.scene, "simulator": self.sim_name(), "motion": self.motion, "pose": w.pose(), "stopped": bool(w.stopped),
                "believed_obstacles": sorted(map(list, self.believed_obstacles())), "no_go_cells": sorted(map(list, self.no_go)),
                "cleaned_cells": sorted(map(list, self.cleaned_known)), "reports": list(w.reports), "calls": dict(self.calls),
                "call_budget_left": max(0, self.max_calls - self.calls["total"]), "last_veto": self.last_veto, "audit_log": self.log_path}

    def believed_map(self) -> dict:
        """ASCII map of what the robot believes (not the simulator's truth). Row y = room_h-1 is printed first (y up)."""
        self._ensure()
        p = self.world.pose(); w, h = p["room_w"], p["room_h"]
        arrow = {0: ">", 90: "^", 180: "<", 270: "v"}[p["yaw_deg"] % 360]
        obst, labels = self.believed_obstacles(), {tuple(o["world_xy"]): o["label"] for o in self.last_view}
        rows = []
        for y in range(h - 1, -1, -1):
            line = []
            for x in range(w):
                c = (x, y)
                line.append(arrow if c == (p["x"], p["y"]) else "#" if c in obst else "X" if c in self.no_go else "*" if c in self.cleaned_known else ".")
            rows.append(f"{y:2d} " + " ".join(line))
        rows.append("   " + " ".join(str(x % 10) for x in range(w)))
        return {"map": "\n".join(rows), "legend": f"{arrow} robot (heading), # believed obstacle, X no-go, * cleaned, . unknown/free",
                "labels_last_view": {f"{k[0]},{k[1]}": v for k, v in labels.items()}}

    def check_task(self) -> dict:
        """Evaluator score of a task:<family> scene (uses ground truth; never used to control the robot)."""
        self._ensure()
        if self.task is None: return {"task": None, "message": "this scene has no benchmark task; load a task:<family> scene"}
        from .tasks2d import check_success
        return {"task_id": self.task.task_id, "instruction": [p.text for p in self.task.phases], "result": check_success(self.world, self.task)}

    def capabilities(self) -> dict:
        tools = {}
        for t in self.robot_tools():
            s = REGISTRY[t]
            tools[t] = {"category": s.category, "args": {k: {"kind": a.kind, "min": a.min, "max": a.max, "values": list(a.values), "required": a.required}
                                                          for k, a in s.args.items()},
                        "preconditions": list(s.pre), "retry_safe": s.retry_safe}
        return {"robot": "TurtleBot3 Burger (differential drive, camera with object recognition, LDS-01 lidar)", "simulator": self.sim_name(),
                "motion": self.motion, "robot_tools": tools, "max_cells_per_move": self.guard.max_steps_per_call, "call_budget": self.max_calls,
                "safety": "every robot call passes the PEL registry and safety guard; nothing can switch them off"}

    # ------------------------------------------------------------------------------------------------------------ audit log
    def _log(self, rec):
        if not self.log_path: return
        try:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps({"t": round(time.time(), 3), **rec}, default=str) + "\n")
        except OSError:
            pass
