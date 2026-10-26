r"""Live trial harness: ONE fresh Supervisor process per trial against the real Webots NAO house.

A trial opens a HumanoidSession(sim="webots") (the only simulator), reads the TRUE pose from the Supervisor FOR EVALUATION ONLY
(the robot is driven only through session.robot_call, which runs the guard), captures screenshots / console / a true-pose timeline,
shows a HUD via setLabel, and writes an evidence pack to results_live/<run>/<trial>/.

It never teleports the robot during a task; the only pose write is load_scene at the scene start. If another extern controller is
already connected (port 1235 in use by someone else) the trial refuses to run.

Usage (inside a per-trial worker process):

    from tools.live.webots_session import LiveTrial
    with LiveTrial(run="demo_ts", trial="go_kitchen", instruction="go to the kitchen", start_room="living_room") as t:
        # t.session is a live HumanoidSession; drive it via t.session.robot_call / the meta-tool
        t.hud(step="go_to_room kitchen")
        r = t.session.robot_call("go_to_room", {"room": "kitchen"})
        t.event("tool", tool="go_to_room", status=r["status"])
        t.set_verdict(pass_if=lambda s: s._pose()["room"] == "kitchen",
                      measure=lambda s: s._pose(), condition="final room == kitchen")
    # on exit: end.png, true_poses.csv, metrics.json, verdict.json, webots_console.txt are flushed.
"""
from __future__ import annotations
import csv, io, json, os, socket, sys, threading, time, contextlib

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path: sys.path.insert(0, ROOT)
RESULTS = os.path.join(ROOT, "results_live")

def port_listening(port=1235, host="127.0.0.1"):
    try:
        with socket.create_connection((host, port), timeout=1): return True
    except OSError: return False

def ensure_webots(timeout_s=200):
    """Start Webots via start_webots.ps1 if 1235 is not listening, and wait until the controller port is up."""
    if port_listening(): return True
    import subprocess
    ps = os.path.join(ROOT, "start_webots.ps1")
    subprocess.Popen(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", ps], cwd=ROOT)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        time.sleep(2)
        if port_listening(): time.sleep(3); return True
    return False


class LiveTrial:
    def __init__(self, run, trial, instruction, start_room="living_room", sample_s=0.5):
        self.run, self.trial, self.instruction, self.start_room = run, trial, instruction, start_room
        self.sample_s = sample_s
        self.dir = os.path.join(RESULTS, run, trial)
        os.makedirs(self.dir, exist_ok=True)
        self._trace = open(os.path.join(self.dir, "trace.jsonl"), "w", encoding="utf-8")
        self._poses = []            # (t, x, y, yaw, fallen)
        self._shots = []            # screenshot metadata (file, camera_mode, phase, t)
        self._sampling = False
        self._verdict = {"condition": None, "measured": None, "result": "INVALID"}
        self._plan = None
        self._metrics = {}
        self._t0 = time.time()
        self._invalid_reason = None
        self.session = None

    # ---- lifecycle ----
    def __enter__(self):
        from orbit_ros.humanoid import HumanoidSession
        self.event("start", instruction=self.instruction, start_room=self.start_room,
                   webots_up=port_listening(), pid=os.getpid())
        self.session = HumanoidSession(sim="webots", log_dir=False, memory=False)
        # refuse if the controller is somehow not ours (a real second controller would make getSelf fail)
        self.session.load_scene(self.start_room)
        cs = self.session.connection_status()
        if not cs.get("connected"):
            self._invalid_reason = "no live controller connection"
            raise RuntimeError("LiveTrial: not connected to the live Webots controller")
        self.event("connected", **{k: cs.get(k) for k in ("backend", "controller_port", "uses_port_1240")})
        # adaptive viewpoint (display only; reuses the session's Supervisor, creates nothing)
        try:
            from orbit_ros.viewpoint import Viewpoint
            self.vp = Viewpoint(self.session.nao)
            self._cam_mode = "overview"
            if self.vp.available(): self.vp.overview()      # official start/end screenshots are in overview
            self.event("camera", mode="overview", ok=self.vp.available())
        except Exception as e:
            self.vp = None; self._cam_mode = "overview"; self.event("camera_init_failed", error=str(e))
        self.hud(step="(start)")
        self.screenshot("start.png", mode="overview")
        self._sampling = True
        self._sampler = threading.Thread(target=self._sample_loop, daemon=True)
        self._sampler.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self._sampling = False
        try: self._sampler.join(timeout=2)
        except Exception: pass
        if exc is not None:
            self.event("error", error=f"{exc_type.__name__ if exc_type else '?'}: {exc}")
        try:
            if getattr(self, "vp", None) and self.vp.available():
                self.vp.overview()                                  # official end.png in overview
                try: self.vp.render(frames=8)                       # render the overview before the shot (fields alone do not render)
                except Exception: pass
            self.screenshot("end.png", mode="overview", phase="after")
        except Exception: pass
        self._flush()
        self._trace.close()
        try: self.session and self.session.memory and self.session.memory.close()
        except Exception: pass
        return False        # never swallow exceptions

    # ---- true-pose sampling (evaluation only) ----
    def _sample_loop(self):
        while self._sampling:
            try:
                x, y, yaw, fallen = self.session.nao.pose()
                self._poses.append((round(time.time() - self._t0, 2), round(x, 3), round(y, 3), round(yaw, 1), int(bool(fallen))))
            except Exception:
                pass
            time.sleep(self.sample_s)

    # ---- evidence helpers ----
    def event(self, kind, **kw):
        rec = {"t": round(time.time() - self._t0, 3), "event": kind, **kw}
        self._trace.write(json.dumps(rec, default=str) + "\n"); self._trace.flush()

    def hud(self, step="", verify=""):
        try:
            nao = self.session.nao
            if hasattr(nao, "sup"):
                nao.sup.setLabel(0, f"instruction: {self.instruction[:60]}", 0.01, 0.01, 0.06, 0xFFFFFF, 0.0, "Lucida Console")
                nao.sup.setLabel(1, f"step: {step[:70]}", 0.01, 0.05, 0.06, 0x00FF88, 0.0, "Lucida Console")
                if verify: nao.sup.setLabel(2, f"verify: {verify[:70]}", 0.01, 0.09, 0.06, 0xFFCC00, 0.0, "Lucida Console")
        except Exception:
            pass

    def camera(self, mode, target_xy=None, zoom=None, **kw):
        """Switch the adaptive viewpoint. Display only — never moves the robot, resets physics or calls the LLM."""
        if not getattr(self, "vp", None) or not self.vp.available():
            self.event("camera", mode=mode, ok=False, reason="no viewpoint"); return {"ok": False}
        if mode == "overview": r = self.vp.overview()
        elif mode == "reset": r = self.vp.reset()
        elif mode == "follow": r = self.vp.follow(**kw)
        elif mode == "demo": r = self.vp.demo()
        elif mode == "closeup": r = self.vp.closeup()
        elif mode == "human": r = self.vp.human()
        elif mode == "target": r = self.vp.target(target_xy or self._robot_xy())
        elif mode == "zoom": r = self.vp.zoom(zoom if zoom is not None else "in")
        else: r = {"ok": False, "mode": mode, "reason": "unknown mode"}
        # optional dolly after a framing mode
        if zoom is not None and mode != "zoom" and r.get("ok"):
            zr = self.vp.zoom(zoom); r["zoom"] = zoom; r["zoom_result"] = zr
        # render the new view through the existing controller so the screenshot reflects it (lifecycle 4.7); display only
        if r.get("ok"):
            try: r["rendered"] = self.vp.render(frames=8)
            except Exception: r["rendered"] = False
        self._cam_mode = r.get("mode", mode) if r.get("ok") else self._cam_mode
        self.event("camera", **r)
        return r

    def _robot_xy(self):
        try: p = self.session.nao.node.getPosition(); return (p[0], p[1])
        except Exception: return (5.0, 4.0)

    def screenshot(self, name, mode=None, phase="during"):
        mode = mode or getattr(self, "_cam_mode", "overview")
        try:
            nao = self.session.nao
            if hasattr(nao, "sup"):
                nao.sup.exportImage(os.path.join(self.dir, name), 90)
                rec = {"file": name, "camera_mode": mode, "phase": phase, "t": round(time.time() - self._t0, 2)}
                self._shots.append(rec)
                self.event("screenshot", **rec)
        except Exception as e:
            self.event("screenshot_failed", file=name, error=str(e))

    def set_plan(self, plan):
        self._plan = plan
        with open(os.path.join(self.dir, "plan.json"), "w", encoding="utf-8") as f:
            json.dump(plan, f, indent=2, default=str)
        self.event("plan", plan=plan)

    def set_metrics(self, metrics):
        self._metrics = dict(metrics or {})

    def mark_invalid(self, reason):
        self._invalid_reason = reason
        self.event("invalid", reason=reason)

    def set_verdict(self, pass_if, measure=None, condition=""):
        """Evaluate the pass condition from the live session (true pose/state). Sets PASS/FAIL (or INVALID if a fallback/reset occurred)."""
        measured = None
        try:
            measured = measure(self.session) if measure else None
            ok = bool(pass_if(self.session))
        except Exception as e:
            self._verdict = {"condition": condition, "measured": f"error: {e}", "result": "INVALID"}
            self.event("verdict", **self._verdict); return self._verdict
        # honesty: a fallback or supervisor reset during the task invalidates the trial
        fb = getattr(self.session.nao, "fallbacks", 0)
        resets = getattr(self.session.nao, "supervisor_resets", 0)
        if self._invalid_reason or fb or resets:
            result = "INVALID"
            if not self._invalid_reason: self._invalid_reason = f"fallbacks={fb} supervisor_resets={resets}"
        else:
            result = "PASS" if ok else "FAIL"
        self._verdict = {"condition": condition, "measured": measured, "result": result,
                         "invalid_reason": self._invalid_reason, "fallbacks": fb, "supervisor_resets": resets}
        self.event("verdict", **self._verdict)
        return self._verdict

    def _flush(self):
        with open(os.path.join(self.dir, "screenshots.json"), "w", encoding="utf-8") as f:
            json.dump(self._shots, f, indent=2)
        with open(os.path.join(self.dir, "true_poses.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f); w.writerow(["t", "x", "y", "yaw", "fallen"]); w.writerows(self._poses)
        self._metrics.setdefault("wall_s", round(time.time() - self._t0, 2))
        self._metrics["pose_samples"] = len(self._poses)
        self._metrics["motions_observed"] = _count_motion(self._poses)
        with open(os.path.join(self.dir, "metrics.json"), "w", encoding="utf-8") as f:
            json.dump(self._metrics, f, indent=2, default=str)
        with open(os.path.join(self.dir, "verdict.json"), "w", encoding="utf-8") as f:
            json.dump(self._verdict, f, indent=2, default=str)
        # Webots console: we cannot read Webots' own stdout from here, but we record the controller-side messages we saw.
        if not os.path.exists(os.path.join(self.dir, "webots_console.txt")):
            with open(os.path.join(self.dir, "webots_console.txt"), "w", encoding="utf-8") as f:
                f.write("\n")


def _count_motion(poses):
    """A coarse count of how much the robot actually moved (for the 0-motions acceptance checks)."""
    if len(poses) < 2: return 0.0
    d = 0.0
    for a, b in zip(poses, poses[1:]):
        d += ((b[1]-a[1])**2 + (b[2]-a[2])**2) ** 0.5
    return round(d, 3)


# A tiny synthetic (no-robot) evidence-pack writer, used by the code check for H.1.
def _synthetic_pack(run, trial, dir_poses):
    """Write a complete evidence pack WITHOUT a robot, from provided fake poses — for the code check only.
    This is a TEST helper and is clearly not a robot result (it never connects to Webots)."""
    d = os.path.join(RESULTS, run, trial); os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "trace.jsonl"), "w", encoding="utf-8") as f:
        f.write(json.dumps({"event": "start", "note": "synthetic code-check pack, not a robot result"}) + "\n")
    with open(os.path.join(d, "true_poses.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f); w.writerow(["t", "x", "y", "yaw", "fallen"]); w.writerows(dir_poses)
    for n in ("plan.json", "metrics.json", "verdict.json"):
        json.dump({"synthetic": True}, open(os.path.join(d, n), "w", encoding="utf-8"))
    open(os.path.join(d, "webots_console.txt"), "w").write("\n")
    return d
