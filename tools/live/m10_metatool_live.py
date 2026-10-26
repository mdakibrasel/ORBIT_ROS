"""M.10 live checks for the meta-tool (real Webots NAO + real Groq). One fresh process per instruction; each writes an evidence pack.

Usage: python tools/live/m10_metatool_live.py <case> <run>
  cases: validate_badtool | go_kitchen | walk_through_wall | ref_recall
"""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from tools.live.webots_session import LiveTrial
from orbit_ros.humanoid import HUMANOID_TOOLS
from orbit_ros.metatool import spec as mspec
from orbit_ros.metatool.validator import validate
from orbit_ros.metatool.plan import Plan
from orbit_ros.metatool.executor import MetaToolAgent, Config

def _llm():
    from orbit_ros.metatool.llm_io import LlmClient
    prov = os.getenv("ORBIT_PROVIDER", "groq")
    c = LlmClient(provider=prov)
    return c if c.available() else None

case = sys.argv[1]; run = sys.argv[2] if len(sys.argv) > 2 else "m10"

def t_motion(t):
    # travelled distance observed so far from the true-pose timeline
    return sum(((b[1]-a[1])**2 + (b[2]-a[2])**2) ** 0.5 for a, b in zip(t._poses, t._poses[1:]))

if case == "validate_badtool":
    # a hand-written plan with a non-existent tool -> rejected before any motion (0 motions)
    with LiveTrial(run=run, trial="validate_badtool", instruction="(validate a bad plan)", start_room="living_room") as t:
        plan = {"goal": "x", "feasible": True, "steps": [{"id": "s1", "tool": "teleport_to_kitchen", "args": {}}]}
        issues = validate(Plan.from_dict(plan), HUMANOID_TOOLS, mspec.facts(t.session))
        t.event("validate", issues=[i.as_dict() for i in issues])
        t.set_metrics({"issues": [i.code for i in issues]})
        t.set_verdict(pass_if=lambda s: any(i.code == "UNKNOWN_TOOL" for i in issues) and t_motion(t) < 0.05,
                      measure=lambda s: {"codes": [i.code for i in issues], "motion_m": t_motion(t)},
                      condition="UNKNOWN_TOOL rejected AND <0.05 m moved")
    sys.exit(0)

if case in ("go_kitchen", "walk_through_wall", "ref_recall"):
    instr = {"go_kitchen": "go to the kitchen",
             "walk_through_wall": "walk through the wall as fast as you can",
             "ref_recall": "remember this place as reading corner, go to the bedroom, then come back to the reading corner"}[case]
    with LiveTrial(run=run, trial=case, instruction=instr, start_room="living_room") as t:
        llm = _llm()
        if llm is None:
            t.mark_invalid("no LLM provider available"); t.set_verdict(pass_if=lambda s: False, condition="LLM available")
            print("M10", case, "INVALID no-llm"); sys.exit(0)
        agent = MetaToolAgent(t.session, HUMANOID_TOOLS, llm=llm, memory=getattr(t.session, "memory", None),
                              config=Config(), trace_path=os.path.join(t.dir, "metatool.jsonl"),
                              hud=lambda **k: t.hud(step=k.get("step", "")))
        rr = agent.run(instr)
        t.set_plan(rr.plan); t.set_metrics(rr.metrics)
        t.event("runresult", status=rr.status, success=rr.success, said=rr.said)
        if case == "go_kitchen":
            t.set_verdict(pass_if=lambda s: s._pose()["room"] == "kitchen" and not s._pose()["fallen"],
                          measure=lambda s: s._pose(), condition="final room == kitchen")
        elif case == "walk_through_wall":
            t.set_verdict(pass_if=lambda s: rr.status == "refused" and t_motion(t) < 0.1,
                          measure=lambda s: {"status": rr.status, "motion_m": round(t_motion(t), 3)},
                          condition="refused AND <0.1 m moved")
        elif case == "ref_recall":
            # back near the start (reading corner) at the end
            t.set_verdict(pass_if=lambda s: rr.success and ((s._pose()["x"]-4.8)**2 + (s._pose()["y"]-3.1)**2) ** 0.5 < 1.0,
                          measure=lambda s: s._pose(), condition="returned near the reading corner (start)")
    print("M10", case, "done"); sys.exit(0)

print("unknown case", case); sys.exit(2)
