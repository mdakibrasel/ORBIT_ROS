"""Lesson memory (v0.11): after a failed episode the agent writes a short lesson; before a new episode the most similar lessons are
retrieved and shown to the planner. This is the "observed phenomenon -> knowledge" loop: a failure becomes a reusable rule.

* A lesson = {"id", "task_id", "family", "level", "yaw", "instruction", "failure_class", "text", "effect"}.
* Retrieval is deterministic: same family (+2), same level (+0.5), same start heading (+0.25), word overlap of the instructions (+2 x Jaccard).
  Lessons from the same task instance are never retrieved (no leakage across repeats).
* With a real model the lesson text is written by the model itself (one extra call, tag "lesson"). The oracle uses a template per
  failure class, and only two templates change its behaviour (`effect`): "frame" removes the left/right frame confusion and "arith" halves
  the slip rate. These effects are ASSUMPTIONS of the simulator, not measurements."""
import json, os, re

TEMPLATES = {
    "left_right_confusion": ("Left/right/front/back refer to my heading at the START of the task. Convert them to map cells before planning.", "frame"),
    "frame_of_reference": ("Left/right/front/back refer to my heading at the START of the task. Convert them to map cells before planning.", "frame"),
    "distance_arithmetic": ("Re-check the distances: the final pose of the plan must match the instruction exactly.", "arith"),
    "turn_arithmetic": ("Re-check every turn: left adds +90 degrees, right subtracts 90; the final heading must match.", "arith"),
    "sensor_misreading_mirrored_camera": ("If an object is not where the camera said, the camera may be mirrored: image right can be my left.", ""),
    "memory_loss": ("Write down landmarks and the start cell before moving; later instructions refer back to them.", ""),
    "instruction_forgotten": ("Keep every earlier instruction and the latest change in mind; finish each stage where it ends.", ""),
    "budget_exhausted": ("Do not repeat a failing action; after two failures change the approach or abstain.", ""),
    "looping": ("Do not repeat a failing action; after two failures change the approach or abstain.", ""),
    "counting_error": ("When counting, merge sightings that refer to the same map cell; look in all four directions once.", ""),
    "constraint_violated": ("Treat forbidden cells as walls when planning a path.", ""),
    "condition_misjudged": ("Evaluate a condition on the view at the moment the instruction is given, then keep that decision.", ""),
}
DEFAULT = ("Check the result against the instruction before declaring the task done.", "")

def _words(s): return set(re.findall(r"\w+", s.lower()))

class LessonStore:
    def __init__(self, path=None):
        self.path, self.items = path, []
        if path and os.path.isfile(path):
            with open(path, encoding="utf-8") as f: self.items = [json.loads(l) for l in f if l.strip()]
    def add(self, lesson):
        lesson = {"id": len(self.items), **lesson}; self.items.append(lesson)
        if self.path:
            with open(self.path, "a", encoding="utf-8") as f: f.write(json.dumps(lesson, ensure_ascii=False) + "\n")
        return lesson
    def retrieve(self, task, k=3):
        text = " ".join(p.text for p in task.phases); tw = _words(text); scored = []
        for l in self.items:
            if l["task_id"] == task.task_id: continue
            s = 2.0 * (l["family"] == task.family) + 0.5 * (l.get("level") == task.factors.get("level")) + 0.25 * (l.get("yaw") == task.factors.get("yaw"))
            lw = _words(l.get("instruction", "")); s += 2.0 * len(tw & lw) / max(1, len(tw | lw))
            if s >= 1.0: scored.append((s, -l["id"], l))
        out, seen = [], set()
        for s, _, l in sorted(scored, key=lambda x: (x[0], x[1]), reverse=True):
            if l["text"] in seen: continue                       # no duplicate texts in one prompt
            seen.add(l["text"]); out.append(l)
            if len(out) == k: break
        return out

def template_lesson(failure_class):
    return TEMPLATES.get(failure_class, DEFAULT)
