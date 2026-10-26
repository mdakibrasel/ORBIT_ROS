"""Instruction wording variants (does the model depend on the exact phrasing?).

canonical : the benchmark's original English sentences (default; reproduces all earlier results)
paraphrase: the same task in different English words
bn        : the same task in Bengali (object labels in the camera output stay English, so this also tests cross-lingual grounding)

Only the text the model reads changes. Phase kind/params, the world and the success check are identical, so runs are paired across wordings.
The oracle backend ignores the text, so phrasing effects can only be measured on a real model."""
import re

STYLES = ("canonical", "paraphrase", "bn")
BN_LABEL = {"red box": "লাল বাক্স", "blue box": "নীল বাক্স", "green box": "সবুজ বাক্স", "red shelf": "লাল তাক", "blue shelf": "নীল তাক",
            "red chair": "লাল চেয়ার", "box": "বাক্স", "purple box": "বেগুনি বাক্স"}
BN_DIGITS = str.maketrans("0123456789", "০১২৩৪৫৬৭৮৯")

def _bn_num(n): return str(n).translate(BN_DIGITS)

def _motion_text(ops, style):
    if style == "paraphrase":
        f = lambda v: f"drive {v} meter{'s' if v > 1 else ''} straight ahead"
        t = lambda v: "do a U-turn" if v == "around" else f"rotate 90 degrees to the {v}"
        parts = [f(v) if o == "forward" else t(v) for o, v in ops]
        s = "First " + parts[0] + "".join((", and finally " if i == len(parts) - 1 else ", next ") + p for i, p in enumerate(parts[1:], start=1))
        return s + "."
    f = lambda v: f"{_bn_num(v)} মিটার সামনে যাও"
    t = lambda v: "পেছনে ঘুরে দাঁড়াও" if v == "around" else ("ডানে ঘোরো" if v == "right" else "বাঁয়ে ঘোরো")
    return ", তারপর ".join(f(v) if o == "forward" else t(v) for o, v in ops) + "।"

def _square_text(d, style):
    if style == "paraphrase": return f"Trace a square whose sides are {d} meter{'s' if d > 1 else ''} long, turning right at every corner, until you are back where you began."
    return f"{_bn_num(d)} মিটার বাহুর একটি বর্গক্ষেত্র ধরে চলো: {_bn_num(d)} মিটার সামনে যাও আর ডানে ঘোরো, এভাবে চারবার।"

FIXED = {  # canonical -> (paraphrase, bn)
    "Clean the left side of the room.": ("Vacuum the half of the room that is on your left.", "ঘরের বাঁ দিকটা পরিষ্কার করো।"),
    "Clean the right side of the room.": ("Vacuum the half of the room that is on your right.", "ঘরের ডান দিকটা পরিষ্কার করো।"),
    "Clean the front half of the room (the half you are facing).": ("Vacuum the half of the room in front of you.", "ঘরের সামনের অর্ধেক (তুমি যেদিকে মুখ করে আছ) পরিষ্কার করো।"),
    "Clean the back half of the room (the half behind you).": ("Vacuum the half of the room behind your back.", "ঘরের পেছনের অর্ধেক (তোমার পেছনের দিক) পরিষ্কার করো।"),
    "Go to the object that is closest to you.": ("Drive to whichever object is nearest to you.", "তোমার সবচেয়ে কাছের জিনিসটার কাছে যাও।"),
    "Go to the object that appears furthest to your left.": ("Of the objects you see, drive to the one that is the most to your left.", "যে জিনিসটা তোমার সবচেয়ে বাঁ দিকে দেখা যাচ্ছে, সেটার কাছে যাও।"),
    "Go to the object that is furthest away from you.": ("Drive to the object that is the farthest from you.", "তোমার থেকে সবচেয়ে দূরের জিনিসটার কাছে যাও।"),
    "Now return to the exact cell where you started.": ("Finally, drive back to the exact spot you started from.", "এবার ঠিক যেখান থেকে শুরু করেছিলে, সেই জায়গায় ফিরে যাও।"),
    "Turn around, then move forward 2 meters.": ("Do a U-turn and then drive 2 meters ahead.", "পেছনে ঘুরে দাঁড়াও, তারপর ২ মিটার সামনে যাও।"),
    "Go to the box on your left.": ("Drive to the box that is on your left-hand side.", "তোমার বাঁ দিকের বাক্সটার কাছে যাও।"),
    "Actually, never mind. Go back to the exact cell where you started.": ("Wait, forget that. Return to exactly where you began.", "আসলে থাক। ঠিক যেখান থেকে শুরু করেছিলে সেখানে ফিরে যাও।"),
    "Clean the left side of the room, but do not clean any cell right next to the red box.":
        ("Vacuum the left half of the room, leaving the cells directly beside the red box untouched.", "ঘরের বাঁ দিকটা পরিষ্কার করো, কিন্তু লাল বাক্সের ঠিক পাশের কোনো ঘর পরিষ্কার করবে না।"),
    "Clean the whole room, but don't touch the red box: never drive into a cell right next to it.":
        ("Vacuum the entire room while keeping clear of the red box; do not enter any cell beside it.", "পুরো ঘর পরিষ্কার করো, কিন্তু লাল বাক্স ছোঁবে না: ওটার ঠিক পাশের কোনো ঘরে কখনো ঢুকবে না।"),
    "If you can see a blue box right now, go to it; otherwise turn around and go to the green box.":
        ("Check your current view: if a blue box is in it, drive to the blue box; if not, turn around and drive to the green box.",
         "এই মুহূর্তে যদি একটা নীল বাক্স দেখতে পাও, তার কাছে যাও; না হলে পেছনে ঘুরে সবুজ বাক্সের কাছে যাও।"),
    "How many boxes are in the room? Look around, then give the number with the report tool.":
        ("Count the boxes in this room. Look all around first, then state the count with the report tool.", "ঘরে কয়টা বাক্স আছে? চারপাশে দেখো, তারপর report টুল দিয়ে সংখ্যাটা জানাও।"),
    "How many red objects are in the room? Look around, then give the number with the report tool.":
        ("Count the red objects in this room. Look all around first, then state the count with the report tool.", "ঘরে কয়টা লাল জিনিস আছে? চারপাশে দেখো, তারপর report টুল দিয়ে সংখ্যাটা জানাও।"),
}
DOOR = "The room's door is in the wall you are facing. "
DOOR_PB = ("The entrance door is in the wall in front of you. ", "ঘরের দরজাটা তোমার সামনের দেয়ালে। ")
DOOR_TAIL = {"Go to the box that is on the LEFT for a person standing in the doorway and looking into the room.":
                 ("Drive to the box that would be on the LEFT of someone standing in the doorway and looking into the room.",
                  "দরজায় দাঁড়িয়ে ঘরের ভেতরের দিকে তাকিয়ে থাকা একজন মানুষের বাঁ দিকে যে বাক্সটা, সেটার কাছে যাও।"),
             "Go to the box on the left side.": ("Drive to the box on the left side.", "বাঁ দিকের বাক্সটার কাছে যাও।")}
PATTERNS = [  # (regex on canonical text, paraphrase template, bn template); {L} = label
    (r"Go to the (.+)\.", "Drive up to the {L} and stop next to it.", "{L}টার কাছে যাও।"),
    (r"Now go to the (.+)\.", "Next, head over to the {L}.", "এবার {L}টার কাছে যাও।"),
    (r"Actually, go to the (.+) instead\.", "Change of plan: drive to the {L} instead.", "আসলে, তার বদলে {L}টার কাছে যাও।"),
    (r"Remember the (.+) that is in front of you\.", "Memorise where the {L} in front of you is.", "তোমার সামনের {L}টা কোথায় আছে মনে রাখো।"),
    (r"Go to the (.+) I showed you\.", "Drive to the {L} I pointed out to you earlier.", "যে {L}টা তোমাকে দেখিয়েছিলাম, সেটার কাছে যাও।"),
]

def rephrase_text(text, kind, params, style):
    if style == "canonical": return text
    k = 0 if style == "paraphrase" else 1
    if kind == "motion":
        ops = params["ops"]
        if len(ops) == 8: return _square_text(ops[0][1], style)
        return _motion_text(ops, style)
    if text in FIXED: return FIXED[text][k]
    if text.startswith(DOOR) and text[len(DOOR):] in DOOR_TAIL: return DOOR_PB[k] + DOOR_TAIL[text[len(DOOR):]][k]
    for rx, para, bn in PATTERNS:
        m = re.fullmatch(rx, text)
        if m:
            lab = m.group(1)
            return (para if k == 0 else bn).format(L=lab if k == 0 else BN_LABEL.get(lab, lab))
    raise KeyError(f"no {style} wording for: {text!r}")

def rephrase(task, style):
    """In place: replace every phase text with the chosen wording. Returns the task."""
    if style not in STYLES: raise ValueError(style)
    if style != "canonical":
        for ph in task.phases: ph.text = rephrase_text(ph.text, ph.kind, ph.params, style)
    task.factors = {**task.factors}
    return task
