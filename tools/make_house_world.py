"""Write the Webots world of the ORBIT house (v0.16) from orbit_ros/house.py.

    python tools/make_house_world.py            # writes webots/worlds/orbit_house.wbt
    python tools/make_house_world.py --check    # exit 1 if the file on disk differs from what house.py gives

The world is generated so the robot's map (guard, path planner) and the picture in Webots can never disagree: edit house.py, not the
.wbt file, and run this tool again.
"""
from __future__ import annotations
import argparse, os, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from orbit_ros import house                                   # noqa: E402

OUT = os.path.join(ROOT, "webots", "worlds", "orbit_house.wbt")

def house_world() -> str:
    # Saved world Viewpoint is the straight-down ("bird's-eye") ORBIT_VIEW, matching the shipped orbit_house.wbt and the
    # Phase-16 design ("the saved view is straight-down; the camera() overview mode is the angled one"). This is the saved
    # view only; the runtime camera() tool still provides the angled overview. Keep this in sync with the shipped world.
    pos = (house.W / 2, house.H / 2, 30.0)                     # straight down over the house centre; no ceiling, so every room shows
    rot = (0.0, 1.0, 0.0, 1.5708)                              # clean top-down orientation (matches the shipped saved view)
    return house.world_text(pos, rot, house.ROOMS["living_room"][0]) + "\n"

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args(argv)
    text = house_world()
    if a.check:
        same = os.path.exists(OUT) and open(OUT, encoding="utf-8").read() == text
        print("orbit_house.wbt is up to date" if same else "orbit_house.wbt differs from orbit_ros/house.py: run tools/make_house_world.py")
        return 0 if same else 1
    with open(OUT, "w", encoding="utf-8", newline="\n") as f: f.write(text)
    print(f"wrote {OUT} ({len(house.ITEMS)} items, {len(house.WALLS)} wall segments)")
    return 0

if __name__ == "__main__":
    sys.exit(main())
