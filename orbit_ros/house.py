"""ORBIT house (v0.16): a furnished one-floor home for the NAO humanoid, one definition used three ways:

* `tools/make_house_world.py` writes the Webots world `webots/worlds/orbit_house.wbt` from it (walls, floors and furniture);
* the humanoid session uses it as the robot's map (the safety guard and the path planner);
* the FakeNao test backend uses it for contacts.

Layout (metres, x to the right, y up, house [0, 10] x [0, 8]):

    y=8 +------------------------+-------------+
        |  kitchen + dining      |  bathroom   |
        |  fridge, counter,      |  bath, WC,  |
        |  dining table, chairs  |  washbasin  |
    4.5 +---------------+-door-+-------------+     (x=6 wall: doors at y 1.4-2.6 and 5.9-7.1; 1.2 m doorways)
        |  living room           |  bedroom    |
        |  TV, coffee table,     |  bed, desk, |
        |  sofa, armchair        |  wardrobe   |
    y=0 +------------------------+-------------+
        x=0                     x=6          x=10

Everything is built from Webots R2025a's own object library where it has the object (checked against the R2025a sources and the
"complete_apartment" sample world that ships with Webots): Sofa, Armchair, Table, Chair, OfficeChair, Desk, Bed, Fridge, Sink,
HotPlate, Cookware, Plate, FruitBowl, Apple, Orange, Television, Laptop, PottedTree, BunchOfSunFlowers, FloorLight, CeilingLight,
LandscapePainting, Clock, Bathtube, Toilet, BathroomSink, WashingMachine, RubberDuck, Wall and Window, with the Parquetry,
PorcelainChevronTiles, CementTiles, CarpetFibers and PaintedWood materials. Simple boxes only where Webots has no ready object (TV
stand, bookshelf, wardrobe, kitchen counter, stove, washbasin cabinet). Walls are 2.4 m high like a real house; there is no ceiling, so you
can look in from above. Every item has a footprint box used by the guard and the planner.
"""
from __future__ import annotations
import heapq, math

W, H = 10.0, 8.0
WALL_T, WALL_H = 0.12, 2.4
# wall segments (x1, y1, x2, y2), axis-aligned; doorways are the gaps
WALLS = [(0, 0, 10, 0), (0, 8, 10, 8), (0, 0, 0, 8), (10, 0, 10, 8),
         (0, 4.5, 4.4, 4.5), (5.6, 4.5, 6, 4.5), (6, 4.5, 10, 4.5),          # living | kitchen, door x 4.4-5.6
         (6, 0, 6, 1.4), (6, 2.6, 6, 5.9), (6, 7.1, 6, 8)]                     # x=6: doors y 1.4-2.6 (bedroom) and 5.9-7.1 (bathroom)
# windows in the outside walls: (x1, y1, x2, y2) on a wall line, sill height, window height (metres)
WINDOWS = [(4.0, 0, 5.4, 0, 0.8, 1.4),        # living room, south
           (0, 1.2, 0, 2.4, 0.8, 1.4),        # living room, west
           (1.8, 8, 3.2, 8, 1.05, 1.0),       # kitchen, above the counter
           (8.0, 0, 9.4, 0, 0.8, 1.4),        # bedroom, south
           (10, 5.9, 10, 6.7, 1.4, 0.7)]      # bathroom, high
AP = {   # Webots appearance nodes used here
    "parquet_light": 'Parquetry { type "light strip" textureTransform TextureTransform { scale 6 4.5 } }',
    "parquet_dark": 'Parquetry { type "dark strip" textureTransform TextureTransform { scale 4 4.5 } }',
    "kitchen_tiles": "PorcelainChevronTiles { textureTransform TextureTransform { scale 6 3.5 } }",
    "bath_tiles": "CementTiles { textureTransform TextureTransform { scale 4 3.5 } }",
    "carpet": "CarpetFibers { colorOverride 0.75 0.3 0.3 textureTransform TextureTransform { scale 4 3 } }",
    "dark_wood": "PaintedWood { colorOverride 0.25 0.2 0.18 }",
    "light_wood": "PaintedWood { colorOverride 0.82 0.7 0.52 }",
    "white": "PaintedWood { colorOverride 0.93 0.93 0.91 }",
}
APPEARANCE_PROTOS = {"Parquetry": "appearances/protos/Parquetry", "PorcelainChevronTiles": "appearances/protos/PorcelainChevronTiles",
                     "CementTiles": "appearances/protos/CementTiles", "CarpetFibers": "appearances/protos/CarpetFibers",
                     "PaintedWood": "appearances/protos/PaintedWood", "Roughcast": "appearances/protos/Roughcast"}

ROOMS = {   # name: (start / target point x, y, yaw for a start), floor rectangle, floor appearance
    "living_room": ((4.8, 3.1, 180), (0, 0, 6, 4.5), "parquet_light"),
    "kitchen": ((3.9, 5.8, 180), (0, 4.5, 6, 8), "kitchen_tiles"),
    "bedroom": ((7.0, 2.9, 0), (6, 0, 10, 4.5), "parquet_dark"),
    "bathroom": ((7.2, 5.6, 90), (6, 4.5, 10, 8), "bath_tiles"),
}
LIGHTS = [  # ceiling lights (x, y, colour, intensity): the light of each room
    (3.0, 2.3, "1 0.9 0.75", 2.5), (3.0, 6.25, "1 0.95 0.85", 2.5), (8.0, 2.3, "1 0.85 0.65", 2.0), (8.0, 6.25, "0.9 0.95 1", 2.0)]

def _i(label, x, y, sx, sy, sz, proto=None, rot=0, color=(0.7, 0.7, 0.7), parts=None, obstacle=True, z=0.0, fields=""):
    return {"label": label, "x": x, "y": y, "sx": sx, "sy": sy, "sz": sz, "proto": proto, "rot": rot, "color": color,
            "parts": parts, "obstacle": obstacle, "z": z, "fields": fields}

# x, y: where the item stands (for a PROTO its origin); sx, sy: footprint after rotation, used by the guard and the planner.
# parts: (dx, dy, dz_bottom, sx, sy, sz, colour or AP key) relative to the item centre, for items built from boxes.
# Webots objects face +x before rotation (Sofa, Armchair, Chair, Fridge, Television, Toilet: back at -x); rot turns them (degrees).
ITEMS = [
    # living room
    _i("sofa", 2.8, 0.6, 1.8, 1.0, 0.8, proto="Sofa", rot=90, fields="color 0.55 0.6 0.7"),
    _i("armchair", 4.25, 1.75, 1.0, 1.1, 0.8, proto="Armchair", rot=180, fields="color 0.55 0.6 0.7"),
    _i("coffee table", 2.8, 1.75, 1.0, 0.6, 0.4, proto="Table", fields="size 1 0.6 0.4"),
    _i("TV remote", 2.8, 1.75, 0.18, 0.07, 0.03,
       parts=[(0, 0, 0.4, 0.18, 0.07, 0.025, (0.12, 0.12, 0.13)),
              (0, 0, 0.426, 0.045, 0.025, 0.008, (0.75, 0.16, 0.12))], obstacle=False),
    _i("carpet", 2.8, 1.9, 2.2, 1.6, 0.004, parts=[(0, 0, 0, 2.2, 1.6, 0.004, "carpet")], obstacle=False),
    _i("TV stand", 2.8, 4.15, 1.2, 0.45, 0.5, parts=[(0, 0, 0, 1.2, 0.45, 0.5, "dark_wood"), (-0.3, -0.226, 0.08, 0.5, 0.005, 0.3, (0.15, 0.12, 0.1)),
                                                     (0.3, -0.226, 0.08, 0.5, 0.005, 0.3, (0.15, 0.12, 0.1))]),
    _i("television", 2.8, 4.2, 1.0, 0.1, 0.6, proto="Television", rot=-90, z=0.8, obstacle=False),
    _i("potted plant", 0.45, 0.45, 0.6, 0.6, 1.3, proto="PottedTree"),
    _i("floor lamp", 1.45, 0.4, 0.4, 0.4, 1.7, proto="FloorLight", fields="pointLightIntensity 0.6 pointLightRadius 3"),
    _i("sunflowers", 0.35, 4.15, 0.35, 0.35, 1.0, proto="BunchOfSunFlowers"),
    _i("bookshelf", 0.25, 3.3, 0.35, 0.8, 1.1, parts=[(0, 0, 0, 0.35, 0.8, 1.1, "light_wood"),
                                                     (0.02, -0.2, 0.75, 0.3, 0.3, 0.25, (0.7, 0.15, 0.15)), (0.02, 0.2, 0.4, 0.3, 0.3, 0.25, (0.15, 0.3, 0.7)),
                                                     (0.02, 0.15, 0.08, 0.3, 0.4, 0.25, (0.2, 0.5, 0.3))]),
    _i("painting", 0.09, 3.3, 0.05, 1.5, 1.0, proto="LandscapePainting", z=1.25, obstacle=False),
    # kitchen and dining
    _i("fridge", 0.42, 7.57, 0.75, 0.75, 1.8, proto="Fridge", rot=-90),
    _i("kitchen counter", 2.6, 7.65, 2.6, 0.6, 0.9, parts=[(0, 0, 0, 2.6, 0.6, 0.86, "white"), (0, 0, 0.86, 2.6, 0.62, 0.04, (0.22, 0.22, 0.24))]),
    _i("sink", 2.2, 7.65, 0.6, 0.45, 0.05, proto="Sink", rot=-90, z=0.9, obstacle=False),
    _i("stove", 4.3, 7.65, 0.6, 0.6, 0.9, parts=[(0, 0, 0, 0.6, 0.6, 0.88, (0.15, 0.15, 0.15)), (0, -0.301, 0.15, 0.5, 0.005, 0.45, (0.05, 0.05, 0.06))]),
    _i("hot plate", 4.3, 7.65, 0.55, 0.55, 0.03, proto="HotPlate", rot=-90, z=0.92, obstacle=False),
    _i("cooking pot", 4.18, 7.55, 0.25, 0.25, 0.15, proto="Cookware", z=0.94, obstacle=False),
    _i("dining table", 2.2, 5.9, 1.2, 0.8, 0.74, proto="Table", fields="size 1.2 0.8 0.74"),
    _i("chair", 1.35, 5.9, 0.5, 0.5, 0.95, proto="Chair", rot=0, fields="color 0.6 0.45 0.3"),
    _i("chair", 3.05, 5.9, 0.5, 0.5, 0.95, proto="Chair", rot=180, fields="color 0.6 0.45 0.3"),
    _i("fruit bowl", 2.2, 5.9, 0.3, 0.3, 0.1, proto="FruitBowl", z=0.74, obstacle=False),
    _i("apple", 2.16, 5.88, 0.08, 0.08, 0.08, proto="Apple", z=0.8, obstacle=False),
    _i("orange", 2.25, 5.93, 0.08, 0.08, 0.08, proto="Orange", z=0.8, obstacle=False),
    _i("plate", 1.8, 5.9, 0.22, 0.22, 0.01, proto="Plate", z=0.75, obstacle=False),
    _i("plate", 2.6, 5.9, 0.22, 0.22, 0.01, proto="Plate", z=0.75, obstacle=False),
    _i("clock", 0.08, 6.1, 0.05, 0.33, 0.33, proto="Clock", z=1.8, obstacle=False),
    # bedroom
    _i("bed", 8.88, 1.3, 2.12, 1.75, 0.9, proto="Bed", rot=180),
    _i("wardrobe", 9.65, 3.6, 0.6, 1.4, 1.8, parts=[(0, 0, 0, 0.6, 1.4, 1.8, "light_wood"), (-0.301, 0, 0.05, 0.005, 0.01, 1.7, (0.3, 0.25, 0.2)),
                                                    (-0.305, -0.08, 0.95, 0.01, 0.03, 0.2, (0.6, 0.6, 0.62)), (-0.305, 0.08, 0.95, 0.01, 0.03, 0.2, (0.6, 0.6, 0.62))]),
    _i("desk", 6.8, 0.43, 1.2, 0.7, 0.72, proto="Desk", rot=90),
    _i("desk chair", 6.8, 1.12, 0.6, 0.6, 1.0, proto="OfficeChair", rot=-90, fields="enablePhysics FALSE"),
    _i("laptop", 6.8, 0.4, 0.35, 0.25, 0.25, proto="Laptop", rot=90, z=0.72, obstacle=False),
    _i("potted plant", 6.4, 4.1, 0.6, 0.6, 1.3, proto="PottedTree"),
    _i("painting", 9.91, 1.3, 0.05, 1.5, 1.0, proto="LandscapePainting", rot=180, z=1.35, obstacle=False),
    # bathroom
    _i("bathtub", 8.98, 7.47, 1.9, 0.93, 0.6, proto="Bathtube", rot=90),
    _i("rubber duck", 8.7, 7.45, 0.1, 0.1, 0.1, proto="RubberDuck", z=0.25, obstacle=False),
    _i("toilet", 6.5, 7.51, 0.58, 0.86, 0.8, proto="Toilet", rot=-90),
    _i("washbasin", 7.6, 7.72, 0.7, 0.45, 0.9, parts=[(0, 0, 0, 0.7, 0.45, 0.8, "white")]),
    _i("bathroom sink", 7.6, 7.94, 0.7, 0.35, 0.2, proto="BathroomSink", rot=-90, z=0.8, obstacle=False),
    _i("washing machine", 9.55, 4.95, 0.73, 0.77, 1.05, proto="WashingMachine", z=0.53),
]

PROTOS = {   # every path checked to exist in Webots R2025a (github.com/cyberbotics/webots, tag R2025a)
    "Sofa": "living_room_furniture/protos/Sofa", "Armchair": "living_room_furniture/protos/Armchair", "Table": "tables/protos/Table",
    "Desk": "tables/protos/Desk", "Chair": "chairs/protos/Chair", "OfficeChair": "chairs/protos/OfficeChair", "Bed": "bedroom/protos/Bed",
    "Fridge": "kitchen/fridge/protos/Fridge", "Sink": "kitchen/components/protos/Sink", "HotPlate": "kitchen/components/protos/HotPlate",
    "Cookware": "kitchen/utensils/protos/Cookware", "Plate": "kitchen/utensils/protos/Plate", "FruitBowl": "fruits/protos/FruitBowl",
    "Apple": "fruits/protos/Apple", "Orange": "fruits/protos/Orange", "Television": "television/protos/Television",
    "Laptop": "computers/protos/Laptop", "PottedTree": "plants/protos/PottedTree", "BunchOfSunFlowers": "plants/protos/BunchOfSunFlowers",
    "CleaningBrush": "../protos/CleaningBrush.proto",
    "FloorLight": "lights/protos/FloorLight", "CeilingLight": "lights/protos/CeilingLight", "LandscapePainting": "paintings/protos/LandscapePainting",
    "Clock": "school_furniture/protos/Clock", "Bathtube": "bathroom/protos/Bathtube", "Toilet": "bathroom/protos/Toilet",
    "BathroomSink": "bathroom/protos/BathroomSink", "WashingMachine": "bathroom/protos/WashingMachine", "RubberDuck": "toys/protos/RubberDuck",
    "Wall": "apartment_structure/protos/Wall", "Window": "apartment_structure/protos/Window",
}
WEBOTS_PROTO_BASE = "webots://projects"
PROTOS = {k: f"{WEBOTS_PROTO_BASE}/objects/{v}.proto" for k, v in PROTOS.items()}
PROTOS["CleaningBrush"] = "../protos/CleaningBrush.proto"

def wall_boxes(t=WALL_T):
    """Walls as footprint boxes {x, y, sx, sy} (the guard and the planner treat them like furniture)."""
    out = []
    for x1, y1, x2, y2 in WALLS:
        out.append({"label": "wall", "x": (x1 + x2) / 2, "y": (y1 + y2) / 2, "sx": abs(x2 - x1) + t, "sy": abs(y2 - y1) + t, "wall": True})
    return out

def obstacles():
    """Everything the robot must not walk into."""
    return wall_boxes() + [{k: it[k] for k in ("label", "x", "y", "sx", "sy")} for it in ITEMS if it["obstacle"]]

def room_of(x, y):
    for name, (_, (x0, y0, x1, y1), _) in ROOMS.items():
        if x0 <= x <= x1 and y0 <= y <= y1: return name
    return None

def blocks_view(x0, y0, x1, y1) -> bool:
    """True when the straight line between two points crosses a wall (used for what the camera can see)."""
    for wx1, wy1, wx2, wy2 in WALLS:
        if wx1 == wx2:                                           # vertical wall
            if (x0 - wx1) * (x1 - wx1) < 0:
                t = (wx1 - x0) / (x1 - x0); yy = y0 + t * (y1 - y0)
                if min(wy1, wy2) <= yy <= max(wy1, wy2): return True
        else:
            if (y0 - wy1) * (y1 - wy1) < 0:
                t = (wy1 - y0) / (y1 - y0); xx = x0 + t * (x1 - x0)
                if min(wx1, wx2) <= xx <= max(wx1, wx2): return True
    return False

# ------------------------------------------------------------------------------------------------------------- path planner
def plan_path(start, goal, obstacle_boxes, clearance, res=0.1, w=W, h=H, comfort=0.2, escape=0.6, body=0.2):
    """A* on a res-metre grid over the free space (boxes inflated by `clearance`). Cells closer than clearance + comfort to an obstacle
    cost more, so paths keep to the middle of doorways and gaps. A robot that stands too close already (after a bump) may cross
    cells within `escape` m of the start that keep `body` m from everything, to get clear. -> list of waypoints (first = start) or None."""
    nx, ny = int(round(w / res)), int(round(h / res))
    def free_at(i, j, c):
        x, y = (i + 0.5) * res, (j + 0.5) * res
        if not (c <= x <= w - c and c <= y <= h - c): return False
        return not any(abs(x - o["x"]) < o["sx"] / 2 + c and abs(y - o["y"]) < o["sy"] / 2 + c for o in obstacle_boxes)
    cell = lambda p: (min(nx - 1, max(0, int(p[0] / res))), min(ny - 1, max(0, int(p[1] / res))))
    s, g = cell(start), cell(goal)
    grid, wide = {}, {}
    def ok(c):
        if c not in grid:
            grid[c] = free_at(*c, clearance) or (math.hypot((c[0] + 0.5) * res - start[0], (c[1] + 0.5) * res - start[1]) <= escape
                                                  and free_at(*c, body))
        return grid[c]
    def roomy(c):
        if c not in wide: wide[c] = free_at(*c, clearance + comfort)
        return wide[c]
    if not free_at(*g, clearance): return None
    openq, came, cost = [(0.0, s)], {s: None}, {s: 0.0}
    steps = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while openq:
        _, c = heapq.heappop(openq)
        if c == g: break
        for di, dj, dc in steps:
            n = (c[0] + di, c[1] + dj)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or not ok(n): continue
            if di and dj and not (ok((c[0] + di, c[1])) and ok((c[0], c[1] + dj))): continue
            nc = cost[c] + dc * (1 if roomy(n) else 4)
            if nc < cost.get(n, 1e18):
                cost[n], came[n] = nc, c; heapq.heappush(openq, (nc + math.hypot(g[0] - n[0], g[1] - n[1]), n))
    if g not in came: return None
    path, c = [], g
    while c is not None: path.append(((c[0] + 0.5) * res, (c[1] + 0.5) * res)); c = came[c]
    path = path[::-1]; path[0] = tuple(start); path[-1] = tuple(goal)
    # keep only the corners: drop points a straight line can skip; the line must keep the roomy clearance where the path itself did
    def clear(a, b):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res / 2)))
        cells = [cell((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)) for k in range(n + 1)]
        return all(ok(c) for c in cells) and all(roomy(c) for c in cells[1:-1])
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not clear(path[i], path[j]): j -= 1
        out.append(path[j]); i = j
    return out

# ------------------------------------------------------------------------------------------------------------- Webots world
def _rot(deg): return f"0 0 1 {math.radians(deg):.4f}"

def _box(name, x, y, z, sx, sy, sz, color, bounding=True, rough=0.7):
    r, g, b = color; sx, sy, sz = (f"{round(v, 4):g}" for v in (sx, sy, sz))
    shape = f"Shape {{ appearance PBRAppearance {{ baseColor {r} {g} {b} roughness {rough} metalness 0 }} geometry Box {{ size {sx} {sy} {sz} }} }}"
    bo = f" boundingObject Box {{ size {sx} {sy} {sz} }}" if bounding else ""
    return f"Solid {{ translation {x:.3f} {y:.3f} {z:.3f} name \"{name}\" children [ {shape} ]{bo} }}"

def _ap(c, rough=0.6):
    """Appearance node: a colour tuple or a key of AP (a Webots material)."""
    if isinstance(c, str): return AP[c]
    r, g, b = c
    return f"PBRAppearance {{ baseColor {r} {g} {b} roughness {rough} metalness 0 }}"

def _n(v): return f"{round(v, 4):g}"

def wall_pieces():
    """Wall segments split around the windows: [(kind, x1, y1, x2, y2, sill, height)], kind = wall | window."""
    out = []
    for x1, y1, x2, y2 in WALLS:
        horiz = y1 == y2
        a, b = (min(x1, x2), max(x1, x2)) if horiz else (min(y1, y2), max(y1, y2))
        pt = (lambda u: (u, y1)) if horiz else (lambda u: (x1, u))
        cuts = []
        for wx1, wy1, wx2, wy2, sill, hgt in WINDOWS:
            on = (horiz and wy1 == wy2 == y1) or (not horiz and wx1 == wx2 == x1)
            c0, c1 = sorted((wx1, wx2) if horiz else (wy1, wy2))
            if on and a <= c0 < c1 <= b: cuts.append((c0, c1, sill, hgt))
        pos = a
        for c0, c1, sill, hgt in sorted(cuts):
            if c0 > pos: out.append(("wall", *pt(pos), *pt(c0), None, None))
            out.append(("window", *pt(c0), *pt(c1), sill, hgt)); pos = c1
        if pos < b: out.append(("wall", *pt(pos), *pt(b), None, None))
    return out

def world_text(view_pos, view_rot, nao_start) -> str:
    """The Webots world file text (written by tools/make_house_world.py)."""
    used = sorted({it["proto"] for it in ITEMS if it["proto"]} | {"Wall", "Window", "CeilingLight", "CleaningBrush"})
    L = ["#VRML_SIM R2025a utf8", "",
         f'EXTERNPROTO "{WEBOTS_PROTO_BASE}/robots/softbank/nao/protos/Nao.proto"',
         f'EXTERNPROTO "{WEBOTS_PROTO_BASE}/objects/backgrounds/protos/TexturedBackground.proto"',
         f'EXTERNPROTO "{WEBOTS_PROTO_BASE}/objects/backgrounds/protos/TexturedBackgroundLight.proto"']
    L += [f'EXTERNPROTO "{WEBOTS_PROTO_BASE}/{v}.proto"' for v in APPEARANCE_PROTOS.values()]
    L += [f'EXTERNPROTO "{PROTOS[p]}"' for p in used]
    L += ["", "# ORBIT house (v0.16): a furnished home for the NAO humanoid. GENERATED by tools/make_house_world.py from orbit_ros/house.py;",
          "# edit house.py and run the tool again instead of editing this file. The NAO's controller is <extern>: start the world with",
          "# .\\start_webots.ps1 -Humanoid , then the MCP server (orbit_mcp_server.py --robot nao --sim webots) drives it. Not run by the author.",
          "WorldInfo {", '  title "ORBIT house (NAO humanoid, extern controller for the MCP server)"', "  basicTimeStep 20", '  coordinateSystem "ENU"',
          "  contactProperties [", '    ContactProperties {', '      material2 "NAO foot material"', "      coulombFriction [", "        7", "      ]",
          "      bounce 0.3", "      bounceVelocity 0.003", "    }", "  ]", "}",
          "DEF ORBIT_VIEW Viewpoint {", f"  orientation {' '.join(f'{v:.4f}' for v in view_rot)}", f"  position {' '.join(f'{v:.3f}' for v in view_pos)}",
          "  near 0.05", "  exposure 0.8", "  ambientOcclusionRadius 1", "}",
          'TexturedBackground {', '  texture "noon_cloudy_countryside"', "}", "TexturedBackgroundLight {", '  texture "noon_cloudy_countryside"',
          "  luminosity 0.7", "  castShadows TRUE", "}"]
    L.append(_box("ground", W / 2, H / 2, -0.06, 30, 30, 0.1, (0.42, 0.55, 0.32), rough=1))
    floors = " ".join(f"Pose {{ translation {_n((x0 + x1) / 2 - W / 2)} {_n((y0 + y1) / 2 - H / 2)} 0 children [ Shape {{ appearance {AP[ap]} "
                      f"geometry Plane {{ size {_n(x1 - x0)} {_n(y1 - y0)} }} }} ] }}" for _, (x0, y0, x1, y1), ap in ROOMS.values())
    L.append(f'DEF FLOOR Solid {{ translation {_n(W / 2)} {_n(H / 2)} 0 name "floor" children [ {floors} ] boundingObject Plane {{ size {_n(W)} {_n(H)} }} }}')
    wall_ap = "Roughcast { colorOverride 0.95 0.92 0.86 textureTransform TextureTransform { scale 4 2 } }"
    for k, (kind, x1, y1, x2, y2, sill, hgt) in enumerate(wall_pieces()):
        horiz = y1 == y2; length = abs(x2 - x1) + abs(y2 - y1) + (WALL_T if kind == "wall" else 0)
        rot = "0 0 1 1.5708" if horiz else "0 0 1 0"
        common = f"translation {_n((x1 + x2) / 2)} {_n((y1 + y2) / 2)} 0 rotation {rot} name \"{kind}_{k}\" size {_n(WALL_T)} {_n(length)} {_n(WALL_H)}"
        if kind == "wall": L.append(f"Wall {{ {common} appearance {wall_ap} }}")
        else: L.append(f"Window {{ {common} bottomWallHeight {_n(sill)} windowHeight {_n(hgt)} wallAppearance {wall_ap} }}")
    for k, (x, y, col, inten) in enumerate(LIGHTS):
        L.append(f'CeilingLight {{ translation {_n(x)} {_n(y)} {_n(WALL_H)} name "ceiling light {k}" pointLightColor {col} pointLightIntensity {_n(inten)} '
                 f"pointLightRadius 6 }}")
    for k, it in enumerate(ITEMS):
        name = f"{it['label'].replace(' ', '_')}_{k}"
        if it["proto"]:
            extra = f" {it['fields']}" if it["fields"] else ""
            L.append(f"{it['proto']} {{ translation {it['x']:.3f} {it['y']:.3f} {it['z']:.3f} rotation {_rot(it['rot'])} name \"{name}\"{extra} }}")
        else:
            kids = []
            for dx, dy, dz, sx, sy, sz, col in it["parts"]:
                kids.append(f"Pose {{ translation {_n(dx)} {_n(dy)} {_n(dz + sz / 2)} children [ Shape {{ appearance {_ap(col)} "
                            f"geometry Box {{ size {_n(sx)} {_n(sy)} {_n(sz)} }} }} ] }}")
            bo = f" boundingObject Pose {{ translation 0 0 {_n(it['sz'] / 2)} children [ Box {{ size {_n(it['sx'])} {_n(it['sy'])} {_n(it['sz'])} }} ] }}" if it["obstacle"] else ""
            L.append(f"Solid {{ translation {it['x']:.3f} {it['y']:.3f} {it['z']:.3f} rotation {_rot(it['rot'])} name \"{name}\" children [ {' '.join(kids)} ]{bo} }}")
    x, y, yaw = nao_start
    L += [f'DEF NAO_CLEANING_BRUSH CleaningBrush {{ translation {x + 0.32 * math.cos(math.radians(yaw)):.3f} {y + 0.32 * math.sin(math.radians(yaw)):.3f} 0.001 rotation {_rot(yaw)} name "nao_cleaning_brush" mode "vacuum" }}',
           "Nao {", f"  translation {x} {y} 0.334", f"  rotation {_rot(yaw)}", '  name "orbit_nao"', '  controller "<extern>"', "  supervisor TRUE", "}", ""]
    return "\n".join(L)
