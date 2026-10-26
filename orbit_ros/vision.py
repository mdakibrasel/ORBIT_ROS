"""Rendered camera image for vision models (v0.12, `run_spatial.py --vision`). Standard library only (zlib PNG encoder).

The image is drawn from the camera's own output for this frame (describe_view: image_x, apparent_size, label), so it shows exactly
what the camera sees, including mirroring, noise, dropped detections and stale frames. Objects are drawn far to near: horizontal position
from image_x, size from apparent_size (= 1/distance), colour from the label's first word, shape from its last word
(box = square, shelf = tall rectangle with shelves, chair = seat + backrest, ball = disc, anything else = triangle). No text is drawn:
a vision model has to recognise the objects. This is a stylised render, not a photo; it tests image reading, not real-world perception."""
import struct, zlib

COLORS = {"red": (210, 40, 40), "blue": (40, 80, 220), "green": (40, 160, 60), "purple": (140, 60, 180), "yellow": (230, 200, 40),
          "orange": (240, 140, 30), "black": (30, 30, 30), "white": (245, 245, 245)}
WALL, FLOOR, EDGE = (225, 222, 210), (150, 140, 125), (60, 60, 60)

def png_bytes(px, w, h):
    raw = b"".join(b"\x00" + bytes(c for p in px[y * w:(y + 1) * w] for c in p) for y in range(h))
    def chunk(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")

def render_view(view, w=320, h=160):
    """view: list of {"label", "image_x", "apparent_size"} -> PNG bytes."""
    horizon = h // 2
    px = [WALL if y < horizon else FLOOR for y in range(h) for _ in range(w)]
    def rect(x0, y0, x1, y1, col):
        for y in range(max(0, int(y0)), min(h, int(y1))):
            for x in range(max(0, int(x0)), min(w, int(x1))): px[y * w + x] = col
    def disc(cx, cy, r, col):
        for y in range(max(0, int(cy - r)), min(h, int(cy + r) + 1)):
            for x in range(max(0, int(cx - r)), min(w, int(cx + r) + 1)):
                if (x - cx) ** 2 + (y - cy) ** 2 <= r * r: px[y * w + x] = col
    def tri(cx, base, s, col):
        for y in range(max(0, int(base - s)), min(h, int(base))):
            half = (y - (base - s)) / 2
            for x in range(max(0, int(cx - half)), min(w, int(cx + half) + 1)): px[y * w + x] = col
    for v in sorted(view, key=lambda v: v["apparent_size"]):           # far first
        a = max(0.05, min(1.5, float(v["apparent_size"])))
        cx = (float(v["image_x"]) + 1) / 2 * (w - 1)
        base = horizon + a * (h - horizon) * 0.9                        # nearer objects stand lower in the image
        s = a * h * 0.6
        words = str(v["label"]).lower().split()
        col = COLORS.get(words[0] if words else "", (128, 128, 128)); kind = words[-1] if words else ""
        if kind == "box":
            rect(cx - s / 2 - 1, base - s - 1, cx + s / 2 + 1, base + 1, EDGE); rect(cx - s / 2, base - s, cx + s / 2, base, col)
        elif kind == "shelf":
            rect(cx - s / 3 - 1, base - 1.6 * s - 1, cx + s / 3 + 1, base + 1, EDGE); rect(cx - s / 3, base - 1.6 * s, cx + s / 3, base, col)
            for k in (1, 2, 3): rect(cx - s / 3, base - 0.4 * k * s, cx + s / 3, base - 0.4 * k * s + max(1, s / 20), EDGE)
        elif kind == "chair":
            rect(cx - s / 2, base - 0.5 * s, cx + s / 2, base - 0.4 * s, col)                       # seat
            rect(cx + s / 2 - s / 8, base - 1.2 * s, cx + s / 2, base - 0.4 * s, col)              # backrest
            for lx in (cx - s / 2, cx + s / 2 - s / 10): rect(lx, base - 0.4 * s, lx + s / 10, base, EDGE)
        elif kind == "ball":
            disc(cx, base - s / 2, s / 2 + 1, EDGE); disc(cx, base - s / 2, s / 2, col)
        else:
            tri(cx, base, s, col)
    return png_bytes(px, w, h)
