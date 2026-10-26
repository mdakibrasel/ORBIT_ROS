"""Tool-manifest integrity for the meta-tool / MCP server (Phase 17, feature 11). Pure code: no robot, no LLM.

Builds a DETERMINISTIC canonical representation of the registered tool catalogue (names, categories, argument
schemas, and prompt/description text) and hashes it with SHA-256. The expected hash is stored in
`configs/tool_manifest.lock`. At startup the server can verify the live catalogue against the lock; an unexpected
difference must BLOCK robot actions until the discrepancy is investigated (fail-closed), because a changed tool
surface can be an injection or a tampering signal.

Deliberate updates follow a documented procedure: run `write_lock(registry)` on purpose (e.g. after adding a tool)
and commit the new lock, exactly as measured baselines are regenerated on purpose elsewhere in this project.

This module never executes a tool and never changes the MCP transport; it only compares catalogue fingerprints."""
from __future__ import annotations
import hashlib, json, os

LOCK_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                         "configs", "tool_manifest.lock")


def _tools_dict(registry):
    if isinstance(registry, dict): return dict(registry)
    if hasattr(registry, "items"): return dict(registry.items())
    return {s.name: s for s in registry}


def _arg_canon(a):
    """Canonical, order-stable dict for one tool argument (kind/required/range/enum/doc)."""
    return {
        "kind": getattr(a, "kind", ""),
        "required": bool(getattr(a, "required", True)),
        "min": getattr(a, "min", None),
        "max": getattr(a, "max", None),
        "values": list(getattr(a, "values", ()) or ()),
        "doc": getattr(a, "doc", "") or "",
    }


def canonical(registry):
    """A deterministic, JSON-serialisable canonical form of the whole tool catalogue (sorted by tool name)."""
    tools = _tools_dict(registry)
    out = []
    for name in sorted(tools):
        spec = tools[name]
        args = {k: _arg_canon(a) for k, a in sorted((getattr(spec, "args", {}) or {}).items())}
        out.append({
            "name": name,
            "category": getattr(spec, "category", ""),
            "side_effect": bool(getattr(spec, "side_effect", False)),
            "robot": bool(getattr(spec, "robot", True)),
            "prompt": getattr(spec, "prompt", "") or "",
            "args": args,
        })
    return out


def canonical_json(registry):
    return json.dumps(canonical(registry), sort_keys=True, separators=(",", ":"))


def manifest_hash(registry):
    return hashlib.sha256(canonical_json(registry).encode("utf-8")).hexdigest()


def write_lock(registry, path=LOCK_PATH):
    """Write the lock file on purpose (documented, deliberate update). Returns the hash written."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    h = manifest_hash(registry)
    payload = {"version": 1, "sha256": h, "tool_count": len(_tools_dict(registry)),
               "tools": sorted(_tools_dict(registry).keys())}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=1, sort_keys=True)
    return h


def read_lock(path=LOCK_PATH):
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def verify(registry, path=LOCK_PATH):
    """Verify the live catalogue against the lock.

    Returns (ok, detail). ok=False means robot actions should be blocked (fail-closed) until investigated.
    If no lock exists yet, ok=False with a clear reason (never silently trust an unlocked catalogue)."""
    lock = read_lock(path)
    live = manifest_hash(registry)
    if lock is None:
        return False, f"no tool_manifest.lock at {path}; run write_lock to establish the expected catalogue"
    want = lock.get("sha256")
    if want != live:
        live_names = set(_tools_dict(registry).keys())
        lock_names = set(lock.get("tools") or [])
        added = sorted(live_names - lock_names)
        removed = sorted(lock_names - live_names)
        return False, (f"tool manifest mismatch: expected {want}, got {live}; "
                       f"added={added} removed={removed} (schema/description change if names match)")
    return True, f"tool manifest verified ({live})"
