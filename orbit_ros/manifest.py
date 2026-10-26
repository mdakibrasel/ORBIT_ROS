"""Provenance helpers (v0.13): hashes of the code, the prompts and the environment, written into every run_metadata.json and
into the baseline record (tools/baseline_record.py). A hash gives traceability; it does not by itself make a result valid."""
import hashlib, importlib, os, platform, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGES = ("numpy", "pandas", "matplotlib", "google.genai", "rclpy")

def sha256_file(path, chunk=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(chunk), b""): h.update(b)
    return h.hexdigest()

def source_files(root=ROOT):
    """Python sources of the harness (package, runners, tests); results, caches and .env are never included."""
    out = []
    for d, dirs, files in os.walk(root):
        dirs[:] = sorted(x for x in dirs if not x.startswith((".", "__pycache__", "results", "audit_v07")))
        out += [os.path.join(d, f) for f in sorted(files) if f.endswith(".py")]
    return sorted(out)

def code_hash(root=ROOT):
    h = hashlib.sha256()
    for p in source_files(root):
        h.update(os.path.relpath(p, root).replace(os.sep, "/").encode()); h.update(sha256_file(p).encode())
    return h.hexdigest()[:16]

def prompt_hash():
    from .backends2d import CONVENTIONS, TOOLS, INTERACTIVE_TOOLS, LOCALIZE_TOOL, LLMBackend
    from .navtools import TOOL_TEXT
    parts = [CONVENTIONS, TOOLS, INTERACTIVE_TOOLS, LOCALIZE_TOOL, TOOL_TEXT, LLMBackend.PLAN_FORMAT, LLMBackend.PLAN_FORMAT_H]
    return hashlib.sha256("\n".join(parts).encode()).hexdigest()[:16]

def package_versions():
    out = {}
    for name in PACKAGES:
        try:
            m = importlib.import_module(name); out[name] = getattr(m, "__version__", "installed")
        except Exception: out[name] = None
    return out

def environment():
    return {"python": sys.version.split()[0], "implementation": platform.python_implementation(), "platform": platform.platform(),
            "machine": platform.machine(), "packages": package_versions()}

def run_manifest():
    return {"code_hash": code_hash(), "prompt_hash": prompt_hash(), "environment": environment()}
