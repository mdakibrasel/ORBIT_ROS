"""Build a traceable inventory of test evidence without changing source files."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path


REPORT_EXTENSIONS = {
    ".csv", ".html", ".htm", ".json", ".jsonl", ".log", ".md", ".out",
    ".png", ".txt", ".tsv", ".xml",
}
EVIDENCE_ROOTS = (
    "docs",
    "results_live",
    "results_mcp",
    "results_nao_benchmark",
    "results_spatial_oracle",
)
ROOT_REPORT_NAMES = re.compile(
    r"(?i)(report|result|evidence|benchmark|coverage|test|(?:^|[_-])logs?(?:[_-]|$))"
)
STATUS_PATTERNS = (
    ("PASS", re.compile(r"(?i)(?<![A-Z])PASS(?:ED)?(?![A-Z])|(?:^|[\s,])OK(?:$|[\s,])")),
    ("FAIL", re.compile(r"(?i)(?<![A-Z])FAIL(?:ED|URE)?(?![A-Z])")),
    ("SKIP", re.compile(r"(?i)(?<![A-Z])SKIP(?:PED)?(?![A-Z])")),
    ("ERROR", re.compile(r"(?i)(?<![A-Z])ERROR(?:S)?(?![A-Z])")),
    ("INCOMPLETE", re.compile(r"(?i)\bINCOMPLETE\b")),
    ("NOT RUN", re.compile(r"(?i)\bNOT RUN\b")),
    ("UNVERIFIED", re.compile(r"(?i)\bUNVERIFIED\b")),
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def text_preview(path: Path) -> str:
    if path.suffix.lower() not in {".csv", ".html", ".htm", ".json", ".jsonl", ".log", ".md", ".out", ".tsv", ".txt", ".xml"}:
        return ""
    try:
        return path.read_text(encoding="utf-8", errors="replace")[:200_000]
    except OSError:
        return ""


def status_for(text: str) -> tuple[str, str]:
    explicit = re.findall(r"(?im)^\s*(?:result|status)\s*[:=]\s*([A-Z][A-Z ]+)", text)
    for value in explicit:
        value = value.strip()
        if value in {item[0] for item in STATUS_PATTERNS}:
            return value, "explicit status field"
    hits = [label for label, pattern in STATUS_PATTERNS if pattern.search(text)]
    if len(hits) == 1:
        return hits[0], "single unambiguous status marker"
    if len(hits) > 1:
        return "UNVERIFIED", "multiple status markers; source contains mixed outcomes"
    return "UNVERIFIED", "no reliable result marker found"


def category_for(path: Path, text: str) -> tuple[str, str]:
    path_value = str(path).lower().replace("\\", "/")
    if path_value.startswith("results_live/"):
        return "webots_live", "source is under results_live"
    if path_value.startswith("results_mcp/"):
        return "mcp", "source is under results_mcp"
    if path_value.startswith("results_nao_benchmark/") or path_value.startswith("results_spatial_oracle/"):
        return "performance", "source is under a benchmark/oracle results root"
    value = path_value + "\n" + text[:20_000].lower()
    rules = (
        ("performance", ("benchmark", "timing", "latency")),
        ("connection_lifecycle", ("reconnect", "connection", "startup", "keep-alive", "listener")),
        ("mcp", ("mcp", "json-rpc", "jsonrpc")),
        ("regression", ("regression", "release_evidence", "cleanup_verification")),
        ("self_test", ("self-test", "selftest", "conformance")),
        ("integration", ("integration", "e2e", "end-to-end")),
        ("system", ("system", "reliability", "maintenance")),
        ("unit", ("unit test", "unittest", "pytest")),
    )
    for category, terms in rules:
        if any(term in value for term in terms):
            return category, f"matched {category} evidence terms"
    return "UNCLASSIFIED", "no reliable category marker found"


def framework_for(text: str, path: Path) -> str:
    value = (str(path) + "\n" + text[:30_000]).lower()
    frameworks = (
        ("unittest", "unittest"),
        ("pytest", "pytest"),
        ("powershell", "powershell"),
        ("webots", "webots"),
        ("mcp", "mcp/json-rpc"),
        ("csv", "csv artifact"),
    )
    found = [name for needle, name in frameworks if needle in value]
    return ", ".join(dict.fromkeys(found)) if found else "unknown"


def command_for(text: str) -> str | None:
    patterns = (
        r"(?im)^\s*`((?:python|pytest|powershell|pwsh|\.venv[\\/][^`\r\n]+)[^`\r\n]*)`",
        r"(?im)^\s*(?:(?:command|test command|run))\s*[:=]\s*([^\r\n]+)",
    )
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return match.group(1).strip().rstrip("`")
    return None


def date_for(path: Path, text: str) -> str | None:
    candidates = re.findall(r"20\d{2}(?:[-_]?\d{2}){2}(?:T?\d{2}(?:\d{2}(?:\d{2})?)?Z?)?", str(path) + "\n" + text[:5000])
    for candidate in candidates:
        digits = re.sub(r"\D", "", candidate)
        if len(digits) >= 8:
            try:
                return datetime.strptime(digits[:8], "%Y%m%d").date().isoformat()
            except ValueError:
                pass
    return None


def counts_for(text: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    patterns = {
        "tests": r"(?i)\b(?:ran|tests?)\s+(\d+)",
        "passed": r"(?i)\b(\d+)\s+passed\b",
        "failed": r"(?i)\b(\d+)\s+failed\b",
        "skipped": r"(?i)\b(\d+)\s+skipped\b",
        "errors": r"(?i)\b(\d+)\s+errors?\b",
    }
    for name, pattern in patterns.items():
        match = re.search(pattern, text)
        if match:
            counts[name] = int(match.group(1))
    exit_match = re.search(r"(?i)\bexit(?: code)?\s*[:=]?\s*(-?\d+)\b", text)
    if exit_match:
        counts["exit_code"] = int(exit_match.group(1))
    return counts


def discover(root: Path, output_names: set[str]) -> tuple[list[Path], list[dict[str, str]]]:
    files: list[Path] = []
    exclusions: list[dict[str, str]] = []
    for relative_root in EVIDENCE_ROOTS:
        base = root / relative_root
        if not base.exists():
            exclusions.append({"path": relative_root, "reason": "evidence root does not exist"})
            continue
        for path in base.rglob("*"):
            if not path.is_file():
                continue
            relative = path.relative_to(root).as_posix()
            if relative in output_names:
                exclusions.append({"path": relative, "reason": "generated master index"})
                continue
            files.append(path)
    for path in root.iterdir():
        if path.is_file() and ROOT_REPORT_NAMES.search(path.name):
            files.append(path)
    return sorted(set(files)), exclusions


def record_for(root: Path, path: Path, duplicate_group: str | None, canonical_path: str | None) -> dict:
    relative = path.relative_to(root).as_posix()
    text = text_preview(path)
    status, status_basis = status_for(text)
    category, category_basis = category_for(path.relative_to(root), text)
    suffix = path.suffix.lower()
    notes = []
    if suffix in {".db", ".sqlite", ".sqlite3"}:
        notes.append("supporting database artifact; result semantics are not inferred")
    if suffix == ".png":
        notes.append("screenshot/image evidence; visual result was not inferred")
    if not text and suffix not in {".png"}:
        status = "UNVERIFIED"
        status_basis = "binary or unreadable format"
    return {
        "test_id": "TR-" + hashlib.sha1(relative.encode("utf-8")).hexdigest()[:12].upper(),
        "original_test_name": path.stem,
        "standardized_category": category,
        "original_filename": path.name,
        "source_path": relative,
        "source_format": suffix[1:] if suffix else "no_extension",
        "test_command": command_for(text),
        "testing_framework": framework_for(text, path),
        "execution_date": date_for(path, text),
        "file_modified_at": datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat(),
        "result": status,
        "result_basis": status_basis,
        "verification_status": "VERIFIED" if status != "UNVERIFIED" else "UNVERIFIED",
        "counts": counts_for(text),
        "evidence_sha256": sha256(path),
        "duplicate_group": duplicate_group,
        "canonical_source_path": canonical_path,
        "run_period": "cleanup_or_release" if re.search(r"(?i)(cleanup|release|maintenance|2026102[5-6])", relative) else "historical",
        "limitations_and_notes": notes,
        "classification_basis": category_basis,
    }


def write_markdown(index: dict, destination: Path) -> None:
    counts = Counter(item["standardized_category"] for item in index["records"])
    results = Counter(item["result"] for item in index["records"])
    lines = [
        "# ORBIT ROS test evidence index",
        "",
        "Generated by `tools/normalize_test_reports.py`. Original evidence files are unchanged.",
        "",
        f"- Generated at: `{index['generated_at']}`",
        f"- Discovered files: **{index['validation']['discovered_files']}**",
        f"- Indexed records: **{index['validation']['indexed_files']}**",
        f"- Duplicate files: **{index['validation']['duplicate_files']}**",
        f"- Unverified records: **{index['validation']['unverified_files']}**",
        f"- Unclassified records: **{index['validation']['unclassified_files']}**",
        "",
        "## Result totals",
        "",
    ]
    for key, value in sorted(results.items()):
        lines.append(f"- `{key}`: {value}")
    lines += ["", "## Category totals", ""]
    for key, value in sorted(counts.items()):
        lines.append(f"- `{key}`: {value}")
    lines += [
        "",
        "## Indexed evidence",
        "",
        "| Test ID | Category | Result | Original name | Source path | Format | SHA-256 |",
        "|---|---|---|---|---|---|---|",
    ]
    for item in index["records"]:
        lines.append(
            f"| `{item['test_id']}` | `{item['standardized_category']}` | `{item['result']}` | "
            f"`{item['original_test_name']}` | `{item['source_path']}` | `{item['source_format']}` | "
            f"`{item['evidence_sha256'][:16]}…` |"
        )
    lines += ["", "## Exclusions and validation", ""]
    for exclusion in index["exclusions"]:
        lines.append(f"- `{exclusion['path']}` — {exclusion['reason']}")
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    output_dir = (args.output_dir or root / "docs").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    output_names = {
        "docs/TEST_REPORT_INDEX.json",
        "docs/TEST_REPORT_INDEX.md",
    }
    paths, exclusions = discover(root, output_names)
    hashes: dict[str, list[str]] = defaultdict(list)
    for path in paths:
        hashes[sha256(path)].append(path.relative_to(root).as_posix())
    duplicate_groups = {
        digest: f"DUP-{index:04d}"
        for index, (digest, members) in enumerate(sorted(hashes.items()))
        if len(members) > 1
    }
    records = [
        record_for(
            root,
            path,
            duplicate_groups.get(sha256(path)),
            min(hashes[sha256(path)]) if len(hashes[sha256(path)]) > 1 else None,
        )
        for path in paths
    ]
    duplicate_files = sum(max(0, len(members) - 1) for members in hashes.values())
    index = {
        "schema_version": "1.0",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "source_inventory": {
            "evidence_roots": list(EVIDENCE_ROOTS),
            "root_filename_pattern": ROOT_REPORT_NAMES.pattern,
            "original_files_unchanged": True,
        },
        "records": records,
        "exclusions": exclusions + [
            {"path": ".git/.venv/__pycache__", "reason": "repository metadata, environment, or generated cache; not test evidence"},
            {"path": "tests/*.py", "reason": "test sources are implementation inputs, not executed evidence reports"},
        ],
        "validation": {
            "discovered_files": len(paths),
            "indexed_files": len(records),
            "verified_files": sum(item["result"] in {"PASS", "FAIL", "SKIP", "ERROR"} for item in records),
            "unverified_files": sum(item["result"] == "UNVERIFIED" for item in records),
            "unclassified_files": sum(item["standardized_category"] == "UNCLASSIFIED" for item in records),
            "duplicate_files": duplicate_files,
            "duplicate_groups": len(duplicate_groups),
            "excluded_entries": len(exclusions) + 2,
            "inventory_matches_index": len(paths) == len(records),
        },
    }
    json_path = output_dir / "TEST_REPORT_INDEX.json"
    md_path = output_dir / "TEST_REPORT_INDEX.md"
    json_path.write_text(json.dumps(index, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_markdown(index, md_path)
    print(json.dumps(index["validation"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
