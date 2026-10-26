"""Plan JSON schema for constrained decoding (section 4.3).

plan_json_schema(registry, strict=True) builds a schema whose steps.items is an anyOf with one variant per tool: tool is an enum of that
one name, args is that tool's own schema (types/enum/min/max, additionalProperties:false; strict makes optional args nullable)."""
from __future__ import annotations


def _arg_schema(a, strict):
    """A single tool-argument schema from a pel.registry.Arg."""
    d = {}
    if getattr(a, "doc", ""): d["description"] = a.doc
    kind = a.kind
    if kind == "int":
        d["type"] = "integer"
        if a.min is not None: d["minimum"] = a.min
        if a.max is not None: d["maximum"] = a.max
    elif kind == "intlike":
        d["type"] = "integer"
    elif kind == "enum":
        d["enum"] = list(a.values)
        d["type"] = "integer" if all(isinstance(v, int) and not isinstance(v, bool) for v in a.values) else "string"
    elif kind == "cells":
        d = {"type": "array", "items": {"type": "array", "items": {"type": "integer"}, "minItems": 2, "maxItems": 2}}
    elif kind == "text":
        d["type"] = "string"
    else:   # "any"
        d["type"] = ["number", "string", "boolean"]
    if strict and not a.required and "enum" not in d:
        # optional args nullable in strict mode (so a provider's strict json_schema accepts their absence as null)
        t = d.get("type")
        if isinstance(t, str): d["type"] = [t, "null"]
        elif isinstance(t, list) and "null" not in t: d["type"] = t + ["null"]
    return d


def tool_args_schema(spec, strict):
    props = {k: _arg_schema(a, strict) for k, a in spec.args.items()}
    required = [k for k, a in spec.args.items() if a.required]
    return {"type": "object", "properties": props, "required": required, "additionalProperties": False}


def _tool_specs(registry):
    """Accept either a dict {name: ToolSpec} (HUMANOID_TOOLS) or an object exposing .items()/values()."""
    if isinstance(registry, dict): return dict(registry)
    if hasattr(registry, "items"): return dict(registry.items())
    return {s.name: s for s in registry}


def plan_json_schema(registry, strict=True):
    specs = _tool_specs(registry)
    variants = []
    for name, spec in specs.items():
        variants.append({"type": "object", "additionalProperties": False,
                         "properties": {
                             "id": {"type": "string"},
                             "tool": {"enum": [name]},
                             "args": tool_args_schema(spec, strict),
                             "after": {"type": "array", "items": {"type": "string"}},
                             "on_fail": {"enum": ["repair", "skip", "abort"]},
                             "why": {"type": "string"}},
                         "required": ["id", "tool", "args"]})
    return {
        "type": "object", "additionalProperties": False,
        "properties": {
            "goal": {"type": "string"},
            "goal_struct": {"type": "object", "properties": {
                "verb": {"type": "string"}, "targets": {"type": "array", "items": {"type": "string"}},
                "constraints": {"type": "array", "items": {"type": "string"}},
                "negations": {"type": "array", "items": {"type": "string"}}}},
            "feasible": {"type": "boolean"},
            "refusal_reason": {"type": "string"},
            "done_when": {"type": "string"},
            "steps": {"type": "array", "maxItems": 20, "items": {"anyOf": variants}}},
        "required": ["goal", "feasible", "steps"]}


def tool_count(registry):
    return len(_tool_specs(registry))
