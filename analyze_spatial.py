"""Builds REPORT.md (+ figure) from trial_results.csv: Wilson CIs, paired exact McNemar, factor breakdowns, cost."""
import csv, json, math, os, sys, collections, statistics as st

def wilson(k, n, z=1.96):
    if n == 0: return (float("nan"),) * 2
    p = k / n; d = 1 + z * z / n; c = p + z * z / (2 * n)
    r = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)); return (c - r) / d, (c + r) / d

def mcnemar_exact(b, c):               # b: A fail/B success, c: A success/B fail
    n = b + c
    if n == 0: return 1.0
    k = min(b, c); return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)

def holm(ps):
    """Holm-Bonferroni adjusted p-values (same order as the input)."""
    order = sorted(range(len(ps)), key=lambda i: ps[i]); adj = [0.0] * len(ps); run = 0.0
    for rank, i in enumerate(order):
        run = max(run, min(1.0, (len(ps) - rank) * ps[i])); adj[i] = run
    return adj

def wilcoxon_signed_rank(diffs):
    """Two-sided paired Wilcoxon signed-rank test, normal approximation with tie correction (zeros dropped). Returns (n, W+, p)."""
    d = [x for x in diffs if x != 0]; n = len(d)
    if n == 0: return 0, 0.0, 1.0
    order = sorted(range(n), key=lambda i: abs(d[i])); ranks = [0.0] * n; i = 0; ties = 0.0
    while i < n:
        j = i
        while j + 1 < n and abs(d[order[j + 1]]) == abs(d[order[i]]): j += 1
        for k in range(i, j + 1): ranks[order[k]] = (i + j) / 2 + 1
        t = j - i + 1; ties += t ** 3 - t; i = j + 1
    wp = sum(r for r, x in zip(ranks, d) if x > 0); mu = n * (n + 1) / 4
    sd = math.sqrt(n * (n + 1) * (2 * n + 1) / 24 - ties / 48)
    if sd == 0: return n, wp, 1.0
    z = (abs(wp - mu) - 0.5) / sd
    return n, wp, min(1.0, math.erfc(max(0.0, z) / math.sqrt(2)))

def fnum(x):
    try: return float(x)
    except (TypeError, ValueError): return None

FAM_ORDER = ("motion", "goto", "clean_left", "remember_return", "clean_side", "sequence", "relative", "ambiguous", "change", "prohibit", "conditional", "report", "impossible")
CFG_DESC = {"S0": "reactive", "S1": "persistent plan", "S2": "+sensor grounding", "S3": "+memory", "S4": "+dry-run verification", "S5": "+runtime repair",
            "C1": "paper C1 prompt only", "C2": "paper C2 non-persistent plan", "C3": "paper C3 persistent scratchpad", "C4": "paper C4 plan-constrained execution", "S6": "S1 + lesson memory"}

def cfg_key(c):
    return (0 if c.startswith("S") else 1 if c.startswith("C") else 2, c)

def comparison_pairs(cfgs):
    """Adjacent steps inside each ladder, plus the paper's headline contrast C1 -> C4."""
    pairs = []
    for ladder in (["S0", "S1", "S2", "S3", "S4", "S5"], ["C1", "C2", "C3", "C4"]):    # S6 is compared with S1 (its base), below
        have = [c for c in ladder if c in cfgs]; pairs += list(zip(have, have[1:]))
    if "C1" in cfgs and "C4" in cfgs and ("C1", "C4") not in pairs: pairs.append(("C1", "C4"))
    pairs += [(c[:-2], c) for c in cfgs if c.endswith("+L") and c[:-2] in cfgs]
    if "S1" in cfgs and "S6" in cfgs: pairs.append(("S1", "S6"))
    vs = sorted(c for c in cfgs if c.startswith("S4v"))
    if vs and "S4" in cfgs: pairs += [("S4", v) for v in vs]
    return pairs

def main(d):
    rows = list(csv.DictReader(open(os.path.join(d, "trial_results.csv"), encoding="utf-8")))
    meta = json.load(open(os.path.join(d, "run_metadata.json")))
    for r in rows: r.setdefault("rep", "0"); r["rep"] = r["rep"] or "0"
    cfgs = sorted({r["config"] for r in rows}, key=cfg_key); fams = [f for f in FAM_ORDER if any(r["family"] == f for r in rows)]
    succ = {(r["task_id"], r["rep"], r["config"]): int(r["success"]) for r in rows}
    out = []; w = out.append
    synthetic = meta["planner"] == "oracle"
    w("# Spatial benchmark report\n")
    if synthetic:
        w("> **SYNTHETIC RUN – harness validation only.** The oracle backend uses a hand-written error model "
          f"(slip_p={meta['slip_p']}, half_confusion_p={meta['half_confusion_p']}) plus the architectural mechanisms. "
          "These numbers show that the pipeline works; they are NOT evidence about any LLM.\n")
    w(f"Planner/model: `{meta['model']}` · temperature {meta['temperature']} · task instances {meta['instances']} · repeats {meta.get('repeats', 1)} · "
      f"wording `{meta.get('phrasing', 'canonical')}` · run {meta['utc'][:19]}Z\n")
    if synthetic and meta.get("deviation_p") is not None:
        w(f"> Oracle C3/C4 deviation probability (assumed) = {meta['deviation_p']}. Under the oracle C1 and C2 are the same policy by construction "
          "(a hand-written solver has no 'reasoning' that a stored plan could change), so any C1/C2 difference here is noise; it can only be measured on a real model.\n")
    w("Configs: " + " · ".join(f"{c} {CFG_DESC.get(c, '')}" for c in cfgs) + "\n")
    if all(c in cfgs for c in ("C1", "C3", "C4")):
        w("## 0. Pre-registered primary tests (docs/PROTOCOL_v0_10.md), pooled over all families, Holm over these three\n")
        keys = sorted({(r["task_id"], r["rep"]) for r in rows})
        def mc(a, b):
            t = [k for k in keys if (*k, a) in succ and (*k, b) in succ]
            ao = sum(succ[(*k, a)] and not succ[(*k, b)] for k in t); bo = sum(succ[(*k, b)] and not succ[(*k, a)] for k in t)
            pa = 100 * sum(succ[(*k, a)] for k in t) / max(1, len(t)); pb = 100 * sum(succ[(*k, b)] for k in t) / max(1, len(t))
            return len(t), pa, pb, ao, bo, mcnemar_exact(bo, ao)
        rp = {(r["task_id"], r["rep"], r["config"]): fnum(r.get("repeated_call_pct")) for r in rows}
        pairs = [k for k in keys if rp.get((*k, "C1")) is not None and rp.get((*k, "C4")) is not None]
        n3, wp, p3 = wilcoxon_signed_rank([rp[(*k, "C4")] - rp[(*k, "C1")] for k in pairs])
        r1, r4 = (st.mean([rp[(*k, c)] for k in pairs]) if pairs else float("nan") for c in ("C1", "C4"))
        t1, t2 = mc("C1", "C4"), mc("C3", "C4")
        adj = holm([t1[5], t2[5], p3])
        w("| hypothesis | n | A | B | A only ok / B only ok | test | p | p (Holm) |\n|---|---|---|---|---|---|---|---|")
        w(f"| H1 success C4 > C1 | {t1[0]} | {t1[1]:.1f}% | {t1[2]:.1f}% | {t1[3]} / {t1[4]} | exact McNemar | {t1[5]:.3g} | {adj[0]:.3g} |")
        w(f"| H2 success C4 > C3 | {t2[0]} | {t2[1]:.1f}% | {t2[2]:.1f}% | {t2[3]} / {t2[4]} | exact McNemar | {t2[5]:.3g} | {adj[1]:.3g} |")
        w(f"| H3 repeated% C4 < C1 | {len(pairs)} | {r1:.1f}% | {r4:.1f}% | {n3} non-zero pairs | Wilcoxon signed-rank | {p3:.3g} | {adj[2]:.3g} |")
        w("\nTwo-sided p-values; check the direction in columns A and B (A = first config named).\n")
    w("## 1. Success rate by task family (k/n, 95% Wilson CI)\n")
    w("| family | " + " | ".join(cfgs) + " |\n|---|" + "---|" * len(cfgs))
    for f in fams:
        cells = []
        for c in cfgs:
            rr = [int(r["success"]) for r in rows if r["family"] == f and r["config"] == c]
            lo, hi = wilson(sum(rr), len(rr)); cells.append(f"{sum(rr)}/{len(rr)} ({100*sum(rr)/len(rr):.0f}%) [{100*lo:.0f}–{100*hi:.0f}]")
        w(f"| {f} | " + " | ".join(cells) + " |")
    w("\n## 2. Paired comparisons (same worlds, exact McNemar, Holm-adjusted over this whole table)\n")
    w("Pairs: adjacent steps of the S ladder and of the paper's C ladder, plus C1 → C4.\n")
    w("| family | A → B | A only ok | B only ok | p (two-sided) | p (Holm) |\n|---|---|---|---|---|---|")
    tids = sorted({(r["task_id"], r["rep"]) for r in rows}); cmp_rows = []
    for f in fams:
        for a, b in comparison_pairs(cfgs):
            t = [x for x in tids if x[0].startswith(f + "|") and (*x, a) in succ and (*x, b) in succ]
            ao = sum(succ[(*x, a)] and not succ[(*x, b)] for x in t); bo = sum(succ[(*x, b)] and not succ[(*x, a)] for x in t)
            cmp_rows.append((f, a, b, ao, bo, mcnemar_exact(bo, ao)))
    adj = holm([c[5] for c in cmp_rows])
    for (f, a, b, ao, bo, pv), pa in zip(cmp_rows, adj): w(f"| {f} | {a} → {b} | {ao} | {bo} | {pv:.3g} | {pa:.3g} |")
    w("")
    for fac, title in (("yaw", "initial orientation (deg)"), ("mirrored", "mirrored camera"), ("n_objects", "number of objects"), ("level", "instruction level within the family")):
        w(f"## 3. Success by {title}\n")
        vals = sorted({r[fac] for r in rows}, key=lambda v: int(v))
        if fac == "level" and len(vals) < 2: continue
        w(f"| family | {fac} | " + " | ".join(cfgs) + " |\n|---|---|" + "---|" * len(cfgs))
        for f in fams:
            for v in vals:
                rs = [r for r in rows if r["family"] == f and r[fac] == v]
                if not rs: continue
                if fac == "level" and len({r[fac] for r in rows if r["family"] == f}) < 2: continue
                w(f"| {f} | {v} | " + " | ".join(
                    f"{100*sum(int(r['success']) for r in rs if r['config']==c)/max(1,sum(1 for r in rs if r['config']==c)):.0f}%" for c in cfgs) + " |")
        w("")
    w("## 4a. Paper metrics (pooled over all episodes of a config)\n")
    w("repeated % = repeated tool calls / executed tool calls (repeated: same tool and arguments as the previous call, from the same robot state); "
      "invalid % = invalid tool calls / executed tool calls; gate-rejected % (C4) = proposals the plan gate refused / proposals; "
      "deviation % (C3) = calls that left the stored plan / proposals; execution time = wall clock per episode (model latency + harness).\n")
    w("| config | episodes | success % | repeated % | invalid % | gate-rejected % | deviation % | parse errors | exec time s (mean) | model latency s (mean) |\n|---|---|---|---|---|---|---|---|---|---|")
    def tot(rs, col): return sum(fnum(r.get(col)) or 0 for r in rs)
    def pct(num, den): return f"{100 * num / den:.1f}" if den else "–"
    for c in cfgs:
        rs = [r for r in rows if r["config"] == c]; ex = tot(rs, "actions_executed"); pr = tot(rs, "proposals")
        et = [fnum(r.get("elapsed_s")) for r in rs if fnum(r.get("elapsed_s")) is not None]
        lt = [fnum(r.get("llm_latency_s")) for r in rs if fnum(r.get("llm_latency_s")) is not None]
        w(f"| {c} | {len(rs)} | {pct(sum(int(r['success']) for r in rs), len(rs))} | {pct(tot(rs, 'repeated_calls'), ex)} | {pct(tot(rs, 'invalid_calls'), ex)} | "
          f"{pct(tot(rs, 'gate_rejections'), pr)} | {pct(tot(rs, 'plan_deviations'), pr)} | {int(tot(rs, 'parse_errors'))} | "
          f"{f'{st.mean(et):.2f}' if et else '–'} | {f'{st.mean(lt):.2f}' if lt else '–'} |")
    w("\n## 4b. Cost per success\n")
    w("| config | successes | model calls / success | tokens (in+out) / success | exec time s / success |\n|---|---|---|---|---|")
    for c in cfgs:
        rs = [r for r in rows if r["config"] == c]; k = sum(int(r["success"]) for r in rs)
        toks = tot(rs, "input_tokens") + tot(rs, "output_tokens")
        w(f"| {c} | {k} | {tot(rs, 'llm_calls') / k:.1f} | {(f'{toks / k:.0f}') if toks else '–'} | {tot(rs, 'elapsed_s') / k:.2f} |" if k else f"| {c} | 0 | – | – | – |")
    if any(r["family"] == "ambiguous" for r in rows):
        w("\n## 4c. Ambiguous references: which reading the robot chose (level 3 accepts either; levels 1/2 name the frame)\n")
        w("| config | level | robot frame | door frame | neither |\n|---|---|---|---|---|")
        for c in cfgs:
            for lv in sorted({r["level"] for r in rows if r["family"] == "ambiguous"}):
                rs = [r for r in rows if r["config"] == c and r["family"] == "ambiguous" and r["level"] == lv]
                cnt = collections.Counter(r.get("interpretation") or "none" for r in rs)
                w(f"| {c} | {lv} | {cnt['robot_frame']} | {cnt['door_frame']} | {cnt['none']} |")
    if any("safety_violations" in r for r in rows):
        w("\n## 4e. Safety and clarification (v0.11)\n")
        w("safe success = success with no wall bump, no object hit and no forbidden cell; ask rate = questions asked in instructions that "
          "needed one / such instructions; unnecessary asks = questions where nothing was ambiguous; abstention is correct on the `impossible` family "
          "and a false abstention everywhere else.\n")
        w("| config | success % | safe success % | violations / episode | wall bumps | object hits | ask rate when needed | unnecessary asks | correct abstentions | false abstentions |")
        w("|---|---|---|---|---|---|---|---|---|---|")
        for c in cfgs:
            rs = [r for r in rows if r["config"] == c]; n = max(1, len(rs))
            need = tot(rs, "ask_needed_phases"); imp = [r for r in rs if r["family"] == "impossible"]
            w(f"| {c} | {pct(sum(int(r['success']) for r in rs), n)} | {pct(tot(rs, 'safe_success'), n)} | {tot(rs, 'safety_violations') / n:.2f} | "
              f"{int(tot(rs, 'wall_bumps'))} | {int(tot(rs, 'object_hits'))} | {pct(sum(1 for r in rs if (fnum(r.get('ask_needed_phases')) or 0) and (fnum(r.get('asks_when_needed')) or 0)), need)} | "
              f"{int(tot(rs, 'unnecessary_asks'))} | {int(sum(fnum(r.get('abstained')) or 0 for r in imp))}/{len(imp)} | "
              f"{int(sum(fnum(r.get('abstained')) or 0 for r in rs if r['family'] != 'impossible'))} |")
    lcfgs = [c for c in cfgs if c == "S6" or c.endswith("+L")]
    if lcfgs:
        w("\n## 4f. Lesson memory: does it learn over the run? (episodes in run order, split into halves)\n")
        w("| config | base | lessons written | 1st half: config / base | 2nd half: config / base | gain of the 2nd half over the base |\n|---|---|---|---|---|---|")
        for c in lcfgs:
            base = "S1" if c == "S6" else c[:-2]
            rs = [r for r in rows if r["config"] == c]; h = len(rs) // 2
            bmap = {(r["task_id"], r["rep"]): int(r["success"]) for r in rows if r["config"] == base}
            def half(part):
                ks = [(r["task_id"], r["rep"]) for r in part if (r["task_id"], r["rep"]) in bmap]
                return (100 * sum(int(r["success"]) for r in part) / max(1, len(part)), 100 * sum(bmap[k] for k in ks) / max(1, len(ks)) if ks else float("nan"))
            (a1, b1), (a2, b2) = half(rs[:h]), half(rs[h:])
            nl = sum(1 for r in rs if str(r.get("lesson_written", "")) not in ("", "None"))
            w(f"| {c} | {base} | {nl} | {a1:.1f}% / {b1:.1f}% | {a2:.1f}% / {b2:.1f}% | {a2 - b2:+.1f} pp |")
        w("\nWith the oracle, lessons can only remove the frame confusion and halve the slip rate (assumed effects, `orbit_ros/lessons.py`).\n")
    rz = meta.get("realism") or {}
    if any(rz.values()) or (meta.get("room_scale") or 1) != 1 or meta.get("localizer", "none") != "none" or meta.get("visual_reloc"):
        w("\n## 4g. Realism and localisation settings and what they did (v0.11-v0.12)\n")
        w("Settings: " + ", ".join(f"{k}={v}" for k, v in rz.items()) + f", room_scale={meta.get('room_scale', 1)}, "
          f"localizer={meta.get('localizer', 'none')}, visual_reloc={meta.get('visual_reloc', False)}. "
          "odom = the wheels over/under-run by 1 m while the reported pose assumes the commanded distance.\n")
        if any(rz.values()):
            w("| config | success | odometry errors | final pose error (m) | localise calls | visual corrections | stale views | dropped detections | object moves |\n|---|---|---|---|---|---|---|---|---|")
            for c in cfgs:
                rs = [r for r in rows if r["config"] == c]
                m = lambda col: sum(fnum(r.get(col)) or 0 for r in rs) / max(1, len(rs))
                w(f"| {c} | {100 * m('success'):.1f}% | {m('odom_errors'):.2f} | {m('final_pose_error_m'):.2f} | {m('localize_calls'):.2f} | {m('visual_corrections'):.2f} | {m('stale_views'):.2f} | {m('dropped_detections'):.2f} | {m('object_moves'):.2f} |")
    w("\n## 4. Cost and behaviour (means per episode)\n")
    cols = ["progress", "llm_calls", "env_tool_calls", "actions_executed", "collisions", "invalid_calls", "repeated_calls", "gate_rejections", "plan_deviations",
            "repairs", "verify_failures", "replans", "input_tokens", "output_tokens", "elapsed_s", "llm_latency_s"]
    cols = [c for c in cols if any(c in r for r in rows)]
    w("| config | " + " | ".join(cols) + " |\n|---|" + "---|" * len(cols))
    for c in cfgs:
        rs = [r for r in rows if r["config"] == c]
        cells = []
        for col in cols:
            v = [fnum(r.get(col)) for r in rs if fnum(r.get(col)) is not None]
            cells.append(f"{st.mean(v):.1f}" if v else "–")
        w(f"| {c} | " + " | ".join(cells) + " |")
    w("\n`progress` is filled for the `sequence` family only (fraction of its instructions that were completed in order), so its mean covers those episodes only.\n")
    w("## 4d. How the failed episodes failed (coarse, from the trial rows; `failure_taxonomy.py` gives the finer classes)\n")
    w("| config | failed | aborted by the verifier (no repair available) | tool-call budget exhausted | ended in a wrong state |\n|---|---|---|---|---|")
    for c in cfgs:
        fr = [r for r in rows if r["config"] == c and int(r["success"]) == 0]
        ab = sum(1 for r in fr if (fnum(r.get("aborted")) or 0) > 0)
        bu = sum(1 for r in fr if not (fnum(r.get("aborted")) or 0) > 0 and (fnum(r.get("env_tool_calls")) or 0) >= (fnum(r.get("tool_call_cap")) or 300) - 10)
        w(f"| {c} | {len(fr)} | {ab} | {bu} | {len(fr) - ab - bu} |")
    errs = [r for r in rows if r.get("error")]
    w(f"\nEpisodes with runtime errors (counted as failures): {len(errs)}\n")
    if any(r.get("outcome_code") for r in rows):
        from orbit_ros.agent2d import OUTCOME_CODES
        codes = [k for k in OUTCOME_CODES if any(r.get("outcome_code") == k for r in rows)]
        w("## 4h. Outcome codes (one code per episode, first match in `docs/OUTCOME_CODES.md` order)\n")
        w("| config | " + " | ".join(codes) + " |\n|---|" + "---|" * len(codes))
        for c in cfgs:
            rs = [r for r in rows if r["config"] == c]
            w(f"| {c} | " + " | ".join(str(sum(1 for r in rs if r.get("outcome_code") == k)) for k in codes) + " |")
        cl = ("horizon_replans", "feedback_replans", "safety_rejections", "backtracks", "stale_retries")
        if any(r.get(k) not in (None, "") for r in rows for k in cl):
            w("\n**Closed-loop and safety counters** (means per episode)\n")
            w("| config | " + " | ".join(cl) + " |\n|---|" + "---|" * len(cl))
            for c in cfgs:
                rs = [r for r in rows if r["config"] == c]
                w(f"| {c} | " + " | ".join(f"{sum(fnum(r.get(k)) or 0 for r in rs) / len(rs):.2f}" for k in cl) + " |")
        w("")
    if any(r.get("scenarios") for r in rows):
        from orbit_ros.scenario_sets import SCENARIOS
        w("## 6. Scenarios A-D (an instance may belong to several sets; A is the heading-0 part of B's goto)\n")
        def cell(rr):
            if not rr: return "–"
            lo, hi = wilson(sum(rr), len(rr)); return f"{100*sum(rr)/len(rr):.0f}% [{100*lo:.0f}–{100*hi:.0f}] n={len(rr)}"
        names = [k for k in SCENARIOS if any(k in (r.get("scenarios") or "").split() for r in rows)]
        insc = lambda r, k: k in (r.get("scenarios") or "").split()
        w("| scenario | " + " | ".join(cfgs) + " |\n|---|" + "---|" * len(cfgs))
        for k in names:
            w(f"| {k} {SCENARIOS[k]['title']} | " + " | ".join(cell([int(r["success"]) for r in rows if insc(r, k) and r["config"] == c]) for c in cfgs) + " |")
        if "B" in names:
            w("\n**B by start heading** (success %; same instructions, only the heading differs)\n")
            w("| heading | " + " | ".join(cfgs) + " |\n|---|" + "---|" * len(cfgs))
            for y in ("0", "90", "180", "270"):
                cells = []
                for c in cfgs:
                    rr = [int(r["success"]) for r in rows if insc(r, "B") and r["config"] == c and str(r["yaw"]) == y]
                    cells.append(f"{100*sum(rr)/len(rr):.0f}%" if rr else "–")
                w(f"| {y} | " + " | ".join(cells) + " |")
        if "C" in names:
            w("\n**C by task** (sequence: the order is part of success; relative: the right object out of 3-5)\n")
            w("| task | " + " | ".join(cfgs) + " |\n|---|" + "---|" * len(cfgs))
            for fam, lv in sorted({(r["family"], r["level"]) for r in rows if insc(r, "C")}):
                cells = []
                for c in cfgs:
                    rr = [int(r["success"]) for r in rows if insc(r, "C") and r["config"] == c and r["family"] == fam and r["level"] == lv]
                    cells.append(f"{100*sum(rr)/len(rr):.0f}%" if rr else "–")
                w(f"| {fam} L{lv} | " + " | ".join(cells) + " |")
        if "D" in names:
            w("\n**D: recovery** (motor faults are injected at the 2nd and 5th move; obstacles the instruction does not mention move every 6 calls)\n")
            w("| config | success | success after >= 1 motor fault | faults hit / ep | replans + repairs / ep | object moves / ep | stages done (sequence) |\n|---|---|---|---|---|---|---|")
            for c in cfgs:
                rs = [r for r in rows if insc(r, "D") and r["config"] == c]
                if not rs: continue
                m = lambda col: sum(fnum(r.get(col)) or 0 for r in rs) / len(rs)
                hit = [int(r["success"]) for r in rs if (fnum(r.get("injected_failures")) or 0) > 0]
                prog = [fnum(r.get("progress")) for r in rs if r["family"] == "sequence" and fnum(r.get("progress")) is not None]
                w(f"| {c} | {100*m('success'):.0f}% | " + (f"{100*sum(hit)/len(hit):.0f}% (n={len(hit)})" if hit else "–") +
                  f" | {m('injected_failures'):.2f} | {m('replans') + m('repairs'):.2f} | {m('object_moves'):.1f} | " + (f"{100*sum(prog)/len(prog):.0f}%" if prog else "–") + " |")
        w("")
    w("## 5. Interpretation (fill in after a REAL LLM run)\n\n- What improved with each added component, and for which families/factors?\n"
      "- Which failures remain (read `event_log.csv`; classify: sensor misreading, frame confusion, instruction drift, forgetting, arithmetic slip)?\n"
      "- Cost trade-off: extra LLM calls/tokens/latency per success.\n- Threats to validity: simulator is a 2-D grid, perfect odometry, simplified camera; small n; single model.\n")
    open(os.path.join(d, "REPORT.md"), "w", encoding="utf-8").write("\n".join(out))
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(max(8, 1.1 * len(fams)), 4.2)); wd = 0.8 / len(cfgs)
        for j, c in enumerate(cfgs):
            ys, lo, hi = [], [], []
            for f in fams:
                rr = [int(r["success"]) for r in rows if r["family"] == f and r["config"] == c]
                p = sum(rr) / len(rr); l, h = wilson(sum(rr), len(rr)); ys.append(100 * p); lo.append(100 * (p - l)); hi.append(100 * (h - p))
            ax.bar([i + j * wd for i in range(len(fams))], ys, wd, yerr=[lo, hi], label=c, capsize=2)
        ax.set_xticks([i + 0.4 - wd / 2 for i in range(len(fams))]); ax.set_xticklabels(fams, rotation=30, ha="right", fontsize=7); ax.set_ylabel("success %"); ax.set_ylim(0, 105)
        ax.legend(ncol=min(len(cfgs), 10), fontsize=7); ax.set_title("Success by family and architecture" + (" (SYNTHETIC)" if synthetic else ""))
        fig.tight_layout(); fig.savefig(os.path.join(d, "success_by_family.png"), dpi=150)
    except ImportError: pass
    print("wrote", os.path.join(d, "REPORT.md"))
if __name__ == "__main__": main(sys.argv[1])
