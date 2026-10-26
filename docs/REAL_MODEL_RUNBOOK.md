# Real-model runs: step by step (v0.10, switches added in v0.11 and v0.12)

Nothing in this repository has been measured on a real model yet. This page is the order in which to do it. Every command runs in the
VS Code terminal from the project folder (Windows PowerShell; on Linux/macOS use `python3`). Keys go in `.env` only (`python check_env.py`
shows set/missing, never the key).

## 0. Keys and model ids
| model (paper) | provider | `--planner` | `--model` (check with step 1) | key in `.env` |
|---|---|---|---|---|
| GPT-OSS-120B | Groq | `groq` | `openai/gpt-oss-120b` | `GROQ_API_KEY` |
| GPT-OSS-20B | Groq | `groq` | `openai/gpt-oss-20b` | `GROQ_API_KEY` |
| Kimi-K2 | Groq | `groq` | `moonshotai/kimi-k2-instruct-0905` (take the exact id from `--list-models`) | `GROQ_API_KEY` |
| Gemini (closed model) | Google AI Studio | `gemini` | e.g. `gemini-2.5-flash` (take the exact id from `--list-models`) | `GEMINI_API_KEY` |

Gemini needs `python -m pip install google-genai`. The Groq client uses only the standard library.

## 1. Preflight (a handful of calls per model)
```powershell
python preflight_llm.py --planner groq --list-models
python preflight_llm.py --planner groq   --model openai/gpt-oss-120b
python preflight_llm.py --planner groq   --model openai/gpt-oss-20b
python preflight_llm.py --planner groq   --model moonshotai/kimi-k2-instruct-0905
python preflight_llm.py --planner gemini --model gemini-2.5-flash
```
It checks (1) the model id exists, (2) one JSON round trip (prints tokens, latency, `finish_reason`), (3) one easy episode through the real
prompt path with S1 and C4. Exit code 0 = the plumbing works. **Send me the full output of the first failing preflight.**
Typical fixes: `finish_reason=length` -> raise `LLM_MAX_OUTPUT_TOKENS` in `.env` or pass `--reasoning-effort low`; HTTP 429 -> set
`LLM_MIN_INTERVAL=2` (seconds between calls) in `.env`; HTTP 400 about `response_format` is handled automatically (plain-text mode).

## 2. Cost estimate (no API call)
```powershell
python run_spatial.py --planner groq --suite all --seeds 1 --limit 20 --configs C1 C2 C3 C4 S1 S5 --estimate
```
The estimate counts the oracle's model calls for the same grid: about 50 calls per episode for C1/C2 (one per tool call, many
episodes run into the step budget), 25-30 for C3/C4, 6-10 for S1-S5. A real model usually needs more (retries, replans).

## 3. Pilot: `--limit 20` (this is also the first real test of the Groq client)
```powershell
python run_spatial.py --planner groq --model openai/gpt-oss-120b --suite all --seeds 1 --limit 20 --configs C1 C2 C3 C4 S1 S5 --events --max-llm-calls 6000 --out results_groq120b_pilot
python analyze_spatial.py results_groq120b_pilot
python failure_taxonomy.py results_groq120b_pilot
```
* Results are written after every episode (`*.jsonl`). If the run stops (network, rate limit, `--max-llm-calls`), run the **same command with
  `--resume`**: finished episodes are kept, errored ones are re-run.
* Read `api_usage.csv` (tokens, latency, `finish_reason`, retries) and `REPORT.md` section 4a (repeated %, invalid %, parse errors).
* A pilot is for plumbing and prompt problems only. Its numbers are not results and are not pooled with the main run (protocol, section 6).
* Then the same pilot for `openai/gpt-oss-20b`, the Kimi-K2 id and Gemini (change `--planner/--model/--out`).

## 4. Main run (pre-registered: `docs/PROTOCOL_v0_10.md`)
```powershell
python run_spatial.py --planner groq --model openai/gpt-oss-120b --suite all --seeds 3 --per-family 20 --configs C1 C2 C3 C4 S1 S5 --events --out results_groq120b_main
python run_spatial.py --planner groq --model openai/gpt-oss-120b --suite all --seeds 3 --per-family 20 --limit 40 --repeats 2 --configs C1 C4 --out results_groq120b_repeat
python analyze_spatial.py results_groq120b_main ; python failure_taxonomy.py results_groq120b_main
```
`--per-family 20` gives exactly 20 trials per (model, configuration, family) cell, as in the paper. 12 families x 6 configurations = 1,440
episodes per model. Add `--configs S0 S2 S3 S4` later if the budget allows (S2-S5 are cheap: 6-10 calls per episode).

## 5. Follow-up experiments (same commands, one switch each)
| question | switch |
|---|---|
| does the model depend on the wording? (paired with the main run) | `--phrasing paraphrase`, `--phrasing bn` (Bengali) |
| sensor documentation ablation (grounded configs S2-S5) | `--no-sensor-doc`; interpreted view off: `--no-interp-view` |
| history window of step-wise agents (S0, C1-C4) | `--window 2`, `--window 12` |
| why does verification alone hurt (S4)? how many verifier retries help? | `--configs S3 S4 S4v1 S4v2 S4v3 S5` |
| harder instructions only | `--suite hard` |
| run-to-run variance | `--repeats 3` (same instances, same seed) |
| v0.11: does the model ask when an instruction is ambiguous, and refuse an impossible one? | default (ask/abstain tools are offered); control without them: `--no-ask`; families `ambiguous` L3/L4 and `impossible` (`--suite hard`) |
| v0.11: coverage planner as a tool vs the model writing sweeps | `--nav-tools` (adds `goto_cell`, `cover_region`) |
| v0.11: room -> regions -> tool calls | `--hierarchical` |
| v0.11: learning from failures | `--configs S1 S6` or `<config>+L` (e.g. `C4 C4+L`); lessons are written by the model, one extra call per failed episode; `--lesson-k 3` |
| v0.11: bigger rooms | `--room-scale 2` (clean_left / clean_side / goto only; new task ids `...|r2`, so pair with another `--room-scale 2` run) |
| v0.11: sensing / actuation noise | `--odom-noise 0.1 --det-drop 0.1 --img-noise 0.05 --stale-view 0.1` |
| v0.12: does the model localise when odometry drifts? | `--odom-noise 0.15 --localizer tool` (control: no `--localizer`; upper bound: `--localizer auto`) |
| v0.12: visual re-localisation by the runtime | `--odom-noise 0.15 --visual-reloc` |
| v0.12: moving obstacles | `--moving-every 5` (objects the instruction does not mention step to a free neighbouring cell) |
| v0.12: can the model read a camera image? | `--vision` with a vision model (e.g. `--planner gemini --model gemini-2.5-flash`); the object list is replaced by a rendered PNG |
| v0.12: self-consistency | `--temperature 0.7 --self-consistency 5` (k model calls per decision; vote by dry-run outcome) |
| v0.12: big model plans, small model steps | `--planner groq --model openai/gpt-oss-120b --step-planner groq --step-model openai/gpt-oss-20b --configs C3 C4` |
| v0.12.1: scenario sets A-D (basic, orientations, multiple objects, long-horizon) | `--scenario A B C D` (`docs/SCENARIOS.md`; REPORT section 6) |
| v0.13: does a short action horizon help? (closed vs open loop) | `--horizon 4` (and `--horizon 1`), paired with the main run |
| v0.13: does feedback re-planning help on top of plan-constrained execution? | `--configs C4 C5` |
| v0.13: simulator safety gate / fail-closed on stale views / per-episode time limit | `--safety-gate`, `--stale-view 0.3 --fail-closed`, `--episode-timeout 120` |
| v0.13: held-out worlds | `--heldout` (seed offset 1000; never used while tuning prompts) |
| v0.13: all options in one file | `--config configs/run_pilot_groq.json` (command-line options win) |
| v0.12: cost cap | `--token-budget 60000` (per episode; `token_budget_exhausted` column); `api_usage.csv` now has `cached_tokens` and `role` |

Compare any two runs of the same grid, paired, with Holm: `python compare_runs.py results_A results_B` (writes `COMPARE_vs_A.md` into B; lists the
settings that differ, and the paired risk difference with a 95% interval). Only one switch should differ.

## 6. What to send back
`results_*/REPORT.md`, `FAILURES.md`, `run_metadata.json`, `api_usage.csv`, and for a failure the console text. Not `.env`.
