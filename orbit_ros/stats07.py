"""Paired statistics for the v0.7 evaluation: cluster bootstrap (episodes of one task cell share a world) and Holm correction."""
from __future__ import annotations
import numpy as np

def holm(pvals: list[float]) -> list[float]:
    """Holm-Bonferroni adjusted p-values (monotone, capped at 1), returned in the input order."""
    m = len(pvals); order = sorted(range(m), key=lambda i: pvals[i]); adj = [0.0] * m; run = 0.0
    for rank, i in enumerate(order):
        run = max(run, min(1.0, (m - rank) * pvals[i])); adj[i] = run
    return adj

def cluster_bootstrap_diff(a, b, clusters, B: int = 4000, seed: int = 0):
    """Paired difference mean(a-b) with a cluster bootstrap. Returns (diff, ci_lo, ci_hi, two-sided p).
    p = 2*min(P*(boot<=0), P*(boot>=0)), floored at 1/B; it is a bootstrap sign p-value, not an exact test."""
    d = np.asarray(a, float) - np.asarray(b, float)
    labels, inv = np.unique(np.asarray(clusters, dtype=object).astype(str), return_inverse=True)
    sums, cnt = np.bincount(inv, weights=d), np.bincount(inv).astype(float)
    rng = np.random.default_rng(seed); k = len(labels)
    idx = rng.integers(0, k, size=(B, k))
    boot = sums[idx].sum(axis=1) / cnt[idx].sum(axis=1)
    diff = float(d.mean())
    p = max(2 * min((boot <= 0).mean(), (boot >= 0).mean()), 1.0 / B)
    return diff, float(np.quantile(boot, 0.025)), float(np.quantile(boot, 0.975)), float(min(1.0, p))
