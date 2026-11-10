"""Qualification: run a partner's acceptance tests and decide PASS / FAIL against their thresholds.

Every listed condition is run on every site, `missions_per_site` times (missions cycled, a new seed
each run). The checks map one-to-one onto the partner's `acceptance:` block.
"""
from multiprocessing import Pool
import numpy as np

from . import maps
from .config import lint, errors
from .sim import run_mission


def _job(args):
    cfg, site, i, cond, seed = args
    return run_mission(cfg, site, i, cond, seed)


def plan_runs(cfg, seed0=1000):
    runs = []
    n = int(cfg["acceptance"]["missions_per_site"])
    for site in cfg["sites"]:
        for ci, cond in enumerate(cfg["conditions"]):
            for i in range(n):
                runs.append((cfg, site, i, cond, seed0 + 100 * ci + i))
    return runs


def evaluate(cfg, results):
    acc = cfg["acceptance"]
    checks = []

    def add(cid, name, ok, measured, limit):
        checks.append({"id": cid, "name": name, "pass": bool(ok), "measured": measured, "limit": limit})

    nav = [r for r in results if r["condition"] in ("nominal", "low_friction", "glass_walls")]
    rate = np.mean([r["success"] for r in results]) if results else 0.0
    add("Q1", "Mission success rate (all conditions)", rate >= acc["mission_success_rate"],
        f"{rate:.0%} ({sum(r['success'] for r in results)}/{len(results)})", f">= {acc['mission_success_rate']:.0%}")
    p95 = max((r["pos_err_p95"] for r in nav if r.get("pos_err_p95") is not None), default=None)
    add("Q2", "Localization error, worst run p95", p95 is not None and p95 <= acc["position_error_p95"],
        f"{p95:.3f} m" if p95 is not None else "n/a", f"<= {acc['position_error_p95']} m")
    clr = min((r["min_clearance"] for r in results if "min_clearance" in r), default=None)
    coll = sum(r["collision"] for r in results)
    add("Q3", "Minimum clearance to walls and obstacles", clr is not None and clr >= acc["min_clearance"] and coll == 0,
        f"{clr:.3f} m, {coll} contacts" if clr is not None else "n/a", f">= {acc['min_clearance']} m, 0 contacts")
    pop = [r for r in results if r["condition"] == "popup_obstacle"]
    if pop:
        gaps = [r.get("popup_gap") for r in pop]
        ok = all(g is not None for g in gaps) and min(gaps) >= acc.get("popup_min_gap", 0.2) and all(r["success"] for r in pop)
        add("Q4", "Person steps into the path: stop gap, then resume", ok,
            f"min gap {min(g for g in gaps if g is not None):.2f} m, {sum(r['success'] for r in pop)}/{len(pop)} resumed"
            if any(g is not None for g in gaps) else "no stop recorded", f">= {acc.get('popup_min_gap', 0.2)} m")
    es = [r for r in results if r["condition"] == "estop"]
    if es:
        d = [r.get("estop_stop_distance") for r in es]
        ok = all(x is not None for x in d) and max(d) <= acc["estop_stop_distance"] and all(r["success"] for r in es)
        add("Q5", "E-stop: stopping distance, resume after reset", ok,
            f"max {max(x for x in d if x is not None):.2f} m" if any(x is not None for x in d) else "no stop", f"<= {acc['estop_stop_distance']} m")
    wd = [r for r in results if r["condition"] == "cmd_loss"]
    if wd:
        ts = [r.get("watchdog_stop_time") for r in wd]
        ok = all(x is not None for x in ts) and max(ts) <= acc["watchdog_stop_time"] and all(r["success"] for r in wd)
        add("Q6", "Command loss: base watchdog stop time", ok,
            f"max {max(x for x in ts if x is not None):.2f} s" if any(x is not None for x in ts) else "no stop",
            f"<= {acc['watchdog_stop_time']} s")
    return checks


def qualify(cfg, jobs=2, seed0=1000):
    findings = lint(cfg)
    if errors(findings):
        return {"partner": cfg.get("partner"), "lint": findings, "results": [], "checks": [],
                "verdict": "BLOCKED (config errors)"}
    runs = plan_runs(cfg, seed0)
    if jobs > 1:
        with Pool(jobs) as p:
            results = p.map(_job, runs)
    else:
        results = [_job(r) for r in runs]
    checks = evaluate(cfg, results)
    verdict = "PASS" if all(c["pass"] for c in checks) else "FAIL"
    return {"partner": cfg.get("partner"), "lint": findings, "results": results, "checks": checks, "verdict": verdict}
