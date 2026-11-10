#!/usr/bin/env python3
"""Qualify one or more partner configurations and write customer-ready reports.

    python tools/qualify.py configs/partners/partner_a_warehouse_amr.yaml
    python tools/qualify.py --all                 # every config in configs/partners/
    python tools/qualify.py --all --quick         # 2 missions per site (smoke test)
Exit code 1 if any partner is FAIL or BLOCKED.
"""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from rik import config, qualification, report  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("configs", nargs="*")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--quick", action="store_true", help="2 missions per site")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--out", default=str(ROOT / "results"))
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args()
    paths = config.default_paths() if a.all or not a.configs else [Path(p) for p in a.configs]
    rows, bad = [], 0
    for p in paths:
        cfg = config.load(p)
        if a.quick and "acceptance" in cfg:
            cfg["acceptance"]["missions_per_site"] = 2
        q = qualification.qualify(cfg, jobs=a.jobs)
        md = report.write(q, cfg, Path(a.out) / report.slug(p.stem), make_plots=not a.no_plots)
        n = len(q["results"])
        ok = sum(r["success"] for r in q["results"])
        rows.append(f"| {cfg.get('partner', p.stem)} | {cfg['platform']['type']} | {q['verdict']} | "
                    f"{sum(c['pass'] for c in q['checks'])}/{len(q['checks'])} | {ok}/{n} | [{md.parent.name}]({md.parent.name}/REPORT.md) |")
        print(f"{q['verdict']:8s} {cfg.get('partner', p.stem)}  ({ok}/{n} missions)  -> {md}")
        for c in q["checks"]:
            print(f"    {c['id']} {'PASS' if c['pass'] else 'FAIL'}  {c['name']}: {c['measured']} (limit {c['limit']})")
        bad += q["verdict"] != "PASS"
    summary = ["# Qualification summary", "", "| Partner | Platform | Verdict | Checks | Missions | Report |", "|---|---|---|---|---|---|"] + rows
    Path(a.out).mkdir(parents=True, exist_ok=True)
    (Path(a.out) / "SUMMARY.md").write_text("\n".join(summary) + "\n")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
