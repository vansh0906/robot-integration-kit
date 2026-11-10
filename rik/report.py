"""Customer-facing qualification report: Markdown + plots + JSON + JUnit, one folder per partner."""
import json
import re
from collections import defaultdict
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from . import maps  # noqa: E402
from .sim import run_mission  # noqa: E402
from .platforms import make_platform  # noqa: E402

COND_TEXT = {
    "nominal": "Nominal",
    "low_friction": "Wet floor: wheels slip ~12 %",
    "glass_walls": "Glass partitions: LiDAR sees 10 % of glass returns",
    "popup_obstacle": "Person steps onto the path",
    "estop": "E-stop pressed mid-mission, released, reset",
    "cmd_loss": "Navigation commands lost for 3 s",
}


def slug(text):
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")[:40]


def plot_site(cfg, site, out_png, seed=7):
    grid = maps.get(site)
    pf = make_platform(cfg["platform"])
    fig, ax = plt.subplots(figsize=(8, 8 * grid.height / grid.width + 0.6))
    ax.imshow(grid.occ, origin="lower", extent=[0, grid.width, 0, grid.height], cmap="Greys", alpha=0.85)
    if grid.glass.any():
        g = np.ma.masked_where(~grid.glass, grid.glass)
        ax.imshow(g, origin="lower", extent=[0, grid.width, 0, grid.height], cmap="winter", alpha=0.9)
    colors = plt.cm.tab10(np.arange(len(grid.missions)))
    for i, (s, g_) in enumerate(grid.missions):
        r = run_mission(cfg, site, i, "nominal", seed + i, log=True)
        tr = r.get("trace")
        if tr is not None and len(tr):
            ax.plot(tr[:, 1], tr[:, 2], color=colors[i], lw=1.6, label=f"mission {i + 1}: {'pass' if r['success'] else 'FAIL'}")
            for k in range(0, len(tr), max(1, len(tr) // 6)):
                fp = pf.footprint_points(tr[k, 1:4], n=4)
                ax.fill(fp[[0, 1, 2, 3, 7, 6, 5, 4], 0], fp[[0, 1, 2, 3, 7, 6, 5, 4], 1], color=colors[i], alpha=0.12)
        ax.plot(*s[:2], "o", color=colors[i], ms=6)
        ax.plot(*g_[:2], "*", color=colors[i], ms=12)
    ax.set_xlim(0, grid.width)
    ax.set_ylim(0, grid.height)
    ax.set_aspect("equal")
    ax.set_title(f"{site}: nominal missions (o start, * goal; shaded = robot footprint)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def write(q, cfg, out_dir, make_plots=True):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    res = q["results"]
    p = cfg["platform"]
    lines = [f"# Qualification report: {cfg.get('partner', '')}", "",
             f"**Verdict: {q['verdict']}**  ",
             f"Platform: `{p['type']}`, footprint {p['footprint']['length']} x {p['footprint']['width']} m, "
             f"max {p['max_speed']} m/s. Sites: {', '.join(cfg.get('sites', []))}. "
             f"{len(res)} missions across {len(cfg.get('conditions', []))} conditions.", "",
             "All results are from the modeled simulator in this kit, not from the partner's hardware.", ""]
    lines += ["## Configuration check", ""]
    if q["lint"]:
        lines += ["| Severity | Code | Finding | Fix |", "|---|---|---|---|"]
        lines += [f"| {f.severity} | {f.code} | {f.message} | {f.fix} |" for f in q["lint"]]
    else:
        lines.append("No findings.")
    lines += ["", "## Acceptance checks", "", "| ID | Check | Result | Measured | Agreed limit |", "|---|---|---|---|---|"]
    for c in q["checks"]:
        lines.append(f"| {c['id']} | {c['name']} | {'PASS' if c['pass'] else '**FAIL**'} | {c['measured']} | {c['limit']} |")
    by = defaultdict(list)
    for r in res:
        by[r["condition"]].append(r)
    lines += ["", "## By condition", "", "| Condition | Missions passed | Min clearance | Worst p95 error | Mean time |",
              "|---|---|---|---|---|"]
    for cond, rs in by.items():
        lines.append(f"| {COND_TEXT.get(cond, cond)} | {sum(r['success'] for r in rs)}/{len(rs)} | "
                     f"{min(r['min_clearance'] for r in rs):.3f} m | {max(r['pos_err_p95'] for r in rs):.3f} m | "
                     f"{np.mean([r['time_s'] for r in rs]):.1f} s |")
    fails = [r for r in res if not r["success"]]
    if fails:
        lines += ["", "## Failed missions", "", "| Site | Mission | Condition | Seed | Reason | Min clearance |", "|---|---|---|---|---|---|"]
        lines += [f"| {r['site']} | {r['mission'] + 1} | {r['condition']} | {r['seed']} | {r.get('reason', 'contact')} | "
                  f"{r['min_clearance']} m |" for r in fails]
    if make_plots and res:
        lines += ["", "## Missions", ""]
        for site in cfg.get("sites", []):
            png = f"{site}.png"
            plot_site(cfg, site, out / png)
            lines.append(f"![{site}]({png})")
    lines += ["", "Re-run: `python tools/qualify.py " + cfg.get("_path", "<config>") + "`", ""]
    (out / "REPORT.md").write_text("\n".join(lines))
    clean = [{k: v for k, v in r.items() if k not in ("trace", "path")} for r in res]
    (out / "results.json").write_text(json.dumps({"partner": cfg.get("partner"), "verdict": q["verdict"],
                                                  "checks": q["checks"], "runs": clean}, indent=1, default=float))
    cases = []
    for c in q["checks"]:
        body = "" if c["pass"] else f'<failure message="{escape(str(c["measured"]))}"/>'
        cases.append(f'  <testcase classname="{escape(slug(cfg.get("partner", "partner")))}" name="{c["id"]} {escape(c["name"])}">{body}</testcase>')
    n_fail = sum(not c["pass"] for c in q["checks"])
    (out / "junit.xml").write_text(f'<?xml version="1.0"?>\n<testsuite name="{escape(cfg.get("partner", ""))}" tests="{len(cases)}" '
                                   f'failures="{n_fail}">\n' + "\n".join(cases) + "\n</testsuite>\n")
    return out / "REPORT.md"
