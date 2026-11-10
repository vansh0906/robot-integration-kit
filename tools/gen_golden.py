#!/usr/bin/env python3
"""Generate golden kinematics vectors from the Python platforms for the C++ library's tests.

    python tools/gen_golden.py      # writes tests/golden/kinematics.csv
The C++ test (cpp/tests/test_kinematics.cpp) must reproduce every row to 1e-9.
"""
import csv
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from rik import config  # noqa: E402
from rik.platforms import make_platform  # noqa: E402


def main():
    rng = np.random.default_rng(42)
    rows = []
    for p in config.default_paths():
        pc = config.load(p)["platform"]
        pf = make_platform(pc)
        for _ in range(60):
            vx = rng.uniform(-1.5, 1.5) * pf.max_speed
            if pf.kind == "ackermann" and abs(vx) < 0.05:
                vx = 0.3
            cmd = np.array([vx, rng.uniform(-1.5, 1.5) * pf.max_speed, rng.uniform(-1.5, 1.5) * pf.max_yaw_rate])
            a = pf.command(cmd)
            a = np.r_[a, np.zeros(4 - len(a))]
            fwd = pf.from_actuators(pf.command(cmd))
            rows.append([pf.kind, p.stem, *cmd, *a, *fwd])
    out = ROOT / "tests" / "golden" / "kinematics.csv"
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["kind", "config", "vx", "vy", "wz", "a0", "a1", "a2", "a3", "fvx", "fvy", "fwz"])
        for r in rows:
            w.writerow([r[0], r[1]] + [f"{v:.17g}" for v in r[2:]])
    params = ROOT / "tests" / "golden" / "platforms.csv"
    with open(params, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["config", "kind", "wheel_radius", "track_width", "wheel_base", "max_wheel_speed", "max_steer_deg",
                    "max_speed", "max_yaw_rate"])
        for p in config.default_paths():
            pc = config.load(p)["platform"]
            w.writerow([p.stem, pc["type"], pc["wheel_radius"], pc.get("track_width", 0), pc.get("wheel_base", 0),
                        pc["max_wheel_speed"], pc.get("max_steer_deg", 0), pc["max_speed"], pc["max_yaw_rate"]])
    print(f"wrote {len(rows)} vectors to {out}")


if __name__ == "__main__":
    main()
