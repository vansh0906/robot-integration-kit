# Partner feedback → product requirements

The three partners in `configs/partners/` are **fictional example profiles**. Every item below came from
this kit's own qualification runs (`tools/qualify.py`) failing or misbehaving while it was being built.
Each one is written the way it would go back to the product team: what the partner sees, why, and what
changed (product code, partner configuration, or the agreed spec).

First full qualification: **A FAIL (80 %), B FAIL (60 %), C FAIL (81 %)**. Current: **A, B, C PASS
(148/148 missions, 0 contacts)**.

| ID | Partner / area | What the partner would see | Root cause | Change | Before → after |
|---|---|---|---|---|---|
| F1 | C, localization | Cart thinks it is 2.3 m behind where it is; never reaches the goal | Scan-match field scored 0 *inside* walls, so pushing scan points into a wall looked like a perfect fit | Product: two-sided surface-distance field, bilinear interpolation | 2.3 m → 0.01 m error |
| F2 | A, localization | 0.55 m position error in long aisles | Along an aisle every candidate pose scores the same; the search picked the window edge every scan | Product: MAP search with the EKF prior | p95 0.54 m → 0.07-0.09 m |
| F3 | A, obstacle stop | Robot stops forever at an aisle entrance | Obstacle check looked straight ahead and saw the shelf end the path turns away from | Product: check obstacles along the planned path | 3/5 → 5/5 missions |
| F4 | All, test metrics | Clearance read 0.0 next to walls | Clearance measured on 0.1 m cells | Kit: 5x finer distance field for metrics | metric resolution 0.05 m → 0.02 m |
| F5 | B, planning | Yard tug clips the building corner | Grid path had corners tighter than the tug can steer | Product: Hybrid A* (curvature-limited, full-footprint checks) for Ackermann | contact → 0.21 m clearance |
| F6 | C, obstacle stop | Cart stops at the corridor corner | Obstacle band (0.31 m) wider than the planner's clearance band minus LiDAR noise | Product: band kept inside the planner's clearance | timeout → pass |
| F7 | C, control | Cart hits the doorway jamb | Holonomic controller drove straight at the look-ahead point and cut the corner | Product: tangent + cross-track tracking; crab with fixed heading | contact → pass |
| F8 | A, B, C, wet floor | 0/28 slippery-floor missions succeed; 2-3 m error | Odometry bias modelled as white noise → over-confident filter | Product: odometry-scale state in the EKF; search window sized from the covariance | 0/28 → 7/28 |
| F9 | B, test design | Tug hits a person who steps out 2 m ahead | Test spawned the person inside the tug's physical stopping distance | Kit: person appears at stopping distance + margin + 1 m | unfair test removed |
| F10 | A, C, spec | Watchdog check fails every run | Agreed limit (0.6 s) impossible at 1.4 m/s with 1.2 m/s² brakes (1.42 s) | Linter rule CFG032; partner limit revised to 1.6 s (A) / 1.2 s (C) | spec now physically possible |
| F11 | A, B, wet floor | Error still grows along featureless straights | Fixed 3 cm scan-match covariance pinned the along-aisle position | Product: measurement = likelihood-weighted mean, covariance from the shape of the match score | 7/28 → 25/28 |
| F12 | C, spec | 0.07 m clearance vs 0.08 m agreed | Path smoothing runs to the edge of the inflated band, leaving no room for tracking error | Linter rule CFG034; inflation 0.12 → 0.15 m (C), 0.25 → 0.30 m (B) | 0.070 → 0.079 m |
| F13 | C, control | Cart overshoots into the jamb turning into a 0.9 m door | Crabbing at 0.8 m/s into a 90° turn; braking limit | Product: slow to 0.25 m/s before path corners | 0.07 → 0.11 m |
| F14 | B, wet floor | Tug stands still forever after drifting 0.8 m | Pose drifted outside the search window; standing still never widens it | Product: wide-search relocalization when the match degrades, accepted only if clearly better (identical aisles can't alias) | timeout → pass |
| F15 | A, B, wet floor | Slip error 0.12 m (A) and 0.7 m (B) along long straights | Slip only observable where the map has features | Product: IMU accelerometer slip estimate during speed changes | 25/28 → 28/28; A 0.118 → 0.067 m, B 0.68 → 0.13 m |
| F16 | C, planning | 0.079 m at the doorway (limit 0.08 m) | Planner treated the 0.50 x 0.42 m cart as a 0.25 m disc; a shortcut swung its corner at the jamb | Product: shortcut smoothing checks the oriented footprint | 0.079 → 0.116 m |

Things tried and rolled back (kept here so nobody tries them again):
- Treating flat scan-match directions as unobserved: made nominal error worse (B p95 0.06 → 0.37 m).
- Re-centring the search on the window edge: jumped into the wrong, identical warehouse aisle (4-10 m error).
