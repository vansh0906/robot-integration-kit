# Partner integration guide

For integration engineers and test leads bringing a new robot platform onto the navigation stack.
Expect about half a day for a first pass: write the config, lint it, run qualification, read the report.

## 1. What you need from the partner

| Item | Why | Typical mistake |
|---|---|---|
| Drive type: differential, Ackermann or mecanum | picks the kinematics model | calling a skid-steer "Ackermann" |
| Wheel radius, track width, wheel base (m) | kinematics | millimetres instead of metres |
| Max wheel speed (rad/s at the wheel) and max wheel acceleration | actuator limits | motor RPM before the gearbox |
| Max speed, max yaw rate, rated braking deceleration | speed planning and stopping distance | "top speed" from a brochure, not the tested value |
| Outer footprint length x width, bumpers included | clearance | measuring the chassis only |
| LiDAR mount (x, y in m from the robot centre), range, FOV, rate | perception and stopping | mount offsets with the wrong sign |
| IMU and wheel encoder noise (datasheet) | localization | leaving the defaults for a very different sensor |
| Site maps and the missions that matter | what to qualify on | qualifying on the easy route only |
| Acceptance thresholds, agreed in writing | the pass/fail line | thresholds that physics cannot meet (the linter checks) |

## 2. Write the config

Copy the closest example from `configs/partners/` and edit it. Every field is in metres, seconds and
radians unless the name says otherwise (`max_steer_deg`, `fov_deg`).

## 3. Lint it

```bash
python tools/lint_config.py configs/partners/<partner>.yaml
```

ERRORs block qualification; WARNs do not. Each finding prints the fix. The checks that most often catch
real problems:

| Code | Catches |
|---|---|
| CFG010 | wheel radius in mm or inches |
| CFG012 | max speed the wheels cannot reach (usually RPM vs rad/s) |
| CFG023 | stopping distance at full speed longer than the LiDAR can see |
| CFG031 | success rate written as 95 instead of 0.95 |
| CFG032 / CFG033 | watchdog or E-stop limits that physics cannot meet at the configured speed |
| CFG034 | inflation that leaves no room for path-tracking error, so clearance will fail |

`configs/examples/new_partner_draft.yaml` is a deliberately bad first draft: lint it to see them all.

## 4. Qualify

```bash
python tools/qualify.py configs/partners/<partner>.yaml            # full run
python tools/qualify.py configs/partners/<partner>.yaml --quick    # 2 missions per site, as a smoke test
```

Each condition listed in the config runs on every site, `missions_per_site` times, with a fresh seed each
run. Conditions: `nominal`, `low_friction` (wet floor, about 12 % wheel slip), `glass_walls` (LiDAR sees
10 % of glass returns), `popup_obstacle` (a person steps onto the path), `estop`, `cmd_loss`
(navigation commands lost for 3 s).

## 5. Read the report

`results/<partner>/REPORT.md` is written for the partner: verdict, config findings, the six acceptance
checks against the agreed limits, a per-condition table, failed missions with seeds, and a plot of every
mission with the robot footprint. `results.json` has every run; `junit.xml` plugs into CI.

To reproduce one failed mission exactly, use its site, mission number, condition and seed:

```python
from rik import config, sim
cfg = config.load("configs/partners/<partner>.yaml")
r = sim.run_mission(cfg, "narrow_corridor", 1, "popup_obstacle", seed=1301, log=True)
```

## 6. Troubleshooting

| Symptom in the report | First thing to check |
|---|---|
| Missions time out with the robot standing still | Obstacle stop triggering on a wall: is `navigation.inflation` too small for the footprint? |
| Large localization error in long aisles or corridors | Wheel slip (run `low_friction`), encoder noise in the config, LiDAR range too short to see aisle ends |
| Clearance just under the limit at doorways | Footprint vs door width; inflation vs `min_clearance` (CFG034) |
| Ackermann robot clips corners | `max_steer_deg` and `wheel_base`: the planner keeps turns above the minimum turning radius they imply |
| E-stop or watchdog check fails | Rated braking (`decel`, `max_wheel_accel`) vs the agreed limit (CFG032/033) |
| Person-step-out check fails | Speed vs LiDAR range and `stop_margin` |

## 7. Send feedback upstream

Anything that needed a product change, not a config change, goes in `docs/FEEDBACK_LOG.md`: what the
partner saw, root cause, the change, and before/after numbers.

## 8. On the robot (ROS 2)

`ros2_ws/src/rik_ros` runs the same base driver (watchdog, E-stop latch, deceleration ramp) as a ROS 2
node: it takes `/cmd_vel` and `/estop`, publishes wheel commands and odometry for the configured platform.
`cpp/include/rik/kinematics.hpp` is the same kinematics and watchdog as a dependency-free C++17 header for
the partner's motor-control board, checked against the Python version on 180 golden vectors.
