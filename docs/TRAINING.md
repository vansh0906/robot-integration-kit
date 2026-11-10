---
marp: true
title: Partner integration training
paginate: true
---

# Partner integration training
### Getting a new robot onto the navigation stack in half a day

Audience: partner integration engineers and test leads
Hands-on: bring a laptop with Python 3.10+ and this repo

---

## Agenda (90 min)

1. How the stack is put together (15 min)
2. Writing a platform config (20 min, hands-on)
3. Linting: catching mistakes before they cost a test day (10 min, hands-on)
4. Qualification: what each test proves (20 min)
5. Reading the report and debugging a failure (20 min, hands-on)
6. Sending feedback upstream (5 min)

---

## 1. The stack

```
LiDAR + IMU + wheel encoders
        │
  Map localizer (EKF + scan matching, slip estimate)
        │
  Planner (A* or Hybrid A* for car-like robots)
        │
  Path follower (per drive type) + obstacle stop
        │
  Base driver: watchdog · E-stop latch · decel ramp   ← same code on the robot (ROS 2 / C++)
        │
  Your motor controllers
```

One config file describes your robot. Nothing else changes per partner.

---

## 2. Your config (hands-on)

- Start from the closest file in `configs/partners/`
- Units: m, s, rad (except `max_steer_deg`, `fov_deg`)
- **Wheel speed is rad/s at the wheel**, not motor RPM
- **Footprint includes bumpers**
- Acceptance limits: agree them in writing *before* qualification

Exercise: write a config for a 0.6 x 0.5 m differential-drive robot, 1.0 m/s, 10 m LiDAR.

---

## 3. Lint (hands-on)

```bash
python tools/lint_config.py configs/examples/new_partner_draft.yaml
```

The draft has 8 mistakes. Fix them one by one until it prints `OK`.

Ones that bite in the field:
- stopping distance longer than the LiDAR range (CFG023)
- a watchdog limit physics cannot meet (CFG032)
- no room for tracking error between inflation and clearance (CFG034)

---

## 4. What qualification proves

| Condition | Proves |
|---|---|
| nominal | the robot completes the partner's missions, with margin |
| low_friction | localization survives ~12 % wheel slip |
| glass_walls | localization survives a LiDAR that sees through glass |
| popup_obstacle | it stops for a person who steps out, then resumes |
| estop | stop distance, and no motion until an explicit reset |
| cmd_loss | the base stops on its own if navigation goes silent |

---

## 5. Reading the report (hands-on)

```bash
python tools/qualify.py configs/partners/partner_c_mecanum_cart.yaml --quick
```

Open `results/partner_c_mecanum_cart/REPORT.md`:
- the verdict and the six checks
- the per-condition table: where is the margin smallest?
- the mission plot: where does the footprint come closest to a wall?

Exercise: set `navigation.inflation: 0.12`. What does the linter say? What does qualification say?

---

## Debugging a failure

1. Find the failed mission in the report: site, mission, condition, **seed**
2. Re-run exactly that mission with `log=True`
3. Plot truth vs estimate; find the first moment they diverge
4. Ask: config, site, or product?

Most first-week failures are config: units, footprint, inflation, braking.

---

## 6. Feedback upstream

If it needed a product change, log it in `docs/FEEDBACK_LOG.md`:

| What the partner saw | Root cause | Change | Before → after |
|---|---|---|---|

16 entries so far, each from a real qualification run. Read F10 and F12: both were *spec* problems, not code.

---

## Checklist before you leave

- [ ] Your config lints clean
- [ ] `--quick` qualification passes
- [ ] You can re-run a single mission by seed
- [ ] You know who owns the acceptance limits on your side
