"""Run one mission for one partner config on one site under one condition."""
import numpy as np

from .platforms import make_platform
from .sensors import Lidar, Imu, Encoders
from .driver import BaseDriver
from .localization import MapLocalizer, wrap
from .planner import plan
from .controller import PathFollower
from . import maps

DT = 0.02
CONDITIONS = ["nominal", "low_friction", "glass_walls", "popup_obstacle", "estop", "cmd_loss"]
SLIP = 0.12               # low_friction: wheels over-read distance by ~12 %
GLASS_SEE = 0.1           # glass_walls: 10 % of beams return from glass


def clearance(pf, pose, grid, circles):
    pts = pf.footprint_points(pose, n=10)
    gap = float(grid.fine_dist(pts).min())
    for c, r in circles:
        gap = min(gap, float(np.hypot(*(pts - np.asarray(c)).T).min() - r))
    return gap


def run_mission(cfg, site, mission_idx, condition="nominal", seed=0, planner_fn=None, log=False):
    rng = np.random.default_rng(seed)
    grid = maps.get(site)
    start, goal = grid.missions[mission_idx % len(grid.missions)]
    pf = make_platform(cfg["platform"])
    s = cfg.get("sensors", {})
    lidar, imu, enc = Lidar(s["lidar"], rng), Imu(s.get("imu", {}), rng), Encoders(s.get("encoders", {}), rng)
    nav = cfg.get("navigation", {})
    drv = BaseDriver(pf)
    pose = np.array(start, float)
    loc = MapLocalizer(pose + rng.normal(0, [0.03, 0.03, np.radians(1)]), grid, lidar)
    fol = PathFollower(pf, nav)
    planner_fn = planner_fn or plan
    path = planner_fn(grid, pf, loc.pose, goal, float(nav.get("inflation", 0.15)))
    res = {"partner": cfg.get("partner", ""), "platform": pf.kind, "site": site, "mission": mission_idx,
           "condition": condition, "seed": seed, "planned": path is not None}
    if path is None:
        res.update(success=False, reason="no path", collision=False)
        return res
    fol.set_path(path, heading=pose[2])
    path_len = float(np.hypot(*np.diff(path, axis=0).T).sum())
    timeout = path_len / (0.25 * pf.max_speed) + 30.0
    slip = SLIP if condition == "low_friction" else 0.0
    see = GLASS_SEE if condition == "glass_walls" else 1.0
    circles, popup = [], None
    estop_t = cmd_drop = None
    if condition == "estop":
        estop_t = 6.0
    if condition == "cmd_loss":
        cmd_drop = (6.0, 9.0)                 # 3 s: long enough for the base to finish stopping
    pts_robot = None
    true_tw = np.zeros(3)
    prev_tw = np.zeros(3)
    t, travelled = 0.0, 0.0
    errs, min_clr, collision = [], np.inf, False
    trace = []
    stop_metrics = {}
    replans = 0
    k = 0
    while t < timeout:
        # ---- events
        if condition == "popup_obstacle" and popup is None and travelled > 0.4 * path_len:
            # place the person where a stop is physically possible: front of the robot + stopping
            # distance at full speed + the stop margin + 1 m (finding F9)
            d_need = (pf.length / 2 + pf.max_speed ** 2 / (2 * pf.decel) + fol.stop_margin + 1.0)
            ahead = path[min(len(path) - 1, fol.i + int(round(d_need / 0.1)))]
            popup = {"c": ahead, "t_on": t, "t_off": t + 6.0, "v_at_on": None}
            circles = [(ahead, 0.25)]
            stop_metrics["popup_speed"] = float(np.hypot(*true_tw[:2])) if k else 0.0
        if popup and t >= popup["t_off"] and circles:
            circles = []
        if estop_t is not None:
            if abs(t - estop_t) < DT / 2:
                drv.press_estop(t)
                stop_metrics.update(estop_speed=float(np.hypot(*true_tw[:2])), estop_pos=pose[:2].copy())
            if abs(t - (estop_t + 3.0)) < DT / 2:
                drv.release_estop(t)
            if abs(t - (estop_t + 4.0)) < DT / 2:
                drv.reset(t)
        # ---- sensing (10 Hz lidar)
        if k % int(round(1 / (lidar.rate * DT))) == 0:
            ranges = lidar.scan(pose, grid, circles, glass_see_prob=see)
            loc.correct(ranges)
            pts_robot = lidar.points(pose, ranges, frame="robot")
        # ---- navigation (50 Hz)
        est = loc.pose
        if np.hypot(*(path[min(fol.i, len(path) - 1)] - est[:2])) > 1.0 and replans < 3:
            new = planner_fn(grid, pf, est, goal, float(nav.get("inflation", 0.15)))
            if new is not None:
                path = new
                fol.set_path(path)
                replans += 1
        cmd = fol.command(est, pts_robot)
        if fol.done:
            break
        dropped = cmd_drop is not None and cmd_drop[0] <= t < cmd_drop[1]
        if not dropped:
            drv.on_cmd(t, cmd)
        target = drv.update(t, DT)
        true_tw, meas_tw = pf.step(target, DT, slip)
        # ---- truth integration
        c, sn = np.cos(pose[2]), np.sin(pose[2])
        pose = pose + np.array([(c * true_tw[0] - sn * true_tw[1]) * DT, (sn * true_tw[0] + c * true_tw[1]) * DT,
                                true_tw[2] * DT])
        pose[2] = wrap(pose[2])
        travelled += float(np.hypot(*true_tw[:2])) * DT
        meas = enc.twist(meas_tw)
        loc.predict(meas, imu.gyro(true_tw[2]), DT)
        a_body = (true_tw[:2] - prev_tw[:2]) / DT
        loc.update_slip(meas, imu.accel(a_body), DT, acc_bias=imu.acc_bias.max())
        prev_tw = true_tw.copy()
        # ---- metrics
        clr = clearance(pf, pose, grid, circles)
        min_clr = min(min_clr, clr)
        collision |= clr <= 0.0
        errs.append(float(np.hypot(*(loc.pose[:2] - pose[:2]))))
        speed = float(np.hypot(*true_tw[:2]))
        if estop_t is not None and "estop_stop_time" not in stop_metrics and t > estop_t and speed < 0.01:
            stop_metrics["estop_stop_time"] = t - estop_t
            stop_metrics["estop_stop_distance"] = float(np.hypot(*(pose[:2] - stop_metrics["estop_pos"])))
        if cmd_drop is not None and "watchdog_stop_time" not in stop_metrics and t > cmd_drop[0] and speed < 0.01:
            stop_metrics["watchdog_stop_time"] = t - cmd_drop[0]
        if popup and circles and "popup_gap" not in stop_metrics and speed < 0.01 and t > popup["t_on"]:
            stop_metrics["popup_gap"] = clr
        if log and k % 5 == 0:
            trace.append((t, *pose, *loc.pose, speed, clr))
        t += DT
        k += 1
    xy_goal = np.asarray(goal[:2])
    reached = fol.done and np.hypot(*(pose[:2] - xy_goal)) < 2.5 * fol.goal_tol
    res.update(success=bool(reached and not collision), reached=bool(reached), collision=bool(collision),
               time_s=round(t, 2), path_m=round(path_len, 2), travelled_m=round(travelled, 2),
               min_clearance=round(min_clr, 3), pos_err_p95=round(float(np.percentile(errs, 95)), 3) if errs else None,
               pos_err_max=round(float(max(errs)), 3) if errs else None, final_err=round(float(np.hypot(*(pose[:2] - xy_goal))), 3),
               replans=replans, loc_matches=loc.matches, relocalizations=loc.relocalizations, slip_updates=loc.slip_updates, events=[e[1] for e in drv.events])
    res.update({k_: (round(v, 3) if isinstance(v, float) else v) for k_, v in stop_metrics.items() if k_ != "estop_pos"})
    if not reached:
        res["reason"] = "timeout" if t >= timeout else "stopped"
    if log:
        res["trace"] = np.array(trace)
        res["path"] = path
    return res
