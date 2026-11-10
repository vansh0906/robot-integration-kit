import numpy as np
import pytest
from rik import maps
from rik.sensors import Lidar, Imu
from rik.localization import MapLocalizer
from rik.platforms import make_platform
from rik.planner import plan, plan_grid_only


def _lidar(cfg, seed=0):
    return Lidar(cfg["sensors"]["lidar"], np.random.default_rng(seed))


@pytest.mark.parametrize("site,pose", [("narrow_corridor", (1.2, 6.7, 0.0)), ("narrow_corridor", (16.7, 3.0, -1.57)),
                                        ("warehouse_aisles", (2.0, 7.0, 0.0)), ("yard_loop", (27.0, 10.0, 1.57))])
def test_scan_match_pulls_offset_back(cfgs, site, pose):
    g = maps.get(site)
    lid = _lidar(cfgs["c"])
    true = np.array(pose)
    loc = MapLocalizer(true + [0.08, -0.06, 0.02], g, lid)
    for k in range(10):
        loc.correct(lid.scan(true, g))
    assert np.hypot(*(loc.pose[:2] - true[:2])) < 0.05
    assert abs(loc.pose[2] - true[2]) < np.radians(1.0)


def test_scan_points_inside_walls_score_badly(cfgs):
    """Finding F1: the likelihood field must penalise points pushed *into* a wall."""
    g = maps.get("narrow_corridor")
    assert g.surface_at(np.array([[0.1, 6.7]])) > 0.0          # inside the west wall
    assert g.surface_at(np.array([[0.2, 6.7]]))[0] == pytest.approx(0.0, abs=0.06)


def test_featureless_aisle_keeps_prediction(cfgs):
    """Finding F2: along a long aisle the match is flat; the estimate must not slide to a window edge."""
    g = maps.get("warehouse_aisles")
    lid = _lidar(cfgs["a"])
    true = np.array([12.0, 3.5, 0.0])
    loc = MapLocalizer(true, g, lid)
    for _ in range(20):
        loc.correct(lid.scan(true, g))
    assert abs(loc.pose[0] - true[0]) < 0.05


def test_imu_slip_estimate_learns_odometry_scale(cfgs):
    g = maps.get("yard_loop")
    loc = MapLocalizer((2.0, 2.0, 0.0), g, _lidar(cfgs["b"]))
    imu = Imu({"accel_noise": 0.05, "accel_bias": 0.02}, np.random.default_rng(1))
    v, dt = 0.0, 0.02
    for _ in range(125):                                   # accelerate 0 -> 2 m/s at 0.8 m/s^2 on 12 % slip
        v_true = v + 0.8 * dt
        loc.predict(np.array([v_true / 0.88, 0, 0]), 0.0, dt)
        loc.update_slip(np.array([v_true / 0.88, 0, 0]), imu.accel(np.array([0.8, 0.0])), dt, acc_bias=0.02)
        v = v_true
    assert loc.s[4] == pytest.approx(1 / 0.88, abs=0.04)


def test_ackermann_plan_respects_turning_radius(cfgs):
    g = maps.get("yard_loop")
    pf = make_platform(cfgs["b"]["platform"])
    s, goal = g.missions[1]
    path = plan(g, pf, s, goal, cfgs["b"]["navigation"]["inflation"])
    seg = np.diff(path, axis=0)
    hd = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    curv = np.abs(np.diff(hd)) / 0.1
    assert np.percentile(curv, 99) <= 1.0 / pf.min_turn_radius * 1.3


def test_grid_planner_alone_is_not_feasible_for_ackermann(cfgs):
    """Finding F5: the plain A* path has corners tighter than the tug can steer."""
    g = maps.get("yard_loop")
    pf = make_platform(cfgs["b"]["platform"])
    s, goal = g.missions[1]
    path = plan_grid_only(g, pf, s, goal, cfgs["b"]["navigation"]["inflation"])
    seg = np.diff(path, axis=0)
    hd = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    assert (np.abs(np.diff(hd)) / 0.1).max() > 1.0 / pf.min_turn_radius * 2


def test_mecanum_path_keeps_footprint_clear_of_door_jamb(cfgs):
    """Finding F16: the oriented footprint along the planned path stays clear of the doorway."""
    g = maps.get("narrow_corridor")
    pf = make_platform(cfgs["c"]["platform"])
    s, goal = g.missions[3]
    path = plan(g, pf, s, goal, cfgs["c"]["navigation"]["inflation"])
    gaps = [g.fine_dist(pf.footprint_points((x, y, s[2]), n=8)).min() for x, y in path]
    assert min(gaps) >= cfgs["c"]["navigation"]["inflation"] - 0.02
