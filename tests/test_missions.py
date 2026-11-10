"""Closed-loop missions (slower): one per platform and condition, plus finding reproductions."""
import pytest
from rik.sim import run_mission
from rik.planner import plan_grid_only
from rik.qualification import evaluate


@pytest.mark.parametrize("who,site,cond", [
    ("a", "warehouse_aisles", "nominal"), ("a", "warehouse_aisles", "low_friction"), ("a", "warehouse_aisles", "popup_obstacle"),
    ("b", "yard_loop", "nominal"), ("b", "yard_loop", "low_friction"), ("b", "yard_loop", "estop"),
    ("c", "narrow_corridor", "nominal"), ("c", "narrow_corridor", "glass_walls"), ("c", "narrow_corridor", "cmd_loss"),
])
def test_mission(cfgs, who, site, cond):
    r = run_mission(cfgs[who], site, 1, cond, seed=7)
    acc = cfgs[who]["acceptance"]
    assert r["success"], r
    assert r["min_clearance"] >= acc["min_clearance"]
    if cond in ("nominal", "low_friction", "glass_walls"):
        assert r["pos_err_p95"] <= acc["position_error_p95"]


def test_ackermann_with_grid_planner_hits_corner(cfgs):
    """Finding F5 reproduced: plain A* + shortcutting makes the tug clip the building corner."""
    r = run_mission(cfgs["b"], "yard_loop", 1, "nominal", seed=2, planner_fn=plan_grid_only)
    assert r["min_clearance"] < cfgs["b"]["acceptance"]["min_clearance"]


def test_evaluate_flags_a_failure(cfgs):
    good = run_mission(cfgs["c"], "narrow_corridor", 0, "nominal", seed=3)
    bad = dict(good, success=False, min_clearance=0.0, collision=True)
    checks = {c["id"]: c["pass"] for c in evaluate(cfgs["c"], [good, bad])}
    assert not checks["Q1"] and not checks["Q3"]
