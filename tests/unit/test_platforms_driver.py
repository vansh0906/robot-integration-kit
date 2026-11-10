import numpy as np
import pytest
from rik.platforms import make_platform
from rik.driver import BaseDriver, WATCHDOG_TIMEOUT


def test_round_trip_all_platforms(cfgs):
    for c in cfgs.values():
        pf = make_platform(c["platform"])
        cmd = np.array([0.5 * pf.max_speed, 0.2 if pf.holonomic else 0.0, 0.2])
        tw = pf.from_actuators(pf.command(cmd))
        assert np.allclose(tw, cmd if pf.kind != "ackermann" else [cmd[0], 0, cmd[2]], atol=1e-9)


def test_saturation_keeps_curvature(cfgs):
    pf = make_platform(cfgs["a"]["platform"])
    tw = pf.from_actuators(pf.command([5.0, 0, 5.0]))        # far over every limit
    assert tw[2] / tw[0] == pytest.approx(1.0)
    assert np.abs(pf.command([5.0, 0, 5.0])).max() <= pf.w_max + 1e-9


def test_ackermann_cannot_turn_tighter_than_min_radius(cfgs):
    pf = make_platform(cfgs["b"]["platform"])
    tw = pf.from_actuators(pf.command([1.0, 0, 5.0]))
    assert tw[0] / tw[2] == pytest.approx(pf.min_turn_radius, rel=1e-6)


def test_diff_drive_ignores_lateral_command(cfgs):
    pf = make_platform(cfgs["a"]["platform"])
    assert pf.from_actuators(pf.command([0.5, 0.5, 0]))[1] == 0.0


def _drive(drv, pf, t0, t1, cmd=None, dt=0.02):
    t = t0
    while t < t1:
        if cmd is not None:
            drv.on_cmd(t, cmd)
        pf.step(drv.update(t, dt), dt)
        t += dt
    return t


def test_watchdog_stops_base_when_commands_stop(cfgs):
    pf = make_platform(cfgs["a"]["platform"])
    drv = BaseDriver(pf)
    _drive(drv, pf, 0, 3, [1.0, 0, 0])
    assert pf.from_actuators(pf.state)[0] > 0.9
    _drive(drv, pf, 3, 3 + WATCHDOG_TIMEOUT + 1.2)
    assert abs(pf.from_actuators(pf.state)[0]) < 0.01
    assert ("WATCHDOG" in [e[1] for e in drv.events])


def test_estop_latched_until_reset(cfgs):
    pf = make_platform(cfgs["c"]["platform"])
    drv = BaseDriver(pf)
    _drive(drv, pf, 0, 2, [0.5, 0, 0])
    drv.press_estop(2.0)
    _drive(drv, pf, 2, 3, [0.5, 0, 0])
    assert abs(pf.from_actuators(pf.state)[0]) < 1e-6
    drv.release_estop(3.0)
    _drive(drv, pf, 3, 4, [0.5, 0, 0])
    assert abs(pf.from_actuators(pf.state)[0]) < 1e-6          # still latched
    drv.reset(4.0)
    _drive(drv, pf, 4, 6, [0.5, 0, 0])
    assert pf.from_actuators(pf.state)[0] > 0.45


def test_reset_refused_while_estop_held(cfgs):
    pf = make_platform(cfgs["a"]["platform"])
    drv = BaseDriver(pf)
    drv.press_estop(0.0)
    drv.reset(0.1)
    assert drv.estop_latched
