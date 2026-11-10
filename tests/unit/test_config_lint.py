import copy
from pathlib import Path
from rik import config

ROOT = Path(__file__).resolve().parents[2]


def codes(cfg):
    return {f.code for f in config.lint(cfg)}


def test_shipped_partner_configs_are_clean(cfgs):
    for c in cfgs.values():
        assert not config.errors(config.lint(c)), c["partner"]


def test_draft_config_catches_typical_mistakes():
    c = config.load(ROOT / "configs" / "examples" / "new_partner_draft.yaml")
    got = codes(c)
    for code in ("CFG010", "CFG023", "CFG025", "CFG030", "CFG031", "CFG041", "CFG021", "CFG022"):
        assert code in got, code


def test_impossible_watchdog_threshold_is_an_error(cfgs):
    c = copy.deepcopy(cfgs["a"])
    c["acceptance"]["watchdog_stop_time"] = 0.6           # the original Partner A ask (finding F10)
    assert "CFG032" in codes(c)


def test_inflation_without_tracking_budget_is_an_error(cfgs):
    c = copy.deepcopy(cfgs["c"])
    c["navigation"]["inflation"] = 0.12                    # the original Partner C value (finding F12)
    assert "CFG034" in codes(c)


def test_max_speed_beyond_wheels(cfgs):
    c = copy.deepcopy(cfgs["a"])
    c["platform"]["max_speed"] = 5.0
    assert "CFG012" in codes(c)


def test_unknown_platform():
    assert codes({"platform": {"type": "hovercraft"}}) == {"CFG002"}
