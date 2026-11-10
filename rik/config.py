"""Partner configuration: load a YAML file and lint it before anything runs.

The linter is the first thing a partner integration engineer runs. Each finding has a code,
a severity (ERROR blocks qualification, WARN does not) and a fix hint written for the partner.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import yaml

REQUIRED = {
    "diff_drive": ["wheel_radius", "track_width", "max_wheel_speed", "max_wheel_accel"],
    "mecanum": ["wheel_radius", "track_width", "wheel_base", "max_wheel_speed", "max_wheel_accel"],
    "ackermann": ["wheel_radius", "wheel_base", "max_steer_deg", "max_steer_rate_deg", "max_wheel_speed", "max_wheel_accel"],
}
COMMON = ["type", "max_speed", "max_yaw_rate", "decel", "footprint"]
ACCEPTANCE = ["mission_success_rate", "position_error_p95", "min_clearance", "estop_stop_distance",
              "watchdog_stop_time", "missions_per_site"]
CMD_LATENCY = 0.1            # s, command path latency assumed when checking stopping distance
TRACKING_BUDGET = {"diff_drive": 0.05, "mecanum": 0.05, "ackermann": 0.10}   # m, measured path-tracking error


@dataclass
class Finding:
    severity: str
    code: str
    message: str
    fix: str

    def __str__(self):
        return f"{self.severity:5s} {self.code}: {self.message}\n      fix: {self.fix}"


def load(path):
    with open(path) as fh:
        cfg = yaml.safe_load(fh)
    cfg["_path"] = str(path)
    return cfg


def lint(cfg):
    out = []
    E = lambda c, m, f: out.append(Finding("ERROR", c, m, f))     # noqa: E731
    W = lambda c, m, f: out.append(Finding("WARN", c, m, f))      # noqa: E731
    p = cfg.get("platform")
    if not isinstance(p, dict):
        E("CFG001", "missing 'platform' section", "add a platform: block (see configs/partners/partner_a_*.yaml)")
        return out
    kind = p.get("type")
    if kind not in REQUIRED:
        E("CFG002", f"unknown platform type '{kind}'", f"use one of {sorted(REQUIRED)}")
        return out
    for k in COMMON + REQUIRED[kind]:
        if k not in p:
            E("CFG003", f"platform.{k} is missing", f"add platform.{k} (units in docs/INTEGRATION_GUIDE.md)")
    if out:
        return out
    r = float(p["wheel_radius"])
    if not 0.02 <= r <= 0.6:
        E("CFG010", f"wheel_radius {r} m is outside 0.02-0.6 m", "wheel radius is in metres, not mm or inches")
    fp = p["footprint"]
    if fp.get("length", 0) <= 0 or fp.get("width", 0) <= 0:
        E("CFG011", "footprint length/width must be positive metres", "measure the outer body, bumpers included")
    w_lim = float(p["max_wheel_speed"]) * r
    if float(p["max_speed"]) > w_lim + 1e-9:
        E("CFG012", f"max_speed {p['max_speed']} m/s exceeds what the wheels allow ({w_lim:.2f} m/s)",
          "lower max_speed or check max_wheel_speed (rad/s, not RPM)")
    if kind == "ackermann":
        rmin = float(p["wheel_base"]) / math.tan(math.radians(float(p["max_steer_deg"])))
        cfg["_min_turn_radius"] = rmin
        if float(p["max_steer_deg"]) > 45:
            W("CFG013", f"max_steer_deg {p['max_steer_deg']} is unusually large", "confirm the steering stop angle")
    s = cfg.get("sensors", {})
    lid = s.get("lidar")
    if not lid:
        E("CFG020", "no lidar configured", "obstacle stopping needs a lidar: add sensors.lidar")
    else:
        if abs(lid.get("x", 0)) > fp["length"] / 2 or abs(lid.get("y", 0)) > fp["width"] / 2:
            W("CFG021", "lidar mount is outside the footprint", "check the mount offset sign and units (m)")
        if lid.get("fov_deg", 0) < 180:
            W("CFG022", f"lidar FOV {lid.get('fov_deg')} deg cannot see to the sides", "use >= 180 deg or add a second sensor")
        stop = float(p["max_speed"]) ** 2 / (2 * float(p["decel"])) + float(p["max_speed"]) * CMD_LATENCY
        margin = float(cfg.get("navigation", {}).get("stop_margin", 0.3))
        if stop + margin > float(lid.get("range", 0)):
            E("CFG023", f"stopping distance {stop:.2f} m + margin {margin} m exceeds lidar range {lid.get('range')} m",
              "lower max_speed, raise decel, or use a longer-range lidar")
        if float(lid.get("rate", 10)) < 5:
            W("CFG024", f"lidar rate {lid.get('rate')} Hz is low for {p['max_speed']} m/s", "use >= 10 Hz")
    if "encoders" not in s:
        E("CFG025", "no wheel encoders configured", "odometry needs sensors.encoders")
    acc = cfg.get("acceptance", {})
    for k in ACCEPTANCE:
        if k not in acc:
            E("CFG030", f"acceptance.{k} is missing", "agree a threshold with the partner and add it")
    if acc.get("mission_success_rate", 1) > 1:
        E("CFG031", "mission_success_rate is a fraction (0-1), not a percentage", "e.g. 0.95, not 95")
    if not out and "watchdog_stop_time" in acc:
        from .driver import WATCHDOG_TIMEOUT
        need = WATCHDOG_TIMEOUT + float(p["max_speed"]) / float(p["decel"])
        if float(acc["watchdog_stop_time"]) < need:
            E("CFG032", f"watchdog_stop_time {acc['watchdog_stop_time']} s is physically impossible: "
                        f"{WATCHDOG_TIMEOUT} s timeout + {p['max_speed']} m/s / {p['decel']} m/s^2 = {need:.2f} s",
              "agree a threshold of at least that with the partner, or lower max_speed")
    if not out and "estop_stop_distance" in acc:
        a_brake = float(p["max_wheel_accel"]) * float(p["wheel_radius"])
        need = float(p["max_speed"]) ** 2 / (2 * a_brake) + float(p["max_speed"]) * CMD_LATENCY
        if float(acc["estop_stop_distance"]) < need:
            E("CFG033", f"estop_stop_distance {acc['estop_stop_distance']} m is below the physical minimum {need:.2f} m",
              "raise the threshold or lower max_speed")
    nav = cfg.get("navigation", {})
    if not out and "min_clearance" in acc:
        budget = TRACKING_BUDGET.get(kind, 0.05)
        need = float(acc["min_clearance"]) + budget
        if float(nav.get("inflation", 0.0)) < need - 1e-9:
            E("CFG034", f"navigation.inflation {nav.get('inflation')} m leaves no room for path-tracking error: "
                        f"min_clearance {acc['min_clearance']} m + {budget} m tracking budget = {need:.2f} m",
              f"set navigation.inflation >= {need:.2f} (paths are smoothed to the edge of the inflated band)")
    sites = cfg.get("sites", [])
    if not sites:
        E("CFG040", "no sites listed", "list the partner's site maps under sites:")
    from .maps import MAPS
    for site in sites:
        if site not in MAPS:
            E("CFG041", f"unknown site '{site}'", f"use one of {sorted(MAPS)}")
    return out


def errors(findings):
    return [f for f in findings if f.severity == "ERROR"]


def default_paths():
    root = Path(__file__).resolve().parent.parent / "configs" / "partners"
    return sorted(p for p in root.glob("*.yaml"))
