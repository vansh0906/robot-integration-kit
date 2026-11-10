"""Drive platforms a partner can bring: differential drive, Ackermann (car-like) and mecanum.

Every platform turns a body-frame velocity command (vx, vy, wz) into actuator commands, limits
them without changing the commanded path curvature, and turns measured actuator states back into
body velocity (wheel odometry). `step()` adds actuator lag / rate limits and wheel slip.
"""
import numpy as np


class Platform:
    holonomic = False
    kind = ""

    def __init__(self, p):
        self.p = p
        self.r = float(p["wheel_radius"])
        self.max_speed = float(p["max_speed"])
        self.max_yaw_rate = float(p["max_yaw_rate"])
        self.decel = float(p["decel"])
        self.length = float(p["footprint"]["length"])
        self.width = float(p["footprint"]["width"])
        self.state = np.zeros(self.n_act)          # actuator state (wheel speeds, steer)

    # body twist limits shared by all platforms, scaled together so the path shape is kept
    def limit_body(self, cmd):
        vx, vy, wz = (float(c) for c in cmd)
        if not self.holonomic:
            vy = 0.0
        s = 1.0
        sp = np.hypot(vx, vy)
        if sp > self.max_speed:
            s = min(s, self.max_speed / sp)
        if abs(wz) > self.max_yaw_rate:
            s = min(s, self.max_yaw_rate / abs(wz))
        return np.array([vx, vy, wz]) * s

    def command(self, cmd):
        """Body command -> actuator targets within actuator limits."""
        return self.saturate(self.to_actuators(self.limit_body(cmd)))

    def step(self, target, dt, slip=0.0):
        """Advance actuators toward their targets; return (true body twist, measured body twist)."""
        self._advance(target, dt)
        measured = self.from_actuators(self.state)
        true = measured * (1.0 - slip)
        return true, measured

    def footprint_points(self, pose, n=6):
        x, y, th = pose
        L, W = self.length / 2, self.width / 2
        xs = np.linspace(-L, L, n)
        ys = np.linspace(-W, W, max(3, n // 2))
        pts = np.vstack([np.c_[xs, np.full(n, W)], np.c_[xs, np.full(n, -W)],
                         np.c_[np.full(len(ys), L), ys], np.c_[np.full(len(ys), -L), ys]])
        c, s = np.cos(th), np.sin(th)
        return pts @ np.array([[c, s], [-s, c]]) + np.array([x, y])

    @property
    def radius(self):
        return float(np.hypot(self.length, self.width) / 2)


class DiffDrive(Platform):
    kind = "diff_drive"
    n_act = 2

    def __init__(self, p):
        super().__init__(p)
        self.B = float(p["track_width"])
        self.w_max = float(p["max_wheel_speed"])
        self.a_max = float(p["max_wheel_accel"])

    def to_actuators(self, cmd):
        v, _, wz = cmd
        return np.array([(v - wz * self.B / 2) / self.r, (v + wz * self.B / 2) / self.r])

    def from_actuators(self, w):
        wl, wr = w
        return np.array([self.r * (wl + wr) / 2, 0.0, self.r * (wr - wl) / self.B])

    def saturate(self, w):
        m = np.max(np.abs(w))
        return w * (self.w_max / m) if m > self.w_max else w

    def _advance(self, target, dt):
        self.state += np.clip(target - self.state, -self.a_max * dt, self.a_max * dt)


class Mecanum(Platform):
    kind = "mecanum"
    holonomic = True
    n_act = 4

    def __init__(self, p):
        super().__init__(p)
        self.lx = float(p["wheel_base"]) / 2
        self.ly = float(p["track_width"]) / 2
        self.w_max = float(p["max_wheel_speed"])
        self.a_max = float(p["max_wheel_accel"])

    def to_actuators(self, cmd):
        vx, vy, wz = cmd
        k = self.lx + self.ly
        return np.array([vx - vy - k * wz, vx + vy + k * wz, vx + vy - k * wz, vx - vy + k * wz]) / self.r

    def from_actuators(self, w):
        w1, w2, w3, w4 = w
        k = self.lx + self.ly
        return self.r / 4 * np.array([w1 + w2 + w3 + w4, -w1 + w2 + w3 - w4, (-w1 + w2 - w3 + w4) / k])

    saturate = DiffDrive.saturate
    _advance = DiffDrive._advance


class Ackermann(Platform):
    """Rear-wheel drive, front steering. Actuators: [rear wheel speed (rad/s), steer angle (rad)]."""
    kind = "ackermann"
    n_act = 2

    def __init__(self, p):
        super().__init__(p)
        self.L = float(p["wheel_base"])
        self.steer_max = np.radians(float(p["max_steer_deg"]))
        self.steer_rate = np.radians(float(p["max_steer_rate_deg"]))
        self.w_max = float(p["max_wheel_speed"])
        self.a_max = float(p["max_wheel_accel"])

    @property
    def min_turn_radius(self):
        return self.L / np.tan(self.steer_max)

    def to_actuators(self, cmd):
        v, _, wz = cmd
        steer = np.arctan(self.L * wz / v) if abs(v) > 1e-3 else self.state[1]
        return np.array([v / self.r, steer])

    def from_actuators(self, a):
        v = self.r * a[0]
        return np.array([v, 0.0, v * np.tan(a[1]) / self.L])

    def saturate(self, a):
        return np.array([np.clip(a[0], -self.w_max, self.w_max), np.clip(a[1], -self.steer_max, self.steer_max)])

    def _advance(self, target, dt):
        self.state[0] += np.clip(target[0] - self.state[0], -self.a_max * dt, self.a_max * dt)
        self.state[1] += np.clip(target[1] - self.state[1], -self.steer_rate * dt, self.steer_rate * dt)


PLATFORMS = {c.kind: c for c in (DiffDrive, Mecanum, Ackermann)}


def make_platform(p):
    return PLATFORMS[p["type"]](p)
