"""LiDAR ray casting on the grid (with dynamic circular obstacles and glass), IMU and wheel encoders."""
import numpy as np


class Lidar:
    def __init__(self, cfg, rng):
        self.x, self.y = float(cfg.get("x", 0.0)), float(cfg.get("y", 0.0))
        self.range = float(cfg["range"])
        self.n = int(cfg.get("beams", 270))
        fov = np.radians(float(cfg.get("fov_deg", 270)))
        self.angles = np.linspace(-fov / 2, fov / 2, self.n)
        self.noise = float(cfg.get("noise", 0.02))
        self.rate = float(cfg.get("rate", 10))
        self.rng = rng

    def origin(self, pose):
        x, y, th = pose
        return np.array([x + self.x * np.cos(th) - self.y * np.sin(th), y + self.x * np.sin(th) + self.y * np.cos(th)])

    def scan(self, pose, grid, circles=(), glass_see_prob=1.0):
        o = self.origin(pose)
        ang = pose[2] + self.angles
        d = np.cos(ang), np.sin(ang)
        step = grid.res / 5
        ts = np.arange(step, self.range + step, step)
        px = o[0] + np.outer(d[0], ts)
        py = o[1] + np.outer(d[1], ts)
        iy, ix = np.floor(py / grid.res).astype(int), np.floor(px / grid.res).astype(int)
        ok = grid.inside(iy, ix)
        hit = np.ones_like(ok)
        hit[ok] = grid.occ[iy[ok], ix[ok]]
        if glass_see_prob < 1.0:
            gl = np.zeros_like(ok)
            gl[ok] = grid.glass[iy[ok], ix[ok]]
            see = self.rng.random(self.n) < glass_see_prob
            hit &= ~(gl & ~see[:, None])
        first = np.where(hit.any(1), hit.argmax(1), -1)
        r = np.where(first >= 0, ts[np.clip(first, 0, None)] - step / 2, np.inf)
        for c, rad in circles:
            f = o - np.asarray(c)
            b = f[0] * d[0] + f[1] * d[1]
            disc = b * b - (f @ f - rad * rad)
            t = -b - np.sqrt(np.clip(disc, 0, None))
            r = np.where((disc >= 0) & (t > 0) & (t < r), t, r)
        r = r + self.rng.normal(0, self.noise, self.n)
        r[r > self.range] = np.inf
        return r

    def points(self, pose, ranges, frame="world"):
        """Beam end points (finite returns only), in world or robot frame."""
        ok = np.isfinite(ranges)
        a = self.angles[ok]
        lx = self.x + ranges[ok] * np.cos(a)
        ly = self.y + ranges[ok] * np.sin(a)
        if frame == "robot":
            return np.c_[lx, ly]
        x, y, th = pose
        c, s = np.cos(th), np.sin(th)
        return np.c_[x + c * lx - s * ly, y + s * lx + c * ly]


class Imu:
    def __init__(self, cfg, rng):
        self.noise = float(cfg.get("gyro_noise", 0.003))
        self.bias = float(cfg.get("gyro_bias", 0.002))
        self.acc_noise = float(cfg.get("accel_noise", 0.05))
        self.acc_bias = float(cfg.get("accel_bias", 0.02)) * rng.choice([-1.0, 1.0], 2)
        self.rng = rng

    def gyro(self, wz):
        return wz + self.bias + self.rng.normal(0, self.noise)

    def accel(self, a_body):
        """Planar specific force in the body frame (x forward, y left), m/s^2."""
        return np.asarray(a_body[:2]) + self.acc_bias + self.rng.normal(0, self.acc_noise, 2)


class Encoders:
    def __init__(self, cfg, rng):
        self.noise = float(cfg.get("noise", 0.01))
        self.rng = rng

    def twist(self, measured):
        return measured * (1 + self.rng.normal(0, self.noise, 3))
