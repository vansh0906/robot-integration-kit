"""Map-based localization.

EKF state [x, y, yaw, gyro bias, odometry scale], predicted with wheel odometry + gyro and corrected
by matching each LiDAR scan against the site map (likelihood field + local MAP search).

The odometry-scale state matters on slippery floors: wheel odometry then over-reads by a fixed
fraction, which white noise cannot represent. With the scale state the covariance grows with the
distance driven, the search window widens to match, and scan matches where the map has features
(aisle ends, corners) pull the position back and learn the slip.
"""
import numpy as np

SIGMA_HIT = 0.08          # m, likelihood-field width
DEGRADED_SCORE = 0.70     # match score below which the pose may have drifted out of the search window


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class MapLocalizer:
    def __init__(self, pose0, grid, lidar):
        self.s = np.r_[np.asarray(pose0, float), 0.0, 1.0]          # x, y, yaw, gyro bias, odom scale
        self.P = np.diag([0.05 ** 2, 0.05 ** 2, np.radians(2) ** 2, 0.005 ** 2, 0.08 ** 2])   # slip up to ~15 % plausible
        self.grid, self.lidar = grid, lidar
        self.last_score = 0.0
        self.matches = 0
        self.relocalizations = 0
        self.slip_updates = 0
        self._win = []            # (dt, wheel body velocity, imu accel) for the slip estimate

    @property
    def x(self):
        return self.s[:3]

    @property
    def pose(self):
        return self.s[:3].copy()

    def sigma_xy(self):
        return float(np.sqrt(np.linalg.eigvalsh(self.P[:2, :2]).max()))

    def predict(self, twist, gyro, dt):
        vx_m, vy_m, _ = twist
        k = max(self.s[4], 0.5)
        vx, vy = vx_m / k, vy_m / k
        th = self.s[2]
        c, sn = np.cos(th), np.sin(th)
        wx, wy = c * vx - sn * vy, sn * vx + c * vy
        self.s = self.s + np.array([wx * dt, wy * dt, (gyro - self.s[3]) * dt, 0.0, 0.0])
        self.s[2] = wrap(self.s[2])
        F = np.eye(5)
        F[0, 2] = -wy * dt
        F[1, 2] = wx * dt
        F[2, 3] = -dt
        F[0, 4] = -wx / k * dt
        F[1, 4] = -wy / k * dt
        sp = np.hypot(vx, vy)
        q = (0.01 * sp * dt) ** 2 + (0.002 * dt) ** 2
        Q = np.diag([q, q, (0.005 * dt) ** 2 + (0.02 * abs(gyro) * dt) ** 2, (1e-4) ** 2 * dt,
                     (0.02 * sp) ** 2 * dt])           # slip can change when the floor does
        self.P = F @ self.P @ F.T + Q

    def update_slip(self, v_wheel_body, acc_body, dt, window=1.0, min_dv=0.6, acc_bias=0.02):
        """Wheel slip from the IMU (finding F15): while the robot speeds up or slows down, compare the
        change in wheel-measured velocity with the change the accelerometer integrates over the same
        window. Their ratio is the odometry scale. Only used when the speed change is large enough
        for the accelerometer bias to be small in comparison."""
        self._win.append((dt, np.asarray(v_wheel_body[:2], float), np.asarray(acc_body[:2], float)))
        T = sum(w[0] for w in self._win)
        while T > window and len(self._win) > 2:
            T -= self._win.pop(0)[0]
        dv_wheel = self._win[-1][1] - self._win[0][1]
        dv_imu = sum((w[0] * w[2] for w in self._win[1:]), np.zeros(2))
        n_imu = float(np.hypot(*dv_imu))
        if n_imu < min_dv:
            return False
        z = float(np.hypot(*dv_wheel)) / n_imu
        r = (acc_bias * T * 1.5 / n_imu) ** 2 + 0.02 ** 2
        H = np.zeros(5)
        H[4] = 1.0
        S = self.P[4, 4] + r
        K = self.P[:, 4] / S
        self.s = self.s + K * (z - self.s[4])
        self.P = self.P - np.outer(K, H @ self.P)
        self._win = self._win[-1:]                 # use each window once
        self.slip_updates += 1
        return True

    def score(self, poses, pts_robot):
        th = poses[:, 2][:, None]
        c, s = np.cos(th), np.sin(th)
        wx = poses[:, 0][:, None] + c * pts_robot[:, 0] - s * pts_robot[:, 1]
        wy = poses[:, 1][:, None] + s * pts_robot[:, 0] + c * pts_robot[:, 1]
        d = self.grid.surface_at(np.stack([wx, wy], -1))
        return np.exp(-0.5 * (d / SIGMA_HIT) ** 2).mean(1)

    def _pass(self, centre, step_xy, step_th, n, pts):
        g = np.arange(-n, n + 1)
        dx, dy, dth = np.meshgrid(g * step_xy, g * step_xy, g * step_th, indexing="ij")
        d = np.c_[dx.ravel(), dy.ravel(), dth.ravel()]
        sc = self.score(centre + d, pts)
        ll = np.log(sc + 1e-9) * len(pts) * 0.1
        w = np.exp(ll - ll.max())
        w /= w.sum()
        mean_d = w @ d
        dm = d - mean_d
        C = (dm * w[:, None]).T @ dm + np.diag([0.015 ** 2, 0.015 ** 2, np.radians(0.4) ** 2])
        best = d[int(np.argmax(sc))]
        on_edge = max(abs(best[0]), abs(best[1])) >= n * step_xy - 1e-9
        return centre + mean_d, C, float(sc.max()), on_edge, centre + best

    def _relocalize(self, pts, radius=1.0, step=0.1):
        """Wide search when the match has degraded (finding F14). Accepted only if it is clearly
        better than the current pose, so identical-looking warehouse aisles cannot pull the robot
        into the wrong one."""
        g = np.arange(-radius, radius + 1e-9, step)
        a = np.radians(np.arange(-6, 7, 2))
        dx, dy, dth = np.meshgrid(g, g, a, indexing="ij")
        cand = self.x + np.c_[dx.ravel(), dy.ravel(), dth.ravel()]
        sc = self.score(cand, pts)
        i = int(np.argmax(sc))
        here = float(self.score(self.x[None], pts)[0])
        if sc[i] < here + 0.08:
            return False
        z, _, self.last_score, _, _ = self._pass(cand[i], 0.02, np.radians(0.5), 3, pts)
        self.s[:3] = z
        self.P[:3, :3] = np.diag([0.05 ** 2, 0.05 ** 2, np.radians(1.0) ** 2])
        self.P[4, 4] = max(self.P[4, 4], 0.05 ** 2)       # the drift says the slip estimate was off
        self.relocalizations += 1
        return True

    def correct(self, ranges):
        pts = self.lidar.points(self.x, ranges, frame="robot")
        if len(pts) < 20:
            return False
        pts = pts[:: max(1, len(pts) // 90)]
        sxy = self.sigma_xy()
        sth = float(np.clip(np.sqrt(self.P[2, 2]), np.radians(1.5), np.radians(5)))
        # Coarse pass over about +-3 sigma of the prediction, then a fine pass around its mean.
        # The measurement is the likelihood-weighted mean (not the argmax) with the weighted spread
        # as its covariance, so along a featureless aisle it stays near the prediction with a wide
        # covariance instead of snapping to a window edge (findings F2, F11).
        step = max(0.06, min(0.5, sxy))
        mean, R, _, _, _ = self._pass(self.x.copy(), step, sth, 3, pts)
        z, _, self.last_score, _, _ = self._pass(mean, 0.02, np.radians(0.5), 3, pts)
        if self.last_score < DEGRADED_SCORE and self._relocalize(pts):
            return True
        if self.last_score < 0.35:            # poor match (e.g. glass, clutter): skip the update
            return False
        best = z
        H = np.hstack([np.eye(3), np.zeros((3, 2))])
        y = best - self.x
        y[2] = wrap(y[2])
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + R)
        self.s = self.s + K @ y
        self.s[2] = wrap(self.s[2])
        self.P = (np.eye(5) - K @ H) @ self.P
        self.matches += 1
        return True
