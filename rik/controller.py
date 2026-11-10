"""Local control: path following per platform type + speed limits from obstacles near the path.

- diff drive: pure pursuit; turns in place first when the path is far off its heading
- Ackermann: pure pursuit with the steering limit (cannot turn in place)
- mecanum: holonomic tracking (drives straight at the look-ahead point, turns to the path heading)
Obstacle speed limit: LiDAR points inside the swept corridor ahead -> slow down, stop at stop_margin.
"""
import numpy as np
from .localization import wrap

LAT_ACCEL = 0.8           # m/s^2 allowed in turns


class PathFollower:
    def __init__(self, platform, nav):
        self.pf = platform
        self.Ld0 = float(nav.get("lookahead", 0.6))
        self.goal_tol = float(nav.get("goal_tolerance", 0.15))
        self.stop_margin = float(nav.get("stop_margin", 0.3))
        self.path = None
        self.i = 0
        self.done = False
        self.blocked = False
        self.obstacle_gap = np.inf
        self.half_w = platform.width / 2 + 0.05     # must stay inside the planner's clearance band
        if platform.holonomic:
            self.half_w = max(platform.length, platform.width) / 2 + 0.05
        self.hold_heading = None
        self.check_dist = platform.max_speed ** 2 / (2 * platform.decel) + 3.0

    def set_path(self, path, heading=None):
        self.path, self.i, self.done = path, 0, False
        if heading is not None:
            self.hold_heading = heading

    def _progress(self, xy):
        win = self.path[self.i:self.i + 40]
        self.i += int(np.argmin(np.hypot(*(win - xy).T)))

    def obstacle_limit(self, pose, pts_robot):
        """Max speed from LiDAR points that lie on the path ahead (within the robot's half width).

        Checking against the planned path rather than a straight line ahead means a wall the path
        turns away from never stops the robot, while anything standing on the path does.
        """
        self.obstacle_gap = np.inf
        if pts_robot is None or len(pts_robot) == 0:
            return np.inf
        x, y, th = pose
        c, s = np.cos(th), np.sin(th)
        pts = np.c_[x + c * pts_robot[:, 0] - s * pts_robot[:, 1], y + s * pts_robot[:, 0] + c * pts_robot[:, 1]]
        ahead = self.path[self.i:self.i + int(self.check_dist / 0.1) + 1]
        if len(ahead) < 2:
            return np.inf
        near = np.hypot(*(pts - np.array([x, y])).T) < self.check_dist + 1.0
        pts = pts[near]
        if len(pts) == 0:
            return np.inf
        d = np.hypot(pts[:, None, 0] - ahead[None, :, 0], pts[:, None, 1] - ahead[None, :, 1])
        on_path = d.min(0) < self.half_w                      # path samples blocked by some point
        front = self.pf.length / 2 if not self.pf.holonomic else max(self.pf.length, self.pf.width) / 2
        idx = np.nonzero(on_path)[0]
        idx = idx[idx * 0.1 > front * 0.5]                    # ignore samples under the robot itself
        if len(idx) == 0:
            return np.inf
        gap = float(idx[0] * 0.1 - front)
        self.obstacle_gap = gap
        return float(np.sqrt(2 * self.pf.decel * 0.8 * max(0.0, gap - self.stop_margin)))

    def corner_limit(self):
        """Slow down before sharp corners in the path so the robot can make the turn without
        overshooting (a mecanum cart crabbing into a 0.9 m doorway, finding F13)."""
        ahead = self.path[self.i:self.i + int(self.check_dist / 0.1) + 3]
        if len(ahead) < 5:
            return np.inf
        seg = np.diff(ahead, axis=0)
        hd = np.arctan2(seg[:, 1], seg[:, 0])
        if len(hd) < 6:
            return np.inf
        turn = np.abs(wrap(hd[5:] - hd[:-5]))              # heading change over 0.5 m (catches gradual corners too)
        sharp = np.nonzero(turn > np.radians(30))[0]
        if len(sharp) == 0:
            return np.inf
        dist = sharp[0] * 0.1
        v_c = 0.25
        return float(np.sqrt(v_c ** 2 + 2 * self.pf.decel * 0.7 * dist))

    def command(self, pose, pts_robot=None, speed_cap=np.inf):
        x, y, th = pose
        xy = np.array([x, y])
        self._progress(xy)
        goal = self.path[-1]
        d_goal = float(np.hypot(*(goal - xy)))
        if d_goal < self.goal_tol or (self.i >= len(self.path) - 2 and d_goal < 2 * self.goal_tol):
            self.done = True
            return np.zeros(3)
        Ld = self.Ld0 + 0.3 * self.pf.max_speed
        rest = self.path[self.i:]
        k = int(np.searchsorted(np.cumsum(np.r_[0, np.hypot(*np.diff(rest, axis=0).T)]), Ld))
        tgt = rest[min(k, len(rest) - 1)]
        c, s = np.cos(th), np.sin(th)
        dx, dy = tgt - xy
        lx, ly = c * dx + s * dy, -s * dx + c * dy           # target in robot frame
        dist = max(np.hypot(lx, ly), 1e-3)
        v_goal = np.sqrt(2 * self.pf.decel * 0.6 * d_goal)
        v_max = min(self.pf.max_speed, v_goal, speed_cap, self.corner_limit())
        if self.pf.holonomic:
            # follow the path tangent and pull back onto the line (no corner cutting in doorways)
            j = min(self.i + 1, len(self.path) - 1)
            tan = self.path[j] - self.path[max(0, j - 1)]
            tan = tan / max(np.hypot(*tan), 1e-6)
            # a mecanum base crabs: it holds the heading it started with instead of turning,
            # so its corners never sweep a wall in a corridor corner or a doorway
            heading = self.hold_heading if self.hold_heading is not None else th
            v = min(v_max, self.obstacle_limit(pose, pts_robot))
            e = self.path[self.i] - xy
            w = tan * v + 2.0 * e
            sp = np.hypot(*w)
            if sp > max(v, 0.05):
                w = w * max(v, 0.05) / sp
            if v < 0.02:
                w = np.zeros(2)
            self.blocked = v < 0.02
            return np.array([c * w[0] + s * w[1], -s * w[0] + c * w[1],
                             np.clip(1.5 * wrap(heading - th), -self.pf.max_yaw_rate, self.pf.max_yaw_rate)])
        alpha = np.arctan2(ly, lx)
        kappa = 2 * np.sin(alpha) / dist
        if self.pf.kind == "diff_drive" and abs(alpha) > np.radians(60):
            self.blocked = False
            return np.array([0.0, 0.0, np.sign(alpha) * min(self.pf.max_yaw_rate, 1.0)])
        v_turn = np.sqrt(LAT_ACCEL / max(abs(kappa), 1e-3))
        v = min(v_max, v_turn)
        v = min(v, self.obstacle_limit(pose, pts_robot))
        self.blocked = v < 0.02
        return np.array([v, 0.0, v * kappa])
