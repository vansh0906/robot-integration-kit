"""Global planning on the site grid: A* over an inflated costmap, then line-of-sight smoothing."""
import heapq
import numpy as np


def inflate(grid, radius):
    """Cells closer than `radius` to an obstacle are blocked."""
    return grid.dist < radius


def astar(grid, blocked, start, goal, clearance_weight=2.0, prefer=0.6):
    """8-connected A*. Cost adds a penalty for running closer than `prefer` metres to obstacles."""
    sy, sx = (int(v) for v in grid.to_cell(np.asarray(start[:2])))
    gy, gx = (int(v) for v in grid.to_cell(np.asarray(goal[:2])))
    if blocked[sy, sx] or blocked[gy, gx]:
        return None
    pen = clearance_weight * np.clip(prefer - grid.dist, 0, None)
    nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    g = {(sy, sx): 0.0}
    came = {}
    h = lambda y, x: np.hypot(y - gy, x - gx)          # noqa: E731
    pq = [(h(sy, sx), sy, sx)]
    closed = set()
    while pq:
        _, y, x = heapq.heappop(pq)
        if (y, x) in closed:
            continue
        if (y, x) == (gy, gx):
            break
        closed.add((y, x))
        for dy, dx, c in nbrs:
            ny, nx = y + dy, x + dx
            if not (0 <= ny < grid.ny and 0 <= nx < grid.nx) or blocked[ny, nx]:
                continue
            ng = g[(y, x)] + c * (1.0 + pen[ny, nx])
            if ng < g.get((ny, nx), np.inf):
                g[(ny, nx)] = ng
                came[(ny, nx)] = (y, x)
                heapq.heappush(pq, (ng + h(ny, nx), ny, nx))
    if (gy, gx) not in came and (gy, gx) != (sy, sx):
        return None
    cells = [(gy, gx)]
    while cells[-1] != (sy, sx):
        cells.append(came[cells[-1]])
    cells.reverse()
    pts = np.array([[(x + 0.5) * grid.res, (y + 0.5) * grid.res] for y, x in cells])
    pts[0], pts[-1] = start[:2], goal[:2]
    return pts


def segment_free(grid, blocked, a, b):
    n = max(2, int(np.hypot(*(b - a)) / (grid.res / 2)))
    t = np.linspace(0, 1, n)[:, None]
    p = a + (b - a) * t
    iy, ix = grid.to_cell(p)
    return not blocked[iy, ix].any()


def footprint_free(grid, platform, heading, clearance):
    """Segment check with the real, oriented footprint (used for a mecanum base holding its heading):
    a disc model lets a shortcut swing the footprint's corner into a door jamb (finding F16)."""
    def ok(a, b):
        n = max(2, int(np.hypot(*(b - a)) / 0.05))
        for t in np.linspace(0, 1, n):
            p = a + (b - a) * t
            if (grid.fine_dist(platform.footprint_points((p[0], p[1], heading), n=8)) < clearance).any():
                return False
        return True
    return ok


def smooth(grid, blocked, pts, extra_check=None):
    """Greedy line-of-sight shortcutting, then resample every 0.1 m."""
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not (segment_free(grid, blocked, pts[i], pts[j])
                                 and (extra_check is None or extra_check(pts[i], pts[j]))):
            j -= 1
        out.append(pts[j])
        i = j
    return resample(np.array(out))


def resample(pts, step=0.1):
    seg = np.diff(pts, axis=0)
    L = np.hypot(seg[:, 0], seg[:, 1])
    s = np.concatenate([[0], np.cumsum(L)])
    q = np.arange(0, s[-1], step)
    q = np.append(q, s[-1])
    return np.c_[np.interp(q, s, pts[:, 0]), np.interp(q, s, pts[:, 1])]


def plan_radius(platform):
    """Half-size the planner must keep free around the path.

    diff drive: turns in place anywhere -> circumscribed radius.
    mecanum: holds its heading and crabs (never rotates en route) -> half of its longer side.
    Ackermann: planned separately with the full footprint (hybrid_astar)."""
    if platform.kind == "diff_drive":
        return platform.radius
    if platform.holonomic:
        return max(platform.length, platform.width) / 2
    return platform.width / 2


def dijkstra_field(grid, blocked, goal):
    """Cost-to-go from every free cell to the goal (metres), used as the Hybrid A* heuristic."""
    gy, gx = (int(v) for v in grid.to_cell(np.asarray(goal[:2])))
    D = np.full(blocked.shape, np.inf)
    D[gy, gx] = 0.0
    pq = [(0.0, gy, gx)]
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    while pq:
        d, y, x = heapq.heappop(pq)
        if d > D[y, x]:
            continue
        for dy, dx, c in steps:
            ny, nx = y + dy, x + dx
            if 0 <= ny < grid.ny and 0 <= nx < grid.nx and not blocked[ny, nx]:
                nd = d + c * grid.res
                if nd < D[ny, nx]:
                    D[ny, nx] = nd
                    heapq.heappush(pq, (nd, ny, nx))
    return D


def hybrid_astar(grid, platform, start, goal, inflation, ds=0.6, xy_bin=0.25, th_bins=36, max_expand=60000):
    """Kinematically feasible planning for car-like robots: forward arcs no tighter than the
    minimum turning radius, each checked against the full footprint (not a disc)."""
    kmax = 1.0 / (platform.min_turn_radius * 1.1)
    curvs = np.array([-kmax, -kmax / 2, 0.0, kmax / 2, kmax])
    blocked2d = inflate(grid, platform.width / 2 + inflation)
    H = dijkstra_field(grid, blocked2d, goal)
    gxy = np.asarray(goal[:2], float)

    def key(p):
        return (int(p[0] // xy_bin), int(p[1] // xy_bin), int(((p[2] + np.pi) % (2 * np.pi)) // (2 * np.pi / th_bins)))

    def h(p):
        iy, ix = grid.to_cell(np.asarray(p[:2]))
        hv = H[int(iy), int(ix)] if grid.inside(iy, ix) else np.inf
        return max(hv, float(np.hypot(*(p[:2] - gxy))))

    def arc(p, k):
        sub = np.linspace(ds / 6, ds, 6)
        th = p[2] + k * sub
        if abs(k) < 1e-9:
            x = p[0] + sub * np.cos(p[2])
            y = p[1] + sub * np.sin(p[2])
        else:
            x = p[0] + (np.sin(th) - np.sin(p[2])) / k
            y = p[1] - (np.cos(th) - np.cos(p[2])) / k
        return np.c_[x, y, th]

    def free(states):
        pts = np.vstack([platform.footprint_points(st, n=6) for st in states])
        return bool((grid.fine_dist(pts) > inflation).all())

    s0 = np.asarray(start, float)
    pq = [(h(s0), 0.0, 0, tuple(s0), -1, ())]
    nodes = []                      # (state, parent index, segment points)
    seen = {}
    expanded = 0
    while pq and expanded < max_expand:
        f, g, _, st, parent, seg_in = heapq.heappop(pq)
        k = key(st)
        if k in seen:
            continue
        seen[k] = len(nodes)
        nodes.append((np.array(st), parent, seg_in))
        me = len(nodes) - 1
        expanded += 1
        # analytic finish: if the goal lies nearly straight ahead, drive straight onto it
        dxg, dyg = gxy[0] - st[0], gxy[1] - st[1]
        along = dxg * np.cos(st[2]) + dyg * np.sin(st[2])
        lateral = -dxg * np.sin(st[2]) + dyg * np.cos(st[2])
        finish = None
        if 0.0 < along < 2.0 and abs(lateral) < 0.12:
            tail = np.array([[st[0] + t * np.cos(st[2]), st[1] + t * np.sin(st[2]), st[2]]
                             for t in np.linspace(0.1, along, max(2, int(along / 0.1)))])
            if free(tail):
                finish = tail[:, :2]
        if finish is not None or np.hypot(dxg, dyg) < 0.2:
            # rebuild from the arc samples (not chords between nodes, which would show false kinks)
            pieces = [finish] if finish is not None else []
            i = me
            while i >= 0:
                pieces.append(np.array(nodes[i][2]).reshape(-1, 2) if nodes[i][2] else nodes[i][0][None, :2])
                i = nodes[i][1]
            return resample(np.vstack(pieces[::-1]))
        for kc in curvs:
            seg = arc(np.array(st), kc)
            nxt = seg[-1]
            if key(nxt) in seen or not free(seg):
                continue
            ng = g + ds * (1.0 + 0.3 * abs(kc) / kmax)
            heapq.heappush(pq, (ng + 1.2 * h(nxt), ng, len(nodes) + len(pq), tuple(nxt), me, tuple(map(tuple, seg[:, :2]))))
    return None


def plan(grid, platform, start, goal, inflation):
    if platform.kind == "ackermann":
        return hybrid_astar(grid, platform, start, goal, inflation)
    blocked = inflate(grid, plan_radius(platform) + inflation)
    raw = astar(grid, blocked, start, goal)
    if raw is None:
        return None
    extra = footprint_free(grid, platform, start[2], inflation) if platform.holonomic else None
    return smooth(grid, blocked, raw, extra)


def plan_grid_only(grid, platform, start, goal, inflation):
    """The original planner for every platform (kept to reproduce finding F5 in tests)."""
    blocked = inflate(grid, plan_radius(platform) + inflation)
    raw = astar(grid, blocked, start, goal)
    return None if raw is None else smooth(grid, blocked, raw)
