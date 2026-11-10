"""Site maps as occupancy grids (0.1 m cells), with missions (start pose -> goal pose).

Some walls can be flagged as glass: they are physical obstacles, but the LiDAR mostly sees
through them, which is one of the 'challenging environment' conditions.
"""
from dataclasses import dataclass, field
import numpy as np
from scipy.ndimage import distance_transform_edt

RES = 0.1


@dataclass
class Map:
    name: str
    occ: np.ndarray                 # [ny, nx] bool, True = obstacle
    glass: np.ndarray               # [ny, nx] bool, subset of occ that the lidar mostly cannot see
    missions: list = field(default_factory=list)
    res: float = RES

    def __post_init__(self):
        self.ny, self.nx = self.occ.shape
        self.width, self.height = self.nx * self.res, self.ny * self.res
        # metres from a cell centre to the nearest obstacle surface (cell edge), 0 inside obstacles
        self.dist = np.clip(distance_transform_edt(~self.occ) * self.res - self.res / 2, 0, None)
        # unsigned distance to the nearest surface from either side: a scan point pushed *into* a wall
        # must score as badly as one floating in free space (used by the localizer)
        inside = np.clip(distance_transform_edt(self.occ) * self.res - self.res / 2, 0, None)
        self.surface = np.where(self.occ, inside, self.dist)

    def to_cell(self, xy):
        xy = np.asarray(xy, float)
        return np.floor(xy[..., 1] / self.res).astype(int), np.floor(xy[..., 0] / self.res).astype(int)

    def inside(self, iy, ix):
        return (iy >= 0) & (iy < self.ny) & (ix >= 0) & (ix < self.nx)

    def dist_at(self, xy):
        iy, ix = self.to_cell(xy)
        ok = self.inside(iy, ix)
        d = np.zeros(np.shape(iy))
        d[ok] = self.dist[iy[ok], ix[ok]]
        return d

    def fine_dist(self, xy, k=5):
        """Distance to the nearest obstacle on a k-times finer grid (used for clearance metrics)."""
        if not hasattr(self, "_fine"):
            occ = np.repeat(np.repeat(self.occ, k, 0), k, 1)
            self._fine = distance_transform_edt(~occ) * self.res / k
            self._k = k
        xy = np.asarray(xy, float)
        f = self.res / self._k
        iy = np.clip(np.floor(xy[..., 1] / f).astype(int), 0, self._fine.shape[0] - 1)
        ix = np.clip(np.floor(xy[..., 0] / f).astype(int), 0, self._fine.shape[1] - 1)
        return np.clip(self._fine[iy, ix] - f / 2, 0, None)

    def surface_at(self, xy):
        """Bilinear interpolation of the surface-distance field (cell values sit at cell centres)."""
        xy = np.asarray(xy, float)
        fx = xy[..., 0] / self.res - 0.5
        fy = xy[..., 1] / self.res - 0.5
        x0, y0 = np.floor(fx).astype(int), np.floor(fy).astype(int)
        ax, ay = fx - x0, fy - y0
        x0 = np.clip(x0, 0, self.nx - 2)
        y0 = np.clip(y0, 0, self.ny - 2)
        S = self.surface
        return ((1 - ax) * (1 - ay) * S[y0, x0] + ax * (1 - ay) * S[y0, x0 + 1]
                + (1 - ax) * ay * S[y0 + 1, x0] + ax * ay * S[y0 + 1, x0 + 1])

    def occupied(self, xy):
        iy, ix = self.to_cell(xy)
        ok = self.inside(iy, ix)
        o = np.ones(np.shape(iy), bool)
        o[ok] = self.occ[iy[ok], ix[ok]]
        return o


class _Builder:
    def __init__(self, w, h):
        self.occ = np.zeros((int(round(h / RES)), int(round(w / RES))), bool)
        self.glass = np.zeros_like(self.occ)
        self.rect(0, 0, w, 0.2)
        self.rect(0, h - 0.2, w, h)
        self.rect(0, 0, 0.2, h)
        self.rect(w - 0.2, 0, w, h)

    def rect(self, x0, y0, x1, y1, glass=False):
        iy0, iy1 = int(round(y0 / RES)), int(round(y1 / RES))
        ix0, ix1 = int(round(x0 / RES)), int(round(x1 / RES))
        self.occ[iy0:iy1, ix0:ix1] = True
        if glass:
            self.glass[iy0:iy1, ix0:ix1] = True

    def clear(self, x0, y0, x1, y1):
        iy0, iy1 = int(round(y0 / RES)), int(round(y1 / RES))
        ix0, ix1 = int(round(x0 / RES)), int(round(x1 / RES))
        self.occ[iy0:iy1, ix0:ix1] = False
        self.glass[iy0:iy1, ix0:ix1] = False


def warehouse_aisles():
    """24 x 14 m: five shelving rows, 1.8 m aisles, 3 m cross aisles at both ends."""
    b = _Builder(24, 14)
    for k in range(5):
        y0 = 1.8 + k * 2.6
        b.rect(4.0, y0, 20.0, y0 + 0.8)
    m = [((1.5, 1.0, 0.0), (22.5, 12.8, np.pi / 2)),
         ((2.0, 7.0, 0.0), (22.0, 7.0, 0.0)),
         ((22.0, 1.0, np.pi), (2.0, 12.8, np.pi)),
         ((1.5, 12.8, 0.0), (21.5, 4.0, 0.0)),
         ((12.0, 3.5, 0.0), (12.0, 11.3, np.pi))]
    return Map("warehouse_aisles", b.occ, b.glass, m)


def narrow_corridor():
    """20 x 12 m office block: a 1.4 m corridor with a 90 deg turn, glass partition, 0.9 m doorway."""
    b = _Builder(20, 12)
    b.rect(0.2, 2.0, 16.0, 6.0)                 # block south of the upper corridor
    b.rect(0.2, 7.4, 13.0, 11.8)                 # rooms north of the corridor
    b.rect(5.0, 7.4, 9.0, 7.6, glass=True)       # glass partition (part of the corridor's north wall)
    b.clear(5.0, 7.6, 9.0, 11.0)                 # glass-walled meeting room behind the partition
    b.rect(16.0, 0.2, 19.8, 6.0)                 # block east of the vertical corridor
    b.clear(16.0, 0.2, 17.4, 6.0)               # vertical corridor (1.4 m) going south
    b.clear(10.0, 7.4, 10.9, 7.6)               # 0.9 m doorway into a room
    b.clear(9.2, 7.6, 12.6, 11.0)               # the room
    m = [((1.0, 6.7, 0.0), (16.7, 0.9, -np.pi / 2)),
         ((1.0, 6.7, 0.0), (10.9, 9.5, np.pi / 2)),
         ((16.7, 0.9, np.pi / 2), (1.0, 6.7, np.pi)),
         ((10.9, 9.5, -np.pi / 2), (16.7, 1.2, -np.pi / 2))]
    return Map("narrow_corridor", b.occ, b.glass, m)


def yard_loop():
    """30 x 20 m outdoor yard: a 3.4 m road around a central building, pallets on the verges."""
    b = _Builder(30, 20)
    b.rect(4.0, 4.0, 26.0, 16.0)                 # central building
    b.rect(10.0, 0.2, 11.0, 0.5)                 # pallets along the outer fence (do not block the road)
    b.rect(19.0, 19.5, 20.0, 19.8)
    m = [((2.0, 2.0, 0.0), (28.0, 18.0, np.pi / 2)),
         ((15.0, 2.0, 0.0), (15.0, 18.0, np.pi)),
         ((28.0, 2.0, np.pi / 2), (2.0, 18.0, np.pi)),
         ((2.0, 10.0, -np.pi / 2), (28.0, 10.0, np.pi / 2))]
    return Map("yard_loop", b.occ, b.glass, m)


MAPS = {"warehouse_aisles": warehouse_aisles, "narrow_corridor": narrow_corridor, "yard_loop": yard_loop}


def get(name):
    return MAPS[name]()
