"""Log-odds occupancy grid, independent of ROS.

Each cell stores l = log(p / (1 - p)). A laser beam lowers l for every cell it
passes through (free) and raises l at the cell where it hits something
(occupied). Adding in log-odds space is the Bayes update for independent cells.
"""
import math

import numpy as np


def bresenham(x0, y0, x1, y1):
    """Integer grid cells on the line from (x0, y0) to (x1, y1), inclusive."""
    cells = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return cells
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def prob_to_logodds(p):
    return math.log(p / (1.0 - p))


class OccupancyGrid:

    def __init__(self, width_m=12.0, height_m=12.0, resolution=0.05, origin=(-1.0, -1.0),
                 p_hit=0.7, p_miss=0.4, l_min=-4.0, l_max=4.0):
        self.resolution = resolution
        self.origin = origin
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self.l_hit = prob_to_logodds(p_hit)
        self.l_miss = prob_to_logodds(p_miss)
        self.l_min, self.l_max = l_min, l_max
        self.logodds = np.zeros((self.height, self.width), dtype=np.float32)  # row = y
        self.observed = np.zeros((self.height, self.width), dtype=bool)

    # ------------------------------------------------------------------ coordinates
    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.width and 0 <= cy < self.height

    # ------------------------------------------------------------------ updates
    def integrate_beam(self, sx, sy, ex, ey, hit):
        """Update along one beam from sensor (sx, sy) to end point (ex, ey), world metres."""
        c0 = self.world_to_cell(sx, sy)
        c1 = self.world_to_cell(ex, ey)
        cells = bresenham(c0[0], c0[1], c1[0], c1[1])
        free, last = cells[:-1], cells[-1]
        for cx, cy in free:
            if self.in_bounds(cx, cy):
                self.logodds[cy, cx] += self.l_miss
                self.observed[cy, cx] = True
        if self.in_bounds(*last):
            self.logodds[last[1], last[0]] += self.l_hit if hit else self.l_miss
            self.observed[last[1], last[0]] = True

    def integrate_scan(self, sensor_x, sensor_y, sensor_yaw, ranges, angle_min,
                       angle_increment, range_min, range_max, max_use_range=None):
        limit = min(range_max, max_use_range or range_max)
        for i, r in enumerate(ranges):
            if math.isnan(r) or r < range_min:
                continue
            hit = math.isfinite(r) and r < limit
            dist = r if hit else limit  # no return: clear up to the usable range
            a = sensor_yaw + angle_min + i * angle_increment
            self.integrate_beam(sensor_x, sensor_y,
                                sensor_x + dist * math.cos(a), sensor_y + dist * math.sin(a), hit)
        np.clip(self.logodds, self.l_min, self.l_max, out=self.logodds)

    # ------------------------------------------------------------------ output
    def probabilities(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds))

    def to_ros_data(self):
        """Flat row-major list with -1 unknown, 0..100 occupancy, as nav_msgs expects."""
        occ = np.rint(self.probabilities() * 100.0).astype(np.int8)
        occ[~self.observed] = -1
        return occ.flatten().tolist()

    def to_pgm(self, path, occupied_thresh=0.65, free_thresh=0.25):
        """Write a map_server compatible PGM (free=254, occupied=0, unknown=205)."""
        prob = self.probabilities()
        img = np.full(prob.shape, 205, dtype=np.uint8)
        img[self.observed & (prob < free_thresh)] = 254
        img[self.observed & (prob > occupied_thresh)] = 0
        img = np.flipud(img)  # image row 0 is the top (max y)
        with open(path, 'wb') as f:
            f.write(f'P5\n{self.width} {self.height}\n255\n'.encode())
            f.write(img.tobytes())
