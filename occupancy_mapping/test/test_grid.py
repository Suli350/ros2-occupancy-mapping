import math
import os

import numpy as np
import pytest

from occupancy_mapping.grid import OccupancyGrid, bresenham, prob_to_logodds


def test_bresenham_endpoints_and_continuity():
    for x1, y1 in [(5, 2), (-3, 7), (0, -4), (6, 6), (0, 0)]:
        cells = bresenham(0, 0, x1, y1)
        assert cells[0] == (0, 0)
        assert cells[-1] == (x1, y1)
        for (ax, ay), (bx, by) in zip(cells, cells[1:]):
            assert max(abs(ax - bx), abs(ay - by)) == 1


def test_logodds():
    assert prob_to_logodds(0.5) == pytest.approx(0.0)
    assert prob_to_logodds(0.7) > 0 > prob_to_logodds(0.4)


def test_world_to_cell():
    g = OccupancyGrid(width_m=2.0, height_m=2.0, resolution=0.1, origin=(-1.0, -1.0))
    assert (g.width, g.height) == (20, 20)
    assert g.world_to_cell(-1.0, -1.0) == (0, 0)
    assert g.world_to_cell(0.05, 0.05) == (10, 10)


def test_single_beam_marks_free_then_occupied():
    g = OccupancyGrid(width_m=4.0, height_m=4.0, resolution=0.1, origin=(0.0, 0.0))
    for _ in range(8):
        g.integrate_beam(0.55, 2.05, 2.55, 2.05, hit=True)
    p = g.probabilities()
    hit = g.world_to_cell(2.55, 2.05)
    before = g.world_to_cell(1.5, 2.05)
    behind = g.world_to_cell(3.5, 2.05)
    assert p[hit[1], hit[0]] > 0.9
    assert p[before[1], before[0]] < 0.1
    assert not g.observed[behind[1], behind[0]]


def test_scan_of_a_circular_room():
    # Sensor in the middle of a 2 m radius room: the ring should be occupied.
    g = OccupancyGrid(width_m=6.0, height_m=6.0, resolution=0.05, origin=(-3.0, -3.0))
    beams = 360
    for _ in range(3):
        g.integrate_scan(0.0, 0.0, 0.0, [2.0] * beams, -math.pi, 2 * math.pi / beams, 0.1, 6.0)
    p = g.probabilities()
    cx, cy = g.world_to_cell(2.0, 0.0)
    ox, oy = g.world_to_cell(1.0, 0.0)
    assert p[cy, cx] > 0.8
    assert p[oy, ox] < 0.2
    data = g.to_ros_data()
    assert len(data) == g.width * g.height
    assert min(data) == -1 and max(data) <= 100


def test_inf_ranges_clear_space_but_add_no_hits():
    g = OccupancyGrid(width_m=6.0, height_m=6.0, resolution=0.1, origin=(-3.0, -3.0))
    g.integrate_scan(0.0, 0.0, 0.0, [float('inf')] * 36, -math.pi, 2 * math.pi / 36, 0.1, 6.0,
                     max_use_range=2.0)
    assert (g.probabilities()[g.observed] < 0.5).all()


def test_pgm_export(tmp_path):
    g = OccupancyGrid(width_m=1.0, height_m=0.5, resolution=0.1, origin=(0.0, 0.0))
    g.integrate_beam(0.05, 0.25, 0.85, 0.25, hit=True)
    path = os.path.join(tmp_path, 'm.pgm')
    g.to_pgm(path)
    raw = open(path, 'rb').read()
    header = b'P5\n10 5\n255\n'
    assert raw.startswith(header)
    pixels = np.frombuffer(raw[len(header):], dtype=np.uint8)
    assert pixels.size == 50
    assert set(np.unique(pixels)) <= {0, 205, 254}
