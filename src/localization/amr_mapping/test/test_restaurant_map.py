"""tools/restaurant_map.py: 경로 계획용(로봇 키까지 전부) / 위치추정용(--scan-height, 스캔 평면만) 칠하기. ROS 없이 돈다."""

import importlib.util
import os

import numpy as np

TOOL = os.path.join(os.path.dirname(__file__), '..', 'tools', 'restaurant_map.py')
spec = importlib.util.spec_from_file_location('restaurant_map', TOOL)
rm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rm)

# 테이블 하나: 상판 상자(높이 0.70~0.75 m, 1.2 x 0.75 m)와 다리 4개(원기둥 r 0.03, 0~0.70 m)
TOP = (0.0, 0.0, 0.0, 0.6, 0.375, 0.70, 0.75, 'box')
LEGS = [(x, y, 0.0, 0.03, 0.03, 0.0, 0.70, 'cyl') for x in (-0.55, 0.55) for y in (-0.33, 0.33)]
LAMP = (3.0, 3.0, 0.0, 0.2, 0.2, 1.5, 2.0, 'box')      # 로봇 키보다 높다


def occupied(img, x, y):
    return img[rm.to_cell(x, y)] == rm.OCCUPIED


def test_planning_map_fills_table_top_and_skips_high_things():
    img, used = rm.rasterize([TOP, *LEGS, LAMP])
    assert used == 5
    assert occupied(img, 0.0, 0.0)            # 상판 밑 가운데도 점유 (경로가 테이블 밑으로 안 간다)
    assert not occupied(img, 3.0, 3.0)


def test_scan_map_keeps_only_what_the_lidar_plane_crosses():
    img, used = rm.rasterize([TOP, *LEGS, LAMP], scan_height=0.38)
    assert used == 4                           # 다리만
    assert not occupied(img, 0.0, 0.0)         # 상판 밑은 비어 있다 (LiDAR가 보는 대로)
    assert occupied(img, 0.55, 0.33)
    assert not occupied(img, 3.0, 3.0)


def test_scan_map_is_a_subset_of_the_planning_map():
    shapes = [TOP, *LEGS, LAMP]
    plan, _ = rm.rasterize(shapes)
    scan, _ = rm.rasterize(shapes, scan_height=0.38)
    assert not np.any((scan == rm.OCCUPIED) & (plan != rm.OCCUPIED))
