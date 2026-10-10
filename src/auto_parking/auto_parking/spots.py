"""地图先验车位：位置来自 park.world，不靠激光认车位。

空/满可用静态占位车，也可用在线障碍物/占用栅格刷新。
"""

import math
from dataclasses import dataclass

from . import collision, vehicle


@dataclass
class ParkingSpot:
    spot_id: int
    kind: str          # 'perpendicular' / 'parallel'
    center: tuple      # (x, y, yaw) 车位几何中心
    goal: tuple        # (x, y, yaw) 停好后的后轴中心
    occupied: bool
    label: str


# park.world 里 6 个 parking_spot 模型的中心。
# 垂直车位 yaw=0（车头朝东），侧方位 yaw=-pi/2（车头朝南，与 planned 一致）。
CANDIDATES = [
    (1, 'perpendicular', (-15.5053, 12.5311, 0.0), '垂直-中'),
    (2, 'perpendicular', (-15.5055, 9.4514, 0.0), '垂直-南'),
    (3, 'perpendicular', (-15.5002, 15.5545, 0.0), '垂直-北'),
    (4, 'parallel', (-10.6359, 4.2692, -math.pi / 2.0), '侧方-北'),
    (5, 'parallel', (-10.6385, -1.62928, -math.pi / 2.0), '侧方-中'),
    (6, 'parallel', (-10.6389, -7.5034, -math.pi / 2.0), '侧方-南'),
]


def _occupied(center, obstacles):
    """车位中心落入某辆占位车外包络（外扩 0.3 m）即视为已占用。"""
    cx, cy, _ = center
    for obs in obstacles:
        if isinstance(obs, collision.Circle):
            continue
        if obs.name == 'house_1':
            continue
        if obs.clearance(cx, cy) < 0.3:
            return True
    return False


def detect_spots(obstacles=None):
    obstacles = obstacles or collision.park_world_obstacles()
    spots = []
    for spot_id, kind, center, label in CANDIDATES:
        occupied = _occupied(center, obstacles)
        spots.append(ParkingSpot(
            spot_id=spot_id,
            kind=kind,
            center=center,
            goal=vehicle.goal_from_spot_center(*center),
            occupied=occupied,
            label=label,
        ))
    return spots


def nearest_free(spots, x, y):
    """距离 (x, y) 最近的空车位；没有空位返回 None。"""
    best, best_d = None, float('inf')
    for spot in spots:
        if spot.occupied:
            continue
        d = math.hypot(spot.center[0] - x, spot.center[1] - y)
        if d < best_d:
            best, best_d = spot, d
    return best
