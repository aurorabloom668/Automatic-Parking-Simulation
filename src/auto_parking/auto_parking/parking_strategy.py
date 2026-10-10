"""已知车位的附近准入、前后运动代价和真实终点验收。"""

import math
from dataclasses import dataclass

from . import collision, vehicle

SPOT_LENGTH = 5.42
SPOT_WIDTH = 2.50


def spot_corners(spot):
    return collision.Rect('slot', *spot.center, SPOT_LENGTH, SPOT_WIDTH).corners()


def distance_to_spot(x, y, spot):
    """后轴定位点到车位矩形边界的最短距离；位于车位内时为零。"""
    return collision.Rect('slot', *spot.center, SPOT_LENGTH, SPOT_WIDTH).clearance(x, y)


def body_in_spot(pose, spot):
    cx, cy, yaw = spot.center
    c, s = math.cos(yaw), math.sin(yaw)
    return all(abs((x - cx) * c + (y - cy) * s) <= SPOT_LENGTH / 2 + 1e-9
               and abs(-(x - cx) * s + (y - cy) * c) <= SPOT_WIDTH / 2 + 1e-9
               for x, y in vehicle.footprint(*pose)[:4])


def parking_errors(pose, goal):
    dx, dy = pose[0] - goal[0], pose[1] - goal[1]
    c, s = math.cos(goal[2]), math.sin(goal[2])
    return dx * c + dy * s, -dx * s + dy * c, vehicle.normalize_angle(pose[2] - goal[2])


def parking_complete(pose, speed, spot, obstacles, position_tolerance=0.10,
                     yaw_tolerance=math.radians(3), stop_speed=0.05, clearance=0.10):
    longitudinal, lateral, yaw = parking_errors(pose, spot.goal)
    return (abs(longitudinal) <= position_tolerance
            and abs(lateral) <= position_tolerance and abs(yaw) <= yaw_tolerance
            and abs(speed) <= stop_speed and body_in_spot(pose, spot)
            and collision.pose_clearance(*pose, obstacles) >= clearance)


@dataclass
class ParkingStrategy:
    kind: str
    goal: tuple
    spot_center: tuple
    half_length: float
    half_width: float
    forward_cost: float = 1.0
    reverse_cost: float = 0.95
    reverse_cost_far: float = 1.10
    gear_change_cost: float = 1.2
    near_goal: float = 8.0

    def in_spot(self, x, y):
        cx, cy, yaw = self.spot_center
        dx, dy = x - cx, y - cy
        along = dx * math.cos(yaw) + dy * math.sin(yaw)
        lat = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return abs(along) < self.half_length and abs(lat) < self.half_width

    def contains_body(self, x, y, yaw):
        return all(self.in_spot(px, py) for px, py in vehicle.footprint(x, y, yaw)[:4])

    def reverse_weight(self, x, y):
        if self.in_spot(x, y):
            return self.reverse_cost
        if math.hypot(self.goal[0] - x, self.goal[1] - y) < self.near_goal:
            return self.reverse_cost
        return self.reverse_cost_far


def parking_goal(spot, obstacles=None):
    """最终停车姿态：与画线平行，垂直车头朝通道（和邻车一致）。"""
    cx, cy, yaw = spot.center
    if spot.kind == 'perpendicular':
        aisle = _infer_aisle(spot, obstacles)
        hx, hy = math.cos(yaw), math.sin(yaw)
        if hx * aisle[0] + hy * aisle[1] < 0.0:
            yaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))
    return vehicle.goal_from_spot_center(cx, cy, yaw)


def _infer_aisle(spot, obstacles):
    """垂直车位的通道在车头/车尾方向中较空的一侧。"""
    cx, cy, yaw = spot.center
    hx, hy = math.cos(yaw), math.sin(yaw)
    dirs = ((hx, hy), (-hx, -hy))

    def ray_clear(sx, sy):
        best = 10.0
        for dist in (2.0, 3.5, 5.5):
            px, py = cx + dist * sx, cy + dist * sy
            for obs in obstacles or []:
                best = min(best, obs.clearance(px, py))
        return best

    a, b = dirs
    ca, cb = ray_clear(*a), ray_clear(*b)
    if abs(ca - cb) < 1.2:
        da = math.hypot(cx + 7.0 * a[0], cy + 7.0 * a[1])
        db = math.hypot(cx + 7.0 * b[0], cy + 7.0 * b[1])
        return a if da < db else b
    return a if ca >= cb else b


def from_spot(spot):
    return ParkingStrategy(
        kind=spot.kind, goal=spot.goal, spot_center=spot.center,
        half_length=SPOT_LENGTH / 2, half_width=SPOT_WIDTH / 2,
        reverse_cost_far=1.15 if spot.kind == 'parallel' else 1.10,
    )
