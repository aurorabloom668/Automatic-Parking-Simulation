"""统一的车身碰撞和间隙检查，接触、相交和包含都返回零间隙。"""

import math
from dataclasses import dataclass

from . import vehicle


@dataclass
class Circle:
    name: str
    x: float
    y: float
    radius: float

    def clearance(self, px, py):
        return max(0.0, math.hypot(px - self.x, py - self.y) - self.radius)


@dataclass
class Rect:
    name: str
    x: float
    y: float
    yaw: float
    length: float
    width: float

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + c * x - s * y, self.y + s * x + c * y)
                for x, y in ((self.length / 2, self.width / 2),
                             (self.length / 2, -self.width / 2),
                             (-self.length / 2, -self.width / 2),
                             (-self.length / 2, self.width / 2))]

    def clearance(self, px, py):
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.x, py - self.y
        lx = cos_y * dx + sin_y * dy
        ly = -sin_y * dx + cos_y * dy
        ex = max(abs(lx) - self.length / 2.0, 0.0)
        ey = max(abs(ly) - self.width / 2.0, 0.0)
        return math.hypot(ex, ey)


# park.world <state> 段的最终位姿。尺寸按各模型碰撞网格实测：
# hatchback 4.00 x 2.14（含后视镜）、suv 5.02 x 2.60（含后视镜）；
# 锥桶底座 0.5 m 见方但向上收窄，取保险杠接触高度的有效半径 0.20 m。
def park_world_obstacles():
    return [
        Rect('suv', -15.4511, 9.4142, 0.0, 5.02, 2.60),
        Rect('hatchback_red', -15.7045, 15.4810, 0.0, 4.00, 2.14),
        Rect('hatchback_blue', -10.6478, -7.4079, -1.5737, 4.00, 2.14),
        Rect('hatchback_blue_clone', -10.5897, 4.5084, -1.5737, 4.00, 2.14),
        Rect('house_1', -19.3040, -2.1549, -1.5676, 9.0, 9.0),
        Circle('cone_c', -12.5956, 7.4049, 0.20),
    ]


def point_segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def polygons_overlap(a, b):
    """凸多边形分离轴检查；边界接触也算相交。"""
    for polygon in (a, b):
        for p, q in zip(polygon[:2], polygon[1:3]):
            nx, ny = q[1] - p[1], p[0] - q[0]
            pa = [x * nx + y * ny for x, y in a]
            pb = [x * nx + y * ny for x, y in b]
            if max(pa) < min(pb) - 1e-9 or max(pb) < min(pa) - 1e-9:
                return False
    return True


def body_clearance(body, obstacle):
    edges = list(zip(body, body[1:] + body[:1]))
    if isinstance(obstacle, Circle):
        p = (obstacle.x, obstacle.y)
        crosses = [(b[0] - a[0]) * (p[1] - a[1])
                   - (b[1] - a[1]) * (p[0] - a[0]) for a, b in edges]
        if all(v <= 0.0 for v in crosses) or all(v >= 0.0 for v in crosses):
            return 0.0
        return max(0.0, min(point_segment_distance(p, a, b) for a, b in edges)
                   - obstacle.radius)
    other = obstacle.corners()
    if polygons_overlap(body, other):
        return 0.0
    other_edges = list(zip(other, other[1:] + other[:1]))
    return min(min(point_segment_distance(p, a, b) for p in other for a, b in edges),
               min(point_segment_distance(p, a, b) for p in body for a, b in other_edges))


def pose_clearance(x, y, yaw, obstacles):
    body = vehicle.footprint(x, y, yaw)[:4]
    cx = x + vehicle.REAR_AXLE_TO_CENTER * math.cos(yaw)
    cy = y + vehicle.REAR_AXLE_TO_CENTER * math.sin(yaw)
    reach = math.hypot(vehicle.LENGTH / 2, vehicle.WIDTH / 2)
    minimum = float('inf')
    for obs in obstacles:
        if obs.clearance(cx, cy) - reach >= minimum:
            continue
        minimum = min(minimum, body_clearance(body, obs))
        if minimum == 0.0:
            break
    return minimum


def pose_collides(x, y, yaw, obstacles, margin=0.10):
    cx = x + vehicle.REAR_AXLE_TO_CENTER * math.cos(yaw)
    cy = y + vehicle.REAR_AXLE_TO_CENTER * math.sin(yaw)
    reach = math.hypot(vehicle.LENGTH / 2, vehicle.WIDTH / 2)
    body = vehicle.footprint(x, y, yaw)[:4]
    for obs in obstacles:
        if obs.clearance(cx, cy) > reach + margin:
            continue
        if body_clearance(body, obs) <= margin:
            return True
    return False


def sampled_poses(path, spacing=0.10):
    """加密路径，限制后轴平移和最远车角转动之和不超过 spacing。"""
    if not path:
        return
    yield path[0].x, path[0].y, path[0].yaw
    reach = math.hypot(vehicle.WHEELBASE + vehicle.FRONT_OVERHANG, vehicle.WIDTH / 2)
    for a, b in zip(path, path[1:]):
        dyaw = vehicle.normalize_angle(b.yaw - a.yaw)
        travel = math.hypot(b.x - a.x, b.y - a.y) + reach * abs(dyaw)
        count = max(1, math.ceil(travel / spacing))
        for i in range(1, count + 1):
            t = i / count
            yield (a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                   vehicle.normalize_angle(a.yaw + t * dyaw))


def path_clearance(path, obstacles, spacing=0.10):
    """连续加密检查，返回全程和逐障碍物最小间隙；包含首末点。"""
    per_obstacle = {obs.name: (float('inf'), None) for obs in obstacles}
    for x, y, yaw in sampled_poses(path, spacing):
        body = vehicle.footprint(x, y, yaw)[:4]
        for obs in obstacles:
            best, _ = per_obstacle[obs.name]
            d = body_clearance(body, obs)
            if d < best:
                per_obstacle[obs.name] = (d, (x, y, yaw))
    overall = min((d for d, _ in per_obstacle.values()), default=float('inf'))
    return overall, per_obstacle


def path_min_clearance(path, obstacles, spacing=0.10):
    """在线流程只需整体间隙，利用宽相位跳过不可能更近的障碍。"""
    minimum = float('inf')
    for pose in sampled_poses(path, spacing):
        minimum = min(minimum, pose_clearance(*pose, obstacles))
        if minimum == 0.0:
            break
    return minimum


def format_report(per_obstacle):
    lines = []
    for name, (d, pose) in sorted(per_obstacle.items(), key=lambda kv: kv[1][0]):
        if pose is None or d > 3.0:
            continue
        lines.append(
            f'  {name}: 最小间隙 {d:.2f} m '
            f'(车辆位于 x={pose[0]:.2f} y={pose[1]:.2f})')
    return lines or ['  所有障碍物间隙 > 3 m']
