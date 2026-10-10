"""直线段与圆弧段拼接，生成带挡位信息的离散路径。

路径点参考点是后轴中心（base_link），与 vehicle 模块一致。
自行车模型：dx/ds = gear*cos(yaw)，dy/ds = gear*sin(yaw)，dyaw/ds = 曲率。
gear 为 +1 前进、-1 倒车；s 始终是正的弧长。
"""

import math
from dataclasses import dataclass

FORWARD = 1
REVERSE = -1


@dataclass
class PathPoint:
    x: float
    y: float
    yaw: float
    gear: int
    curvature: float = 0.0


def _sample_count(length, step):
    return max(1, int(round(length / step)))


def straight(start, length, gear, step=0.05):
    """从 start 沿当前朝向走一段直线，返回不含起点的路径点。"""
    x0, y0, yaw = start
    n = _sample_count(length, step)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    points = []
    for i in range(1, n + 1):
        s = length * i / n
        points.append(PathPoint(
            x0 + gear * cos_y * s,
            y0 + gear * sin_y * s,
            yaw, gear, 0.0))
    return points


def arc(start, radius, delta_yaw, gear, step=0.05):
    """从 start 走一段圆弧。

    radius 取正值；delta_yaw 是这段弧上朝向的带符号变化量，决定左转还是右转。
    采样用解析积分而不是欧拉递推，避免几十厘米级的累积误差影响停车精度。
    """
    x0, y0, yaw0 = start
    length = radius * abs(delta_yaw)
    curvature = math.copysign(1.0 / radius, delta_yaw)
    n = _sample_count(length, step)
    scale = gear / curvature
    points = []
    for i in range(1, n + 1):
        s = length * i / n
        yaw = yaw0 + curvature * s
        points.append(PathPoint(
            x0 + scale * (math.sin(yaw) - math.sin(yaw0)),
            y0 - scale * (math.cos(yaw) - math.cos(yaw0)),
            yaw, gear, curvature))
    return points


def build(start, segments, step=0.05):
    """按顺序拼接若干段，返回含起点的完整路径。

    segments 每项为 ("straight", length, gear) 或 ("arc", radius, delta_yaw, gear)。
    """
    x, y, yaw = start
    first_gear = segments[0][-1] if segments else FORWARD
    path = [PathPoint(x, y, yaw, first_gear, 0.0)]
    for segment in segments:
        kind = segment[0]
        if kind == 'straight':
            _, length, gear = segment
            path.extend(straight((x, y, yaw), length, gear, step))
        elif kind == 'arc':
            _, radius, delta_yaw, gear = segment
            path.extend(arc((x, y, yaw), radius, delta_yaw, gear, step))
        else:
            raise ValueError(f'unknown segment type: {kind}')
        last = path[-1]
        x, y, yaw = last.x, last.y, last.yaw
    return path


def transform_to_goal(path, goal):
    """整体刚体变换路径，使其终点落在 goal 上。

    先在局部坐标系里按段拼出形状，再一次性搬到世界坐标，
    比反向解析推导起点要简单得多，且两种泊车方式通用。
    """
    if not path:
        return []
    end = path[-1]
    gx, gy, gyaw = goal
    delta = gyaw - end.yaw
    cos_d, sin_d = math.cos(delta), math.sin(delta)
    moved = []
    for point in path:
        dx, dy = point.x - end.x, point.y - end.y
        moved.append(PathPoint(
            x=gx + cos_d * dx - sin_d * dy,
            y=gy + sin_d * dx + cos_d * dy,
            yaw=point.yaw + delta,
            gear=point.gear,
            curvature=point.curvature,
        ))
    return moved


def split_by_gear(path):
    """按挡位切分路径，返回 [(gear, points), ...]。

    换挡处把交界点同时放进前后两段，保证条带视觉上连续；
    每段的 gear 是该段实际的行驶方向，换挡点即需要停车换向的位置。
    """
    if not path:
        return []
    chunks = [(path[0].gear, [path[0]])]
    for point in path[1:]:
        gear, points = chunks[-1]
        if point.gear == gear:
            points.append(point)
        else:
            chunks.append((point.gear, [points[-1], point]))
    return chunks
