"""用 3D 激光做占用栅格：抽障碍、刷新已知车位是否被占。

车位坐标是地图先验，这里不负责发现车位。
"""

import math
from dataclasses import dataclass

from . import collision, vehicle


GRID_RES = 0.25
# 覆盖六个车位的 12 m 准入范围、5 m 搜索余量和完整车身。
BOUNDS = (-40.0, 12.0, -32.0, 40.0)
Z_MIN = 0.22
Z_MAX = 2.20
HIT_THRESH = 1
CLOUD_STRIDE = 4

SPOT_LENGTH = 5.42
SPOT_WIDTH = 2.50


@dataclass
class DetectResult:
    obstacles: list
    occupied_cells: int = 0
    grid: tuple = None


def parse_cloud_xy(msg, stride=CLOUD_STRIDE):
    """从 PointCloud2 抽 (x,y,z)，不经过 NumPy。"""
    fields = {f.name: f for f in msg.fields}
    if 'x' not in fields or 'y' not in fields or 'z' not in fields:
        return []
    off_x, off_y, off_z = fields['x'].offset, fields['y'].offset, fields['z'].offset
    step = msg.point_step
    data = msg.data
    n = len(data) // step
    points = []
    for i in range(0, n, max(1, stride)):
        base = i * step
        if base + off_z + 4 > len(data):
            break
        x = struct_f32(data, base + off_x)
        y = struct_f32(data, base + off_y)
        z = struct_f32(data, base + off_z)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        points.append((x, y, z))
    return points


def struct_f32(data, offset):
    import struct
    return struct.unpack_from('<f', data, offset)[0]


def transform_xy(points, tx, ty, tz, qw, qx, qy, qz):
    """激光系点 -> 世界 XY。四元数按 (x,y,z,w) 旋转。"""
    # 归一化
    nrm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    qw, qx, qy, qz = qw / nrm, qx / nrm, qy / nrm, qz / nrm
    out = []
    for x, y, z in points:
        # v' = q * v * q^{-1}
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
        ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
        rz = iz * qw - iw * qz - ix * qy + iy * qx
        if Z_MIN <= rz + tz <= Z_MAX:
            out.append((rx + tx, ry + ty))
    return out


def _grid_shape():
    xmin, xmax, ymin, ymax = BOUNDS
    cols = int(math.ceil((xmax - xmin) / GRID_RES))
    rows = int(math.ceil((ymax - ymin) / GRID_RES))
    return xmin, ymin, cols, rows


def build_grid(points_xy):
    xmin, ymin, cols, rows = _grid_shape()
    hits = [0] * (cols * rows)
    for x, y in points_xy:
        ix = math.floor((x - xmin) / GRID_RES)
        iy = math.floor((y - ymin) / GRID_RES)
        if 0 <= ix < cols and 0 <= iy < rows:
            hits[iy * cols + ix] += 1
    occupied = [h >= HIT_THRESH for h in hits]
    return occupied, xmin, ymin, cols, rows


def _grid_obstacles(grid):
    """只合并相邻的已占用格；L/U 形点簇内部的空格仍可通行。"""
    occupied, xmin, ymin, cols, rows = grid
    runs, active = [], {}
    for iy in range(rows):
        row_runs = {}
        ix = 0
        while ix < cols:
            if not occupied[iy * cols + ix]:
                ix += 1
                continue
            start = ix
            while ix < cols and occupied[iy * cols + ix]:
                ix += 1
            key = (start, ix)
            first = active.pop(key, iy)
            row_runs[key] = first
        runs.extend((a, b, first, iy) for (a, b), first in active.items())
        active = row_runs
    runs.extend((a, b, first, rows) for (a, b), first in active.items())
    return [collision.Rect(f'laser_{i}', xmin + (a + b) * GRID_RES / 2,
                           ymin + (first + last) * GRID_RES / 2, 0.0,
                           (b - a) * GRID_RES, (last - first) * GRID_RES)
            for i, (a, b, first, last) in enumerate(runs)]


def _spot_blocked(cx, cy, yaw, occupied, xmin, ymin, cols, rows):
    """车位矩形内占用格子过多则不是空位。"""
    hx, hy = math.cos(yaw), math.sin(yaw)
    hits = total = 0
    for sl in (-0.35, -0.15, 0.0, 0.15, 0.35):
        for sw in (-0.35, -0.15, 0.0, 0.15, 0.35):
            x = cx + sl * SPOT_LENGTH * hx - sw * SPOT_WIDTH * hy
            y = cy + sl * SPOT_LENGTH * hy + sw * SPOT_WIDTH * hx
            ix = int((x - xmin) / GRID_RES)
            iy = int((y - ymin) / GRID_RES)
            total += 1
            if 0 <= ix < cols and 0 <= iy < rows and occupied[iy * cols + ix]:
                hits += 1
    return total > 0 and hits / total > 0.12


def detect_from_points(points_xy, ego=None, static_obstacles=()):
    """世界系 XY 点 -> 局部障碍物，只排除本车轮廓，不发现新车位。"""
    if ego is not None:
        ex, ey, yaw = ego
        c, s = math.cos(yaw), math.sin(yaw)
        points_xy = [p for p in points_xy if not (
            -vehicle.REAR_OVERHANG - 0.1 <= (p[0] - ex) * c + (p[1] - ey) * s
            <= vehicle.WHEELBASE + vehicle.FRONT_OVERHANG + 0.1
            and abs(-(p[0] - ex) * s + (p[1] - ey) * c) <= vehicle.WIDTH / 2 + 0.1)]

    grid = build_grid(points_xy)
    # 地图已经保留完整邻车；其表面回波不再量化成向外膨胀的第二辆车。
    # 0.03 m 对应本项目激光 0.01 m 高斯噪声的三倍；地图外的新障碍仍保留。
    new_points = [p for p in points_xy
                  if not any(obs.clearance(*p) <= 0.03 for obs in static_obstacles)]
    obstacles = _grid_obstacles(build_grid(new_points))
    return DetectResult(obstacles, sum(grid[0]), grid)


def mark_occupied(spot_list, grid):
    """用占用栅格刷新地图车位的空/满，不改坐标。"""
    if not grid:
        return spot_list
    occupied, xmin, ymin, cols, rows = grid
    for spot in spot_list:
        cx, cy, yaw = spot.center
        spot.occupied = _spot_blocked(
            cx, cy, yaw, occupied, xmin, ymin, cols, rows)
    return spot_list
