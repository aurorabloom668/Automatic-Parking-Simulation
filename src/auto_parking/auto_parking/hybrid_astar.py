"""从狭窄目标向外搜索的 Hybrid A*，输出可前进/倒车揉库的执行路径。"""

import heapq
import math
import time
from dataclasses import dataclass, replace

from . import collision, parking_strategy, reeds_shepp, vehicle
from .path_geometry import PathPoint

STEER_FRACS = (-1.0, -0.5, 0.0, 0.5, 1.0)
STEER_CHANGE_COST = 0.25
CLEARANCE_PREFERENCE = 0.45
CLEARANCE_COST = 2.0
HEURISTIC_WEIGHT = 1.12
# 给实际控制器的横向/航向反馈留出转向余量，避免整段圆弧持续满舵。
PLANNING_STEER = 0.85 * vehicle.MAX_STEER
PLANNING_RADIUS = vehicle.WHEELBASE / math.tan(PLANNING_STEER)


@dataclass
class PlannerSettings:
    activation_radius: float = 12.0
    xy_resolution: float = 0.30
    yaw_resolution: float = math.radians(6)
    step: float = 0.60
    clearance: float = 0.12
    local_margin: float = 5.0
    max_time: float = 15.0
    max_expand: int = 40000
    position_tolerance: float = 0.08
    yaw_tolerance: float = math.radians(3)


@dataclass
class _Node:
    x: float
    y: float
    yaw: float
    gear: int
    steer: float
    g: float
    parent: int


@dataclass
class PlanResult:
    path: list
    ok: bool
    reason: str
    elapsed_ms: float
    expanded: int
    path_length: float
    gear_changes: int


def _key(x, y, yaw, gear, settings):
    bins = round(2 * math.pi / settings.yaw_resolution)
    return (round(x / settings.xy_resolution), round(y / settings.xy_resolution),
            round(vehicle.normalize_angle(yaw) / settings.yaw_resolution) % bins, gear)


def _integrate(x, y, yaw, gear, steer, length):
    curvature = math.tan(steer) / vehicle.WHEELBASE
    if abs(curvature) < 1e-8:
        return x + gear * length * math.cos(yaw), y + gear * length * math.sin(yaw), yaw, 0.0
    yaw1 = yaw + gear * curvature * length
    return (x + (math.sin(yaw1) - math.sin(yaw)) / curvature,
            y - (math.cos(yaw1) - math.cos(yaw)) / curvature,
            vehicle.normalize_angle(yaw1), curvature)


def _collides(x, y, yaw, obstacles, margin=0.12):
    return collision.pose_collides(x, y, yaw, obstacles, margin)


def _primitive(node, gear, steer, step):
    count = math.ceil(step / 0.05)
    return [PathPoint(*_integrate(node.x, node.y, node.yaw, gear, steer, step * i / count)[:3],
                      gear, math.tan(steer) / vehicle.WHEELBASE)
            for i in range(1, count + 1)]


def _inside_bounds(x, y, bounds):
    xmin, xmax, ymin, ymax = bounds
    return xmin <= x <= xmax and ymin <= y <= ymax


def _motion_cost(node, points, strategy, obstacles=()):
    """搜索原语与解析连接使用相同的长度、换挡、转向和贴障代价。"""
    x, y, gear, steer = node.x, node.y, node.gear, node.steer
    cost = 0.0
    for point in points:
        length = math.hypot(point.x - x, point.y - y)
        if length <= 1e-8:
            continue
        # 搜索从停车目标向外生长，执行路径反过来走，实际挡位取反。
        weight = (strategy.reverse_weight(point.x, point.y) if point.gear > 0
                  else strategy.forward_cost)
        if obstacles:
            gap = collision.pose_clearance(point.x, point.y, point.yaw, obstacles)
            weight += CLEARANCE_COST * max(0.0, 1.0 - gap / CLEARANCE_PREFERENCE)
        next_steer = math.atan(point.curvature * vehicle.WHEELBASE)
        cost += length * weight + STEER_CHANGE_COST * abs(next_steer - steer)
        if gear and point.gear != gear:
            cost += strategy.gear_change_cost
        x, y, gear, steer = point.x, point.y, point.gear, next_steer
    return cost


def _grid_heuristic(goal, obstacles, bounds):
    """局部二维距离场与 Reeds-Shepp 启发式共同引导搜索。"""
    res = 0.5
    ix0, ix1 = math.floor(bounds[0] / res), math.ceil(bounds[1] / res)
    iy0, iy1 = math.floor(bounds[2] / res), math.ceil(bounds[3] / res)
    blocked = {(ix, iy) for ix in range(ix0, ix1 + 1) for iy in range(iy0, iy1 + 1)
               if any(obs.clearance(ix * res, iy * res) <= 0.75 for obs in obstacles)}
    target = (round(goal[0] / res), round(goal[1] / res))
    blocked.discard(target)
    distances = {target: 0.0}
    heap = [(0.0, *target)]
    while heap:
        distance, ix, iy = heapq.heappop(heap)
        if distance > distances[(ix, iy)]:
            continue
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            cell = (ix + dx, iy + dy)
            if not (ix0 <= cell[0] <= ix1 and iy0 <= cell[1] <= iy1) or cell in blocked:
                continue
            next_distance = distance + res * math.hypot(dx, dy)
            if next_distance < distances.get(cell, float('inf')):
                distances[cell] = next_distance
                heapq.heappush(heap, (next_distance, *cell))
    return distances


def _try_analytic(node, goal, obstacles, strategy, bounds, settings):
    start = (node.x, node.y, node.yaw)
    candidates = [(_motion_cost(node, points, strategy), points)
                  for points in reeds_shepp.paths(start, goal, PLANNING_RADIUS, step=0.10)]
    best_cost, best_path = float('inf'), None
    for lower_cost, points in sorted(candidates, key=lambda pair: pair[0]):
        if lower_cost >= best_cost:
            break
        if not all(_inside_bounds(p.x, p.y, bounds) for p in points):
            continue
        # 先淘汰明显穿过邻车的连接，避免逐点约束与连续检查占满搜索预算。
        if any(_collides(p.x, p.y, p.yaw, obstacles, settings.clearance)
               for p in points[::5]):
            continue
        if all(not _collides(x, y, yaw, obstacles, settings.clearance)
               for x, y, yaw in collision.sampled_poses(points)):
            cost = _motion_cost(node, points, strategy, obstacles)
            if cost < best_cost:
                best_cost, best_path = cost, points
    return best_path


def _reconstruct(nodes, last, step):
    chain = []
    while last >= 0:
        chain.append(nodes[last])
        last = nodes[last].parent
    chain.reverse()
    first = chain[0]
    gear = chain[1].gear if len(chain) > 1 else 1
    path = [PathPoint(first.x, first.y, first.yaw, gear)]
    for a, b in zip(chain, chain[1:]):
        path.extend(_primitive(a, b.gear, b.steer, step))
    return path


def _finish(path, obstacles, strategy, settings, started, expanded):
    # 反转每条真实运动边；曲率仍是车辆转向曲率，不能随挡位一起取反。
    search_path = path
    path = [PathPoint(search_path[-1].x, search_path[-1].y, search_path[-1].yaw,
                      -search_path[-1].gear)]
    path.extend(PathPoint(a.x, a.y, a.yaw, -b.gear, b.curvature)
                for a, b in reversed(list(zip(search_path, search_path[1:]))))
    elapsed = (time.perf_counter() - started) * 1000
    end = path[-1]
    longitudinal, lateral, yaw = parking_strategy.parking_errors(
        (end.x, end.y, end.yaw), strategy.goal)
    if (abs(longitudinal) > settings.position_tolerance
            or abs(lateral) > settings.position_tolerance or abs(yaw) > settings.yaw_tolerance
            or not strategy.contains_body(end.x, end.y, end.yaw)):
        return PlanResult([], False, '终点位置、航向或车身包络不满足', elapsed, expanded, 0.0, 0)
    minimum = collision.path_min_clearance(path, obstacles)
    if minimum <= settings.clearance:
        return PlanResult([], False, '连续车身碰撞验证失败', elapsed, expanded, 0.0, 0)
    length = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(path, path[1:]))
    changes = sum(a.gear != b.gear for a, b in zip(path, path[1:]))
    return PlanResult(path, True, '局部 Hybrid A* / 前后调整入位', elapsed, expanded, length, changes)


def plan_local_to_spot(start, spot, obstacles, settings=None,
                       adjust=False, cancel=None):
    settings = settings or PlannerSettings()
    if not adjust and parking_strategy.distance_to_spot(*start[:2], spot) > settings.activation_radius:
        return PlanResult([], False, '请手动接近目标车位后开始', 0.0, 0, 0.0, 0)
    strategy = parking_strategy.from_spot(spot)
    if adjust:
        settings = replace(settings, xy_resolution=0.08, yaw_resolution=math.radians(2),
                           step=0.15, position_tolerance=0.04,
                           yaw_tolerance=math.radians(1.5))
    anchors = [start, spot.goal, *parking_strategy.spot_corners(spot)]
    margin = settings.local_margin
    bounds = (min(p[0] for p in anchors) - margin, max(p[0] for p in anchors) + margin,
              min(p[1] for p in anchors) - margin, max(p[1] for p in anchors) + margin)
    return plan(start, strategy.goal, obstacles, bounds, strategy, settings, cancel)


def plan(start, goal, obstacles, bounds, strategy, settings=None, cancel=None):
    settings = settings or PlannerSettings()
    started = time.perf_counter()
    if _collides(*start, obstacles, settings.clearance):
        return PlanResult([], False, '起点碰撞或间隙不足', 0.0, 0, 0.0, 0)
    if _collides(*goal, obstacles, settings.clearance):
        return PlanResult([], False, '终点碰撞或间隙不足', 0.0, 0, 0.0, 0)
    # 先展开车位内的可行出入方向，再连接当前位姿，避免在邻车封住的
    # 目标附近反复试错。Reeds-Shepp 连接精确落到当前位姿，不吸附起点。
    target = start
    grid = _grid_heuristic(target, obstacles, bounds)
    rs_cache = {}

    def heuristic(x, y, yaw):
        key = _key(x, y, yaw, 1, settings)[:3]
        if key not in rs_cache:
            rs_cache[key] = reeds_shepp.length((x, y, yaw), target, PLANNING_RADIUS)
        rs = min(strategy.forward_cost, strategy.reverse_cost) * rs_cache[key]
        return max(rs, grid.get((round(x / 0.5), round(y / 0.5)), 0.0))

    nodes = [_Node(*goal, 0, 0.0, 0.0, -1)]
    heap = [(heuristic(*goal), 0)]
    best = {_key(*goal, 0, settings): 0.0}
    expanded = 0
    while heap and expanded < settings.max_expand:
        if cancel is not None and cancel.is_set():
            return PlanResult([], False, '已取消', (time.perf_counter() - started) * 1000,
                              expanded, 0.0, 0)
        if time.perf_counter() - started > settings.max_time:
            break
        _, index = heapq.heappop(heap)
        node = nodes[index]
        if node.g > best.get(_key(node.x, node.y, node.yaw, node.gear, settings), float('inf')):
            continue
        expanded += 1
        longitudinal, lateral, yaw_error = parking_strategy.parking_errors(
            (node.x, node.y, node.yaw), target)
        if (math.hypot(longitudinal, lateral) < 1e-7 and abs(yaw_error) < 1e-7):
            path = _reconstruct(nodes, index, settings.step)
            return _finish(path, obstacles, strategy, settings, started, expanded)
        if expanded == 1 or expanded % 10 == 0:
            extra = _try_analytic(node, target, obstacles, strategy, bounds, settings)
            if extra:
                path = _reconstruct(nodes, index, settings.step)
                if len(path) == 1:
                    path[0].gear = extra[0].gear
                path.extend(extra[1:])
                return _finish(path, obstacles, strategy, settings, started, expanded)
        for gear in (1, -1):
            for frac in STEER_FRACS:
                steer = frac * PLANNING_STEER
                points = _primitive(node, gear, steer, settings.step)
                if not all(_inside_bounds(p.x, p.y, bounds) for p in points):
                    continue
                if any(_collides(p.x, p.y, p.yaw, obstacles, settings.clearance) for p in points):
                    continue
                point = points[-1]
                cost = node.g + _motion_cost(node, points, strategy, obstacles)
                key = _key(point.x, point.y, point.yaw, gear, settings)
                if cost >= best.get(key, float('inf')):
                    continue
                best[key] = cost
                nodes.append(_Node(point.x, point.y, point.yaw, gear, steer, cost, index))
                heapq.heappush(heap, (cost + HEURISTIC_WEIGHT * heuristic(point.x, point.y, point.yaw),
                                     len(nodes) - 1))
    elapsed = (time.perf_counter() - started) * 1000
    reason = '局部搜索超时' if elapsed >= settings.max_time * 1000 else '未找到无碰撞的前后调整路径'
    return PlanResult([], False, reason, elapsed, expanded, 0.0, 0)
