"""全程泊车任务规划：原点出发 -> 垂直倒库 -> 驶出南行 -> 侧方位揉库。

整条任务由直线和圆弧解析拼接，杜绝数值搜索的不确定性：

  阶段 A  出生点 (0,0,pi) 向西直行 -> 右转 90 度上北行车道 -> 沿车道北行 ->
          倒挡右打方向 90 度弧线 + 倒直线，进垂直车位。
  阶段 B  前进驶出车位 -> 右转 90 度拐上车道南行，停在侧方位空位前车
          （hatchback_blue）旁边，与其平行。
  阶段 C  先停在与前车平行的预停点，再倒车入位（朝南，车位在右侧）：
          倒挡打方向陡切入 -> 前进反打回正 -> 倒挡收尾把姿态完全回正 ->
          前进直线归位。

南口两只锥桶已从场景中去掉，因此可以顺着南行车道直接倒入，不再需要
三点式调头；路径段数和换挡次数都比朝北方案少一截。

阶段 C 的横向几何有闭式解：横向位移之和等于车道横距 d，同半径时化简为
cos(beta_min) = 1 - d / (2R)，陡切入角 = beta_min + delta，前进修正角 =
delta，闭合与 delta 取值无关；切入弧与收尾弧半径不同时用二分求根。

北行去垂直车位走 x=-7（右转上道时左前角的外甩才不会剐蹭侧方位的邻车），
侧方位南行走 x=-7.8；垂直倒库半径由北行车道反解
（R_perp = lane_north - goal_x - 倒直线长），三个阶段路径天然连续。
"""

import math
from dataclasses import dataclass, field

from . import collision, vehicle
from .path_geometry import FORWARD, REVERSE, build, split_by_gear

DEFAULTS = {
    'spawn': [0.0, 0.0, math.pi],
    'lane_north_x': -7.0,     # 阶段 A 北行车道
    'lane_par_x': -7.8,       # 阶段 B/C 南行车道
    'route_radius': 5.0,
    'perpendicular_spot': [-15.5053, 12.5311, 0.0],
    'parallel_spot': [-10.6385, -1.62928, -1.57185],
    'perp_final_straight': 2.5,
    # 朝南揉库离线扫参结果：全程对邻车最小间隙约 0.25 m
    'par_radius': 4.5,        # 陡切入与前进修正共用的半径
    'par_exit_radius': 4.2,   # 倒挡收尾回正的半径
    'par_entry_extra': 0.25,  # 前进揉库修正角
    # 倒车切入前先做的小角度 S 形"斜溜"预移角（0 关闭）
    'par_pre_angle': 0.0,
    'par_settle': 0.35,       # 前进归位直线长
    # 找不到前车时，预停点相对倒库起点再沿车头方向多走的距离
    'par_approach': 2.0,
    # 沿车头方向（南）的偏移，负值把整车往北挪，远离南侧邻车。
    'par_goal_shift': -0.25,
    'spot_length': 5.42,
    'spot_width': 2.50,
    'speed_cruise': 1.4,
    'speed_reverse': 0.8,
    'speed_fine': 0.6,
}


@dataclass
class Chunk:
    gear: int
    points: list
    speed_cap: float
    label: str = ''


@dataclass
class Mission:
    chunks: list
    path: list
    spawn: tuple
    perp_spot: tuple
    par_spot: tuple
    perp_goal: tuple
    par_goal: tuple
    notes: list = field(default_factory=list)


def _snap_yaw(yaw, tol=0.02):
    """世界文件里手摆的模型 yaw 有千分之几弧度的歪斜，规划时对齐到直角。"""
    snapped = round(yaw / (math.pi / 2.0)) * (math.pi / 2.0)
    return snapped if abs(vehicle.normalize_angle(yaw - snapped)) < tol else yaw


def _require(cond, msg):
    if not cond:
        raise ValueError(msg)


def _cfg(cfg=None):
    p = dict(DEFAULTS)
    if cfg:
        p.update({k: v for k, v in cfg.items() if v is not None})
    return p


def shift_parallel_goal(goal, cfg=None):
    """沿车头方向微调停稳点，负值远离入库方向的前车。"""
    shift = _cfg(cfg)['par_goal_shift']
    return (
        goal[0] + shift * math.cos(goal[2]),
        goal[1] + shift * math.sin(goal[2]),
        goal[2],
    )


def front_neighbor(goal, obstacles, max_along=12.0, max_lat=3.5):
    """车位前方那辆占位车（沿目标朝向、横向仍在同一列）。"""
    gx, gy, gyaw = goal
    hx, hy = math.cos(gyaw), math.sin(gyaw)
    best, best_s = None, 0.5
    for obs in obstacles:
        if not hasattr(obs, 'length') or getattr(obs, 'name', '') == 'house_1':
            continue
        dx, dy = obs.x - gx, obs.y - gy
        along = dx * hx + dy * hy
        lat = abs(-dx * hy + dy * hx)
        if 0.5 < along < max_along and lat < max_lat and along > best_s:
            best, best_s = obs, along
    return best


def _solve_parallel_angles(lane_offset, p):
    """三段弧横向闭合，返回 (beta_in, beta_min, r_in, r_out, delta, settle, pre)。"""
    r_in = p['par_radius']
    r_out = p['par_exit_radius']
    delta = p['par_entry_extra']
    pre = p['par_pre_angle']
    pre_shift = 2.0 * r_in * (1.0 - math.cos(pre))
    main_offset = lane_offset - pre_shift
    _require(main_offset > 0.3, f'预移角过大，剩余横距 {main_offset:.2f} m')

    def lateral_residual(b1):
        return (main_offset
                - r_in * (1.0 - math.cos(b1))
                + r_in * (math.cos(b1 - delta) - math.cos(b1))
                - r_out * (1.0 - math.cos(b1 - delta)))

    lo, hi = delta + 1e-4, 2.3
    _require(lateral_residual(lo) > 0.0 > lateral_residual(hi),
             f'横距 {lane_offset:.2f} m 超出半径 {r_in:.2f}/{r_out:.2f} m 揉库能力')
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if lateral_residual(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    beta_in = 0.5 * (lo + hi)
    return beta_in, beta_in - delta, r_in, r_out, delta, p['par_settle'], pre


def parallel_reverse_geometry(goal, cfg=None, lane_x=None):
    """车道上倒库起点 -> 目标位姿的几何揉库（不含驶向预停点）。

    局部 +X、车位在右侧：倒挡陡切入 -> 前进反打 -> 倒挡回正 -> 前进归位。
    起点已落在南行车道上，朝向与 goal 一致。
    """
    p = _cfg(cfg)
    lane_x = p['lane_par_x'] if lane_x is None else lane_x
    lane_offset = lane_x - goal[0]
    _require(lane_offset > 0.5, f'车道横距异常: {lane_offset:.2f} m')
    beta_in, beta_min, r_in, r_out, delta, settle, pre = _solve_parallel_angles(
        lane_offset, p)
    segments = []
    if pre > 0.01:
        segments += [('arc', r_in, pre, REVERSE), ('arc', r_in, -pre, REVERSE)]
    segments += [
        ('arc', r_in, beta_in, REVERSE),
        ('arc', r_in, -delta, FORWARD),
        ('arc', r_out, -beta_min, REVERSE),
    ]
    if settle > 0.05:
        segments.append(('straight', settle, FORWARD))
    from .path_geometry import transform_to_goal
    path = transform_to_goal(build((0.0, 0.0, 0.0), segments), goal)
    start = path[0]
    _require(abs(start.x - lane_x) < 0.08,
             f'倒库起点偏离车道: x={start.x:.2f}')
    return path, {
        'beta_in': beta_in,
        'beta_min': beta_min,
        'delta': delta,
        'reverse_start': (start.x, start.y, start.yaw),
    }


def parallel_staging_pose(goal, obstacles=None, cfg=None, lane_x=None):
    """与前车大致平行的预停点：同车道、同朝向，纵向对齐前车中心。"""
    obstacles = obstacles or collision.park_world_obstacles()
    p = _cfg(cfg)
    lane_x = p['lane_par_x'] if lane_x is None else lane_x
    rev, info = parallel_reverse_geometry(goal, p, lane_x)
    rsx, rsy, rsyaw = info['reverse_start']
    hx, hy = math.cos(rsyaw), math.sin(rsyaw)
    neighbor = front_neighbor(goal, obstacles)
    if neighbor is not None:
        along = (neighbor.x - rsx) * hx + (neighbor.y - rsy) * hy
        staging = (rsx + along * hx, rsy + along * hy, rsyaw)
        return staging, neighbor, rev, info
    extra = max(float(p.get('par_approach', 2.0)), 2.0)
    staging = (rsx + extra * hx, rsy + extra * hy, rsyaw)
    return staging, None, rev, info


def _xyyaw(pose):
    if hasattr(pose, 'x'):
        return pose.x, pose.y, getattr(pose, 'yaw', 0.0)
    return pose[0], pose[1], pose[2] if len(pose) > 2 else 0.0


def _connect_straight(start, end, step=0.12):
    """同朝向直线连接。end 在车头前方用前进，在后方用倒车。"""
    from .path_geometry import PathPoint
    sx, sy, yaw = _xyyaw(start)
    ex, ey, _ = _xyyaw(end)
    dx, dy = ex - sx, ey - sy
    dist = math.hypot(dx, dy)
    if dist < 0.08:
        return []
    proj = dx * math.cos(yaw) + dy * math.sin(yaw)
    gear = FORWARD if proj >= 0.0 else REVERSE
    n = max(2, int(round(dist / step)))
    return [
        PathPoint(sx + dx * i / n, sy + dy * i / n, yaw, gear, 0.0)
        for i in range(n + 1)
    ]


def parallel_entry_path(goal, obstacles=None, cfg=None, lane_x=None):
    """预停点（与前车平行）-> 倒车入位的完整几何路径。

    返回 (staging, path, info)。path[0] 就是 staging。
    """
    staging, neighbor, rev, info = parallel_staging_pose(
        goal, obstacles, cfg, lane_x)
    connect = _connect_straight(staging, info['reverse_start'])
    if connect:
        path = connect + rev[1:]
    else:
        path = rev
        path[0] = type(path[0])(
            staging[0], staging[1], staging[2], path[0].gear, path[0].curvature)
    info = dict(info)
    info['staging'] = staging
    info['neighbor'] = None if neighbor is None else neighbor.name
    return staging, path, info


def build_mission(cfg=None):
    p = _cfg(cfg)

    spawn = tuple(p['spawn'])
    lane_north = p['lane_north_x']
    lane_par = p['lane_par_x']
    route_r = p['route_radius']
    notes = []

    perp_spot = list(p['perpendicular_spot'])
    par_spot = list(p['parallel_spot'])
    perp_spot[2] = _snap_yaw(perp_spot[2])
    # 朝南入库：与 park.world 里侧方位车位轴线一致（约 -pi/2）
    par_spot[2] = _snap_yaw(par_spot[2])

    perp_goal = vehicle.goal_from_spot_center(*perp_spot)
    par_goal = shift_parallel_goal(vehicle.goal_from_spot_center(*par_spot), p)

    # ---- 阶段 C（先解，阶段 B 需要知道它的起点在车道上的哪个位置）----
    # 预停点与前车平行，再倒车入位；不再先前进斜着开进库。
    c_start, phase_c, par_info = parallel_entry_path(par_goal, cfg=p, lane_x=lane_par)
    notes.append(
        f'侧方位揉库(朝南): 陡切入角 {math.degrees(par_info["beta_in"]):.1f} deg, '
        f'前进修正角 {math.degrees(par_info["delta"]):.1f} deg, '
        f'预停点 y={c_start[1]:.2f}（与前车 {par_info["neighbor"] or "默认"} 平行）')

    # ---- 阶段 A：出生点 -> 北行车道 -> 垂直倒库 ----
    r_perp = lane_north - perp_goal[0] - p['perp_final_straight']
    _require(r_perp > vehicle.MIN_TURN_RADIUS + 0.2,
             f'垂直倒库半径 {r_perp:.2f} m 小于最小转弯半径')
    l1 = spawn[0] - lane_north - route_r
    stage_y = perp_goal[1] + r_perp
    l2 = stage_y - spawn[1] - route_r
    _require(l1 > 0.1 and l2 > 0.1, f'出生点到车道的几何不成立 l1={l1:.2f} l2={l2:.2f}')
    seg_a = [
        ('straight', l1, FORWARD),
        ('arc', route_r, -math.pi / 2.0, FORWARD),
        ('straight', l2, FORWARD),
        ('arc', r_perp, -math.pi / 2.0, REVERSE),
        ('straight', p['perp_final_straight'], REVERSE),
    ]
    phase_a = build(spawn, seg_a)
    end_a = phase_a[-1]
    _require(math.hypot(end_a.x - perp_goal[0], end_a.y - perp_goal[1]) < 0.02,
             '阶段 A 终点没有落在垂直车位目标位姿上')
    notes.append(
        f'垂直倒库: 倒车半径 {r_perp:.2f} m, '
        f'换向点 ({lane_north:.2f}, {stage_y:.2f})')

    # ---- 阶段 B：驶出车位 -> 南行至前车旁（不再调头）----
    e1 = lane_par - route_r - perp_goal[0]
    s1 = (perp_goal[1] - route_r) - c_start[1]
    _require(e1 > 0.1 and s1 > 0.1,
             f'阶段 B 几何不成立 e1={e1:.2f} s1={s1:.2f}')
    seg_b = [
        ('straight', e1, FORWARD),
        ('arc', route_r, -math.pi / 2.0, FORWARD),
        ('straight', s1, FORWARD),
    ]
    phase_b = build(perp_goal, seg_b)
    end_b = phase_b[-1]
    _require(math.hypot(end_b.x - c_start[0], end_b.y - c_start[1]) < 0.05,
             '阶段 B 终点没有接上侧方位预停点')

    full = phase_a + phase_b[1:] + phase_c[1:]

    # ---- 分挡位并配速 ----
    chunks = []
    labels = ['驶向车道并北行', '倒车入垂直车位', '驶出并南行至前车旁',
              '倒挡陡切入车位', '前进揉库回正', '倒挡收尾回正', '前进归位']
    for i, (gear, points) in enumerate(split_by_gear(full)):
        length = sum(math.hypot(b.x - a.x, b.y - a.y)
                     for a, b in zip(points, points[1:]))
        if gear == FORWARD:
            cap = p['speed_cruise'] if length > 5.0 else p['speed_fine']
        else:
            cap = p['speed_reverse'] if length > 3.0 else p['speed_fine']
        label = labels[i] if i < len(labels) else f'段 {i}'
        chunks.append(Chunk(gear, points, cap, label))
        notes.append(
            f'段 {i + 1} [{label}]: '
            f'{"前进" if gear == FORWARD else "倒车"} {length:.1f} m, '
            f'限速 {cap:.1f} m/s')

    return Mission(chunks, full, spawn, tuple(perp_spot), tuple(par_spot),
                   perp_goal, par_goal, notes)


def clearance_report(mission, obstacles=None):
    """全程路径对场景障碍物的最小间隙，返回 (最小间隙, 文本行列表)。"""
    obstacles = obstacles or collision.park_world_obstacles()
    overall, per_obstacle = collision.path_clearance(mission.path, obstacles)
    return overall, collision.format_report(per_obstacle)
