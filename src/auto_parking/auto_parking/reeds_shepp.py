"""Reeds-Shepp：带最小转弯半径、允许前进/倒车的最短姿态路径。

公式来自 Reeds & Shepp (1990) 的 48 字族，供 Hybrid A* 作启发式
和近终点解析连接。不依赖 NumPy。
"""

import math
from dataclasses import dataclass


def _mod2pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _left_straight_left(x, y, phi):
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if 0.0 <= t <= math.pi:
        v = _mod2pi(phi - t)
        if 0.0 <= v <= math.pi:
            return True, [t, u, v], ['L', 'S', 'L']
    return False, [], []


def _left_straight_right(x, y, phi):
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 ** 2
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return True, [t, u, v], ['L', 'S', 'R']
    return False, [], []


def _left_x_right_x_left(x, y, phi):
    u1, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 <= 4.0:
        a = math.acos(max(-1.0, min(1.0, 0.25 * u1)))
        t = _mod2pi(a + theta + math.pi / 2.0)
        u = _mod2pi(math.pi - 2.0 * a)
        v = _mod2pi(phi - t - u)
        return True, [t, -u, v], ['L', 'R', 'L']
    return False, [], []


def _left_x_right_left(x, y, phi):
    u1, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 <= 4.0:
        a = math.acos(max(-1.0, min(1.0, 0.25 * u1)))
        t = _mod2pi(a + theta + math.pi / 2.0)
        u = _mod2pi(math.pi - 2.0 * a)
        v = _mod2pi(-phi + t + u)
        return True, [t, -u, -v], ['L', 'R', 'L']
    return False, [], []


def _left_right_x_left(x, y, phi):
    u1, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if 1e-6 < u1 <= 4.0:
        u = math.acos(max(-1.0, min(1.0, 1.0 - u1 ** 2 * 0.125)))
        a = math.asin(max(-1.0, min(1.0, 2.0 * math.sin(u) / u1)))
        t = _mod2pi(-a + theta + math.pi / 2.0)
        v = _mod2pi(t - u - phi)
        return True, [t, u, -v], ['L', 'R', 'L']
    return False, [], []


def _left_right_x_left_right(x, y, phi):
    u1, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 <= 2.0:
        a = math.acos(max(-1.0, min(1.0, (u1 + 2.0) * 0.25)))
        t = _mod2pi(theta + a + math.pi / 2.0)
        u = _mod2pi(a)
        v = _mod2pi(phi - t + 2.0 * u)
        if t >= 0.0 and u >= 0.0 and v >= 0.0:
            return True, [t, u, -u, -v], ['L', 'R', 'L', 'R']
    return False, [], []


def _left_x_right_left_x_right(x, y, phi):
    u1, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u2 = (20.0 - u1 ** 2) / 16.0
    if 0.0 <= u2 <= 1.0 and u1 > 1e-6:
        u = math.acos(u2)
        a = math.asin(max(-1.0, min(1.0, 2.0 * math.sin(u) / u1)))
        t = _mod2pi(theta + a + math.pi / 2.0)
        v = _mod2pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return True, [t, -u, -u, v], ['L', 'R', 'L', 'R']
    return False, [], []


def _left_x_right90_straight_left(x, y, phi):
    u1, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 >= 2.0:
        u = math.sqrt(u1 ** 2 - 4.0) - 2.0
        a = math.atan2(2.0, math.sqrt(u1 ** 2 - 4.0))
        t = _mod2pi(theta + a + math.pi / 2.0)
        v = _mod2pi(t - phi + math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, -math.pi / 2.0, -u, -v], ['L', 'R', 'S', 'L']
    return False, [], []


def _left_straight_right90_x_left(x, y, phi):
    u1, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 >= 2.0:
        u = math.sqrt(u1 ** 2 - 4.0) - 2.0
        a = math.atan2(math.sqrt(u1 ** 2 - 4.0), 2.0)
        t = _mod2pi(theta - a + math.pi / 2.0)
        v = _mod2pi(t - phi - math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, u, math.pi / 2.0, -v], ['L', 'S', 'R', 'L']
    return False, [], []


def _left_x_right90_straight_right(x, y, phi):
    u1, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 >= 2.0:
        t = _mod2pi(theta + math.pi / 2.0)
        u = u1 - 2.0
        v = _mod2pi(phi - t - math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, -math.pi / 2.0, -u, -v], ['L', 'R', 'S', 'R']
    return False, [], []


def _left_straight_left90_x_right(x, y, phi):
    u1, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 >= 2.0:
        t = _mod2pi(theta)
        u = u1 - 2.0
        v = _mod2pi(phi - t - math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, u, math.pi / 2.0, -v], ['L', 'S', 'L', 'R']
    return False, [], []


def _left_x_right90_straight_left90_x_right(x, y, phi):
    u1, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if u1 >= 4.0:
        u = math.sqrt(u1 ** 2 - 4.0) - 4.0
        a = math.atan2(2.0, math.sqrt(u1 ** 2 - 4.0))
        t = _mod2pi(theta + a + math.pi / 2.0)
        v = _mod2pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return True, [t, -math.pi / 2.0, -u, -math.pi / 2.0, v], [
                'L', 'R', 'S', 'L', 'R']
    return False, [], []


_PATH_FUNCS = (
    _left_straight_left,
    _left_straight_right,
    _left_x_right_x_left,
    _left_x_right_left,
    _left_right_x_left,
    _left_right_x_left_right,
    _left_x_right_left_x_right,
    _left_x_right90_straight_left,
    _left_x_right90_straight_right,
    _left_straight_right90_x_left,
    _left_straight_left90_x_right,
    _left_x_right90_straight_left90_x_right,
)


def _reflect(ctypes):
    return [{'L': 'R', 'R': 'L'}.get(c, c) for c in ctypes]


def _candidates(x, y, phi):
    """归一化局部坐标下的全部 48 族候选 (lengths, ctypes, L)。"""
    found = []
    for func in _PATH_FUNCS:
        for fx, fy, fp, flip, refl in (
            (x, y, phi, False, False),
            (-x, y, -phi, True, False),
            (x, -y, -phi, False, True),
            (-x, -y, phi, True, True),
        ):
            ok, lengths, ctypes = func(fx, fy, fp)
            if not ok:
                continue
            if flip:
                lengths = [-s for s in lengths]
            if refl:
                ctypes = _reflect(ctypes)
            total = sum(abs(s) for s in lengths)
            if total < 1e-4:
                continue
            found.append((lengths, ctypes, total))
    return found


def _words(start, goal, radius):
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    maxc = 1.0 / radius
    x = (c * dx + s * dy) * maxc
    y = (-s * dx + c * dy) * maxc
    phi = _mod2pi(goal[2] - start[2])
    found = []
    for item in _candidates(x, y, phi):
        px = py = yaw = 0.0
        for distance, mode in zip(item[0], item[1]):
            px, py, yaw = _interpolate(distance, mode, 1.0, px, py, yaw)
        if math.hypot(px - x, py - y) < 1e-7 and abs(_mod2pi(yaw - phi)) < 1e-7:
            found.append(item)
    return sorted(found, key=lambda item: item[2]), maxc


def length(start, goal, radius):
    """无障碍最短 Reeds-Shepp 弧长（米）。"""
    if math.hypot(goal[0] - start[0], goal[1] - start[1]) < 1e-4:
        if abs(_mod2pi(goal[2] - start[2])) < 1e-4:
            return 0.0
    words, maxc = _words(start, goal, radius)
    if not words:
        return math.hypot(goal[0] - start[0], goal[1] - start[1])
    return words[0][2] / maxc


@dataclass
class RSPoint:
    x: float
    y: float
    yaw: float
    gear: int
    curvature: float = 0.0


def _interpolate(dist, mode, maxc, ox, oy, oyaw):
    if mode == 'S':
        return (
            ox + dist / maxc * math.cos(oyaw),
            oy + dist / maxc * math.sin(oyaw),
            oyaw,
        )
    ldx = math.sin(dist) / maxc
    if mode == 'L':
        ldy = (1.0 - math.cos(dist)) / maxc
        yaw = oyaw + dist
    else:
        ldy = (1.0 - math.cos(dist)) / -maxc
        yaw = oyaw - dist
    gdx = math.cos(-oyaw) * ldx + math.sin(-oyaw) * ldy
    gdy = -math.sin(-oyaw) * ldx + math.cos(-oyaw) * ldy
    return ox + gdx, oy + gdy, yaw


def _sample_word(start, word, maxc, step):
    lengths, ctypes, _ = word
    norm_step = step * maxc
    first_gear = 1 if next(s for s in lengths if abs(s) > 1e-8) > 0 else -1
    xs, ys, yaws, gears, curves = [0.0], [0.0], [0.0], [first_gear], [0.0]
    ox = oy = oyaw = 0.0
    for seg_len, mode in zip(lengths, ctypes):
        if abs(seg_len) < 1e-8:
            continue
        gear = 1 if seg_len > 0.0 else -1
        curvature = {'L': maxc, 'R': -maxc, 'S': 0.0}[mode]
        delta = norm_step if seg_len > 0.0 else -norm_step
        dist = delta
        while (seg_len > 0.0 and dist < seg_len) or (seg_len < 0.0 and dist > seg_len):
            x, y, yaw = _interpolate(dist, mode, maxc, ox, oy, oyaw)
            xs.append(x)
            ys.append(y)
            yaws.append(yaw)
            gears.append(gear)
            curves.append(curvature)
            dist += delta
        x, y, yaw = _interpolate(seg_len, mode, maxc, ox, oy, oyaw)
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
        gears.append(gear)
        curves.append(curvature)
        ox, oy, oyaw = x, y, yaw

    c0, s0 = math.cos(-start[2]), math.sin(-start[2])
    points = []
    for x, y, yaw, gear, curvature in zip(xs, ys, yaws, gears, curves):
        points.append(RSPoint(
            x=c0 * x + s0 * y + start[0],
            y=-s0 * x + c0 * y + start[1],
            yaw=_mod2pi(yaw + start[2]),
            gear=gear,
            curvature=curvature,
        ))
    return points


def paths(start, goal, radius, step=0.10):
    """所有真实到达 goal 的 RS 候选，按长度排序；不吸附末点。"""
    words, maxc = _words(start, goal, radius)
    for word in words:
        yield _sample_word(start, word, maxc, step)


def path(start, goal, radius, step=0.10):
    return next(paths(start, goal, radius, step), [])
