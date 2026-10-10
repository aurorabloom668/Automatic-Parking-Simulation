"""把规划路径画成与车同宽的浅蓝色条带，以及车位框、目标车身轮廓。

条带用 TRIANGLE_LIST 而不是 LINE_STRIP：LINE_STRIP 在 RViz 里是 billboard，
会始终朝向相机，斜视时明显失真；三角带是真正贴地的多边形。
"""

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from . import parking_strategy, vehicle
from .path_geometry import REVERSE, split_by_gear

FORWARD_COLOR = ColorRGBA(r=0.35, g=0.78, b=1.00, a=0.85)
REVERSE_COLOR = ColorRGBA(r=0.15, g=0.50, b=0.95, a=0.90)
SPOT_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
GOAL_COLOR = ColorRGBA(r=0.30, g=1.00, b=0.60, a=0.9)
SHIFT_COLOR = ColorRGBA(r=1.00, g=0.85, b=0.20, a=0.95)

GROUND_Z = 0.08


def _point(x, y, z=GROUND_Z):
    return Point(x=float(x), y=float(y), z=float(z))


def _edges(point, half_width):
    """路径点左右两侧的边界点。"""
    nx = -math.sin(point.yaw) * half_width
    ny = math.cos(point.yaw) * half_width
    return (point.x + nx, point.y + ny), (point.x - nx, point.y - ny)


def _blank(frame_id, stamp, namespace, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.frame_locked = True
    return marker


def path_ribbon(path, frame_id, stamp, marker_id=0,
                width=vehicle.WIDTH, namespace='planned_path'):
    """路径 -> 车宽浅蓝条带。倒车段颜色更深。

    每个四边形发正反两面三角形：RViz 对 TRIANGLE_LIST 做背面剔除，
    只发一面时从俯视相机看会完全消失。同时写 marker.color，避免
    仅用 per-vertex colors 时部分 RViz 版本把 alpha 当成 0。
    """
    marker = _blank(frame_id, stamp, namespace, marker_id, Marker.TRIANGLE_LIST)
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color = FORWARD_COLOR

    half = width / 2.0
    for gear, points in split_by_gear(path):
        color = REVERSE_COLOR if gear == REVERSE else FORWARD_COLOR
        for current, nxt in zip(points, points[1:]):
            (lx0, ly0), (rx0, ry0) = _edges(current, half)
            (lx1, ly1), (rx1, ry1) = _edges(nxt, half)
            # 从 +Z 看为逆时针，法线朝上；再补一面朝下，俯视/仰视都可见
            top = [
                _point(lx0, ly0), _point(lx1, ly1), _point(rx0, ry0),
                _point(lx1, ly1), _point(rx1, ry1), _point(rx0, ry0),
            ]
            bottom = [
                _point(lx0, ly0), _point(rx0, ry0), _point(lx1, ly1),
                _point(lx1, ly1), _point(rx0, ry0), _point(rx1, ry1),
            ]
            marker.points.extend(top)
            marker.points.extend(bottom)
            marker.colors.extend([color] * (len(top) + len(bottom)))
    return marker


def polyline(points, frame_id, stamp, marker_id, color,
             namespace='outline', line_width=0.06, z=GROUND_Z):
    marker = _blank(frame_id, stamp, namespace, marker_id, Marker.LINE_STRIP)
    marker.scale.x = line_width
    marker.color = color
    marker.points = [_point(x, y, z) for x, y in points]
    return marker


def spot_outline(cx, cy, yaw, frame_id, stamp, marker_id,
                 length=5.42, width=2.50, namespace='parking_spot'):
    """车位白框，尺寸默认与 park.world 里 parking_spot 模型一致。"""
    half_l, half_w = length / 2.0, width / 2.0
    corners = [
        (half_l, half_w), (half_l, -half_w),
        (-half_l, -half_w), (-half_l, half_w), (half_l, half_w),
    ]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    world = [
        (cx + lx * cos_y - ly * sin_y, cy + lx * sin_y + ly * cos_y)
        for lx, ly in corners
    ]
    return polyline(world, frame_id, stamp, marker_id, SPOT_COLOR,
                    namespace=namespace, line_width=0.08)


def goal_footprint(x, y, yaw, frame_id, stamp, marker_id,
                   namespace='goal_footprint'):
    """停好后的车身轮廓，用来直观检查车位余量。"""
    return polyline(vehicle.footprint(x, y, yaw), frame_id, stamp, marker_id,
                    GOAL_COLOR, namespace=namespace, line_width=0.07,
                    z=GROUND_Z + 0.01)


def spot_label(text, x, y, frame_id, stamp, marker_id,
               color=SPOT_COLOR, namespace='spot_label', z=1.4):
    """车位编号文字，方便在 RViz 里对照点选。"""
    marker = _blank(frame_id, stamp, namespace, marker_id, Marker.TEXT_VIEW_FACING)
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = float(z)
    marker.scale.z = 0.55
    marker.color = color
    marker.text = text
    return marker


def shift_points(path, frame_id, stamp, marker_id, namespace='gear_shift'):
    """换挡点（需要停车换向的位置），用黄色小球标出。"""
    marker = _blank(frame_id, stamp, namespace, marker_id, Marker.SPHERE_LIST)
    marker.scale.x = 0.35
    marker.scale.y = 0.35
    marker.scale.z = 0.35
    marker.color = SHIFT_COLOR
    for _, points in split_by_gear(path)[1:]:
        marker.points.append(_point(points[0].x, points[0].y, GROUND_Z + 0.2))
    return marker


def parking_area(spot, radius, ready, frame_id, stamp):
    """车位矩形向外扩 radius 的准入边界，没有固定预停框或朝向。"""
    color = GOAL_COLOR if ready else SHIFT_COLOR
    cx, cy, yaw = spot.center
    half_l, half_w = parking_strategy.SPOT_LENGTH / 2, parking_strategy.SPOT_WIDTH / 2
    points = []
    for x, y, angle in ((half_l, half_w, 0), (-half_l, half_w, math.pi / 2),
                        (-half_l, -half_w, math.pi), (half_l, -half_w, 3 * math.pi / 2)):
        for i in range(17):
            theta = angle + i * math.pi / 32
            px, py = x + radius * math.cos(theta), y + radius * math.sin(theta)
            points.append((cx + px * math.cos(yaw) - py * math.sin(yaw),
                           cy + px * math.sin(yaw) + py * math.cos(yaw)))
    boundary = polyline(points + [points[0]], frame_id, stamp,
                        60, color, namespace='parking_area', line_width=0.08)
    text = '可开始自动泊车' if ready else f'车位附近 {radius:g} m 内停稳后开始'
    label = spot_label(text, cx, cy, frame_id, stamp, 62,
                       color=color, namespace='parking_area', z=2.4)
    return [boundary, label]
