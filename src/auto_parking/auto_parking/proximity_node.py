#!/usr/bin/env python3
"""用 12 路超声波在 RViz 上标近距障碍。

小于 0.5 m：黄色数字（cm）+ 两道同心弧；小于 0.10 m 改红色。
每个方向（前/后/左/右）只取最近一只，5 Hz 发布。

Gazebo 无回波时 Range.range 是 +inf / FLT_MAX，Humble 反序列化会抛
RuntimeError。这里用 raw 订阅，自己解 CDR，避开那条转换路径。
"""

import math
import struct
from collections import namedtuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

SONARS = (
    ('/prius/front_sonar/left_far_range', 'front'),
    ('/prius/front_sonar/left_middle_range', 'front'),
    ('/prius/front_sonar/right_far_range', 'front'),
    ('/prius/front_sonar/right_middle_range', 'front'),
    ('/prius/back_sonar/left_far_range', 'rear'),
    ('/prius/back_sonar/left_middle_range', 'rear'),
    ('/prius/back_sonar/right_far_range', 'rear'),
    ('/prius/back_sonar/right_middle_range', 'rear'),
    ('/prius/side_sonar/left_front_range', 'left'),
    ('/prius/side_sonar/left_rear_range', 'left'),
    ('/prius/side_sonar/right_front_range', 'right'),
    ('/prius/side_sonar/right_rear_range', 'right'),
)

SIDES = ('front', 'rear', 'left', 'right')
YELLOW = ColorRGBA(r=1.00, g=0.85, b=0.05, a=1.0)
RED = ColorRGBA(r=1.00, g=0.12, b=0.10, a=1.0)
ARC_HALF = 0.50
ARC_POINTS = 14
NS = 'proximity'
Hit = namedtuple('Hit', 'frame_id range max_range')


class _SafeExecutor(SingleThreadedExecutor):
    def _take_subscription(self, sub):
        try:
            return super()._take_subscription(sub)
        except RuntimeError:
            return None


def parse_range_cdr(data):
    """从 ROS 2 CDR 字节里取出 frame_id / range / max_range。"""
    if not data or len(data) < 28:
        return None
    try:
        offset = 4
        offset += 8
        slen = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        if slen <= 0 or offset + slen > len(data) or len(data) < 20:
            return None
        frame = data[offset:offset + slen].split(b'\x00', 1)[0].decode(
            'ascii', 'replace')
        _fov, _min_r, max_r, rng = struct.unpack_from('<ffff', data, len(data) - 16)
    except (struct.error, UnicodeDecodeError, IndexError):
        return None
    return Hit(frame, rng, max_r)


def _point(x, y, z=1.25):
    return Point(x=float(x), y=float(y), z=float(z))


def _valid(hit):
    if hit is None or not math.isfinite(hit.range):
        return False
    if hit.range > hit.max_range * 0.98:
        return False
    # 0.05 是传感器下限，贴着车身自打时几乎都是这个数
    if hit.range < 0.08:
        return False
    return True


def _arc_points(radius, half_angle=ARC_HALF, count=ARC_POINTS):
    points = []
    for i in range(count):
        angle = -half_angle + 2.0 * half_angle * i / (count - 1)
        points.append(_point(radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _blank(frame_id, stamp, marker_id, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = NS
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.frame_locked = True
    return marker


def _delete(stamp, marker_id):
    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.header.stamp = stamp
    marker.ns = NS
    marker.id = marker_id
    marker.action = Marker.DELETE
    return marker


class ProximityNode(Node):
    def __init__(self):
        super().__init__('proximity_warn')
        self.declare_parameter('warn_distance', 0.5)
        self.declare_parameter('danger_distance', 0.10)
        self.declare_parameter('publish_rate', 5.0)

        self.warn = self.get_parameter('warn_distance').value
        self.danger = self.get_parameter('danger_distance').value
        rate = self.get_parameter('publish_rate').value

        self.latest = {}
        for topic, side in SONARS:
            self.create_subscription(
                Range, topic,
                lambda data, key=topic, grp=side: self._on_raw(key, grp, data),
                qos_profile_sensor_data,
                raw=True)

        self.pub = self.create_publisher(MarkerArray, '/parking/proximity_markers', 10)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f'12 路超声近距标注: < {self.warn:.2f} m 黄色, '
            f'< {self.danger:.2f} m 红色, {rate:.0f} Hz')

    def _on_raw(self, topic, side, data):
        hit = parse_range_cdr(data)
        if hit is not None:
            self.latest[topic] = (side, hit)

    def _closest_by_side(self):
        best = {}
        for side, hit in self.latest.values():
            if not _valid(hit) or hit.range >= self.warn:
                continue
            current = best.get(side)
            if current is None or hit.range < current.range:
                best[side] = hit
        return best

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        closest = self._closest_by_side()
        for index, side in enumerate(SIDES):
            base = index * 4
            hit = closest.get(side)
            if hit is None:
                array.markers.extend(
                    _delete(stamp, base + offset) for offset in range(4))
                continue
            color = RED if hit.range < self.danger else YELLOW
            frame = hit.frame_id or 'base_link'
            outer = max(hit.range, 0.04)
            inner = max(outer - 0.08, outer * 0.65)
            label_x = max(outer * 0.55, 0.15)
            label_z = 2.20

            arc_a = _blank(frame, stamp, base, Marker.LINE_STRIP)
            arc_a.scale.x = 0.045
            arc_a.color = color
            arc_a.points = _arc_points(outer)
            array.markers.append(arc_a)

            arc_b = _blank(frame, stamp, base + 1, Marker.LINE_STRIP)
            arc_b.scale.x = 0.035
            arc_b.color = color
            arc_b.points = _arc_points(inner)
            array.markers.append(arc_b)

            text = _blank(frame, stamp, base + 2, Marker.TEXT_VIEW_FACING)
            # 探头约 0.5 m 高，文字再抬 2.2 m，超过车顶；RViz 无真正的 2D 叠加层
            text.pose.position.x = label_x
            text.pose.position.z = label_z
            text.scale.z = 0.36
            text.color = color
            text.text = f'{int(round(hit.range * 100.0))} cm'
            array.markers.append(text)

            stem = _blank(frame, stamp, base + 3, Marker.LINE_STRIP)
            stem.scale.x = 0.02
            stem.color = color
            stem.points = [_point(label_x, 0.0, 1.25), _point(label_x, 0.0, label_z)]
            array.markers.append(stem)
        self.pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ProximityNode()
    executor = _SafeExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
