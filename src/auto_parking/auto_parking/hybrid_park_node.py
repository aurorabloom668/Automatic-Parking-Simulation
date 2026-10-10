#!/usr/bin/env python3
"""车位附近停稳、点选车位和显式开始的在线泊车监督节点。"""

import math
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from prius_msgs.msg import Control
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener
from auto_parking.msg import ParkingStatus

from . import collision, hybrid_astar, parking_strategy, ribbon, spot_detector, spots, vehicle
from .mission import Chunk
from .path_geometry import PathPoint, split_by_gear

FREE_COLOR = ColorRGBA(r=0.2, g=0.95, b=0.4, a=0.95)
BUSY_COLOR = ColorRGBA(r=0.95, g=0.25, b=0.25, a=0.95)
PICK_COLOR = ColorRGBA(r=1.0, g=0.85, b=0.15, a=1.0)


class ParkingSupervisor(Node):
    def __init__(self):
        super().__init__('parking_supervisor')
        defaults = {
            'frame_id': 'odom', 'control_rate': 50.0, 'detect_rate': 2.0,
            'heading_gain': 1.4, 'lateral_gain': 1.0, 'steering_preview_time': 0.10,
            'throttle_kp': 0.35, 'brake_kp': 0.5,
            'activation_radius': 12.0, 'entry_speed': 0.15,
            'position_tolerance': 0.10, 'yaw_tolerance': math.radians(3),
            'stop_speed': 0.05, 'minimum_clearance': 0.10, 'max_adjustments': 5,
            'speed_forward': 0.7, 'speed_reverse': 0.45, 'speed_fine': 0.20,
            'cloud_topic': '/prius/center_laser/scan', 'click_topic': '/clicked_point',
            'xy_resolution': 0.30, 'yaw_resolution': math.radians(6),
            'search_step': 0.60, 'search_clearance': 0.12,
            'local_margin': 5.0, 'search_max_time': 15.0, 'search_max_expand': 40000,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        self.frame_id = self.cfg['frame_id']
        self.settings = hybrid_astar.PlannerSettings(
            activation_radius=self.cfg['activation_radius'],
            xy_resolution=self.cfg['xy_resolution'], yaw_resolution=self.cfg['yaw_resolution'],
            step=self.cfg['search_step'], clearance=self.cfg['search_clearance'],
            local_margin=self.cfg['local_margin'], max_time=self.cfg['search_max_time'],
            max_expand=self.cfg['search_max_expand'],
            position_tolerance=0.8 * self.cfg['position_tolerance'],
            yaw_tolerance=self.cfg['yaw_tolerance'])
        self.static_obstacles = collision.park_world_obstacles()
        self.dynamic_obstacles = []
        self.spots = spots.detect_spots(self.static_obstacles)
        self.selected = None
        self.pose = None
        self.speed = 0.0
        self.mode = 'MANUAL'
        self.state = 'MANUAL'
        self.phase = '等待选位'
        self.reason = '使用 PublishPoint 点选绿色空闲车位'
        self.ready = False
        self.path = []
        self.chunks = []
        self.chunk_cumdist = []
        self.chunk_idx = self.point_idx = 0
        self.path_length = 0.0
        self.min_clearance = float('nan')
        self.adjustments = 0
        self.braking = False
        self.replan_pending = False
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.cancel_event = threading.Event()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(ParkingStatus, '/parking/status', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/parking/preview_markers', qos)
        self.control_pub = self.create_publisher(Control, '/parking/auto_control', 10)
        self.create_subscription(Odometry, '/prius/ground_truth', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PointStamped, self.cfg['click_topic'], self._on_click, 10)
        self.create_subscription(String, '/parking/command', self._on_command, 10)
        self.create_subscription(PointCloud2, self.cfg['cloud_topic'], self._on_cloud, qos_profile_sensor_data)
        self.create_timer(1.0 / self.cfg['control_rate'], self._control_step)
        self.create_timer(1.0 / self.cfg['detect_rate'], self._perceive_step)
        self.create_timer(0.1, self._status_step)
        self.create_timer(0.4, self._publish_markers)
        self._publish_status()

    def _planning_obstacles(self):
        return self.static_obstacles + self.dynamic_obstacles

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw)
        v = msg.twist.twist.linear
        self.speed = v.x * math.cos(yaw) + v.y * math.sin(yaw)

    def _on_cloud(self, msg):
        self.cloud = msg

    def _perceive_step(self):
        if self.cloud is None:
            return
        try:
            local = spot_detector.parse_cloud_xy(self.cloud)
            stamp = rclpy.time.Time.from_msg(self.cloud.header.stamp)
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.cloud.header.frame_id,
                                                  stamp)
            base = self.tf_buffer.lookup_transform(self.frame_id, 'base_link', stamp)
        except TransformException as exc:
            self.get_logger().warn(f'等待激光 TF: {exc}', throttle_duration_sec=5.0)
            return
        t, q = tf.transform.translation, tf.transform.rotation
        points = spot_detector.transform_xy(local, t.x, t.y, t.z, q.w, q.x, q.y, q.z)
        t, q = base.transform.translation, base.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._update_obstacles(spot_detector.detect_from_points(
            points, ego=(t.x, t.y, yaw), static_obstacles=self.static_obstacles))

    def _update_obstacles(self, result):
        self.dynamic_obstacles = result.obstacles
        spot_detector.mark_occupied(self.spots, result.grid)
        for spot in self.spots:
            spot.occupied = spot.occupied or spots._occupied(spot.center, self.static_obstacles)
        if self.state == 'AUTO_PARKING' and self.path and self.future is None and not self.replan_pending:
            tail = self._remaining_path()
            minimum = collision.path_min_clearance(tail, self._planning_obstacles())
            self.min_clearance = minimum
            if minimum <= self.cfg['minimum_clearance']:
                if collision.pose_collides(*self.pose, self._planning_obstacles(),
                                           self.cfg['minimum_clearance']):
                    self._hold('车辆当前车身与局部障碍间隙不足')
                else:
                    self._request_replan('局部障碍变化，停车后重新规划')
        self._status_step()

    def _on_click(self, msg):
        if self.state == 'AUTO_PARKING':
            return
        x, y = msg.point.x, msg.point.y
        picked = min(self.spots, key=lambda s: math.hypot(s.center[0] - x, s.center[1] - y))
        if math.hypot(picked.center[0] - x, picked.center[1] - y) > 3.0 or picked.occupied:
            self.reason = '请点选已知的绿色空闲车位'
            self._publish_status()
            return
        self._clear_path()
        self.selected = picked
        picked.goal = parking_strategy.parking_goal(picked, self._planning_obstacles())
        self.adjustments = 0
        self.state = 'SLOT_SELECTED'
        self.phase = '车位已选'
        self.ready = False
        self._update_admission()
        self._publish_status()
        self._publish_markers()

    def _on_command(self, msg):
        command = msg.data
        if command in ('cancel', 'estop'):
            self._clear_path()
            if command == 'cancel':
                self.selected = None
            self._hold('急停：切换手动模式后接管' if command == 'estop' else '已取消：切换手动模式后接管')
        elif command in ('manual', 'auto'):
            self._clear_path()
            self.mode = 'MANUAL' if command == 'manual' else 'AUTO'
            self.state = 'MANUAL_APPROACH' if self.selected else 'MANUAL'
            self.phase = '手动接近' if self.selected else '等待选位'
            self.reason = ''
            self._update_admission()
        elif command == 'start':
            self._update_admission()
            if self.state == 'READY_TO_PARK' and self.ready:
                self.mode = 'AUTO'
                self.adjustments = 0
                self._begin_plan()
        self._publish_status()
        self._publish_markers()

    def _clear_path(self):
        self.cancel_event.set()
        self.future = None
        self.path = []
        self.chunks = []
        self.chunk_cumdist = []
        self.path_length = 0.0
        self.min_clearance = float('nan')
        self.braking = False
        self.replan_pending = False

    def _hold(self, reason):
        self.cancel_event.set()
        self.future = None
        self.replan_pending = False
        self.state = 'HOLD_FOR_MANUAL'
        self.phase = '等待手动接管'
        self.reason = reason
        self.ready = False
        self._publish_control(brake=1.0)
        self._publish_status()

    def _update_admission(self):
        if (self.selected is None
                or self.state in ('AUTO_PARKING', 'COMPLETE', 'HOLD_FOR_MANUAL')):
            return
        self.ready = False
        self.state = 'MANUAL_APPROACH'
        self.phase = '手动接近'
        if self.pose is None:
            self.reason = '等待车辆定位'
        elif parking_strategy.distance_to_spot(*self.pose[:2], self.selected) > self.cfg['activation_radius']:
            self.reason = '请手动接近目标车位，在附近停稳后开始'
        elif abs(self.speed) > self.cfg['entry_speed']:
            self.reason = '请减速或停车'
        elif self.selected.occupied:
            self.reason = '目标车位已被占用'
        elif any(s.spot_id != self.selected.spot_id and parking_strategy.body_in_spot(self.pose, s)
                 for s in self.spots):
            self.reason = '请先手动驶出当前车位，再开始新车位泊车'
        elif collision.pose_collides(*self.pose, self._planning_obstacles(),
                                     self.cfg['minimum_clearance']):
            self.reason = '车辆当前车身与障碍物间隙不足'
        elif collision.pose_collides(*self.selected.goal, self._planning_obstacles(),
                                     self.cfg['minimum_clearance']):
            self.reason = '目标车位安全间隙不足'
        else:
            self.state = 'READY_TO_PARK'
            self.phase = '可以开始泊车'
            self.reason = '点击开始自动泊车，系统将自动接管'
            self.ready = True

    def _begin_plan(self, adjust=False):
        self.state = 'AUTO_PARKING'
        self.phase = '纠偏规划' if adjust else '局部规划'
        self.reason = ''
        self.replan_pending = False
        self.ready = False
        self.cancel_event = threading.Event()
        self.future = self.worker.submit(
            hybrid_astar.plan_local_to_spot, self.pose, replace(self.selected),
            self._planning_obstacles(), self.settings, adjust, self.cancel_event)
        self._publish_control(brake=1.0)
        self._publish_status()

    def _poll_plan(self):
        if not self.future.done():
            self._publish_control(brake=1.0)
            return
        future, self.future = self.future, None
        try:
            result = future.result()
        except Exception as exc:
            self._hold(f'规划失败：{exc}')
            return
        if not result.ok:
            self._hold(result.reason)
            return
        self.min_clearance = collision.path_min_clearance(result.path, self._planning_obstacles())
        if self.min_clearance <= self.cfg['minimum_clearance']:
            self._request_replan('规划期间障碍变化，停车后重新规划')
            return
        self._install_path(result)

    def _request_replan(self, reason):
        if self.adjustments >= self.cfg['max_adjustments']:
            self._hold('重新规划次数已耗尽：' + reason)
            return
        self.path = []
        self.chunks = []
        self.chunk_cumdist = []
        self.replan_pending = True
        self.phase = '停车重规划'
        self.reason = reason
        self._publish_control(brake=1.0)
        self._publish_status()
        self._publish_markers()

    def _install_path(self, result):
        self.path = result.path
        self.path_length = result.path_length
        self.chunks = []
        self.chunk_cumdist = []
        for gear, points in split_by_gear(self.path):
            cum = [0.0]
            for a, b in zip(points, points[1:]):
                cum.append(cum[-1] + math.hypot(b.x - a.x, b.y - a.y))
            if cum[-1] <= 1e-6:
                continue
            cap = self.cfg['speed_forward'] if gear > 0 else self.cfg['speed_reverse']
            self.chunks.append(Chunk(gear, points, cap, '前进调整 / 入位' if gear > 0 else '倒车调整 / 入位'))
            self.chunk_cumdist.append(cum)
        self.chunk_idx = self.point_idx = 0
        self.braking = False
        if not self.chunks:
            self._advance_chunk()
            return
        self.phase = self.chunks[0].label
        self._publish_status()
        self._publish_markers()

    def _remaining_path(self):
        chunk = self.chunks[self.chunk_idx]
        self._update_progress(chunk)
        tail = [PathPoint(*self.pose, chunk.gear)] + chunk.points[self.point_idx + 1:]
        for later in self.chunks[self.chunk_idx + 1:]:
            tail += later.points[1:]
        return tail

    def _control_step(self):
        if self.state != 'AUTO_PARKING':
            return
        if self.pose is None:
            self._hold('无法跟踪：缺少定位')
            return
        clearance = collision.pose_clearance(*self.pose, self._planning_obstacles())
        if clearance <= self.cfg['minimum_clearance']:
            self._hold('跟踪期间车身安全间隙不足')
            return
        if self.replan_pending:
            self._publish_control(brake=1.0)
            if abs(self.speed) <= self.cfg['stop_speed']:
                self.adjustments += 1
                self._begin_plan(adjust=parking_strategy.body_in_spot(self.pose, self.selected))
            return
        if self.future is not None:
            self._poll_plan()
            return
        if not self.chunks:
            self._hold('无法跟踪：缺少规划路径')
            return
        if self.braking:
            self._publish_control(brake=1.0)
            if abs(self.speed) <= self.cfg['stop_speed']:
                self._advance_chunk()
            return
        chunk = self.chunks[self.chunk_idx]
        self._update_progress(chunk)
        end = chunk.points[-1]
        distance = math.hypot(end.x - self.pose[0], end.y - self.pose[1])
        along = chunk.gear * ((self.pose[0] - end.x) * math.cos(end.yaw)
                             + (self.pose[1] - end.y) * math.sin(end.yaw))
        if distance <= 0.05 or (self.point_idx == len(chunk.points) - 1 and along >= 0.0):
            self.braking = True
            self.phase = '停车验收' if self.chunk_idx == len(self.chunks) - 1 else '停车换挡'
            self._publish_control(brake=1.0)
            return
        steering = self._tracking_steer(chunk)
        target_speed = min(chunk.speed_cap, max(0.06, 0.7 * distance))
        if distance < 0.5 or clearance < 0.5:
            target_speed = min(target_speed, self.cfg['speed_fine'])
        error = target_speed - chunk.gear * self.speed
        throttle = min(0.04 + self.cfg['throttle_kp'] * error, 0.4) if error > 0 else 0.0
        brake = min(-self.cfg['brake_kp'] * error, 0.8) if error < 0 else 0.0
        self._publish_control(throttle, brake, steering,
                              Control.FORWARD if chunk.gear > 0 else Control.REVERSE)

    def _update_progress(self, chunk):
        x, y, _ = self.pose
        self.point_idx = min(
            range(self.point_idx, min(self.point_idx + 80, len(chunk.points))),
            key=lambda i: (chunk.points[i].x - x) ** 2 + (chunk.points[i].y - y) ** 2)

    def _tracking_steer(self, chunk):
        x, y, yaw = self.pose
        reference = chunk.points[self.point_idx]
        cum = self.chunk_cumdist[self.chunk_idx]
        preview = abs(self.speed) * self.cfg['steering_preview_time']
        target = reference
        for i in range(self.point_idx, len(chunk.points)):
            if cum[i] - cum[self.point_idx] >= preview:
                target = chunk.points[i]
                break
        lateral = (-(x - reference.x) * math.sin(reference.yaw)
                   + (y - reference.y) * math.cos(reference.yaw))
        curvature = (target.curvature + chunk.gear * self.cfg['heading_gain']
                     * vehicle.normalize_angle(reference.yaw - yaw)
                     - self.cfg['lateral_gain'] * lateral)
        angle = math.atan(vehicle.WHEELBASE * curvature)
        return max(-1.0, min(1.0, angle / vehicle.MAX_STEER))

    def _settled(self):
        return parking_strategy.parking_complete(
            self.pose, self.speed, self.selected, self._planning_obstacles(),
            self.cfg['position_tolerance'], self.cfg['yaw_tolerance'],
            self.cfg['stop_speed'], self.cfg['minimum_clearance'])

    def _advance_chunk(self):
        if self.chunk_idx + 1 < len(self.chunks):
            self.chunk_idx += 1
            self.point_idx = 0
            self.braking = False
            self.phase = self.chunks[self.chunk_idx].label
        elif self._settled():
            self.state = 'COMPLETE'
            self.phase = '泊车完成'
            self.reason = ''
            self.min_clearance = collision.pose_clearance(*self.pose, self._planning_obstacles())
            self._publish_status()
        elif self.adjustments >= self.cfg['max_adjustments']:
            self._hold('纠偏次数已耗尽，真实位置、航向或车身包络未通过验收')
        else:
            self.adjustments += 1
            self._begin_plan(adjust=True)

    def _publish_control(self, throttle=0.0, brake=0.0, steer=0.0, gear=Control.NEUTRAL):
        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle, msg.brake, msg.steer = float(throttle), float(brake), float(steer)
        msg.shift_gears = gear
        self.control_pub.publish(msg)

    def _status_step(self):
        self._update_admission()
        self._publish_status()

    def _publish_status(self):
        msg = ParkingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.mode, msg.state, msg.phase, msg.reason = self.mode, self.state, self.phase, self.reason
        msg.ready_to_park = self.ready
        msg.entry_distance = (parking_strategy.distance_to_spot(*self.pose[:2], self.selected)
                              if self.selected and self.pose else float('inf'))
        msg.activation_radius = self.cfg['activation_radius']
        msg.path_length, msg.min_clearance = float(self.path_length), float(self.min_clearance)
        msg.longitudinal_error = msg.lateral_error = msg.yaw_error = float('nan')
        if self.selected:
            msg.selected_slot = self.selected.spot_id
            msg.slot_label, msg.slot_kind = self.selected.label, self.selected.kind
            msg.occupied = self.selected.occupied
            if self.pose:
                errors = parking_strategy.parking_errors(self.pose, self.selected.goal)
                msg.longitudinal_error, msg.lateral_error, msg.yaw_error = errors
                msg.body_inside = parking_strategy.body_in_spot(self.pose, self.selected)
        msg.adjustments = self.adjustments
        self.status_pub.publish(msg)

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for spot in self.spots:
            selected = self.selected and spot.spot_id == self.selected.spot_id
            color = PICK_COLOR if selected else BUSY_COLOR if spot.occupied else FREE_COLOR
            marker = ribbon.spot_outline(*spot.center, self.frame_id, stamp, spot.spot_id)
            marker.color = color
            array.markers.append(marker)
            flag = '占用' if spot.occupied else '已选' if selected else '空闲'
            array.markers.append(ribbon.spot_label(
                f'{spot.spot_id} {spot.label} {flag}', *spot.center[:2],
                self.frame_id, stamp, spot.spot_id, color=color))
        if self.selected:
            array.markers.append(ribbon.goal_footprint(
                *self.selected.goal, self.frame_id, stamp, 50))
            array.markers.extend(ribbon.parking_area(
                self.selected, self.cfg['activation_radius'], self.ready, self.frame_id, stamp))
        if self.path:
            array.markers.append(ribbon.path_ribbon(self.path, self.frame_id, stamp, 100))
            array.markers.append(ribbon.shift_points(self.path, self.frame_id, stamp, 101))
        self.marker_pub.publish(array)

    def destroy_node(self):
        self.cancel_event.set()
        self.worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
