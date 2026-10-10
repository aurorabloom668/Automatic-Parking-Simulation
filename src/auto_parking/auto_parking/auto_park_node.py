#!/usr/bin/env python3
"""自动泊车执行器：沿 mission 规划的多段路径驱动车辆完成全流程。

分层结构：
  规划   mission.build_mission() 给出按挡位切分的路径段（离线几何解）。
  跟踪   横向 Pure Pursuit（倒车段用"朝向翻转 + 转角取反"的镜像技巧，
        等价于把倒车视为一辆朝向相反的前进车）；纵向 P 控制 + 接近段
        终点线性降速。
  安全   独立于跟踪逻辑：倒车监视后向超声，前进监视前向超声，
        距离低于阈值立即刹停，解除后自动恢复。近距 RViz 标注由
        proximity_warn 节点用 12 路超声完成。

状态机：WAIT（等待仿真就绪）-> DRIVE / BRAKE（段间停稳换挡）循环 -> DONE。
"""

import math
import traceback

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from nav_msgs.msg import Odometry
from prius_msgs.msg import Control
from visualization_msgs.msg import MarkerArray

from . import mission, ribbon, vehicle
from .path_geometry import FORWARD

GEAR_MSG = {FORWARD: Control.FORWARD, -1: Control.REVERSE}


class _SafeExecutor(SingleThreadedExecutor):
    """take_message 失败时丢掉这一帧，而不是把整个节点打死。

    Humble 早期版本在 float 字段为 NaN / FLT_MAX 时会在转换里抛
    RuntimeError；当前 LaserScan / Range 已可正常订阅。
    """

    def _take_subscription(self, sub):
        try:
            return super()._take_subscription(sub)
        except RuntimeError:
            return None


class AutoParkNode(Node):
    def __init__(self):
        super().__init__('planned_park')

        # 任务几何参数（与 mission.DEFAULTS 对应，可被 parking.yaml 覆盖）
        for key, value in mission.DEFAULTS.items():
            self.declare_parameter(key, value)
        cfg = {key: self.get_parameter(key).value for key in mission.DEFAULTS}

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('lookahead_forward', 2.2)
        self.declare_parameter('lookahead_reverse', 1.3)
        self.declare_parameter('throttle_kp', 0.35)
        self.declare_parameter('brake_kp', 0.5)
        self.declare_parameter('end_tolerance', 0.15)
        self.declare_parameter('sonar_stop_distance', 0.35)
        self.declare_parameter('laser_stop_distance', 0.6)
        self.declare_parameter('safety_enabled', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.lookahead_fwd = self.get_parameter('lookahead_forward').value
        self.lookahead_rev = self.get_parameter('lookahead_reverse').value
        self.throttle_kp = self.get_parameter('throttle_kp').value
        self.brake_kp = self.get_parameter('brake_kp').value
        self.end_tol = self.get_parameter('end_tolerance').value
        self.sonar_stop = self.get_parameter('sonar_stop_distance').value
        self.laser_stop = self.get_parameter('laser_stop_distance').value
        self.safety_enabled = self.get_parameter('safety_enabled').value

        self.mission = mission.build_mission(cfg)
        for note in self.mission.notes:
            self.get_logger().info(note)
        clear, lines = mission.clearance_report(self.mission)
        self.get_logger().info(f'规划路径对场景障碍物的最小间隙 {clear:.2f} m')
        for line in lines:
            self.get_logger().info(line)
        if clear < 0.10:
            self.get_logger().warn('间隙不足 0.10 m，请检查参数！')

        # 每段的累计弧长表，用于计算剩余距离
        self.chunk_cumdist = []
        for chunk in self.mission.chunks:
            cum = [0.0]
            for a, b in zip(chunk.points, chunk.points[1:]):
                cum.append(cum[-1] + math.hypot(b.x - a.x, b.y - a.y))
            self.chunk_cumdist.append(cum)

        # 运行状态
        self.pose = None            # (x, y, yaw)
        self.speed = 0.0            # 沿车头方向的带符号速度
        self.state = 'WAIT'
        self.chunk_idx = 0
        self.point_idx = 0
        self.wait_ticks = 0
        self.blocked_reason = None
        self.sonar_ranges = {}

        self.control_pub = self.create_publisher(Control, '/prius/control', 10)
        # Transient Local：RViz 晚于节点启动也能拿到最后一帧条带
        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/parking/preview_markers', marker_qos)

        self.create_subscription(
            Odometry, '/prius/ground_truth', self._on_odom,
            qos_profile_sensor_data)

        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_step)
        self.create_timer(0.5, self._publish_markers)
        self.static_markers = self._build_static_markers()
        ribbon_pts = sum(
            len(m.points) for m in self.static_markers.markers
            if m.ns == 'planned_path')
        self.get_logger().info(
            f'规划条带已生成: {len(self.static_markers.markers)} 个 marker, '
            f'条带顶点数 {ribbon_pts}')
        self._publish_markers()

    # ------------------------------------------------------------ 传感器
    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw)
        v = msg.twist.twist.linear
        self.speed = v.x * math.cos(yaw) + v.y * math.sin(yaw)

    # ------------------------------------------------------------ 主循环
    def _control_step(self):
        try:
            self._tick()
        except Exception as exc:  # noqa: BLE001 — 控制循环不能把节点打死
            self.get_logger().error(f'控制循环异常: {exc}')

    def _tick(self):
        if self.pose is None:
            return

        if self.state == 'WAIT':
            # 等出生下落稳定
            self.wait_ticks += 1
            if self.wait_ticks > 60 and abs(self.speed) < 0.05:
                self.state = 'DRIVE'
                self._log_chunk()
            else:
                self._publish_control(0.0, 0.6, 0.0, Control.NEUTRAL)
            return

        if self.state == 'DONE':
            self._publish_control(0.0, 1.0, 0.0, Control.NEUTRAL)
            return

        chunk = self.mission.chunks[self.chunk_idx]

        if self.state == 'BRAKE':
            self._publish_control(0.0, 0.8, 0.0, Control.NEUTRAL)
            if abs(self.speed) < 0.06:
                self._advance_chunk()
            return

        # DRIVE
        if self.safety_enabled and self._blocked(chunk.gear):
            self._publish_control(0.0, 1.0, 0.0, Control.NEUTRAL)
            return

        self._update_progress(chunk)
        dist_end = self._distance_to_end(chunk)
        if dist_end < self.end_tol:
            self.state = 'BRAKE'
            self._publish_control(0.0, 0.8, 0.0, Control.NEUTRAL)
            return

        steer = self._pure_pursuit(chunk)
        v_des = min(chunk.speed_cap, 0.25 + 0.55 * dist_end)
        drive_err = v_des - chunk.gear * self.speed
        if drive_err > 0.0:
            throttle = min(0.08 + self.throttle_kp * drive_err, 0.6)
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-self.brake_kp * drive_err, 0.8) if drive_err < -0.15 else 0.0
        self._publish_control(throttle, brake, steer, GEAR_MSG[chunk.gear])

    # ------------------------------------------------------------ 跟踪
    def _update_progress(self, chunk):
        """在局部窗口内把进度指针推到离车最近的路径点，避免跨圈跳跃。"""
        x, y, _ = self.pose
        points = chunk.points
        best_i, best_d = self.point_idx, float('inf')
        for i in range(self.point_idx, min(self.point_idx + 80, len(points))):
            d = (points[i].x - x) ** 2 + (points[i].y - y) ** 2
            if d < best_d:
                best_i, best_d = i, d
        self.point_idx = best_i

    def _distance_to_end(self, chunk):
        cum = self.chunk_cumdist[self.chunk_idx]
        remain = cum[-1] - cum[self.point_idx]
        end = chunk.points[-1]
        return min(remain, math.hypot(end.x - self.pose[0], end.y - self.pose[1]))

    def _pure_pursuit(self, chunk):
        x, y, yaw = self.pose
        lookahead = (self.lookahead_fwd if chunk.gear == FORWARD
                     else self.lookahead_rev)
        cum = self.chunk_cumdist[self.chunk_idx]
        target = chunk.points[-1]
        for i in range(self.point_idx, len(chunk.points)):
            if cum[i] - cum[self.point_idx] >= lookahead:
                target = chunk.points[i]
                break

        # 倒车段镜像：朝向翻转后按前进车计算，转角取反
        heading = yaw if chunk.gear == FORWARD else yaw + math.pi
        alpha = vehicle.normalize_angle(
            math.atan2(target.y - y, target.x - x) - heading)
        dist = max(math.hypot(target.x - x, target.y - y), 0.3)
        steer_angle = math.atan2(
            2.0 * vehicle.WHEELBASE * math.sin(alpha), dist)
        if chunk.gear != FORWARD:
            steer_angle = -steer_angle
        return max(-1.0, min(1.0, steer_angle / vehicle.MAX_STEER))

    # ------------------------------------------------------------ 安全层
    def _blocked(self, gear):
        prefix = 'front_' if gear == FORWARD else 'back_'
        readings = [d for k, d in self.sonar_ranges.items() if k.startswith(prefix)]
        closest = min(readings) if readings else float('inf')
        if gear == FORWARD:
            limit, sensor = self.laser_stop, '前声纳'
        else:
            limit, sensor = self.sonar_stop, '后声纳'

        if self.blocked_reason:
            # 已刹停，留 0.15 m 迟滞防止抖动
            if closest > limit + 0.15:
                self.get_logger().info(f'{sensor}障碍解除，继续行驶')
                self.blocked_reason = None
                return False
            return True
        if closest < limit:
            self.blocked_reason = sensor
            self.get_logger().warn(
                f'{sensor}检测到 {closest:.2f} m 处障碍，紧急刹停')
            return True
        return False

    # ------------------------------------------------------------ 状态机
    def _advance_chunk(self):
        chunk = self.mission.chunks[self.chunk_idx]
        end = chunk.points[-1]
        err = math.hypot(end.x - self.pose[0], end.y - self.pose[1])
        err_yaw = math.degrees(vehicle.normalize_angle(end.yaw - self.pose[2]))
        self.get_logger().info(
            f'段 {self.chunk_idx + 1} [{chunk.label}] 完成，'
            f'位置误差 {err:.2f} m, 朝向误差 {err_yaw:.1f} deg')

        if self.chunk_idx + 1 >= len(self.mission.chunks):
            gx, gy, gyaw = self.mission.par_goal
            final_err = math.hypot(gx - self.pose[0], gy - self.pose[1])
            final_yaw = math.degrees(
                vehicle.normalize_angle(gyaw - self.pose[2]))
            self.get_logger().info(
                f'=== 泊车完成 === 最终误差 {final_err:.2f} m / {final_yaw:.1f} deg')
            self.state = 'DONE'
            return
        self.chunk_idx += 1
        self.point_idx = 0
        self.state = 'DRIVE'
        self._log_chunk()

    def _log_chunk(self):
        chunk = self.mission.chunks[self.chunk_idx]
        length = self.chunk_cumdist[self.chunk_idx][-1]
        gear = '前进' if chunk.gear == FORWARD else '倒车'
        self.get_logger().info(
            f'开始段 {self.chunk_idx + 1}/{len(self.mission.chunks)} '
            f'[{chunk.label}] {gear} {length:.1f} m')

    def _publish_control(self, throttle, brake, steer, gear):
        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.throttle = float(throttle)
        msg.brake = float(brake)
        msg.steer = float(steer)
        msg.shift_gears = gear
        self.control_pub.publish(msg)

    # ------------------------------------------------------------ 可视化
    def _build_static_markers(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        marker_id = 0
        for spot in (self.mission.perp_spot, self.mission.par_spot):
            array.markers.append(ribbon.spot_outline(
                spot[0], spot[1], spot[2], self.frame_id, stamp, marker_id))
            marker_id += 1
        for goal in (self.mission.perp_goal, self.mission.par_goal):
            array.markers.append(ribbon.goal_footprint(
                goal[0], goal[1], goal[2], self.frame_id, stamp, marker_id))
            marker_id += 1
        array.markers.append(ribbon.path_ribbon(
            self.mission.path, self.frame_id, stamp, marker_id))
        marker_id += 1
        array.markers.append(ribbon.shift_points(
            self.mission.path, self.frame_id, stamp, marker_id))
        return array

    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        for marker in self.static_markers.markers:
            marker.header.stamp = stamp
        self.marker_pub.publish(self.static_markers)


def main(args=None):
    rclpy.init(args=args)
    node = AutoParkNode()
    executor = _SafeExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().fatal(traceback.format_exc())
        raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
