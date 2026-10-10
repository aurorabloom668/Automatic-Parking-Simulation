"""离线自行车模型闭环；执行真实监督节点控制器、换挡和终点验收。"""
import math
import struct

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from prius_msgs.msg import Control
from sensor_msgs.msg import PointCloud2, PointField
from auto_parking import collision, hybrid_astar, parking_strategy, spot_detector, vehicle
from test_local_planning import PARKING_CASES
from test_supervisor import click, supervisor


def run_closed_loop(node, sensor_step=None):
    start = node.pose
    command = Control(brake=1.0, shift_gears=Control.NEUTRAL)
    class Recorder:
        def publish(self, msg):
            nonlocal command
            command = msg
    publisher = node.control_pub
    node.control_pub = Recorder()
    try:
        dt, steering, lowest_clearance = 0.02, 0.0, float('inf')
        previous_gear, gear_changes = None, 0
        for step in range(20000):
            if sensor_step is not None:
                sensor_step(step)
            if node.future is not None:
                node.future.result(timeout=30)
            node._control_step()
            if command.shift_gears != Control.NEUTRAL:
                if previous_gear is not None and command.shift_gears != previous_gear:
                    gear_changes += 1
                previous_gear = command.shift_gears
            old_speed = node.speed
            gear = -1 if command.shift_gears == Control.REVERSE else 1
            acceleration = gear * 1.6 * command.throttle - 0.35 * node.speed
            if abs(node.speed) > 1e-6:
                acceleration -= math.copysign(5.0 * command.brake, node.speed)
            node.speed += dt * acceleration
            if command.brake and old_speed * node.speed < 0:
                node.speed = 0.0
            steering += min(1.0, dt / 0.10) * (command.steer * vehicle.MAX_STEER - steering)
            movement = 0.5 * (old_speed + node.speed) * dt
            node.pose = hybrid_astar._integrate(*node.pose, 1 if movement >= 0 else -1,
                                               steering, abs(movement))[:3]
            lowest_clearance = min(lowest_clearance, collision.pose_clearance(
                *node.pose, node._planning_obstacles()))
            if node.state in ('COMPLETE', 'HOLD_FOR_MANUAL'):
                break
        errors = parking_strategy.parking_errors(node.pose, node.selected.goal)
        print(f'spot={node.selected.spot_id} start={start} state={node.state} '
              f'seconds={step * dt:.2f} errors={errors} '
              f'clearance={lowest_clearance:.6f} adjustments={node.adjustments} '
              f'gear_changes={gear_changes} speed={node.speed:.4f} '
              f'gear={command.shift_gears} steering={steering:.4f} '
              f'plan_clearance={node.min_clearance:.6f}')
        assert node.state == 'COMPLETE', (
            node.state, node.reason, errors, step, node.adjustments, node.pose,
            node.speed, lowest_clearance)
        assert node._settled()
        assert lowest_clearance >= node.cfg['minimum_clearance']
        return gear_changes
    finally:
        node.control_pub = publisher


@pytest.mark.parametrize('index,start', PARKING_CASES)
def test_closed_loop_parking(supervisor, index, start):
    node = supervisor
    click(node, index)
    node.pose = start
    result = hybrid_astar.plan_local_to_spot(node.pose, node.selected,
                                            node._planning_obstacles(), node.settings)
    assert result.ok, result.reason
    node.mode, node.state = 'AUTO', 'AUTO_PARKING'
    node._install_path(result)
    run_closed_loop(node)


def test_entry_error_triggers_real_in_slot_alignment(supervisor):
    node = supervisor
    click(node, 4)
    x, y, yaw = node.selected.goal
    node.pose = (x - 0.20, y + 0.15, yaw + math.radians(8))
    assert parking_strategy.body_in_spot(node.pose, node.selected) and not node._settled()
    node.mode, node.state = 'AUTO', 'AUTO_PARKING'
    node._advance_chunk()
    changes = run_closed_loop(node)
    assert node.adjustments >= 1 and changes >= 1


def test_new_obstacle_replans_and_finishes_without_manual_takeover(supervisor):
    node = supervisor
    click(node, 4)
    result = hybrid_astar.plan_local_to_spot(node.pose, node.selected,
                                            node._planning_obstacles(), node.settings)
    assert result.ok
    node.mode, node.state = 'AUTO', 'AUTO_PARKING'
    node._install_path(result)
    points = node.chunks[0].points
    point = points[len(points) // 2]
    node._update_obstacles(spot_detector.DetectResult(
        [collision.Circle('new', point.x, point.y, 0.20)]))
    assert node.replan_pending and node.state == 'AUTO_PARKING'
    run_closed_loop(node)
    assert node.adjustments == 1


def test_scan_updates_do_not_stop_screenshot_pose_on_filled_empty_area(supervisor):
    node = supervisor
    node.pose = (-7.9745, -3.92928, math.radians(-53.88))
    click(node, 4)
    result = hybrid_astar.plan_local_to_spot(node.pose, node.selected,
                                            node._planning_obstacles(), node.settings)
    assert result.ok
    node.mode, node.state = 'AUTO', 'AUTO_PARKING'
    node._install_path(result)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
    tf.transform.translation.z = 1.73
    tf.transform.rotation.w = 1.0
    node.tf_buffer.set_transform_static(tf, 'test')
    world = [(-3, -10 + i * 0.1) for i in range(201)]
    world += [(-3 + i * 0.1, y) for y in (-10, 10) for i in range(51)]
    for obs in node.static_obstacles:
        if isinstance(obs, collision.Rect):
            corners = obs.corners()
            for a, b in zip(corners, corners[1:] + corners[:1]):
                # 模拟邻车可见表面的厘米级测量误差。
                world += [(a[0] + (b[0] - a[0]) * i / 30 + 0.01 * math.sin(i),
                           a[1] + (b[1] - a[1]) * i / 30 + 0.01 * math.cos(i))
                          for i in range(31)]
    scans = 0
    def replay(step):
        nonlocal scans
        if step % 25:
            return
        stamp = Time(sec=1 + step // 50, nanosec=(step % 50) * 20000000)
        x, y, yaw = node.pose
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
        tf.header.stamp = stamp
        tf.transform.translation.x, tf.transform.translation.y = float(x), float(y)
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        node.tf_buffer.set_transform(tf, 'test')
        body = vehicle.footprint(*node.pose)[:4]
        c, s = math.cos(yaw), math.sin(yaw)
        local = [((px - x) * c + (py - y) * s, -(px - x) * s + (py - y) * c, -1.0)
                 for px, py in world + body]
        cloud = PointCloud2(height=1, width=4 * len(local), point_step=12,
                            row_step=48 * len(local))
        cloud.header.frame_id, cloud.header.stamp = 'laser', stamp
        cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                        for name, offset in (('x', 0), ('y', 4), ('z', 8))]
        cloud.data = b''.join(struct.pack('<fff', *p) * 4 for p in local)
        node._on_cloud(cloud)
        node._perceive_step()
        scans += 1
    run_closed_loop(node, replay)
    assert scans > 10 and node.adjustments == 0
