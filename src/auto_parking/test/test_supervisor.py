import math

import pytest
import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from auto_parking import collision, hybrid_astar, parking_strategy, spot_detector
from auto_parking.hybrid_park_node import ParkingSupervisor


@pytest.fixture
def supervisor():
    rclpy.init()
    node = ParkingSupervisor()
    node.pose = (0, 0, math.pi)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def click(node, index):
    msg = PointStamped()
    msg.point.x, msg.point.y = node.spots[index].center[:2]
    node._on_click(msg)
    node._status_step()


def test_distant_selection_is_manual_and_goal_is_frozen(supervisor, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('远处选位不得调用规划器')
    monkeypatch.setattr(hybrid_astar, 'plan_local_to_spot', forbidden)
    supervisor._on_command(String(data='auto'))
    click(supervisor, 0)
    goal = supervisor.selected.goal
    for pose in ((0, 0, math.pi), (3, 0, 0), (2, -3, 1)):
        supervisor.pose = pose
        supervisor._status_step()
        supervisor._on_command(String(data='start'))
        assert supervisor.state == 'MANUAL_APPROACH'
        assert supervisor.future is None and not supervisor.path
        assert supervisor.selected.goal == goal


def test_nearby_selection_admits_perpendicular_heading_and_requires_slow_free_body(supervisor):
    supervisor.pose = (-8.0, 12.5311, math.pi / 2)
    click(supervisor, 0)
    assert supervisor.ready and supervisor.state == 'READY_TO_PARK'
    assert supervisor.mode == 'MANUAL' and supervisor.future is None
    supervisor.speed = 0.16
    supervisor._status_step()
    assert not supervisor.ready
    supervisor.speed = 0
    supervisor.dynamic_obstacles = [collision.Circle('current', *supervisor.pose[:2], 0.3)]
    supervisor._status_step()
    assert not supervisor.ready


def test_activation_radius_is_measured_from_slot_boundary(supervisor):
    click(supervisor, 0)
    x, y, _ = supervisor.selected.center
    supervisor.pose = (x + parking_strategy.SPOT_LENGTH / 2 + supervisor.cfg['activation_radius'] - 0.01, y, 0)
    supervisor._status_step()
    assert supervisor.ready
    supervisor.pose = (x + parking_strategy.SPOT_LENGTH / 2 + supervisor.cfg['activation_radius'] + 0.01, y, 0)
    supervisor._status_step()
    assert not supervisor.ready


def test_unused_old_staging_area_is_not_an_admission_requirement(supervisor):
    supervisor.pose = (-8.0, 12.5311, math.pi / 2)
    supervisor.dynamic_obstacles = [collision.Circle('old_staging', -11.9053, 15.0311, 0.3)]
    click(supervisor, 0)
    assert supervisor.ready
    supervisor.dynamic_obstacles = [collision.Circle('goal', *supervisor.selected.goal[:2], 0.3)]
    supervisor._status_step()
    assert not supervisor.ready and supervisor.reason == '目标车位安全间隙不足'


def test_start_from_manual_mode_takes_over_and_plans_from_actual_pose(supervisor):
    from concurrent.futures import Future
    supervisor.pose = (-8.0, 12.5311, -math.pi / 2)
    click(supervisor, 0)
    calls = []
    class Worker:
        def submit(self, function, *args):
            calls.append((function, args))
            return Future()
    worker = supervisor.worker
    supervisor.worker = Worker()
    try:
        assert supervisor.mode == 'MANUAL' and supervisor.ready and not calls
        supervisor._on_command(String(data='start'))
        assert supervisor.mode == 'AUTO' and supervisor.state == 'AUTO_PARKING'
        function, args = calls[0]
        assert function is hybrid_astar.plan_local_to_spot and args[0] == supervisor.pose
        assert args[1].goal == supervisor.selected.goal
    finally:
        supervisor.worker = worker


def test_complete_to_far_slot_does_not_plan(supervisor, monkeypatch):
    click(supervisor, 4)
    supervisor.pose = supervisor.selected.goal
    supervisor.state = 'COMPLETE'
    monkeypatch.setattr(hybrid_astar, 'plan_local_to_spot', lambda *a, **kw: pytest.fail('不能跨场规划'))
    click(supervisor, 0)
    assert supervisor.state == 'MANUAL_APPROACH' and supervisor.future is None


def test_static_neighbors_remain_when_lidar_does_not_see_them(supervisor):
    supervisor._update_obstacles(spot_detector.DetectResult([]))
    assert {o.name for o in supervisor._planning_obstacles()} == {o.name for o in collision.park_world_obstacles()}
    assert [s.spot_id for s in supervisor.spots if s.occupied] == [2, 3, 4, 6]


def test_correction_limit_never_reports_complete(supervisor):
    click(supervisor, 4)
    x, y, yaw = supervisor.selected.goal
    supervisor.pose = (x + 0.2, y, yaw + 0.1)
    supervisor.state = 'AUTO_PARKING'
    supervisor.adjustments = supervisor.cfg['max_adjustments']
    supervisor._advance_chunk()
    assert supervisor.state == 'HOLD_FOR_MANUAL'


def test_planner_failure_stays_hold(supervisor):
    from concurrent.futures import Future
    click(supervisor, 0)
    supervisor.state = 'AUTO_PARKING'
    supervisor.future = Future()
    supervisor.future.set_result(hybrid_astar.PlanResult([], False, '未找到无碰撞路径', 0, 0, 0, 0))
    supervisor._poll_plan()
    assert supervisor.state == 'HOLD_FOR_MANUAL' and supervisor.reason == '未找到无碰撞路径'


def test_live_obstacle_brakes_then_replans_remaining_path(supervisor):
    click(supervisor, 0)
    supervisor.pose = (-8.0, 12.5311, math.pi / 2)
    result = hybrid_astar.plan_local_to_spot(supervisor.pose, supervisor.selected,
                                            supervisor._planning_obstacles())
    assert result.ok
    supervisor.mode, supervisor.state = 'AUTO', 'AUTO_PARKING'
    supervisor._install_path(result)
    reverse = next(i for i, chunk in enumerate(supervisor.chunks) if chunk.gear < 0)
    supervisor.chunk_idx = reverse
    first = supervisor.chunks[reverse].points[0]
    supervisor.pose = (first.x, first.y, first.yaw)
    blocked = supervisor.chunks[reverse].points[len(supervisor.chunks[reverse].points) // 2]
    supervisor._update_obstacles(spot_detector.DetectResult(
        [collision.Circle('live', blocked.x, blocked.y, 0.15)]))
    assert supervisor.state == 'AUTO_PARKING' and supervisor.replan_pending
    supervisor.speed = 0.2
    supervisor._control_step()
    assert supervisor.future is None and supervisor.adjustments == 0
    supervisor.speed = 0.0
    supervisor._control_step()
    assert supervisor.future is not None and supervisor.adjustments == 1


def test_live_obstacle_at_current_body_still_holds(supervisor):
    supervisor.pose = (-8, 12.5311, math.pi / 2)
    click(supervisor, 0)
    supervisor.mode, supervisor.state = 'AUTO', 'AUTO_PARKING'
    supervisor.dynamic_obstacles = [collision.Circle('live', *supervisor.pose[:2], 0.15)]
    supervisor._control_step()
    assert supervisor.state == 'HOLD_FOR_MANUAL' and not supervisor.replan_pending


def test_already_parked_start_completes_without_empty_chunk(supervisor):
    click(supervisor, 4)
    supervisor.pose = supervisor.selected.goal
    result = hybrid_astar.plan_local_to_spot(supervisor.pose, supervisor.selected,
                                            supervisor._planning_obstacles())
    assert result.ok and result.path_length == 0.0
    supervisor.mode, supervisor.state = 'AUTO', 'AUTO_PARKING'
    supervisor._install_path(result)
    assert supervisor.state == 'COMPLETE'
