import math

import pytest

from auto_parking import collision, hybrid_astar, parking_strategy, spots
from test_geometry import assert_kinematic_path


PARKING_CASES = [
    pytest.param(0, (-11.9053, 15.0311, math.pi / 2), id='perpendicular-original-pose'),
    pytest.param(0, (-11.9053, 15.0311, -math.pi / 2), id='perpendicular-opposite-heading'),
    pytest.param(0, (-8.9053, 15.0311, math.pi), id='perpendicular-cross-heading'),
    pytest.param(0, (-8.0, 12.5311, math.pi / 2), id='perpendicular-front-aisle'),
    pytest.param(0, (-10.5, 18.0, -math.pi / 2), id='perpendicular-north-aisle'),
    pytest.param(0, (-11.2, 12.5, 0), id='perpendicular-facing-away'),
    pytest.param(0, (-11.3, 12.5, math.pi), id='perpendicular-facing-slot'),
    pytest.param(0, (-10.0, 7.55, 0), id='perpendicular-outside-old-admission'),
    pytest.param(4, (-7.8385, -4.82928, -math.pi / 2), id='parallel-original-pose'),
    pytest.param(4, (-6.5, -1.6, -math.pi / 2), id='parallel-next-to-slot'),
    pytest.param(4, (-5.0, 3.0, 0), id='parallel-outside-old-admission'),
    pytest.param(4, (-7.9745, -3.92928, math.radians(-53.88)), id='parallel-screenshot-estimate'),
    pytest.param(4, (0, 0, math.pi), id='parallel-nine-metres-from-slot'),
    pytest.param(4, (0, -5, math.pi / 2), id='parallel-wide-aisle'),
    pytest.param(4, (-7, 7, math.pi / 2), id='parallel-past-north-neighbor'),
    pytest.param(0, (-2, 12.5311, math.pi / 2), id='perpendicular-ten-metres-from-slot'),
    pytest.param(0, (-20.5, 12.5311, 0), id='perpendicular-forward-entry'),
    pytest.param(4, (-10.6385, 0.75, -math.pi / 2), id='parallel-forward-entry'),
    pytest.param(4, (-10.8385, -0.07928, -math.pi / 2 + math.radians(8)), id='parallel-inside-misaligned'),
    pytest.param(0, (-21.3803810968, 21.5134492076, -1.0126079768), id='perpendicular-northwest-diagonal'),
    pytest.param(0, (-21.3044032230, 7.6113980235, 1.1343055216), id='perpendicular-southwest-diagonal'),
    pytest.param(0, (-19.3565071567, 19.9615537774, -2.9002635291), id='perpendicular-behind-north-neighbor'),
    pytest.param(0, (-15.2292680342, 19.4738284903, -2.9462307382), id='perpendicular-above-north-neighbor'),
    pytest.param(4, (-3.0780658499, -6.4892865929, 2.5922333016), id='parallel-south-aisle-diagonal'),
]


@pytest.mark.parametrize('spot_index,start', PARKING_CASES)
def test_local_parking(spot_index, start):
    obstacles = collision.park_world_obstacles()
    spot = spots.detect_spots(obstacles)[spot_index]
    spot.goal = parking_strategy.parking_goal(spot, obstacles)
    assert parking_strategy.distance_to_spot(*start[:2], spot) <= 12.0
    result = hybrid_astar.plan_local_to_spot(start, spot, obstacles)
    assert result.ok, (result.reason, result.expanded, result.elapsed_ms)
    assert collision.path_clearance(result.path, obstacles)[0] > 0.12
    assert_kinematic_path(result.path)
    end = result.path[-1]
    assert parking_strategy.parking_complete((end.x, end.y, end.yaw), 0, spot, obstacles)


@pytest.mark.parametrize('index,start', [(0, (-20.5, 12.5311, 0)),
                                       (4, (-10.6385, 0.75, -math.pi / 2))])
def test_straight_forward_entry_avoids_unnecessary_reverse_detour(index, start):
    obstacles = collision.park_world_obstacles()
    spot = spots.detect_spots(obstacles)[index]
    result = hybrid_astar.plan_local_to_spot(start, spot, obstacles)
    assert result.ok
    assert {p.gear for p in result.path} == {1}
    assert result.gear_changes == 0
    assert result.path_length == pytest.approx(math.dist(start[:2], spot.goal[:2]), abs=1e-3)


def test_inside_slot_allows_forward_and_reverse_alignment():
    obstacles = collision.park_world_obstacles()
    spot = spots.detect_spots(obstacles)[4]
    start = (spot.goal[0] - 0.20, spot.goal[1] + 0.15, spot.goal[2] + math.radians(8))
    assert parking_strategy.body_in_spot(start, spot)
    result = hybrid_astar.plan_local_to_spot(start, spot, obstacles, adjust=True)
    assert result.ok, result.reason
    assert {p.gear for p in result.path} == {-1, 1}
    assert_kinematic_path(result.path)
    end = result.path[-1]
    assert parking_strategy.parking_complete((end.x, end.y, end.yaw), 0, spot, obstacles)
    assert collision.path_clearance(result.path, obstacles)[0] > 0.12


def test_distant_pose_does_not_search():
    obstacles = collision.park_world_obstacles()
    spot = spots.detect_spots(obstacles)[0]
    result = hybrid_astar.plan_local_to_spot((0, 0, math.pi), spot, obstacles)
    assert not result.ok and result.expanded == 0


def test_blocked_goal_does_not_fall_back_to_ignoring_obstacle():
    obstacles = collision.park_world_obstacles()
    spot = spots.detect_spots(obstacles)[0]
    obstacles += [collision.Circle('live', *spot.goal[:2], 0.3)]
    result = hybrid_astar.plan_local_to_spot((-8.0, 12.5311, math.pi / 2), spot, obstacles)
    assert not result.ok and result.expanded == 0
