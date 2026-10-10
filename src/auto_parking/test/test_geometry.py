import math

import pytest

from auto_parking import collision, hybrid_astar, parking_strategy, reeds_shepp, spots, vehicle
from auto_parking.path_geometry import PathPoint


def test_contact_and_contained_obstacles_collide():
    obstacles = [collision.Rect('touch', 4.25, 0, 0, 1, 1)]
    assert hybrid_astar._collides(0, 0, 0, obstacles, margin=0.0)
    for obstacle in (collision.Circle('inside', 1, 0, 0.1),
                     collision.Rect('inside', 1, 0, 0, 0.2, 0.2)):
        assert collision.pose_clearance(0, 0, 0, [obstacle]) == 0


def test_clearance_is_exact_and_checks_last_point():
    obstacle = collision.Rect('car', 0, 2, 0, 6, 1)
    assert collision.pose_clearance(0, 0, 0, [obstacle]) == pytest.approx(0.6)
    path = [PathPoint(0, 0, 0, 1), PathPoint(0, 0.7, 0, 1)]
    assert collision.path_clearance(path, [obstacle])[0] == 0


def test_dense_validation_detects_obstacle_between_endpoints():
    path = [PathPoint(0, 0, 0, 1), PathPoint(8, 0, 0, 1)]
    obstacle = collision.Circle('between', 5, 0, 0.04)
    assert not collision.pose_collides(0, 0, 0, [obstacle])
    assert not collision.pose_collides(8, 0, 0, [obstacle])
    assert collision.path_clearance(path, [obstacle])[0] == 0


def test_reverse_bicycle_integration():
    forward = hybrid_astar._integrate(0, 0, 0, 1, 0.4, 1)
    reverse = hybrid_astar._integrate(*forward[:3], -1, 0.4, 1)
    assert reverse[:3] == pytest.approx((0, 0, 0), abs=1e-9)


@pytest.mark.parametrize('goal', [(5, 3, 1), (-4, 2, -1.5), (0.01, 0.1, 0.2)])
def test_reeds_shepp_reaches_goal_without_snap(goal):
    paths = list(reeds_shepp.paths((0, 0, 0), goal, vehicle.MIN_TURN_RADIUS))
    assert paths
    for path in paths:
        assert (path[-1].x, path[-1].y, path[-1].yaw) == pytest.approx(goal, abs=1e-6)
        assert_kinematic_path(path)


def assert_kinematic_path(path):
    for a, b in zip(path, path[1:]):
        yaw_change = vehicle.normalize_angle(b.yaw - a.yaw)
        chord = math.hypot(b.x - a.x, b.y - a.y)
        if chord < 1e-8:
            continue
        length = (abs(yaw_change / b.curvature) if abs(b.curvature) > 1e-8 else chord)
        expected = hybrid_astar._integrate(a.x, a.y, a.yaw, b.gear,
                                           math.atan(b.curvature * vehicle.WHEELBASE), length)
        assert (b.x, b.y, b.yaw) == pytest.approx(expected[:3], abs=1e-6)


def test_completion_requires_actual_pose_speed_body_and_clearance():
    obstacles = collision.park_world_obstacles()
    for spot in (spots.detect_spots(obstacles)[0], spots.detect_spots(obstacles)[4]):
        assert parking_strategy.parking_complete(spot.goal, 0, spot, obstacles)
        x, y, yaw = spot.goal
        assert not parking_strategy.parking_complete((x + 0.25, y, yaw), 0, spot, obstacles)
        assert not parking_strategy.parking_complete((x, y, yaw + 0.1), 0, spot, obstacles)
        assert not parking_strategy.parking_complete(spot.goal, 0.2, spot, obstacles)
        assert not parking_strategy.parking_complete(spot.goal, 0, spot,
                                                     [collision.Circle('blocked', x, y, 0.1)])
        assert not parking_strategy.body_in_spot((x, y, yaw + 0.3), spot)


def test_fast_minimum_clearance_matches_full_geometry_report():
    obstacles = collision.park_world_obstacles()
    path = [PathPoint(-7, -4, -1, 1), PathPoint(-6, -6, -1.5, 1)]
    assert collision.path_min_clearance(path, obstacles) == pytest.approx(
        collision.path_clearance(path, obstacles)[0])


@pytest.mark.parametrize('index', [0, 4])
def test_nearby_marker_matches_distance_to_rotated_slot(index):
    from builtin_interfaces.msg import Time
    from auto_parking import ribbon
    spot = spots.detect_spots()[index]
    boundary, label = ribbon.parking_area(spot, 6.0, False, 'odom', Time())
    assert 'staging' not in boundary.ns and '6 m' in label.text
    for point in boundary.points:
        assert parking_strategy.distance_to_spot(point.x, point.y, spot) == pytest.approx(6.0)
    assert parking_strategy.distance_to_spot(*spot.center[:2], spot) == 0.0
