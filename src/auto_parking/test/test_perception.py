import math
import struct

import pytest

from sensor_msgs.msg import PointCloud2, PointField
from auto_parking import collision, spot_detector
from test_supervisor import supervisor


def test_cloud_height_is_filtered_in_world_frame():
    cloud = PointCloud2(height=1, width=2, point_step=12, row_step=24)
    cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                    for name, offset in (('x', 0), ('y', 4), ('z', 8))]
    cloud.data = struct.pack('<ffffff', 4, 0, -0.73, 4, 0, -1.73)
    local = spot_detector.parse_cloud_xy(cloud, stride=1)
    assert len(local) == 2
    assert spot_detector.transform_xy(local, 0, 0, 1.73, 1, 0, 0, 0) == [(4, 0)]


def test_near_obstacle_is_retained_outside_ego_body():
    result = spot_detector.detect_from_points([(0, 1.4), (1, 0)], ego=(0, 0, 0))
    assert result.occupied_cells == 1 and len(result.obstacles) == 1
    assert collision.pose_collides(0, 0.5, 0, result.obstacles)


def test_lidar_cluster_updates_known_spot_occupancy():
    from auto_parking.spots import detect_spots
    known = detect_spots()
    x, y, _ = known[0].center
    points = [(x + i * 0.25, y + j * 0.25) for i in range(-5, 6) for j in range(-3, 4)]
    result = spot_detector.detect_from_points(points)
    spot_detector.mark_occupied(known, result.grid)
    assert known[0].occupied


@pytest.mark.parametrize('upper_arm', [False, True], ids=['L-shaped', 'U-shaped'])
def test_sparse_surface_does_not_fill_free_interior(upper_arm):
    points = [(-8 + i * 0.1, -2) for i in range(31)]
    points += [(-8, -2 + i * 0.1) for i in range(81 if upper_arm else 41)]
    if upper_arm:
        points += [(-8 + i * 0.1, 6) for i in range(31)]
    result = spot_detector.detect_from_points(points)
    assert not collision.pose_collides(-5.6, 0.2, math.pi / 2, result.obstacles)
    assert collision.pose_collides(-8.5, -1.5, math.pi / 2, result.obstacles)
    assert sum(o.length * o.width for o in result.obstacles) == pytest.approx(
        result.occupied_cells * spot_detector.GRID_RES ** 2)


def test_static_surface_returns_do_not_inflate_map_neighbors():
    obstacles = collision.park_world_obstacles()
    points = []
    for obs in obstacles:
        if isinstance(obs, collision.Rect):
            corners = obs.corners()
            for a, b in zip(corners, corners[1:] + corners[:1]):
                points += [(a[0] + (b[0] - a[0]) * i / 40,
                            a[1] + (b[1] - a[1]) * i / 40) for i in range(41)]
    result = spot_detector.detect_from_points(points, static_obstacles=obstacles)
    assert result.occupied_cells > 0 and result.obstacles == []
    result = spot_detector.detect_from_points(points + [(-8, 0)], static_obstacles=obstacles)
    assert len(result.obstacles) == 1 and result.obstacles[0].clearance(-8, 0) == 0.0


@pytest.mark.parametrize('point', [(5, 0), (-30, 12), (-8, -20)])
def test_lidar_bounds_cover_wider_nearby_maneuvers(point):
    result = spot_detector.detect_from_points([point])
    assert result.occupied_cells == 1
    assert any(obstacle.clearance(*point) == 0.0 for obstacle in result.obstacles)


def test_cloud_and_ego_are_transformed_at_scan_time(supervisor):
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    for stamp, x in ((10, 0.0), (11, 1.0)):
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
        tf.header.stamp = Time(sec=stamp)
        tf.transform.translation.x = x
        tf.transform.rotation.w = 1.0
        supervisor.tf_buffer.set_transform(tf, 'test')
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'base_link', 'laser'
    tf.transform.translation.z = 1.73
    tf.transform.rotation.w = 1.0
    supervisor.tf_buffer.set_transform_static(tf, 'test')
    cloud = PointCloud2(height=1, width=8, point_step=12, row_step=96)
    cloud.header.frame_id, cloud.header.stamp = 'laser', Time(sec=10)
    cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                    for name, offset in (('x', 0), ('y', 4), ('z', 8))]
    cloud.data = b''.join(struct.pack('<fff', *p) for p in
                          [(0, 0, -0.73)] * 4 + [(-5, 1, -0.73)] * 4)
    supervisor.pose = (3, 0, 0)
    supervisor._on_cloud(cloud)
    supervisor._perceive_step()
    assert len(supervisor.dynamic_obstacles) == 1
    obstacle = supervisor.dynamic_obstacles[0]
    assert obstacle.clearance(-5, 1) == 0.0 and obstacle.clearance(-4, 1) > 0.0
    assert obstacle.clearance(0, 0) > 0.0
