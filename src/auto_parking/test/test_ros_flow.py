"""真实 ROS 话题测试，不启动 Gazebo/RViz。"""
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from prius_msgs.msg import Control
from std_msgs.msg import String
from auto_parking.msg import ParkingStatus
from auto_parking.control_mux import ControlMux
from auto_parking.hybrid_park_node import ParkingSupervisor


def test_selection_manual_start_cancel_and_single_control_publisher():
    rclpy.init()
    supervisor, mux, driver = ParkingSupervisor(), ControlMux(), Node('offline_driver')
    executor = SingleThreadedExecutor()
    for node in (supervisor, mux, driver):
        executor.add_node(node)
    controls, statuses, auto_controls = [], [], []
    driver.create_subscription(Control, '/prius/control', controls.append, 10)
    driver.create_subscription(Control, '/parking/auto_control', auto_controls.append, 10)
    driver.create_subscription(ParkingStatus, '/parking/status', statuses.append,
                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = driver.create_publisher(Odometry, '/prius/ground_truth', 10)
    click_pub = driver.create_publisher(PointStamped, '/clicked_point', 10)
    request_pub = driver.create_publisher(String, '/parking/command', 10)
    manual_pub = driver.create_publisher(Control, '/parking/manual_control', 10)

    def wait_for(predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate()

    def publish_pose(pose):
        odom = Odometry()
        odom.pose.pose.position.x, odom.pose.pose.position.y = float(pose[0]), float(pose[1])
        odom.pose.pose.orientation.z = math.sin(pose[2] / 2)
        odom.pose.pose.orientation.w = math.cos(pose[2] / 2)
        odom_pub.publish(odom)

    try:
        wait_for(lambda: bool(statuses) and request_pub.get_subscription_count() == 2
                 and click_pub.get_subscription_count() == 1 and odom_pub.get_subscription_count() == 1)
        publish_pose((0, 0, math.pi))
        wait_for(lambda: supervisor.pose is not None)
        point = PointStamped()
        point.point.x, point.point.y = supervisor.spots[0].center[:2]
        click_pub.publish(point)
        wait_for(lambda: statuses[-1].state == 'MANUAL_APPROACH' and statuses[-1].selected_slot == 1)
        assert not auto_controls and supervisor.future is None
        assert driver.count_publishers('/prius/control') == 1
        assert driver.get_publishers_info_by_topic('/prius/control')[0].node_name == 'control_mux'
        manual_pub.publish(Control(throttle=0.35, shift_gears=Control.FORWARD))
        wait_for(lambda: any(c.throttle == 0.35 for c in controls))
        manual_pub.publish(Control(brake=1.0, shift_gears=Control.NEUTRAL))
        publish_pose((-8.0, 12.5311, math.pi / 2))
        wait_for(lambda: statuses[-1].ready_to_park)
        assert statuses[-1].mode == 'MANUAL' and supervisor.future is None
        frozen_goal = supervisor.selected.goal
        request_pub.publish(String(data='start'))
        wait_for(lambda: statuses[-1].mode == 'AUTO' and statuses[-1].state == 'AUTO_PARKING')
        assert supervisor.selected.goal == frozen_goal
        wait_for(lambda: bool(supervisor.path), timeout=20)
        wait_for(lambda: any(c.throttle > 0 for c in auto_controls))
        wait_for(lambda: any(c.throttle > 0 for c in controls))
        manual_pub.publish(Control(throttle=0.9, shift_gears=Control.FORWARD))
        for _ in range(10):
            executor.spin_once(timeout_sec=0.01)
        assert mux.source == 'auto' and mux.command.throttle != 0.9
        request_pub.publish(String(data='cancel'))
        wait_for(lambda: statuses[-1].state == 'HOLD_FOR_MANUAL')
        wait_for(lambda: mux.source is None and controls[-1].brake == 1.0)
        assert driver.count_publishers('/prius/control') == 1
    finally:
        executor.shutdown()
        for node in (driver, mux, supervisor):
            node.destroy_node()
        rclpy.shutdown()
