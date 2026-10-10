#!/usr/bin/env python3
"""在线流程唯一的 Prius 控制输出。"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from prius_msgs.msg import Control
from std_msgs.msg import String
from auto_parking.msg import ParkingStatus


class ControlMux(Node):
    def __init__(self):
        super().__init__('control_mux')
        self.source = None
        self.command = None
        self.command_time = 0.0
        self.status_time = 0.0
        self.pub = self.create_publisher(Control, '/prius/control', 10)
        self.create_subscription(Control, '/parking/manual_control', self._manual, 10)
        self.create_subscription(Control, '/parking/auto_control', self._auto, 10)
        self.create_subscription(String, '/parking/command', self._request, 10)
        self.create_subscription(ParkingStatus, '/parking/status', self._status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.02, self._tick)

    def _status(self, status):
        self.status_time = time.monotonic()
        source = None
        if status.mode == 'MANUAL' and status.state in (
                'MANUAL', 'SLOT_SELECTED', 'MANUAL_APPROACH', 'READY_TO_PARK'):
            source = 'manual'
        elif status.mode == 'AUTO' and status.state == 'AUTO_PARKING':
            source = 'auto'
        if source != self.source:
            self.command = None
            self._brake()
        self.source = source

    def _request(self, msg):
        if msg.data in ('manual', 'auto', 'start', 'cancel', 'estop'):
            self.source = None
            self.command = None
            self._brake()

    def _manual(self, msg):
        if self.source == 'manual':
            self.command, self.command_time = msg, time.monotonic()

    def _auto(self, msg):
        if self.source == 'auto':
            self.command, self.command_time = msg, time.monotonic()

    def _brake(self):
        msg = Control()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.brake = 1.0
        msg.shift_gears = Control.NEUTRAL
        self.pub.publish(msg)

    def _tick(self):
        now = time.monotonic()
        if (self.source is None or self.command is None
                or now - self.command_time > 0.3 or now - self.status_time > 0.6):
            self._brake()
        else:
            self.command.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.command)


def main(args=None):
    rclpy.init(args=args)
    node = ControlMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._brake()
        node.destroy_node()
        rclpy.shutdown()
