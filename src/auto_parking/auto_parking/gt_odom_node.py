#!/usr/bin/env python3
"""把 Gazebo 真值里程计转成 TF，供 RViz 在世界坐标下显示路径。

prius.urdf 的根 link 是 dummy，robot_state_publisher 已经发布
dummy -> base_link（恒等变换）。所以这里必须广播 odom -> dummy，
否则 base_link 会有两个父节点，TF 树报冲突。
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('gt_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'dummy')
        self.declare_parameter('ground_truth_topic', '/prius/ground_truth')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        topic = self.get_parameter('ground_truth_topic').value

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, topic, self._on_odom, qos_profile_sensor_data)
        self._warned = False
        self.get_logger().info(
            f'{topic} -> TF {self.odom_frame} -> {self.base_frame}')

    def _on_odom(self, msg):
        pose = msg.pose.pose
        if not self._warned:
            self._warned = True
            self.get_logger().info(
                f'首帧真值: x={pose.position.x:.2f} y={pose.position.y:.2f}')

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
