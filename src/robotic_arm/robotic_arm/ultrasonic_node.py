import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from robotic_arm.ik_utils import forward_kinematics


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        self.declare_parameter('obstacle_x', 1.2)
        self.declare_parameter('obstacle_y', 0.5)
        self.declare_parameter('obstacle_z', 0.3)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.obs = (
            float(self.get_parameter('obstacle_x').value),
            float(self.get_parameter('obstacle_y').value),
            float(self.get_parameter('obstacle_z').value),
        )
        self.rate = float(self.get_parameter('publish_rate_hz').value)

        self._lock = threading.Lock()
        self._joints = [0.0, 0.0, 0.0]

        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_cb, 10
        )
        self.pub = self.create_publisher(Float32, 'ultrasonic_distance', 10)
        self.timer = self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(
            f'Ultrasonic node: simulating distance from EE to obstacle at {self.obs}'
        )

    def _joint_cb(self, msg: JointState):
        if len(msg.position) >= 3:
            with self._lock:
                self._joints = list(msg.position[:3])

    def _publish(self):
        with self._lock:
            t1, t2, t3 = self._joints
        ee, _, _, _ = forward_kinematics(t1, t2, t3)
        dx = ee[0] - self.obs[0]
        dy = ee[1] - self.obs[1]
        dz = ee[2] - self.obs[2]
        dist_cm = math.sqrt(dx * dx + dy * dy + dz * dz) * 100.0
        msg = Float32()
        msg.data = float(dist_cm)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()