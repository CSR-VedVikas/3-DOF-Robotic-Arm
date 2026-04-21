import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from robotic_arm.ik_utils import forward_kinematics


class EEMarkerNode(Node):
    def __init__(self):
        super().__init__('ee_marker_node')
        self.create_subscription(JointState, 'joint_states', self._cb, 10)
        self._pub = self.create_publisher(Marker, 'ee_marker', 10)

    def _cb(self, msg: JointState):
        if len(msg.position) < 3:
            return
        ee, _, _, _ = forward_kinematics(
            msg.position[0], msg.position[1], msg.position[2]
        )
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'end_effector'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(ee[0])
        m.pose.position.y = float(ee[1])
        m.pose.position.z = float(ee[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.09
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        self._pub.publish(m)


def main():
    rclpy.init()
    node = EEMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()