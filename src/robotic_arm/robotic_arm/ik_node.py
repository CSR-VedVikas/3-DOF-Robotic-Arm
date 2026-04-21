import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from robotic_arm.ik_utils import inverse_kinematics


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.target_sub = self.create_subscription(
            Float32MultiArray,
            'ik_target_position',
            self.target_callback,
            10
        )

        self.angles_pub = self.create_publisher(
            Float32MultiArray,
            'joint_angles',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            'ik_status',
            10
        )

        self.declare_parameter('l1', 1.0)
        self.declare_parameter('l2', 1.0)
        self.declare_parameter('l3', 1.0)
        self.declare_parameter('base_height', 0.05)

    def target_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warning('ik_target_position needs 3 values')
            return

        x, y, z = float(msg.data[0]), float(msg.data[1]), float(msg.data[2])

        l1 = float(self.get_parameter('l1').value)
        l2 = float(self.get_parameter('l2').value)
        l3 = float(self.get_parameter('l3').value)
        base_height = float(self.get_parameter('base_height').value)

        theta1, theta2, theta3, reachable = inverse_kinematics(
            x, y, z, l1=l1, l2=l2, l3=l3, base_height=base_height
        )

        status = String()

        if theta1 is None or theta2 is None or theta3 is None:
            status.data = 'IK_FAILED'
            self.status_pub.publish(status)
            self.get_logger().warning(f'IK failed for target ({x:.2f}, {y:.2f}, {z:.2f})')
            return

        angles_msg = Float32MultiArray()
        angles_msg.data = [theta1, theta2, theta3]
        self.angles_pub.publish(angles_msg)

        if reachable:
            status.data = 'IK_OK'
            self.get_logger().info(
                f'Target ({x:.2f}, {y:.2f}, {z:.2f}) -> '
                f'θ1={math.degrees(theta1):.2f}, '
                f'θ2={math.degrees(theta2):.2f}, '
                f'θ3={math.degrees(theta3):.2f}'
            )
        else:
            status.data = 'IK_CLAMPED'
            self.get_logger().warn(
                f'Target ({x:.2f}, {y:.2f}, {z:.2f}) is outside reach; using closest solution.'
            )

        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()