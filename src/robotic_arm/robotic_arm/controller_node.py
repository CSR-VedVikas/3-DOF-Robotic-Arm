import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.joint_names = ['joint1', 'joint2', 'joint3']
        self.current_positions = [0.0, 0.0, 0.0]

        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'joint_angles',
            self.angle_callback,
            10
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        self.timer = self.create_timer(0.05, self.publish_joint_state)

        self.get_logger().info('Controller active: publishing /joint_states for RViz.')

    def angle_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return

        self.current_positions = [
            float(msg.data[0]),
            float(msg.data[1]),
            float(msg.data[2])
        ]

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        self.joint_state_pub.publish(msg)


def main():
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()