import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')

        self.declare_parameter('obstacle_x', 1.2)
        self.declare_parameter('obstacle_y', 0.5)
        self.declare_parameter('obstacle_z', 0.3)
        self.declare_parameter('obstacle_radius', 0.25)
        self.declare_parameter('stop_distance_cm', 30.0)
        self.declare_parameter('warning_distance_cm', 60.0)

        self.obs_x = float(self.get_parameter('obstacle_x').value)
        self.obs_y = float(self.get_parameter('obstacle_y').value)
        self.obs_z = float(self.get_parameter('obstacle_z').value)
        self.obs_r = float(self.get_parameter('obstacle_radius').value)
        self.stop_d = float(self.get_parameter('stop_distance_cm').value)
        self.warn_d = float(self.get_parameter('warning_distance_cm').value)

        self.marker_pub = self.create_publisher(Marker, 'obstacle_marker', 10)
        self.detect_pub = self.create_publisher(Bool, 'obstacle_detected', 10)
        self.status_pub = self.create_publisher(String, 'obstacle_status', 10)

        self.create_subscription(Float32, 'ultrasonic_distance', self._dist_cb, 10)
        # Publish static obstacle marker at 2 Hz regardless of arm state
        self.create_timer(0.5, self._publish_obstacle_marker)

        self._last_status = None

    def _publish_obstacle_marker(self):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'obstacle'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = self.obs_x
        m.pose.position.y = self.obs_y
        m.pose.position.z = self.obs_z
        m.pose.orientation.w = 1.0
        d = self.obs_r * 2.0
        m.scale.x = d
        m.scale.y = d
        m.scale.z = d
        # Orange semi-transparent obstacle
        m.color.r = 1.0
        m.color.g = 0.45
        m.color.b = 0.0
        m.color.a = 0.75
        self.marker_pub.publish(m)

    def _dist_cb(self, msg: Float32):
        dist = float(msg.data)

        if dist <= self.stop_d:
            status, obstacle = 'DANGER', True
        elif dist <= self.warn_d:
            status, obstacle = 'WARNING', True
        else:
            status, obstacle = 'CLEAR', False

        b = Bool()
        b.data = obstacle
        self.detect_pub.publish(b)

        s = String()
        s.data = status
        self.status_pub.publish(s)

        if self._last_status != status:
            self._last_status = status
            self.get_logger().info(f'EE→obstacle: {dist:.1f} cm → {status}')


def main():
    rclpy.init()
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()