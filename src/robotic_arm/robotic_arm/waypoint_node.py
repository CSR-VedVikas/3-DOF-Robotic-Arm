import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')
        self.declare_parameter('interval_sec', 1.5)
        self._interval = float(self.get_parameter('interval_sec').value)
        self._queue = []

        self.create_subscription(
            Float32MultiArray, 'planned_waypoints', self._wps_cb, 10
        )
        self._pub = self.create_publisher(Float32MultiArray, 'ik_target_position', 10)
        self.create_timer(self._interval, self._tick)
        self.get_logger().info('WaypointNode ready.')

    def _wps_cb(self, msg):
        data = list(msg.data)
        new_queue = [[data[i],data[i+1],data[i+2]] for i in range(0,len(data),3)]
        if not self._queue or new_queue[-1] != self._queue[-1]:
            self._queue = new_queue
    def _tick(self):
        if not self._queue:
            return
        wp = self._queue.pop(0)
        out = Float32MultiArray()
        out.data = [float(v) for v in wp]
        self._pub.publish(out)
        self.get_logger().info(f'Sending to IK: {[f"{v:.3f}" for v in wp]}')


def main():
    rclpy.init()
    node = WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()