import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker

from robotic_arm.ik_utils import forward_kinematics

# ── pure-Python 3-D vector helpers ─────────────────────────────────────────

def _sub(a, b):    return [a[i] - b[i] for i in range(3)]
def _add(a, b):    return [a[i] + b[i] for i in range(3)]
def _scale(a, s):  return [a[i] * s for i in range(3)]
def _dot(a, b):    return sum(a[i] * b[i] for i in range(3))
def _norm(a):      return math.sqrt(sum(x * x for x in a))
def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def _unit(a):
    n = _norm(a)
    return _scale(a, 1.0 / n) if n > 1e-9 else [0.0, 0.0, 0.0]
def _dist(a, b):   return _norm(_sub(a, b))


# ── collision geometry ──────────────────────────────────────────────────────

def _segment_sphere_hit(p1, p2, center, radius) -> bool:
    """True if segment p1→p2 passes within radius of center."""
    d = _sub(p2, p1)
    f = _sub(p1, center)
    a = _dot(d, d)
    if a < 1e-12:
        return _norm(f) < radius
    b = 2.0 * _dot(f, d)
    c = _dot(f, f) - radius * radius
    disc = b * b - 4.0 * a * c
    if disc < 0:
        return False
    sq = math.sqrt(disc)
    t1 = (-b - sq) / (2.0 * a)
    t2 = (-b + sq) / (2.0 * a)
    return (0.0 <= t1 <= 1.0) or (0.0 <= t2 <= 1.0) or (t1 < 0.0 < t2)


def _plan_waypoints(start, goal, obs, obs_r, margin=0.35):
    """
    Returns list of (x,y,z) waypoints from start→goal avoiding the obstacle sphere.
    The returned list ends with the goal position.
    """
    total_r = obs_r + margin

    if not _segment_sphere_hit(start, goal, obs, total_r):
        return [goal]

    # Closest point on path to obstacle
    d = _sub(goal, start)
    d_len = _norm(d)
    if d_len < 1e-6:
        return [goal]
    d_unit = _unit(d)
    t = max(0.0, min(d_len, _dot(_sub(obs, start), d_unit)))
    closest = _add(start, _scale(d_unit, t))

    # Perpendicular basis vectors for generating candidates
    up = [0.0, 0.0, 1.0]
    right = _unit(_cross(d_unit, up))
    if _norm(right) < 0.01:
        right = [1.0, 0.0, 0.0]

    bypass_r = obs_r + margin + 0.25

    # Five candidate bypass points around the obstacle
    s2 = math.sqrt(2.0)
    candidates = [
        _add(obs, _scale(up, bypass_r)),                                        # above
        _add(obs, _scale(right, bypass_r)),                                     # right
        _add(obs, _scale(right, -bypass_r)),                                    # left
        _add(obs, _scale(_unit(_add(up, right)), bypass_r)),                    # upper-right
        _add(obs, _scale(_unit(_sub(up, right)), bypass_r)),                    # upper-left
    ]
    candidates = [c for c in candidates if c[2] >= 0.05]

    check_r = obs_r + margin * 0.6
    best, best_cost = None, float('inf')
    for c in candidates:
        ok = (not _segment_sphere_hit(start, c, obs, check_r) and
              not _segment_sphere_hit(c, goal, obs, check_r))
        cost = _dist(start, c) + _dist(c, goal)
        if ok and cost < best_cost:
            best, best_cost = c, cost

    if best:
        return [best, goal]

    # Fallback: climb high above obstacle
    high = _add(obs, [0.0, 0.0, obs_r + margin + 0.8])
    return [high, goal]

# ── existing helpers above (_sub, _add, _dist, _plan_waypoints etc.) ──

def _links_clear(joints, obs, total_r):
    """Check all arm link segments against the obstacle sphere."""
    for i in range(len(joints) - 1):
        if _segment_sphere_hit(joints[i], joints[i+1], obs, total_r):
            return False
    return True
# ── ROS node ────────────────────────────────────────────────────────────────

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.create_timer(0.2, self._recheck_collision)
        self.declare_parameter('obstacle_x', 1.2)
        self.declare_parameter('obstacle_y', 0.5)
        self.declare_parameter('obstacle_z', 0.3)
        self.declare_parameter('obstacle_radius', 0.25)
        self.declare_parameter('safety_margin', 0.35)

        self._obs = [
            float(self.get_parameter('obstacle_x').value),
            float(self.get_parameter('obstacle_y').value),
            float(self.get_parameter('obstacle_z').value),
        ]
        self._obs_r = float(self.get_parameter('obstacle_radius').value)
        self._margin = float(self.get_parameter('safety_margin').value)

        self._current_ee = [0.0, 0.0, 0.05]

        self.create_subscription(Float32MultiArray, 'target_position', self._target_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)

        self._wp_pub = self.create_publisher(Float32MultiArray, 'planned_waypoints', 10)
        self._path_pub = self.create_publisher(Marker, 'path_marker', 10)
        self._tgt_pub = self.create_publisher(Marker, 'target_marker', 10)

        self.get_logger().info('PlannerNode ready — geometric obstacle avoidance active.')

    def _joint_cb(self, msg: JointState):
        if len(msg.position) >= 3:
            ee, _, _, _ = forward_kinematics(
                msg.position[0], msg.position[1], msg.position[2]
            )
            self._current_ee = list(ee)

    def _recheck_collision(self):
        if not hasattr(self, '_last_goal') or self._last_goal is None:
            return
        start = list(self._current_ee)
        wps = _plan_waypoints(start, self._last_goal, self._obs, self._obs_r, self._margin)
        flat = [v for wp in wps for v in wp]
        out = Float32MultiArray()
        out.data = flat
        self._wp_pub.publish(out)

    def _target_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        goal = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
        self._last_goal = goal
        start = list(self._current_ee)

        wps = _plan_waypoints(start, goal, self._obs, self._obs_r, self._margin)

        # ── NEW: validate each waypoint using full arm link geometry ──
        total_r = self._obs_r + self._margin
        safe_wps = []
        for wp in wps:
            from robotic_arm.ik_utils import inverse_kinematics, forward_kinematics
            t1, t2, t3, reachable = inverse_kinematics(wp[0], wp[1], wp[2])
            if not reachable:
                continue
            ee, j3, j2, j1 = forward_kinematics(t1, t2, t3)
            joints = [list(j1), list(j2), list(j3), list(ee)]
            if _links_clear(joints, self._obs, total_r):
                safe_wps.append(wp)
            else:
                # link clips obstacle — nudge waypoint higher and retry
                nudged = [wp[0], wp[1], wp[2] + 0.3]
                t1n, t2n, t3n, rn = inverse_kinematics(nudged[0], nudged[1], nudged[2])
                if rn:
                    ee2, j3n, j2n, j1n = forward_kinematics(t1n, t2n, t3n)
                    joints2 = [list(j1n), list(j2n), list(j3n), list(ee2)]
                    if _links_clear(joints2, self._obs, total_r):
                        safe_wps.append(nudged)

        # fall back to original wps if all got filtered out
        final_wps = safe_wps if safe_wps else wps
        # ── END NEW ──

        flat = [v for wp in final_wps for v in wp]
        out = Float32MultiArray()
        out.data = flat
        self._wp_pub.publish(out)

        self._publish_path_marker(start, final_wps)
        self._publish_target_marker(goal)

    def _make_point(self, xyz):
        p = Point()
        p.x, p.y, p.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        return p

    def _publish_path_marker(self, start, wps):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'planned_path'
        m.id = 10
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.035
        m.color.r = 0.0
        m.color.g = 0.85
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime.sec = 15

        m.points.append(self._make_point(start))
        for wp in wps:
            m.points.append(self._make_point(wp))

        self._path_pub.publish(m)

        # Also publish sphere markers at each waypoint
        for i, wp in enumerate(wps[:-1]):   # intermediate waypoints only
            sm = Marker()
            sm.header.frame_id = 'base_link'
            sm.header.stamp = self.get_clock().now().to_msg()
            sm.ns = 'waypoint_spheres'
            sm.id = 20 + i
            sm.type = Marker.SPHERE
            sm.action = Marker.ADD
            sm.pose.position = self._make_point(wp)
            sm.pose.orientation.w = 1.0
            sm.scale.x = sm.scale.y = sm.scale.z = 0.12
            sm.color.r = 0.0
            sm.color.g = 0.85
            sm.color.b = 1.0
            sm.color.a = 0.8
            sm.lifetime.sec = 15
            self._path_pub.publish(sm)

    def _publish_target_marker(self, goal):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'target'
        m.id = 30
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = self._make_point(goal)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r = 0.1
        m.color.g = 1.0
        m.color.b = 0.1
        m.color.a = 1.0
        m.lifetime.sec = 60
        self._tgt_pub.publish(m)




def main():
    rclpy.init()
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()