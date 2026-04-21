
import math
from typing import Tuple


def forward_kinematics(
    theta1: float,
    theta2: float,
    theta3: float,
    l1: float = 1.0,
    l2: float = 1.0,
    l3: float = 1.0,
    base_height: float = 0.05,
) -> Tuple[float, float, float]:
    """
    Simple 3-DOF FK matching your IK model:
      joint1 = yaw about Z
      joint2 = pitch
      joint3 = pitch
    """
    planar_r = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)
    z = base_height + l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)
    x = planar_r * math.cos(theta1)
    y = planar_r * math.sin(theta1)
    return x, y, z