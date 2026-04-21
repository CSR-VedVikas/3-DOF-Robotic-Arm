import math
from typing import Optional, Tuple


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def forward_kinematics(
    theta1: float,
    theta2: float,
    theta3: float,
    l1: float = 1.0,
    l2: float = 1.0,
    l3: float = 1.0,
    base_height: float = 0.05,
) -> Tuple[Tuple, Tuple, Tuple, Tuple]:
    """
    Returns (end_effector, joint3_pos, joint2_pos, joint1_pos) as (x, y, z) tuples.
    Matches the URDF: joint1 yaws about Z, joint2+3 pitch about Y.
    """
    cx = math.cos(theta1)
    cy = math.sin(theta1)

    j1 = (0.0, 0.0, base_height)
    j2 = (l1 * cx, l1 * cy, base_height)
    j3 = (
        j2[0] + l2 * cx * math.cos(theta2),
        j2[1] + l2 * cy * math.cos(theta2),
        j2[2] + l2 * math.sin(theta2),
    )
    ee = (
        j3[0] + l3 * cx * math.cos(theta2 + theta3),
        j3[1] + l3 * cy * math.cos(theta2 + theta3),
        j3[2] + l3 * math.sin(theta2 + theta3),
    )
    return ee, j3, j2, j1


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    l1: float = 1.0,
    l2: float = 1.0,
    l3: float = 1.0,
    base_height: float = 0.05,
) -> Tuple[Optional[float], Optional[float], Optional[float], bool]:
    theta1 = math.atan2(y, x)
    r = math.sqrt(x * x + y * y)
    u = r - l1
    v = z - base_height
    dist2 = u * u + v * v
    reachable = (abs(l2 - l3)) ** 2 <= dist2 <= (l2 + l3) ** 2
    D = clamp((dist2 - l2 * l2 - l3 * l3) / (2.0 * l2 * l3), -1.0, 1.0)
    theta3 = math.atan2(-math.sqrt(max(0.0, 1.0 - D * D)), D)
    theta2 = math.atan2(v, u) - math.atan2(
        l3 * math.sin(theta3), l2 + l3 * math.cos(theta3)
    )
    return theta1, theta2, theta3, reachable