"""Quaternion and frame-transform math.

Pure functions, no state, no side effects. Conventions:
    - NED frame (x=north, y=east, z=down).
    - Quaternions are [w, x, y, z] (w-first).
    - Euler angles are Tait-Bryan ZYX (yaw, pitch, roll) in radians.
"""

import math


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    """Tait-Bryan ZYX (yaw→pitch→roll) to quaternion [w, x, y, z]."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]


def world_to_body_xy(north: float, east: float, yaw: float) -> tuple[float, float]:
    """Rotate a horizontal world-NED vector into body frame using yaw.

    Returns (forward, right). Forward is +x_body, right is +y_body.
    """
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    forward = cy * north + sy * east
    right = -sy * north + cy * east
    return forward, right


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
