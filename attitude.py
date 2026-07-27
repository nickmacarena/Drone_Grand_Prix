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


# ── Vector helpers (plain tuples/lists; no numpy — the Windows VM has none) ──

def vec_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def vec_dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_norm(a) -> float:
    return math.sqrt(vec_dot(a, a))


def vec_unit(a):
    n = vec_norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def angle_between(a, b) -> float:
    """Angle in radians between two vectors."""
    ua, ub = vec_unit(a), vec_unit(b)
    return math.acos(clamp(vec_dot(ua, ub), -1.0, 1.0))


# ── Quaternion helpers. [w, x, y, z], rotating BODY vectors into WORLD. ──

def quat_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def quat_rotate(q, v):
    """Rotate a BODY vector into the WORLD frame."""
    w, x, y, z = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v); v' = v + w*t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def quat_rotate_inv(q, v):
    """Rotate a WORLD vector into the BODY frame."""
    return quat_rotate([q[0], -q[1], -q[2], -q[3]], v)


def quat_from_two_vectors(u, v):
    """Shortest-arc quaternion rotating unit vector `u` onto unit vector `v`."""
    u, v = vec_unit(u), vec_unit(v)
    d = clamp(vec_dot(u, v), -1.0, 1.0)
    if d > 1.0 - 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    if d < -1.0 + 1e-9:
        # Antiparallel: rotate pi about any axis perpendicular to u.
        axis = vec_cross(u, (1.0, 0.0, 0.0))
        if vec_norm(axis) < 1e-6:
            axis = vec_cross(u, (0.0, 1.0, 0.0))
        ax, ay, az = vec_unit(axis)
        return [0.0, ax, ay, az]
    axis = vec_cross(u, v)
    s = math.sqrt((1.0 + d) * 2.0)
    return quat_normalize([s * 0.5, axis[0] / s, axis[1] / s, axis[2] / s])


def quat_to_euler_zyx(q) -> tuple[float, float, float]:
    """Quaternion to Tait-Bryan ZYX (roll, pitch, yaw) in radians.

    Meaningful as aerospace roll/pitch/yaw in an FRD-body / NED-world setup.
    In other conventions treat it as a diagnostic readout only.
    """
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = clamp(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
