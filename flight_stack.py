"""Attitude controller + motor mixer for the Elodin physics core.

Replaces Betaflight (whose SITL bridge in the practice harness is broken —
see docs/JOURNAL.md 2026-06-11). Drives Elodin's clean rigid-body physics
directly with ground-truth state:

    planner efforts (tilt_x, tilt_y, vertical)
        │
        ▼
    reduced-attitude P  — desired thrust direction from tilt efforts;
                          error = body-z × desired-z  → desired body rates
    rate P              — torque = I · Kp_rate · rate error
    mixer               — collective + roll/pitch/yaw differentials → 4 motors

Frames: Elodin FLU body (x fwd, y left, z up), ENU world.
Motor order [BR, FR, BL, FL] matching sim/physics.py.

Gains are derived from the racing-quad preset physics (sim/config.py
create_5inch_racing_quad): mass 0.65, I=[0.002, 0.002, 0.0035],
arm 0.11, Fmax 14 N/motor, torque coeff 0.012.
"""

import math

import numpy as np


# ── Physics constants (must match sim/config.py racing preset) ────────
MASS = 0.65
GRAVITY = 9.81
F_MAX = 14.0                  # N per motor
ARM = 0.11 * math.sqrt(2) / 2  # moment arm per axis (X layout)
IXX, IYY, IZZ = 0.0020, 0.0020, 0.0035
K_TORQUE = 0.012              # yaw reaction torque per N of thrust

HOVER_CMD = MASS * GRAVITY / (4.0 * F_MAX)   # ≈ 0.114

# ── Controller gains ──────────────────────────────────────────────────
KP_ATT = 8.0       # desired rate (rad/s) per rad of tilt error
KP_RATE = 30.0     # torque = I * KP_RATE * rate error
KD_YAW = 8.0       # yaw rate damping (we don't hold yaw angle)
MAX_TILT_RAD = math.radians(25)

# Mixer tables, motor order [BR, FR, BL, FL]
ROLL_MIX = np.array([-1.0, -1.0, +1.0, +1.0])   # +x torque: left side up
PITCH_MIX = np.array([+1.0, -1.0, +1.0, -1.0])  # +y torque: back up, nose down
YAW_MIX = np.array([+1.0, -1.0, -1.0, +1.0])    # spin directions


def quat_to_rotmat(qx, qy, qz, qw):
    """Rotation matrix (body→world) from scalar-last quaternion."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def motor_commands(
    quat_xyzw,        # [qx, qy, qz, qw] body→world
    gyro_body,        # [wx, wy, wz] rad/s, FLU body
    tilt_x: float,    # [-1, 1] desired tilt toward world +x
    tilt_y: float,    # [-1, 1] desired tilt toward world +y
    thrust_cmd: float,  # collective motor command [0, 1]
) -> np.ndarray:
    """One control step → 4 motor commands in [0, 1], order [BR, FR, BL, FL]."""
    R = quat_to_rotmat(*quat_xyzw)

    # Desired thrust direction in world: unit vector tilted toward (x, y).
    tx = math.tan(MAX_TILT_RAD) * max(-1.0, min(1.0, tilt_x))
    ty = math.tan(MAX_TILT_RAD) * max(-1.0, min(1.0, tilt_y))
    z_des = np.array([tx, ty, 1.0])
    z_des /= np.linalg.norm(z_des)

    # Reduced-attitude error: rotation that takes body z to z_des,
    # expressed in the body frame.
    z_body_world = R[:, 2]
    err_world = np.cross(z_body_world, z_des)
    err_body = R.T @ err_world

    # Attitude P → desired body rates (yaw: damp to zero)
    w_des = np.array([KP_ATT * err_body[0], KP_ATT * err_body[1], 0.0])

    w = np.asarray(gyro_body, dtype=np.float64)
    torque = np.array([
        IXX * KP_RATE * (w_des[0] - w[0]),
        IYY * KP_RATE * (w_des[1] - w[1]),
        IZZ * KD_YAW * (0.0 - w[2]),
    ])

    # Differential motor commands from torque demands
    u_roll = torque[0] / (4.0 * F_MAX * ARM)
    u_pitch = torque[1] / (4.0 * F_MAX * ARM)
    u_yaw = torque[2] / (4.0 * F_MAX * K_TORQUE)

    # Tilt compensation: keep vertical thrust component as commanded
    cos_tilt = max(0.6, float(z_body_world[2]))
    collective = thrust_cmd / cos_tilt

    m = collective + u_roll * ROLL_MIX + u_pitch * PITCH_MIX + u_yaw * YAW_MIX
    return np.clip(m, 0.0, 1.0)
