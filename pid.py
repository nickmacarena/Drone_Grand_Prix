"""Cascaded PID controller — pure functions.

Three nested loops convert a target position into an attitude+thrust command:

    POSITION (P)  ───→  desired velocity
    VELOCITY (PD) ───→  desired acceleration (world NED)
    ACCEL→ATTITUDE ──→  (q_target, thrust)

The sim's inner attitude loop tracks the quaternion. We don't close that loop.

Gains are derived from measurements (see measurements.py) plus a few rules of thumb:
    - Outer loop bandwidth ≈ 1/5 of next-inner loop bandwidth
    - Damping ≈ critically-damped (ζ = 1) for smooth tracking
"""

import math

from attitude import clamp, normalize_angle, world_to_body_xy
from measurements import (
    HORIZONTAL_ACCEL_PER_RAD_TILT,
    HOVER_THRUST,
    MAX_TILT_RAD,
    THRUST_MAX,
    THRUST_MIN,
    VERTICAL_ACCEL_PER_UNIT_THRUST,
)


# ─── Loop gains ───────────────────────────────────────────────────────
# Position loop: position error → desired velocity (m/s per m).
# Bandwidth ~1 rad/s → KP_POS ≈ 1.0
KP_POS = 1.0

# Velocity loop: velocity error → desired acceleration (m/s² per m/s).
# Bandwidth ~3 rad/s → KP_VEL ≈ 3.0
# Damping: don't need much; sim has its own dynamics
KP_VEL = 3.0
KD_VEL = 1.0  # provides damping against position-loop output

# Yaw loop: yaw error → yaw setpoint (rad).
# We compute target yaw and just set it; sim's inner loop tracks.
# No gain needed — target yaw IS the command.

# Speed cap on the velocity setpoint — prevents the outer loop from
# asking for unattainable speeds when far from the target.
MAX_SPEED_MPS = 8.0


def position_to_velocity(target_pos, current_pos) -> tuple[float, float, float]:
    """Position-loop P controller. NED in, NED velocity out."""
    en = target_pos[0] - current_pos[0]
    ee = target_pos[1] - current_pos[1]
    ed = target_pos[2] - current_pos[2]

    vn = KP_POS * en
    ve = KP_POS * ee
    vd = KP_POS * ed

    # Clamp velocity magnitude
    speed = math.sqrt(vn * vn + ve * ve + vd * vd)
    if speed > MAX_SPEED_MPS:
        scale = MAX_SPEED_MPS / speed
        vn *= scale
        ve *= scale
        vd *= scale

    return vn, ve, vd


def velocity_to_acceleration(target_vel, current_vel) -> tuple[float, float, float]:
    """Velocity-loop PD controller. Returns desired acceleration in world NED."""
    en = target_vel[0] - current_vel[0]
    ee = target_vel[1] - current_vel[1]
    ed = target_vel[2] - current_vel[2]

    # P term: drive toward target velocity
    # D term: brake on current velocity (acts as damping against position loop)
    an = KP_VEL * en - KD_VEL * current_vel[0]
    ae = KP_VEL * ee - KD_VEL * current_vel[1]
    ad = KP_VEL * ed - KD_VEL * current_vel[2]

    return an, ae, ad


def acceleration_to_attitude(
    desired_accel,
    target_yaw: float,
) -> tuple[float, float, float, float]:
    """Algebraic conversion from desired world-NED acceleration to (roll, pitch, yaw, thrust).

    The drone's thrust vector points along body -z (up in body frame, since z=down).
    To produce a desired acceleration vector a_world, we need:
        thrust direction = a_world + gravity (since net force = mass × accel + weight reaction)

    For small tilt angles:
        horizontal_accel ≈ g * tilt_angle
        vertical_accel   ≈ (thrust - hover_thrust) * VERTICAL_ACCEL_PER_UNIT_THRUST
    """
    an, ae, ad = desired_accel

    # Vertical: thrust offset from hover. ad is in NED (positive down),
    # so negative ad = want to accelerate upward = need more thrust.
    thrust = HOVER_THRUST + (-ad) / VERTICAL_ACCEL_PER_UNIT_THRUST
    thrust = clamp(thrust, THRUST_MIN, THRUST_MAX)

    # Horizontal: rotate world acceleration into body frame using TARGET yaw,
    # because the drone will be facing target_yaw once it gets there.
    a_fwd, a_right = world_to_body_xy(an, ae, target_yaw)

    # Forward acceleration via nose-down pitch (negative pitch in body NED).
    # Right acceleration via right-roll (positive roll).
    pitch = -clamp(a_fwd / HORIZONTAL_ACCEL_PER_RAD_TILT, -MAX_TILT_RAD, MAX_TILT_RAD)
    roll = clamp(a_right / HORIZONTAL_ACCEL_PER_RAD_TILT, -MAX_TILT_RAD, MAX_TILT_RAD)

    return roll, pitch, target_yaw, thrust


def compute_attitude_target(
    drone_state,
    target_pos,
) -> tuple[float, float, float, float]:
    """Run the full cascade. Returns (roll, pitch, yaw, thrust) in radians + 0..1."""
    current_pos = (drone_state.north_m, drone_state.east_m, drone_state.down_m)
    current_vel = (drone_state.vn_mps, drone_state.ve_mps, drone_state.vd_mps)

    # Outer: position → velocity
    target_vel = position_to_velocity(target_pos, current_pos)

    # Middle: velocity → acceleration
    target_accel = velocity_to_acceleration(target_vel, current_vel)

    # Yaw target: face the gate in the NED horizontal plane
    dn = target_pos[0] - current_pos[0]
    de = target_pos[1] - current_pos[1]
    target_yaw = math.atan2(de, dn) if (dn * dn + de * de) > 0.01 else drone_state.yaw_rad

    # Inner conversion: acceleration → attitude+thrust
    return acceleration_to_attitude(target_accel, target_yaw)
