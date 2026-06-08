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
# Conservative — first time flying a real course. Tune up once stable.
KP_POS = 0.4
KP_VEL = 1.5
KD_VEL = 1.0

# Speed cap. Max horizontal accel is ~g (limited by tilt), so high target
# speeds just cause saturation. Keep this low for stability.
MAX_SPEED_MPS = 4.0

# Cap on commanded acceleration magnitude (per-axis). Prevents the velocity
# loop from asking for impossibilities when the position loop saturates.
MAX_ACCEL_MPSS = 6.0

# Yaw is only worth tracking when the drone is far from the target.
# Close to target, the bearing direction flips around and confuses things.
YAW_TRACK_DIST_M = 3.0


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

    an = KP_VEL * en - KD_VEL * current_vel[0]
    ae = KP_VEL * ee - KD_VEL * current_vel[1]
    ad = KP_VEL * ed - KD_VEL * current_vel[2]

    # Cap per-axis to avoid asking for impossibilities
    an = clamp(an, -MAX_ACCEL_MPSS, MAX_ACCEL_MPSS)
    ae = clamp(ae, -MAX_ACCEL_MPSS, MAX_ACCEL_MPSS)
    ad = clamp(ad, -MAX_ACCEL_MPSS, MAX_ACCEL_MPSS)

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

    # Yaw target: only chase when the drone is far from the target.
    # Close to the target, the bearing direction flips around and the
    # drone tries to spin in place, destabilizing everything.
    dn = target_pos[0] - current_pos[0]
    de = target_pos[1] - current_pos[1]
    horiz_dist_sq = dn * dn + de * de
    if horiz_dist_sq > YAW_TRACK_DIST_M * YAW_TRACK_DIST_M:
        target_yaw = math.atan2(de, dn)
    else:
        target_yaw = drone_state.yaw_rad  # hold current yaw

    # Inner conversion: acceleration → attitude+thrust
    return acceleration_to_attitude(target_accel, target_yaw)
