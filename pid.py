"""Heuristic controller — fixed-yaw + velocity-based braking.

After two unstable runs we learned:
    - The drone overshoots horizontally because we never tilt backward.
    - Chasing the bearing to a gate via yaw causes 180° spins on overshoot.
    - Altitude needs its own decoupled thrust loop.

Strategy:
    - Yaw: held fixed at the course's general direction (south for VQ1).
    - Pitch: P controller on forward-velocity error in body frame. Lets us
            tilt backward to BRAKE when we're moving too fast for the
            remaining distance, not just forward to accelerate.
    - Roll:  same idea for lateral motion (small corrections only).
    - Thrust: PD on altitude error. Decoupled from horizontal.

Body frame is aligned with TARGET_YAW (fixed). World-NED errors are
projected into body frame using TARGET_YAW.
"""

import math

from attitude import clamp
from measurements import (
    HOVER_THRUST,
    THRUST_MAX,
    THRUST_MIN,
)


# Course goes south. Hold yaw facing south (=π rad) for the whole run.
# Negative atan2 result chosen so the body-x axis points south.
TARGET_YAW = math.pi  # 180° = south in NED

# ─── Pitch (forward) and Roll (lateral) ──────────────────────────────
# Smaller gains and tighter tilt limits than before. The point is to
# steer, not to accelerate hard.
KP_VEL_HORIZ = 0.3       # m/s of desired velocity per meter of error
MAX_HORIZ_SPEED = 2.0    # m/s
KP_TILT = 0.15           # rad of tilt per (m/s) of velocity error
MAX_TILT_RAD = math.radians(15)

# Suppress pitch/roll when altitude is way off — focus on getting to
# the right altitude before sliding horizontally.
ALT_ERR_FULL_M = 2.0     # below this altitude error, full horizontal authority
ALT_ERR_NONE_M = 8.0     # above this, no horizontal motion

# ─── Thrust (altitude) ───────────────────────────────────────────────
KP_VD = 0.6              # desired vd per meter of altitude error
MAX_VD = 2.0             # m/s, NED (positive = descending)
KP_THRUST = 0.04         # thrust units per (m/s) of vd error


def compute_attitude_target(drone_state, target_pos):
    """Return (roll, pitch, yaw, thrust)."""
    dn = target_pos[0] - drone_state.north_m
    de = target_pos[1] - drone_state.east_m
    dd = target_pos[2] - drone_state.down_m  # >0 means gate below us in NED

    # ─── Project errors and velocity into target-yaw body frame ───
    cy = math.cos(TARGET_YAW)
    sy = math.sin(TARGET_YAW)
    forward_err = cy * dn + sy * de
    right_err = -sy * dn + cy * de
    forward_vel = cy * drone_state.vn_mps + sy * drone_state.ve_mps
    right_vel = -sy * drone_state.vn_mps + cy * drone_state.ve_mps

    # ─── Thrust (altitude) ───────────────────────────────────────
    # In NED, dd > 0 = gate is below = want vd > 0 (descend) → less thrust.
    desired_vd = clamp(KP_VD * dd, -MAX_VD, MAX_VD)
    vd_err = desired_vd - drone_state.vd_mps
    # vd_err > 0 → need more descent → reduce thrust.
    thrust = HOVER_THRUST - KP_THRUST * vd_err
    thrust = clamp(thrust, THRUST_MIN, THRUST_MAX)

    # ─── Pitch (forward) ──────────────────────────────────────────
    # Velocity setpoint shrinks with proximity. P controller on velocity
    # error gives us tilt in either direction (brake or accelerate).
    desired_forward_vel = clamp(KP_VEL_HORIZ * forward_err, -MAX_HORIZ_SPEED, MAX_HORIZ_SPEED)
    forward_vel_err = desired_forward_vel - forward_vel
    pitch = -clamp(KP_TILT * forward_vel_err, -MAX_TILT_RAD, MAX_TILT_RAD)

    # ─── Roll (lateral) ───────────────────────────────────────────
    desired_right_vel = clamp(KP_VEL_HORIZ * right_err, -MAX_HORIZ_SPEED, MAX_HORIZ_SPEED)
    right_vel_err = desired_right_vel - right_vel
    roll = clamp(KP_TILT * right_vel_err, -MAX_TILT_RAD, MAX_TILT_RAD)

    # ─── Altitude gating ──────────────────────────────────────────
    alt_err_abs = abs(dd)
    if alt_err_abs >= ALT_ERR_NONE_M:
        scale = 0.0
    elif alt_err_abs <= ALT_ERR_FULL_M:
        scale = 1.0
    else:
        scale = (ALT_ERR_NONE_M - alt_err_abs) / (ALT_ERR_NONE_M - ALT_ERR_FULL_M)
    pitch *= scale
    roll *= scale

    return roll, pitch, TARGET_YAW, thrust
