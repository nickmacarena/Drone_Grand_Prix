"""Simplified controller.

After the cascaded PID went unstable, we backed off to a heuristic
controller that decouples altitude from horizontal motion:

    - Yaw  : face the active gate (or hold yaw when close).
    - Thrust: PD on altitude error — climb/descend to gate altitude.
    - Pitch: gentle forward tilt, scaled DOWN when altitude error is large.
             So drone gets to the right altitude before it tries to move.
    - Roll : zero. Drone steers via yaw, not via lateral roll.

This is less ambitious than full cascaded PID but much easier to keep stable.
Once VQ1 is in the bag we can revisit and improve.
"""

import math

from attitude import clamp, normalize_angle
from measurements import (
    HOVER_THRUST,
    THRUST_MAX,
    THRUST_MIN,
    VERTICAL_ACCEL_PER_UNIT_THRUST,
)


# ─── Heuristic gains ──────────────────────────────────────────────────
# Altitude (vertical) PD
K_ALT_P = 0.15           # thrust units per meter of altitude error
K_ALT_D = 0.10           # thrust units per (m/s) of vertical velocity

# Forward pitch when on altitude
CRUISE_PITCH_DEG = 12.0  # forward tilt to glide toward gate
ALT_TOLERANCE_M = 3.0    # within this altitude error, full cruise pitch
ALT_DEADZONE_M = 8.0     # outside this altitude error, no forward motion

# Yaw: only chase the gate bearing when far horizontally
YAW_TRACK_DIST_M = 3.0


def compute_attitude_target(drone_state, target_pos):
    """Return (roll, pitch, yaw, thrust)."""
    dn = target_pos[0] - drone_state.north_m
    de = target_pos[1] - drone_state.east_m
    dd = target_pos[2] - drone_state.down_m

    # ─── Yaw ──────────────────────────────────────────────────────────
    horiz_dist_sq = dn * dn + de * de
    if horiz_dist_sq > YAW_TRACK_DIST_M * YAW_TRACK_DIST_M:
        target_yaw = math.atan2(de, dn)
    else:
        target_yaw = drone_state.yaw_rad

    # ─── Thrust (altitude PD) ─────────────────────────────────────────
    # dd > 0 means gate is BELOW us → we need to descend → less thrust.
    # In NED, vd > 0 = descending. -vd brakes descent / supports climb.
    altitude_error = -dd  # positive = drone needs to climb
    thrust = HOVER_THRUST + K_ALT_P * altitude_error - K_ALT_D * (-drone_state.vd_mps)
    thrust = clamp(thrust, THRUST_MIN, THRUST_MAX)

    # ─── Pitch (forward only when altitude is roughly right) ──────────
    alt_err_abs = abs(dd)
    if alt_err_abs >= ALT_DEADZONE_M:
        pitch_scale = 0.0
    elif alt_err_abs <= ALT_TOLERANCE_M:
        pitch_scale = 1.0
    else:
        pitch_scale = (ALT_DEADZONE_M - alt_err_abs) / (ALT_DEADZONE_M - ALT_TOLERANCE_M)
    pitch = -math.radians(CRUISE_PITCH_DEG) * pitch_scale

    # ─── Roll ─────────────────────────────────────────────────────────
    roll = 0.0

    return roll, pitch, target_yaw, thrust
