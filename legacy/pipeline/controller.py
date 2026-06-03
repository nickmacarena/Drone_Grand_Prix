"""Controller stage: target waypoint + drone state → attitude command.

Implements PID control to convert position error into attitude commands.
The sim's stabilized controller handles the low-level motor mixing —
we just output the desired throttle/roll/pitch/yaw.

Coordinate reminder:
    NED frame. Down is positive. Altitude is negative down.
    To fly north: pitch nose down (negative pitch).
    To fly east: roll right (positive roll).
    To climb: increase throttle above hover (~0.5).
"""

import math
from drone_types import AttitudeCommand, DroneState, Waypoint


# PID gains — these will need tuning.
# Start conservative (low gains) to avoid oscillation.
POS_P = 0.3   # position proportional
POS_D = 0.2   # position derivative (uses velocity as proxy)
ALT_P = 0.5   # altitude proportional
ALT_D = 0.3   # altitude derivative
YAW_P = 1.0   # yaw proportional

# Limits to prevent extreme commands.
MAX_TILT_RAD = math.radians(25)  # max roll/pitch
HOVER_THROTTLE = 0.5


def compute_attitude_command(
    waypoint: Waypoint,
    state: DroneState,
) -> AttitudeCommand:
    """Compute the attitude command to fly toward a waypoint.

    Args:
        waypoint: Where we want to go.
        state: Where we are now.

    Returns:
        AttitudeCommand with throttle, roll, pitch, yaw.
    """
    # --- Position error in NED ---
    err_n = waypoint.north_m - state.north_m
    err_e = waypoint.east_m - state.east_m
    err_d = waypoint.down_m - state.down_m

    # --- Horizontal control (position → roll/pitch) ---
    # PD controller on north/east error.
    # Use velocity as the derivative term (avoids needing to store previous error).
    cmd_n = POS_P * err_n - POS_D * state.vn_mps
    cmd_e = POS_P * err_e - POS_D * state.ve_mps

    # Rotate from NED into the drone's body frame using current yaw.
    # This is necessary because roll/pitch are relative to the drone's
    # heading, not to north/east.
    cos_yaw = math.cos(state.yaw_rad)
    sin_yaw = math.sin(state.yaw_rad)
    cmd_forward = cos_yaw * cmd_n + sin_yaw * cmd_e
    cmd_right = -sin_yaw * cmd_n + cos_yaw * cmd_e

    # Forward acceleration → pitch down (negative pitch in NED).
    # Right acceleration → roll right (positive roll).
    pitch_cmd = -clamp(cmd_forward, -MAX_TILT_RAD, MAX_TILT_RAD)
    roll_cmd = clamp(cmd_right, -MAX_TILT_RAD, MAX_TILT_RAD)

    # --- Altitude control (position → throttle) ---
    # PD on altitude (down axis). Negative error = we're too low = need more throttle.
    throttle_cmd = HOVER_THROTTLE - (ALT_P * err_d - ALT_D * state.vd_mps)
    throttle_cmd = clamp(throttle_cmd, 0.0, 1.0)

    # --- Yaw control ---
    if waypoint.yaw_rad is not None:
        yaw_err = normalize_angle(waypoint.yaw_rad - state.yaw_rad)
        yaw_cmd = state.yaw_rad + YAW_P * yaw_err
    else:
        # Point toward the waypoint
        yaw_cmd = math.atan2(err_e, err_n)

    return AttitudeCommand(
        throttle=throttle_cmd,
        roll_rad=roll_cmd,
        pitch_rad=pitch_cmd,
        yaw_rad=yaw_cmd,
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
