"""MAVLink command senders.

Thin wrappers around pymavlink to send control commands to the sim.
"""

import time

from pymavlink import mavutil


# Bitmask for SET_POSITION_TARGET_LOCAL_NED to use velocity only
# (ignore position, acceleration, yaw, and yaw rate).
VELOCITY_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def send_velocity_ned(mavlink_conn, system_boot_ms, vn, ve, vd):
    """Send a NED velocity setpoint."""
    now_ms = int(time.time() * 1000)
    mavlink_conn.mav.set_position_target_local_ned_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        VELOCITY_ONLY_MASK,
        0.0, 0.0, 0.0,        # position (ignored)
        vn, ve, vd,           # velocity NED
        0.0, 0.0, 0.0,        # acceleration (ignored)
        0.0, 0.0,             # yaw, yaw_rate (ignored)
    )


# Bitmask for SET_POSITION_TARGET_LOCAL_NED to use position only
# (ignore velocity, acceleration, yaw, and yaw rate).
POSITION_ONLY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def send_position_ned(mavlink_conn, system_boot_ms, n, e, d):
    """Send a NED position target. Sim handles trajectory if it supports position mode."""
    now_ms = int(time.time() * 1000)
    mavlink_conn.mav.set_position_target_local_ned_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_ONLY_MASK,
        n, e, d,              # position NED
        0.0, 0.0, 0.0,        # velocity (ignored)
        0.0, 0.0, 0.0,        # acceleration (ignored)
        0.0, 0.0,             # yaw, yaw_rate (ignored)
    )


def send_attitude_rates(mavlink_conn, system_boot_ms, roll_rate, pitch_rate, yaw_rate, thrust):
    """Send body rate + thrust attitude command. Ignores attitude quaternion."""
    now_ms = int(time.time() * 1000)
    mask = mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
    mavlink_conn.mav.set_attitude_target_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mask,
        [1.0, 0.0, 0.0, 0.0],  # dummy quaternion (ignored)
        roll_rate,
        pitch_rate,
        yaw_rate,
        thrust,
    )


def send_attitude_quaternion(mavlink_conn, system_boot_ms, q, thrust):
    """Send target attitude quaternion + thrust. Ignores body rates.

    q is [w, x, y, z] for the desired attitude in NED.
    """
    now_ms = int(time.time() * 1000)
    mask = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
    )
    mavlink_conn.mav.set_attitude_target_send(
        now_ms - system_boot_ms,
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mask,
        q,
        0.0, 0.0, 0.0,  # rates (ignored)
        thrust,
    )


def send_arm(mavlink_conn):
    """Arm the drone."""
    mavlink_conn.mav.command_long_send(
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0,
    )
