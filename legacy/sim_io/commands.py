"""Send attitude commands to the sim via MAVSDK."""

import math
from mavsdk import System
from mavsdk.offboard import Attitude
from drone_types import AttitudeCommand


async def send_attitude_command(drone: System, cmd: AttitudeCommand):
    """Send an AttitudeCommand to the sim via SET_ATTITUDE_TARGET."""
    await drone.offboard.set_attitude(Attitude(
        math.degrees(cmd.roll_rad),
        math.degrees(cmd.pitch_rad),
        math.degrees(cmd.yaw_rad),
        cmd.throttle,
    ))
