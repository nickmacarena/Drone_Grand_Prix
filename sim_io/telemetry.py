"""Read MAVSDK telemetry streams into DroneState."""

import asyncio
import math
from mavsdk import System
from drone_types import DroneState


async def get_drone_state(drone: System) -> DroneState:
    """Read one telemetry snapshot from MAVSDK and return a DroneState."""
    pv = await anext(drone.telemetry.position_velocity_ned().__aiter__())
    att = await anext(drone.telemetry.attitude_euler().__aiter__())

    return DroneState(
        north_m=pv.position.north_m,
        east_m=pv.position.east_m,
        down_m=pv.position.down_m,
        vn_mps=pv.velocity.north_m_s,
        ve_mps=pv.velocity.east_m_s,
        vd_mps=pv.velocity.down_m_s,
        roll_rad=math.radians(att.roll_deg),
        pitch_rad=math.radians(att.pitch_deg),
        yaw_rad=math.radians(att.yaw_deg),
        timestamp_s=asyncio.get_event_loop().time(),
    )
