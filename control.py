"""Drone control compatible with the competition MAVLink interface.

Sends SET_POSITION_TARGET_LOCAL_NED commands via MAVSDK offboard.
No GPS, no arming — competition sim starts the drone ready to receive commands.
Readiness is determined by receiving ODOMETRY (local position estimate).
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def wait_for_attitude(drone):
    """Block until the sim is publishing ATTITUDE — available in both PX4 SITL and competition sim."""
    async for att in drone.telemetry.attitude_euler():
        print(f"  attitude received: roll={att.roll_deg:.1f} pitch={att.pitch_deg:.1f} yaw={att.yaw_deg:.1f}")
        return


async def main():
    drone = System()

    print("Connecting...")
    await drone.connect(system_address="udpin://0.0.0.0:14550")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    # Use odometry as the readiness signal — works without GPS
    print("Waiting for attitude...")
    await wait_for_attitude(drone)
    print("Ready.")

    print("Arming...")
    try:
        await drone.action.arm()
    except Exception as e:
        # Competition sim may not require arming — log and continue
        print(f"  arm() note: {e}")

    # Must set a setpoint before starting offboard
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        # Competition sim may not require mode switching — log and continue
        print(f"  offboard.start() note: {e}")

    print("Taking off to 3m...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(5)

    print("Flying 5m forward (north)...")
    await drone.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(5)

    print("Returning to origin...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(5)

    print("Descending...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(5)

    await drone.offboard.stop()
    print("Done.")


if __name__ == "__main__":
    asyncio.run(main())
