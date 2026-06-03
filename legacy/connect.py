"""Barebones MAVLink connection to PX4 SITL via MAVSDK."""

import asyncio
from mavsdk import System


async def main():
    drone = System()

    # PX4 SITL GCS MAVLink UDP port
    print("Connecting to PX4 simulator...")
    await drone.connect(system_address="udpin://0.0.0.0:14550")

    # Wait for connection
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        print(f"  connection state: {state}")
        if state.is_connected:
            print("Connected to drone!")
            break

    # Print telemetry to confirm link is alive
    print("\nReceiving telemetry:")
    async for position in drone.telemetry.position():
        print(
            f"  lat={position.latitude_deg:.6f}, "
            f"lon={position.longitude_deg:.6f}, "
            f"alt={position.relative_altitude_m:.2f}m"
        )


if __name__ == "__main__":
    asyncio.run(main())
