"""Check which telemetry streams are available from the connected sim."""

import asyncio
from mavsdk import System


async def try_stream(name, gen, timeout=3.0):
    try:
        await asyncio.wait_for(anext(gen.__aiter__()), timeout=timeout)
        print(f"  [OK]   {name}")
    except asyncio.TimeoutError:
        print(f"  [HANG] {name}")
    except Exception as e:
        print(f"  [ERR]  {name}: {e}")


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14550")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected. Checking telemetry streams...\n")
            break

    await try_stream("odometry",             drone.telemetry.odometry())
    await try_stream("position_velocity_ned", drone.telemetry.position_velocity_ned())
    await try_stream("position",              drone.telemetry.position())
    await try_stream("attitude_euler",        drone.telemetry.attitude_euler())
    await try_stream("imu",                   drone.telemetry.imu())
    await try_stream("raw_imu",               drone.telemetry.raw_imu())


if __name__ == "__main__":
    asyncio.run(main())
