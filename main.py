"""Main loop: wires the pipeline together and runs it against the sim."""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import Attitude, OffboardError

from sim_io.telemetry import get_drone_state
from sim_io.commands import send_attitude_command
from pipeline.perception import detect_gates, STUB_GATES_NED
from pipeline.planner import plan_next_waypoint
from pipeline.controller import compute_attitude_command


LOOP_HZ = 50
LOOP_PERIOD_S = 1.0 / LOOP_HZ


async def main():
    drone = System()

    print("Connecting...")
    await drone.connect(system_address="udpin://0.0.0.0:14550")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    # Wait for telemetry
    print("Waiting for telemetry...")
    async for att in drone.telemetry.attitude_euler():
        print(f"  attitude: roll={att.roll_deg:.1f} pitch={att.pitch_deg:.1f} yaw={att.yaw_deg:.1f}")
        break
    print("Ready.")

    # Arm (PX4 SITL requires this; competition sim may not)
    print("Arming...")
    await drone.action.arm()

    # Set an initial setpoint before starting offboard
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        return

    # --- Pipeline loop ---
    next_gate_index = 0
    total_gates = len(STUB_GATES_NED)

    print(f"Flying course ({total_gates} gates)...")

    while next_gate_index < total_gates:
        state = await get_drone_state(drone)

        observations = detect_gates(None, state)

        waypoint, next_gate_index = plan_next_waypoint(
            observations, state, next_gate_index,
        )

        cmd = compute_attitude_command(waypoint, state)

        await send_attitude_command(drone, cmd)

        print(
            f"  gate={next_gate_index}/{total_gates}"
            f"  pos=({state.north_m:.1f}, {state.east_m:.1f}, {state.down_m:.1f})"
            f"  thr={cmd.throttle:.2f}"
            f"  r={math.degrees(cmd.roll_rad):.1f}"
            f"  p={math.degrees(cmd.pitch_rad):.1f}",
            end="\r",
        )

        await asyncio.sleep(LOOP_PERIOD_S)

    print("\nCourse complete!")

    await drone.offboard.stop()
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(main())
