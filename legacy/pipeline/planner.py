"""Planner stage: gate observations + drone state → target waypoint.

Takes what perception sees and decides where to fly next.
Handles gate sequencing (must pass gates in order).
"""

import math
from drone_types import DroneState, GateObservation, Waypoint


# How close (meters) the drone must be to a gate to count as "passed".
GATE_REACHED_RADIUS_M = 3.0


def plan_next_waypoint(
    observations: list[GateObservation],
    state: DroneState,
    next_gate_index: int,
) -> tuple[Waypoint, int]:
    """Pick the next waypoint based on gate observations.

    Args:
        observations: Gates detected this frame.
        state: Current drone state.
        next_gate_index: The index of the gate we're trying to reach.

    Returns:
        (waypoint, updated_next_gate_index). The gate index advances
        when the drone gets close enough to the current target gate.
    """
    # Find the observation matching our target gate
    target_obs = None
    for obs in observations:
        if obs.gate_index == next_gate_index:
            target_obs = obs
            break

    if target_obs is None:
        # Target gate not visible — hold position
        return Waypoint(state.north_m, state.east_m, state.down_m), next_gate_index

    # Convert relative observation back to absolute position
    gate_n = state.north_m + target_obs.rel_north_m
    gate_e = state.east_m + target_obs.rel_east_m
    gate_d = state.down_m + target_obs.rel_down_m

    # Check if we've reached the gate
    distance = math.sqrt(
        target_obs.rel_north_m**2
        + target_obs.rel_east_m**2
        + target_obs.rel_down_m**2
    )

    if distance < GATE_REACHED_RADIUS_M:
        next_gate_index += 1

    # Aim yaw toward the gate
    yaw = math.atan2(target_obs.rel_east_m, target_obs.rel_north_m)

    return Waypoint(gate_n, gate_e, gate_d, yaw_rad=yaw), next_gate_index
