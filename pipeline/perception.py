"""Perception stage: camera frame → gate observations.

STUB: Returns hardcoded gate positions as if detected.
Replace with real CV-based detection in Phase 3.
"""

from drone_types import DroneState, GateObservation


# Hardcoded gate positions in NED (absolute, meters from sim origin).
# Simple course for PX4 SITL testing.
STUB_GATES_NED = [
    (10.0, 0.0, -3.0),
    (20.0, 5.0, -3.0),
    (30.0, 0.0, -4.0),
    (40.0, -5.0, -3.0),
    (50.0, 0.0, -3.0),
]


def detect_gates(frame, state: DroneState) -> list[GateObservation]:
    """Detect gates in a camera frame.

    STUB: Ignores the frame entirely. Returns all hardcoded gates
    with positions converted to relative (from drone's perspective).
    """
    return [
        GateObservation(
            rel_north_m=gn - state.north_m,
            rel_east_m=ge - state.east_m,
            rel_down_m=gd - state.down_m,
            confidence=1.0,
            gate_index=i,
        )
        for i, (gn, ge, gd) in enumerate(STUB_GATES_NED)
    ]
